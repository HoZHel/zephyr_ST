/*
 * Copyright (c) 2021 Eug Krashtan
 * Copyright (c) 2022 Wouter Cappelle
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>
#include <stm32_ll_adc.h>
#if defined(CONFIG_SOC_SERIES_STM32H5X)
#include <stm32_ll_icache.h>
#endif /* CONFIG_SOC_SERIES_STM32H5X */

LOG_MODULE_REGISTER(stm32_temp, CONFIG_SENSOR_LOG_LEVEL);

#define CAL_RES 12
#define MAX_CALIB_POINTS 2

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_temp)
#define DT_DRV_COMPAT st_stm32_temp
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_temp_cal)
#define DT_DRV_COMPAT st_stm32_temp_cal
#define HAS_DUAL_CALIBRATION 1
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32c0_temp_cal)
#define DT_DRV_COMPAT st_stm32c0_temp_cal
#define HAS_SINGLE_CALIBRATION 1
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32wb0_temp_cal)
/* This matches only the STM32WB05 and STM32WB09 MCUs.
 * They have 2 calibration points as well, but the formula
 * to calculate temperature in Celcius is different from
 * STM32WB06/STM32WB07.
 */
#define DT_DRV_COMPAT st_stm32wb0_temp_cal
#define HAS_STM32WB0_TEMP_CAL 1
#define HAS_DUAL_CALIBRATION 1
#else
#error "No compatible devicetree node found"
#endif

#if defined(HAS_SINGLE_CALIBRATION) || defined(HAS_DUAL_CALIBRATION)
#define HAS_CALIBRATION 1
#endif

struct stm32_temp_data {
	const struct device *adc;
	const struct adc_channel_cfg adc_cfg;
	ADC_TypeDef *adc_base;
	struct adc_sequence adc_seq;
	struct k_mutex mutex;
	int16_t sample_buffer;
	int16_t raw; /* raw adc Sensor value */
};

struct stm32_temp_config {
#if HAS_CALIBRATION
	const void *cal1_addr;
	int cal1_temp;
#if defined(HAS_SINGLE_CALIBRATION)
	int avgslope;
#elif defined(HAS_DUAL_CALIBRATION)
	const void *cal2_addr;
	int cal2_temp;
#endif
	int cal_vrefanalog;
	int ts_cal_shift;
#else
	int avgslope;
	int v25_mv;
#endif
	bool is_ntc;
};

static inline void enable_adc_ts_channel(ADC_TypeDef *adc)
{
#if defined(CONFIG_SOC_SERIES_STM32WB0)
	/* Temperature sensor channel always enabled on WB0x.
	 * Silence compiler warning since `adc` is unused.
	 */
	ARG_UNUSED(adc);
#else
	uint32_t path = LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(data->adc_base));

	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(data->adc_base),
				       LL_ADC_PATH_INTERNAL_TEMPSENSOR | path);

	k_usleep(LL_ADC_DELAY_TEMPSENSOR_STAB_US);
#endif /* CONFIG_SOC_SERIES_STM32WB0 */
}

static inline void disable_adc_ts_channel(ADC_TypeDef *adc)
{
#if defined(CONFIG_SOC_SERIES_STM32WB0)
	/* Temperature sensor channel always enabled on WB0x.
	 * Silence compiler warning since `adc` is unused.
	 */
	ARG_UNUSED(adc);
#else
	uint32_t path = LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(data->adc_base));

	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(data->adc_base),
				       path &= ~LL_ADC_PATH_INTERNAL_TEMPSENSOR);
#endif /* CONFIG_SOC_SERIES_STM32WB0 */

}

#if defined(HAS_CALIBRATION)
static inline uint32_t read_mfg_flash_data(const void *addr)
{
#if defined(CONFIG_SOC_SERIES_STM32WB0)
	return sys_read32((mem_addr_t)addr);
#else
	return (uint32_t)sys_read16((mem_addr_t)addr);
#endif /* CONFIG_SOC_SERIES_STM32WB0 */
}

/**
 * @returns the following information depending on calibration type:
 *   HAS_SINGLE_CALIBRATION: calib_data[0] = TS_CAL
 *
 *   HAS_STM32WB0_TEMP_CAL:  calib_data[0] = C30
 *                           calib_data[1] = TCK
 *
 *   HAS_DUAL_CALIBRATION:   calib_data[0] = TS_CAL1
 *                           calib_data[1] = TS_CAL2
 */
static inline void read_calibration_data(const struct stm32_temp_config *cfg,
					uint32_t calib_data[MAX_CALIB_POINTS])
{
#if defined(CONFIG_SOC_SERIES_STM32H5X)
	/* Manufacturing flash accesses must be non-cacheable on STM32H5x */
	LL_ICACHE_Disable();
#endif /* CONFIG_SOC_SERIES_STM32H5X */

	calib_data[0] = read_mfg_flash_data(cfg->cal1_addr) >> cfg->ts_cal_shift;
#if HAS_DUAL_CALIBRATION
	calib_data[1] = read_mfg_flash_data(cfg->cal2_addr) >> cfg->ts_cal_shift;
#endif

#if defined(CONFIG_SOC_SERIES_STM32H5X)
	/* Enable back ICACHE (unconditonally, as in soc.c) */
	LL_ICACHE_Enable();
#endif /* CONFIG_SOC_SERIES_STM32H5X */
}
#endif /* HAS_CALIBRATION */

static float calculate_temperature_from_sample(const struct device *dev)
{
	struct stm32_temp_data *data = dev->data;
	const struct stm32_temp_config *cfg = dev->config;
	float temp;

#if defined(HAS_CALIBRATION)
	/* calib_data must be zero-initialized, as read_calibration_data
	 * may write only to the [0]th element (if HAS_SINGLE_CALIBRATION)
	 */
	const __maybe_unused uint32_t vref = adc_ref_internal(data->adc);
	uint32_t calib_data[MAX_CALIB_POINTS] = { 0 };

	read_calibration_data(cfg, calib_data);

#if !defined(HAS_STM32WB0_TEMP_CAL)
	/* Convert raw value to voltage */
	temp = ((float)vref / cfg->cal_vrefanalog) * data->raw;

	/**
	 * There are two possible formulas depending on SINGLE or DUAL:
	 *  - for SINGLE: Tj = (Vmes / Avg_Slope) + TS_CAL1_TEMP
	 *  - for DUAL: Tj = (k * (Vmes - TS_CAL1)) + TS_CAL1_TEMP
	 *     - k = (TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)
	 * where Vmes is, depending on the MCU, one of:
	 *     - (Vsense - TS_CAL1)
	 *     - (TS_CAL1 - Vsense)
	 */
#	if DT_INST_PROP_OR(0, ntc, 0)
	temp = (calib_data[0] - temp);
#	else
	temp = (temp - calib_data[0]);
#	endif

#	if defined(HAS_SINGLE_CALIBRATION)
	temp /= (cfg->avgslope * 4096) / (cfg->cal_vrefanalog * 1000);
#	else /* HAS_DUAL_CALIBRATION */
	temp *= (cfg->cal2_temp - cfg->cal1_temp);
	temp /= ((cal2 - cal1) >> cfg->ts_cal_shift);
#	endif

	temp += cfg->cal1_temp;
#else
	/* For STM32WB0 temperature sensor, Tj = ((Cmeas - C30 + TCK) / 10) */
	temp = ((float)data->raw - calib_data[0] + calib_data[1]) / 10.0f;
#endif /* HAS_STM32WB0_TEMP_CAL */

#else /* !HAS_CALIBRATION */
	/* Sensor value in millivolts */
	int32_t mv = data->raw * adc_ref_internal(data->adc) / 0x0FFF;

	if (cfg->is_ntc) {
		temp = (float)(cfg->v25_mv - mv);
	} else {
		temp = (float)(mv - cfg->v25_mv);
	}
	temp = (temp / cfg->avgslope) * 10;
	temp += 25;
#endif
	return temp;
}

static int stm32_temp_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct stm32_temp_data *data = dev->data;
	struct adc_sequence *sp = &data->adc_seq;
	int rc;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);
	pm_device_runtime_get(data->adc);

	rc = adc_channel_setup(data->adc, &data->adc_cfg);
	if (rc) {
		LOG_DBG("Setup AIN%u got %d", data->adc_cfg.channel_id, rc);
		goto unlock;
	}

	enable_adc_ts_channel(data->adc_base);

	rc = adc_read(data->adc, sp);
	if (rc == 0) {
		data->raw = data->sample_buffer;
	}

	disable_adc_ts_channel(data->adc_base);

unlock:
	pm_device_runtime_put(data->adc);
	k_mutex_unlock(&data->mutex);

	return rc;
}

static int stm32_temp_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	const float temp = calculate_temperature_from_sample(dev);

	return sensor_value_from_float(val, temp);
}

static const struct sensor_driver_api stm32_temp_driver_api = {
	.sample_fetch = stm32_temp_sample_fetch,
	.channel_get = stm32_temp_channel_get,
};

#if defined(HAS_STM32WB0_TEMP_CAL)
/**
 * Workaround for errata in STM32WB05/09 ADC.
 * The first temperature sensor reading after powering
 * on the ADC is incorrect (usually ~0x300 too low).
 *
 * Ensure we get proper data by always sampling twice.
 */
enum adc_action adc_seq_cb(const struct device *dev,
			const struct adc_sequence *sequence,
			uint16_t sampling_index)
{
	static bool first = true;
	bool repeat = first;

	first = !first;

	if (repeat) {
		return ADC_ACTION_REPEAT;
	} else {
		return ADC_ACTION_FINISH;
	}
}

static struct adc_sequence_options adc_seq_opts = {
	.callback = adc_seq_cb
};
#endif

static int stm32_temp_init(const struct device *dev)
{
	struct stm32_temp_data *data = dev->data;
	struct adc_sequence *asp = &data->adc_seq;

	k_mutex_init(&data->mutex);

	if (!device_is_ready(data->adc)) {
		LOG_ERR("ADC device %s is not ready", data->adc->name);
		return -ENODEV;
	}

	*asp = (struct adc_sequence){
		.channels = BIT(data->adc_cfg.channel_id),
		.buffer = &data->sample_buffer,
		.buffer_size = sizeof(data->sample_buffer),
		.resolution = CAL_RES,
#if defined(HAS_STM32WB0_TEMP_CAL)
		.options = &adc_seq_opts,
#endif
	};

	return 0;
}

static struct stm32_temp_data stm32_temp_dev_data = {
	.adc = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR(0)),
	.adc_base = (ADC_TypeDef *)DT_REG_ADDR(DT_INST_IO_CHANNELS_CTLR(0)),
	.adc_cfg = {
		.gain = ADC_GAIN_1,
#if defined(CONFIG_SOC_SERIES_STM32WB0)
		.reference = ADC_REF_VDD_1_3,
		/* TODO: can ADC_ACQ_TIME_DEFAULT be used for all series? */
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
#else
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_MAX,
#endif
		.channel_id = DT_INST_IO_CHANNELS_INPUT(0),
		.differential = 0
	},
};

static const struct stm32_temp_config stm32_temp_dev_config = {
#if HAS_CALIBRATION
	.cal1_addr = (const void *)DT_INST_PROP(0, ts_cal1_addr),
#if defined(HAS_SINGLE_CALIBRATION)
	.cal1_temp = DT_INST_PROP(0, ts_cal1_temp),
	.avgslope = DT_INST_PROP(0, avgslope),
#elif defined(HAS_DUAL_CALIBRATION)
#	if !defined(HAS_STM32WB0_TEMP_CAL)
	.cal2_addr = (const void *)DT_INST_PROP(0, ts_cal2_addr),
	.cal2_temp = DT_INST_PROP(0, ts_cal2_temp),
#else
	.cal2_addr = (const void *)DT_INST_PROP(0, ts_tck_addr),
	.cal2_temp = 0, /* doesn't matter */
#endif /* HAS_STM32WB0_TEMP_CAL */
#else
	.avgslope = DT_INST_PROP(0, avgslope),
#endif
	.ts_cal_shift = (DT_INST_PROP(0, ts_cal_resolution) - CAL_RES),
	.cal_vrefanalog = DT_INST_PROP(0, ts_cal_vrefanalog),
#else
	.avgslope = DT_INST_PROP(0, avgslope),
	.v25_mv = DT_INST_PROP(0, v25),
#endif
	.is_ntc = DT_INST_PROP_OR(0, ntc, false)
};

SENSOR_DEVICE_DT_INST_DEFINE(0, stm32_temp_init, NULL,
			     &stm32_temp_dev_data, &stm32_temp_dev_config,
			     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
			     &stm32_temp_driver_api);
