/*  Bluetooth Audio Broadcast Sink */

/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include "../host/conn_internal.h"
#include "../host/iso_internal.h"

#include "audio_internal.h"
#include "bap_iso.h"
#include "bap_endpoint.h"
#include "pacs_internal.h"

LOG_MODULE_REGISTER(bt_bap_broadcast_sink, CONFIG_BT_BAP_BROADCAST_SINK_LOG_LEVEL);

#include "common/bt_str.h"

#define PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO 20 /* Set the timeout relative to interval */
#define BROADCAST_SYNC_MIN_INDEX  (BIT(1))

static struct bt_bap_ep broadcast_sink_eps[CONFIG_BT_BAP_BROADCAST_SNK_COUNT]
					  [CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
static struct bt_bap_broadcast_sink broadcast_sinks[CONFIG_BT_BAP_BROADCAST_SNK_COUNT];

struct codec_cap_lookup_id_data {
	uint8_t id;
	const struct bt_audio_codec_cap *codec_cap;
};

static sys_slist_t sink_cbs = SYS_SLIST_STATIC_INIT(&sink_cbs);

/* The mod_src_param is and shall only be used by the BT RX thread. It is statically defined due to
 * the size of it, and that it's configurable in size, and can cause stack overflow issues otherwise
 */
static struct bt_bap_scan_delegator_mod_src_param mod_src_param;

static void broadcast_sink_cleanup(struct bt_bap_broadcast_sink *sink);

static bool find_recv_state_by_sink_cb(const struct bt_bap_scan_delegator_recv_state *recv_state,
				       void *user_data)
{
	const struct bt_bap_broadcast_sink *sink = user_data;

	if (atomic_test_bit(sink->flags, BT_BAP_BROADCAST_SINK_FLAG_SRC_ID_VALID) &&
	    sink->bass_src_id == recv_state->src_id) {
		return true;
	}

	return false;
}

static bool find_recv_state_by_pa_sync_cb(const struct bt_bap_scan_delegator_recv_state *recv_state,
					  void *user_data)
{
	struct bt_le_per_adv_sync *sync = user_data;
	struct bt_le_per_adv_sync_info sync_info;
	int err;

	err = bt_le_per_adv_sync_get_info(sync, &sync_info);
	if (err != 0) {
		LOG_DBG("Failed to get sync info: %d", err);

		return false;
	}

	if (bt_addr_le_eq(&recv_state->addr, &sync_info.addr) &&
	    recv_state->adv_sid == sync_info.sid) {
		return true;
	}

	return false;
};

static void update_recv_state_big_synced(const struct bt_bap_broadcast_sink *sink)
{
	const struct bt_bap_scan_delegator_recv_state *recv_state;
	int err;

	recv_state = bt_bap_scan_delegator_find_state(find_recv_state_by_sink_cb, (void *)sink);
	if (recv_state == NULL) {
		LOG_WRN("Failed to find receive state for sink %p", sink);

		return;
	}

	(void)memset(&mod_src_param, 0, sizeof(mod_src_param));

	mod_src_param.num_subgroups = sink->subgroup_count;
	for (uint8_t i = 0U; i < sink->subgroup_count; i++) {
		struct bt_bap_bass_subgroup *subgroup_param = &mod_src_param.subgroups[i];
		const struct bt_bap_broadcast_sink_subgroup *sink_subgroup = &sink->subgroups[i];

		/* Set the bis_sync value to the indexes available per subgroup */
		subgroup_param->bis_sync = sink_subgroup->bis_indexes & sink->indexes_bitfield;
	}

	if (recv_state->encrypt_state == BT_BAP_BIG_ENC_STATE_BCODE_REQ ||
	    recv_state->encrypt_state == BT_BAP_BIG_ENC_STATE_BAD_CODE) {
		mod_src_param.encrypt_state = BT_BAP_BIG_ENC_STATE_DEC;
	} else {
		mod_src_param.encrypt_state = recv_state->encrypt_state;
	}

	/* Since the mod_src_param struct is 0-initialized the metadata won't
	 * be modified by this
	 */

	/* Copy existing unchanged data */
	mod_src_param.src_id = recv_state->src_id;
	mod_src_param.broadcast_id = recv_state->broadcast_id;

	err = bt_bap_scan_delegator_mod_src(&mod_src_param);
	if (err != 0) {
		LOG_WRN("Failed to modify Receive State for sink %p: %d", sink, err);
	}
}

static void update_recv_state_big_cleared(const struct bt_bap_broadcast_sink *sink,
					  uint8_t reason)
{
	const struct bt_bap_scan_delegator_recv_state *recv_state;
	bool sink_is_streaming = false;
	int err;

	recv_state = bt_bap_scan_delegator_find_state(find_recv_state_by_sink_cb, (void *)sink);
	if (recv_state == NULL) {
		/* This is likely due to the receive state being removed while we are BIG synced */
		LOG_DBG("Could not find receive state for sink %p", sink);

		return;
	}

	(void)memset(&mod_src_param, 0, sizeof(mod_src_param));

	if ((recv_state->encrypt_state == BT_BAP_BIG_ENC_STATE_BCODE_REQ ||
	     recv_state->encrypt_state == BT_BAP_BIG_ENC_STATE_DEC) &&
	    reason == BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL) {
		/* Sync failed due to bad broadcast code */
		mod_src_param.encrypt_state = BT_BAP_BIG_ENC_STATE_BAD_CODE;
	} else {
		mod_src_param.encrypt_state = BT_BAP_BIG_ENC_STATE_NO_ENC;
	}

	/* Determine if the previous receive state reported that streaming was active
	 * If it was previously active, then we need to set the BIS_sync state to 0
	 * (not streaming), and if not then we consider this a BIG Sync failure and
	 * set BT_BAP_BIS_SYNC_FAILED
	 */
	for (uint8_t i = 0U; i < recv_state->num_subgroups && !sink_is_streaming; i++) {
		sink_is_streaming = recv_state->subgroups[i].bis_sync != 0 &&
				    recv_state->subgroups[i].bis_sync != BT_BAP_BIS_SYNC_FAILED;
	}

	if (!sink_is_streaming) {
		/* BASS spec 3.1.1.5: Set Sync Failed when the server fails to sync to the BIG */
		for (uint8_t i = 0U; i < recv_state->num_subgroups; i++) {
			mod_src_param.subgroups[i].bis_sync = BT_BAP_BIS_SYNC_FAILED;
		}
	}

	/* Since the metadata_len is 0 then the metadata won't be modified by the operation either*/

	/* Copy existing unchanged data */
	mod_src_param.num_subgroups = recv_state->num_subgroups;
	mod_src_param.src_id = recv_state->src_id;
	mod_src_param.broadcast_id = recv_state->broadcast_id;

	err = bt_bap_scan_delegator_mod_src(&mod_src_param);
	if (err != 0) {
		LOG_WRN("Failed to modify Receive State for sink %p: %d",
			sink, err);
	}
}

static void broadcast_sink_clear_big(struct bt_bap_broadcast_sink *sink,
				     uint8_t reason)
{
	sink->big = NULL;

	update_recv_state_big_cleared(sink, reason);
}

static struct bt_bap_broadcast_sink *broadcast_sink_lookup_iso_chan(
	const struct bt_iso_chan *chan)
{
	for (size_t i = 0U; i < ARRAY_SIZE(broadcast_sinks); i++) {
		for (uint8_t j = 0U; j < broadcast_sinks[i].stream_count; j++) {
			if (broadcast_sinks[i].bis[j].chan == chan) {
				return &broadcast_sinks[i];
			}
		}
	}

	return NULL;
}

static void broadcast_sink_set_ep_state(struct bt_bap_ep *ep, uint8_t state)
{
	uint8_t old_state;

	old_state = ep->status.state;

	LOG_DBG("ep %p id 0x%02x %s -> %s", ep, ep->status.id, bt_bap_ep_state_str(old_state),
		bt_bap_ep_state_str(state));

	switch (old_state) {
	case BT_BAP_EP_STATE_IDLE:
		if (state != BT_BAP_EP_STATE_QOS_CONFIGURED) {
			LOG_DBG("Invalid broadcast sync endpoint state transition");
			return;
		}
		break;
	case BT_BAP_EP_STATE_QOS_CONFIGURED:
		if (state != BT_BAP_EP_STATE_IDLE && state != BT_BAP_EP_STATE_STREAMING) {
			LOG_DBG("Invalid broadcast sync endpoint state transition");
			return;
		}
		break;
	case BT_BAP_EP_STATE_STREAMING:
		if (state != BT_BAP_EP_STATE_IDLE) {
			LOG_DBG("Invalid broadcast sync endpoint state transition");
			return;
		}
		break;
	default:
		LOG_ERR("Invalid broadcast sync endpoint state: %s",
			bt_bap_ep_state_str(old_state));
		return;
	}

	ep->status.state = state;

	if (state == BT_BAP_EP_STATE_IDLE) {
		struct bt_bap_stream *stream = ep->stream;

		if (stream != NULL) {
			bt_bap_iso_unbind_ep(ep->iso, ep);
			stream->ep = NULL;
			stream->codec_cfg = NULL;
			ep->stream = NULL;
		}
	}
}

static void broadcast_sink_iso_recv(struct bt_iso_chan *chan,
				    const struct bt_iso_recv_info *info,
				    struct net_buf *buf)
{
	struct bt_bap_iso *iso = CONTAINER_OF(chan, struct bt_bap_iso, chan);
	const struct bt_bap_stream_ops *ops;
	struct bt_bap_stream *stream;
	struct bt_bap_ep *ep = iso->rx.ep;
	size_t buf_len;

	if (ep == NULL) {
		LOG_ERR("iso %p not bound with ep", chan);
		return;
	}

	stream = ep->stream;
	if (stream == NULL) {
		LOG_ERR("No stream for ep %p", ep);
		return;
	}

	ops = stream->ops;

	buf_len = net_buf_frags_len(buf);
	if (IS_ENABLED(CONFIG_BT_BAP_DEBUG_STREAM_DATA)) {
		LOG_DBG("stream %p ep %p len %zu", stream, stream->ep, buf_len);
	}

	if (buf_len > stream->qos->sdu) {
		LOG_WRN("Received %u octets but stream %p was only configured for %u", buf_len,
			stream, stream->qos->sdu);
	}

	if (ops != NULL && ops->recv != NULL) {
		ops->recv(stream, info, buf);
	} else {
		LOG_WRN("No callback for recv set");
	}
}

static bool broadcast_sink_is_in_state(struct bt_bap_broadcast_sink *sink,
				       enum bt_bap_ep_state state)
{
	struct bt_bap_stream *stream;

	if (sink == NULL) {
		LOG_DBG("sink is NULL");

		return state == BT_BAP_EP_STATE_IDLE;
	}

	if (sys_slist_is_empty(&sink->streams)) {
		LOG_DBG("Sink does not have any streams");

		return state == BT_BAP_EP_STATE_IDLE;
	}

	SYS_SLIST_FOR_EACH_CONTAINER(&sink->streams, stream, _node) {
		if (stream->ep != NULL && stream->ep->status.state != state) {
			return false;
		}
	}

	return true;
}

static void broadcast_sink_iso_connected(struct bt_iso_chan *chan)
{
	struct bt_bap_iso *iso = CONTAINER_OF(chan, struct bt_bap_iso, chan);
	const struct bt_bap_stream_ops *ops;
	struct bt_bap_broadcast_sink *sink;
	struct bt_bap_stream *stream;
	struct bt_bap_ep *ep = iso->rx.ep;

	if (ep == NULL) {
		LOG_ERR("iso %p not bound with ep", chan);
		return;
	}

	stream = ep->stream;
	if (stream == NULL) {
		LOG_ERR("No stream for ep %p", ep);
		return;
	}

	LOG_DBG("stream %p", stream);

	ops = stream->ops;
	if (ops != NULL && ops->connected != NULL) {
		ops->connected(stream);
	}

	sink = broadcast_sink_lookup_iso_chan(chan);
	if (sink == NULL) {
		LOG_ERR("Could not lookup sink by iso %p", chan);
		return;
	}

	broadcast_sink_set_ep_state(ep, BT_BAP_EP_STATE_STREAMING);

	/* Setup the ISO data path */
	bt_bap_setup_iso_data_path(stream);

	if (ops != NULL && ops->started != NULL) {
		ops->started(stream);
	} else {
		LOG_WRN("No callback for started set");
	}

	if (broadcast_sink_is_in_state(sink, BT_BAP_EP_STATE_STREAMING)) {
		update_recv_state_big_synced(sink);
	}
}

static void broadcast_sink_iso_disconnected(struct bt_iso_chan *chan,
					    uint8_t reason)
{
	struct bt_bap_iso *iso = CONTAINER_OF(chan, struct bt_bap_iso, chan);
	const struct bt_bap_stream_ops *ops;
	struct bt_bap_stream *stream;
	struct bt_bap_ep *ep = iso->rx.ep;
	struct bt_bap_broadcast_sink *sink;

	if (ep == NULL) {
		LOG_ERR("iso %p not bound with ep", chan);
		return;
	}

	stream = ep->stream;
	if (stream == NULL) {
		LOG_ERR("No stream for ep %p", ep);
		return;
	}

	LOG_DBG("stream %p ep %p reason 0x%02x", stream, ep, reason);

	ops = stream->ops;
	if (ops != NULL && ops->disconnected != NULL) {
		ops->disconnected(stream, reason);
	}

	broadcast_sink_set_ep_state(ep, BT_BAP_EP_STATE_IDLE);

	sink = broadcast_sink_lookup_iso_chan(chan);
	if (sink == NULL) {
		LOG_ERR("Could not lookup sink by iso %p", chan);
	} else {
		if (!sys_slist_find_and_remove(&sink->streams, &stream->_node)) {
			LOG_DBG("Could not find and remove stream %p from sink %p", stream, sink);
		}
	}

	if (ops != NULL && ops->stopped != NULL) {
		ops->stopped(stream, reason);
	} else {
		LOG_WRN("No callback for stopped set");
	}
}

static struct bt_iso_chan_ops broadcast_sink_iso_ops = {
	.recv		= broadcast_sink_iso_recv,
	.connected	= broadcast_sink_iso_connected,
	.disconnected	= broadcast_sink_iso_disconnected,
};

static struct bt_bap_broadcast_sink *broadcast_sink_free_get(void)
{
	/* Find free entry */
	for (int i = 0; i < ARRAY_SIZE(broadcast_sinks); i++) {
		if (!atomic_test_bit(broadcast_sinks[i].flags,
				     BT_BAP_BROADCAST_SINK_FLAG_INITIALIZED)) {
			broadcast_sinks[i].index = i;
			broadcast_sinks[i].broadcast_id = BT_BAP_INVALID_BROADCAST_ID;

			return &broadcast_sinks[i];
		}
	}

	return NULL;
}

static struct bt_bap_broadcast_sink *broadcast_sink_get_by_pa(struct bt_le_per_adv_sync *sync)
{
	for (int i = 0; i < ARRAY_SIZE(broadcast_sinks); i++) {
		if (broadcast_sinks[i].pa_sync == sync) {
			return &broadcast_sinks[i];
		}
	}

	return NULL;
}

static struct bt_bap_broadcast_sink *broadcast_sink_get_by_big(const struct bt_iso_big *big)
{
	for (size_t i = 0U; i < ARRAY_SIZE(broadcast_sinks); i++) {
		if (broadcast_sinks[i].big == big) {
			return &broadcast_sinks[i];
		}
	}

	return NULL;
}

static void broadcast_sink_add_src(struct bt_bap_broadcast_sink *sink)
{
	struct bt_bap_scan_delegator_add_src_param add_src_param;
	struct bt_le_per_adv_sync_info sync_info;
	int err;

	err = bt_le_per_adv_sync_get_info(sink->pa_sync, &sync_info);
	__ASSERT_NO_MSG(err == 0);

	bt_addr_le_copy(&add_src_param.addr, &sync_info.addr);
	add_src_param.sid = sync_info.sid;
	add_src_param.broadcast_id = sink->broadcast_id;
	/* Will be updated when we receive the BASE */
	add_src_param.encrypt_state = BT_BAP_BIG_ENC_STATE_NO_ENC;
	add_src_param.num_subgroups = 0U;

	err = bt_bap_scan_delegator_add_src(&add_src_param);
	if (err < 0) {
		LOG_WRN("Failed to add sync as Receive State for sink %p: %d",
			sink, err);
	} else {
		sink->bass_src_id = (uint8_t)err;
		atomic_set_bit(sink->flags,
			       BT_BAP_BROADCAST_SINK_FLAG_SRC_ID_VALID);
	}
}

static bool base_subgroup_meta_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
	struct bt_bap_bass_subgroup *subgroup_param;
	uint8_t *meta;
	int ret;

	ret = bt_bap_base_get_subgroup_codec_meta(subgroup, &meta);
	if (ret < 0) {
		return false;
	}

	subgroup_param = &mod_src_param.subgroups[mod_src_param.num_subgroups++];
	subgroup_param->metadata_len = (uint8_t)ret;
	memcpy(subgroup_param->metadata, meta, subgroup_param->metadata_len);

	return true;
}

static int update_recv_state_base_copy_meta(const struct bt_bap_base *base)
{
	int err;

	err = bt_bap_base_foreach_subgroup(base, base_subgroup_meta_cb, NULL);
	if (err != 0) {
		LOG_DBG("Failed to parse subgroups: %d", err);
		return err;
	}

	return 0;
}

static void update_recv_state_base(const struct bt_bap_broadcast_sink *sink,
				   const struct bt_bap_base *base)
{
	const struct bt_bap_scan_delegator_recv_state *recv_state;
	int err;

	recv_state = bt_bap_scan_delegator_find_state(find_recv_state_by_sink_cb, (void *)sink);
	if (recv_state == NULL) {
		LOG_WRN("Failed to find receive state for sink %p", sink);

		return;
	}

	(void)memset(&mod_src_param, 0, sizeof(mod_src_param));

	err = update_recv_state_base_copy_meta(base);
	if (err != 0) {
		LOG_WRN("Failed to modify Receive State for sink %p: %d", sink, err);
		return;
	}

	/* Copy existing unchanged data */
	mod_src_param.src_id = recv_state->src_id;
	mod_src_param.encrypt_state = recv_state->encrypt_state;
	mod_src_param.broadcast_id = recv_state->broadcast_id;
	mod_src_param.num_subgroups = sink->subgroup_count;
	for (uint8_t i = 0U; i < sink->subgroup_count; i++) {
		struct bt_bap_bass_subgroup *subgroup_param = &mod_src_param.subgroups[i];

		/* Leave the bis_sync unchanged */
		subgroup_param->bis_sync = recv_state->subgroups[i].bis_sync;
	}

	err = bt_bap_scan_delegator_mod_src(&mod_src_param);
	if (err != 0) {
		LOG_WRN("Failed to modify Receive State for sink %p: %d", sink, err);
	}
}

static bool base_subgroup_bis_count_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
	uint8_t *bis_cnt = user_data;
	int ret;

	ret = bt_bap_base_get_subgroup_bis_count(subgroup);
	if (ret < 0) {
		return false;
	}

	*bis_cnt += (uint8_t)ret;

	return true;
}

static int base_get_bis_count(const struct bt_bap_base *base)
{
	uint8_t bis_cnt = 0U;
	int err;

	err = bt_bap_base_foreach_subgroup(base, base_subgroup_bis_count_cb, &bis_cnt);
	if (err != 0) {
		LOG_DBG("Failed to parse subgroups: %d", err);
		return err;
	}

	return bis_cnt;
}

static bool base_decode_subgroup_bis_cb(const struct bt_bap_base_subgroup_bis *bis, void *user_data)
{
	uint32_t *base_bis_index_bitfield = user_data;

	*base_bis_index_bitfield |= BT_ISO_BIS_INDEX_BIT(bis->index);

	return true;
}

static bool base_decode_subgroup_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
	struct bt_bap_broadcast_sink *sink = (struct bt_bap_broadcast_sink *)user_data;
	const struct bt_audio_codec_cap *codec_cap;
	struct bt_audio_codec_cfg codec_cfg;
	struct bt_pac_codec codec_id;
	int ret;

	if (sink->subgroup_count == ARRAY_SIZE(sink->subgroups)) {
		/* We've parsed as many subgroups as we support */
		LOG_DBG("Could only store %u subgroups", sink->subgroup_count);
		return false;
	}

	uint32_t *subgroup_bis_indexes = &sink->subgroups[sink->subgroup_count].bis_indexes;

	*subgroup_bis_indexes = 0;

	ret = bt_bap_base_subgroup_codec_to_codec_cfg(subgroup, &codec_cfg);
	if (ret < 0) {
		LOG_DBG("Could not store codec_cfg: %d", ret);
		return false;
	}

	/* Lookup and assign path_id based on capabilities */
	codec_id.id = codec_cfg.id;
	codec_id.cid = codec_cfg.cid;
	codec_id.vid = codec_cfg.vid;

	codec_cap = bt_pacs_get_codec_cap(BT_AUDIO_DIR_SINK, &codec_id);
	if (codec_cap == NULL) {
		LOG_DBG("Codec with id 0x%02x cid 0x%04x and vid 0x%04x is not supported by our "
			"capabilities",
			codec_id.id, codec_id.cid, codec_id.vid);
	} else {
		ret = bt_bap_base_subgroup_foreach_bis(subgroup, base_decode_subgroup_bis_cb,
						       subgroup_bis_indexes);

		if (ret != 0) {
			LOG_DBG("Could not parse BISes: %d", ret);
			return false;
		}

		sink->valid_indexes_bitfield |= *subgroup_bis_indexes;
	}

	sink->subgroup_count++;

	return true;
}

static bool pa_decode_base(struct bt_data *data, void *user_data)
{
	struct bt_bap_broadcast_sink *sink = (struct bt_bap_broadcast_sink *)user_data;
	const struct bt_bap_base *base = bt_bap_base_get_base_from_ad(data);
	struct bt_bap_broadcast_sink_cb *listener;
	int base_size;

	/* Base is NULL if the data does not contain a valid BASE */
	if (base == NULL) {
		return true;
	}

	/* We provide the BASE without the service data UUID */
	base_size = bt_bap_base_get_size(base);
	if (base_size != sink->base_size || memcmp(base, sink->base, base_size) != 0) {
		/* New BASE, parse */

		if (atomic_test_bit(sink->flags, BT_BAP_BROADCAST_SINK_FLAG_BIGINFO_RECEIVED)) {
			int ret;

			ret = base_get_bis_count(base);

			if (ret < 0) {
				LOG_DBG("Invalid BASE: %d", ret);
				return false;
			} else if (ret != sink->biginfo_num_bis) {
				LOG_DBG("BASE contains different amount of BIS (%u) than reported "
					"by BIGInfo (%u)",
					ret, sink->biginfo_num_bis);
				return false;
			}
		}

		/* Store newest BASE info until we are BIG synced */
		if (sink->big == NULL) {
			sink->qos_cfg.pd = bt_bap_base_get_pres_delay(base);

			sink->subgroup_count = 0;
			sink->valid_indexes_bitfield = 0;
			bt_bap_base_foreach_subgroup(base, base_decode_subgroup_cb, sink);

			LOG_DBG("Updating BASE for sink %p with %d subgroups\n", sink,
				sink->subgroup_count);

			memcpy(sink->base, base, base_size);
			sink->base_size = base_size;
		}

		if (atomic_test_bit(sink->flags, BT_BAP_BROADCAST_SINK_FLAG_SRC_ID_VALID)) {
			update_recv_state_base(sink, base);
		}
	}

	SYS_SLIST_FOR_EACH_CONTAINER(&sink_cbs, listener, _node) {
		if (listener->base_recv != NULL) {
			listener->base_recv(sink, base, (size_t)base_size);
		}
	}

	return false;
}

static void pa_recv(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	struct bt_bap_broadcast_sink *sink = broadcast_sink_get_by_pa(sync);

	if (sink == NULL) {
		/* Not a PA sync that we control */
		return;
	}

	if (sys_slist_is_empty(&sink_cbs)) {
		/* Terminate early if we do not have any broadcast sink listeners */
		return;
	}

	bt_data_parse(buf, pa_decode_base, (void *)sink);
}

static void pa_term_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_le_per_adv_sync_term_info *info)
{
	struct bt_bap_broadcast_sink *sink = broadcast_sink_get_by_pa(sync);

	if (sink != NULL) {
		sink->pa_sync = NULL;
		sink->base_size = 0U;
	}
}

static void update_recv_state_encryption(const struct bt_bap_broadcast_sink *sink)
{
	const struct bt_bap_scan_delegator_recv_state *recv_state;
	int err;

	__ASSERT(sink->big == NULL, "Encryption state shall not be updated while synced");

	recv_state = bt_bap_scan_delegator_find_state(find_recv_state_by_sink_cb, (void *)sink);
	if (recv_state == NULL) {
		LOG_WRN("Failed to find receive state for sink %p", sink);

		return;
	}

	(void)memset(&mod_src_param, 0, sizeof(mod_src_param));

	/* Only change the encrypt state, and leave the rest as is */
	if (atomic_test_bit(sink->flags,
			    BT_BAP_BROADCAST_SINK_FLAG_BIG_ENCRYPTED)) {
		mod_src_param.encrypt_state = BT_BAP_BIG_ENC_STATE_BCODE_REQ;
	} else {
		mod_src_param.encrypt_state = BT_BAP_BIG_ENC_STATE_NO_ENC;
	}

	if (mod_src_param.encrypt_state == recv_state->encrypt_state) {
		/* No change, abort*/
		return;
	}

	/* Copy existing data */
	/* TODO: Maybe we need more refined functions to set only specific fields? */
	mod_src_param.src_id = recv_state->src_id;
	mod_src_param.broadcast_id = recv_state->broadcast_id;
	mod_src_param.num_subgroups = recv_state->num_subgroups;
	(void)memcpy(mod_src_param.subgroups,
		     recv_state->subgroups,
		     sizeof(recv_state->num_subgroups));

	err = bt_bap_scan_delegator_mod_src(&mod_src_param);
	if (err != 0) {
		LOG_WRN("Failed to modify Receive State for sink %p: %d", sink, err);
	}
}

static void biginfo_recv(struct bt_le_per_adv_sync *sync,
			 const struct bt_iso_biginfo *biginfo)
{
	struct bt_bap_broadcast_sink_cb *listener;
	struct bt_bap_broadcast_sink *sink;

	sink = broadcast_sink_get_by_pa(sync);
	if (sink == NULL) {
		/* Not ours */
		return;
	}

	if (sink->big != NULL) {
		/* Already synced - ignore */
		return;
	}

	atomic_set_bit(sink->flags,
		       BT_BAP_BROADCAST_SINK_FLAG_BIGINFO_RECEIVED);
	sink->iso_interval = biginfo->iso_interval;
	sink->biginfo_num_bis = biginfo->num_bis;
	if (biginfo->encryption != atomic_test_bit(sink->flags,
						   BT_BAP_BROADCAST_SINK_FLAG_BIG_ENCRYPTED)) {
		atomic_set_bit_to(sink->flags,
				  BT_BAP_BROADCAST_SINK_FLAG_BIG_ENCRYPTED,
				  biginfo->encryption);

		if (atomic_test_bit(sink->flags,
				    BT_BAP_BROADCAST_SINK_FLAG_SRC_ID_VALID)) {
			update_recv_state_encryption(sink);
		}
	}

	sink->qos_cfg.framing = biginfo->framing;
	sink->qos_cfg.phy = biginfo->phy;
	sink->qos_cfg.sdu = biginfo->max_sdu;
	sink->qos_cfg.interval = biginfo->sdu_interval;

	SYS_SLIST_FOR_EACH_CONTAINER(&sink_cbs, listener, _node) {
		if (listener->syncable != NULL) {
			listener->syncable(sink, biginfo);
		}
	}
}

static uint16_t interval_to_sync_timeout(uint16_t interval)
{
	uint32_t interval_us;
	uint32_t timeout;

	/* Add retries and convert to unit in 10's of ms */
	interval_us = BT_GAP_PER_ADV_INTERVAL_TO_US(interval);
	timeout =
		BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(interval_us) * PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO;

	/* Enforce restraints */
	timeout = CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);

	return (uint16_t)timeout;
}

static void big_started_cb(struct bt_iso_big *big)
{
	struct bt_bap_broadcast_sink *sink = broadcast_sink_get_by_big(big);
	struct bt_bap_broadcast_sink_cb *listener;

	if (sink == NULL) {
		/* Not one of ours */
		return;
	}

	SYS_SLIST_FOR_EACH_CONTAINER(&sink_cbs, listener, _node) {
		if (listener->started != NULL) {
			listener->started(sink);
		}
	}
}

static void big_stopped_cb(struct bt_iso_big *big, uint8_t reason)
{
	struct bt_bap_broadcast_sink *sink = broadcast_sink_get_by_big(big);
	struct bt_bap_broadcast_sink_cb *listener;

	if (sink == NULL) {
		/* Not one of ours */
		return;
	}

	broadcast_sink_clear_big(sink, reason);

	SYS_SLIST_FOR_EACH_CONTAINER(&sink_cbs, listener, _node) {
		if (listener->stopped != NULL) {
			listener->stopped(sink, reason);
		}
	}
}

int bt_bap_broadcast_sink_register_cb(struct bt_bap_broadcast_sink_cb *cb)
{
	static bool iso_big_cb_registered;

	CHECKIF(cb == NULL) {
		LOG_DBG("cb is NULL");

		return -EINVAL;
	}

	if (sys_slist_find(&sink_cbs, &cb->_node, NULL)) {
		LOG_DBG("cb %p is already registered", cb);

		return -EEXIST;
	}

	if (!iso_big_cb_registered) {
		static struct bt_iso_big_cb big_cb = {
			.started = big_started_cb,
			.stopped = big_stopped_cb,
		};
		const int err = bt_iso_big_register_cb(&big_cb);

		if (err != 0) {
			__ASSERT(false, "Failed to register ISO BIG callbacks: %d", err);
		}

		iso_big_cb_registered = true;
	}

	sys_slist_append(&sink_cbs, &cb->_node);

	return 0;
}

bool bt_bap_ep_is_broadcast_snk(const struct bt_bap_ep *ep)
{
	for (int i = 0; i < ARRAY_SIZE(broadcast_sink_eps); i++) {
		if (PART_OF_ARRAY(broadcast_sink_eps[i], ep)) {
			return true;
		}
	}

	return false;
}

static void broadcast_sink_ep_init(struct bt_bap_ep *ep)
{
	LOG_DBG("ep %p", ep);

	(void)memset(ep, 0, sizeof(*ep));
	ep->dir = BT_AUDIO_DIR_SINK;
	ep->iso = NULL;
}

static struct bt_bap_ep *broadcast_sink_new_ep(uint8_t index)
{
	for (size_t i = 0; i < ARRAY_SIZE(broadcast_sink_eps[index]); i++) {
		struct bt_bap_ep *ep = &broadcast_sink_eps[index][i];

		/* If ep->stream is NULL the endpoint is unallocated */
		if (ep->stream == NULL) {
			broadcast_sink_ep_init(ep);
			return ep;
		}
	}

	return NULL;
}

static int bt_bap_broadcast_sink_setup_stream(struct bt_bap_broadcast_sink *sink,
					      struct bt_bap_stream *stream,
					      struct bt_audio_codec_cfg *codec_cfg)
{
	struct bt_bap_iso *iso;
	struct bt_bap_ep *ep;

	if (stream->group != NULL) {
		LOG_DBG("Stream %p already in group %p", stream, stream->group);
		return -EALREADY;
	}

	ep = broadcast_sink_new_ep(sink->index);
	if (ep == NULL) {
		LOG_DBG("Could not allocate new broadcast endpoint");
		return -ENOMEM;
	}

	iso = bt_bap_iso_new();
	if (iso == NULL) {
		LOG_DBG("Could not allocate iso");
		return -ENOMEM;
	}

	bt_bap_iso_init(iso, &broadcast_sink_iso_ops);
	bt_bap_iso_bind_ep(iso, ep);

	bt_bap_qos_cfg_to_iso_qos(iso->chan.qos->rx, &sink->qos_cfg);

	bt_bap_iso_unref(iso);

	bt_bap_stream_attach(NULL, stream, ep, codec_cfg);
	stream->qos = &sink->qos_cfg;

	return 0;
}

static void broadcast_sink_cleanup_streams(struct bt_bap_broadcast_sink *sink)
{
	struct bt_bap_stream *stream, *next;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&sink->streams, stream, next, _node) {
		if (stream->ep != NULL) {
			bt_bap_iso_unbind_ep(stream->ep->iso, stream->ep);
			stream->ep->stream = NULL;
			stream->ep = NULL;
		}

		stream->qos = NULL;
		stream->codec_cfg = NULL;
		stream->group = NULL;

		sys_slist_remove(&sink->streams, NULL, &stream->_node);
	}

	sink->stream_count = 0;
	sink->indexes_bitfield = 0U;
}

static void broadcast_sink_cleanup(struct bt_bap_broadcast_sink *sink)
{
	if (sink->stream_count > 0U) {
		broadcast_sink_cleanup_streams(sink);
	}

	(void)memset(sink, 0, sizeof(*sink)); /* also clears flags */
}

int bt_bap_broadcast_sink_create(struct bt_le_per_adv_sync *pa_sync, uint32_t broadcast_id,
				 struct bt_bap_broadcast_sink **out_sink)
{
	const struct bt_bap_scan_delegator_recv_state *recv_state;
	struct bt_bap_broadcast_sink *sink;

	CHECKIF(pa_sync == NULL) {
		LOG_DBG("pa_sync is NULL");

		return -EINVAL;
	}

	CHECKIF(broadcast_id > BT_AUDIO_BROADCAST_ID_MAX) {
		LOG_DBG("Invalid broadcast_id: 0x%X", broadcast_id);

		return -EINVAL;
	}

	CHECKIF(out_sink == NULL) {
		LOG_DBG("sink was NULL");

		return -EINVAL;
	}

	sink = broadcast_sink_free_get();
	if (sink == NULL) {
		LOG_DBG("No more free broadcast sinks");

		return -ENOMEM;
	}

	sink->broadcast_id = broadcast_id;
	sink->pa_sync = pa_sync;

	recv_state = bt_bap_scan_delegator_find_state(find_recv_state_by_pa_sync_cb,
						      (void *)pa_sync);
	if (recv_state == NULL) {
		broadcast_sink_add_src(sink);
	} else {
		/* The PA sync is known by the Scan Delegator */
		if (recv_state->broadcast_id != broadcast_id) {
			LOG_DBG("Broadcast ID mismatch: 0x%X != 0x%X",
				recv_state->broadcast_id, broadcast_id);

			broadcast_sink_cleanup(sink);
			return -EINVAL;
		}

		sink->bass_src_id = recv_state->src_id;
		atomic_set_bit(sink->flags, BT_BAP_BROADCAST_SINK_FLAG_SRC_ID_VALID);
	}
	atomic_set_bit(sink->flags, BT_BAP_BROADCAST_SINK_FLAG_INITIALIZED);

	*out_sink = sink;
	return 0;
}

static uint8_t bit_count(uint32_t bitfield)
{
#ifdef POPCOUNT
	return POPCOUNT(bitfield);
#else
	uint8_t cnt = 0U;

	while (bitfield != 0U) {
		cnt += bitfield & 1U;
		bitfield >>= 1U;
	}

	return cnt;
#endif
}

struct sync_base_info_data {
	struct bt_audio_codec_cfg codec_cfgs[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
	struct bt_audio_codec_cfg *subgroup_codec_cfg;
	uint32_t sync_indexes_bitfield;
	uint8_t subgroup_count;
	uint8_t stream_count;
};

static bool merge_bis_and_subgroup_data_cb(struct bt_data *data, void *user_data)
{
	struct bt_audio_codec_cfg *codec_cfg = user_data;
	int err;

	err = bt_audio_codec_cfg_set_val(codec_cfg, data->type, data->data, data->data_len);
	if (err < 0) {
		LOG_DBG("Failed to set type %u with len %u in codec_cfg: %d", data->type,
			data->data_len, err);

		return false;
	}

	return true;
}

static bool sync_base_subgroup_bis_index_cb(const struct bt_bap_base_subgroup_bis *bis,
					    void *user_data)
{
	struct sync_base_info_data *data = user_data;
	struct bt_audio_codec_cfg *codec_cfg;

	/* Only process selected BISes */
	if ((data->sync_indexes_bitfield & BT_ISO_BIS_INDEX_BIT(bis->index)) == 0) {
		return true;
	}

#if CONFIG_BT_AUDIO_CODEC_CFG_MAX_DATA_SIZE > 0

	codec_cfg = &data->codec_cfgs[data->stream_count];

	memcpy(codec_cfg, data->subgroup_codec_cfg, sizeof(struct bt_audio_codec_cfg));

	if (bis->data_len > 0) {
		/* Merge subgroup codec configuration with the BIS configuration
		 * As per the BAP spec, if a value exist at level 2 (subgroup) and 3 (BIS), then it
		 * is the value at level 3 that shall be used
		 */
		if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
			int err;

			memcpy(codec_cfg, data->subgroup_codec_cfg,
			       sizeof(struct bt_audio_codec_cfg));

			err = bt_audio_data_parse(bis->data, bis->data_len,
						  merge_bis_and_subgroup_data_cb, codec_cfg);
			if (err != 0) {
				LOG_DBG("Could not merge BIS and subgroup config in codec_cfg: %d",
					err);

				return false;
			}
		} else {
			/* If it is not LC3, then we don't know how to merge the subgroup and BIS
			 * codecs, so we just append them
			 */
			if (codec_cfg->data_len + bis->data_len > sizeof(codec_cfg->data)) {
				LOG_DBG("Could not store BIS and subgroup config in codec_cfg (%u "
					"> %u)",
					codec_cfg->data_len + bis->data_len,
					sizeof(codec_cfg->data));

				return false;
			}

			memcpy(&codec_cfg->data[codec_cfg->data_len], bis->data, bis->data_len);
			codec_cfg->data_len += bis->data_len;
		}
	}
#endif /* CONFIG_BT_AUDIO_CODEC_CFG_MAX_DATA_SIZE > 0 */

	data->stream_count++;

	return true;
}

static bool sync_base_subgroup_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
	struct sync_base_info_data *data = user_data;
	const struct bt_audio_codec_cap *codec_cap;
	struct bt_audio_codec_cfg codec_cfg;
	struct bt_pac_codec codec_id;
	int ret;

	if (data->subgroup_count == CONFIG_BT_BAP_BROADCAST_SNK_SUBGROUP_COUNT) {
		/* We've parsed as many subgroups as we support */
		LOG_DBG("Could only store %u subgroups", data->subgroup_count);
		return false;
	}

	ret = bt_bap_base_subgroup_codec_to_codec_cfg(subgroup, &codec_cfg);
	if (ret < 0) {
		LOG_DBG("Could not store codec_cfg: %d", ret);
		return false;
	}

	/* Lookup and assign path_id based on capabilities */
	codec_id.id = codec_cfg.id;
	codec_id.cid = codec_cfg.cid;
	codec_id.vid = codec_cfg.vid;

	codec_cap = bt_pacs_get_codec_cap(BT_AUDIO_DIR_SINK, &codec_id);
	if (codec_cap == NULL) {
		LOG_DBG("Codec with id 0x%02x cid 0x%04x and vid 0x%04x is not supported by our "
			"capabilities",
			codec_id.id, codec_id.cid, codec_id.vid);
	} else {
		codec_cfg.path_id = codec_cap->path_id;
		codec_cfg.ctlr_transcode = codec_cap->ctlr_transcode;

		data->subgroup_codec_cfg = &codec_cfg;

		ret = bt_bap_base_subgroup_foreach_bis(subgroup, sync_base_subgroup_bis_index_cb,
						       data);
		if (ret < 0) {
			LOG_DBG("Could not parse BISes: %d", ret);
			return false;
		}

		data->subgroup_count++;
	}

	return true;
}

int bt_bap_broadcast_sink_sync(struct bt_bap_broadcast_sink *sink, uint32_t indexes_bitfield,
			       struct bt_bap_stream *streams[],
			       const uint8_t broadcast_code[BT_ISO_BROADCAST_CODE_SIZE])
{
	static struct sync_base_info_data data;
	struct bt_iso_big_sync_param param;
	struct bt_iso_chan *bis_channels[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
	uint8_t bis_count;
	uint8_t stream_count;
	int err;
	int ret;

	CHECKIF(sink == NULL) {
		LOG_DBG("sink is NULL");
		return -EINVAL;
	}

	CHECKIF(indexes_bitfield == 0U || indexes_bitfield > BIT_MASK(BT_ISO_BIS_INDEX_MAX)) {
		LOG_DBG("Invalid indexes_bitfield: 0x%08X", indexes_bitfield);
		return -EINVAL;
	}

	CHECKIF(streams == NULL) {
		LOG_DBG("streams is NULL");
		return -EINVAL;
	}

	if (sink->pa_sync == NULL) {
		LOG_DBG("Sink is not PA synced");
		return -EINVAL;
	}

	if (!atomic_test_bit(sink->flags,
			     BT_BAP_BROADCAST_SINK_FLAG_BIGINFO_RECEIVED)) {
		/* TODO: We could store the request to sync and start the sync
		 * once the BIGInfo has been received, and then do the sync
		 * then. This would be similar how LE Create Connection works.
		 */
		LOG_DBG("BIGInfo not received, cannot sync yet");
		return -EAGAIN;
	}

	if (atomic_test_bit(sink->flags,
			    BT_BAP_BROADCAST_SINK_FLAG_BIG_ENCRYPTED) &&
	    broadcast_code == NULL) {
		LOG_DBG("Broadcast code required");

		return -EINVAL;
	}

	/* Validate that number of bits set is within supported range */
	bis_count = bit_count(indexes_bitfield);
	if (bis_count > CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT) {
		LOG_DBG("Cannot sync to more than %d streams (%u was requested)",
			CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT, bis_count);
		return -EINVAL;
	}

	/* Validate that the bits set are present in BASE */
	if ((indexes_bitfield & sink->valid_indexes_bitfield) != indexes_bitfield) {
		LOG_DBG("Request BIS indexes (0x%08X) contains bits not present in BASE (0x%08X)",
			indexes_bitfield, sink->valid_indexes_bitfield);
		return -EINVAL;
	}

	memset(&data, 0, sizeof(data));

	data.sync_indexes_bitfield = indexes_bitfield;

	ret = bt_bap_base_foreach_subgroup((const struct bt_bap_base *)sink->base,
					   sync_base_subgroup_cb, &data);
	if (ret != 0) {
		LOG_DBG("Failed to parse all subgroups: %d", ret);
		return ret;
	}

	stream_count = data.stream_count;

	for (size_t i = 0; i < stream_count; i++) {
		CHECKIF(streams[i] == NULL) {
			LOG_DBG("streams[%zu] is NULL", i);
			return -EINVAL;
		}
	}

	sink->stream_count = 0U;
	for (size_t i = 0; i < stream_count; i++) {
		struct bt_bap_stream *stream;
		struct bt_audio_codec_cfg *codec_cfg;

		stream = streams[i];
		codec_cfg = &data.codec_cfgs[i];

		err = bt_bap_broadcast_sink_setup_stream(sink, stream, codec_cfg);
		if (err != 0) {
			LOG_DBG("Failed to setup streams[%zu]: %d", i, err);
			broadcast_sink_cleanup_streams(sink);
			return err;
		}

		sink->bis[i].chan = bt_bap_stream_iso_chan_get(stream);
		sys_slist_append(&sink->streams, &stream->_node);
		sink->stream_count++;

		bis_channels[i] = sink->bis[i].chan;
	}

	param.bis_channels = bis_channels;
	param.num_bis = sink->stream_count;
	param.bis_bitfield = indexes_bitfield;
	param.mse = 0; /* Let controller decide */
	param.sync_timeout = interval_to_sync_timeout(sink->iso_interval);
	param.encryption = atomic_test_bit(sink->flags,
					   BT_BAP_BROADCAST_SINK_FLAG_BIG_ENCRYPTED);
	if (param.encryption) {
		memcpy(param.bcode, broadcast_code, sizeof(param.bcode));
	} else {
		memset(param.bcode, 0, sizeof(param.bcode));
	}

	err = bt_iso_big_sync(sink->pa_sync, &param, &sink->big);
	if (err != 0) {
		broadcast_sink_cleanup_streams(sink);
		return err;
	}

	sink->indexes_bitfield = indexes_bitfield;
	for (size_t i = 0; i < stream_count; i++) {
		struct bt_bap_ep *ep = streams[i]->ep;

		ep->broadcast_sink = sink;
		broadcast_sink_set_ep_state(ep, BT_BAP_EP_STATE_QOS_CONFIGURED);
	}

	return 0;
}

int bt_bap_broadcast_sink_stop(struct bt_bap_broadcast_sink *sink)
{
	int err;

	CHECKIF(sink == NULL) {
		LOG_DBG("sink is NULL");
		return -EINVAL;
	}

	if (sys_slist_is_empty(&sink->streams)) {
		LOG_DBG("Source does not have any streams (already stopped)");
		return -EALREADY;
	}

	if (broadcast_sink_is_in_state(sink, BT_BAP_EP_STATE_IDLE)) {
		LOG_DBG("Broadcast sink %p in idle state", sink);
		return -EBADMSG;
	}

	err = bt_iso_big_terminate(sink->big);
	if (err) {
		LOG_DBG("Failed to terminate BIG (err %d)", err);
		return err;
	}

	return 0;
}

int bt_bap_broadcast_sink_delete(struct bt_bap_broadcast_sink *sink)
{

	CHECKIF(sink == NULL) {
		LOG_DBG("sink is NULL");
		return -EINVAL;
	}

	if (!broadcast_sink_is_in_state(sink, BT_BAP_EP_STATE_IDLE)) {
		LOG_DBG("Broadcast sink %p not in idle state", sink);
		return -EBADMSG;
	}

	/* Reset the broadcast sink */
	broadcast_sink_cleanup(sink);

	return 0;
}

static int broadcast_sink_init(void)
{
	static struct bt_le_per_adv_sync_cb cb = {
		.recv = pa_recv,
		.biginfo = biginfo_recv,
		.term = pa_term_cb,
	};

	bt_le_per_adv_sync_cb_register(&cb);

	return 0;
}

SYS_INIT(broadcast_sink_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
