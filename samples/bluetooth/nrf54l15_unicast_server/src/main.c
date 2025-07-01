/*
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define AVAILABLE_SINK_CONTEXT  (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
				 BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
				 BT_AUDIO_CONTEXT_TYPE_MEDIA | \
				 BT_AUDIO_CONTEXT_TYPE_GAME | \
				 BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_PROHIBITED

NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_ANY, BT_AUDIO_CODEC_CAP_DURATION_10,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(2), 80u, 240u, 1u,
	AVAILABLE_SINK_CONTEXT);

static struct bt_conn *default_conn;
static struct k_work_delayable audio_send_work;
static struct bt_bap_stream sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static struct audio_source {
	struct bt_bap_stream stream;
	uint16_t seq_num;
	uint16_t max_sdu;
	size_t len_to_send;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];
static size_t configured_source_stream_count;

static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 40000, 10000, 40000);

static uint8_t unicast_server_addata[] = {
	BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL), /* ASCS UUID */
	BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, /* Target Announcement */
	BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
	BT_BYTES_LIST_LE16(AVAILABLE_SOURCE_CONTEXT),
	0x00, /* Metadata length */
};

/* TODO: Expand with BAP data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
	BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

#define AUDIO_DATA_TIMEOUT_US 1000000UL /* Send data every 1 second */
#define SDU_INTERVAL_US       10000UL   /* 10 ms SDU interval */

static struct bt_le_ext_adv *adv;
static struct k_work adv_work;

static void advertising_process(struct k_work *work)
{
       int err;
       err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
       if (err) {
               LOG_INF("Failed to start advertising set (err %d)", err);
       }
       LOG_INF("Advertising successfully started");
}

#include "lc3.h"

#define MAX_SAMPLE_RATE         48000
#define MAX_FRAME_DURATION_US   10000
#define MAX_NUM_SAMPLES         ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

static int16_t audio_buf[MAX_NUM_SAMPLES*2];
static lc3_decoder_t lc3_decoder[2];
static lc3_decoder_mem_48k_t lc3_decoder_mem[2];
static int frames_per_sdu;


void print_hex(const uint8_t *ptr, size_t len)
{
	while (len-- != 0) {
		printk("%02x", *ptr++);
	}
}

static bool print_cb(struct bt_data *data, void *user_data)
{
	const char *str = (const char *)user_data;

	LOG_INF("%s: type 0x%02x value_len %u", str, data->type, data->data_len);
	LOG_HEXDUMP_INF(data->data, data->data_len, "value:");

	return true;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	LOG_INF("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u", codec_cfg->id, codec_cfg->cid,
	       codec_cfg->vid, codec_cfg->data_len);

	if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
		enum bt_audio_location chan_allocation;
		int ret;

		/* LC3 uses the generic LTV format - other codecs might do as well */

		bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, print_cb, "data");

		ret = bt_audio_codec_cfg_get_freq(codec_cfg);
		if (ret > 0) {
			LOG_INF("  Frequency: %d Hz", bt_audio_codec_cfg_freq_to_freq_hz(ret));
		}

		ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
		if (ret > 0) {
			LOG_INF("  Frame Duration: %d us",
			       bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
		}

		ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
		if (ret == 0) {
			LOG_INF("  Channel allocation: 0x%x", chan_allocation);
		}

		LOG_INF("  Octets per frame: %d (negative means value not pressent)",
		       bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
		LOG_INF("  Frames per SDU: %d",
		       bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
	} else {
		print_hex(codec_cfg->data, codec_cfg->data_len);
	}

	bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
	LOG_INF("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
	       "rtn %u latency %u pd %u",
	       qos->interval, qos->framing, qos->phy, qos->sdu,
	       qos->rtn, qos->latency, qos->pd);
}

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++) {
		if (stream == &source_streams[i].stream) {
			return BT_AUDIO_DIR_SOURCE;
		}
	}

	for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
		if (stream == &sink_streams[i]) {
			return BT_AUDIO_DIR_SINK;
		}
	}

	__ASSERT(false, "Invalid stream %p", (void *) stream);
	return 0;
}

static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir)
{
	if (dir == BT_AUDIO_DIR_SOURCE) {
		for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
			struct bt_bap_stream *stream = &source_streams[i].stream;

			if (!stream->conn) {
				return stream;
			}
		}
	} else {
		for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++) {
			struct bt_bap_stream *stream = &sink_streams[i];

			if (!stream->conn) {
				return stream;
			}
		}
	}

	return NULL;
}

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("ASE Codec Config: conn %p ep %p dir %u", (void *) conn, (void *) ep, dir);

	print_codec_cfg(codec_cfg);

	*stream = stream_alloc(dir);
	if (*stream == NULL) {
		LOG_INF("No streams available");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	LOG_INF("ASE Codec Config stream %p", (void *) *stream);

	if (dir == BT_AUDIO_DIR_SOURCE) {
		configured_source_stream_count++;
	}

	*pref = qos_pref;

	/* Nothing to free as static memory is used */
	for (int i = 0; i < 2; i++) {
			lc3_decoder[i] = NULL;
	}

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("ASE Codec Reconfig: stream %p", (void *)stream);

	print_codec_cfg(codec_cfg);

	/* Nothing to free as static memory is used */
	for(int i = 0; i < 2; i++) {
		lc3_decoder[i] = NULL;
	}

	*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);

	/* We only support one QoS at the moment, reject changes */
	return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("QoS: stream %p qos %p", (void *)stream, (void *)qos);

	print_qos(qos);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].max_sdu = qos->sdu;
			break;
		}
	}

	return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
		      struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Enable: stream %p meta_len %zu", (void *)stream, meta_len);

	int frame_duration_us;
	int freq;
	int ret;

	ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
	if (ret > 0) {
		freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
	} else {
		LOG_INF("Error: Codec frequency not set, cannot start codec.");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
						BT_BAP_ASCS_REASON_CODEC_DATA);
		return ret;
	}

	ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
	if (ret > 0) {
		frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
	} else {
		LOG_INF("Error: Frame duration not set, cannot start codec.");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
						BT_BAP_ASCS_REASON_CODEC_DATA);
		return ret;
	}

	frames_per_sdu =
		bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);

	for (int i = 0; i <2; i++){
		lc3_decoder[i] = lc3_setup_decoder(frame_duration_us,
						freq,
						0, /* No resampling */
						&lc3_decoder_mem[i]);

		if (lc3_decoder[i] == NULL) {
			LOG_INF("ERROR: Failed to setup LC3 encoder - wrong parameters?");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
						BT_BAP_ASCS_REASON_CODEC_DATA);
			return -1;
		}
	}


	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Start: stream %p", (void *)stream);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].seq_num = 0U;
			source_streams[i].len_to_send = 0U;
			break;
		}
	}

	if (configured_source_stream_count > 0 &&
	    !k_work_delayable_is_pending(&audio_send_work)) {

		/* Start send timer */
		k_work_schedule(&audio_send_work, K_MSEC(0));
	}

	return 0;
}

static bool data_func_cb(struct bt_data *data, void *user_data)
{
	struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

	if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type)) {
		LOG_INF("Invalid metadata type %u or length %u", data->type, data->data_len);
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

		return -EINVAL;
	}

	return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
			struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Metadata: stream %p meta_len %zu", (void *)stream, meta_len);

	return bt_audio_data_parse(meta, meta_len, data_func_cb, rsp);
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Disable: stream %p", (void *)stream);

	return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Stop: stream %p", (void *)stream);

	return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Release: stream %p", (void *)stream);
	return 0;
}

static struct bt_bap_unicast_server_register_param param = {
	CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
	CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT
};

static const struct bt_bap_unicast_server_cb unicast_server_cb = {
	.config = lc3_config,
	.reconfig = lc3_reconfig,
	.qos = lc3_qos,
	.enable = lc3_enable,
	.start = lc3_start,
	.metadata = lc3_metadata,
	.disable = lc3_disable,
	.stop = lc3_stop,
	.release = lc3_release,
};

static void stream_recv_lc3_codec(struct bt_bap_stream *stream,
				  const struct bt_iso_recv_info *info,
				  struct net_buf *buf)
{
	const bool valid_data = (info->flags & BT_ISO_FLAGS_VALID) != 0;
	const int octets_per_frame = buf->len / frames_per_sdu;

	for (int i = 0; i < 2; i++) {
		if (lc3_decoder[i] == NULL) {
			LOG_INF("LC3 decoder not setup, cannot decode data.");
			return;
		}
	}


	if (!valid_data) {
		LOG_INF("Bad packet: 0x%02X", info->flags);
	}
	//LOG_INF("RX stream %p len %u", stream, buf->len);

	for (int i = 0; i < frames_per_sdu; i++) {
		for (int j = 0; j < 2; j++) {
			const int err = lc3_decode(
				lc3_decoder[j], valid_data ? net_buf_pull_mem(buf, octets_per_frame/2) : NULL,
				octets_per_frame/2, LC3_PCM_FORMAT_S16, audio_buf+1, 2);
			if (err == 1) {
				LOG_INF("[%d]: Decoder performed PLC", i);
			} else if (err < 0) {
				LOG_INF("[%d]: Decoder failed - wrong parameters?: %d", i, err);
			}
		}
	}
}


static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	LOG_INF("Audio Stream %p stopped with reason 0x%02X", (void *)stream, reason);

	/* Stop send timer */
	k_work_cancel_delayable(&audio_send_work);
}

static void stream_started(struct bt_bap_stream *stream)
{
	LOG_INF("Audio Stream %p started", (void *)stream);
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
	/* The unicast server is responsible for starting sink ASEs after the
	 * client has enabled them.
	 */
	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		const int err = bt_bap_stream_start(stream);

		if (err != 0) {
			LOG_INF("Failed to start stream %p: %d", (void *)stream, err);
		}
	}
}

static struct bt_bap_stream_ops stream_ops = {
	.recv = stream_recv_lc3_codec,
	.stopped = stream_stopped,
	.started = stream_started,
	.enabled = stream_enabled_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		LOG_INF("Failed to connect to %s %u %s", addr, err, bt_hci_err_to_str(err));

		default_conn = NULL;
		return;
	}

	LOG_INF("Connected: %s", addr);
	default_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(default_conn);
	default_conn = NULL;

	configured_source_stream_count = 0;
	k_work_cancel_delayable(&audio_send_work);

	k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static struct bt_pacs_cap cap_sink = {
	.codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};

static int set_location(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SINK,
						(BT_AUDIO_LOCATION_FRONT_LEFT |
					    BT_AUDIO_LOCATION_FRONT_RIGHT));
		if (err != 0) {
			LOG_INF("Failed to set sink location (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE,
					   (BT_AUDIO_LOCATION_FRONT_LEFT |
					    BT_AUDIO_LOCATION_FRONT_RIGHT));
		if (err != 0) {
			LOG_INF("Failed to set source location (err %d)", err);
			return err;
		}
	}

	LOG_INF("Location successfully set");

	return 0;
}

static int set_supported_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK,
						     AVAILABLE_SINK_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set sink supported contexts (err %d)",
			       err);

			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
						     AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set source supported contexts (err %d)",
			       err);

			return err;
		}
	}

	LOG_INF("Supported contexts successfully set");

	return 0;
}

static int set_available_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK,
						     AVAILABLE_SINK_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set sink available contexts (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
						     AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set source available contexts (err %d)", err);
			return err;
		}
	}

	LOG_INF("Available contexts successfully set");
	return 0;
}

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err != 0) {
		LOG_INF("Bluetooth init failed (err %d)", err);
		return 0;
	}

	LOG_INF("Bluetooth initialized");

	bt_bap_unicast_server_register(&param);
	bt_bap_unicast_server_register_cb(&unicast_server_cb);

	bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
	bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);

	for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++) {
		bt_bap_stream_cb_register(&sink_streams[i], &stream_ops);
	}

	for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
		bt_bap_stream_cb_register(&source_streams[i].stream,
					    &stream_ops);
	}

	err = set_location();
	if (err != 0) {
		return 0;
	}

	err = set_supported_contexts();
	if (err != 0) {
		return 0;
	}

	err = set_available_contexts();
	if (err != 0) {
		return 0;
	}

	/* Create a connectable advertising set */
	err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_CONN_QUICK, NULL, &adv);
	if (err) {
		LOG_INF("Failed to create advertising set (err %d)", err);
		return 0;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Failed to set advertising data (err %d)", err);
		return 0;
	}

    k_work_init(&adv_work, advertising_process);
    k_work_submit(&adv_work);

	while (true) {
		k_sleep(K_SECONDS(1));
	}
	return 0;
}
