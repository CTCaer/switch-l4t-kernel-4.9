/*
 * Copyright (c) 2014 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/vmalloc.h>
#include <linux/wakelock.h>
#include <net/cfg80211.h>
#include <net/netlink.h>

#include <brcmu_wifi.h>
#include "fwil_types.h"
#include "core.h"
#include "p2p.h"
#include "debug.h"
#include "cfg80211.h"
#include "vendor.h"
#include "fwil.h"
#include "android.h"

enum andr_vendor_subcmd {
	GSCAN_SUBCMD_GET_CAPABILITIES = 0x1000,
	GSCAN_SUBCMD_SET_CONFIG,
	GSCAN_SUBCMD_SET_SCAN_CONFIG,
	GSCAN_SUBCMD_ENABLE_GSCAN,
	GSCAN_SUBCMD_GET_SCAN_RESULTS,
	GSCAN_SUBCMD_SCAN_RESULTS,
	GSCAN_SUBCMD_SET_HOTLIST,
	GSCAN_SUBCMD_SET_SIGNIFICANT_CHANGE_CONFIG,
	GSCAN_SUBCMD_ENABLE_FULL_SCAN_RESULTS,
	GSCAN_SUBCMD_GET_CHANNEL_LIST,
	ANDR_WIFI_SUBCMD_GET_FEATURE_SET,
	ANDR_WIFI_SUBCMD_GET_FEATURE_SET_MATRIX,
	ANDR_WIFI_RANDOM_MAC_OUI,
	ANDR_WIFI_NODFS_CHANNELS,
	ANDR_WIFI_SET_COUNTRY
};

enum gscan_attributes {
	GSCAN_ATTRIBUTE_NUM_BUCKETS = 10,
	GSCAN_ATTRIBUTE_BASE_PERIOD,
	GSCAN_ATTRIBUTE_BUCKETS_BAND,
	GSCAN_ATTRIBUTE_BUCKET_ID,
	GSCAN_ATTRIBUTE_BUCKET_PERIOD,
	GSCAN_ATTRIBUTE_BUCKET_NUM_CHANNELS,
	GSCAN_ATTRIBUTE_BUCKET_CHANNELS,
	GSCAN_ATTRIBUTE_NUM_AP_PER_SCAN,
	GSCAN_ATTRIBUTE_REPORT_THRESHOLD,
	GSCAN_ATTRIBUTE_NUM_SCANS_TO_CACHE,
	GSCAN_ATTRIBUTE_BAND = GSCAN_ATTRIBUTE_BUCKETS_BAND,

	GSCAN_ATTRIBUTE_ENABLE_FEATURE = 20,
	GSCAN_ATTRIBUTE_SCAN_RESULTS_COMPLETE,
	GSCAN_ATTRIBUTE_FLUSH_FEATURE,
	GSCAN_ATTRIBUTE_ENABLE_FULL_SCAN_RESULTS,
	GSCAN_ATTRIBUTE_REPORT_EVENTS,
	GSCAN_ATTRIBUTE_NUM_OF_RESULTS = 30,
	GSCAN_ATTRIBUTE_FLUSH_RESULTS,
	GSCAN_ATTRIBUTE_SCAN_RESULTS,
	GSCAN_ATTRIBUTE_SCAN_ID,
	GSCAN_ATTRIBUTE_SCAN_FLAGS,
	GSCAN_ATTRIBUTE_AP_FLAGS,
	GSCAN_ATTRIBUTE_NUM_CHANNELS,
	GSCAN_ATTRIBUTE_CHANNEL_LIST,
	GSCAN_ATTRIBUTE_CH_BUCKET_BITMASK
};

enum andr_wifi_attr {
	ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET,
	ANDR_WIFI_ATTRIBUTE_FEATURE_SET,
	ANDR_WIFI_ATTRIBUTE_RANDOM_MAC_OUI,
	ANDR_WIFI_ATTRIBUTE_NODFS_SET,
	ANDR_WIFI_ATTRIBUTE_COUNTRY,
	ANDR_WIFI_ATTRIBUTE_ND_OFFLOAD_VALUE,
	ANDR_WIFI_ATTRIBUTE_TCPACK_SUP_VALUE
};

#define GSCAN_BG_BAND_MASK	0x1
#define GSCAN_A_BAND_MASK	0x2
#define GSCAN_DFS_MASK		0x4
#define GSCAN_ABG_BAND_MASK	(GSCAN_A_BAND_MASK | GSCAN_BG_BAND_MASK)
#define GSCAN_BAND_MASK		(GSCAN_ABG_BAND_MASK | GSCAN_DFS_MASK)

/* Basic infrastructure mode */
#define WIFI_FEATURE_INFRA		0x0001
/* Support for 5 GHz Band */
#define WIFI_FEATURE_INFRA_5G		0x0002
/* Support for GAS/ANQP */
#define WIFI_FEATURE_HOTSPOT		0x0004
/* Wifi-Direct */
#define WIFI_FEATURE_P2P		0x0008
/* Soft AP */
#define WIFI_FEATURE_SOFT_AP		0x0010
/* Google-Scan APIs */
#define WIFI_FEATURE_GSCAN		0x0020
/* Neighbor Awareness Networking */
#define WIFI_FEATURE_NAN		0x0040
/* Device-to-device RTT */
#define WIFI_FEATURE_D2D_RTT		0x0080
/* Device-to-AP RTT */
#define WIFI_FEATURE_D2AP_RTT		0x0100
/* Batched Scan (legacy) */
#define WIFI_FEATURE_BATCH_SCAN		0x0200
/* Preferred network offload */
#define WIFI_FEATURE_PNO		0x0400
/* Support for two STAs */
#define WIFI_FEATURE_ADDITIONAL_STA	0x0800
/* Tunnel directed link setup */
#define WIFI_FEATURE_TDLS		0x1000
/* Support for TDLS off channel */
#define WIFI_FEATURE_TDLS_OFFCHANNEL	0x2000
/* Enhanced power reporting */
#define WIFI_FEATURE_EPR		0x4000
/* Support for AP STA Concurrency */
#define WIFI_FEATURE_AP_STA		0x8000
/* Support for Linkstats */
#define WIFI_FEATURE_LINKSTAT		0x10000
/* WiFi PNO enhanced */
#define WIFI_FEATURE_HAL_EPNO		0x40000
/* RSSI Monitor */
#define WIFI_FEATURE_RSSI_MONITOR	0x80000
/* ND offload configure */
#define WIFI_FEATURE_CONFIG_NDO		0x200000
/* Invalid Feature */
#define WIFI_FEATURE_INVALID		0xFFFFFFFF

static int brcmf_cfg80211_vndr_cmds_dcmd_handler(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data, int len)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	const struct brcmf_vndr_dcmd_hdr *cmdhdr = data;
	struct sk_buff *reply;
	unsigned int payload, ret_len;
	void *dcmd_buf = NULL, *wr_pointer;
	u16 msglen, maxmsglen = PAGE_SIZE - 0x100;
	int ret;

	if (len < sizeof(*cmdhdr)) {
		brcmf_err("vendor command too short: %d\n", len);
		return -EINVAL;
	}

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	brcmf_dbg(TRACE, "ifidx=%d, cmd=%d\n", ifp->ifidx, cmdhdr->cmd);

	if (cmdhdr->offset > len) {
		brcmf_err("bad buffer offset %d > %d\n", cmdhdr->offset, len);
		return -EINVAL;
	}

	brcmf_android_wake_lock(ifp->drvr);

	len -= cmdhdr->offset;
	ret_len = cmdhdr->len;
	if (ret_len > 0 || len > 0) {
		if (len > BRCMF_DCMD_MAXLEN) {
			brcmf_err("oversize input buffer %d\n", len);
			len = BRCMF_DCMD_MAXLEN;
		}
		if (ret_len > BRCMF_DCMD_MAXLEN) {
			brcmf_err("oversize return buffer %d\n", ret_len);
			ret_len = BRCMF_DCMD_MAXLEN;
		}
		payload = max_t(unsigned int, ret_len, len) + 1;
		dcmd_buf = vzalloc(payload);
		if (!dcmd_buf) {
			brcmf_android_wake_unlock(ifp->drvr);
			return -ENOMEM;
		}

		memcpy(dcmd_buf, (void *)cmdhdr + cmdhdr->offset, len);
		*(char *)(dcmd_buf + len)  = '\0';
	}

	if (cmdhdr->set)
		ret = brcmf_fil_cmd_data_set(ifp, cmdhdr->cmd, dcmd_buf,
					     ret_len);
	else
		ret = brcmf_fil_cmd_data_get(ifp, cmdhdr->cmd, dcmd_buf,
					     ret_len);

	if (ret != 0) {
		brcmf_dbg(INFO, "error(%d), return -EPERM\n", ret);
		ret = -EPERM;
		goto exit;
	}

	wr_pointer = dcmd_buf;
	while (ret_len > 0) {
		msglen = ret_len > maxmsglen ? maxmsglen : ret_len;
		ret_len -= msglen;
		payload = msglen + sizeof(msglen);
		reply = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, payload);
		if (NULL == reply) {
			ret = -ENOMEM;
			break;
		}

		if (nla_put(reply, BRCMF_NLATTR_DATA, msglen, wr_pointer) ||
		    nla_put_u16(reply, BRCMF_NLATTR_LEN, msglen)) {
			kfree_skb(reply);
			ret = -ENOBUFS;
			break;
		}

		ret = cfg80211_vendor_cmd_reply(reply);
		if (ret)
			break;

		wr_pointer += msglen;
	}

exit:
	vfree(dcmd_buf);
	brcmf_android_wake_unlock(ifp->drvr);
	return ret;
}

static int
brcmf_cfg80211_gscan_get_channel_list_handler(struct wiphy *wiphy,
					      struct wireless_dev *wdev,
					      const void *data, int len)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct sk_buff *reply;
	int ret, gscan_band, i;
	struct ieee80211_supported_band *band_2g, *band_5g;
	uint *channels;
	uint num_channels = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	brcmf_android_wake_lock(ifp->drvr);

	brcmf_dbg(TRACE, "ifidx=%d, enter\n", ifp->ifidx);

	if (nla_type(data) == GSCAN_ATTRIBUTE_BAND) {
		gscan_band = nla_get_u32(data);
		if ((gscan_band & GSCAN_BAND_MASK) == 0) {
			ret = -EINVAL;
			goto exit;
		}
	} else {
		ret =  -EINVAL;
		goto exit;
	}

	band_2g = wiphy->bands[NL80211_BAND_2GHZ];
	band_5g = wiphy->bands[NL80211_BAND_5GHZ];
	channels = vzalloc((band_2g->n_channels + band_5g->n_channels) *
			   sizeof(uint));
	if (!channels) {
		ret = -ENOMEM;
		goto exit;
	}

	if (gscan_band & GSCAN_BG_BAND_MASK) {
		for (i = 0; i < band_2g->n_channels; i++) {
			if (band_2g->channels[i].flags &
			    IEEE80211_CHAN_DISABLED)
				continue;
			if (!(gscan_band & GSCAN_DFS_MASK) &&
			    (band_2g->channels[i].flags &
			     (IEEE80211_CHAN_RADAR | IEEE80211_CHAN_NO_IR)))
				continue;

			channels[num_channels] =
			    band_2g->channels[i].center_freq;
			num_channels++;
		}
	}
	if (gscan_band & GSCAN_A_BAND_MASK) {
		for (i = 0; i < band_5g->n_channels; i++) {
			if (band_5g->channels[i].flags &
			    IEEE80211_CHAN_DISABLED)
				continue;
			if (!(gscan_band & GSCAN_DFS_MASK) &&
			    (band_5g->channels[i].flags &
			     (IEEE80211_CHAN_RADAR | IEEE80211_CHAN_NO_IR)))
				continue;

			channels[num_channels] =
			    band_5g->channels[i].center_freq;
			num_channels++;
		}
	}

	reply =
	    cfg80211_vendor_cmd_alloc_reply_skb(wiphy, ((num_channels + 1) *
							sizeof(uint)));
	nla_put_u32(reply, GSCAN_ATTRIBUTE_NUM_CHANNELS, num_channels);
	nla_put(reply, GSCAN_ATTRIBUTE_CHANNEL_LIST,
		num_channels * sizeof(uint), channels);
	ret = cfg80211_vendor_cmd_reply(reply);

	vfree(channels);
exit:
	brcmf_android_wake_unlock(ifp->drvr);

	return ret;
}

static int
brcmf_cfg80211_andr_get_feature_set_handler(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    const void *data, int len)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct sk_buff *reply;
	int ret;
	int feature_set = 0;
	char caps[256];

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	brcmf_android_wake_lock(ifp->drvr);

	brcmf_dbg(TRACE, "ifidx=%d, enter\n", ifp->ifidx);

	ret = brcmf_fil_iovar_data_get(ifp, "cap", caps, sizeof(caps));
	if (ret) {
		brcmf_err("get capa error, ret = %d\n", ret);
		goto exit;
	}

	if (strnstr(caps, "sta", sizeof(caps)))
		feature_set |= WIFI_FEATURE_INFRA;
	if (strnstr(caps, "dualband", sizeof(caps)))
		feature_set |= WIFI_FEATURE_INFRA_5G;
	if (strnstr(caps, "p2p", sizeof(caps)))
		feature_set |= WIFI_FEATURE_P2P;
	if (wdev->iftype == NL80211_IFTYPE_AP ||
	    wdev->iftype == NL80211_IFTYPE_P2P_GO)
		feature_set |= WIFI_FEATURE_SOFT_AP;
	if (strnstr(caps, "tdls", sizeof(caps)))
		feature_set |= WIFI_FEATURE_TDLS;
	if (strnstr(caps, "vsdb", sizeof(caps)))
		feature_set |= WIFI_FEATURE_TDLS_OFFCHANNEL;
	if (strnstr(caps, "nan", sizeof(caps))) {
		feature_set |= WIFI_FEATURE_NAN;
		if (strnstr(caps, "rttd2d", sizeof(caps)))
			feature_set |= WIFI_FEATURE_D2D_RTT;
	}
	/* TODO:
	 * RTT_SUPPORT
	 * LINKSTAT_SUPPORT
	 * PNO_SUPPORT
	 * GSCAN_SUPPORT
	 * RSSI_MONITOR_SUPPORT
	 * WL11U
	 * NDO_CONFIG_SUPPORT
	 */
	reply = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(int));
	nla_put_nohdr(reply, sizeof(int), &feature_set);
	ret = cfg80211_vendor_cmd_reply(reply);
exit:
	brcmf_android_wake_unlock(ifp->drvr);

	return ret;
}

static int
brcmf_cfg80211_andr_set_country_handler(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct net_device *ndev;
	int ret;
	char *country_code;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	ndev = ifp->ndev;

	brcmf_android_wake_lock(ifp->drvr);

	brcmf_dbg(TRACE, "ifidx=%d, enter\n", ifp->ifidx);

	if (nla_type(data) == ANDR_WIFI_ATTRIBUTE_COUNTRY) {
		country_code = nla_data(data);
		brcmf_err("country=%s\n", country_code);
		if (strlen(country_code) != 2)
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	ret = brcmf_set_country(ndev, country_code);
	if (ret)
		brcmf_err("set country code %s failed, ret=%d\n",
			  country_code, ret);

	brcmf_android_wake_unlock(ifp->drvr);

	return ret;
}

s32
brcmf_wiphy_phy_temp_evt_handler(struct brcmf_if *ifp,
				 const struct brcmf_event_msg *e, void *data)

{
	struct brcmf_cfg80211_info *cfg = ifp->drvr->config;
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct sk_buff *skb;
	struct nlattr *phy_temp_data;
	u32 version, temp, tempdelta;
	struct brcmf_phy_temp_evt *phy_temp_evt;

	phy_temp_evt = (struct brcmf_phy_temp_evt *)data;

	version = le32_to_cpu(phy_temp_evt->version);
	temp = le32_to_cpu(phy_temp_evt->temp);
	tempdelta = le32_to_cpu(phy_temp_evt->tempdelta);

	skb = cfg80211_vendor_event_alloc(wiphy, NULL,
					  sizeof(*phy_temp_evt),
					  BRCMF_VNDR_EVTS_PHY_TEMP,
					  GFP_KERNEL);

	if (!skb) {
		brcmf_dbg(EVENT, "NO MEM: can't allocate skb for vendor PHY_TEMP_EVENT\n");
		return -ENOMEM;
	}

	phy_temp_data = nla_nest_start(skb, NL80211_ATTR_VENDOR_EVENTS);
	if (!phy_temp_data) {
		nla_nest_cancel(skb, phy_temp_data);
		kfree_skb(skb);
		brcmf_dbg(EVENT, "skb could not nest vendor attributes\n");
		return -EMSGSIZE;
	}

	if (nla_put_u32(skb, BRCMF_NLATTR_VERS, version) ||
	    nla_put_u32(skb, BRCMF_NLATTR_PHY_TEMP, temp) ||
	    nla_put_u32(skb, BRCMF_NLATTR_PHY_TEMPDELTA, tempdelta)) {
		kfree_skb(skb);
		brcmf_dbg(EVENT, "NO ROOM in skb for vendor PHY_TEMP_EVENT\n");
		return -EMSGSIZE;
	}

	nla_nest_end(skb, phy_temp_data);

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return 0;
}


const struct wiphy_vendor_command brcmf_vendor_cmds[] = {
	{
		{
			.vendor_id = BROADCOM_OUI,
			.subcmd = BRCMF_VNDR_CMDS_DCMD
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = brcmf_cfg80211_vndr_cmds_dcmd_handler
	},
	{
		{
			.vendor_id = GOOGLE_OUI,
			.subcmd = GSCAN_SUBCMD_GET_CHANNEL_LIST
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = brcmf_cfg80211_gscan_get_channel_list_handler
	},
	{
		{
			.vendor_id = GOOGLE_OUI,
			.subcmd = ANDR_WIFI_SET_COUNTRY
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = brcmf_cfg80211_andr_set_country_handler
	},
	{
		{
			.vendor_id = GOOGLE_OUI,
			.subcmd = ANDR_WIFI_SUBCMD_GET_FEATURE_SET
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = brcmf_cfg80211_andr_get_feature_set_handler
	},
};

const struct nl80211_vendor_cmd_info brcmf_vendor_events[] = {
	{
		.vendor_id = BROADCOM_OUI,
		.subcmd = BRCMF_VNDR_EVTS_PHY_TEMP,
	},
};

void brcmf_set_vndr_cmd(struct wiphy *wiphy)
{
	wiphy->vendor_commands = brcmf_vendor_cmds;
	wiphy->n_vendor_commands = ARRAY_SIZE(brcmf_vendor_cmds);
}
