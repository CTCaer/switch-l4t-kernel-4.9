/*
 * Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor
 * Corporation or one of its subsidiaries ("Cypress") and is protected by
 * and subject to worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA"). If no EULA applies, Cypress hereby grants
 * you a personal, nonexclusive, non-transferable license to copy, modify,
 * and compile the Software source code solely for use in connection with
 * Cypress's integrated circuit products. Any reproduction, modification,
 * translation, compilation, or representation of this Software except as
 * specified above is prohibited without the express written permission of
 * Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#include <linux/mmc/card.h>
#include <linux/wakelock.h>
#include <defs.h>
#include <brcmu_utils.h>
#include "core.h"
#include "android.h"
#include "cfg80211.h"
#include "debug.h"
#include "sdio.h"
#include "fwil.h"
#include "vendor.h"

#ifdef CPTCFG_BRCMFMAC_NV_PRIV_CMD
#include "nv_common.h"
#endif /* CPTCFG_BRCMFMAC_NV_PRIV_CMD */
#ifdef CPTCFG_NV_CUSTOM_SYSFS_TEGRA
#include "nv_custom_sysfs_tegra.h"
#endif /* CPTCFG_NV_CUSTOM_SYSFS_TEGRA */

#define CMD_START		"START"
#define CMD_STOP		"STOP"
#define CMD_SCAN_ACTIVE		"SCAN-ACTIVE"
#define CMD_SCAN_PASSIVE	"SCAN-PASSIVE"
#define CMD_RSSI		"RSSI"
#define CMD_LINKSPEED		"LINKSPEED"
#define CMD_RXFILTER_START	"RXFILTER-START"
#define CMD_RXFILTER_STOP	"RXFILTER-STOP"
#define CMD_RXFILTER_ADD	"RXFILTER-ADD"
#define CMD_RXFILTER_REMOVE	"RXFILTER-REMOVE"
#define CMD_BTCOEXSCAN_START	"BTCOEXSCAN-START"
#define CMD_BTCOEXSCAN_STOP	"BTCOEXSCAN-STOP"
#define CMD_BTCOEXMODE		"BTCOEXMODE"
#define CMD_SETSUSPENDOPT	"SETSUSPENDOPT"
#define CMD_SETSUSPENDMODE	"SETSUSPENDMODE"
#define CMD_MAXDTIM_IN_SUSPEND	"MAX_DTIM_IN_SUSPEND"
#define CMD_P2P_DEV_ADDR	"P2P_DEV_ADDR"
#define CMD_SETFWPATH		"SETFWPATH"
#define CMD_SETBAND		"SETBAND"
#define CMD_GETBAND		"GETBAND"
#define CMD_COUNTRY		"COUNTRY"
#define CMD_P2P_SET_NOA		"P2P_SET_NOA"
#define CMD_MIRACAST		"MIRACAST"

#ifdef CPTCFG_BRCMFMAC_NV_COUNTRY_CODE
#define CMD_NV_COUNTRY		"NV_COUNTRY"
#endif /* CPTCFG_BRCMFMAC_NV_COUNTRY_CODE */
#ifdef CPTCFG_BRCMFMAC_NV_PRIV_CMD
#define CMD_SET_IM_MODE		"SETMIRACAST"
#define CMD_UPDATE_CHANNEL_LIST "UPDATE_CHANNEL_LIST"
#define CMD_RESTRICT_BW_20      "RESTRICT_BW_20"
#define CMD_MAXLINKSPEED	"MAXLINKSPEED"
#define CMD_SETROAMMODE		"SETROAMMODE"
#define CMD_AUTOSLEEP		"AUTOSLEEP" /* only for SDIO based chip */
#define CMD_SETBTCPARAMS	"SETBTCPARAMS"
#define CMD_GETBTCPARAMS	"GETBTCPARAMS"
#define CMD_MKEEP_ALIVE		"MKEEP_ALIVE" /* TODO */
u32 restrict_bw_20;
bool builtin_roam_disabled;
#endif /* CPTCFG_BRCMFMAC_NV_PRIV_CMD */

#define DEFAULT_WIFI_TURNON_DELAY	200

/* miracast related definition */
#define MIRACAST_MODE_OFF		0
#define MIRACAST_MODE_SOURCE		1
#define MIRACAST_MODE_SINK		2

#define MIRACAST_AMPDU_SIZE		8

int brcmf_android_wifi_on(struct brcmf_pub *drvr, struct net_device *ndev)
{
	int ret = 0;
	struct brcmf_android *android = drvr->android;

	brcmf_dbg(ANDROID, "enter\n");

	if (!ndev) {
		brcmf_err("net device is null\n");
		return -EINVAL;
	}

	if (!android) {
		brcmf_err("not supported\n");
		return -EOPNOTSUPP;
	}

	if (!(android->wifi_on)) {
		ret = brcmf_set_power(true, DEFAULT_WIFI_TURNON_DELAY);
		if (ret) {
			brcmf_err("power up wifi chip failed, err=%d\n", ret);
			return ret;
		}
		android->wifi_on = true;
#ifdef CPTCFG_NV_CUSTOM_SYSFS_TEGRA
		tegra_sysfs_on();
#endif
	}

	return ret;
}

int
brcmf_android_wifi_off(struct brcmf_pub *drvr, struct net_device *ndev)
{
	struct brcmf_if *ifp =  netdev_priv(ndev);
	struct brcmf_android *android = drvr->android;
	int ret = 0;

	brcmf_dbg(ANDROID, "enter\n");

	if (!android) {
		brcmf_err("not supported\n");
		return -EOPNOTSUPP;
	}

	if (android->wifi_on) {
#ifdef CPTCFG_NV_CUSTOM_SYSFS_TEGRA
		tegra_sysfs_off();
#endif
		if (android->init_done)
			ret = brcmf_fil_cmd_int_set(ifp, BRCMF_C_DOWN, 1);
		brcmf_set_power(false, 0);
		android->wifi_on = false;
	}

	return ret;
}

static int brcmf_android_set_suspendmode(struct net_device *ndev,
					 char *command, int total_len)
{
	int ret = 0;

#if !defined(CONFIG_HAS_EARLYSUSPEND)
	int suspend_flag;

	brcmf_dbg(ANDROID, "enter\n");

	suspend_flag = *(command + strlen(CMD_SETSUSPENDMODE) + 1) - '0';
	if (suspend_flag != 0 && suspend_flag != 1)
		return -EINVAL;

	ret = brcmf_pktfilter_enable(ndev, (bool)suspend_flag);
	if (ret)
		brcmf_err("suspend failed\n");
#endif

	return ret;
}

static int brcmf_android_set_country(struct net_device *ndev, char *command,
				     int total_len)
{
	struct brcmf_if *ifp =  netdev_priv(ndev);
	struct brcmf_pub *drvr = ifp->drvr;
	struct brcmf_android *android = drvr->android;
#ifdef CPTCFG_BRCMFMAC_NV_COUNTRY_CODE
	char *country_code = command + strlen(CMD_NV_COUNTRY) + 1;
#else
	char *country_code = command + strlen(CMD_COUNTRY) + 1;
#endif /* CPTCFG_BRCMFMAC_NV_COUNTRY_CODE */

	int ret = 0;

	ret = brcmf_set_country(ndev, country_code);

	if (!ret)
		strncpy(android->country, country_code, 2);

	return ret;
}

static
int brcmf_android_set_btcoexmode(struct net_device *ndev, char *command,
				 int total_len)
{
	int ret = 0;
	int btcoex_mode = 0;

	btcoex_mode = *(command + strlen(CMD_BTCOEXMODE) + 1) - '0';

	if (btcoex_mode == 1) {
		ret = brcmf_crit_proto_start(ndev);
	} else if (btcoex_mode == 2) {
		ret = brcmf_crit_proto_stop(ndev);
	} else {
		brcmf_err("unknown btcode mode(%d)\n", btcoex_mode);
		ret = -EINVAL;
	}

	return ret;
}

static
int brcmf_android_pktfilter_add(struct net_device *ndev, char *command,
				int total_len)
{
	int ret = 0;
	int filter_num = *(command + strlen(CMD_RXFILTER_ADD) + 1) - '0';

	ret = brcmf_pktfilter_add_remove(ndev, filter_num, true);

	return ret;
}

static
int brcmf_android_pktfilter_remove(struct net_device *ndev, char *command,
				   int total_len)
{
	int ret = 0;
	int filter_num = *(command + strlen(CMD_RXFILTER_REMOVE) + 1) - '0';

	ret = brcmf_pktfilter_add_remove(ndev, filter_num, false);

	return ret;
}

static
int brcmf_android_set_miracast(struct net_device *ndev, char *command,
			       int total_len)
{
	struct brcmf_if *ifp =  netdev_priv(ndev);
	int miracast_mode = *(command + strlen(CMD_MIRACAST) + 1) - '0';
	static int miracast_off_ampdu_size;
	int ret = 0;

	brcmf_dbg(ANDROID, "set miracast mode %d\n", miracast_mode);

	//TODO: Do we need to set mchan_algo & mchan_bw?
	if (miracast_mode == MIRACAST_MODE_OFF) {
		ret = brcmf_fil_iovar_int_set(ifp, "ampdu_mpdu",
					      miracast_off_ampdu_size);
	} else if (miracast_mode == MIRACAST_MODE_SOURCE ||
			miracast_mode == MIRACAST_MODE_SINK) {
		ret = brcmf_fil_iovar_int_get(ifp, "ampdu_mpdu",
					      &miracast_off_ampdu_size);
		if (!ret)
			ret = brcmf_fil_iovar_int_set(ifp, "ampdu_mpdu",
						      MIRACAST_AMPDU_SIZE);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int
brcmf_handle_private_cmd(struct brcmf_pub *drvr, struct net_device *ndev,
			 char *command, u32 cmd_len)
{
	int bytes_written = 0;
	struct brcmf_android *android = drvr->android;
	struct brcmf_android_wifi_priv_cmd priv_cmd;
#ifdef CPTCFG_BRCMFMAC_NV_PRIV_CMD
	int val;
#endif /* CPTCFG_BRCMFMAC_NV_PRIV_CMD */

	brcmf_dbg(ANDROID, "enter\n");
	brcmf_err("command = %s received\n", command);
	if (!android) {
		brcmf_err("not supported\n");
		return -EOPNOTSUPP;
	}

	if (!(android->wifi_on)) {
		brcmf_err("ignore cmd \"%s\" - iface is down\n", command);
		return 0;
	}

	memset(&priv_cmd, 0, sizeof(struct brcmf_android_wifi_priv_cmd));
	priv_cmd.total_len = cmd_len;

	if (strncmp(command, CMD_SETSUSPENDMODE,
		    strlen(CMD_SETSUSPENDMODE)) == 0) {
		bytes_written =
		    brcmf_android_set_suspendmode(ndev, command,
						  priv_cmd.total_len);
	} else if (strncmp(command, CMD_COUNTRY, strlen(CMD_COUNTRY)) == 0) {
#ifndef CPTCFG_BRCMFMAC_NV_COUNTRY_CODE
		bytes_written =
		    brcmf_android_set_country(ndev, command,
					      priv_cmd.total_len);
#else
		bytes_written = 0;
	} else if (strncmp(command, CMD_NV_COUNTRY, strlen(CMD_NV_COUNTRY)) == 0) {
		bytes_written =
			brcmf_android_set_country(ndev, command,
						priv_cmd.total_len);
#endif /* CPTCFG_BRCMFMAC_NV_COUNTRY_CODE */
	} else if (strncmp(command, CMD_BTCOEXMODE,
		   strlen(CMD_BTCOEXMODE)) == 0) {
		bytes_written =
		    brcmf_android_set_btcoexmode(ndev, command,
						 priv_cmd.total_len);
	} else if (strncmp(command, CMD_RXFILTER_START,
		   strlen(CMD_RXFILTER_START)) == 0) {
		bytes_written =
		    brcmf_pktfilter_enable(ndev, true);
	} else if (strncmp(command, CMD_RXFILTER_STOP,
		   strlen(CMD_RXFILTER_STOP)) == 0) {
		bytes_written =
		    brcmf_pktfilter_enable(ndev, false);
	} else if (strncmp(command, CMD_RXFILTER_ADD,
		   strlen(CMD_RXFILTER_ADD)) == 0) {
		bytes_written =
		    brcmf_android_pktfilter_add(ndev, command,
						priv_cmd.total_len);
	} else if (strncmp(command, CMD_RXFILTER_REMOVE,
		   strlen(CMD_RXFILTER_REMOVE)) == 0) {
		bytes_written =
		    brcmf_android_pktfilter_remove(ndev, command,
						   priv_cmd.total_len);
	} else if (strncmp(command, CMD_MIRACAST,
		   strlen(CMD_MIRACAST)) == 0) {
		bytes_written =
		    brcmf_android_set_miracast(ndev, command,
					       priv_cmd.total_len);
	} else if (strncmp(command, CMD_BTCOEXSCAN_START,
		   strlen(CMD_BTCOEXSCAN_START)) == 0) {
		//TODO: Handle BTCOEXSCAN_START command
	} else if (strncmp(command, CMD_BTCOEXSCAN_STOP,
		   strlen(CMD_BTCOEXSCAN_STOP)) == 0) {
		//TODO: Handle BTCOEXSCAN_STOP command
#ifdef CPTCFG_BRCMFMAC_NV_PRIV_CMD
	} else if (strncmp(command, CMD_SET_IM_MODE,
			strlen(CMD_SET_IM_MODE)) == 0) {
		bytes_written =
			nv_brcmf_android_set_im_mode(drvr, ndev, command,
						priv_cmd.total_len);
	} else if (strncmp(command, CMD_UPDATE_CHANNEL_LIST,
			strlen(CMD_UPDATE_CHANNEL_LIST)) == 0) {
		//brcmf_setup_wiphybands
	} else if (strncmp(command, CMD_RESTRICT_BW_20, strlen(CMD_GETBAND)) == 0) {
		bytes_written = -1;
		val = *(command + strlen(CMD_RESTRICT_BW_20) + 1) - '0';
		if (val == 0 || val == 1) {
			restrict_bw_20 = val;
			bytes_written = 0;
		}
	} else if (strncmp(command, CMD_MAXLINKSPEED, strlen(CMD_MAXLINKSPEED))== 0) {
		bytes_written = brcmf_get_max_linkspeed(ndev, command, priv_cmd.total_len);
	} else if (strncmp(command, CMD_SETBAND, strlen(CMD_SETBAND)) == 0) {
		uint band = *(command + strlen(CMD_SETBAND) + 1) - '0';
		bytes_written = brcmf_set_band(ndev, band);
	} else if (!builtin_roam_disabled && strncmp(command, CMD_SETROAMMODE, strlen(CMD_SETROAMMODE)) == 0) {
		 bytes_written = nv_set_roam_mode(ndev, command, priv_cmd.total_len);
	} else if (strncmp(command, CMD_SETBTCPARAMS, strlen(CMD_SETBTCPARAMS)) == 0) {
		bytes_written = nv_btcoex_set_btcparams(ndev, command, priv_cmd.total_len);
	} else if (strncmp(command, CMD_GETBTCPARAMS, strlen(CMD_GETBTCPARAMS)) == 0) {
		bytes_written = nv_btcoex_get_btcparams(ndev, command, priv_cmd.total_len);
	} else if (strncmp(command, CMD_MKEEP_ALIVE,
		strlen(CMD_MKEEP_ALIVE)) == 0) {
		brcmf_err("CMD_MKEEP_ALIVE\n");
		bytes_written = nv_android_mkeep_alive(ndev, command, priv_cmd.total_len);
#endif /* CPTCFG_BRCMFMAC_NV_PRIV_CMD */
	} else {
		brcmf_err("unknown PRIVATE command %s - ignored\n", command);
		snprintf(command, 5, "FAIL");
		bytes_written = strlen("FAIL");
	}

	return bytes_written;
}

int brcmf_android_attach(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = NULL;

	if (brcmf_android_is_attached(drvr))
		return 0;

	brcmf_dbg(TRACE, "enter\n");

	android = kzalloc(sizeof(*android), GFP_KERNEL);
	if (!android)
		return -ENOMEM;

	drvr->android = android;
	android->drvr = drvr;
	android->wifi_on = true;
#ifdef CPTCFG_NV_CUSTOM_SYSFS_TEGRA
	tegra_sysfs_on();
#endif
	android->wifi_reset = false;
	android->init_done = false;
	android->wakelock_counter = 0;
	android->wakelock_waive = false;
	android->wakelock_waive_counter = 0;
	android->country[0] = 0;
	android->country[1] = 0;
	android->country[2] = 0;
	wake_lock_init(&android->wakelock, WAKE_LOCK_SUSPEND,
		       "brcm_wlan_wake");
	wake_lock_init(&android->rx_wakelock, WAKE_LOCK_SUSPEND,
		       "brcm_wlan_rxwake");
	spin_lock_init(&android->wakelock_spinlock);

	return 0;
}

int brcmf_android_detach(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = drvr->android;

	brcmf_dbg(TRACE, "enter\n");

	wake_lock_destroy(&android->wakelock);
	wake_lock_destroy(&android->rx_wakelock);
	kfree(drvr->android);
	drvr->android = NULL;

	return 0;
}

bool brcmf_android_is_attached(struct brcmf_pub *drvr)
{
	return !!(drvr->android);
}

bool brcmf_android_is_built_in(struct brcmf_pub *drvr)
{
#ifdef BRCMFMAC_ANDROID_BUILT_IN
	return brcmf_android_is_attached(drvr);
#else
	return false;
#endif
}

bool brcmf_android_wifi_is_on(struct brcmf_pub *drvr)
{
	if (!brcmf_android_is_attached(drvr))
		return false;

	return drvr->android->wifi_on;
}

bool brcmf_android_in_reset(struct brcmf_pub *drvr)
{
	if (!brcmf_android_is_attached(drvr))
		return false;

	return drvr->android->wifi_reset;
}

void brcmf_android_set_reset(struct brcmf_pub *drvr, bool is_reset)
{
	brcmf_dbg(TRACE, "enter\n");

	if (!brcmf_android_is_attached(drvr))
		return;

	drvr->android->wifi_reset = is_reset;
}

int brcmf_android_wake_unlock_timeout(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = drvr->android;
	unsigned long flags;
	unsigned int ret = 0;

	brcmf_dbg(TRACE, "enter\n");

	if (!brcmf_android_is_attached(drvr))
		return -EOPNOTSUPP;

	spin_lock_irqsave(&android->wakelock_spinlock, flags);
	wake_lock_timeout(&android->rx_wakelock, msecs_to_jiffies(200));
	spin_unlock_irqrestore(&android->wakelock_spinlock, flags);

	ret = brcmf_android_wake_unlock(drvr);

	return ret;
}

int brcmf_android_wake_lock(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = drvr->android;
	unsigned long flags;
	unsigned int ret = 0;

	brcmf_dbg(TRACE, "enter\n");

	if (!brcmf_android_is_attached(drvr))
		return -EOPNOTSUPP;

	if (brcmf_android_wake_lock_is_waive(drvr)) {
		spin_lock_irqsave(&android->wakelock_spinlock, flags);
		android->wakelock_waive_counter++;
		spin_unlock_irqrestore(&android->wakelock_spinlock, flags);
	} else {
		spin_lock_irqsave(&android->wakelock_spinlock, flags);

		if (android->wakelock_counter == 0)
			wake_lock(&android->wakelock);

		android->wakelock_counter++;
		ret = android->wakelock_counter;

		spin_unlock_irqrestore(&android->wakelock_spinlock, flags);
	}

	return ret;
}

int brcmf_android_wake_unlock(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = drvr->android;
	unsigned long flags;
	unsigned int ret = 0;

	brcmf_dbg(TRACE, "enter\n");

	if (!brcmf_android_is_attached(drvr))
		return -EOPNOTSUPP;

	spin_lock_irqsave(&android->wakelock_spinlock, flags);

	if (android->wakelock_waive_counter > 0) {
		android->wakelock_waive_counter--;
	} else {
		if (android->wakelock_counter > 0)
			android->wakelock_counter--;

		if (android->wakelock_counter == 0)
			wake_unlock(&android->wakelock);

		ret = android->wakelock_counter;
	}

	spin_unlock_irqrestore(&android->wakelock_spinlock, flags);

	return ret;
}

void brcmf_android_wake_lock_waive(struct brcmf_pub *drvr, bool is_waive)
{
	struct brcmf_android *android = drvr->android;
	unsigned long flags;

	brcmf_dbg(TRACE, "enter\n");

	if (!brcmf_android_is_attached(drvr))
		return;

	spin_lock_irqsave(&android->wakelock_spinlock, flags);
	android->wakelock_waive = is_waive;
	spin_unlock_irqrestore(&android->wakelock_spinlock, flags);
}

bool brcmf_android_wake_lock_is_waive(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = drvr->android;
	unsigned long flags;

	bool is_waive = false;

	if (!brcmf_android_is_attached(drvr))
		return false;

	spin_lock_irqsave(&android->wakelock_spinlock, flags);
	is_waive = android->wakelock_waive;
	spin_unlock_irqrestore(&android->wakelock_spinlock, flags);

	return is_waive;
}

int brcmf_android_init(struct brcmf_pub *drvr)
{
	int err = 0;

#ifdef CPTCFG_BRCM_INSMOD_NO_FW
	err = brcmf_cfg80211_register_if(drvr);
	if (err) {
		brcmf_err("init failed, err=%d\n", err);
		return err;
	}

	brcmf_android_set_reset(drvr, true);
	brcmf_android_wifi_off(drvr, NULL);
	g_drvr = drvr;
#endif
	return err;
}

int brcmf_android_reset_country(struct brcmf_pub *drvr)
{
	struct brcmf_android *android = drvr->android;
	char *country = NULL;
	int ret = 0;

	if (!brcmf_android_is_attached(drvr))
		return false;

	country = android->country;

	if (country[0] && country[1])
		ret = brcmf_set_country(drvr->iflist[0]->ndev, country);

	return ret;
}

int brcmf_android_set_extra_wiphy(struct wiphy *wiphy, struct brcmf_if *ifp)
{
	struct cfg80211_wowlan *brcmf_wowlan_config = NULL;

	brcmf_dbg(TRACE, "enter\n");

	brcmf_set_vndr_cmd(wiphy);

	if (!wiphy->wowlan_config) {
		brcmf_wowlan_config = kzalloc(sizeof(*brcmf_wowlan_config),
				GFP_KERNEL);
		if (brcmf_wowlan_config) {
			brcmf_wowlan_config->disconnect = true;
			brcmf_wowlan_config->gtk_rekey_failure = true;
			brcmf_wowlan_config->eap_identity_req = true;
			brcmf_wowlan_config->four_way_handshake = true;
			brcmf_wowlan_config->patterns = NULL;
			brcmf_wowlan_config->n_patterns = 0;
			brcmf_wowlan_config->tcp = NULL;
		} else {
			brcmf_err("Can not allocate memory for brcmf_wowlan_config\n");
			return -ENOMEM;
		}
		wiphy->wowlan_config = brcmf_wowlan_config;
	}

	return 0;
}
