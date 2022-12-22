/*
 * Copyright (c) 2019, Ezekiel Bethel <zek@9net.org>
 * Copyright (c) 2021-2022, CTCaer <ctcaer@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/pmc.h>

#define TEGRA_SIP_R2P_COPY_TO_IRAM 0xC2FFFE02u
#define  R2P_WRITE_IRAM            (0U << 0)
#define  R2P_READ_IRAM             (1U << 0)
#define TEGRA_SIP_R2P_DO_REBOOT    0xC2FFFE03u
#define TEGRA_SIP_R2P_SET_BIN_CFG  0xC2FFFE04u

#define IRAM_PAYLOAD_BASE    0x40010000u
#define IRAM_CHUNK_SIZE      0x4000

#define RTC_REBOOT_REASON_MAGIC 0x77

/* hekate/Nyx */
#define BOOT_CFG_AUTOBOOT_EN BIT(0)
#define BOOT_CFG_FROM_ID     BIT(2)

#define EXTRA_CFG_NYX_UMS    BIT(5)
#define EXTRA_CFG_NYX_RELOAD BIT(6)

enum {
	NYX_UMS_SD_CARD = 0,
	NYX_UMS_EMMC_BOOT0,
	NYX_UMS_EMMC_BOOT1,
	NYX_UMS_EMMC_GPP,
	NYX_UMS_EMUMMC_BOOT0,
	NYX_UMS_EMUMMC_BOOT1,
	NYX_UMS_EMUMMC_GPP
};

enum {
	REBOOT_REASON_NOP   = 0, /* Use [config]. */
	REBOOT_REASON_SELF  = 1, /* Use autoboot_idx/autoboot_list. */
	REBOOT_REASON_MENU  = 2, /* Force menu. */
	REBOOT_REASON_UMS   = 3, /* Force selected UMS partition. */
	REBOOT_REASON_REC   = 4, /* Set PMC_SCRATCH0_MODE_RECOVERY and reboot to self. */
	REBOOT_REASON_PANIC = 5  /* Inform bootloader that panic occured if T210B01. */
};

struct __attribute__((__packed__)) hekate_boot_cfg
{
	u8 boot_cfg;
	u8 autoboot;
	u8 autoboot_list;
	u8 extra_cfg;
	union {
		struct {
			char id[8];
			char emummc_path[0x78];
		};
		u8 ums;
		u8 xt_str[0x80];
	};
};

struct rtc_rr_dec
{
	u16 reason:4;
	u16 autoboot_idx:4;
	u16 autoboot_list:1;
	u16 ums_idx:3;
};

struct rtc_rr_enc
{
	u16 val1:6; /* 6-bit reg. */
	u16 val2:6; /* 6-bit reg. */
};

struct rtc_reboot_reason
{
	union {
		struct rtc_rr_dec dec;
		struct rtc_rr_enc enc;
	};
};

struct reboot_driver_state
{
	char action[16];
	int  param1;
	int  param2;
	char entry_id[8];
};

static int enabled = 0;
static int param1 = 0;
static int param2 = 0;
static char* action = NULL;
static char* entry_id = NULL;

module_param(enabled, int, 0664);
module_param(action, charp, 0664);
module_param(param1, int, 0664);
module_param(param2, int, 0664);
module_param(entry_id, charp, 0664);

MODULE_PARM_DESC(enabled, "Enable Reboot 2 Payload function");
MODULE_PARM_DESC(action, "Reboot action to take");
MODULE_PARM_DESC(param1, "Autoboot entry or UMS device index");
MODULE_PARM_DESC(param2, "Autooboot entry is ini list");
MODULE_PARM_DESC(entry_id, "Autoboot entry id");

struct platform_device *r2p_device = NULL;

static u32 tegra_pmc_r2p_set_cfg(struct hekate_boot_cfg *hekate_bcfg)
{
	struct pmc_smc_regs regs;
	u64 hekate_boot_smc_cfg[4];

	memcpy(hekate_boot_smc_cfg, hekate_bcfg, sizeof(hekate_boot_smc_cfg));

	regs.args[0] = hekate_boot_smc_cfg[0];
	regs.args[1] = hekate_boot_smc_cfg[1];
	regs.args[2] = hekate_boot_smc_cfg[2];
	regs.args[3] = hekate_boot_smc_cfg[3];
	regs.args[4] = 0;
	regs.args[5] = 0;
	pmc_send_smc(TEGRA_SIP_R2P_SET_BIN_CFG, &regs);
	return (u32)regs.args[0];
}

void tegra_pmc_r2p_setup(const char *cmd, bool panic_occurred)
{
	struct rtc_reboot_reason rr = {};
	struct hekate_boot_cfg hekate_bcfg = {};
	struct reboot_driver_state *state;

	uint32_t reboot_reason = REBOOT_REASON_NOP;
	uint32_t hid, chipid, major;
	bool tegra210b01;

	if (r2p_device == NULL)
		return;

	hid = tegra_read_chipid();
	chipid = tegra_hidrev_get_chipid(hid);
	major = tegra_hidrev_get_majorrev(hid);
	tegra210b01 = chipid == TEGRA210B01 && major >= 2;

	state = dev_get_drvdata(&r2p_device->dev);

	/* Linux reboot reason */
	if (cmd) {
		if (!strcmp(cmd, "recovery")) {
			reboot_reason = REBOOT_REASON_REC;
		} else if (!strcmp(cmd, "bootloader")) {
			reboot_reason = REBOOT_REASON_MENU;
		} else if (!strcmp(cmd, "forced-recovery")) { /* RCM */
			if (tegra210b01) return;
		} else {
			dev_err(&r2p_device->dev,
				"Command '%s' not recognized\n", cmd);
		}
	}

	/* Missing reboot reason or not matching, use r2p action. */
	if (reboot_reason == REBOOT_REASON_NOP && strlen(state->action)) {
		if (!strcmp(state->action, "self") ||
		    !strcmp(state->action, "via-payload")) { /* Deprecated */
			reboot_reason = REBOOT_REASON_SELF;
		} else if (!strcmp(state->action, "bootloader")) {
			reboot_reason = REBOOT_REASON_MENU;
		} else if (!strcmp(state->action, "ums")) {
			reboot_reason = REBOOT_REASON_UMS;
		} else if (!strcmp(state->action, "normal")) {
			reboot_reason = REBOOT_REASON_NOP;
		} else {
			dev_err(&r2p_device->dev,
				"Action '%s' not recognized\n", state->action);
		}
	}

	/* If T210B01 and panic happened override reboot reason */
	if (tegra210b01 && panic_occurred) {
		reboot_reason = REBOOT_REASON_PANIC;
	}

	/* Prepare boot config data */
	switch (reboot_reason) {
	case REBOOT_REASON_NOP:
		if (tegra210b01) return;
		break;
	case REBOOT_REASON_REC:
	case REBOOT_REASON_SELF:
		hekate_bcfg.boot_cfg = BOOT_CFG_AUTOBOOT_EN;
		if (strlen(state->entry_id) != 0) {
			hekate_bcfg.boot_cfg |= BOOT_CFG_FROM_ID;
			strcpy(hekate_bcfg.id, state->entry_id);
		}
		hekate_bcfg.autoboot = state->param1;
		hekate_bcfg.autoboot_list = state->param2;

		rr.dec.reason = reboot_reason;
		rr.dec.autoboot_idx = state->param1;
		rr.dec.autoboot_list = state->param2;
		break;
	case REBOOT_REASON_MENU:
		hekate_bcfg.boot_cfg = BOOT_CFG_AUTOBOOT_EN;

		rr.dec.reason = reboot_reason;
		break;
	case REBOOT_REASON_UMS:
		hekate_bcfg.boot_cfg = BOOT_CFG_AUTOBOOT_EN;
		hekate_bcfg.extra_cfg = EXTRA_CFG_NYX_UMS;
		hekate_bcfg.ums = state->param1;

		rr.dec.reason = reboot_reason;
		rr.dec.ums_idx = state->param1;
		break;
	case REBOOT_REASON_PANIC:
		rr.dec.reason = reboot_reason;
		break;
	}

	/* Notify TZ to use its preloaded payload */
	if (!tegra210b01) {
		/* Set hekate boot config. */
		tegra_pmc_r2p_set_cfg(&hekate_bcfg);
	} else {
		struct tegra_br_cmd_cfg bcfg[4] = {
			{ 0, 0, rr.enc.val1 },
			{ 0, 1, rr.enc.val2 },
			{ 0, 2, RTC_REBOOT_REASON_MAGIC },
			{ 0, 3, RTC_REBOOT_REASON_MAGIC }
		};
		tegra_pmc_edit_bootrom_scratch_reset(bcfg, 4);
	}
}

static ssize_t action_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);
	int res;

	if (strlen(state->action))
		res = sprintf(buf, "%s\n", state->action);
	else
		res = sprintf(buf, "Not-set\n");

	return res;
}

static ssize_t action_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);

	if (count > sizeof(state->action) - 1)
		return -EINVAL;

	strncpy(state->action, buf, sizeof(state->action) - 1);
	state->action[sizeof(state->action) - 1] = 0;

	/* Strip newline if it exists */
	if (state->action[count - 1] == '\n')
		state->action[count - 1] = 0;

	return count;
}

static ssize_t entry_id_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);
	int res;

	if (strlen(state->entry_id))
		res = sprintf(buf, "%s\n", state->entry_id);
	else
		res = sprintf(buf, "Not-set\n");

	return res;
}

static ssize_t entry_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);

	if (count > sizeof(state->entry_id) - 1)
		return -EINVAL;

	strncpy(state->entry_id, buf, sizeof(state->entry_id) - 1);
	state->entry_id[sizeof(state->entry_id) - 1] = 0;

	/* Strip newline if it exists */
	if (state->entry_id[count - 1] == '\n')
		state->entry_id[count - 1] = 0;

	return count;
}

static ssize_t param1_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);

	return sprintf(buf, "%d\n", (int)state->param1);
}

static ssize_t param1_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);
	ssize_t res;
	long value;

	res = kstrtol(buf, 0, &value);
	if (!res)
		state->param1 = value;

	return res ? res : count;
}

static ssize_t param2_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);

	return sprintf(buf, "%d\n", (int)state->param2);
}

static ssize_t param2_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct reboot_driver_state *state = dev_get_drvdata(&r2p_device->dev);
	ssize_t res;
	long value;

	res = kstrtol(buf, 0, &value);
	if (!res)
		state->param2 = value;

	return res ? res : count;
}

static DEVICE_ATTR_RW(action);
static DEVICE_ATTR_RW(entry_id);
static DEVICE_ATTR_RW(param1);
static DEVICE_ATTR_RW(param2);

static struct attribute *r2p_sysfs_attrs[] = {
	&dev_attr_action.attr,
	&dev_attr_entry_id.attr,
	&dev_attr_param1.attr,
	&dev_attr_param2.attr,
	NULL,
};
ATTRIBUTE_GROUPS(r2p_sysfs);

static int tegra_pmc_r2p_driver_probe(struct platform_device *pdev)
{
	struct reboot_driver_state *state;

	if (!enabled)
		return 0;

	r2p_device = pdev;

	state = devm_kzalloc(&pdev->dev, sizeof(struct reboot_driver_state), GFP_KERNEL);

	/* Copy reboot action if valid */
	if (action) {
		strncpy(state->action, action, sizeof(state->action) - 1);
		state->action[sizeof(state->action) - 1] = 0;
	}

	/* Copy entry ID if valid */
	if (entry_id) {
		strncpy(state->entry_id, entry_id, sizeof(state->entry_id) - 1);
		state->entry_id[sizeof(state->entry_id) - 1] = 0;
	}

	/* Set reboot reason parameters. */
	state->param1 = param1;
	state->param2 = param2;

	dev_set_drvdata(&pdev->dev, state);

	if (sysfs_create_groups(&pdev->dev.kobj, r2p_sysfs_groups))
		dev_err(&pdev->dev, "sysfs creation failed\n");

	return 0;
}

static const struct of_device_id tegra_pmc_r2p_match[] = {
	{ .compatible = "tegra-r2p", },
	{ }
};

static struct platform_driver tegra_pmc_r2p_driver = {
	.probe   = tegra_pmc_r2p_driver_probe,
	.driver  = {
		.name  = "tegra-r2p",
		.owner = THIS_MODULE,
		.of_match_table = tegra_pmc_r2p_match
	},
};

builtin_platform_driver(tegra_pmc_r2p_driver);
