/*
 * Copyright (c) 2019, Ezekiel Bethel <zek@9net.org>
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
#include <soc/tegra/pmc.h>

#define TEGRA_SIP_R2P_COPY_TO_IRAM 0xC2FFFE02
#define  R2P_WRITE_IRAM            (0U << 0)
#define  R2P_READ_IRAM             (1U << 0)
#define TEGRA_SIP_R2P_DO_REBOOT    0xC2FFFE03

#define IRAM_PAYLOAD_BASE    0x40010000u
#define IRAM_CHUNK_SIZE      0x4000

/* hekate/Nyx */
#define BOOT_CFG_AUTOBOOT_EN BIT(0)
#define BOOT_CFG_FROM_LAUNCH BIT(1)
#define BOOT_CFG_FROM_ID     BIT(2)
#define BOOT_CFG_TO_EMUMMC   BIT(3)

#define EXTRA_CFG_KEYS       BIT(0)
#define EXTRA_CFG_PAYLOAD    BIT(1)
#define EXTRA_CFG_MODULE     BIT(2)

#define EXTRA_CFG_NYX_RELOAD BIT(6)
#define EXTRA_CFG_NYX_DUMP   BIT(7)

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
		u8 xt_str[0x80];
	};
};

static int enabled = 0;
static char* reboot_action = NULL;
static char* default_payload = NULL;
static char* hekate_config_id = NULL;

module_param(enabled, int, 0660);
module_param(reboot_action, charp, 0660);
module_param(default_payload, charp, 0660);
module_param(hekate_config_id, charp, 0660);

struct reboot_driver_state
{
	const char *reboot_action;
	const char *default_reboot_payload_name;
	char hekate_id[8];
	char default_payload_storage[192 * 1024];
	size_t default_payload_length;
	char custom_payload_storage[192 * 1024];
	size_t custom_payload_length;
};

struct platform_device *r2p_device = NULL;

u8 iram_copy_buf[IRAM_CHUNK_SIZE];

static u32 r2p_iram_copy(void *dram_addr, uint64_t iram_addr, uint32_t size, uint32_t flag)
{
	struct pmc_smc_regs regs;
	regs.args[0] = virt_to_phys(dram_addr);
	regs.args[1] = iram_addr;
	regs.args[2] = size;
	regs.args[3] = flag;
	regs.args[4] = 0;
	regs.args[5] = 0;
	pmc_send_smc(TEGRA_SIP_R2P_COPY_TO_IRAM, &regs);
	return (u32)regs.args[0];
}

static void copy_payload_to_iram(const char *payload, size_t size)
{
	size_t i;
	size_t copy_size;
	size_t size_remaining = size;

	for (i = 0; i < size; i += IRAM_CHUNK_SIZE, size_remaining -= IRAM_CHUNK_SIZE)
	{
		copy_size = size_remaining > IRAM_CHUNK_SIZE ? IRAM_CHUNK_SIZE : size_remaining;
		memcpy(iram_copy_buf, payload + i, copy_size);
		r2p_iram_copy(iram_copy_buf, IRAM_PAYLOAD_BASE + i, IRAM_CHUNK_SIZE, R2P_WRITE_IRAM);
	}
}

static int load_payload(struct device *dev, const char *payload_fw_name, bool custom)
{
	struct reboot_driver_state *state = dev_get_drvdata(dev);
	const struct firmware *reboot_payload_fw;
	int ret;


	ret = request_firmware(&reboot_payload_fw, payload_fw_name, dev);
	if (ret != 0) {
		dev_err(dev, "requesting firmware failed for %s: %d\n", payload_fw_name, ret);
		return ret;
	}

	if (custom) {
		memcpy(state->custom_payload_storage, reboot_payload_fw->data,
		       reboot_payload_fw->size);
		state->custom_payload_length = reboot_payload_fw->size;
	} else {
		memcpy(state->default_payload_storage, reboot_payload_fw->data,
		       reboot_payload_fw->size);
		state->default_payload_length = reboot_payload_fw->size;
	}

	release_firmware(reboot_payload_fw);
	return 0;
}

void r2p_setup(const char *cmd)
{
	uint32_t empty_payload = 0;
	bool load_default_payload = true;
	bool do_hekate_config = true;
	struct reboot_driver_state *state;
	struct hekate_boot_cfg hekate_config;

	if (r2p_device == NULL)
		return;

	state = dev_get_drvdata(&r2p_device->dev);
	memset(&hekate_config, 0, sizeof(struct hekate_boot_cfg));

	if (cmd) {
		if (strcmp(cmd, "payload") == 0) { /* Custom payload */
			/* If custom payload is present boot that */
			if (state->custom_payload_length != 0)
				load_default_payload = false;
			do_hekate_config = false;
		} else if (strcmp(cmd, "recovery") == 0) { /* Recovery mode */
			load_default_payload = true;
			do_hekate_config = true;
		} else if (strcmp(cmd, "bootloader") == 0) { /* Default payload */
			load_default_payload = true;
			do_hekate_config = false;
		}
	} else if (state->reboot_action) { /* Normal reboot or string not matching */
		if (strcmp(state->reboot_action, "via-payload") == 0) {
			load_default_payload = true;
			do_hekate_config = true;
		} else if (strcmp(state->reboot_action, "bootloader") == 0) {
			load_default_payload = true;
			do_hekate_config = false;
		}
	} else {
		/* Notify TZ to use its preloaded payload */
		r2p_iram_copy(&empty_payload, IRAM_PAYLOAD_BASE, sizeof(uint32_t), R2P_WRITE_IRAM);
		return;
	}

	if (load_default_payload) {
		// Write default payload.
		copy_payload_to_iram(state->default_payload_storage,
				     state->default_payload_length);

		if (do_hekate_config) {
			if (strlen(state->hekate_id) != 0) {
				hekate_config.boot_cfg = BOOT_CFG_FROM_ID | BOOT_CFG_AUTOBOOT_EN;
				memcpy(hekate_config.id, state->hekate_id, 8);

				// Write Hekate config.
				r2p_iram_copy(&hekate_config, IRAM_PAYLOAD_BASE + 0x94,
					      sizeof(struct hekate_boot_cfg), R2P_WRITE_IRAM);
			}
		}
	} else {
		copy_payload_to_iram(state->custom_payload_storage,
				     state->custom_payload_length);
	}

	return;
}

static ssize_t default_payload_ready_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct reboot_driver_state *state = dev_get_drvdata(dev);

	if (load_payload(dev, state->default_reboot_payload_name, false) != 0)
		return -EINVAL;

	return count;
}

static ssize_t custom_payload_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char payload_name[128];

	if (count > sizeof(payload_name)-1)
		return -EINVAL;

	strncpy(payload_name, buf, 127);
	payload_name[count] = 0;

	/* Strip newline if it exists */
	if (payload_name[count-1] == '\n')
		payload_name[count-1] = 0;

	if (load_payload(dev, payload_name, true) != 0)
		return -EINVAL;

	return count;
}

static DEVICE_ATTR_WO(default_payload_ready);
static DEVICE_ATTR_WO(custom_payload);

static struct attribute *reboot_sysfs_attrs[] = {
	&dev_attr_default_payload_ready.attr,
	&dev_attr_custom_payload.attr,
	NULL,
};
ATTRIBUTE_GROUPS(reboot_sysfs);

static int reboot_to_payload_driver_probe(struct platform_device *pdev)
{
	struct reboot_driver_state *state;

	if (!enabled)
		return 0;

	r2p_device = pdev;

	state = devm_kzalloc(&pdev->dev, sizeof(struct reboot_driver_state), GFP_KERNEL);

	state->default_reboot_payload_name = default_payload;
	state->reboot_action = reboot_action;

	/* Copy 7 char hekate id */
	strncpy(state->hekate_id, hekate_config_id, 7);
	state->hekate_id[7] = 0;

	dev_set_drvdata(&pdev->dev, state);

	if (sysfs_create_groups(&pdev->dev.kobj, reboot_sysfs_groups))
		dev_err(&pdev->dev, "sysfs creation failed?\n");

	return 0;
}

static const struct of_device_id tegra_reboot_to_payload_match[] = {
	{ .compatible = "tegra-reboot2payload", },
	{ }
};

static struct platform_driver tegra_reboot_to_payload_driver = {
	.probe   = reboot_to_payload_driver_probe,
	.driver  = {
		.name  = "tegra-reboot2payload",
		.owner = THIS_MODULE,
		.of_match_table = tegra_reboot_to_payload_match
	},
};

builtin_platform_driver(tegra_reboot_to_payload_driver);
