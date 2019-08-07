#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>

#define NR_SMC_REGS		6

struct pmc_smc_regs {
	   u64 args[NR_SMC_REGS];
};

#define ATMOSPHERE_COPY_TO_IRAM_COMMAND_ID	0xC2FFFE02
#define ATMOSPHERE_REBOOT_CONFIG_COMMAND_ID	 0xC2FFFE03

#define IRAM_CHUNK_SIZE 0x4000

#define BOOT_CFG_AUTOBOOT_EN BIT(0)
#define BOOT_CFG_FROM_LAUNCH BIT(1)
#define BOOT_CFG_FROM_ID	 BIT(2)
#define BOOT_CFG_TO_EMUMMC   BIT(3)
#define BOOT_CFG_SEPT_RUN	BIT(7)

#define EXTRA_CFG_KEYS	BIT(0)
#define EXTRA_CFG_PAYLOAD BIT(1)
#define EXTRA_CFG_MODULE  BIT(2)

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

static char* reboot_action = NULL;
static char* default_payload = NULL;
static char* hekate_config_id = NULL;

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

const u32 iram_payload_base = 0x40010000;
static u8 iram_write_buff[IRAM_CHUNK_SIZE];

static void send_smc(u32 func, struct pmc_smc_regs *regs)
{
	u32 ret = func;

	asm volatile(
		"mov x0, %0\n"
		"ldp x1, x2, [%1, #16 * 0]\n"
		"ldp x3, x4, [%1, #16 * 1]\n"
		"ldp x5, x6, [%1, #16 * 2]\n"
		"smc #0\n"
		"mov %0, x0\n"
		"stp x1, x2, [%1, #16 * 0]\n"
		: "+r" (ret)
		: "r" (regs)
		: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8",
		  "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	if (ret) {
			pr_err("%s: failed (ret=%d)\n", __func__, ret);
			WARN_ON(1);
	}
}

static u32 ams_iram_copy(void *dram_addr, uint64_t iram_addr, uint32_t size, uint32_t flag)
{
	struct pmc_smc_regs regs;
	regs.args[0] = virt_to_phys(dram_addr);
	regs.args[1] = iram_addr;
	regs.args[2] = size;
	regs.args[3] = flag;
	regs.args[4] = 0;
	regs.args[5] = 0;
	send_smc(ATMOSPHERE_COPY_TO_IRAM_COMMAND_ID, &regs);
	return (u32)regs.args[0];
}

static void copy_payload(const char *fw_data, size_t fw_size)
{
	size_t i;
	size_t size_remaining;
	size_t copy_size;

	size_remaining = fw_size;
	for(i = 0; i < fw_size; i+=IRAM_CHUNK_SIZE,size_remaining-=IRAM_CHUNK_SIZE)
	{
		copy_size = size_remaining > IRAM_CHUNK_SIZE ? IRAM_CHUNK_SIZE : size_remaining;
		memcpy(iram_write_buff, fw_data + i, copy_size);
		ams_iram_copy(iram_write_buff, iram_payload_base + i, IRAM_CHUNK_SIZE, 0);
	}
}

static int load_payload(struct device *dev, const char *payload_fw_name, bool custom)
{
	const struct firmware *reboot_payload_fw;

	struct reboot_driver_state *state = dev_get_drvdata(dev);

	if(request_firmware_direct(&reboot_payload_fw, payload_fw_name, dev) == 0)
	{
		if(custom)
		{
			memcpy(state->custom_payload_storage, reboot_payload_fw->data, reboot_payload_fw->size);
			state->custom_payload_length = reboot_payload_fw->size;
		}
		else
		{
			memcpy(state->default_payload_storage, reboot_payload_fw->data, reboot_payload_fw->size);
			state->default_payload_length = reboot_payload_fw->size;
		}
		release_firmware(reboot_payload_fw);
		return 0;
	}
	else
	{
		dev_err(dev, "requesting firmware failed for %s :(\n", payload_fw_name);
		return -1;
	}
}

// returns "should reboot to payload" bool
bool ams_prepare_for_r2p(const char *cmd)
{
	bool load_default_payload = true;
	bool do_hekate_config = true;
	struct reboot_driver_state *state;
	struct hekate_boot_cfg hekate_config;

	if (r2p_device == NULL)
		return false;

	state = dev_get_drvdata(&r2p_device->dev);

	memset(&hekate_config, 0, sizeof(struct hekate_boot_cfg));

	if (state->default_payload_length == 0) {
		return false;
	}

	if (cmd != NULL && strcmp(cmd, "payload") == 0) { // custom payload
		do_hekate_config = false;
		// fall back to default if custom is missing.
		if(state->custom_payload_length != 0)
			load_default_payload = false;
	} else if (cmd != NULL && strcmp(cmd, "recovery") == 0) {
		load_default_payload = true;
		do_hekate_config = true;
	} else if (cmd != NULL && strcmp(cmd, "bootloader") == 0) {
		load_default_payload = true;
		do_hekate_config = false;
	} else { // normal reboot or any string not matching
		if (state->reboot_action != NULL && strcmp(state->reboot_action, "via-payload") == 0) {
			load_default_payload = true;
		} else if (state->reboot_action != NULL && strcmp(state->reboot_action, "bootloader") == 0) {
			load_default_payload = true;
			do_hekate_config = false;
		} else {
			return false;
		}
	}

	if (load_default_payload) {
		// Write default payload.
		copy_payload(state->default_payload_storage, state->default_payload_length);

		if (do_hekate_config) {
			if (strlen(state->hekate_id) != 0) {
				hekate_config.boot_cfg = BOOT_CFG_FROM_ID | BOOT_CFG_AUTOBOOT_EN;
				memcpy(hekate_config.id, state->hekate_id, 8);

				// Write Hekate config.
				ams_iram_copy(&hekate_config, iram_payload_base + 0x94, sizeof(struct hekate_boot_cfg), 0);
			}
		}
	} else {
		copy_payload(state->custom_payload_storage, state->custom_payload_length);
	}

	return true;
}

static ssize_t default_payload_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct reboot_driver_state *state = dev_get_drvdata(dev);
	
	if (load_payload(dev, state->default_reboot_payload_name, false) != 0)
		return -EINVAL;
	
	return count;
}

static ssize_t custom_payload_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char payload_name[128];
	
	if (count > sizeof(payload_name)-1)
		return -EINVAL;

	strncpy(payload_name, buf, 127);
	payload_name[count] = 0;

	// strip newline if it exists.
	if (payload_name[count-1] == '\n')
		payload_name[count-1] = 0;

	if (load_payload(dev, payload_name, true) != 0)
		return -EINVAL;

	return count;
}

static DEVICE_ATTR_WO(default_payload);
static DEVICE_ATTR_WO(custom_payload);

static struct attribute *reboot_sysfs_attrs[] = {
	&dev_attr_default_payload.attr,
	&dev_attr_custom_payload.attr,
	NULL,
};
ATTRIBUTE_GROUPS(reboot_sysfs);

static int reboot_to_payload_driver_probe(struct platform_device *pdev)
{
	const char *dt_hekate_id = NULL;
	struct reboot_driver_state *state;
	struct device_node *node = pdev->dev.of_node;
	int len;

	r2p_device = pdev;

	state = vmalloc(sizeof(struct reboot_driver_state));
	memset(state, 0, sizeof(struct reboot_driver_state));

	if (default_payload) { // from cmdline
		state->default_reboot_payload_name = default_payload;
	} else {
		state->default_reboot_payload_name = of_get_property(node, "default-payload", &len);
	}

	if (reboot_action) { // from cmdline
		state->reboot_action = reboot_action;
	} else {
		state->reboot_action = of_get_property(node, "normal-reboot-action", &len);
	}

	// This should be 7 chars (or less). Null terminated.

	if (hekate_config_id) { // from cmdline
		strncpy(state->hekate_id, hekate_config_id, 7);
		state->hekate_id[7] = 0;
	} else {
		dt_hekate_id = of_get_property(node, "hekate-config-id", &len);
		if (dt_hekate_id) {
			strncpy(state->hekate_id, dt_hekate_id, 7);
			state->hekate_id[7] = 0;
		}
	}

	dev_set_drvdata(&pdev->dev, state);

	// This doesn't actually work. Oh well.
	//pdev->dev.groups = reboot_sysfs_groups;
	if (sysfs_create_groups(&pdev->dev.kobj, reboot_sysfs_groups)) {
		dev_err(&pdev->dev, "sysfs creation failed?\n");
	}

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
