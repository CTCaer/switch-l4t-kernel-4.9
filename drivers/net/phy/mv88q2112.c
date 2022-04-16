/*
 * Driver for Marvell 88Q2112 PHY
 *
 * Author: Abdul Mohammed <amohammed@nvidia.com>
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/mdio.h>
#include <linux/of.h>

/* PHY Device ID */
#define MARVELL_PHY_ID_88Q2112		0x002b0980
#define MARVELL_PHY_ID_MASK		0xfffffff0
#define MARVELL_PHY_REV_MASK		0x0000000f

/* Currently support rev A0 only*/
#define PHY_REV_A0		0x1
#define PHY_REV_A1		0x2

/* Delay needed after PHY reset */
#define Q2112_DELAY_MS		50

/* Enable macro to force 100M link speed
 * Default is 1G fixed link speed
 */
#define Q2112_FORCE_100M	0

/* Master/Slave mode config */
#define Q2112_MASTER		1
#define Q2112_SLAVE		0

/* Interrupt Registers */
#define MDIO_INT_EN		0x8010
#define MDIO_INT_STAT		0x8011
#define MDIO_INT_LNKUP		BIT(6)
#define MDIO_INT_LNKDOWN	BIT(7)

/* Autonegotiation Registers */
#define MDIO_AN_CTRL		0x0200
#define MDIO_AN_RESTART		BIT(9)
#define MDIO_AN_EN		BIT(12)
#define MDIO_AN_SWRST		BIT(15)
#define MDIO_AN_STAT		0x0201

/* PMA Registers */
#define MDIO_PMA_CTRL		0x0834
#define MDIO_PMA_SPEED1000	BIT(0)
#define MDIO_PMA_CTRL_MASTER	BIT(14)

/* PCS Control Registers */
#define MDIO_PCS_CTRL		0x0900
#define MDIO_PCS_SWRST		BIT(15)
#define MDIO_PCS_LOOPBACK	BIT(14)

/* PCS Status Registers */
#define MDIO_PCS_STAT1_1000		0x0901
#define MDIO_AN_PCS_STAT		0x8001

#define MDIO_PCS_STAT1_100		0x8109
#define MDIO_PCS_STAT2_100		0x8108

#define MDIO_PCS_RXSTAT		(BIT(12) | BIT(13))

/* Packet Checker Registers */
#define MDIO_PCS_PC_CTRL	0xFD07
#define MDIO_PCS_PC_CNT		0xFD08

#define MDIO_PCS_PC_EN		BIT(2)
#define MDIO_PCS_PC_RXMASK	0x00FF
#define MDIO_PCS_PC_ERRMASK	0xFF00

/* Indirect clause 45 access */
#define MDIO_READ		mv88q2112_read_mmd_indirect
#define MDIO_WRITE		mv88q2112_write_mmd_indirect

struct mv88q2112_stat {
	const char *string;
	int devad;
	u32 regnum;
	u32 mask;
};

static struct mv88q2112_stat mv88q2112_stats[] = {
	{"phy_rx_count", MDIO_MMD_PCS, MDIO_PCS_PC_CNT, MDIO_PCS_PC_RXMASK},
	{"phy_error_count", MDIO_MMD_PCS, MDIO_PCS_PC_CNT,
	 MDIO_PCS_PC_ERRMASK},
};

/* Indirect clause 45 read as per IEEE 802.3 Annex 22D */
static int mv88q2112_read_mmd_indirect(struct phy_device *phydev, int devad,
				       u32 regnum)
{
	struct mii_bus *bus = phydev->mdio.bus;
	int addr = phydev->mdio.addr;
	int val;

	mutex_lock(&bus->mdio_lock);

	bus->write(bus, addr, MII_MMD_CTRL, devad);
	bus->write(bus, addr, MII_MMD_DATA, regnum);
	bus->write(bus, addr, MII_MMD_CTRL, (MII_MMD_CTRL_NOINCR | devad));
	val = bus->read(bus, addr, MII_MMD_DATA);

	mutex_unlock(&bus->mdio_lock);

	return val;
}

/* Indirect clause 45 write as per IEEE 802.3 Annex 22D */
static void mv88q2112_write_mmd_indirect(struct phy_device *phydev, int devad,
					 u32 regnum, u16 data)
{
	struct mii_bus *bus = phydev->mdio.bus;
	int addr = phydev->mdio.addr;

	mutex_lock(&bus->mdio_lock);

	bus->write(bus, addr, MII_MMD_CTRL, devad);
	bus->write(bus, addr, MII_MMD_DATA, regnum);
	bus->write(bus, addr, MII_MMD_CTRL, (MII_MMD_CTRL_NOINCR | devad));
	bus->write(bus, addr, MII_MMD_DATA, data);

	mutex_unlock(&bus->mdio_lock);
}

/* Updates the PHY link status and speed
 * Based on Marvell A0 API v1.00
 */
static int mv88q2112_read_status(struct phy_device *phydev)
{
	int link, status;

	status = MDIO_READ(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL);

	if (status & MDIO_PMA_SPEED1000) {
		link = MDIO_READ(phydev, MDIO_MMD_PCS, MDIO_PCS_STAT1_1000);
		link = MDIO_READ(phydev, MDIO_MMD_PCS, MDIO_PCS_STAT1_1000);
		status = MDIO_READ(phydev, MDIO_MMD_AN, MDIO_AN_PCS_STAT);
		phydev->speed = SPEED_1000;
	} else {
		link = MDIO_READ(phydev, MDIO_MMD_PCS, MDIO_PCS_STAT1_100);
		status = MDIO_READ(phydev, MDIO_MMD_PCS, MDIO_PCS_STAT2_100);
		phydev->speed = SPEED_100;
	}

	if (status < 0)
		return status;

	if (link < 0)
		return link;

	if ((link & MDIO_STAT1_LSTATUS) && (status & MDIO_PCS_RXSTAT))
		phydev->link = 1;
	else
		phydev->link = 0;

	phydev->duplex = DUPLEX_FULL;
	return 0;
}

static int mv88q2112_phy_config_intr(struct phy_device *phydev)
{
	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		MDIO_WRITE(phydev, MDIO_MMD_PCS, MDIO_INT_EN,
			   MDIO_INT_LNKUP | MDIO_INT_LNKDOWN);
	else
		MDIO_WRITE(phydev, MDIO_MMD_PCS, MDIO_INT_EN, 0);
	return 0;
}

static int mv88q2112_phy_ack_interrupt(struct phy_device *phydev)
{
	int rc = MDIO_READ(phydev, MDIO_MMD_PCS, MDIO_INT_STAT);
	return rc;
}

static int mv88q2112_config_aneg(struct phy_device *phydev)
{
	/* Autonegotiation not required
	 * as we implement a fixed link speed
	 */
	return 0;
}

static int mv88q2112_aneg_done(struct phy_device *phydev)
{
	/* Always signal that Autoneg is complete
	 * since it is not configured, and the PHY
	 * state machine will not wait
	 */
	return 1;
}

/* Performs a software reset of the PHY
 * Based on Marvell A0 API v1.00
 */
static int mv88q2112_soft_reset(struct phy_device *phydev)
{
	int reg;
	/* Marvell API v1.0 */
	if (phydev->speed == SPEED_1000) {
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x03E0);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x0000);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x03E1);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x0000);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x0420);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x0000);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x0421);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x0000);
		reg = MDIO_READ(phydev, MDIO_MMD_PCS, MDIO_PCS_CTRL);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, MDIO_PCS_CTRL,
			   reg | MDIO_PCS_SWRST);
		mdelay(1);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x03E0);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x0099);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x03E1);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x0009);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x0420);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x00cc);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC03, 0x0421);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC04, 0x000c);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFFE4, 0x000C);
	} else {
		MDIO_WRITE(phydev, MDIO_MMD_PCS, MDIO_PCS_CTRL, MDIO_PCS_SWRST);
		MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFA07, 0x0200);
	}
	/* Enable the Packet Counter */
	MDIO_WRITE(phydev, MDIO_MMD_PCS, MDIO_PCS_PC_CTRL, MDIO_PCS_PC_EN);
	return 0;
}

/* Initialize the PHY in 100M speed
 * Based on Marvell A0 API v1.00
 */
static void mv88q2112_init_fe(struct phy_device *phydev)
{
	int reg;

	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFA07, 0x0202);
	MDIO_WRITE(phydev, MDIO_MMD_AN, MDIO_AN_CTRL, 0);
	reg = MDIO_READ(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL);
	reg = reg & 0xFFF0;
	MDIO_WRITE(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL, reg);
	mdelay(5);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x8000, 0x0000);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x8100, 0x0200);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFA1E, 0x0002);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFE5C, 0x2402);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFA12, 0x001F);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFA0C, 0x9E05);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbdd, 0x6862);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbde, 0x736e);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbdf, 0x7f79);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbe0, 0x8a85);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbe1, 0x9790);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbe3, 0xa39d);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbe4, 0xb0aa);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xfbe5, 0xb8);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFBFD, 0x0D0A);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFBFE, 0x0906);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x8016, 0x0011);
}

/* Initialize the PHY in 1G speed
 * Based on Marvell A0 API v1.00
 */
static void mv88q2112_init_ge(struct phy_device *phydev)
{
	int reg;

	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFFE4, 0x06B6);
	MDIO_WRITE(phydev, MDIO_MMD_AN, MDIO_AN_CTRL, 0);
	reg = MDIO_READ(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL);
	reg = (reg & 0xFFF0) | MDIO_PMA_SPEED1000;	// fix speed to 1G
	MDIO_WRITE(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL, reg);
	mdelay(5);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFFDE, 0x402F);
	MDIO_WRITE(phydev, MDIO_MMD_AN, 0x8032, 0x0064);
	MDIO_WRITE(phydev, MDIO_MMD_AN, 0x8031, 0x0A01);
	MDIO_WRITE(phydev, MDIO_MMD_AN, 0x8031, 0x0C01);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFE0F, 0x0000);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x800C, 0x0000);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFE2A, 0x3C3D);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC00, 0x01C0);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC17, 0x0425);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC94, 0x5470);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC95, 0x0055);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC19, 0x08d8);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC1a, 0x0110);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC1b, 0x0a10);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC3A, 0x2725);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC61, 0x2627);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC3B, 0x1612);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC62, 0x1C12);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC9D, 0x6367);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC9E, 0x8060);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0xFC00, 0x01C8);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x8000, 0x0000);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x8016, 0x0011);
	MDIO_WRITE(phydev, MDIO_MMD_PCS, 0x8610, 0x0010);
}

#ifdef CONFIG_OF_MDIO
static int mv88q2112_of_get_speed(struct phy_device *phydev)
{
	struct device_node *node = phydev->mdio.dev.of_node;
	u32 speed = 0;

	if (!phydev->mdio.dev.of_node)
		return 0;

	if (!of_property_read_u32(node, "max-speed", &speed))
		return speed;

	return 0;
}

#else
static int mv88q2112_of_get_speed(struct phy_device *phydev)
{
	return 0;
}
#endif

static void mv88q2112_set_mode(struct phy_device *phydev, u8 mode)
{
	u16 reg = MDIO_READ(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL);

	if (mode == Q2112_MASTER)
		reg |= MDIO_PMA_CTRL_MASTER;
	else
		reg &= (~MDIO_PMA_CTRL_MASTER);

	MDIO_WRITE(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_CTRL, reg);
}

static int mv88q2112_config_init(struct phy_device *phydev)
{
	/* Configure PHY in slave mode */
	mv88q2112_set_mode(phydev, Q2112_SLAVE);

	/* Initialize the PHY based on the link speed from DT/Macro */
	if (mv88q2112_of_get_speed(phydev) == SPEED_100 || Q2112_FORCE_100M) {
		phydev->speed = SPEED_100;
		mv88q2112_init_fe(phydev);
	} else {
		phydev->speed = SPEED_1000;
		mv88q2112_init_ge(phydev);
	}

	mv88q2112_soft_reset(phydev);

	mdelay(Q2112_DELAY_MS);

	/* Flag enabled but Autonegotiation not configured */
	phydev->autoneg = AUTONEG_ENABLE;

	/* Set the MII-mode to RGMII */
	phydev->interface = PHY_INTERFACE_MODE_RGMII;

	/* Interrupts disabled for now */
	phydev->interrupts = PHY_INTERRUPT_DISABLED;

	/* A status read here saves a few PHY state machine ticks */
	return mv88q2112_read_status(phydev);
}

static int mv88q2112_get_id(struct phy_device *phydev, u32 *phy_id)
{
	int reg;

	/* Read Device ID first byte */
	reg = MDIO_READ(phydev, MDIO_MMD_PMAPMD, MII_PHYSID1);
	if (reg < 0)
		return reg;

	*phy_id = (reg & 0xFFFF) << 16;

	/* Read Device ID second byte */
	reg = MDIO_READ(phydev, MDIO_MMD_PMAPMD, MII_PHYSID2);
	if (reg < 0)
		return reg;

	*phy_id |= (reg & 0xFFFF);

	if ((*phy_id & MARVELL_PHY_ID_MASK) != MARVELL_PHY_ID_88Q2112)
		return -ENODEV;

	return 0;
}

static int mv88q2112_probe(struct phy_device *phydev)
{
	int ret;
	u32 phy_id = 0, rev;

	ret = mv88q2112_get_id(phydev, &phy_id);
	if (ret < 0)
		return ret;

	rev = phy_id & MARVELL_PHY_REV_MASK;

	/* Currently, only A0 silicon supported */
	if (rev != PHY_REV_A0)
		return -ENODEV;

	return 0;
}

static int mv88q2112_get_sset_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(mv88q2112_stats);
}

static void mv88q2112_get_strings(struct phy_device *phydev, u8 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mv88q2112_stats); i++) {
		strlcpy(data + i * ETH_GSTRING_LEN, mv88q2112_stats[i].string,
			ETH_GSTRING_LEN);
	}
}

static void mv88q2112_get_stats(struct phy_device *phydev,
				struct ethtool_stats *stats, u64 *data)
{
	int i;
	u64 val;

	for (i = 0; i < ARRAY_SIZE(mv88q2112_stats); i++) {
		val =
		    MDIO_READ(phydev, mv88q2112_stats[i].devad,
			      mv88q2112_stats[i].regnum);
		if (val < 0)
			val = U64_MAX;
		val = val & mv88q2112_stats[i].mask;
		data[i] = val;
	}
}

static struct phy_driver mv88q2112_driver[] = {
	{
	 .phy_id = MARVELL_PHY_ID_88Q2112,
	 .phy_id_mask = MARVELL_PHY_ID_MASK,
	 .name = "Marvell 88Q2112",
	 .probe = &mv88q2112_probe,
	 .features = PHY_GBIT_FEATURES,
	 .flags = PHY_HAS_INTERRUPT,
	 .config_init = &mv88q2112_config_init,
	 .soft_reset = &mv88q2112_soft_reset,
	 .read_status = &mv88q2112_read_status,
	 .config_aneg = &mv88q2112_config_aneg,
	 .aneg_done = &mv88q2112_aneg_done,
	 .config_intr = &mv88q2112_phy_config_intr,
	 .ack_interrupt = &mv88q2112_phy_ack_interrupt,
	 .get_sset_count = &mv88q2112_get_sset_count,
	 .get_strings = &mv88q2112_get_strings,
	 .get_stats = &mv88q2112_get_stats,
	 },
};

module_phy_driver(mv88q2112_driver);

static struct mdio_device_id __maybe_unused mv88q2112_tbl[] = {
	{MARVELL_PHY_ID_88Q2112, MARVELL_PHY_ID_MASK},
	{}
};

MODULE_DEVICE_TABLE(mdio, mv88q2112_tbl);

MODULE_DESCRIPTION("Marvell 88Q2112 Ethernet PHY (ver A0) driver (MV88Q2112-A0)");
MODULE_AUTHOR("Abdul Mohammed <amohammed@nvidia.com>");
MODULE_LICENSE("GPL");
