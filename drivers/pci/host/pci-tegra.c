/*
 * PCIe host controller driver for TEGRA SOCs
 *
 * Copyright (c) 2010, CompuLab, Ltd.
 * Author: Mike Rapoport <mike@compulab.co.il>
 *
 * Based on NVIDIA PCIe driver
 * Copyright (c) 2008-2018, NVIDIA Corporation. All rights reserved.
 *
 * Bits taken from arch/arm/mach-dove/pcie.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/msi.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/async.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>
#include <soc/tegra/chip-id.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#include <linux/tegra_prod.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/pm_clock.h>
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
#endif

#include <asm/sizes.h>
#include <asm/mach/pci.h>
#include <asm/io.h>

#include <linux/phy/phy.h>
#include <linux/pci-tegra.h>

#define PCI_CFG_SPACE_SIZE		256
#define PCI_EXT_CFG_SPACE_SIZE	4096

#define AFI_AXI_BAR0_SZ							0x00
#define AFI_AXI_BAR1_SZ							0x04
#define AFI_AXI_BAR2_SZ							0x08
#define AFI_AXI_BAR3_SZ							0x0c
#define AFI_AXI_BAR4_SZ							0x10
#define AFI_AXI_BAR5_SZ							0x14

#define AFI_AXI_BAR0_START						0x18
#define AFI_AXI_BAR1_START						0x1c
#define AFI_AXI_BAR2_START						0x20
#define AFI_AXI_BAR3_START						0x24
#define AFI_AXI_BAR4_START						0x28
#define AFI_AXI_BAR5_START						0x2c

#define AFI_FPCI_BAR0							0x30
#define AFI_FPCI_BAR1							0x34
#define AFI_FPCI_BAR2							0x38
#define AFI_FPCI_BAR3							0x3c
#define AFI_FPCI_BAR4							0x40
#define AFI_FPCI_BAR5							0x44

#define AFI_MSI_BAR_SZ							0x60
#define AFI_MSI_FPCI_BAR_ST						0x64
#define AFI_MSI_AXI_BAR_ST						0x68

#define AFI_MSI_VEC0_0							0x6c
#define AFI_MSI_VEC1_0							0x70
#define AFI_MSI_VEC2_0							0x74
#define AFI_MSI_VEC3_0							0x78
#define AFI_MSI_VEC4_0							0x7c
#define AFI_MSI_VEC5_0							0x80
#define AFI_MSI_VEC6_0							0x84
#define AFI_MSI_VEC7_0							0x88

#define AFI_MSI_EN_VEC0_0						0x8c
#define AFI_MSI_EN_VEC1_0						0x90
#define AFI_MSI_EN_VEC2_0						0x94
#define AFI_MSI_EN_VEC3_0						0x98
#define AFI_MSI_EN_VEC4_0						0x9c
#define AFI_MSI_EN_VEC5_0						0xa0
#define AFI_MSI_EN_VEC6_0						0xa4
#define AFI_MSI_EN_VEC7_0						0xa8

#define AFI_CONFIGURATION						0xac
#define AFI_CONFIGURATION_EN_FPCI					(1 << 0)
#define AFI_CONFIGURATION_CLKEN_OVERRIDE				(1 << 31)

#define AFI_FPCI_ERROR_MASKS						0xb0

#define AFI_INTR_MASK							0xb4
#define AFI_INTR_MASK_INT_MASK					(1 << 0)
#define AFI_INTR_MASK_MSI_MASK					(1 << 8)

#define AFI_INTR_CODE							0xb8
#define AFI_INTR_CODE_MASK						0x1f
#define AFI_INTR_MASTER_ABORT						4
#define AFI_INTR_LEGACY						6
#define AFI_INTR_PRSNT_SENSE						10

#define AFI_INTR_SIGNATURE						0xbc
#define AFI_SM_INTR_ENABLE						0xc4

#define AFI_AFI_INTR_ENABLE						0xc8
#define AFI_INTR_EN_INI_SLVERR						(1 << 0)
#define AFI_INTR_EN_INI_DECERR						(1 << 1)
#define AFI_INTR_EN_TGT_SLVERR						(1 << 2)
#define AFI_INTR_EN_TGT_DECERR						(1 << 3)
#define AFI_INTR_EN_TGT_WRERR						(1 << 4)
#define AFI_INTR_EN_DFPCI_DECERR					(1 << 5)
#define AFI_INTR_EN_AXI_DECERR						(1 << 6)
#define AFI_INTR_EN_FPCI_TIMEOUT					(1 << 7)

#define AFI_PCIE_PME						0x0f0
#define AFI_PCIE_PME_TURN_OFF					0x101
#define AFI_PCIE_PME_ACK					0x420

#define AFI_PCIE_CONFIG						0x0f8
#define AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE			(1 << 1)
#define AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE			(1 << 2)
#define AFI_PCIE_CONFIG_PCIEC2_DISABLE_DEVICE			(1 << 3)
#define AFI_PCIE_CONFIG_XBAR_CONFIG_MASK			(0xf << 20)
#define AFI_PCIE_CONFIG_XBAR_CONFIG_X2_X1			(0x0 << 20)
#define AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X1			(0x1 << 20)
#define AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X0_X1		(0x0 << 20)
#define AFI_PCIE_CONFIG_XBAR_CONFIG_X2_X1_X1		(0x1 << 20)
#define AFI_PCIE_CONFIG_XBAR_CONFIG_X1_X1_X1		(0x2 << 20)
#define AFI_PCIE_CONFIG_PCIEC0_CLKREQ_AS_GPIO		BIT(29)
#define AFI_PCIE_CONFIG_PCIEC1_CLKREQ_AS_GPIO		BIT(30)
#define AFI_PCIE_CONFIG_PCIEC2_CLKREQ_AS_GPIO		BIT(31)

#define AFI_FUSE							0x104
#define AFI_FUSE_PCIE_T0_GEN2_DIS				(1 << 2)

#define AFI_PEX0_CTRL							0x110
#define AFI_PEX1_CTRL							0x118
#define AFI_PEX2_CTRL							0x19C
#define AFI_PEX_CTRL_RST					(1 << 0)
#define AFI_PEX_CTRL_CLKREQ_EN					(1 << 1)
#define AFI_PEX_CTRL_REFCLK_EN					(1 << 3)
#define AFI_PEX_CTRL_OVERRIDE_EN				(1 << 4)

#define AFI_PLLE_CONTROL					0x160
#define AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL		(1 << 9)
#define AFI_PLLE_CONTROL_BYPASS_PCIE2PLLE_CONTROL		(1 << 8)
#define AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN			(1 << 1)
#define AFI_PLLE_CONTROL_PCIE2PLLE_CONTROL_EN			(1 << 0)

#define AFI_PEXBIAS_CTRL_0					0x168
#define AFI_WR_SCRATCH_0					0x120
#define AFI_WR_SCRATCH_0_RESET_VAL				0x00202020
#define AFI_WR_SCRATCH_0_DEFAULT_VAL				0x00000000

#define AFI_MSG_0						0x190
#define AFI_MSG_PM_PME_MASK					0x00100010
#define AFI_MSG_INTX_MASK					0x1f001f00
#define AFI_MSG_PM_PME0						(1 << 4)
#define AFI_MSG_PM_PME1						(1 << 20)
#define AFI_MSG_RP_INT_MASK					0x10001000

#define AFI_MSG_1_0						0x194
#define AFI_MSG_1_PM_PME_MASK					0x00000010
#define AFI_MSG_1_INTX_MASK					0x00001f00
#define AFI_MSG_1_PM_PME					(1 << 4)

#define RP_VEND_XP						0x00000F00
#define RP_VEND_XP_OPPORTUNISTIC_ACK				(1 << 27)
#define RP_VEND_XP_OPPORTUNISTIC_UPDATEFC			(1 << 28)
#define RP_VEND_XP_DL_UP					(1 << 30)
#define RP_VEND_XP_UPDATE_FC_THRESHOLD				(0xFF << 18)
#define RP_VEND_XP_PRBS_STAT					(0xFFFF << 2)
#define RP_VEND_XP_PRBS_EN					(1 << 1)

#define RP_LINK_CONTROL_STATUS					0x00000090
#define RP_LINK_CONTROL_STATUS_BW_MGMT_STATUS	0x40000000
#define RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE	0x20000000
#define RP_LINK_CONTROL_STATUS_LINKSTAT_MASK	0x3fff0000
#define RP_LINK_CONTROL_STATUS_NEG_LINK_WIDTH	(0x3F << 20)
#define RP_LINK_CONTROL_STATUS_LINK_SPEED		(0xF << 16)
#define RP_LINK_CONTROL_STATUS_L0s_ENABLED		0x00000001
#define RP_LINK_CONTROL_STATUS_L1_ENABLED		0x00000002

#define RP_LINK_CONTROL_STATUS_2				0x000000B0
#define RP_LINK_CONTROL_STATUS_2_TRGT_LNK_SPD_MASK	0x0000000F
#define RP_LINK_CONTROL_STATUS_2_TRGT_LNK_SPD_GEN1	0x00000001
#define RP_LINK_CONTROL_STATUS_2_TRGT_LNK_SPD_GEN2	0x00000002

#define NV_PCIE2_RP_RSR					0x000000A0
#define NV_PCIE2_RP_RSR_PMESTAT				(1 << 16)

#define NV_PCIE2_RP_INTR_BCR					0x0000003C
#define NV_PCIE2_RP_INTR_BCR_INTR_LINE			(0xFF << 0)
#define NV_PCIE2_RP_INTR_BCR_SB_RESET			(0x1 << 22)

#define NV_PCIE2_RP_PRIV_XP_DL					0x00000494
#define PCIE2_RP_PRIV_XP_DL_GEN2_UPD_FC_TSHOLD			(0x1FF << 1)

#define NV_PCIE2_RP_RX_HDR_LIMIT				0x00000E00
#define PCIE2_RP_RX_HDR_LIMIT_PW_MASK				(0xFF00)
#define PCIE2_RP_RX_HDR_LIMIT_PW				(0x0E << 8)

#define NV_PCIE2_RP_TX_HDR_LIMIT				0x00000E08
#define PCIE2_RP_TX_HDR_LIMIT_NPT_0				32
#define PCIE2_RP_TX_HDR_LIMIT_NPT_1				4

#define NV_PCIE2_RP_TIMEOUT0					0x00000E24
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP_MASK			(0xFF)
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP				(0xA)
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM_MASK			(0xFFFF00)
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM				(0x180 << 8)
#define PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2_MASK		(0xFF << 24)
#define PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2			(0xA << 24)

#define NV_PCIE2_RP_TIMEOUT1					0x00000E28
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE_MASK		(0xFF << 16)
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE		(0x10 << 16)
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE_MASK	(0xFF << 24)
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE		(0x74 << 24)

#define NV_PCIE2_RP_PRBS						0x00000E34
#define PCIE2_RP_PRBS_LOCKED					(1 << 16)

#define NV_PCIE2_RP_LANE_PRBS_ERR_COUNT				0x00000E38
#define PCIE2_RP_LANE_PRBS_ERR_COUNT				(1 << 16)
#define PCIE2_RP_LANE_PRBS_ERR_SELECT				(1 << 0)

#define NV_PCIE2_RP_LTSSM_DBGREG			0x00000E44
#define PCIE2_RP_LTSSM_DBGREG_LINKFSM15		(1 << 15)
#define PCIE2_RP_LTSSM_DBGREG_LINKFSM16		(1 << 16)
#define PCIE2_RP_LTSSM_DBGREG_LINKFSM17		(1 << 17)

#define NV_PCIE2_RP_LTSSM_TRACE_CONTROL				0x00000E50
#define LTSSM_TRACE_CONTROL_CLEAR_STORE_EN			(1 << 0)
#define LTSSM_TRACE_CONTROL_CLEAR_RAM				(1 << 2)
#define LTSSM_TRACE_CONTROL_TRIG_ON_EVENT			(1 << 3)
#define LTSSM_TRACE_CONTROL_TRIG_LTSSM_MAJOR_OFFSET		4
#define LTSSM_TRACE_CONTROL_TRIG_PTX_LTSSM_MINOR_OFFSET		8
#define LTSSM_TRACE_CONTROL_TRIG_PRX_LTSSM_MAJOR_OFFSET		11

#define NV_PCIE2_RP_LTSSM_TRACE_STATUS				0x00000E54
#define LTSSM_TRACE_STATUS_PRX_MINOR(reg)			(((reg) >> 19) & 0x7)
#define LTSSM_TRACE_STATUS_PTX_MINOR(reg)			(((reg) >> 16) & 0x7)
#define LTSSM_TRACE_STATUS_MAJOR(reg)				(((reg) >> 12) & 0xf)
#define LTSSM_TRACE_STATUS_READ_DATA_VALID(reg)			(((reg) >> 11) & 0x1)
#define LTSSM_TRACE_STATUS_READ_ADDR(reg)			((reg) << 6)
#define LTSSM_TRACE_STATUS_WRITE_POINTER(reg)			(((reg) >> 1) & 0x1f)
#define LTSSM_TRACE_STATUS_RAM_FULL(reg)			(reg & 0x1)


#define NV_PCIE2_RP_XP_REF					0x00000F30
#define PCIE2_RP_XP_REF_MICROSECOND_LIMIT_MASK			(0xFF)
#define PCIE2_RP_XP_REF_MICROSECOND_LIMIT			(0x14)
#define PCIE2_RP_XP_REF_MICROSECOND_ENABLE			(1 << 8)
#define PCIE2_RP_XP_REF_CPL_TO_OVERRIDE			(1 << 13)
#define PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE_MASK		(0x1FFFF << 14)
#define PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE			(0x1770 << 14)

#define NV_PCIE2_RP_PRIV_MISC					0x00000FE0
#define PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_PRSNT			(0xE << 0)
#define PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT			(0xF << 0)
#define PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_THRESHOLD		(0xF << 16)
#define PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_ENABLE		(1 << 23)
#define PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_THRESHOLD		(0xF << 24)
#define PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_ENABLE		(1 << 31)

#define NV_PCIE2_RP_VEND_XP1					0x00000F04
#define NV_PCIE2_RP_VEND_XP2					0x00000F08
#define NV_PCIE2_RP_VEND_XP_LINK_PVT_CTL_L1_ASPM_SUPPORT	(1 << 21)
#define NV_PCIE2_RP_VEND_XP1_RNCTRL_MAXWIDTH_MASK	(0x3F << 0)
#define NV_PCIE2_RP_VEND_XP1_RNCTRL_EN				(1 << 7)

#define NV_PCIE2_RP_VEND_CTL0					0x00000F44
#define PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH_MASK		(0xF << 12)
#define PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH			(0x9 << 12)

#define NV_PCIE2_RP_VEND_CTL1					0x00000F48
#define PCIE2_RP_VEND_CTL1_ERPT				(1 << 13)

#define NV_PCIE2_RP_VEND_XP_BIST				0x00000F4C
#define PCIE2_RP_VEND_XP_BIST_GOTO_L1_L2_AFTER_DLLP_DONE	(1 << 28)

#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN			0x00000F50
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_L1_EN		(1 << 0)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_DYNAMIC_EN		(1 << 1)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_DISABLED_EN		(1 << 2)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_L1_CLKREQ_EN	(1 << 15)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_DYNAMIC_L1PP	(3 << 5)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_L1_L1P	(2 << 3)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_L1_L1PP	(3 << 3)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_L1_CLKREQ_L1P	(2 << 16)
#define NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_L1_CLKREQ_L1PP	(3 << 16)

#define NV_PCIE2_RP_PRIV_XP_RX_L0S_ENTRY_COUNT			0x00000F8C
#define NV_PCIE2_RP_PRIV_XP_TX_L0S_ENTRY_COUNT			0x00000F90
#define NV_PCIE2_RP_PRIV_XP_TX_L1_ENTRY_COUNT			0x00000F94
#define NV_PCIE2_RP_LTR_REP_VAL				0x00000C10
#define NV_PCIE2_RP_L1_1_ENTRY_COUNT				0x00000C14
#define PCIE2_RP_L1_1_ENTRY_COUNT_RESET			(1 << 31)
#define NV_PCIE2_RP_L1_2_ENTRY_COUNT				0x00000C18
#define PCIE2_RP_L1_2_ENTRY_COUNT_RESET			(1 << 31)

#define NV_PCIE2_RP_VEND_CTL2					0x00000FA8
#define PCIE2_RP_VEND_CTL2_PCA_ENABLE				(1 << 7)

#define NV_PCIE2_RP_PRIV_XP_CONFIG				0x00000FAC
#define NV_PCIE2_RP_PRIV_XP_CONFIG_LOW_PWR_DURATION_MASK	0x3
#define NV_PCIE2_RP_PRIV_XP_DURATION_IN_LOW_PWR_100NS	0x00000FB0

#define NV_PCIE2_RP_XP_CTL_1					0x00000FEC
#define PCIE2_RP_XP_CTL_1_OLD_IOBIST_EN_BIT25				(1 << 25)
#define PCIE2_RP_XP_CTL_1_SPARE_BIT29				(1 << 29)

#define NV_PCIE2_RP_L1_PM_SUBSTATES_CYA				0x00000C00
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_MASK		(0xFF << 8)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_SHIFT		(8)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_MASK		(0x3 << 16)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_SHIFT		(16)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_MASK		(0x1F << 19)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_SHIFT		(19)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_HIDE_CAP			(0x1 << 24)

#define NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA			0x00000C04
#define PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY_MASK	(0x1FFF)
#define PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY		(0x26)
#define PCIE2_RP_L1SS_1_CYA_CLKREQ_ASSERTED_DLY_MASK	(0x1FF << 13)
#define PCIE2_RP_L1SS_1_CYA_CLKREQ_ASSERTED_DLY		(0x27 << 13)

#define NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA			0x00000C08
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY_MASK		(0x1FFF)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY		(0x4D)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_MASK	(0xFF << 13)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND		(0x13 << 13)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP_MASK	(0xF << 21)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP	(0x2 << 21)

#define PCIE2_RP_L1SS_SPARE			0xC24
#define PCIE2_RP_L1SS_SPARE_LTR_MSG_INT_EN	(1 << 16)
#define PCIE2_RP_L1SS_SPARE_LTR_MSG_RCV_STS	(1 << 17)

#define PCIE2_RP_L1_PM_SS_CONTROL		0x00000148
#define PCIE2_RP_L1_PM_SS_CONTROL_ASPM_L11_ENABLE	0x00000008
#define PCIE2_RP_L1_PM_SS_CONTROL_ASPM_L12_ENABLE	0x00000004

#define INT_PCI_MSI_NR			(32 * 8)

#define LINK_RETRAIN_TIMEOUT 100000

#define PCIE_LANES_X4_X1		0x14

#define DEBUG 0
#if DEBUG || defined(CONFIG_PCI_DEBUG)
#define PR_FUNC_LINE	pr_info("PCIE: %s(%d)\n", __func__, __LINE__)
#else
#define PR_FUNC_LINE	do {} while (0)
#endif

struct pcie_dvfs {
	u32 afi_clk;
	u32 emc_clk;
};

struct tegra_pcie_soc_data {
	unsigned int	num_ports;
	char			**pcie_regulator_names;
	int				num_pcie_regulators;
	bool			config_pex_io_dpd;
	bool			program_uphy;
	bool			program_clkreq_as_bi_dir;
	bool			enable_wrap;
	bool			mbist_war;
	bool			RAW_violation_war;
	bool			perf_war;
	bool			updateFC_timer_expire_war;
	bool			l1ss_rp_wakeup_war;
	bool			link_speed_war;
	bool			dvfs_mselect;
	bool			dvfs_afi;
	bool			update_clamp_threshold;
	struct pcie_dvfs	dvfs_tbl[10][2];
};

struct tegra_msi {
	struct msi_controller chip;
	DECLARE_BITMAP(used, INT_PCI_MSI_NR);
	struct irq_domain *domain;
	struct mutex lock;
	void *virt;
	u64 phys;
	int irq;
};

static inline struct tegra_msi *to_tegra_msi(struct msi_controller *chip)
{
	return container_of(chip, struct tegra_msi, chip);
}

struct tegra_pcie {
	struct device *dev;
	struct pci_host_bridge *host;

	void __iomem *pads;
	void __iomem *afi;
	int irq;

	struct list_head buses;
	struct list_head sys;
	struct resource *cs;
	struct resource *afi_res;
	struct resource *pads_res;

	struct resource io;
	struct resource pio;
	struct resource mem;
	struct resource prefetch;
	struct resource busn;

	struct {
		resource_size_t mem;
		resource_size_t io;
	} offset;

	void __iomem *cfg_va_base;

	struct tegra_msi msi;

	struct reset_control *afi_rst;
	struct reset_control *pcie_rst;
	struct reset_control *pciex_rst;
	struct tegra_bwmgr_client *emc_bwmgr;

#ifdef CONFIG_THERMAL
	struct thermal_cooling_device *cdev;
	atomic_t therm_state;
	struct completion completion;
	bool is_cooling_dev;
#endif

	struct list_head ports;
	u32 xbar_config;
	int num_ports;

	int power_rails_enabled;
	int pcie_power_enabled;
	struct work_struct hotplug_detect;

	struct regulator	**pcie_regulators;

	struct pinctrl		*pex_pin;
	struct pinctrl_state	*pex_io_dpd_en_state;
	struct pinctrl_state	*pex_io_dpd_dis_state;

	struct tegra_pci_platform_data *plat_data;
	struct tegra_pcie_soc_data *soc_data;
	struct dentry *debugfs;
	struct delayed_work detect_delay;
	struct tegra_prod	*prod_list;
};

static int tegra_pcie_enable_msi(struct tegra_pcie *pcie, bool no_init);
static void tegra_pcie_check_ports(struct tegra_pcie *pcie);
static void tegra_pcie_link_speed(struct tegra_pcie *pcie);
static int tegra_pcie_power_off(struct tegra_pcie *pcie);

#define MAX_PWR_GPIOS 5

struct tegra_pcie_port {
	struct tegra_pcie *pcie;
	struct list_head list;
	struct resource regs;
	void __iomem *base;
	unsigned int index;
	unsigned int lanes;
	unsigned int num_lanes;
	unsigned int loopback_stat;
	int gpio_presence_detection;
	bool disable_clock_request;
	bool ep_status;
	int status;
	int rst_gpio;
	bool has_mxm_port;
	int pwr_gd_gpio;
	struct dentry *port_debugfs;
	struct phy **phy;
	struct device_node *np;
	int n_gpios;
	int *gpios;
};

struct tegra_pcie_bus {
	struct list_head list;
	unsigned int nr;
};

/* used to avoid successive hotplug disconnect or connect */
static bool hotplug_event;
/* pcie mselect, xclk and emc rate */
static u16 bdf;
static u16 config_offset;
static u32 config_val;
static u16 config_aspm_state;

static inline void afi_writel(struct tegra_pcie *pcie, u32 value,
							  unsigned long offset)
{
	writel(value, offset + pcie->afi);
}

static inline u32 afi_readl(struct tegra_pcie *pcie, unsigned long offset)
{
	return readl(offset + pcie->afi);
}

static inline void __maybe_unused pads_writel(struct tegra_pcie *pcie, u32 value,
							   unsigned long offset)
{
	writel(value, offset + pcie->pads);
}

static inline u32 __maybe_unused pads_readl(struct tegra_pcie *pcie, unsigned long offset)
{
	return readl(offset + pcie->pads);
}

static inline void rp_writel(struct tegra_pcie_port *port, u32 value,
							 unsigned long offset)
{
	writel(value, offset + port->base);
}

static inline unsigned int rp_readl(struct tegra_pcie_port *port,
							unsigned long offset)
{
	return readl(offset + port->base);
}

/*
 * The configuration space mapping on Tegra is somewhat similar to the ECAM
 * defined by PCIe. However it deviates a bit in how the 4 bits for extended
 * register accesses are mapped:
 *
 *    [27:24] extended register number
 *    [23:16] bus number
 *    [15:11] device number
 *    [10: 8] function number
 *    [ 7: 0] register number
 *
 * Mapping the whole extended configuration space would required 256 MiB of
 * virtual address space, only a small part of which will actually be used.
 * To work around this, a 4K of region is used to generate required
 * configuration transaction with relevant B:D:F values. This is achieved by
 * dynamically programming base address and size of AFI_AXI_BAR used for
 * end point config space mapping to make sure that the address (access to
 * which generates correct config transaction) falls in this 4K region
 */
static struct tegra_pcie_bus *tegra_pcie_bus_alloc(struct tegra_pcie *pcie,
							unsigned int busnr)
{
	struct tegra_pcie_bus *bus;

	PR_FUNC_LINE;
	bus = devm_kzalloc(pcie->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&bus->list);
	bus->nr = busnr;

	if (!pcie->cfg_va_base) {
		pcie->cfg_va_base = ioremap(pcie->cs->start, SZ_4K);
		if (!pcie->cfg_va_base) {
			dev_err(pcie->dev, "failed to ioremap config space\n");
			kfree(bus);
			bus = (struct tegra_pcie_bus *)-ENOMEM;
		}
	}

	return bus;
}

static void __iomem *tegra_pcie_conf_address(struct pci_bus *bus,
					     unsigned int devfn,
					     int where)
{
	struct pci_host_bridge *host = pci_find_host_bridge(bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);
	void __iomem *addr = NULL;
	u32 val = 0;

	if (bus->number == 0) {
		unsigned int slot = PCI_SLOT(devfn);
		struct tegra_pcie_port *port;

		list_for_each_entry(port, &pcie->ports, list) {
			if ((port->index + 1 == slot) && port->status) {
				addr = port->base + (where & ~3);
				break;
			}
		}
	} else {
		addr = pcie->cfg_va_base;
		val = ((((u32)where & 0xf00) >> 8) << 24) |
		      (bus->number << 16) | (PCI_SLOT(devfn) << 11) |
		      (PCI_FUNC(devfn) << 8) | (where & 0xff);
		addr = (val & (SZ_4K - 1)) + addr;
		val = val & ~(SZ_4K - 1);
		afi_writel(pcie, pcie->cs->start - val, AFI_AXI_BAR0_START);
		afi_writel(pcie, (val + SZ_4K) >> 12, AFI_AXI_BAR0_SZ);
	}

	return addr;
}

static int tegra_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *value)
{
	struct pci_host_bridge *host = pci_find_host_bridge(bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);
	struct tegra_pcie_port *port = NULL;
	u32 rp = 0;
	struct pci_dev *dn_dev;

	if (bus->number) {
		if (bus->number == 1) {
			dn_dev = bus->self;
			rp = PCI_SLOT(dn_dev->devfn);
			list_for_each_entry(port, &pcie->ports, list)
				if (rp == port->index + 1)
					break;
			if (!port->ep_status)
				return PCIBIOS_DEVICE_NOT_FOUND;
		}
		return pci_generic_config_read(bus, devfn, where, size, value);
	}
	return pci_generic_config_read32(bus, devfn, where, size, value);
}

static int tegra_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 value)
{
	struct pci_host_bridge *host = pci_find_host_bridge(bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);
	struct tegra_pcie_port *port = NULL;
	u32 rp = 0;
	struct pci_dev *dn_dev;

	if (bus->number) {
		if (bus->number == 1) {
			dn_dev = bus->self;
			rp = PCI_SLOT(dn_dev->devfn);
			list_for_each_entry(port, &pcie->ports, list)
				if (rp == port->index + 1)
					break;
			if (!port->ep_status)
				return PCIBIOS_DEVICE_NOT_FOUND;
		}
		return pci_generic_config_write(bus, devfn, where, size, value);
	}
	return pci_generic_config_write32(bus, devfn, where, size, value);
}

static void tegra_pcie_fixup_bridge(struct pci_dev *dev)
{
	u16 reg;

	if ((dev->class >> 16) == PCI_BASE_CLASS_BRIDGE) {
		pci_read_config_word(dev, PCI_COMMAND, &reg);
		reg |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
			PCI_COMMAND_MASTER | PCI_COMMAND_SERR);
		pci_write_config_word(dev, PCI_COMMAND, reg);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, tegra_pcie_fixup_bridge);

/* Tegra PCIE root complex wrongly reports device class */
static void tegra_pcie_fixup_class(struct pci_dev *dev)
{
	dev->class = PCI_CLASS_BRIDGE_PCI << 8;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA, 0x0e1c, tegra_pcie_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA, 0x0e1d, tegra_pcie_fixup_class);

/* Tegra PCIE requires relaxed ordering */
static void tegra_pcie_relax_enable(struct pci_dev *dev)
{
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, tegra_pcie_relax_enable);

static int tegra_pcie_request_resources(struct tegra_pcie *pcie)
{
	struct pci_host_bridge *host = pci_host_bridge_from_priv(pcie);
	struct list_head *windows = &host->windows;
	struct device *dev = pcie->dev;
	int err;

	pci_add_resource_offset(windows, &pcie->pio, pcie->offset.io);
	pci_add_resource_offset(windows, &pcie->mem, pcie->offset.mem);
	pci_add_resource_offset(windows, &pcie->prefetch, pcie->offset.mem);
	pci_add_resource(windows, &pcie->busn);

	err = devm_request_pci_bus_resources(dev, windows);
	if (err < 0) {
		pci_free_resource_list(windows);
		return err;
	}

	pci_remap_iospace(&pcie->pio, pcie->io.start);

	return 0;
}

static void tegra_pcie_free_resources(struct tegra_pcie *pcie)
{
	struct pci_host_bridge *host = pci_host_bridge_from_priv(pcie);
	struct list_head *windows = &host->windows;
	struct resource_entry *win, *tmp;

	pci_unmap_iospace(&pcie->pio);

	resource_list_for_each_entry_safe(win, tmp, &host->windows) {
		switch (resource_type(win->res)) {
		case IORESOURCE_IO:
		case IORESOURCE_MEM:
			devm_release_resource(pcie->dev, win->res);
			break;
		default:
			continue;
		}
	}
	pci_free_resource_list(windows);
}

static int tegra_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pci_host_bridge *host = pci_find_host_bridge(dev->bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);

	return pcie->irq;
}

static int tegra_pcie_add_bus(struct pci_bus *bus)
{
	struct tegra_pcie_bus *tbus;
	struct pci_host_bridge *host = pci_find_host_bridge(bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);

	PR_FUNC_LINE;
	/* bus 0 is root complex whose config space is already mapped */
	if (!bus->number)
		return 0;
	if (IS_ENABLED(CONFIG_PCI_MSI))
		bus->msi = &pcie->msi.chip;

	/* Allocate memory for new bus */
	tbus = tegra_pcie_bus_alloc(pcie, bus->number);
	if (IS_ERR(tbus))
		return 0;
	list_add_tail(&tbus->list, &pcie->buses);
	return 0;
}

static struct pci_ops tegra_pcie_ops = {
	.add_bus = tegra_pcie_add_bus,
	.map_bus = tegra_pcie_conf_address,
	.read	= tegra_pcie_read_conf,
	.write	= tegra_pcie_write_conf,
};

static unsigned long tegra_pcie_port_get_pex_ctrl(struct tegra_pcie_port *port)
{
	unsigned long ret = 0;

	switch (port->index) {
	case 0:
		ret = AFI_PEX0_CTRL;
		break;
	case 1:
		ret = AFI_PEX1_CTRL;
		break;
	case 2:
		ret = AFI_PEX2_CTRL;
		break;
	}
	return ret;
}

static void tegra_pcie_config_clkreq(struct tegra_pcie *pcie, u32 index,
				     int bi_dir)
{
	struct pinctrl		   *clkreq_pin = NULL;
	struct pinctrl_state	   *clkreq_dir = NULL;
	int ret = 0;

	PR_FUNC_LINE;

	clkreq_pin = devm_pinctrl_get(pcie->dev);
	if (IS_ERR(clkreq_pin)) {
		dev_err(pcie->dev, "config clkreq dir failed: %ld\n",
			PTR_ERR(clkreq_pin));
		return;
	}

	switch (index) {
	case 0:
		if (bi_dir)
			clkreq_dir =
				pinctrl_lookup_state(clkreq_pin,
						     "clkreq-0-bi-dir-enable");
		else
			clkreq_dir =
				pinctrl_lookup_state(clkreq_pin,
						     "clkreq-0-in-dir-enable");
		break;
	case 1:
		if (bi_dir)
			clkreq_dir =
				pinctrl_lookup_state(clkreq_pin,
						     "clkreq-1-bi-dir-enable");
		else
			clkreq_dir =
				pinctrl_lookup_state(clkreq_pin,
						     "clkreq-1-in-dir-enable");
		break;
	default:
		return;
	}

	if (IS_ERR(clkreq_dir)) {
		dev_err(pcie->dev, "missing clkreq_dir_enable state: %ld\n",
			PTR_ERR(clkreq_dir));
		return;
	}
	ret = pinctrl_select_state(clkreq_pin, clkreq_dir);
	if (ret < 0)
		dev_err(pcie->dev,
			"setting clkreq pin dir state failed: %d\n", ret);
}

#if defined(CONFIG_PCIEASPM)
static void tegra_pcie_enable_ltr_support(void)
{
	struct pci_dev *pdev = NULL;
	u16 val = 0;
	u32 data = 0, pos = 0;

	PR_FUNC_LINE;
	/* enable LTR mechanism for L1.2 support in end points */
	for_each_pci_dev(pdev) {
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		if (!pos)
			continue;
		pci_read_config_dword(pdev, pos + PCI_L1SS_CAP, &data);
		if (!((data & PCI_L1SS_CAP_ASPM_L12S) ||
			(data & PCI_L1SS_CAP_PM_L12S)))
			continue;
		pcie_capability_read_dword(pdev, PCI_EXP_DEVCAP2, &data);
		if (data & PCI_EXP_DEVCAP2_LTR) {
			pcie_capability_read_word(pdev, PCI_EXP_DEVCTL2, &val);
			val |= PCI_EXP_DEVCTL2_LTR_EN;
			pcie_capability_write_word(pdev, PCI_EXP_DEVCTL2, val);
		}
	}
}

struct dev_ids {
	unsigned short	vid;
	unsigned short	did;
};

static const struct pci_device_id aspm_l0s_blacklist[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_BROADCOM, 0x4355), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BROADCOM, 0x43ef), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BROADCOM, 0x4354), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_NEC, 0x0194), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_TOSHIBA, 0x010f), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0953), 0, 0, 0 },
	{ 0 }
};

/* Enable ASPM support of all devices based on it's capability */
static void tegra_pcie_configure_aspm(void)
{
	struct pci_dev *pdev = NULL;
	struct tegra_pcie *pcie = NULL;
	struct pci_host_bridge *host = NULL;

	PR_FUNC_LINE;
	if (!pcie_aspm_support_enabled()) {
		pr_info("PCIE: ASPM not enabled\n");
		return;
	}

	pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);

	host = pci_find_host_bridge(pdev->bus);
	pcie = pci_host_bridge_priv(host);
	pdev = NULL;

	/* disable ASPM-L0s for all links unless the endpoint
	 * is a known device with proper ASPM-L0s functionality
	 */
	for_each_pci_dev(pdev) {
		struct pci_dev *parent = NULL;
		struct tegra_pcie_port *port = NULL;
		unsigned long ctrl = 0;
		u32 rp = 0, val = 0, i = 0;

		if ((pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT) ||
			(pci_pcie_type(pdev) == PCI_EXP_TYPE_DOWNSTREAM))
			continue;
		parent = pdev->bus->self;

		/* following needs to be done only for devices which are
		 * directly connected to Tegra root ports */
		if (parent->bus->self)
			continue;
		rp = PCI_SLOT(parent->devfn);
		list_for_each_entry(port, &pcie->ports, list)
			if (rp == port->index + 1)
				break;
		ctrl = tegra_pcie_port_get_pex_ctrl(port);
		/* AFI_PEX_STATUS is AFI_PEX_CTRL + 4 */
		val = afi_readl(port->pcie, ctrl + 4);
		if ((val & 0x1) || (port->disable_clock_request)) {
			i |= PCIE_LINK_STATE_CLKPM;
			/* disable PADS2PLLE control */
			val = afi_readl(port->pcie, AFI_PLLE_CONTROL);
			val &= ~AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN;
			afi_writel(port->pcie, val, AFI_PLLE_CONTROL);
		}

		/* Disable ASPM-l0s for blacklisted devices */
		if (pci_match_id(aspm_l0s_blacklist, pdev))
			i |= PCIE_LINK_STATE_L0S;

		pci_disable_link_state_locked(pdev, i);

		if (port->disable_clock_request)
			continue;

		if (pcie->soc_data->program_clkreq_as_bi_dir) {
			/* check if L1SS capability is supported */
			i = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
			if (!i)
				continue;
			/* avoid L1SS config if no support of L1SS feature */
			pci_read_config_dword(pdev, i + PCI_L1SS_CAP, &val);
			if ((val & PCI_L1SS_CAP_L1PMS) ||
			    (val & PCI_L1SS_CAP_L1PM_MASK))
				tegra_pcie_config_clkreq(pcie, port->index, 1);
		}
	}
	/* L1.2 specific common configuration */
	tegra_pcie_enable_ltr_support();
}
#endif

static void tegra_pcie_postinit(void)
{
#if defined(CONFIG_PCIEASPM)
	tegra_pcie_configure_aspm();
#endif
}

#ifdef HOTPLUG_ON_SYSTEM_BOOT
/* It enumerates the devices when dock is connected after system boot */
/* this is similar to pcibios_init_hw in bios32.c */
static void __init tegra_pcie_hotplug_init(void)
{
	struct pci_sys_data *sys = NULL;
	int ret, nr;

	if (is_dock_conn_at_boot)
		return;

	PR_FUNC_LINE;
	tegra_pcie_preinit();
	for (nr = 0; nr < tegra_pcie_hw.nr_controllers; nr++) {
		sys = kzalloc(sizeof(struct pci_sys_data), GFP_KERNEL);
		if (!sys)
			panic("PCI: unable to allocate sys data!");

#ifdef CONFIG_PCI_DOMAINS
		sys->domain  = tegra_pcie_hw.domain;
#endif
		sys->busnr   = nr;
		sys->swizzle = tegra_pcie_hw.swizzle;
		sys->map_irq = tegra_pcie_hw.map_irq;
		INIT_LIST_HEAD(&sys->resources);

		ret = tegra_pcie_setup(nr, sys);
		if (ret > 0) {
			if (list_empty(&sys->resources)) {
				pci_add_resource_offset(&sys->resources,
					 &ioport_resource, sys->io_offset);
				pci_add_resource_offset(&sys->resources,
					 &iomem_resource, sys->mem_offset);
			}
			pci_create_root_bus(NULL, nr, &tegra_pcie_ops,
					sys, &sys->resources);
		}
	}
	is_dock_conn_at_boot = true;
}
#endif

static void tegra_pcie_enable_aer(struct tegra_pcie_port *port, bool enable)
{
	unsigned int data;

	PR_FUNC_LINE;
	data = rp_readl(port, NV_PCIE2_RP_VEND_CTL1);
	if (enable)
		data |= PCIE2_RP_VEND_CTL1_ERPT;
	else
		data &= ~PCIE2_RP_VEND_CTL1_ERPT;
	rp_writel(port, data, NV_PCIE2_RP_VEND_CTL1);
}

static int tegra_pcie_attach(struct tegra_pcie *pcie)
{
	struct pci_bus *bus = NULL;
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	if (!hotplug_event)
		return 0;

	/* rescan and recreate all pcie data structures */
	while ((bus = pci_find_next_bus(bus)) != NULL)
		pci_rescan_bus(bus);
	/* unhide AER capability */
	list_for_each_entry(port, &pcie->ports, list)
		if (port->status)
			tegra_pcie_enable_aer(port, true);

	hotplug_event = false;
	return 0;
}

static int tegra_pcie_detach(struct tegra_pcie *pcie)
{
	struct pci_dev *pdev = NULL;
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	if (hotplug_event)
		return 0;
	hotplug_event = true;

	/* hide AER capability to avoid log spew */
	list_for_each_entry(port, &pcie->ports, list)
		if (port->status)
			tegra_pcie_enable_aer(port, false);

	/* remove all pcie data structures */
	for_each_pci_dev(pdev) {
		pci_stop_and_remove_bus_device(pdev);
		break;
	}
	return 0;
}

static void tegra_pcie_prsnt_map_override(struct tegra_pcie_port *port,
					bool prsnt)
{
	unsigned int data;

	PR_FUNC_LINE;
	/* currently only hotplug on root port 0 supported */
	data = rp_readl(port, NV_PCIE2_RP_PRIV_MISC);
	data &= ~PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	if (prsnt)
		data |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_PRSNT;
	else
		data |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	rp_writel(port, data, NV_PCIE2_RP_PRIV_MISC);
}

static void work_hotplug_handler(struct work_struct *work)
{
	struct tegra_pcie *pcie_driver =
		container_of(work, struct tegra_pcie, hotplug_detect);
	int val;

	PR_FUNC_LINE;
	if (pcie_driver->plat_data->gpio_hot_plug == -1)
		return;
	val = gpio_get_value(pcie_driver->plat_data->gpio_hot_plug);
	if (val == 0) {
		dev_info(pcie_driver->dev, "PCIE Hotplug: Connected\n");
		tegra_pcie_attach(pcie_driver);
	} else {
		dev_info(pcie_driver->dev, "PCIE Hotplug: DisConnected\n");
		tegra_pcie_detach(pcie_driver);
	}
}

static irqreturn_t gpio_pcie_detect_isr(int irq, void *arg)
{
	struct tegra_pcie *pcie = arg;
	PR_FUNC_LINE;
	schedule_work(&pcie->hotplug_detect);
	return IRQ_HANDLED;
}

static void handle_sb_intr(struct tegra_pcie *pcie)
{
	u32 mesg;

	PR_FUNC_LINE;
	/* Port 0 and 1 */
	mesg = afi_readl(pcie, AFI_MSG_0);
	if (mesg & AFI_MSG_INTX_MASK)
		/* notify device isr for INTx messages from pcie devices */
		dev_dbg(pcie->dev,
			"Legacy INTx interrupt occurred %x on port 0/1\n", mesg);
	else if (mesg & AFI_MSG_PM_PME_MASK) {
		struct tegra_pcie_port *port, *tmp;
		/* handle PME messages */
		list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
			if ((port->index == 0) && (mesg & AFI_MSG_PM_PME0))
				break;
			if ((port->index == 1) && (mesg & AFI_MSG_PM_PME1))
				break;
		}
		mesg = rp_readl(port, NV_PCIE2_RP_RSR);
		mesg |= NV_PCIE2_RP_RSR_PMESTAT;
		rp_writel(port, mesg, NV_PCIE2_RP_RSR);
	} else
		afi_writel(pcie, mesg, AFI_MSG_0);

	if (pcie->soc_data->num_ports <= 2)
		return;

	/* Port 2 */
	mesg = afi_readl(pcie, AFI_MSG_1_0);
	if (mesg & AFI_MSG_1_INTX_MASK)
		/* notify device isr for INTx messages from pcie devices */
		dev_dbg(pcie->dev,
			"Legacy INTx interrupt occurred %x on port 2\n", mesg);
	else if (mesg & AFI_MSG_1_PM_PME_MASK) {
		struct tegra_pcie_port *port, *tmp;
		/* handle PME messages */
		list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
			if ((port->index == 2) && (mesg & AFI_MSG_1_PM_PME))
				break;
		}
		mesg = rp_readl(port, NV_PCIE2_RP_RSR);
		mesg |= NV_PCIE2_RP_RSR_PMESTAT;
		rp_writel(port, mesg, NV_PCIE2_RP_RSR);
	} else
		afi_writel(pcie, mesg, AFI_MSG_1_0);
}

static irqreturn_t tegra_pcie_isr(int irq, void *arg)
{
	const char *err_msg[] = {
		"Unknown",
		"AXI slave error",
		"AXI decode error",
		"Target abort",
		"Master abort",
		"Invalid write",
		"",
		"Response decoding error",
		"AXI response decoding error",
		"Transcation timeout",
		"",
		"Slot Clock request change",
		"TMS Clock clamp change",
		"TMS power down",
		"Peer to Peer error",
	};
	struct tegra_pcie *pcie = arg;
	u32 code, signature;

	PR_FUNC_LINE;
	code = afi_readl(pcie, AFI_INTR_CODE) & AFI_INTR_CODE_MASK;

	signature = afi_readl(pcie, AFI_INTR_SIGNATURE);

	if (code == AFI_INTR_LEGACY)
		handle_sb_intr(pcie);
	afi_writel(pcie, 0, AFI_INTR_CODE);
	afi_readl(pcie, AFI_INTR_CODE); /* read pushes write */

	if (code >= ARRAY_SIZE(err_msg))
		code = 0;

	/*
	 * do not pollute kernel log with master abort reports since they
	 * happen a lot during enumeration
	 */
	if (code == AFI_INTR_MASTER_ABORT)
		dev_dbg(pcie->dev, "PCIE: %s, signature: %08x\n",
				err_msg[code], signature);
	else if ((code != AFI_INTR_LEGACY) && (code != AFI_INTR_PRSNT_SENSE))
		dev_err(pcie->dev, "PCIE: %s, signature: %08x\n",
				err_msg[code], signature);

	return IRQ_HANDLED;
}

/*
 * FPCI map is as follows:
 * - 0xfdfc000000: I/O space
 * - 0xfdfe000000: type 0 configuration space
 * - 0xfdff000000: type 1 configuration space
 * - 0xfe00000000: type 0 extended configuration space
 * - 0xfe10000000: type 1 extended configuration space
 */
static void tegra_pcie_setup_translations(struct tegra_pcie *pcie)
{
	u32 fpci_bar, size, axi_address;

	/* Bar 0: type 1 extended configuration space */
	fpci_bar = 0xfe100000;
	size = resource_size(pcie->cs);
	axi_address = pcie->cs->start;
	afi_writel(pcie, axi_address, AFI_AXI_BAR0_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR0_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR0);

	/* Bar 1: downstream IO bar */
	fpci_bar = 0xfdfc0000;
	size = resource_size(&pcie->io);
	axi_address = pcie->io.start;
	afi_writel(pcie, axi_address, AFI_AXI_BAR1_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR1_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR1);

	/* Bar 2: prefetchable memory BAR */
	fpci_bar = (((pcie->prefetch.start >> 12) & 0x0fffffff) << 4) | 0x1;
	size = resource_size(&pcie->prefetch);
	axi_address = pcie->prefetch.start;
	afi_writel(pcie, axi_address, AFI_AXI_BAR2_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR2_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR2);

	/* Bar 3: non prefetchable memory BAR */
	fpci_bar = (((pcie->mem.start >> 12) & 0x0fffffff) << 4) | 0x1;
	size = resource_size(&pcie->mem);
	axi_address = pcie->mem.start;
	afi_writel(pcie, axi_address, AFI_AXI_BAR3_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR3_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR3);

	/* NULL out the remaining BARs as they are not used */
	afi_writel(pcie, 0, AFI_AXI_BAR4_START);
	afi_writel(pcie, 0, AFI_AXI_BAR4_SZ);
	afi_writel(pcie, 0, AFI_FPCI_BAR4);

	afi_writel(pcie, 0, AFI_AXI_BAR5_START);
	afi_writel(pcie, 0, AFI_AXI_BAR5_SZ);
	afi_writel(pcie, 0, AFI_FPCI_BAR5);

	/* MSI translations are setup only when needed */
	afi_writel(pcie, 0, AFI_MSI_FPCI_BAR_ST);
	afi_writel(pcie, 0, AFI_MSI_BAR_SZ);
	afi_writel(pcie, 0, AFI_MSI_AXI_BAR_ST);
	afi_writel(pcie, 0, AFI_MSI_BAR_SZ);
}

static int tegra_pcie_get_clocks(struct tegra_pcie *pcie)
{
	int error;
	struct device *dev = pcie->dev;

	PR_FUNC_LINE;
	error = pm_clk_create(dev);
	if (error) {
		dev_err(dev, "pm_clk_create failed %d\n", error);
		return error;
	}

	error = pm_clk_add(dev, "afi");
	if (error) {
		dev_err(dev, "missing afi clock");
		return error;
	}
	error = pm_clk_add(dev, "pex");
	if (error) {
		dev_err(dev, "missing pcie clock");
		return error;
	}
	pcie->emc_bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_PCIE);
	if (!pcie->emc_bwmgr) {
		dev_err(pcie->dev, "couldn't register with EMC BwMgr\n");
		return -EINVAL;
	}
	return 0;
}

static void tegra_pcie_clocks_put(struct tegra_pcie *pcie)
{
	pm_clk_destroy(pcie->dev);
	tegra_bwmgr_set_emc(pcie->emc_bwmgr, 0, TEGRA_BWMGR_SET_EMC_FLOOR);
	tegra_bwmgr_unregister(pcie->emc_bwmgr);
}

static int tegra_pcie_port_get_phy(struct tegra_pcie_port *port)
{
	struct device *dev = port->pcie->dev;
	struct phy *phy;
	int err, i;
	char *name;

	port->phy = devm_kcalloc(dev, sizeof(phy), port->num_lanes, GFP_KERNEL);
	if (!port->phy)
		return -ENOMEM;

	for (i = 0; i < port->num_lanes; i++) {
		name = kasprintf(GFP_KERNEL, "pcie-%u", i);
		if (!name)
			return -ENOMEM;

		phy = devm_of_phy_get(dev, port->np, name);
		kfree(name);
		if (IS_ERR(phy)) {
			if (PTR_ERR(phy) == -EPROBE_DEFER)
				dev_info(dev, "PHY get deferred: %ld\n",
					 PTR_ERR(phy));
			else
				dev_err(dev, "failed to get PHY: %ld\n",
					PTR_ERR(phy));
			return PTR_ERR(phy);
		}

		err = phy_init(phy);
		if (err < 0) {
			dev_err(dev, "failed to initialize PHY: %d\n", err);
			return err;
		}

		port->phy[i] = phy;
	}

	return 0;
}

static int tegra_pcie_phys_get(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port;
	int err;

	list_for_each_entry(port, &pcie->ports, list) {
		err = tegra_pcie_port_get_phy(port);
		if (err < 0)
			return err;
	}

	return 0;
}

static int tegra_pcie_phy_power_on(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port;
	int err, i;

	list_for_each_entry(port, &pcie->ports, list) {
		for (i = 0; i < port->num_lanes; i++) {
			err = phy_power_on(port->phy[i]);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int tegra_pcie_phy_power_off(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port;
	int err, i;

	list_for_each_entry(port, &pcie->ports, list) {
		for (i = 0; i < port->num_lanes; i++) {
			err = phy_power_off(port->phy[i]);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int tegra_pcie_phy_exit(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port;
	int err = 0, i;

	list_for_each_entry(port, &pcie->ports, list) {
		for (i = 0; i < port->num_lanes; i++) {
			if (port->phy && !IS_ERR_OR_NULL(port->phy[i]))
				err = phy_exit(port->phy[i]);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int tegra_pcie_enable_pads(struct tegra_pcie *pcie, bool enable)
{
	int err = 0;

	PR_FUNC_LINE;

	if (!tegra_platform_is_silicon())
		return err;

	if (pcie->soc_data->program_uphy) {
		if (enable)
			err = tegra_pcie_phy_power_on(pcie);
		else
			err = tegra_pcie_phy_power_off(pcie);
		if (err)
			dev_err(pcie->dev, "UPHY operation failed\n");
	}

	return err;
}

static void tegra_pcie_enable_wrap(void)
{
	u32 val;
	void __iomem *msel_base;

	PR_FUNC_LINE;
#define MSELECT_CONFIG_BASE	0x50060000
#define MSELECT_CONFIG_WRAP_TO_INCR_SLAVE1	BIT(28)
#define MSELECT_CONFIG_ERR_RESP_EN_SLAVE1	BIT(24)

	/* Config MSELECT to support wrap trans for normal NC & GRE mapping */
	msel_base = ioremap(MSELECT_CONFIG_BASE, 4);
	val = readl(msel_base);
	/* Enable WRAP_TO_INCR_SLAVE1 */
	val |= MSELECT_CONFIG_WRAP_TO_INCR_SLAVE1;
	/* Disable ERR_RESP_EN_SLAVE1 */
	val &= ~MSELECT_CONFIG_ERR_RESP_EN_SLAVE1;
	writel(val, msel_base);
	iounmap(msel_base);
}

static int tegra_pcie_enable_controller(struct tegra_pcie *pcie)
{
	u32 val;
	struct tegra_pcie_port *port, *tmp;

	PR_FUNC_LINE;
	if (pcie->soc_data->enable_wrap)
		tegra_pcie_enable_wrap();
	/* Enable PLL power down */
	val = afi_readl(pcie, AFI_PLLE_CONTROL);
	val &= ~AFI_PLLE_CONTROL_BYPASS_PCIE2PLLE_CONTROL;
	val &= ~AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL;
	val |= AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN;
	val |= AFI_PLLE_CONTROL_PCIE2PLLE_CONTROL_EN;
	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		if (port->disable_clock_request) {
			val &= ~AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN;
			break;
		}
	}
	afi_writel(pcie, val, AFI_PLLE_CONTROL);

	afi_writel(pcie, 0, AFI_PEXBIAS_CTRL_0);

	/* Enable all PCIE controller and */
	/* system management configuration of PCIE crossbar */
	val = afi_readl(pcie, AFI_PCIE_CONFIG);
	val |= (AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC2_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC0_CLKREQ_AS_GPIO |
			AFI_PCIE_CONFIG_PCIEC1_CLKREQ_AS_GPIO |
			AFI_PCIE_CONFIG_PCIEC2_CLKREQ_AS_GPIO);
	list_for_each_entry(port, &pcie->ports, list) {
		val &= ~(1 << (port->index + 1));
		val &= ~(1 << (port->index + 29));
	}

	/* Configure pcie mode */
	val &= ~AFI_PCIE_CONFIG_XBAR_CONFIG_MASK;
	val |= pcie->xbar_config;

	afi_writel(pcie, val, AFI_PCIE_CONFIG);

	/* Enable Gen 2 capability of PCIE */
	val = afi_readl(pcie, AFI_FUSE) & ~AFI_FUSE_PCIE_T0_GEN2_DIS;
	afi_writel(pcie, val, AFI_FUSE);

	/* Finally enable PCIe */
	val = afi_readl(pcie, AFI_CONFIGURATION);
	val |=  (AFI_CONFIGURATION_EN_FPCI |
			AFI_CONFIGURATION_CLKEN_OVERRIDE);
	afi_writel(pcie, val, AFI_CONFIGURATION);

	val = (AFI_INTR_EN_INI_SLVERR | AFI_INTR_EN_INI_DECERR |
	       AFI_INTR_EN_TGT_SLVERR | AFI_INTR_EN_TGT_DECERR |
	       AFI_INTR_EN_TGT_WRERR | AFI_INTR_EN_DFPCI_DECERR |
	       AFI_INTR_EN_AXI_DECERR );
	afi_writel(pcie, val, AFI_AFI_INTR_ENABLE);
	afi_writel(pcie, 0xffffffff, AFI_SM_INTR_ENABLE);

	/* FIXME: No MSI for now, only INT */
	afi_writel(pcie, AFI_INTR_MASK_INT_MASK, AFI_INTR_MASK);

	/* Disable all execptions */
	afi_writel(pcie, 0, AFI_FPCI_ERROR_MASKS);

	return 0;
}

static int tegra_pcie_enable_regulators(struct tegra_pcie *pcie)
{
	int i;
	PR_FUNC_LINE;
	if (pcie->power_rails_enabled)
		return 0;

	pcie->power_rails_enabled = 1;
	dev_info(pcie->dev, "PCIE: Enable power rails\n");

	for (i = 0; i < pcie->soc_data->num_pcie_regulators; i++) {
		if (pcie->pcie_regulators[i])
			if (regulator_enable(pcie->pcie_regulators[i]))
				dev_err(pcie->dev, "%s: can't enable regulator %s\n",
				__func__,
				pcie->soc_data->pcie_regulator_names[i]);
	}

	return 0;

}

static int tegra_pcie_disable_regulators(struct tegra_pcie *pcie)
{
	int i;
	PR_FUNC_LINE;
	if (pcie->power_rails_enabled == 0)
		return 0;

	for (i = 0; i < pcie->soc_data->num_pcie_regulators; i++) {
		if (pcie->pcie_regulators[i] != NULL)
			if (regulator_disable(pcie->pcie_regulators[i]))
				dev_err(pcie->dev, "%s: can't disable regulator %s\n",
				__func__,
				pcie->soc_data->pcie_regulator_names[i]);
	}

	pcie->power_rails_enabled = 0;
	dev_info(pcie->dev, "PCIE: Disable power rails\n");
	return 0;

}

static int tegra_pcie_map_resources(struct tegra_pcie *pcie)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct resource *pads, *afi, *res;

	PR_FUNC_LINE;
	pads = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pads");

	pcie->pads_res = __devm_request_region(&pdev->dev, &iomem_resource,
			pads->start, resource_size(pads),
			"pcie-pads");

	if (!pcie->pads_res) {
		dev_err(&pdev->dev,
			"PCIE: Failed to request region for pad registers\n");
		return -EBUSY;
	}

	pcie->pads = devm_ioremap_nocache(&pdev->dev, pads->start,
						resource_size(pads));
	if (!(pcie->pads)) {
		dev_err(pcie->dev, "PCIE: Failed to map PAD registers\n");
		return -EADDRNOTAVAIL;
	}

	afi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "afi");

	pcie->afi_res = __devm_request_region(&pdev->dev, &iomem_resource,
			afi->start, resource_size(afi),
			"pcie-afi");

	if (!pcie->afi_res) {
		dev_err(&pdev->dev,
			"PCIE: Failed to request region for afi registers\n");
		return -EBUSY;
	}

	pcie->afi = devm_ioremap_nocache(&pdev->dev, afi->start,
						resource_size(afi));
	if (!(pcie->afi)) {
		dev_err(pcie->dev, "PCIE: Failed to map AFI registers\n");
		return -EADDRNOTAVAIL;
	}

	/* request configuration space, but remap later, on demand */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cs");

	pcie->cs = __devm_request_region(&pdev->dev, &iomem_resource,
			res->start, resource_size(res), "pcie-config-space");
	if (!pcie->cs) {
		dev_err(&pdev->dev, "PCIE: Failed to request region for CS registers\n");
		return -EBUSY;
	}

	return 0;
}

static void tegra_pcie_unmap_resources(struct tegra_pcie *pcie)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);

	PR_FUNC_LINE;

	if (pcie->cs)
		__devm_release_region(&pdev->dev, &iomem_resource,
				pcie->cs->start,
				resource_size(pcie->cs));
	if (pcie->afi_res)
		__devm_release_region(&pdev->dev, &iomem_resource,
				pcie->afi_res->start,
				resource_size(pcie->afi_res));
	if (pcie->pads_res)
		__devm_release_region(&pdev->dev, &iomem_resource,
				pcie->pads_res->start,
				resource_size(pcie->pads_res));

	if (pcie->pads) {
		devm_iounmap(&pdev->dev, pcie->pads);
		pcie->pads = NULL;
	}
	if (pcie->afi) {
		devm_iounmap(&pdev->dev, pcie->afi);
		pcie->afi = NULL;
	}
}

static int tegra_pcie_fpga_phy_init(struct tegra_pcie *pcie)
{
	PR_FUNC_LINE;
	/* Do reset for FPGA pcie phy */
	reset_control_deassert(pcie->afi_rst);
	afi_writel(pcie, AFI_WR_SCRATCH_0_RESET_VAL, AFI_WR_SCRATCH_0);
	udelay(10);
	afi_writel(pcie, AFI_WR_SCRATCH_0_DEFAULT_VAL, AFI_WR_SCRATCH_0);
	udelay(10);
	afi_writel(pcie, AFI_WR_SCRATCH_0_RESET_VAL, AFI_WR_SCRATCH_0);
	return 0;
}

static void tegra_pcie_pme_turnoff(struct tegra_pcie *pcie)
{
	unsigned int data;

	PR_FUNC_LINE;
	if (tegra_platform_is_fpga())
		return;
	data = afi_readl(pcie, AFI_PCIE_PME);
	data |= AFI_PCIE_PME_TURN_OFF;
	afi_writel(pcie, data, AFI_PCIE_PME);
	do {
		data = afi_readl(pcie, AFI_PCIE_PME);
	} while (!(data & AFI_PCIE_PME_ACK));

	/* Required for PLL power down */
	data = afi_readl(pcie, AFI_PLLE_CONTROL);
	data |= AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL;
	afi_writel(pcie, data, AFI_PLLE_CONTROL);
}

static int tegra_pcie_module_power_on(struct tegra_pcie *pcie)
{
	int ret;
	struct device *dev;

	PR_FUNC_LINE;

	dev = pcie->dev;

	if (pcie->pcie_power_enabled) {
		dev_info(dev, "PCIE: Already powered on");
		return 0;
	}
	pcie->pcie_power_enabled = 1;

	if (gpio_is_valid(pcie->plat_data->gpio_wake) &&
		device_may_wakeup(dev)) {
		ret = disable_irq_wake(gpio_to_irq(
			pcie->plat_data->gpio_wake));
		if (ret < 0) {
			dev_err(dev,
				"ID wake-up event failed with error %d\n", ret);
			return ret;
		}
	} else {
		ret = tegra_pcie_enable_regulators(pcie);
		if (ret) {
			dev_err(dev, "PCIE: Failed to enable regulators\n");
			return ret;
		}
	}

	return ret;
}

static int tegra_pcie_module_power_off(struct tegra_pcie *pcie);

static void tegra_pcie_config_plat(struct tegra_pcie *pcie, bool set)
{
	struct tegra_pcie_port *port;
	int count;

	list_for_each_entry(port, &pcie->ports, list) {
		for (count = 0; count < port->n_gpios; ++count)
			gpiod_set_value(gpio_to_desc(port->gpios[count]), set);
	}
}

static int tegra_pcie_restore_device(struct device *dev)
{
	int err = 0;
	struct tegra_pcie *pcie = dev_get_drvdata(dev);

	PR_FUNC_LINE;
	if (!pcie)
		return 0;

	tegra_pcie_config_plat(pcie, 1);
	pm_clk_resume(dev);
	err = tegra_pcie_module_power_on(pcie);
	if (err < 0)
		dev_err(dev, "power on failed: %d\n", err);

	if (pcie->soc_data->config_pex_io_dpd) {
		err = pinctrl_select_state(pcie->pex_pin,
					   pcie->pex_io_dpd_dis_state);
		if (err < 0) {
			dev_err(dev, "disabling pex-io-dpd failed: %d\n", err);
			goto err_exit;
		}
	}

	err = tegra_pcie_map_resources(pcie);
	if (err) {
		dev_err(dev, "PCIE: Failed to map resources\n");
		goto err_map_resource;
	}
	if (tegra_platform_is_fpga()) {
		err = tegra_pcie_fpga_phy_init(pcie);
		if (err)
			dev_err(dev, "PCIE: Failed to initialize FPGA Phy\n");
	}

	tegra_pcie_enable_pads(pcie, true);
	reset_control_deassert(pcie->afi_rst);
	tegra_pcie_enable_controller(pcie);
	tegra_pcie_request_resources(pcie);
	tegra_pcie_setup_translations(pcie);
	/* Set up MSI registers, if MSI have been enabled */
	tegra_pcie_enable_msi(pcie, true);
	reset_control_deassert(pcie->pcie_rst);
	tegra_pcie_check_ports(pcie);
	return 0;

err_map_resource:
	if (pcie->soc_data->config_pex_io_dpd) {
		err = pinctrl_select_state(pcie->pex_pin,
					   pcie->pex_io_dpd_en_state);
		if (err < 0)
			dev_err(dev, "enabling pex-io-dpd failed: %d\n", err);
	}
err_exit:
	tegra_pcie_module_power_off(pcie);
	pm_clk_suspend(dev);

	return err;
}

static int tegra_pcie_power_on(struct tegra_pcie *pcie)
{
	int err = 0;

	PR_FUNC_LINE;
	err = pm_runtime_get_sync(pcie->dev);
	if (err) {
		pm_runtime_put(pcie->dev);
		pcie->pcie_power_enabled = 0;
	}
	return err;
}

static int tegra_pcie_save_device(struct device *dev)
{
	struct tegra_pcie *pcie = dev_get_drvdata(dev);
	struct tegra_pcie_port *port;
	int err = 0;

	PR_FUNC_LINE;
	if (!pcie)
		return 0;

	if (pcie->pcie_power_enabled == 0) {
		dev_info(dev, "PCIE: Already powered off");
		return 0;
	}
	list_for_each_entry(port, &pcie->ports, list) {
		tegra_pcie_prsnt_map_override(port, false);
	}
	tegra_pcie_pme_turnoff(pcie);
	tegra_pcie_free_resources(pcie);
	tegra_pcie_enable_pads(pcie, false);
	tegra_pcie_unmap_resources(pcie);
	if (pcie->soc_data->config_pex_io_dpd) {
		err = pinctrl_select_state(pcie->pex_pin,
					   pcie->pex_io_dpd_en_state);
		if (err < 0)
			dev_err(dev, "enabling pex-io-dpd failed: %d\n", err);
	}
	reset_control_assert(pcie->pciex_rst);
	reset_control_assert(pcie->pcie_rst);
	reset_control_assert(pcie->afi_rst);

	err = tegra_pcie_module_power_off(pcie);
	if (err < 0)
		dev_err(dev, "power off failed: %d\n", err);

	pm_clk_suspend(dev);
	tegra_pcie_config_plat(pcie, 0);

	return err;
}

static int tegra_pcie_module_power_off(struct tegra_pcie *pcie)
{
	int ret;
	struct device *dev;

	PR_FUNC_LINE;

	dev = pcie->dev;
	/* configure PE_WAKE signal as wake sources */
	if (gpio_is_valid(pcie->plat_data->gpio_wake) &&
			device_may_wakeup(dev)) {
		ret = enable_irq_wake(gpio_to_irq(
		pcie->plat_data->gpio_wake));
		if (ret < 0) {
			dev_err(dev,
				"ID wake-up event failed with error %d\n", ret);
		}
	} else {
		ret = tegra_pcie_disable_regulators(pcie);
	}

	pcie->pcie_power_enabled = 0;
	return ret;
}

static int tegra_pcie_power_off(struct tegra_pcie *pcie)
{
	int err = 0;

	PR_FUNC_LINE;
	err = pm_runtime_put_sync(pcie->dev);
	if (err)
		goto err_exit;
	pcie->pcie_power_enabled = 0;
err_exit:
	return err;
}

static int tegra_pcie_get_resets(struct tegra_pcie *pcie)
{
	pcie->afi_rst = devm_reset_control_get(pcie->dev, "afi");
	if (IS_ERR(pcie->afi_rst)) {
		dev_err(pcie->dev, "PCIE : afi reset is missing\n");
		return PTR_ERR(pcie->afi_rst);
	}

	pcie->pcie_rst = devm_reset_control_get(pcie->dev, "pex");
	if (IS_ERR(pcie->pcie_rst)) {
		dev_err(pcie->dev, "PCIE : pcie reset is missing\n");
		return PTR_ERR(pcie->pcie_rst);
	}

	pcie->pciex_rst = devm_reset_control_get(pcie->dev, "pcie_x");
	if (IS_ERR(pcie->pciex_rst)) {
		dev_err(pcie->dev, "PCIE : pcie-xclk reset is missing\n");
		return PTR_ERR(pcie->pciex_rst);
	}

	return 0;
}

static int tegra_pcie_get_resources(struct tegra_pcie *pcie)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	int err;

	PR_FUNC_LINE;
	pcie->power_rails_enabled = 0;
	pcie->pcie_power_enabled = 0;

	err = tegra_pcie_get_clocks(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: failed to get clocks: %d\n", err);
		goto err_clk_get;
	}
	err = tegra_pcie_get_resets(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: failed to get resets: %d\n", err);
		goto err_reset_get;
	}
	err = tegra_pcie_enable_regulators(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to enable regulators\n");
		goto err_reset_get;
	}
	err = platform_get_irq_byname(pdev, "intr");
	if (err < 0) {
		dev_err(pcie->dev, "failed to get IRQ: %d\n", err);
		goto err_irq;
	}

	pcie->irq = err;

	err = devm_request_irq(&pdev->dev, pcie->irq, tegra_pcie_isr,
			IRQF_SHARED, "PCIE", pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to register IRQ: %d\n", err);
		goto err_irq;
	}

	return 0;

err_irq:
	tegra_pcie_disable_regulators(pcie);
err_reset_get:
	tegra_pcie_clocks_put(pcie);
err_clk_get:
	return err;
}

static void tegra_pcie_release_resources(struct tegra_pcie *pcie)
{
	devm_free_irq(pcie->dev, pcie->irq, pcie);
	tegra_pcie_clocks_put(pcie);
}

static void tegra_pcie_port_reset(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	PR_FUNC_LINE;

	/* pulse reset signal */
	/* assert PEX_RST_A */
	if (gpio_is_valid(port->rst_gpio)) {
		gpio_set_value(port->rst_gpio, 0);
	} else {
		value = afi_readl(port->pcie, ctrl);
		value &= ~AFI_PEX_CTRL_RST;
		afi_writel(port->pcie, value, ctrl);
	}

	usleep_range(1000, 2000);

	/* deAssert PEX_RST_A */
	if (gpio_is_valid(port->rst_gpio)) {
		gpio_set_value(port->rst_gpio, 1);
	} else {
		value = afi_readl(port->pcie, ctrl);
		value |= AFI_PEX_CTRL_RST;
		afi_writel(port->pcie, value, ctrl);
	}
}

static void tegra_pcie_port_enable(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	PR_FUNC_LINE;

	/* enable reference clock. Enable SW override so as to allow device
	   to get enumerated. SW override will be removed after enumeration
	*/
	value = afi_readl(port->pcie, ctrl);
	value |= (AFI_PEX_CTRL_REFCLK_EN | AFI_PEX_CTRL_OVERRIDE_EN);
	/* t124 doesn't support pll power down due to RTL bug and some */
	/* platforms don't support clkreq, both needs to disable clkreq and */
	/* enable refclk override to have refclk always ON independent of EP */
	if (port->disable_clock_request)
		value |= AFI_PEX_CTRL_CLKREQ_EN;
	else
		value &= ~AFI_PEX_CTRL_CLKREQ_EN;

	afi_writel(port->pcie, value, ctrl);

	tegra_pcie_port_reset(port);

	/* On platforms where MXM is not directly connected to Tegra root port,
	 * 200 ms delay (worst case) is required after reset, to ensure linkup
	 * between PCIe switch and MXM
	 */
	if (port->has_mxm_port)
		mdelay(200);
}

static void tegra_pcie_port_disable(struct tegra_pcie_port *port)
{
	u32 data;

	PR_FUNC_LINE;
	data = afi_readl(port->pcie, AFI_PCIE_CONFIG);
	switch (port->index) {
	case 0:
		data |= (AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC0_CLKREQ_AS_GPIO);
		break;
	case 1:
		data |= (AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC1_CLKREQ_AS_GPIO);
		break;
	case 2:
		data |= (AFI_PCIE_CONFIG_PCIEC2_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC2_CLKREQ_AS_GPIO);
		break;
	}
	afi_writel(port->pcie, data, AFI_PCIE_CONFIG);
}

void tegra_pcie_port_enable_per_pdev(struct pci_dev *pdev)
{
	struct pci_dev *parent = NULL;
	struct tegra_pcie_port *port = NULL;
	struct pci_host_bridge *host = pci_find_host_bridge(pdev->bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);
	u32 rp = 0;
	u32 data;

	PR_FUNC_LINE;
	parent = pdev->bus->self;
	rp = PCI_SLOT(parent->devfn);
	list_for_each_entry(port, &pcie->ports, list)
		if (rp == port->index + 1)
			break;

	data = afi_readl(port->pcie, AFI_PCIE_CONFIG);
	if (!((AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE << port->index) & data))
		return;

	switch (port->index) {
	case 0:
		data &= ~(AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC0_CLKREQ_AS_GPIO);
		break;
	case 1:
		data &= ~(AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC1_CLKREQ_AS_GPIO);
		break;
	case 2:
		data &= ~(AFI_PCIE_CONFIG_PCIEC2_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC2_CLKREQ_AS_GPIO);
		break;
	}
	afi_writel(port->pcie, data, AFI_PCIE_CONFIG);
}
EXPORT_SYMBOL(tegra_pcie_port_enable_per_pdev);

void tegra_pcie_port_disable_per_pdev(struct pci_dev *pdev)
{
	struct pci_dev *parent = NULL;
	struct tegra_pcie_port *port = NULL;
	struct pci_host_bridge *host = pci_find_host_bridge(pdev->bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);
	u32 rp = 0;
	u32 data;

	PR_FUNC_LINE;
	parent = pdev->bus->self;
	rp = PCI_SLOT(parent->devfn);
	list_for_each_entry(port, &pcie->ports, list)
		if (rp == port->index + 1)
			break;

	data = afi_readl(port->pcie, AFI_PCIE_CONFIG);
	switch (port->index) {
	case 0:
		data |= (AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC0_CLKREQ_AS_GPIO);
		break;
	case 1:
		data |= (AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC1_CLKREQ_AS_GPIO);
		break;
	case 2:
		data |= (AFI_PCIE_CONFIG_PCIEC2_DISABLE_DEVICE |
			AFI_PCIE_CONFIG_PCIEC2_CLKREQ_AS_GPIO);
		break;
	}
	afi_writel(port->pcie, data, AFI_PCIE_CONFIG);
}
EXPORT_SYMBOL(tegra_pcie_port_disable_per_pdev);

/*
 * FIXME: If there are no PCIe cards attached, then calling this function
 * can result in the increase of the bootup time as there are big timeout
 * loops.
 */
#define TEGRA_PCIE_LINKUP_TIMEOUT	200	/* up to 1.2 seconds */
static bool tegra_pcie_port_check_link(struct tegra_pcie_port *port)
{
	struct device *dev = port->pcie->dev;
	unsigned int retries = 3;
	unsigned long value;

	/* override presence detection */
	value = readl(port->base + NV_PCIE2_RP_PRIV_MISC);
	value &= ~PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	value |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_PRSNT;
	writel(value, port->base + NV_PCIE2_RP_PRIV_MISC);

	do {
		unsigned int timeout = TEGRA_PCIE_LINKUP_TIMEOUT;

		do {
			value = readl(port->base + RP_VEND_XP);

			if (value & RP_VEND_XP_DL_UP)
				break;

			usleep_range(1000, 2000);
		} while (--timeout);

		if (!timeout) {
			dev_info(dev, "link %u down, retrying\n", port->index);
			goto retry;
		}

		timeout = TEGRA_PCIE_LINKUP_TIMEOUT;

		do {
			value = readl(port->base + RP_LINK_CONTROL_STATUS);

			if (value & RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE)
				return true;

			usleep_range(1000, 2000);
		} while (--timeout);

retry:
		tegra_pcie_port_reset(port);
	} while (--retries);

	return false;
}

static void tegra_pcie_apply_sw_war(struct tegra_pcie_port *port,
				bool enum_done)
{
	unsigned int data;
	struct tegra_pcie *pcie = port->pcie;
	struct pci_dev *pdev = NULL;

	PR_FUNC_LINE;
	if (enum_done) {
		/* disable msi for port driver to avoid panic */
		for_each_pci_dev(pdev)
			if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
				pdev->msi_enabled = 0;
	} else {

		/* Some of the old PCIe end points don't get enumerated
		 * if RP advertises both Gen-1 and Gen-2 speeds. Hence, the
		 * strategy followed here is to initially advertise only
		 * Gen-1 and after link is up, check end point's capability
		 * for Gen-2 and retrain link to Gen-2 speed
		 */
		data = rp_readl(port, RP_LINK_CONTROL_STATUS_2);
		data &= ~RP_LINK_CONTROL_STATUS_2_TRGT_LNK_SPD_MASK;
		data |= RP_LINK_CONTROL_STATUS_2_TRGT_LNK_SPD_GEN1;
		rp_writel(port, data, RP_LINK_CONTROL_STATUS_2);

		/* disable interrupts for LTR messages */
		data = rp_readl(port, PCIE2_RP_L1SS_SPARE);
		data &= ~PCIE2_RP_L1SS_SPARE_LTR_MSG_INT_EN;
		data &= ~PCIE2_RP_L1SS_SPARE_LTR_MSG_RCV_STS;
		rp_writel(port, data, PCIE2_RP_L1SS_SPARE);

		/* Avoid warning during enumeration for invalid IRQ of RP */
		data = rp_readl(port, NV_PCIE2_RP_INTR_BCR);
		data |= NV_PCIE2_RP_INTR_BCR_INTR_LINE;
		rp_writel(port, data, NV_PCIE2_RP_INTR_BCR);

		/* Power saving configuration for sleep / idle */
		data = rp_readl(port, NV_PCIE2_RP_VEND_XP_PAD_PWRDN);
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_DISABLED_EN;
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_DYNAMIC_EN;
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_L1_EN;
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_L1_CLKREQ_EN;
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_DYNAMIC_L1PP;
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_L1_L1PP;
		data |= NV_PCIE2_RP_VEND_XP_PAD_PWRDN_SLEEP_MODE_L1_CLKREQ_L1PP;
		rp_writel(port, data, NV_PCIE2_RP_VEND_XP_PAD_PWRDN);

		/* resize buffers for better perf, bug#1447522 */
		/* T210 WAR for perf bugs required when LPDDR4 */
		/* memory is used with both ctlrs in X4_X1 config */
		if (pcie->soc_data->perf_war &&
		    (pcie->xbar_config == AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X1) &&
		    (pcie->num_ports == pcie->soc_data->num_ports)) {
			struct tegra_pcie_port *temp_port;
			list_for_each_entry(temp_port, &pcie->ports, list) {
				data = rp_readl(temp_port,
							NV_PCIE2_RP_XP_CTL_1);
				data |= PCIE2_RP_XP_CTL_1_SPARE_BIT29;
				rp_writel(temp_port, data,
					NV_PCIE2_RP_XP_CTL_1);

				data = rp_readl(temp_port,
						NV_PCIE2_RP_TX_HDR_LIMIT);
				if (temp_port->index)
					data |= PCIE2_RP_TX_HDR_LIMIT_NPT_1;
				else
					data |= PCIE2_RP_TX_HDR_LIMIT_NPT_0;
				rp_writel(temp_port, data,
					NV_PCIE2_RP_TX_HDR_LIMIT);
			}
		}

		if (pcie->soc_data->l1ss_rp_wakeup_war) {
		/* Bug#1461732 WAR, set clkreq asserted delay greater than */
		/* power off time (2us) to avoid RP wakeup in L1.2_ENTRY */
			data = rp_readl(port,
					NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);
			data &= ~PCIE2_RP_L1SS_1_CYA_CLKREQ_ASSERTED_DLY_MASK;
			data |= PCIE2_RP_L1SS_1_CYA_CLKREQ_ASSERTED_DLY;
			rp_writel(port, data,
				  NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);
		}

		if (pcie->soc_data->link_speed_war) {
		/* take care of link speed change error in corner cases */
			data = rp_readl(port, NV_PCIE2_RP_VEND_CTL0);
			data &= ~PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH_MASK;
			data |= PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH;
			rp_writel(port, data, NV_PCIE2_RP_VEND_CTL0);
		}

		if (pcie->soc_data->updateFC_timer_expire_war) {
			data = rp_readl(port, RP_VEND_XP);
			data &= ~RP_VEND_XP_UPDATE_FC_THRESHOLD;
			data |= (0x60 << 18);
			rp_writel(port, data, RP_VEND_XP);
		}

		/* Do timer settings only if clk25m freq equal to 19.2 MHz */
		if (clk_get_rate(devm_clk_get(pcie->dev, "clk_m")) != 19200000)
			return;

		data = rp_readl(port, NV_PCIE2_RP_TIMEOUT0);
		data &= ~PCIE2_RP_TIMEOUT0_PAD_PWRUP_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_PWRUP;
		data &= ~PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM;
		data &= ~PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2;
		rp_writel(port, data, NV_PCIE2_RP_TIMEOUT0);

		data = rp_readl(port, NV_PCIE2_RP_TIMEOUT1);
		data &= ~PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE_MASK;
		data |= PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE;
		data &= ~PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE_MASK;
		data |= PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE;
		rp_writel(port, data, NV_PCIE2_RP_TIMEOUT1);

		data = rp_readl(port, NV_PCIE2_RP_XP_REF);
		data &= ~PCIE2_RP_XP_REF_MICROSECOND_LIMIT_MASK;
		data |= PCIE2_RP_XP_REF_MICROSECOND_LIMIT;
		data |= PCIE2_RP_XP_REF_MICROSECOND_ENABLE;
		data |= PCIE2_RP_XP_REF_CPL_TO_OVERRIDE;
		data &= ~PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE_MASK;
		data |= PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE;
		rp_writel(port, data, NV_PCIE2_RP_XP_REF);

		data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY;
		rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);

		data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY;
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND;
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP;
		rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA);

		if (pcie->soc_data->RAW_violation_war) {
			/* WAR for RAW violation on T124/T132 platforms */
			data = rp_readl(port, NV_PCIE2_RP_RX_HDR_LIMIT);
			data &= ~PCIE2_RP_RX_HDR_LIMIT_PW_MASK;
			data |= PCIE2_RP_RX_HDR_LIMIT_PW;
			rp_writel(port, data, NV_PCIE2_RP_RX_HDR_LIMIT);

			data = rp_readl(port, NV_PCIE2_RP_PRIV_XP_DL);
			data |= PCIE2_RP_PRIV_XP_DL_GEN2_UPD_FC_TSHOLD;
			rp_writel(port, data, NV_PCIE2_RP_PRIV_XP_DL);

			data = rp_readl(port, RP_VEND_XP);
			data |= RP_VEND_XP_UPDATE_FC_THRESHOLD;
			rp_writel(port, data, RP_VEND_XP);
		}
	}
}

/* Enable various features of root port */
static void tegra_pcie_enable_rp_features(struct tegra_pcie_port *port)
{
	unsigned int data;

	PR_FUNC_LINE;
	if (port->pcie->prod_list) {
		if (tegra_prod_set_by_name(
				&(port->pcie->pads),
				"prod_c_pad",
				port->pcie->prod_list)) {
			dev_dbg(port->pcie->dev,
				"pad prod settings are not found in DT\n");
		}

		if (tegra_prod_set_by_name(
				&(port->base),
				"prod_c_rp",
				port->pcie->prod_list)) {
			dev_dbg(port->pcie->dev,
				"RP prod settings are not found in DT\n");
		}
	}

	/* Optimal settings to enhance bandwidth */
	data = rp_readl(port, RP_VEND_XP);
	data |= RP_VEND_XP_OPPORTUNISTIC_ACK;
	data |= RP_VEND_XP_OPPORTUNISTIC_UPDATEFC;
	rp_writel(port, data, RP_VEND_XP);

	/* Power mangagement settings */
	/* Enable clock clamping by default and enable card detect */
	data = rp_readl(port, NV_PCIE2_RP_PRIV_MISC);
	data |= PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_ENABLE |
		PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_ENABLE;
	if (port->pcie->soc_data->update_clamp_threshold) {
		data |= PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_THRESHOLD |
			PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_THRESHOLD;
	}
	rp_writel(port, data, NV_PCIE2_RP_PRIV_MISC);

	/* Enable ASPM - L1 state support by default */
	data = rp_readl(port, NV_PCIE2_RP_VEND_XP1);
	data |= NV_PCIE2_RP_VEND_XP_LINK_PVT_CTL_L1_ASPM_SUPPORT;
	rp_writel(port, data, NV_PCIE2_RP_VEND_XP1);

	/* LTSSM wait for DLLP to finish before entering L1 or L2/L3 */
	/* to avoid truncating of PM mesgs resulting in reciever errors */
	data = rp_readl(port, NV_PCIE2_RP_VEND_XP_BIST);
	data |= PCIE2_RP_VEND_XP_BIST_GOTO_L1_L2_AFTER_DLLP_DONE;
	rp_writel(port, data, NV_PCIE2_RP_VEND_XP_BIST);

	/* unhide AER capability */
	tegra_pcie_enable_aer(port, true);

	/* Disable L1SS capability advertisement if CLKREQ is not present */
	if (port->disable_clock_request) {
		data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
		data |= PCIE2_RP_L1_PM_SUBSTATES_CYA_HIDE_CAP;
		rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
	}

	/* program timers for L1 substate support */
	/* set cm_rtime = 30us and t_pwr_on = 70us as per HW team */
	data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
	data &= ~PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_MASK;
	data |= (0x1E << PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_SHIFT);
	rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);

	data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
	data &= ~(PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_MASK |
		PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_MASK);
	data |= (1 << PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_SHIFT) |
		(7 << PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_SHIFT);
	rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);

	tegra_pcie_apply_sw_war(port, false);
}

static void tegra_pcie_update_lane_width(struct tegra_pcie_port *port)
{
	port->lanes = rp_readl(port, RP_LINK_CONTROL_STATUS);
	port->lanes = (port->lanes &
		RP_LINK_CONTROL_STATUS_NEG_LINK_WIDTH) >> 20;
}

static void tegra_pcie_update_pads2plle(struct tegra_pcie_port *port)
{
	unsigned long ctrl = 0;
	u32 val = 0;

	ctrl = tegra_pcie_port_get_pex_ctrl(port);
	/* AFI_PEX_STATUS is AFI_PEX_CTRL + 4 */
	val = afi_readl(port->pcie, ctrl + 4);
	if (val & 0x1) {
		val = afi_readl(port->pcie, AFI_PLLE_CONTROL);
		val &= ~AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN;
		afi_writel(port->pcie, val, AFI_PLLE_CONTROL);
	}
}

static void mbist_war(struct tegra_pcie *pcie, bool apply)
{
	struct tegra_pcie_port *port, *tmp;
	u32 data;

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		/* nature of MBIST bug is such that it needs to be applied
		 * only for RootPort-0 even if there are no devices
		 * connected to it */
		if (port->index == 0) {
			data = rp_readl(port, NV_PCIE2_RP_VEND_CTL2);
			if (apply)
				data |= PCIE2_RP_VEND_CTL2_PCA_ENABLE;
			else
				data &= ~PCIE2_RP_VEND_CTL2_PCA_ENABLE;
			rp_writel(port, data, NV_PCIE2_RP_VEND_CTL2);
		}
	}
}

static int tegra_pcie_mxm_pwr_init(struct tegra_pcie_port *port)
{

	mdelay(100);
	if (!(gpio_get_value(port->pwr_gd_gpio)))
		return 1;

	return 0;
}


static void tegra_pcie_check_ports(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port, *tmp;

	PR_FUNC_LINE;
	pcie->num_ports = 0;

	if (pcie->soc_data->mbist_war)
		mbist_war(pcie, true);

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		dev_info(pcie->dev, "probing port %u, using %u lanes\n",
			 port->index, port->lanes);

		tegra_pcie_port_enable(port);
		tegra_pcie_enable_rp_features(port);
		/* override presence detection */
		if (gpio_is_valid(port->gpio_presence_detection))
			tegra_pcie_prsnt_map_override(port,
				!(gpio_get_value_cansleep(
					port->gpio_presence_detection)));
		else
			tegra_pcie_prsnt_map_override(port, true);
	}

	/* Wait for clock to latch (min of 100us) */
	udelay(100);
	reset_control_deassert(pcie->pciex_rst);
	/* at this point in time, there is no end point which would
	 * take more than 20 msec for root port to detect receiver and
	 * set AUX_TX_RDET_STATUS bit. This would bring link up checking
	 * time from its current value (around 200ms) to flat 20ms
	 */
	usleep_range(19000, 21000);
	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		if (tegra_pcie_port_check_link(port)) {
			port->status = 1;
			port->ep_status = 1;
			pcie->num_ports++;
			tegra_pcie_update_lane_width(port);
			tegra_pcie_update_pads2plle(port);
			continue;
		}
		port->ep_status = 0;
		dev_info(pcie->dev, "link %u down, ignoring\n", port->index);
		tegra_pcie_port_disable(port);
	}
	/* configure all links to gen2 speed by default */
	tegra_pcie_link_speed(pcie);
	if (pcie->soc_data->mbist_war)
		mbist_war(pcie, false);
}

static int tegra_pcie_conf_gpios(struct tegra_pcie *pcie)
{
	int irq, err = 0;
	struct tegra_pcie_port *port, *tmp;

	PR_FUNC_LINE;
	if (gpio_is_valid(pcie->plat_data->gpio_hot_plug)) {
		/* configure gpio for hotplug detection */
		dev_info(pcie->dev, "acquiring hotplug_detect = %d\n",
				pcie->plat_data->gpio_hot_plug);
		err = devm_gpio_request(pcie->dev,
				pcie->plat_data->gpio_hot_plug,
				"pcie_hotplug_detect");
		if (err < 0) {
			dev_err(pcie->dev, "%s: gpio_request failed %d\n",
					__func__, err);
			return err;
		}
		err = gpio_direction_input(
				pcie->plat_data->gpio_hot_plug);
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: gpio_direction_input failed %d\n",
				__func__, err);
			return err;
		}
		irq = gpio_to_irq(pcie->plat_data->gpio_hot_plug);
		if (irq < 0) {
			dev_err(pcie->dev,
				"Unable to get irq for hotplug_detect\n");
			return err;
		}
		err = devm_request_irq(pcie->dev, (unsigned int)irq,
				gpio_pcie_detect_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"pcie_hotplug_detect",
				(void *)pcie);
		if (err < 0) {
			dev_err(pcie->dev,
				"Unable to claim irq for hotplug_detect\n");
			return err;
		}
	}
	if (gpio_is_valid(pcie->plat_data->gpio_x1_slot)) {
		err = devm_gpio_request(pcie->dev,
			pcie->plat_data->gpio_x1_slot, "pcie_x1_slot");
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_x1_slot gpio_request failed %d\n",
				__func__, err);
			return err;
		}
		err = gpio_direction_output(
			pcie->plat_data->gpio_x1_slot, 1);
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_x1_slot gpio_direction_output failed %d\n",
					__func__, err);
			return err;
		}
		gpio_set_value_cansleep(
			pcie->plat_data->gpio_x1_slot, 1);
	}
	if (gpio_is_valid(pcie->plat_data->gpio_wake)) {
		err = devm_gpio_request(pcie->dev,
				pcie->plat_data->gpio_wake, "pcie_wake");
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_wake gpio_request failed %d\n",
				__func__, err);
			return err;
		}
		err = gpio_direction_input(
				pcie->plat_data->gpio_wake);
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_wake gpio_direction_input failed %d\n",
					__func__, err);
			return err;
		}
	}

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		if (gpio_is_valid(port->gpio_presence_detection)) {
			err = devm_gpio_request_one(pcie->dev,
					port->gpio_presence_detection,
					GPIOF_DIR_IN,
					"pcie_presence_detection");
			if (err < 0) {
				dev_err(pcie->dev,
					"%s: pcie_prsnt gpio_request failed %d\n",
					__func__, err);
				return err;
			}
		}
	}
	return 0;
}

static int tegra_pcie_scale_voltage(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port, *tmp;
	struct tegra_pcie_soc_data *sd = pcie->soc_data;
	int err = 0;
	u32 data = 0;
	u32 active_lanes = 0;
	bool is_gen2 = false;

	PR_FUNC_LINE;
	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		if (!port->status)
			continue;
		data = rp_readl(port, RP_LINK_CONTROL_STATUS);
		active_lanes += ((data &
			RP_LINK_CONTROL_STATUS_NEG_LINK_WIDTH) >> 20);
		if (((data & RP_LINK_CONTROL_STATUS_LINK_SPEED) >> 16) == 2)
			is_gen2 = true;
	}

	if (sd->dvfs_mselect) {
		struct clk *mselect_clk;

		active_lanes = 0;
		dev_dbg(pcie->dev, "mselect_clk is set @ %u\n",
			sd->dvfs_tbl[active_lanes][is_gen2].afi_clk);
		mselect_clk = devm_clk_get(pcie->dev, "mselect");
		if (IS_ERR(mselect_clk)) {
			dev_err(pcie->dev, "mselect clk_get failed: %ld\n",
				PTR_ERR(mselect_clk));
			return PTR_ERR(mselect_clk);
		}
		err = clk_set_rate(mselect_clk,
				   sd->dvfs_tbl[active_lanes][is_gen2].afi_clk);
		if (err) {
			dev_err(pcie->dev,
				"setting mselect clk to %u failed : %d\n",
				sd->dvfs_tbl[active_lanes][is_gen2].afi_clk,
				err);
			return err;
		}
	}

	if (sd->dvfs_afi) {
		dev_dbg(pcie->dev, "afi_clk is set @ %u\n",
			sd->dvfs_tbl[active_lanes][is_gen2].afi_clk);
		err = clk_set_rate(devm_clk_get(pcie->dev, "afi"),
				   sd->dvfs_tbl[active_lanes][is_gen2].afi_clk);
		if (err) {
			dev_err(pcie->dev,
				"setting afi clk to %u failed : %d\n",
				sd->dvfs_tbl[active_lanes][is_gen2].afi_clk,
				err);
			return err;
		}
	}

	dev_dbg(pcie->dev, "emc_clk is set @ %u\n",
		sd->dvfs_tbl[active_lanes][is_gen2].emc_clk);
	err = tegra_bwmgr_set_emc(pcie->emc_bwmgr,
		sd->dvfs_tbl[active_lanes][is_gen2].emc_clk,
		TEGRA_BWMGR_SET_EMC_FLOOR);
	if (err < 0) {
		dev_err(pcie->dev, "setting emc clk to %u failed : %d\n",
			sd->dvfs_tbl[active_lanes][is_gen2].emc_clk, err);
		return err;
	}

	return err;
}

static void tegra_pcie_change_link_speed(struct tegra_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct tegra_pcie_port *port, *tmp;
	ktime_t deadline;
	u32 value;

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		/*
		 * "Supported Link Speeds Vector" in "Link Capabilities 2"
		 * is not supported by Tegra. tegra_pcie_change_link_speed()
		 * is called only for Tegra chips which support Gen2.
		 * So there no harm if supported link speed is not verified.
		 */
		value = readl(port->base + RP_LINK_CONTROL_STATUS_2);
		value &= ~PCI_EXP_LNKSTA_CLS;
		value |= PCI_EXP_LNKSTA_CLS_5_0GB;
		writel(value, port->base + RP_LINK_CONTROL_STATUS_2);

		/*
		 * Poll until link comes back from recovery to avoid race
		 * condition.
		 */
		deadline = ktime_add_us(ktime_get(), LINK_RETRAIN_TIMEOUT);

		while (ktime_before(ktime_get(), deadline)) {
			value = readl(port->base + RP_LINK_CONTROL_STATUS);
			if ((value & PCI_EXP_LNKSTA_LT) == 0)
				break;

			usleep_range(2000, 3000);
		}
		if (value & PCI_EXP_LNKSTA_LT)
			dev_warn(dev, "PCIe port %u link is in recovery\n",
				 port->index);

		/* Clear BW Management Status */
		value = readl(port->base + RP_LINK_CONTROL_STATUS);
		value |= RP_LINK_CONTROL_STATUS_BW_MGMT_STATUS;
		writel(value, port->base + RP_LINK_CONTROL_STATUS);

		/* Retrain the link */
		value = readl(port->base + RP_LINK_CONTROL_STATUS);
		value |= PCI_EXP_LNKCTL_RL;
		writel(value, port->base + RP_LINK_CONTROL_STATUS);

		deadline = ktime_add_us(ktime_get(), LINK_RETRAIN_TIMEOUT);

		while (ktime_before(ktime_get(), deadline)) {
			value = readl(port->base + RP_LINK_CONTROL_STATUS);
			if (value & RP_LINK_CONTROL_STATUS_BW_MGMT_STATUS)
				break;

			usleep_range(2000, 3000);
		}
		if (value & PCI_EXP_LNKSTA_LT)
			dev_err(dev, "failed to retrain link of port %u\n",
				port->index);
	}
}

static void tegra_pcie_link_speed(struct tegra_pcie *pcie)
{
	PR_FUNC_LINE;

	tegra_pcie_change_link_speed(pcie);
	tegra_pcie_scale_voltage(pcie);

	return;
}

static void tegra_pcie_enable_features(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	list_for_each_entry(port, &pcie->ports, list) {
		if (port->status)
			tegra_pcie_apply_sw_war(port, true);
	}
}
static int tegra_pcie_enable_msi(struct tegra_pcie *, bool);
static int tegra_pcie_disable_msi(struct tegra_pcie *pcie);

static int tegra_pcie_init(struct tegra_pcie *pcie)
{
	int err = 0;
	struct pci_bus *child;
	struct pci_host_bridge *host = pcie->host;

	PR_FUNC_LINE;

	INIT_WORK(&pcie->hotplug_detect, work_hotplug_handler);
	err = tegra_pcie_get_resources(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: get resources failed\n");
		return err;
	}
	err = tegra_pcie_power_on(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to power on: %d\n", err);
		goto fail_release_resource;
	}
	err = tegra_pcie_conf_gpios(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: configuring gpios failed\n");
		goto fail_power_off;
	}

	if (!pcie->num_ports) {
		dev_info(pcie->dev, "PCIE: no end points detected\n");
		err = -ENODEV;
		goto fail_power_off;
	}
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = tegra_pcie_enable_msi(pcie, false);
		if (err < 0) {
			dev_err(pcie->dev,
				"failed to enable MSI support: %d\n",
				err);
			goto fail_release_resource;
		}
	}

	pci_add_flags(PCI_REASSIGN_ALL_RSRC | PCI_REASSIGN_ALL_BUS);
	host->busnr = pcie->busn.start;
	host->dev.parent = pcie->dev;
	host->ops = &tegra_pcie_ops;

	err = pci_register_host_bridge(host);
	if (err < 0) {
		dev_err(pcie->dev, "failed to register host: %d\n", err);
		goto fail_power_off;
	}

	pci_scan_child_bus(host->bus);

	pci_fixup_irqs(pci_common_swizzle, tegra_pcie_map_irq);
	pci_bus_size_bridges(host->bus);
	pci_bus_assign_resources(host->bus);

	list_for_each_entry(child, &host->bus->children, node)
		pcie_bus_configure_settings(child);

	tegra_pcie_postinit();
	tegra_pcie_enable_features(pcie);

	pci_bus_add_devices(host->bus);

	/* register pcie device as wakeup source */
	device_init_wakeup(pcie->dev, true);

	return 0;

fail_power_off:
	tegra_pcie_power_off(pcie);
fail_release_resource:
	tegra_pcie_release_resources(pcie);
	return err;
}

/* 1:1 matching of these to the MSI vectors, 1 per bit */
/* and each mapping matches one of the available interrupts */
struct msi_map_entry {
	bool used;
	u8 index;
	int irq;
};

/* hardware supports 256 max*/
#if (INT_PCI_MSI_NR > 256)
#error "INT_PCI_MSI_NR too big"
#endif

static int tegra_msi_alloc(struct tegra_msi *chip)
{
	int msi;

	PR_FUNC_LINE;

	mutex_lock(&chip->lock);

	msi = find_first_zero_bit(chip->used, INT_PCI_MSI_NR);
	if (msi < INT_PCI_MSI_NR)
		set_bit(msi, chip->used);
	else
		msi = -ENOSPC;

	mutex_unlock(&chip->lock);

	return msi;
}

static void tegra_msi_free(struct tegra_msi *chip, unsigned long irq)
{
	struct device *dev = chip->chip.dev;

	PR_FUNC_LINE;

	mutex_lock(&chip->lock);

	if (!test_bit(irq, chip->used))
		dev_err(dev, "trying to free unused MSI#%lu\n", irq);
	else
		clear_bit(irq, chip->used);

	mutex_unlock(&chip->lock);
}


static irqreturn_t tegra_pcie_msi_irq(int irq, void *data)
{
	struct tegra_pcie *pcie = data;
	struct tegra_msi *msi = &pcie->msi;
	unsigned int i, processed = 0;

	PR_FUNC_LINE;

	for (i = 0; i < 8; i++) {
		unsigned long reg = afi_readl(pcie, AFI_MSI_VEC0_0 + i * 4);

		while (reg) {
			unsigned int offset = find_first_bit(&reg, 32);
			unsigned int index = i * 32 + offset;
			unsigned int irq_num;

			/* check if there are any interrupts in this reg */
			if (offset == 32)
				break;

			/* clear the interrupt */
			afi_writel(pcie, 1 << offset, AFI_MSI_VEC0_0 + i * 4);

			irq_num = irq_find_mapping(msi->domain, index);
			if (irq_num) {
				if (test_bit(index, msi->used))
					generic_handle_irq(irq_num);
				else
					dev_info(pcie->dev, "unhandled MSI\n");
			} else {
				/*
				 * that's weird who triggered this?
				 * just clear it
				 */
				dev_info(pcie->dev, "unexpected MSI\n");
			}

			/* see if there's any more pending in this vector */
			reg = afi_readl(pcie, AFI_MSI_VEC0_0 + i * 4);

			processed++;
		}
	}

	return processed > 0 ? IRQ_HANDLED : IRQ_NONE;
}

static int tegra_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			       struct msi_desc *desc)
{
	struct tegra_msi *msi = to_tegra_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;

	PR_FUNC_LINE;

	hwirq = tegra_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg.address_lo = lower_32_bits(msi->phys);
#ifdef CONFIG_ARM64
	msg.address_hi = upper_32_bits(msi->phys);
#else
	msg.address_hi = 0;
#endif
	msg.data = hwirq;

	write_msi_msg(irq, &msg);

	return 0;
}

static void tegra_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	struct tegra_msi *msi = to_tegra_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);

	PR_FUNC_LINE;
	tegra_msi_free(msi, d->hwirq);
}

static struct irq_chip tegra_msi_irq_chip = {
	.name = "Tegra PCIe MSI",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

static int tegra_msi_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	PR_FUNC_LINE;
	irq_set_chip_and_handler(irq, &tegra_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = tegra_msi_map,
};

static int tegra_pcie_enable_msi(struct tegra_pcie *pcie, bool no_init)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct tegra_msi *msi = &pcie->msi;
	int err;
	u32 reg;

	PR_FUNC_LINE;

	if (!msi->virt) {
		if (no_init)
			return true;

		mutex_init(&msi->lock);

		msi->chip.dev = pcie->dev;
		msi->chip.setup_irq = tegra_msi_setup_irq;
		msi->chip.teardown_irq = tegra_msi_teardown_irq;

		msi->domain = irq_domain_add_linear(pcie->dev->of_node,
			INT_PCI_MSI_NR, &msi_domain_ops, &msi->chip);
		if (!msi->domain) {
			dev_err(&pdev->dev, "failed to create IRQ domain\n");
			return -ENOMEM;
		}

		err = platform_get_irq_byname(pdev, "msi");
		if (err < 0) {
			dev_err(&pdev->dev, "failed to get IRQ: %d\n", err);
			goto free_irq_domain;
		}

		msi->irq = err;
		err = request_irq(msi->irq, tegra_pcie_msi_irq, IRQF_NO_THREAD,
				  tegra_msi_irq_chip.name, pcie);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to request IRQ: %d\n", err);
			goto free_irq_domain;
		}

		/* setup AFI/FPCI range */
		err = dma_set_coherent_mask(pcie->dev, DMA_BIT_MASK(32));
		if (err < 0) {
			dev_err(&pdev->dev, "dma_set_coherent_mask() failed: %d\n",
				err);
			goto free_irq;
		}

		msi->virt = dma_alloc_coherent(pcie->dev, PAGE_SIZE,
					       &msi->phys, GFP_KERNEL);
		if (!msi->virt) {
			dev_err(&pdev->dev, "%s: failed to alloc dma mem\n", __func__);
			err = -ENOMEM;
			goto free_irq;
		}
	}

	afi_writel(pcie, msi->phys >> 8, AFI_MSI_FPCI_BAR_ST);
	afi_writel(pcie, msi->phys, AFI_MSI_AXI_BAR_ST);
	/* this register is in 4K increments */
	afi_writel(pcie, 1, AFI_MSI_BAR_SZ);

	/* enable all MSI vectors */
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC0_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC1_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC2_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC3_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC4_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC5_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC6_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC7_0);

	/* and unmask the MSI interrupt */
	reg = afi_readl(pcie, AFI_INTR_MASK);
	reg |= AFI_INTR_MASK_MSI_MASK;
	afi_writel(pcie, reg, AFI_INTR_MASK);

	return 0;

free_irq:
	free_irq(msi->irq, pcie);
free_irq_domain:
	irq_domain_remove(msi->domain);
	return err;
}

static int tegra_pcie_disable_msi(struct tegra_pcie *pcie)
{
	struct tegra_msi *msi = &pcie->msi;
	unsigned int i, irq;
	u32 value;

	PR_FUNC_LINE;

	if (pcie->pcie_power_enabled == 0)
		return 0;

	/* mask the MSI interrupt */
	value = afi_readl(pcie, AFI_INTR_MASK);
	value &= ~AFI_INTR_MASK_MSI_MASK;
	afi_writel(pcie, value, AFI_INTR_MASK);

	/* disable all MSI vectors */
	afi_writel(pcie, 0, AFI_MSI_EN_VEC0_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC1_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC2_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC3_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC4_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC5_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC6_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC7_0);

	dma_free_coherent(pcie->dev, PAGE_SIZE, msi->virt, msi->phys);

	if (msi->irq > 0)
		free_irq(msi->irq, pcie);

	for (i = 0; i < INT_PCI_MSI_NR; i++) {
		irq = irq_find_mapping(msi->domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(msi->domain);

	return 0;
}

static void update_rp_lanes(struct tegra_pcie *pcie, u32 lanes)
{
	struct tegra_pcie_port *port = NULL;

	list_for_each_entry(port, &pcie->ports, list)
		port->lanes = (lanes >> (port->index << 3)) & 0xFF;

}

static int tegra_pcie_get_xbar_config(struct tegra_pcie *pcie, u32 lanes,
				      u32 *xbar)
{
	struct device_node *np = pcie->dev->of_node;

	if (of_device_is_compatible(np, "nvidia,tegra210b01-pcie")) {
		switch (lanes) {
		case 0x0104:
			dev_info(pcie->dev, "4x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X1;
			return 0;
		default:
			dev_info(pcie->dev, "wrong configuration updated in DT, "
				"switching to default 4x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X1;
			update_rp_lanes(pcie, 0x0104);
			return 0;
		}
	} else if (of_device_is_compatible(np, "nvidia,tegra124-pcie") ||
		of_device_is_compatible(np, "nvidia,tegra210-pcie")) {
		switch (lanes) {
		case 0x0104:
			dev_info(pcie->dev, "4x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X1;
			return 0;

		case 0x0102:
			dev_info(pcie->dev, "2x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X2_X1;
			return 0;
		default:
			dev_info(pcie->dev, "wrong configuration updated in DT, "
				"switching to default 4x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X1;
			update_rp_lanes(pcie, 0x0104);
			return 0;
		}
	} else if (of_device_is_compatible(np, "nvidia,tegra186-pcie")) {
		switch (lanes) {
		case 0x010004:
			dev_info(pcie->dev, "4x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X4_X0_X1;
			return 0;

		case 0x010102:
			dev_info(pcie->dev, "2x1, 1x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X2_X1_X1;
			return 0;

		case 0x010101:
			dev_info(pcie->dev, "1x1, 1x1, 1x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X1_X1_X1;
			return 0;
		default:
			dev_info(pcie->dev, "wrong configuration updated in DT,"
				" switching to default 2x1, 1x1, 1x1 "
				"configuration\n");
			*xbar = AFI_PCIE_CONFIG_XBAR_CONFIG_X2_X1_X1;
			update_rp_lanes(pcie, 0x010102);
			return 0;
		}
	}

	return -EINVAL;
}


static void tegra_pcie_read_plat_data(struct tegra_pcie *pcie)
{
	struct device_node *node = pcie->dev->of_node;

	PR_FUNC_LINE;
	of_property_read_u32(node, "nvidia,boot-detect-delay",
			&pcie->plat_data->boot_detect_delay);
	pcie->plat_data->gpio_hot_plug =
		of_get_named_gpio(node, "nvidia,hot-plug-gpio", 0);
	pcie->plat_data->gpio_wake =
		of_get_named_gpio(node, "nvidia,wake-gpio", 0);
	pcie->plat_data->gpio_x1_slot =
		of_get_named_gpio(node, "nvidia,x1-slot-gpio", 0);
	pcie->plat_data->has_memtype_lpddr4 =
		of_property_read_bool(node, "nvidia,has_memtype_lpddr4");
}

static char *t124_rail_names[] = {"hvdd-pex", "hvdd-pex-pll-e", "dvddio-pex",
				"avddio-pex", "avdd-pex-pll", "vddio-pex-ctl"};

static char *t210_rail_names[] = { "avdd-pll-uerefe", "hvddio-pex",
				"dvddio-pex", "dvdd-pex-pll",
				"hvdd-pex-pll-e", "vddio-pex-ctl" };

static char *t186_rail_names[] = {"vddio-pexctl-aud"};

static const struct tegra_pcie_soc_data tegra186_pcie_data = {
	.num_ports = 3,
	.pcie_regulator_names = t186_rail_names,
	.num_pcie_regulators =
			sizeof(t186_rail_names) / sizeof(t186_rail_names[0]),
	.dvfs_afi = true,
	.update_clamp_threshold = true,
	.dvfs_tbl = {
		{{0, 0}, {0, 0} },
		{{102000000, 480000000}, {102000000, 480000000} },
		{{102000000, 480000000}, {204000000, 480000000} },
		{{102000000, 480000000}, {204000000, 480000000} },
		{{204000000, 480000000}, {408000000, 480000000} },
		{{204000000, 480000000}, {408000000, 640000000} } },
};

static const struct tegra_pcie_soc_data tegra210b01_pcie_data = {
	.num_ports = 2,
	.pcie_regulator_names = t210_rail_names,
	.num_pcie_regulators =
			sizeof(t210_rail_names) / sizeof(t210_rail_names[0]),
	.program_uphy = true,
	.program_clkreq_as_bi_dir = true,
	.enable_wrap = true,
	.perf_war = true,
	.updateFC_timer_expire_war = true,
	.l1ss_rp_wakeup_war = true,
	.link_speed_war = true,
	.dvfs_mselect = true,
	.dvfs_tbl = {
		{{204000000, 102000000}, {408000000, 528000000} } },
};

static const struct tegra_pcie_soc_data tegra210_pcie_data = {
	.num_ports = 2,
	.pcie_regulator_names = t210_rail_names,
	.num_pcie_regulators =
			sizeof(t210_rail_names) / sizeof(t210_rail_names[0]),
	.config_pex_io_dpd = true,
	.program_uphy = true,
	.program_clkreq_as_bi_dir = true,
	.enable_wrap = true,
	.mbist_war = true,
	.perf_war = true,
	.updateFC_timer_expire_war = true,
	.l1ss_rp_wakeup_war = true,
	.link_speed_war = true,
	.dvfs_mselect = true,
	.update_clamp_threshold = true,
	.dvfs_tbl = {
		{{204000000, 102000000}, {408000000, 528000000} } },
};

static const struct tegra_pcie_soc_data tegra124_pcie_data = {
	.num_ports = 2,
	.pcie_regulator_names = t124_rail_names,
	.num_pcie_regulators =
			sizeof(t124_rail_names) / sizeof(t124_rail_names[0]),
	.RAW_violation_war = true,
};

static struct of_device_id tegra_pcie_of_match[] = {
	{ .compatible = "nvidia,tegra186-pcie", .data = &tegra186_pcie_data },
	{ .compatible = "nvidia,tegra210b01-pcie", .data = &tegra210b01_pcie_data },
	{ .compatible = "nvidia,tegra210-pcie", .data = &tegra210_pcie_data },
	{ .compatible = "nvidia,tegra124-pcie", .data = &tegra124_pcie_data },
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_pcie_of_match);

static int tegra_pcie_parse_dt(struct tegra_pcie *pcie)
{
	struct tegra_pcie_soc_data *soc = pcie->soc_data;
	struct device_node *np = pcie->dev->of_node, *port;
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	u32 lanes = 0;
	struct resource res = {0};
	int err;

	PR_FUNC_LINE;

	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(pcie->dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}

	for_each_of_pci_range(&parser, &range) {
		err = of_pci_range_to_resource(&range, np, &res);
		if (err < 0)
			return err;

		switch (res.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			/* Track the bus -> CPU I/O mapping offset. */
			pcie->offset.io = res.start - range.pci_addr;

			memcpy(&pcie->pio, &res, sizeof(res));
			pcie->pio.name = np->full_name;

			/*
			 * The Tegra PCIe host bridge uses this to program the
			 * mapping of the I/O space to the physical address,
			 * so we override the .start and .end fields here that
			 * of_pci_range_to_resource() converted to I/O space.
			 * We also set the IORESOURCE_MEM type to clarify that
			 * the resource is in the physical memory space.
			 */
			pcie->io.start = range.cpu_addr;
			pcie->io.end = range.cpu_addr + range.size - 1;
			pcie->io.flags = IORESOURCE_MEM;
			pcie->io.name = "I/O";

			memcpy(&res, &pcie->io, sizeof(res));
			break;

		case IORESOURCE_MEM:
			/*
			 * Track the bus -> CPU memory mapping offset. This
			 * assumes that the prefetchable and non-prefetchable
			 * regions will be the last of type IORESOURCE_MEM in
			 * the ranges property.
			 * */
			pcie->offset.mem = res.start - range.pci_addr;

			if (res.flags & IORESOURCE_PREFETCH) {
				memcpy(&pcie->prefetch, &res, sizeof(res));
				pcie->prefetch.name = "prefetchable";
			} else {
				memcpy(&pcie->mem, &res, sizeof(res));
				pcie->mem.name = "non-prefetchable";
			}
			break;
		}
	}

	err = of_pci_parse_bus_range(np, &pcie->busn);
	if (err < 0) {
		dev_err(pcie->dev, "failed to parse ranges property: %d\n",
			err);
		pcie->busn.name = np->name;
		pcie->busn.start = 0;
		pcie->busn.end = 0xff;
		pcie->busn.flags = IORESOURCE_BUS;
	}

	/* parse root ports */
	for_each_child_of_node(np, port) {
		struct tegra_pcie_port *rp;
		unsigned int index;
		u32 value;

#ifdef CONFIG_THERMAL
		if (!strncmp(port->name, "pcie-cool-dev",
			     sizeof("pcie-cool-dev")))
			pcie->is_cooling_dev = true;
#endif

		if (strncmp(port->type, "pci", sizeof("pci")))
			continue;

		err = of_pci_get_devfn(port);
		if (err < 0) {
			dev_err(pcie->dev, "failed to parse address: %d\n",
				err);
			return err;
		}

		index = PCI_SLOT(err);
		if (index < 1 || index > soc->num_ports) {
			dev_err(pcie->dev, "invalid port number: %d\n", index);
			return -EINVAL;
		}

		index--;

		err = of_property_read_u32(port, "nvidia,num-lanes", &value);
		if (err < 0) {
			dev_err(pcie->dev, "failed to parse # of lanes: %d\n",
				err);
			return err;
		}

		if (value > 16) {
			dev_err(pcie->dev, "invalid # of lanes: %u\n", value);
			return -EINVAL;
		}
		lanes |= value << (index << 3);

		if (!of_device_is_available(port)) {
			continue;
		}

		rp = devm_kzalloc(pcie->dev, sizeof(*rp), GFP_KERNEL);
		if (!rp)
			return -ENOMEM;

		err = of_address_to_resource(port, 0, &rp->regs);
		if (err < 0) {
			dev_err(pcie->dev, "failed to parse address: %d\n",
				err);
			return err;
		}

		rp->gpio_presence_detection =
			of_get_named_gpio(port,
				"nvidia,presence-detection-gpio", 0);

		INIT_LIST_HEAD(&rp->list);
		rp->index = index;
		rp->lanes = rp->num_lanes = value;
		rp->pcie = pcie;
		rp->np = port;
		rp->base = devm_ioremap_resource(pcie->dev, &rp->regs);
		if (!(rp->base))
			return -EADDRNOTAVAIL;
		rp->disable_clock_request = of_property_read_bool(port,
			"nvidia,disable-clock-request");

		rp->rst_gpio = of_get_named_gpio(port, "nvidia,rst-gpio", 0);
		if (gpio_is_valid(rp->rst_gpio)) {
			err = devm_gpio_request(pcie->dev,
						rp->rst_gpio, "pex_rst_gpio");
			if (err < 0) {
				dev_err(pcie->dev,
					"%s: pex_rst_gpio request failed %d\n",
					__func__, err);
				return err;
			}
			err = gpio_direction_output(rp->rst_gpio, 0);
			if (err < 0) {
				dev_err(pcie->dev,
					"%s: pex_rst_gpio direction_output failed %d\n",
						__func__, err);
				return err;
			}
		}

		rp->has_mxm_port = of_property_read_bool(port,
			"nvidia,has-mxm-port");
		if (rp->has_mxm_port) {
			rp->pwr_gd_gpio = of_get_named_gpio(port,
						 "nvidia,pwr-gd-gpio", 0);
			if (gpio_is_valid(rp->pwr_gd_gpio)) {
				err = devm_gpio_request(pcie->dev,
						rp->pwr_gd_gpio,
						"pwr_gd_gpio");
				if (err < 0) {
					dev_err(pcie->dev,
						"%s: pwr_gd_gpio request failed %d\n",
						__func__, err);
					return err;
				}
				err = gpio_direction_input(rp->pwr_gd_gpio);
				if (err < 0) {
					dev_err(pcie->dev,
						"%s: pwr_gd_gpio direction_input failed %d\n",
						__func__, err);
				}
			}
		}

		rp->n_gpios = of_gpio_named_count(port, "nvidia,plat-gpios");
		if (rp->n_gpios > 0) {
			int count, gpio;
			enum of_gpio_flags flags;
			unsigned long f;

			rp->gpios = devm_kzalloc(pcie->dev,
						 rp->n_gpios * sizeof(int),
						 GFP_KERNEL);
			if (!rp->gpios)
				return -ENOMEM;

			for (count = 0; count < rp->n_gpios; ++count) {
				gpio = of_get_named_gpio_flags(port,
							"nvidia,plat-gpios",
							count, &flags);
				if (!gpio_is_valid(gpio))
					return gpio;

				f = (flags & OF_GPIO_ACTIVE_LOW) ?
				    (GPIOF_OUT_INIT_LOW | GPIOF_ACTIVE_LOW) :
				    GPIOF_OUT_INIT_HIGH;

				err = devm_gpio_request_one(pcie->dev, gpio, f,
							    NULL);
				if (err < 0) {
					dev_err(pcie->dev, "gpio %d request failed\n",
						gpio);
					return err;
				}
				rp->gpios[count] = gpio;
			}
		}

		list_add_tail(&rp->list, &pcie->ports);
	}
	err = tegra_pcie_get_xbar_config(pcie, lanes, &pcie->xbar_config);
	if (err < 0) {
		dev_err(pcie->dev, "invalid lane configuration\n");
		return err;
	}

	return 0;
}

static int list_devices(struct seq_file *s, void *data)
{
	struct pci_dev *pdev = NULL;
	u16 vendor, device, devclass, speed;
	bool pass = false;
	int ret = 0;

	for_each_pci_dev(pdev) {
		pass = true;
		ret = pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor);
		if (ret) {
			pass = false;
			break;
		}
		ret = pci_read_config_word(pdev, PCI_DEVICE_ID, &device);
		if (ret) {
			pass = false;
			break;
		}
		ret = pci_read_config_word(pdev, PCI_CLASS_DEVICE, &devclass);
		if (ret) {
			pass = false;
			break;
		}
		pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &speed);

		seq_printf(s, "%s  Vendor:%04x  Device id:%04x  ",
				kobject_name(&pdev->dev.kobj), vendor,
				device);
		seq_printf(s, "Class:%04x  Speed:%s  Driver:%s(%s)\n", devclass,
			((speed & PCI_EXP_LNKSTA_CLS_5_0GB) ==
				PCI_EXP_LNKSTA_CLS_5_0GB) ?
			"Gen2" : "Gen1",
			(pdev->driver) ? "enabled" : "disabled",
			(pdev->driver) ? pdev->driver->name : NULL);
	}
	if (!pass)
		seq_printf(s, "Couldn't read devices\n");

	return ret;
}

static int apply_link_speed(struct seq_file *s, void *data)
{
	struct tegra_pcie *pcie = (struct tegra_pcie *)(s->private);

	seq_puts(s, "Changing link speed to Gen2\n");
	tegra_pcie_link_speed(pcie);
	seq_printf(s, "Done\n");
	return 0;
}

static int check_d3hot(struct seq_file *s, void *data)
{
	u16 val;
	struct pci_dev *pdev = NULL;

	/* Force all the devices (including RPs) in d3 hot state */
	for_each_pci_dev(pdev) {
		if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT ||
			pci_pcie_type(pdev) == PCI_EXP_TYPE_DOWNSTREAM)
				continue;
		/* First, keep Downstream component in D3_Hot */
		pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL,
			&val);
		if ((val & PCI_PM_CTRL_STATE_MASK) == PCI_D3hot)
			seq_printf(s, "device[%x:%x] is already in D3_hot]\n",
				pdev->vendor, pdev->device);
		val &= ~PCI_PM_CTRL_STATE_MASK;
		val |= PCI_D3hot;
		pci_write_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL,
			val);
		/* Keep corresponding upstream component in D3_Hot */
		pci_read_config_word(pdev->bus->self,
			pdev->bus->self->pm_cap + PCI_PM_CTRL, &val);
		val &= ~PCI_PM_CTRL_STATE_MASK;
		val |= PCI_D3hot;
		pci_write_config_word(pdev->bus->self,
			pdev->bus->self->pm_cap + PCI_PM_CTRL, val);
		mdelay(100);
		/* check if they have changed their state */
		pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL,
			&val);
		if ((val & PCI_PM_CTRL_STATE_MASK) == PCI_D3hot)
			seq_printf(s, "device[%x:%x] transitioned to D3_hot]\n",
				pdev->vendor, pdev->device);
		else
			seq_printf(s, "device[%x:%x] couldn't transition to D3_hot]\n",
				pdev->vendor, pdev->device);
		pci_read_config_word(pdev->bus->self,
			pdev->bus->self->pm_cap + PCI_PM_CTRL, &val);
		if ((val & PCI_PM_CTRL_STATE_MASK) == PCI_D3hot)
			seq_printf(s, "device[%x:%x] transitioned to D3_hot]\n",
				pdev->bus->self->vendor,
				pdev->bus->self->device);
		else
			seq_printf(s, "device[%x:%x] couldn't transition to D3_hot]\n",
				pdev->bus->self->vendor,
				pdev->bus->self->device);
	}

	return 0;
}

static int dump_config_space(struct seq_file *s, void *data)
{
	u8 val;
	int row, col;
	struct pci_dev *pdev = NULL;

	for_each_pci_dev(pdev) {
		int row_cnt = pci_is_pcie(pdev) ?
			PCI_EXT_CFG_SPACE_SIZE : PCI_CFG_SPACE_SIZE;
		seq_printf(s, "%s\n", kobject_name(&pdev->dev.kobj));
		seq_printf(s, "%s\n", "------------");

		for (row = 0; row < (row_cnt / 16); row++) {
			seq_printf(s, "%02x: ", (row * 16));
			for (col = 0; col < 16; col++) {
				pci_read_config_byte(pdev, ((row * 16) + col),
					&val);
				seq_printf(s, "%02x ", val);
			}
			seq_printf(s, "\n");
		}
	}
	return 0;
}

static int dump_afi_space(struct seq_file *s, void *data)
{
	u32 val, offset;
	struct tegra_pcie_port *port = NULL;
	struct tegra_pcie *pcie = (struct tegra_pcie *)(s->private);

	list_for_each_entry(port, &pcie->ports, list) {
		seq_puts(s, "Offset:  Values\n");
		for (offset = 0; offset < 0x200; offset += 0x10) {
			val = afi_readl(port->pcie, offset);
			seq_printf(s, "%6x: %8x %8x %8x %8x\n", offset,
				afi_readl(port->pcie, offset),
				afi_readl(port->pcie, offset + 4),
				afi_readl(port->pcie, offset + 8),
				afi_readl(port->pcie, offset + 12));
		}
	}
	return 0;
}

static int config_read(struct seq_file *s, void *data)
{
	struct pci_dev *pdev = NULL;

	pdev = pci_get_bus_and_slot((bdf >> 8), (bdf & 0xFF));
	if (!pdev) {
		seq_printf(s, "%02d:%02d.%02d : Doesn't exist\n",
			(bdf >> 8), PCI_SLOT(bdf), PCI_FUNC(bdf));
		seq_printf(s,
			"Enter (bus<<8 | dev<<3 | func) value to bdf file\n");
		goto end;
	}
	if (config_offset >= PCI_EXT_CFG_SPACE_SIZE) {
		seq_printf(s, "Config offset exceeds max (i.e %d) value\n",
			PCI_EXT_CFG_SPACE_SIZE);
	}
	if (!(config_offset & 0x3)) {
		u32 val;
		/* read 32 */
		pci_read_config_dword(pdev, config_offset, &val);
		seq_printf(s, "%08x\n", val);
		config_val = val;
	} else if (!(config_offset & 0x1)) {
		u16 val;
		/* read 16 */
		pci_read_config_word(pdev, config_offset, &val);
		seq_printf(s, "%04x\n", val);
		config_val = val;
	} else {
		u8 val;
		/* read 8 */
		pci_read_config_byte(pdev, config_offset, &val);
		seq_printf(s, "%02x\n", val);
		config_val = val;
	}

end:
	return 0;
}

static int config_write(struct seq_file *s, void *data)
{
	struct pci_dev *pdev = NULL;

	pdev = pci_get_bus_and_slot((bdf >> 8), (bdf & 0xFF));
	if (!pdev) {
		seq_printf(s, "%02d:%02d.%02d : Doesn't exist\n",
			(bdf >> 8), PCI_SLOT(bdf), PCI_FUNC(bdf));
		seq_printf(s,
			"Enter (bus<<8 | dev<<3 | func) value to bdf file\n");
		goto end;
	}
	if (config_offset >= PCI_EXT_CFG_SPACE_SIZE) {
		seq_printf(s, "Config offset exceeds max (i.e %d) value\n",
			PCI_EXT_CFG_SPACE_SIZE);
	}
	if (!(config_offset & 0x3)) {
		/* write 32 */
		pci_write_config_dword(pdev, config_offset, config_val);
	} else if (!(config_offset & 0x1)) {
		/* write 16 */
		pci_write_config_word(pdev, config_offset,
			(u16)(config_val & 0xFFFF));
	} else {
		/* write 8 */
		pci_write_config_byte(pdev, config_offset,
			(u8)(config_val & 0xFF));
	}

end:
	return 0;
}

static int power_down(struct seq_file *s, void *data)
{
	struct tegra_pcie_port *port = NULL;
	struct tegra_pcie *pcie = (struct tegra_pcie *)(s->private);
	u32 val;
	bool pass = false;

	val = afi_readl(pcie, AFI_PCIE_PME);
	val |= AFI_PCIE_PME_TURN_OFF;
	afi_writel(pcie, val, AFI_PCIE_PME);
	do {
		val = afi_readl(pcie, AFI_PCIE_PME);
	} while(!(val & AFI_PCIE_PME_ACK));

	mdelay(1000);
	list_for_each_entry(port, &pcie->ports, list) {
		val = rp_readl(port, NV_PCIE2_RP_LTSSM_DBGREG);
		if (val & PCIE2_RP_LTSSM_DBGREG_LINKFSM16) {
			pass = true;
			goto out;
		}
	}

out:
	if (pass)
		seq_printf(s, "[pass: pcie_power_down]\n");
	else
		seq_printf(s, "[fail: pcie_power_down]\n");
	pr_info("PCIE power_down test END..\n");
	return 0;
}

static int loopback(struct seq_file *s, void *data)
{
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);
	unsigned int new, i, val;

	new = rp_readl(port, RP_LINK_CONTROL_STATUS);

	if (!(new & RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE)) {
		pr_info("PCIE port %d not active\n", port->index);
		return -EINVAL;
	}

	/* trigger trace ram on loopback states */
	val = LTSSM_TRACE_CONTROL_CLEAR_STORE_EN |
		LTSSM_TRACE_CONTROL_TRIG_ON_EVENT |
		(0x08 << LTSSM_TRACE_CONTROL_TRIG_LTSSM_MAJOR_OFFSET) |
		(0x00 << LTSSM_TRACE_CONTROL_TRIG_PTX_LTSSM_MINOR_OFFSET) |
		(0x00 << LTSSM_TRACE_CONTROL_TRIG_PRX_LTSSM_MAJOR_OFFSET);
	rp_writel(port, val, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);

	/* clear trace ram */
	val = rp_readl(port, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);
	val |= LTSSM_TRACE_CONTROL_CLEAR_RAM;
	rp_writel(port, val, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);
	val &= ~LTSSM_TRACE_CONTROL_CLEAR_RAM;
	rp_writel(port, val, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);

	/* reset and clear status */
	port->loopback_stat = 0;

	new = rp_readl(port, RP_VEND_XP);
	new &= ~RP_VEND_XP_PRBS_EN;
	rp_writel(port, new, RP_VEND_XP);

	new = rp_readl(port, NV_PCIE2_RP_XP_CTL_1);
	new &= ~PCIE2_RP_XP_CTL_1_OLD_IOBIST_EN_BIT25;
	rp_writel(port, new, NV_PCIE2_RP_XP_CTL_1);

	rp_writel(port, 0x10000001, NV_PCIE2_RP_VEND_XP_BIST);
	rp_writel(port, 0, NV_PCIE2_RP_PRBS);

	mdelay(1);

	rp_writel(port, 0x90820001, NV_PCIE2_RP_VEND_XP_BIST);
	new = rp_readl(port, NV_PCIE2_RP_VEND_XP_BIST);

	new = rp_readl(port, NV_PCIE2_RP_XP_CTL_1);
	new |= PCIE2_RP_XP_CTL_1_OLD_IOBIST_EN_BIT25;
	rp_writel(port, new, NV_PCIE2_RP_XP_CTL_1);

	new = rp_readl(port, RP_VEND_XP);
	new |= RP_VEND_XP_PRBS_EN;
	rp_writel(port, new, RP_VEND_XP);

	mdelay(1000);

	new = rp_readl(port, RP_VEND_XP);
	port->loopback_stat = (new & RP_VEND_XP_PRBS_STAT) >> 2;
	pr_info("--- loopback status ---\n");
	for (i = 0; i < port->lanes; ++i)
		pr_info("@lane %d: %s\n", i,
			(port->loopback_stat & 0x01 << i) ? "pass" : "fail");

	new = rp_readl(port, NV_PCIE2_RP_PRBS);
	pr_info("--- PRBS pattern locked ---\n");
	for (i = 0; i < port->lanes; ++i)
		pr_info("@lane %d: %s\n", i,
			(new >> 16 & 0x01 << i) ? "Y" : "N");
	pr_info("--- err overflow bits ---\n");
	for (i = 0; i < port->lanes; ++i)
		pr_info("@lane %d: %s\n", i,
			((new & 0xffff) & 0x01 << i) ? "Y" : "N");

	new = rp_readl(port, NV_PCIE2_RP_XP_CTL_1);
	new &= ~PCIE2_RP_XP_CTL_1_OLD_IOBIST_EN_BIT25;
	rp_writel(port, new, NV_PCIE2_RP_XP_CTL_1);

	pr_info("--- err counts ---\n");
	for (i = 0; i < port->lanes; ++i) {
		rp_writel(port, i, NV_PCIE2_RP_LANE_PRBS_ERR_COUNT);
		new = rp_readl(port, NV_PCIE2_RP_LANE_PRBS_ERR_COUNT);
		pr_info("@lane %d: %u\n", i, new >> 16);
	}

	rp_writel(port, 0x90000001, NV_PCIE2_RP_VEND_XP_BIST);

	new = rp_readl(port, RP_VEND_XP);
	new &= ~RP_VEND_XP_PRBS_EN;
	rp_writel(port, new, RP_VEND_XP);

	mdelay(1);

	rp_writel(port, 0x92000001, NV_PCIE2_RP_VEND_XP_BIST);
	rp_writel(port, 0x90000001, NV_PCIE2_RP_VEND_XP_BIST);
	pr_info("pcie loopback test is done\n");

	return 0;
}

static int apply_lane_width(struct seq_file *s, void *data)
{
	unsigned int new;
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);

	if (port->lanes > 0x10) {
		seq_printf(s, "link width cannot be grater than 16\n");
		new = rp_readl(port, RP_LINK_CONTROL_STATUS);
		port->lanes = (new &
			RP_LINK_CONTROL_STATUS_NEG_LINK_WIDTH) >> 20;
		return 0;
	}
	new = rp_readl(port, NV_PCIE2_RP_VEND_XP1);
	new &= ~NV_PCIE2_RP_VEND_XP1_RNCTRL_MAXWIDTH_MASK;
	new |= port->lanes | NV_PCIE2_RP_VEND_XP1_RNCTRL_EN;
	rp_writel(port, new, NV_PCIE2_RP_VEND_XP1);
	mdelay(1);

	new = rp_readl(port, RP_LINK_CONTROL_STATUS);
	new = (new & RP_LINK_CONTROL_STATUS_NEG_LINK_WIDTH) >> 20;
	if (new != port->lanes)
		seq_printf(s, "can't set link width %u, falling back to %u\n",
			port->lanes, new);
	else
		seq_printf(s, "lane width %d applied\n", new);
	port->lanes = new;
	return 0;
}

static int aspm_state_cnt(struct seq_file *s, void *data)
{
	u32 val, cs;
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);

	cs = rp_readl(port, RP_LINK_CONTROL_STATUS);
	/* check if L0s is enabled on this port */
	if (cs & RP_LINK_CONTROL_STATUS_L0s_ENABLED) {
		val = rp_readl(port, NV_PCIE2_RP_PRIV_XP_TX_L0S_ENTRY_COUNT);
		seq_printf(s, "Tx L0s entry count : %u\n", val);
	} else
		seq_printf(s, "Tx L0s entry count : %s\n", "disabled");

	val = rp_readl(port, NV_PCIE2_RP_PRIV_XP_RX_L0S_ENTRY_COUNT);
	seq_printf(s, "Rx L0s entry count : %u\n", val);

	/* check if L1 is enabled on this port */
	if (cs & RP_LINK_CONTROL_STATUS_L1_ENABLED) {
		val = rp_readl(port, NV_PCIE2_RP_PRIV_XP_TX_L1_ENTRY_COUNT);
		seq_printf(s, "Link L1 entry count : %u\n", val);
	} else
		seq_printf(s, "Link L1 entry count : %s\n", "disabled");

	cs = rp_readl(port, PCIE2_RP_L1_PM_SS_CONTROL);
	/* RESETting the count value is not possible by any means
		because of HW Bug : 200034278 */
	/*	check if L1.1 is enabled */
	if (cs & PCIE2_RP_L1_PM_SS_CONTROL_ASPM_L11_ENABLE) {
		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		val |= PCIE2_RP_L1_1_ENTRY_COUNT_RESET;
		rp_writel(port, val, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		seq_printf(s, "Link L1.1 entry count : %u\n", (val & 0xFFFF));
	} else
		seq_printf(s, "Link L1.1 entry count : %s\n", "disabled");
	/*	check if L1.2 is enabled */
	if (cs & PCIE2_RP_L1_PM_SS_CONTROL_ASPM_L12_ENABLE) {
		val = rp_readl(port, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
		val |= PCIE2_RP_L1_2_ENTRY_COUNT_RESET;
		rp_writel(port, val, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
		seq_printf(s, "Link L1.2 entry count : %u\n", (val & 0xFFFF));
	} else
		seq_printf(s, "Link L1.2 entry count : %s\n", "disabled");

	return 0;
}

static char *aspm_states[] = {
	"Tx-L0s",
	"Rx-L0s",
	"L1",
	"IDLE ((Tx-L0s && Rx-L0s) + L1)"
};

static int list_aspm_states(struct seq_file *s, void *data)
{
	u32 i = 0;
	seq_printf(s, "----------------------------------------------------\n");
	seq_printf(s, "Note: Duration of link's residency is calcualated\n");
	seq_printf(s, "      only for one of the ASPM states at a time\n");
	seq_printf(s, "----------------------------------------------------\n");
	seq_printf(s, "write(echo) number from below table corresponding to\n");
	seq_printf(s, "one of the ASPM states for which link duration needs\n");
	seq_printf(s, "to be calculated to 'config_aspm_state'\n");
	seq_printf(s, "-----------------\n");
	for (i = 0; i < ARRAY_SIZE(aspm_states); i++)
		seq_printf(s, "%d : %s\n", i, aspm_states[i]);
	seq_printf(s, "-----------------\n");
	return 0;
}

static int apply_aspm_state(struct seq_file *s, void *data)
{
	u32 val;
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);

	if (config_aspm_state >= ARRAY_SIZE(aspm_states)) {
		seq_printf(s, "Invalid ASPM state : %u\n", config_aspm_state);
		list_aspm_states(s, data);
	} else {
		val = rp_readl(port, NV_PCIE2_RP_PRIV_XP_CONFIG);
		val &= ~NV_PCIE2_RP_PRIV_XP_CONFIG_LOW_PWR_DURATION_MASK;
		val |= config_aspm_state;
		rp_writel(port, val, NV_PCIE2_RP_PRIV_XP_CONFIG);
		seq_printf(s, "Configured for ASPM-%s state...\n",
			aspm_states[config_aspm_state]);
	}
	return 0;
}

static int get_aspm_duration(struct seq_file *s, void *data)
{
	u32 val;
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);

	val = rp_readl(port, NV_PCIE2_RP_PRIV_XP_DURATION_IN_LOW_PWR_100NS);
	/* 52.08 = 1000 / 19.2MHz is rounded to 52	*/
	seq_printf(s, "ASPM-%s duration = %d ns\n",
		aspm_states[config_aspm_state], (u32)((val * 100)/52));
	return 0;
}

static int secondary_bus_reset(struct seq_file *s, void *data)
{
	u32 val;
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);

	val = rp_readl(port, NV_PCIE2_RP_INTR_BCR);
	val |= NV_PCIE2_RP_INTR_BCR_SB_RESET;
	rp_writel(port, val, NV_PCIE2_RP_INTR_BCR);
	udelay(10);
	val = rp_readl(port, NV_PCIE2_RP_INTR_BCR);
	val &= ~NV_PCIE2_RP_INTR_BCR_SB_RESET;
	rp_writel(port, val, NV_PCIE2_RP_INTR_BCR);

	seq_printf(s, "Secondary Bus Reset applied successfully...\n");
	return 0;
}

static void reset_l1ss_counter(struct tegra_pcie_port *port, u32 val,
			unsigned long offset)
{
	int c = 0;

	if ((val & 0xFFFF) == 0xFFFF) {
		pr_info(" Trying reset L1ss entry count to 0\n");
		while (val) {
			if (c++ > 50) {
				pr_info("Timeout: reset did not happen!\n");
				break;
			}
			val |= PCIE2_RP_L1_1_ENTRY_COUNT_RESET;
			rp_writel(port, val, offset);
			mdelay(1);
			val = rp_readl(port, offset);
		}
		if (!val)
			pr_info("L1ss entry count reset to 0\n");
	}
}
static int aspm_l11(struct seq_file *s, void *data)
{
	struct pci_dev *pdev = NULL;
	u32 val = 0, pos = 0;
	struct tegra_pcie_port *port = NULL;
	struct tegra_pcie *pcie = (struct tegra_pcie *)(s->private);

	pr_info("\nPCIE aspm l1.1 test START..\n");
	list_for_each_entry(port, &pcie->ports, list) {
		/* reset RP L1.1 counter */
		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		val |= PCIE2_RP_L1_1_ENTRY_COUNT_RESET;
		rp_writel(port, val, NV_PCIE2_RP_L1_1_ENTRY_COUNT);

		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		pr_info("L1.1 Entry count before %x\n", val);
		reset_l1ss_counter(port, val, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
	}
	/* disable automatic l1ss exit by gpu */
	for_each_pci_dev(pdev)
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT) {
			pci_write_config_dword(pdev, 0x658, 0);
			pci_write_config_dword(pdev, 0x150, 0xE0000015);
		}
	for_each_pci_dev(pdev) {
		u16 aspm;
		pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &aspm);
		aspm |= PCI_EXP_LNKCTL_ASPM_L1;
		pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, aspm);
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &val);
		val &= ~PCI_L1SS_CAP_L1PM_MASK;
		val |= PCI_L1SS_CTRL1_ASPM_L11S;
		pci_write_config_dword(pdev, pos + PCI_L1SS_CTRL1, val);
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
	mdelay(2000);
	for_each_pci_dev(pdev) {
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &val);
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
	list_for_each_entry(port, &pcie->ports, list) {
		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		pr_info("L1.1 Entry count after %x\n", val);
	}

	pr_info("PCIE aspm l1.1 test END..\n");
	return 0;
}

static int aspm_l1ss(struct seq_file *s, void *data)
{
	struct pci_dev *pdev = NULL;
	u32 val = 0, pos = 0;
	struct tegra_pcie_port *port = NULL;
	struct tegra_pcie *pcie = (struct tegra_pcie *)(s->private);

	pr_info("\nPCIE aspm l1ss test START..\n");
	list_for_each_entry(port, &pcie->ports, list) {
		/* reset RP L1.1 L1.2 counters */
		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		val |= PCIE2_RP_L1_1_ENTRY_COUNT_RESET;
		rp_writel(port, val, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		pr_info("L1.1 Entry count before %x\n", val);
		reset_l1ss_counter(port, val, NV_PCIE2_RP_L1_1_ENTRY_COUNT);

		val = rp_readl(port, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
		val |= PCIE2_RP_L1_2_ENTRY_COUNT_RESET;
		rp_writel(port, val, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
		val = rp_readl(port, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
		pr_info("L1.2 Entry count before %x\n", val);
		reset_l1ss_counter(port, val, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
	}
	/* disable automatic l1ss exit by gpu */
	for_each_pci_dev(pdev)
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT) {
			pci_write_config_dword(pdev, 0x658, 0);
			pci_write_config_dword(pdev, 0x150, 0xE0000015);
		}

	for_each_pci_dev(pdev) {
		u16 aspm;
		pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &aspm);
		aspm |= PCI_EXP_LNKCTL_ASPM_L1;
		pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, aspm);
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &val);
		val &= ~PCI_L1SS_CAP_L1PM_MASK;
		val |= (PCI_L1SS_CTRL1_ASPM_L11S | PCI_L1SS_CTRL1_ASPM_L12S);
		pci_write_config_dword(pdev, pos + PCI_L1SS_CTRL1, val);
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
	mdelay(2000);
	for_each_pci_dev(pdev) {
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &val);
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
	list_for_each_entry(port, &pcie->ports, list) {
		u32 ltr_val;
		val = rp_readl(port, NV_PCIE2_RP_L1_1_ENTRY_COUNT);
		pr_info("L1.1 Entry count after %x\n", val);
		val = rp_readl(port, NV_PCIE2_RP_L1_2_ENTRY_COUNT);
		pr_info("L1.2 Entry count after %x\n", val);

		val = rp_readl(port, NV_PCIE2_RP_LTR_REP_VAL);
		pr_info("LTR reproted by EP %x\n", val);
		ltr_val = (val & 0x1FF) * (1 << (5 * ((val & 0x1C00) >> 10)));
		if (ltr_val > (106 * 1000)) {
			pr_info("EP's LTR = %u ns is > RP's threshold = %u ns\n",
					ltr_val, 106 * 1000);
			pr_info("Hence only L1.2 entry allowed\n");
		} else {
			pr_info("EP's LTR = %u ns is < RP's threshold = %u ns\n",
					ltr_val, 106 * 1000);
			pr_info("Hence only L1.1 entry allowed\n");
		}
	}

	pr_info("PCIE aspm l1ss test END..\n");
	return 0;
}

struct ltssm_major_state {
	const char *name;
	const char *minor[8];
};

struct ltssm_state {
	struct ltssm_major_state major[12];
};

static struct ltssm_state ltssm_state = {
	.major[0]  = {"detect", {"quiet", "active", "retry", "wait", "entry"}},
	.major[1]  = {"polling", {"active", "config", "idle", NULL, "compliance", "cspeed"}},
	.major[2]  = {"config", {"link start", "link accept", "lane accept", "lane wait", "idle", "pwrup", "complete"}},
	.major[3]  = {NULL, {NULL}},
	.major[4]  = {"l0", {"normal", "l0s entry", "l0s idle", "l0s wait", "l0s fts", "pwrup"}},
	.major[5]  = {"l1", {"entry", "waitrx", "idle", "wait", "pwrup", "beacon entry", "beacon exit"}},
	.major[6]  = {"l2", {"entry", "waitrx", "transmitwake", "idle"}},
	.major[7]  = {"recovery", {"rcvrlock", "rcvrcfg", "speed", "idle", NULL, NULL, NULL, "finish pkt"}},
	.major[8]  = {"loopback", {"entry", "active", "idle", "exit", "speed", "pre speed"}},
	.major[9]  = {"hotreset", {NULL}},
	.major[10] = {"disabled", {NULL}},
	.major[11] = {"txchar", {NULL}},
};

static const char *ltssm_get_major(unsigned int major)
{
	const char *state;

	state = ltssm_state.major[major].name;
	if (!state)
		return "unknown";
	return state;
}

static const char *ltssm_get_minor(unsigned int major, unsigned int minor)
{
	const char *state;

	state = ltssm_state.major[major].minor[minor];
	if (!state)
		return "unknown";
	return state;
}

static int dump_ltssm_trace(struct seq_file *s, void *data)
{
	struct tegra_pcie_port *port = (struct tegra_pcie_port *)(s->private);
	unsigned int val, ridx, widx, entries;

	seq_printf(s, "LTSSM trace dump:\n");
	val = rp_readl(port, NV_PCIE2_RP_LTSSM_TRACE_STATUS);
	widx = LTSSM_TRACE_STATUS_WRITE_POINTER(val);
	entries = LTSSM_TRACE_STATUS_RAM_FULL(val) ? 32 : widx;
	seq_printf(s, "LTSSM trace dump - %d entries:\n", entries);
	for (ridx = 0; ridx < entries; ridx++) {
		val = LTSSM_TRACE_STATUS_READ_ADDR(ridx);
		rp_writel(port, val, NV_PCIE2_RP_LTSSM_TRACE_STATUS);
		val = rp_readl(port, NV_PCIE2_RP_LTSSM_TRACE_STATUS);

		seq_printf(s, "  [0x%08x] major: %-10s minor_tx: %-15s minor_rx: %s\n", val,
			ltssm_get_major(LTSSM_TRACE_STATUS_MAJOR(val)),
			ltssm_get_minor(LTSSM_TRACE_STATUS_MAJOR(val),  LTSSM_TRACE_STATUS_PTX_MINOR(val)),
			ltssm_get_minor(LTSSM_TRACE_STATUS_MAJOR(val),  LTSSM_TRACE_STATUS_PRX_MINOR(val)));
	}
	/* clear trace ram */
	val = rp_readl(port, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);
	val |= LTSSM_TRACE_CONTROL_CLEAR_RAM;
	rp_writel(port, val, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);
	val &= ~LTSSM_TRACE_CONTROL_CLEAR_RAM;
	rp_writel(port, val, NV_PCIE2_RP_LTSSM_TRACE_CONTROL);

	return 0;
}

static struct dentry *create_tegra_pcie_debufs_file(char *name,
		const struct file_operations *ops,
		struct dentry *parent,
		void *data)
{
	struct dentry *d;

	d = debugfs_create_file(name, S_IRUGO, parent, data, ops);
	if (!d)
		debugfs_remove_recursive(parent);

	return d;
}

#define DEFINE_ENTRY(__name)	\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __name, inode->i_private); \
}									\
static const struct file_operations __name ## _fops = {	\
	.open		= __name ## _open,	\
	.read		= seq_read,	\
	.llseek		= seq_lseek,	\
	.release	= single_release,	\
};

/* common */
DEFINE_ENTRY(list_devices)
DEFINE_ENTRY(apply_link_speed)
DEFINE_ENTRY(check_d3hot)
DEFINE_ENTRY(dump_config_space)
DEFINE_ENTRY(dump_afi_space)
DEFINE_ENTRY(config_read)
DEFINE_ENTRY(config_write)
DEFINE_ENTRY(aspm_l11)
DEFINE_ENTRY(aspm_l1ss)
DEFINE_ENTRY(power_down)

/* Port specific */
DEFINE_ENTRY(loopback)
DEFINE_ENTRY(apply_lane_width)
DEFINE_ENTRY(aspm_state_cnt)
DEFINE_ENTRY(list_aspm_states)
DEFINE_ENTRY(apply_aspm_state)
DEFINE_ENTRY(get_aspm_duration)
DEFINE_ENTRY(secondary_bus_reset)
DEFINE_ENTRY(dump_ltssm_trace)

static int tegra_pcie_port_debugfs_init(struct tegra_pcie_port *port)
{
	struct dentry *d;
	char port_name[2] = {0};

	snprintf(port_name, sizeof(port_name), "%d", port->index);
	port->port_debugfs = debugfs_create_dir(port_name,
							port->pcie->debugfs);
	if (!port->port_debugfs)
		return -ENOMEM;

	d = debugfs_create_u32("lane_width", S_IWUGO | S_IRUGO,
					port->port_debugfs,
					&(port->lanes));
	if (!d)
		goto remove;

	d = debugfs_create_x32("loopback_status", S_IWUGO | S_IRUGO,
					port->port_debugfs,
					&(port->loopback_stat));
	if (!d)
		goto remove;

	d = debugfs_create_file("loopback", S_IRUGO,
					port->port_debugfs, (void *)port,
					&loopback_fops);
	if (!d)
		goto remove;

	d = debugfs_create_file("apply_lane_width", S_IRUGO,
					port->port_debugfs, (void *)port,
					&apply_lane_width_fops);
	if (!d)
		goto remove;

	d = debugfs_create_file("aspm_state_cnt", S_IRUGO,
					port->port_debugfs, (void *)port,
					&aspm_state_cnt_fops);
	if (!d)
		goto remove;

	d = debugfs_create_u16("config_aspm_state", S_IWUGO | S_IRUGO,
					port->port_debugfs,
					&config_aspm_state);
	if (!d)
		goto remove;

	d = debugfs_create_file("apply_aspm_state", S_IRUGO,
					port->port_debugfs, (void *)port,
					&apply_aspm_state_fops);
	if (!d)
		goto remove;

	d = debugfs_create_file("list_aspm_states", S_IRUGO,
					port->port_debugfs, (void *)port,
					&list_aspm_states_fops);
	if (!d)
		goto remove;

	d = debugfs_create_file("dump_ltssm_trace", S_IRUGO,
					port->port_debugfs, (void *)port,
					&dump_ltssm_trace_fops);
	if (!d)
		goto remove;

	d = debugfs_create_file("get_aspm_duration", S_IRUGO,
					port->port_debugfs, (void *)port,
					&get_aspm_duration_fops);
	if (!d)
		goto remove;

	d = debugfs_create_file("secondary_bus_reset", S_IRUGO,
					port->port_debugfs, (void *)port,
					&secondary_bus_reset_fops);
	if (!d)
		goto remove;

	return 0;

remove:
	debugfs_remove_recursive(port->port_debugfs);
	port->port_debugfs = NULL;
	return -ENOMEM;
}

static void *tegra_pcie_ports_seq_start(struct seq_file *s, loff_t *pos)
{
	struct tegra_pcie *pcie = s->private;

	if (list_empty(&pcie->ports))
		return NULL;

	seq_printf(s, "Index  Status\n");

	return seq_list_start(&pcie->ports, *pos);
}

static void *tegra_pcie_ports_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct tegra_pcie *pcie = s->private;

	return seq_list_next(v, &pcie->ports, pos);
}

static void tegra_pcie_ports_seq_stop(struct seq_file *s, void *v)
{
}

static int tegra_pcie_ports_seq_show(struct seq_file *s, void *v)
{
	bool up = false, active = false;
	struct tegra_pcie_port *port;
	unsigned int value;

	port = list_entry(v, struct tegra_pcie_port, list);

	if (!port->status)
		return 0;

	value = readl(port->base + RP_VEND_XP);

	if (value & RP_VEND_XP_DL_UP)
		up = true;

	value = readl(port->base + RP_LINK_CONTROL_STATUS);

	if (value & RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE)
		active = true;

	seq_printf(s, "%2u     ", port->index);

	if (up)
		seq_printf(s, "up");

	if (active) {
		if (up)
			seq_printf(s, ", ");

		seq_printf(s, "active");
	}

	seq_printf(s, "\n");
	return 0;
}

static const struct seq_operations tegra_pcie_ports_seq_ops = {
	.start = tegra_pcie_ports_seq_start,
	.next = tegra_pcie_ports_seq_next,
	.stop = tegra_pcie_ports_seq_stop,
	.show = tegra_pcie_ports_seq_show,
};

static int tegra_pcie_ports_open(struct inode *inode, struct file *file)
{
	struct tegra_pcie *pcie = inode->i_private;
	struct seq_file *s;
	int err;

	err = seq_open(file, &tegra_pcie_ports_seq_ops);
	if (err)
		return err;

	s = file->private_data;
	s->private = pcie;

	return 0;
}

static const struct file_operations tegra_pcie_ports_ops = {
	.owner = THIS_MODULE,
	.open = tegra_pcie_ports_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void tegra_pcie_debugfs_exit(struct tegra_pcie *pcie)
{
	if (pcie->debugfs)
		debugfs_remove_recursive(pcie->debugfs);
}

static int tegra_pcie_debugfs_init(struct tegra_pcie *pcie)
{
	struct dentry *file, *d;
	struct tegra_pcie_port *port;

	pcie->debugfs = debugfs_create_dir("pcie", NULL);
	if (!pcie->debugfs)
		return -ENOMEM;

	file = debugfs_create_file("ports", S_IFREG | S_IRUGO, pcie->debugfs,
				   pcie, &tegra_pcie_ports_ops);
	if (!file)
		goto remove;

	d = create_tegra_pcie_debufs_file("list_devices",
					&list_devices_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("apply_link_speed",
					&apply_link_speed_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("check_d3hot",
					&check_d3hot_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("power_down",
					&power_down_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("dump_config_space",
					&dump_config_space_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("dump_afi_space",
					&dump_afi_space_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = debugfs_create_u16("bus_dev_func", S_IWUGO | S_IRUGO,
					pcie->debugfs,
					&bdf);
	if (!d)
		goto remove;

	d = debugfs_create_u16("config_offset", S_IWUGO | S_IRUGO,
					pcie->debugfs,
					&config_offset);
	if (!d)
		goto remove;

	d = debugfs_create_u32("config_val", S_IWUGO | S_IRUGO,
					pcie->debugfs,
					&config_val);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("config_read",
					&config_read_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	d = create_tegra_pcie_debufs_file("config_write",
					&config_write_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;
	d = create_tegra_pcie_debufs_file("aspm_l11",
					&aspm_l11_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;
	d = create_tegra_pcie_debufs_file("aspm_l1ss",
					&aspm_l1ss_fops, pcie->debugfs,
					(void *)pcie);
	if (!d)
		goto remove;

	list_for_each_entry(port, &pcie->ports, list) {
		if (port->status)
			if (tegra_pcie_port_debugfs_init(port))
				goto remove;
	}

	return 0;

remove:
	tegra_pcie_debugfs_exit(pcie);
	pcie->debugfs = NULL;
	return -ENOMEM;
}

static int tegra_pcie_probe_complete(struct tegra_pcie *pcie)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(pcie->dev);

	PR_FUNC_LINE;
	ret = tegra_pcie_init(pcie);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		if (pcie->num_ports) {
			ret = tegra_pcie_debugfs_init(pcie);
			if (ret < 0)
				dev_err(&pdev->dev, "failed to setup debugfs: %d\n",
					ret);
		}

	return 0;
}

static void pcie_delayed_detect(struct work_struct *work)
{
	struct tegra_pcie *pcie;
	struct platform_device *pdev;
	int ret = 0;

	pcie = container_of(work, struct tegra_pcie, detect_delay.work);
	pdev = to_platform_device(pcie->dev);
#ifdef CONFIG_THERMAL
	if (pcie->is_cooling_dev) {
		dev_info(pcie->dev,
			 "Going to wait till end point temp is above 0-C\n");
		wait_for_completion_interruptible(&pcie->completion);
		dev_info(pcie->dev,
			 "proceeding with PCIe hierarchy enumeraton\n");
	}
#endif
	ret = tegra_pcie_probe_complete(pcie);
	if (ret || !pcie->num_ports) {
		pm_runtime_put_sync(pcie->dev);
		goto release_regulators;
	}
	return;

release_regulators:
	devm_kfree(pcie->dev, pcie->pcie_regulators);
	devm_kfree(pcie->dev, pcie->plat_data);
	pci_free_host_bridge(pcie->host);
	platform_set_drvdata(pdev, NULL);
	return;
}

#ifdef CONFIG_THERMAL
#define TEGRA_PCIE_THERM_MAX_STATE     2
static int tegra_pcie_max_state(struct thermal_cooling_device *tcd,
				unsigned long *state)
{
	struct tegra_pcie *pcie = tcd->devdata;

	dev_info(pcie->dev, "%s state=%d\n", __func__,
		 pcie->therm_state.counter);
	*state = TEGRA_PCIE_THERM_MAX_STATE;

	return 0;
}

static int tegra_pcie_cur_state(struct thermal_cooling_device *tcd,
				unsigned long *state)
{
	struct tegra_pcie *pcie = tcd->devdata;

	dev_info(pcie->dev, "%s state=%d\n", __func__,
		 pcie->therm_state.counter);
	*state = (unsigned long)atomic_read(&pcie->therm_state);

	return 0;
}

static int tegra_pcie_set_state(struct thermal_cooling_device *tcd,
				unsigned long state)
{
	struct tegra_pcie *pcie = tcd->devdata;

	if (state != (TEGRA_PCIE_THERM_MAX_STATE - 1)) {
		if ((unsigned long)atomic_read(&pcie->therm_state))
			dev_err(pcie->dev, "dGPU temp is below zero...!\n");
		else
			return 0;
	} else {
		atomic_set(&pcie->therm_state, state);
		complete(&pcie->completion);
	}
	return 0;
}

/* Cooling device support */
static struct thermal_cooling_device_ops pcie_cdev_ops = {
	.get_max_state = tegra_pcie_max_state,
	.get_cur_state = tegra_pcie_cur_state,
	.set_cur_state = tegra_pcie_set_state,
};

static int pcie_therm_init(struct tegra_pcie  *pcie)
{
	pcie->cdev = thermal_cooling_device_register("tegra-pcie", pcie,
			&pcie_cdev_ops);
	if (IS_ERR(pcie->cdev))
		return PTR_ERR(pcie->cdev);
	if (pcie->cdev == NULL)
		return -ENODEV;
	dev_info(pcie->dev, "PCIE cooling dev registered\n");
	return 0;
}
#endif

static int tegra_pcie_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	const struct of_device_id *match;
	struct tegra_pcie *pcie;
	struct tegra_pcie_port *port, *tmp;
	struct pci_host_bridge *host;

	PR_FUNC_LINE;

	host = pci_alloc_host_bridge(sizeof(*pcie));
	if (!host)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(host);

	platform_set_drvdata(pdev, pcie);
	pcie->dev = &pdev->dev;
	pcie->host = host;

	/* use DT way to init platform data */
	pcie->plat_data = devm_kzalloc(pcie->dev,
		sizeof(*(pcie->plat_data)), GFP_KERNEL);
	if (!(pcie->plat_data)) {
		dev_err(pcie->dev, "memory alloc failed\n");
		ret = -ENOMEM;
		goto release_drvdata;
	}
	tegra_pcie_read_plat_data(pcie);

	match = of_match_device(tegra_pcie_of_match, &pdev->dev);
	if (!match) {
		ret = -ENODEV;
		goto release_platdata;
	}
	pcie->soc_data = (struct tegra_pcie_soc_data *)match->data;

	if (pcie->soc_data->config_pex_io_dpd) {
		pcie->pex_pin = devm_pinctrl_get(pcie->dev);
		if (IS_ERR(pcie->pex_pin)) {
			ret = PTR_ERR(pcie->pex_pin);
			dev_err(pcie->dev, "pex io-dpd config failed: %d\n",
				ret);
			return ret;
		}

		pcie->pex_io_dpd_en_state = pinctrl_lookup_state(pcie->pex_pin,
							     "pex-io-dpd-en");
		if (IS_ERR(pcie->pex_io_dpd_en_state)) {
			ret = PTR_ERR(pcie->pex_io_dpd_en_state);
			dev_err(pcie->dev, "missing pex-io-dpd en state: %d\n",
				ret);
			return ret;
		}
		pcie->pex_io_dpd_dis_state = pinctrl_lookup_state(pcie->pex_pin,
							     "pex-io-dpd-dis");
		if (IS_ERR(pcie->pex_io_dpd_dis_state)) {
			ret = PTR_ERR(pcie->pex_io_dpd_dis_state);
			dev_err(pcie->dev, "missing pex-io-dpd dis state:%ld\n",
				PTR_ERR(pcie->pex_io_dpd_dis_state));
			return ret;
		}
	}

	pcie->pcie_regulators = devm_kzalloc(pcie->dev,
		pcie->soc_data->num_pcie_regulators
			* sizeof(struct regulator *), GFP_KERNEL);

	for (i = 0; i < pcie->soc_data->num_pcie_regulators; i++) {
		pcie->pcie_regulators[i] =
					devm_regulator_get(pcie->dev,
			pcie->soc_data->pcie_regulator_names[i]);
		if (IS_ERR(pcie->pcie_regulators[i])) {
			dev_err(pcie->dev, "%s: unable to get regulator %s\n",
			__func__,
			pcie->soc_data->pcie_regulator_names[i]);
			pcie->pcie_regulators[i] = NULL;
			ret = IS_ERR(pcie->pcie_regulators[i]);
			goto release_regulators;
		}
	}

	INIT_LIST_HEAD(&pcie->buses);
	INIT_LIST_HEAD(&pcie->ports);
	INIT_LIST_HEAD(&pcie->sys);
	INIT_DELAYED_WORK(&pcie->detect_delay, pcie_delayed_detect);

	ret = tegra_pcie_parse_dt(pcie);
	if (ret < 0)
		goto release_regulators;

	if (pcie->soc_data->program_uphy) {
		ret = tegra_pcie_phys_get(pcie);
		if (ret < 0) {
			if (ret == -EPROBE_DEFER)
				dev_info(pcie->dev, "failed to get PHYs: %d\n", ret);
			else
				dev_err(pcie->dev, "failed to get PHYs: %d\n", ret);
			goto release_regulators;
		}
	}

	pcie->prod_list = devm_tegra_prod_get(pcie->dev);
	if (IS_ERR(pcie->prod_list)) {
		dev_info(pcie->dev, "No prod values found\n");
		pcie->prod_list = NULL;
	}

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		if (port->has_mxm_port) {
			if (tegra_pcie_mxm_pwr_init(port))
				dev_info(pcie->dev,
					"pwr_good is down for port %d, ignoring\n",
					 port->index);
		}
		if (pcie->soc_data->program_clkreq_as_bi_dir) {
			/* set clkreq as input to avoid root port control it */
			tegra_pcie_config_clkreq(pcie, port->index, 0);
		}
	}

	/* Enable Runtime PM for PCIe, TODO: Need to add PCIe host device */
	pm_runtime_enable(pcie->dev);

#ifdef CONFIG_THERMAL
	if (pcie->is_cooling_dev) {
		init_completion(&pcie->completion);
		/* register cooling device */
		ret = pcie_therm_init(pcie);
		if (ret != 0) {
			dev_err(pcie->dev,
				"unable to register cooling device err: %d\n",
				ret);
			goto release_regulators;
		}
	}
#endif

	schedule_delayed_work(&pcie->detect_delay,
			      msecs_to_jiffies(
			      pcie->plat_data->boot_detect_delay));
	return ret;

release_regulators:
	if (pcie->soc_data->program_uphy)
		tegra_pcie_phy_exit(pcie);
	devm_kfree(&pdev->dev, pcie->pcie_regulators);
release_platdata:
	devm_kfree(&pdev->dev, pcie->plat_data);
release_drvdata:
	pci_free_host_bridge(host);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int tegra_pcie_remove(struct platform_device *pdev)
{
	struct tegra_pcie *pcie = platform_get_drvdata(pdev);

	PR_FUNC_LINE;

	if (!pcie)
		return 0;

	if (cancel_delayed_work_sync(&pcie->detect_delay))
		return 0;
	if (IS_ENABLED(CONFIG_DEBUG_FS))
		tegra_pcie_debugfs_exit(pcie);
	pci_stop_root_bus(pcie->host->bus);
	pci_bus_remove_resources(pcie->host->bus);
	pci_remove_root_bus(pcie->host->bus);
	iounmap(pcie->cfg_va_base);
	if (IS_ENABLED(CONFIG_PCI_MSI))
		tegra_pcie_disable_msi(pcie);
	tegra_pcie_detach(pcie);
	tegra_pcie_power_off(pcie);
	if (pcie->soc_data->program_uphy)
		tegra_pcie_phy_exit(pcie);
	tegra_pcie_release_resources(pcie);
	pm_runtime_disable(pcie->dev);
	tegra_pcie_free_resources(pcie);
	pci_free_host_bridge(pcie->host);

	return 0;
}

static inline u32 get_pme_port_offset(struct tegra_pcie_port *port)
{
	u32 ret = 0;

	switch (port->index) {
	case 0:
		ret = 0;
		break;
	case 1:
		ret = 8;
		break;
	case 2:
		ret = 12;
		break;
	}
	return ret;
}

static inline u32 get_pme_ack_offset(struct tegra_pcie_port *port)
{
	u32 ret = 0;

	switch (port->index) {
	case 0:
		ret = 5;
		break;
	case 1:
		ret = 10;
		break;
	case 2:
		ret = 14;
		break;
	}
	return ret;
}

int tegra_pcie_pm_control(enum tegra_pcie_pm_opt pm_opt, void *user)
{
	struct pci_dev *epdev = (struct pci_dev *)user;
	struct pci_dev *rpdev = epdev->bus->self;
	struct pci_host_bridge *host = pci_find_host_bridge(epdev->bus);
	struct tegra_pcie *pcie = pci_host_bridge_priv(host);

	struct tegra_pcie_port *port = NULL;
	unsigned long ctrl = 0;
	u32 rp;
	u32 val;
	u32 timeout = 100; /* 10 ms */
	u16 val_16 = 0;

	rp = PCI_SLOT(rpdev->devfn);
	list_for_each_entry(port, &pcie->ports, list)
		if (rp == port->index + 1)
			break;

	switch (pm_opt) {
	case TEGRA_PCIE_SUSPEND:
		pr_debug("---> in suspend\n");

		port->ep_status = 0;
		/* now setting it to '1' */
		val = afi_readl(pcie, AFI_PCIE_PME);
		val |= (0x1 << get_pme_port_offset(port));
		afi_writel(pcie, val, AFI_PCIE_PME);
		/* wait till ack is received */
		do {
			udelay(1);
			val = afi_readl(pcie, AFI_PCIE_PME);
			val = val & (0x1 << get_pme_ack_offset(port));
		} while (!(val));
		usleep_range(10000, 11000); /* 10ms delay */
		/* clear PME_TO */
		val = afi_readl(pcie, AFI_PCIE_PME);
		val &= ~(0x1 << get_pme_port_offset(port));
		afi_writel(pcie, val, AFI_PCIE_PME);

		/* by this time, link would have gone into L2/L3 ready */
		/* assert reset to EP */
		pr_debug("---> asserting EP reset through AFI\n");
		ctrl = tegra_pcie_port_get_pex_ctrl(port);
		val = afi_readl(port->pcie, ctrl);
		val &= ~AFI_PEX_CTRL_RST;
		afi_writel(port->pcie, val, ctrl);

		val = afi_readl(port->pcie, ctrl);
		val &= ~AFI_PEX_CTRL_REFCLK_EN;
		afi_writel(port->pcie, val, ctrl);

		msleep(20);
		break;

	case TEGRA_PCIE_RESUME_PRE:
		pr_debug("---> in resume (pre)\n");

		port->ep_status = 1;
		/* assert SBR on RP */
		pr_debug("---> perform assert,de-assert of SBR\n");
		pci_read_config_word(rpdev, PCI_BRIDGE_CONTROL, &val_16);
		val_16 |= PCI_BRIDGE_CTL_BUS_RESET;
		pci_write_config_word(rpdev, PCI_BRIDGE_CONTROL, val_16);
		msleep(20);
		val_16 &= ~PCI_BRIDGE_CTL_BUS_RESET;
		pci_write_config_word(rpdev, PCI_BRIDGE_CONTROL, val_16);
		msleep(100);
		break;

	case TEGRA_PCIE_RESUME_POST:
		pr_debug("---> in resume (post)\n");

		/* de-assert reset to EP */
		pr_debug("---> de-asserting EP reset through AFI\n");
		ctrl = tegra_pcie_port_get_pex_ctrl(port);
		val = afi_readl(port->pcie, ctrl);
		val |= AFI_PEX_CTRL_RST;
		afi_writel(port->pcie, val, ctrl);

		val = afi_readl(port->pcie, ctrl);
		val |= AFI_PEX_CTRL_REFCLK_EN;
		afi_writel(port->pcie, val, ctrl);

		msleep(100);

		/* make sure that link is up before doing anything */
		do {
			val = readl(port->base + RP_VEND_XP);
			pr_debug("---> checking for link up\n");
			if (val & RP_VEND_XP_DL_UP)
				break;
			usleep_range(100, 200);
		} while (--timeout);
		if (!timeout) {
			dev_err(port->pcie->dev, "link %u is down\n",
				port->index);
			return -1;
		}

		/* try to read one config space register as this would
		 * result in completion timeout, hence we wouldn't be losing
		 * anything later on from second access onwards */
		/* EP device state */
		pr_debug("---> First config read START\n");
		pci_read_config_word(epdev, PCI_DEVICE_ID, &val_16);
		pr_debug("EP device ID = 0x%04X\n", val_16);
		pr_debug("---> First config read END\n");
		msleep(100);
		break;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_pcie_pm_control);

#ifdef CONFIG_PM

static int tegra_pcie_enable_msi(struct tegra_pcie *, bool);

static int tegra_pcie_resume(struct device *dev)
{
	struct tegra_pcie *pcie = dev_get_drvdata(dev);
	PR_FUNC_LINE;
	if (!pcie)
		return 0;
	tegra_pcie_enable_features(pcie);
	return 0;
}

/* Since BCM4359 WiFi driver is not informing the system about its absence
 * when Wifi is turned off, PCIe subsystem tries to do save/restore as part
 * of its routine during SC7 cycle will lead to error interrupt generation
 * which prevents system entering into SC7 state. Hence, it is better to
 * disable interrupts in suspend_late as there are no interrupts after this
 * stage anyway and re-enable in resume_early
 */
static int tegra_pcie_suspend_late(struct device *dev)
{
	struct tegra_pcie *pcie = dev_get_drvdata(dev);
	u32 val = 0;

	PR_FUNC_LINE;
	if (!pcie)
		return 0;
	val = afi_readl(pcie, AFI_INTR_MASK);
	val &= ~AFI_INTR_MASK_INT_MASK;
	val &= ~AFI_INTR_MASK_MSI_MASK;
	afi_writel(pcie, val, AFI_INTR_MASK);
	return 0;
}

static int tegra_pcie_resume_early(struct device *dev)
{
	struct tegra_pcie *pcie = dev_get_drvdata(dev);
	u32 val = 0;

	PR_FUNC_LINE;
	if (!pcie)
		return 0;
	val = afi_readl(pcie, AFI_INTR_MASK);
	val |= AFI_INTR_MASK_INT_MASK;
	val |= AFI_INTR_MASK_MSI_MASK;
	afi_writel(pcie, val, AFI_INTR_MASK);
	return 0;
}

static const struct dev_pm_ops tegra_pcie_pm_ops = {
	.resume = tegra_pcie_resume,
	.suspend_late = tegra_pcie_suspend_late,
	.resume_early = tegra_pcie_resume_early,
	.runtime_suspend = tegra_pcie_save_device,
	.runtime_resume = tegra_pcie_restore_device,
	.suspend_noirq = tegra_pcie_save_device,
	.resume_noirq = tegra_pcie_restore_device,
};
#endif /* CONFIG_PM */

/* driver data is accessed after init, so use __refdata instead of __initdata */
static struct platform_driver __refdata tegra_pcie_driver = {
	.probe   = tegra_pcie_probe,
	.remove  = tegra_pcie_remove,
	.driver  = {
		.name  = "tegra-pcie",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &tegra_pcie_pm_ops,
#endif
		.of_match_table = tegra_pcie_of_match,
	},
};

static int __init tegra_pcie_init_driver(void)
{
	return platform_driver_register(&tegra_pcie_driver);
}

static void __exit tegra_pcie_exit_driver(void)
{
	platform_driver_unregister(&tegra_pcie_driver);
}

module_init(tegra_pcie_init_driver);
module_exit(tegra_pcie_exit_driver);
MODULE_LICENSE("GPL v2");
