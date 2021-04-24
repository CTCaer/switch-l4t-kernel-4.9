/*
 * Minerva Training Cell mini
 * DRAM Training for Icosa platform (T210 Nintendo Switch - LPDDR4).
 *
 * Mini version only supports clock switching and periodic training.
 *
 * Copyright (c) 2018-2020 Kostas Missos <ctcaer@gmail.com>
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
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <soc/tegra/tegra_emc.h>

#include "tegra210-emc-reg.h"

#define DVFS_CLOCK_CHANGE_VERSION "Minerva Training Cell v1.4m_lpddr4"
#define EMC_PRELOCK_VERSION       2101

#define emc_cc_dbg(t, ...) pr_debug(__VA_ARGS__)

#define _REG(base, off) *(volatile unsigned int __force *)((base) + (off))

#define MC(off) _REG(mc_base, off)
#define EMC(off) _REG(emc_base, off)
#define EMC_CH0(off) _REG(emc0_base, off)
#define EMC_CH1(off) _REG(emc1_base, off)

/*
 * PTFV defines - basically just indexes into the per table PTFV array.
 */
#define PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX 0
#define PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX 1
#define PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX 2
#define PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX 3
#define PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX 4
#define PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX 5
#define PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX 6
#define PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX 7
#define PTFV_DVFS_SAMPLES_INDEX         9
#define PTFV_MOVAVG_WEIGHT_INDEX        10
#define PTFV_CONFIG_CTRL_INDEX          11

#define PTFV_CONFIG_CTRL_USE_PREVIOUS_EMA	(1 << 0)

/*
 * Per channel registers offsets based on EMC_BASE.
 */
#define EMC0_MRW10 0x34B4
#define EMC0_MRW11 0x34B8
#define EMC0_MRW12 0x34BC
#define EMC0_MRW13 0x34C0
#define EMC0_DATA_BRLSHFT_0 0x3588
#define EMC0_DATA_BRLSHFT_1 0x358C
#define EMC0_CMD_BRLSHFT_0  0x359C
#define EMC0_QUSE_BRLSHFT_0 0x35AC
#define EMC0_QUSE_BRLSHFT_2 0x35BC
#define EMC0_TRAINING_RW_OFFSET_IB_BYTE0 0x3E98
#define EMC0_TRAINING_RW_OFFSET_IB_BYTE1 0x3E9C
#define EMC0_TRAINING_RW_OFFSET_IB_BYTE2 0x3EA0
#define EMC0_TRAINING_RW_OFFSET_IB_BYTE3 0x3EA4
#define EMC0_TRAINING_RW_OFFSET_IB_MISC  0x3EA8
#define EMC0_TRAINING_RW_OFFSET_OB_BYTE0 0x3EAC
#define EMC0_TRAINING_RW_OFFSET_OB_BYTE1 0x3EB0
#define EMC0_TRAINING_RW_OFFSET_OB_BYTE2 0x3EB4
#define EMC0_TRAINING_RW_OFFSET_OB_BYTE3 0x3EB8
#define EMC0_TRAINING_RW_OFFSET_OB_MISC  0x3EBC
#define EMC0_TRAINING_OPT_DQS_IB_VREF_RANK0 0x3ED4
#define EMC0_TRAINING_OPT_DQS_IB_VREF_RANK1 0x3ED8

#define EMC1_MRW10 0x44B4
#define EMC1_MRW11 0x44B8
#define EMC1_MRW12 0x44BC
#define EMC1_MRW13 0x44C0
#define EMC1_DATA_BRLSHFT_0 0x4588
#define EMC1_DATA_BRLSHFT_1 0x458C
#define EMC1_CMD_BRLSHFT_1  0x45A0
#define EMC1_QUSE_BRLSHFT_1 0x45B8
#define EMC1_QUSE_BRLSHFT_3 0x45C4
#define EMC1_TRAINING_RW_OFFSET_IB_BYTE0 0x4E98
#define EMC1_TRAINING_RW_OFFSET_IB_BYTE1 0x4E9C
#define EMC1_TRAINING_RW_OFFSET_IB_BYTE2 0x4EA0
#define EMC1_TRAINING_RW_OFFSET_IB_BYTE3 0x4EA4
#define EMC1_TRAINING_RW_OFFSET_IB_MISC  0x4EA8
#define EMC1_TRAINING_RW_OFFSET_OB_BYTE0 0x4EAC
#define EMC1_TRAINING_RW_OFFSET_OB_BYTE1 0x4EB0
#define EMC1_TRAINING_RW_OFFSET_OB_BYTE2 0x4EB4
#define EMC1_TRAINING_RW_OFFSET_OB_BYTE3 0x4EB8
#define EMC1_TRAINING_RW_OFFSET_OB_MISC  0x4EBC
#define EMC1_TRAINING_OPT_DQS_IB_VREF_RANK0 0x4ED4
#define EMC1_TRAINING_OPT_DQS_IB_VREF_RANK1 0x4ED8

/*
 * Enable flags for specifying verbosity.
 */
#define INFO            (1 << 0)
#define STEPS           (1 << 1)
#define SUB_STEPS       (1 << 2)
#define PRELOCK         (1 << 3)
#define PRELOCK_STEPS   (1 << 4)
#define ACTIVE_EN       (1 << 5)
#define PRAMP_UP        (1 << 6)
#define PRAMP_DN        (1 << 7)
#define EMA_WRITES      (1 << 10)
#define EMA_UPDATES     (1 << 11)
#define PER_TRAIN       (1 << 16)
#define CC_PRINT        (1 << 17)
#define CCFIFO          (1 << 29)
#define REGS            (1 << 30)
#define REG_LISTS       (1 << 31)

#define EMC_EMC_STATUS_REQ_FIFO_EMPTY  (1 << 0)

enum
{
	DVFS_SEQUENCE = 1,
	WRITE_TRAINING_SEQUENCE = 2,
	PERIODIC_TRAINING_SEQUENCE = 3,
	DVFS_PT1 = 10,
	DVFS_UPDATE = 11,
	TRAINING_PT1 = 12,
	TRAINING_UPDATE = 13,
	PERIODIC_TRAINING_UPDATE = 14
};

enum emc_channels
{
	EMC_CHANNEL0 = 0,
	EMC_CHANNEL1 = 1
};

static const u32 burst_regs_emc_addr_table[221] = {
	EMC_RC,
	EMC_RFC,
	EMC_RFCPB,
	EMC_REFCTRL2,
	EMC_RFC_SLR,
	EMC_RAS,
	EMC_RP,
	EMC_R2W,
	EMC_W2R,
	EMC_R2P,
	EMC_W2P,
	EMC_R2R,
	EMC_TPPD,
	EMC_CCDMW,
	EMC_RD_RCD,
	EMC_WR_RCD,
	EMC_RRD,
	EMC_REXT,
	EMC_WEXT,
	EMC_WDV_CHK,
	EMC_WDV,
	EMC_WSV,
	EMC_WEV,
	EMC_WDV_MASK,
	EMC_WS_DURATION,
	EMC_WE_DURATION,
	EMC_QUSE,
	EMC_QUSE_WIDTH,
	EMC_IBDLY,
	EMC_OBDLY,
	EMC_EINPUT,
	EMC_MRW6,
	EMC_EINPUT_DURATION,
	EMC_PUTERM_EXTRA,
	EMC_PUTERM_WIDTH,
	EMC_QRST,
	EMC_QSAFE,
	EMC_RDV,
	EMC_RDV_MASK,
	EMC_RDV_EARLY,
	EMC_RDV_EARLY_MASK,
	EMC_REFRESH,
	EMC_BURST_REFRESH_NUM,
	EMC_PRE_REFRESH_REQ_CNT,
	EMC_PDEX2WR,
	EMC_PDEX2RD,
	EMC_PCHG2PDEN,
	EMC_ACT2PDEN,
	EMC_AR2PDEN,
	EMC_RW2PDEN,
	EMC_CKE2PDEN,
	EMC_PDEX2CKE,
	EMC_PDEX2MRR,
	EMC_TXSR,
	EMC_TXSRDLL,
	EMC_TCKE,
	EMC_TCKESR,
	EMC_TPD,
	EMC_TFAW,
	EMC_TRPAB,
	EMC_TCLKSTABLE,
	EMC_TCLKSTOP,
	EMC_MRW7,
	EMC_TREFBW,
	EMC_ODT_WRITE,
	EMC_FBIO_CFG5,
	EMC_FBIO_CFG7,
	EMC_CFG_DIG_DLL,
	EMC_CFG_DIG_DLL_PERIOD,
	EMC_PMACRO_IB_RXRT,
	EMC_CFG_PIPE_1,
	EMC_CFG_PIPE_2,
	EMC_PMACRO_QUSE_DDLL_RANK0_4,
	EMC_PMACRO_QUSE_DDLL_RANK0_5,
	EMC_PMACRO_QUSE_DDLL_RANK1_4,
	EMC_PMACRO_QUSE_DDLL_RANK1_5,
	EMC_MRW8,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_5,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_2,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_3,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_5,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_2,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_3,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4,
	EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_5,
	EMC_PMACRO_DDLL_LONG_CMD_0,
	EMC_PMACRO_DDLL_LONG_CMD_1,
	EMC_PMACRO_DDLL_LONG_CMD_2,
	EMC_PMACRO_DDLL_LONG_CMD_3,
	EMC_PMACRO_DDLL_LONG_CMD_4,
	EMC_PMACRO_DDLL_SHORT_CMD_0,
	EMC_PMACRO_DDLL_SHORT_CMD_1,
	EMC_PMACRO_DDLL_SHORT_CMD_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE0_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE1_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE2_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE3_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE4_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE5_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE6_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE7_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD0_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD1_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD2_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD3_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE0_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE1_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE2_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE3_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE4_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE5_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE6_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE7_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD0_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD0_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD0_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD0_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD1_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD1_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD1_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD1_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD2_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD2_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD2_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD2_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD3_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD3_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD3_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_CMD3_3,
	EMC_TXDSRVTTGEN,
	EMC_FDPD_CTRL_DQ,
	EMC_FDPD_CTRL_CMD,
	EMC_FBIO_SPARE,
	EMC_ZCAL_INTERVAL,
	EMC_ZCAL_WAIT_CNT,
	EMC_MRS_WAIT_CNT,
	EMC_MRS_WAIT_CNT2,
	EMC_AUTO_CAL_CHANNEL,
	EMC_DLL_CFG_0,
	EMC_DLL_CFG_1,
	EMC_PMACRO_AUTOCAL_CFG_COMMON,
	EMC_PMACRO_ZCTRL,
	EMC_CFG,
	EMC_CFG_PIPE,
	EMC_DYN_SELF_REF_CONTROL,
	EMC_QPOP,
	EMC_DQS_BRLSHFT_0,
	EMC_DQS_BRLSHFT_1,
	EMC_CMD_BRLSHFT_2,
	EMC_CMD_BRLSHFT_3,
	EMC_PMACRO_PAD_CFG_CTRL,
	EMC_PMACRO_DATA_PAD_RX_CTRL,
	EMC_PMACRO_CMD_PAD_RX_CTRL,
	EMC_PMACRO_DATA_RX_TERM_MODE,
	EMC_PMACRO_CMD_RX_TERM_MODE,
	EMC_PMACRO_CMD_PAD_TX_CTRL,
	EMC_PMACRO_DATA_PAD_TX_CTRL,
	EMC_PMACRO_COMMON_PAD_TX_CTRL,
	EMC_PMACRO_VTTGEN_CTRL_0,
	EMC_PMACRO_VTTGEN_CTRL_1,
	EMC_PMACRO_VTTGEN_CTRL_2,
	EMC_PMACRO_BRICK_CTRL_RFU1,
	EMC_PMACRO_CMD_BRICK_CTRL_FDPD,
	EMC_PMACRO_BRICK_CTRL_RFU2,
	EMC_PMACRO_DATA_BRICK_CTRL_FDPD,
	EMC_PMACRO_BG_BIAS_CTRL_0,
	EMC_CFG_3,
	EMC_PMACRO_TX_PWRD_0,
	EMC_PMACRO_TX_PWRD_1,
	EMC_PMACRO_TX_PWRD_2,
	EMC_PMACRO_TX_PWRD_3,
	EMC_PMACRO_TX_PWRD_4,
	EMC_PMACRO_TX_PWRD_5,
	EMC_CONFIG_SAMPLE_DELAY,
	EMC_PMACRO_TX_SEL_CLK_SRC_0,
	EMC_PMACRO_TX_SEL_CLK_SRC_1,
	EMC_PMACRO_TX_SEL_CLK_SRC_2,
	EMC_PMACRO_TX_SEL_CLK_SRC_3,
	EMC_PMACRO_TX_SEL_CLK_SRC_4,
	EMC_PMACRO_TX_SEL_CLK_SRC_5,
	EMC_PMACRO_DDLL_BYPASS,
	EMC_PMACRO_DDLL_PWRD_0,
	EMC_PMACRO_DDLL_PWRD_1,
	EMC_PMACRO_DDLL_PWRD_2,
	EMC_PMACRO_CMD_CTRL_0,
	EMC_PMACRO_CMD_CTRL_1,
	EMC_PMACRO_CMD_CTRL_2,
	EMC_TR_TIMING_0,
	EMC_TR_DVFS,
	EMC_TR_CTRL_1,
	EMC_TR_RDV,
	EMC_TR_QPOP,
	EMC_TR_RDV_MASK,
	EMC_MRW14,
	EMC_TR_QSAFE,
	EMC_TR_QRST,
	EMC_TRAINING_CTRL,
	EMC_TRAINING_SETTLE,
	EMC_TRAINING_VREF_SETTLE,
	EMC_TRAINING_CA_FINE_CTRL,
	EMC_TRAINING_CA_CTRL_MISC,
	EMC_TRAINING_CA_CTRL_MISC1,
	EMC_TRAINING_CA_VREF_CTRL,
	EMC_TRAINING_QUSE_CORS_CTRL,
	EMC_TRAINING_QUSE_FINE_CTRL,
	EMC_TRAINING_QUSE_CTRL_MISC,
	EMC_TRAINING_QUSE_VREF_CTRL,
	EMC_TRAINING_READ_FINE_CTRL,
	EMC_TRAINING_READ_CTRL_MISC,
	EMC_TRAINING_READ_VREF_CTRL,
	EMC_TRAINING_WRITE_FINE_CTRL,
	EMC_TRAINING_WRITE_CTRL_MISC,
	EMC_TRAINING_WRITE_VREF_CTRL,
	EMC_TRAINING_MPC,
	EMC_MRW15
};

static const u32 burst_reg_per_ch_emc01_addr_table[8] = {
	EMC0_MRW10,
	EMC1_MRW10,
	EMC0_MRW11,
	EMC1_MRW11,
	EMC0_MRW12,
	EMC1_MRW12,
	EMC0_MRW13,
	EMC1_MRW13
};

static const u32 vref_perch_regs_emc01_addr_table[4] = {
	EMC0_TRAINING_OPT_DQS_IB_VREF_RANK0,
	EMC1_TRAINING_OPT_DQS_IB_VREF_RANK0,
	EMC0_TRAINING_OPT_DQS_IB_VREF_RANK1,
	EMC1_TRAINING_OPT_DQS_IB_VREF_RANK1
};

static const u32 trim_regs_emc_addr_table[138] = {
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_2,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_3,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_0,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_1,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_2,
	EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_3,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE0_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE0_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE0_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE1_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE1_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE1_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE2_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE2_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE2_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE3_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE3_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE3_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE4_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE4_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE4_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE5_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE5_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE5_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE6_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE6_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE6_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE7_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE7_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK0_BYTE7_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE0_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE0_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE0_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE1_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE1_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE1_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE2_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE2_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE2_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE3_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE3_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE3_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE4_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE4_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE4_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE5_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE5_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE5_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE6_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE6_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE6_2,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE7_0,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE7_1,
	EMC_PMACRO_IB_DDLL_SHORT_DQ_RANK1_BYTE7_2,
	EMC_PMACRO_IB_VREF_DQS_0,
	EMC_PMACRO_IB_VREF_DQS_1,
	EMC_PMACRO_IB_VREF_DQ_0,
	EMC_PMACRO_IB_VREF_DQ_1,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_5,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE0_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE0_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE0_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE1_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE1_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE1_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE2_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE2_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE2_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE3_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE3_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE3_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE4_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE4_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE4_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE5_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE5_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE5_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE6_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE6_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE6_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE7_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE7_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_BYTE7_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD0_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD0_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD0_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD1_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD1_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD1_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD2_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD2_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD2_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD3_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD3_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK0_CMD3_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE0_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE0_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE0_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE1_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE1_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE1_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE2_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE2_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE2_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE3_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE3_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE3_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE4_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE4_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE4_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE5_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE5_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE5_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE6_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE6_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE6_2,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE7_0,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE7_1,
	EMC_PMACRO_OB_DDLL_SHORT_DQ_RANK1_BYTE7_2,
	EMC_PMACRO_QUSE_DDLL_RANK0_0,
	EMC_PMACRO_QUSE_DDLL_RANK0_1,
	EMC_PMACRO_QUSE_DDLL_RANK0_2,
	EMC_PMACRO_QUSE_DDLL_RANK0_3,
	EMC_PMACRO_QUSE_DDLL_RANK1_0,
	EMC_PMACRO_QUSE_DDLL_RANK1_1,
	EMC_PMACRO_QUSE_DDLL_RANK1_2,
	EMC_PMACRO_QUSE_DDLL_RANK1_3
};

static const u32 trim_perch_regs_emc01_addr_table[10] = {
	EMC0_CMD_BRLSHFT_0,
	EMC1_CMD_BRLSHFT_1,
	EMC0_DATA_BRLSHFT_0,
	EMC1_DATA_BRLSHFT_0,
	EMC0_DATA_BRLSHFT_1,
	EMC1_DATA_BRLSHFT_1,
	EMC0_QUSE_BRLSHFT_0,
	EMC1_QUSE_BRLSHFT_1,
	EMC0_QUSE_BRLSHFT_2,
	EMC1_QUSE_BRLSHFT_3
};

static const u32 burst_mc_regs_addr_table[33] = {
	MC_EMEM_ARB_CFG,
	MC_EMEM_ARB_OUTSTANDING_REQ,
	MC_EMEM_ARB_REFPB_HP_CTRL,
	MC_EMEM_ARB_REFPB_BANK_CTRL,
	MC_EMEM_ARB_TIMING_RCD,
	MC_EMEM_ARB_TIMING_RP,
	MC_EMEM_ARB_TIMING_RC,
	MC_EMEM_ARB_TIMING_RAS,
	MC_EMEM_ARB_TIMING_FAW,
	MC_EMEM_ARB_TIMING_RRD,
	MC_EMEM_ARB_TIMING_RAP2PRE,
	MC_EMEM_ARB_TIMING_WAP2PRE,
	MC_EMEM_ARB_TIMING_R2R,
	MC_EMEM_ARB_TIMING_W2W,
	MC_EMEM_ARB_TIMING_R2W,
	MC_EMEM_ARB_TIMING_CCDMW,
	MC_EMEM_ARB_TIMING_W2R,
	MC_EMEM_ARB_TIMING_RFCPB,
	MC_EMEM_ARB_DA_TURNS,
	MC_EMEM_ARB_DA_COVERS,
	MC_EMEM_ARB_MISC0,
	MC_EMEM_ARB_MISC1,
	MC_EMEM_ARB_MISC2,
	MC_EMEM_ARB_RING1_THROTTLE,
	MC_EMEM_ARB_DHYST_CTRL,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_0,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_1,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_2,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_3,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_4,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_5,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_6,
	MC_EMEM_ARB_DHYST_TIMEOUT_UTIL_7
};

static const u32 la_scale_regs_mc_addr_table[24] = {
	MC_MLL_MPCORER_PTSA_RATE,
	MC_FTOP_PTSA_RATE,
	MC_PTSA_GRANT_DECREMENT,
	MC_LATENCY_ALLOWANCE_XUSB_0,
	MC_LATENCY_ALLOWANCE_XUSB_1,
	MC_LATENCY_ALLOWANCE_TSEC_0,
	MC_LATENCY_ALLOWANCE_SDMMCA_0,
	MC_LATENCY_ALLOWANCE_SDMMCAA_0,
	MC_LATENCY_ALLOWANCE_SDMMC_0,
	MC_LATENCY_ALLOWANCE_SDMMCAB_0,
	MC_LATENCY_ALLOWANCE_PPCS_0,
	MC_LATENCY_ALLOWANCE_PPCS_1,
	MC_LATENCY_ALLOWANCE_MPCORE_0,
	MC_LATENCY_ALLOWANCE_HC_0,
	MC_LATENCY_ALLOWANCE_HC_1,
	MC_LATENCY_ALLOWANCE_AVPC_0,
	MC_LATENCY_ALLOWANCE_GPU_0,
	MC_LATENCY_ALLOWANCE_GPU2_0,
	MC_LATENCY_ALLOWANCE_NVENC_0,
	MC_LATENCY_ALLOWANCE_NVDEC_0,
	MC_LATENCY_ALLOWANCE_VIC_0,
	MC_LATENCY_ALLOWANCE_VI2_0,
	MC_LATENCY_ALLOWANCE_ISP2_0,
	MC_LATENCY_ALLOWANCE_ISP2_1
};

static const u32 periodic_training_addr[10] =
{
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2,
	EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3,
	EMC_DATA_BRLSHFT_0,
	EMC_DATA_BRLSHFT_1
};

static void _request_mmr_data(u32 data, bool dual_channel);
static u32 _digital_dll_prelock(struct emc_table *next_timing, u32 selected_clk_src_emc);
static void _digital_dll_enable_rs(u32 channel1_enabled);
static u32 _dvfs_power_ramp_down(bool flip_backward, struct emc_table *src_emc_table_entry,
	struct emc_table *dst_emc_table_entry, u32 src_clock_period);
static u32 _dvfs_power_ramp_up(bool flip_backward, struct emc_table *src_emc_table_entry,
	struct emc_table *dst_emc_table_entry, u8 needs_training, u32 dst_clock_period);
static u32 _minerva_update_clock_tree_delay(struct emc_table *src_emc_entry,
	struct emc_table *dst_emc_entry, u32 dram_dev_num, u32 channel1_enabled, u32 update_type);
static u32 _minerva_periodic_compensation_handler(struct emc_table *src_emc_entry,
	struct emc_table *dst_emc_entry, u32 dram_dev_num, u32 channel1_enabled);

inline void _ccfifo_write(u32 addr, u32 data_val, u32 delay)
{
	ccfifo_writel(data_val, addr, delay);
}

static bool _wait_emc_status(u32 reg_offset, u32 bit_mask, bool updated_state, s32 emc_channel)
{
	u32 i;
	bool err = true;

	for (i = 0; i < EMC_STATUS_UPDATE_TIMEOUT; i++)
	{
		if (emc_channel)
		{
			if (emc_channel != 1)
				goto done;

			if (((EMC_CH1(reg_offset) & bit_mask) != 0) == updated_state)
			{
				err = false;
				break;
			}
		}
		else if (((EMC(reg_offset) & bit_mask) != 0) == updated_state)
		{
			err = false;
			break;
		}
		udelay(1);
	}

done:
	return err;
}

static void _request_mmr_data(u32 data, bool dual_channel)
{
	u32 emc_cfg;

	emc_cfg = EMC(EMC_CFG);
	if (emc_cfg & EMC_CFG_DRAM_ACPD)
	{
		EMC(EMC_CFG) = emc_cfg & ~EMC_CFG_DRAM_ACPD;
		emc_timing_update(0);
	}

	// Get data.
	EMC(EMC_MRR) = data;
	_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_MRR_DIVLD, true, EMC_CHANNEL0);
	if (dual_channel)
		_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_MRR_DIVLD, true, EMC_CHANNEL1);

	if (emc_cfg & EMC_CFG_DRAM_ACPD)
	{
		EMC(EMC_CFG) = emc_cfg;
		emc_timing_update(0);
	}
}

static u32 _digital_dll_prelock(struct emc_table *next_timing, u32 selected_clk_src_emc)
{
	u32 dual_channel;

	dual_channel = (EMC(EMC_FBIO_CFG7) >> 1) & ((EMC(EMC_FBIO_CFG7) >> 2) & 1);

	EMC(EMC_CFG_DIG_DLL) = (EMC(EMC_CFG_DIG_DLL) & 0xFFFFF824) | 0x3C8;

	emc_timing_update(dual_channel);

	while (EMC(EMC_CFG_DIG_DLL) & EMC_CFG_DIG_DLL_CFG_DLL_EN)
		;
	if (dual_channel)
		while (EMC_CH1(EMC_CFG_DIG_DLL) & EMC_CFG_DIG_DLL_CFG_DLL_EN)
			;

	EMC(EMC_DLL_CFG_0) = next_timing->burst_regs[EMC_DLL_CFG_0_INDEX];
	EMC(EMC_DLL_CFG_1) = next_timing->burst_regs[EMC_DLL_CFG_1_INDEX];

	tegra210_change_dll_src(next_timing, selected_clk_src_emc);

	EMC(EMC_CFG_DIG_DLL) |= EMC_CFG_DIG_DLL_CFG_DLL_EN;

	emc_timing_update(dual_channel);

	while (!(EMC(EMC_CFG_DIG_DLL) & EMC_CFG_DIG_DLL_CFG_DLL_EN))
		;
	if (dual_channel)
		while (!(EMC_CH1(EMC_CFG_DIG_DLL) & EMC_CFG_DIG_DLL_CFG_DLL_EN))
			;

	while ((((EMC(EMC_DIG_DLL_STATUS) >> 17) & 1) ^ 1) | (((EMC(EMC_DIG_DLL_STATUS) >> 15) & 1) ^ 1))
		;

	return EMC(EMC_DIG_DLL_STATUS) & 0x7FF;
}

static void _digital_dll_enable_rs(u32 channel1_enabled)
{
	EMC(EMC_CFG_DIG_DLL) = (EMC(EMC_CFG_DIG_DLL) & 0xFFFFFF24) | 0x89;

	emc_timing_update(channel1_enabled);

	while (!(EMC(EMC_CFG_DIG_DLL) & 1))
		;
	if (channel1_enabled)
		while (!(EMC_CH1(EMC_CFG_DIG_DLL) & 1))
			;
}

static u32 _dvfs_power_ramp_down(bool flip_backward, struct emc_table *src_emc_table_entry,
	struct emc_table *dst_emc_table_entry, u32 src_clock_period)
{
	u32 pmacro_cmd_pad;
	u32 pmacro_rfu1;
	u32 pmacro_cfg5;
	u32 pmacro_common_tx;
	u32 pmacro_dq_pad;
	u32 pmacro_cmd_pad_drvforceon;
	u32 ramp_down_wait;

	u32 src_clk_per_pc = (100000 / src_clock_period) + 1;

	if (flip_backward)
	{
		pmacro_cmd_pad = dst_emc_table_entry->burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad = dst_emc_table_entry->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1 = dst_emc_table_entry->burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5 = dst_emc_table_entry->burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = dst_emc_table_entry->burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	}
	else
	{
		pmacro_cmd_pad = src_emc_table_entry->burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad = (dst_emc_table_entry->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 0x101)
			| src_emc_table_entry->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1 = src_emc_table_entry->burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5 = src_emc_table_entry->burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = src_emc_table_entry->burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	}

	pmacro_cmd_pad_drvforceon = pmacro_cmd_pad | 0x4000000;

	ramp_down_wait = src_clock_period * 12;

	_ccfifo_write(EMC_PMACRO_CMD_PAD_TX_CTRL, pmacro_cmd_pad_drvforceon, 0);
	_ccfifo_write(EMC_FBIO_CFG5, pmacro_cfg5 | 0x100, 12);

	if (src_clock_period >= 1000) // Dvfs high speed threshold.
	{
		_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xF800F800, (u32)(src_clk_per_pc + 19));
		ramp_down_wait += 100000 + (src_clock_period * 20);
	}
	else
	{
		if (src_clock_period >= 416) // Iobrick dcc threshold.
			_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xFEEDFEED, (u32)src_clk_per_pc);
		else
		{
			pmacro_dq_pad = (pmacro_dq_pad & 0xFEFEFDFD) | 0x10200;
			pmacro_cmd_pad_drvforceon = (pmacro_cmd_pad & 0xFAFEFDFD) | 0x4010200;
			_ccfifo_write(EMC_PMACRO_CMD_PAD_TX_CTRL, pmacro_cmd_pad_drvforceon, (u32)src_clk_per_pc);
			_ccfifo_write(EMC_PMACRO_DATA_PAD_TX_CTRL, pmacro_dq_pad, 0);
			_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xFEEDFEED, 0);
		}
		ramp_down_wait += 300000;
		_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xFE40FE40, (u32)src_clk_per_pc);

		if (src_clock_period >= 416) // Iobrick dcc threshold.
			_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xF800F800, (u32)src_clk_per_pc);
		else
		{
			_ccfifo_write(EMC_PMACRO_CMD_PAD_TX_CTRL, pmacro_cmd_pad_drvforceon & 0xFEFEFDFD, (u32)src_clk_per_pc);
			_ccfifo_write(EMC_PMACRO_DATA_PAD_TX_CTRL, pmacro_dq_pad & 0xFEFEFDFD, 0);
			_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xF800F800, 0);
		}
	}

	if (src_clock_period >= 1666) // Dvfs mid speed threshold.
		_ccfifo_write(EMC_PMACRO_COMMON_PAD_TX_CTRL, pmacro_common_tx & 0xFFFFFFF0, (u32)src_clk_per_pc);
	else
	{
		ramp_down_wait += 400000;
		_ccfifo_write(EMC_PMACRO_COMMON_PAD_TX_CTRL, pmacro_common_tx & 0xFFFFFFFA, (u32)src_clk_per_pc);
		_ccfifo_write(EMC_PMACRO_COMMON_PAD_TX_CTRL, pmacro_common_tx & 0xFFFFFFF0, (u32)src_clk_per_pc);
		_ccfifo_write(EMC_INTSTATUS, 0, (u32)src_clk_per_pc);
	}

	return ramp_down_wait;
}

static u32 _dvfs_power_ramp_up(bool flip_backward, struct emc_table *src_emc_table_entry,
	struct emc_table *dst_emc_table_entry, u8 needs_training, u32 dst_clock_period)
{
	u32 pmacro_cmd_pad;
	u32 pmacro_dq_pad;
	u32 pmacro_rfu1;
	u32 pmacro_cfg5;
	u32 pmacro_common_tx;
	u32 pmacro_cmd_pad_data;
	u32 ramp_up_wait = 0;

	u32 dst_clk_per_pc = (100000 / dst_clock_period) + 1;

	if (flip_backward)
	{
		pmacro_cmd_pad = src_emc_table_entry->burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad = src_emc_table_entry->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1 = src_emc_table_entry->burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5 = src_emc_table_entry->burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = src_emc_table_entry->burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	}
	else
	{
		pmacro_cmd_pad = dst_emc_table_entry->burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad = dst_emc_table_entry->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1 = dst_emc_table_entry->burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5 = dst_emc_table_entry->burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = dst_emc_table_entry->burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	}

	pmacro_cmd_pad_data = (pmacro_cmd_pad & 0xFAFEFDFD) | 0x4000000;

	if (dst_clock_period >= 1666) // Dvfs mid speed threshold.
	{
		_ccfifo_write(EMC_PMACRO_COMMON_PAD_TX_CTRL, pmacro_common_tx | 8, 0);

		_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 | 0x600, 0);
		_ccfifo_write(EMC_FBIO_CFG5, pmacro_cfg5 & 0xFFFFFEFF, 12);

		ramp_up_wait = (dst_clock_period * 12) + 0;
	}
	else
	{
		_ccfifo_write(EMC_PMACRO_COMMON_PAD_TX_CTRL, pmacro_common_tx & 0xA, 0);
		_ccfifo_write(EMC_PMACRO_COMMON_PAD_TX_CTRL, pmacro_common_tx & 0xF, dst_clk_per_pc);

		if (dst_clock_period < 1000) // Dvfs high speed threshold.
		{
			if (dst_clock_period >= 416) // Iobrick dcc threshold.
				_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xFE40FE40, dst_clk_per_pc);
			else
			{
				_ccfifo_write(EMC_PMACRO_CMD_PAD_TX_CTRL, (pmacro_cmd_pad & 0xFAFEFDFD) | 0x4010200, dst_clk_per_pc);
				_ccfifo_write(EMC_PMACRO_DATA_PAD_TX_CTRL, (pmacro_dq_pad & 0xFEFEFDFD) | 0x10200, 0);
				_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xFE40FE40, 0);
			}

			_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 & 0xFEEDFEED, dst_clk_per_pc);

			if (dst_clock_period >= 416) // Iobrick dcc threshold.
				_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1, dst_clk_per_pc);
			else
			{
				pmacro_cmd_pad_data = pmacro_cmd_pad | 0x5010202;
				_ccfifo_write(EMC_PMACRO_CMD_PAD_TX_CTRL, pmacro_cmd_pad_data, dst_clk_per_pc);
				_ccfifo_write(EMC_PMACRO_DATA_PAD_TX_CTRL, pmacro_dq_pad | 0x1010202, 0);
				_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1, 0);
			}

			ramp_up_wait = 500000 + (dst_clock_period * 10);
		}
		else // 1000 > dst_clock_period < 1666.
		{
			_ccfifo_write(EMC_PMACRO_BRICK_CTRL_RFU1, pmacro_rfu1 | 0x6000600, dst_clk_per_pc);

			ramp_up_wait = 200000 + (dst_clock_period * 10);
		}

		_ccfifo_write(EMC_FBIO_CFG5, pmacro_cfg5 & 0xFFFFFEFF, dst_clk_per_pc + 9);
	}

	_ccfifo_write(EMC_PMACRO_CMD_PAD_TX_CTRL, pmacro_cmd_pad_data & 0xFBFFFFFF, 5);

	return ramp_up_wait;
}

static u32 _minerva_update_clock_tree_delay(struct emc_table *src_emc_entry,
	struct emc_table *dst_emc_entry, u32 dram_dev_num, u32 channel1_enabled, u32 update_type)
{
	u32 cval = 0;
	u32 adelta = 0;
	s32 tdelta = 0;

	u32 temp_ch0_0 = 0;
	u32 temp_ch0_1 = 0;
	u32 temp_ch1_0 = 0;
	u32 temp_ch1_1 = 0;

	u32 upd_type_bits = 1 << update_type;
	u32 dst_rate_mhz = dst_emc_entry->rate / 1000;
	u32 src_rate_mhz = src_emc_entry->rate / 1000;

	u32 tval = 1000000 * tegra210_actual_osc_clocks(src_emc_entry->run_clocks) / 2;

	if (update_type > PERIODIC_TRAINING_UPDATE)
		return 0;

	if (upd_type_bits & 0x5400)
	{
		_request_mmr_data(0x80130000, channel1_enabled); // Dev0 MRR 19.
		temp_ch0_0 = (EMC(EMC_MRR) & 0xFF) << 8;
		temp_ch0_1 = EMC(EMC_MRR) & 0xFF00;
		if (channel1_enabled)
		{
			temp_ch1_0 = (EMC_CH1(EMC_MRR) & 0xFF) << 8;
			temp_ch1_1 = EMC_CH1(EMC_MRR) & 0xFF00;
		}

		_request_mmr_data(0x80120000, channel1_enabled); // Dev0 MRR 18.
		temp_ch0_0 |= EMC(EMC_MRR) & 0xFF;
		temp_ch0_1 |= (EMC(EMC_MRR) & 0xFF00) >> 8;
		if (channel1_enabled)
		{
			temp_ch1_0 |= EMC_CH1(EMC_MRR) & 0xFF;
			temp_ch1_1 |= (EMC_CH1(EMC_MRR) & 0xFF00) >> 8;
		}
	}

	cval = tval / (src_rate_mhz * temp_ch0_0);
	switch (update_type)
	{
	case DVFS_PT1:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] += 100 * cval;
		if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
			goto calc_td0_0;
		break;
	case DVFS_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] =
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
		break;
	case PERIODIC_TRAINING_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] =
			(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX])
			/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
		break;
	}

	tdelta = dst_emc_entry->current_dram_clktree_c0d0u0 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] / 100);
	if (tdelta < 0)
		tdelta *= -1;
	adelta = tdelta;
	if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
		dst_emc_entry->current_dram_clktree_c0d0u0 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] / 100;

calc_td0_0:
	cval = tval / (src_rate_mhz * temp_ch0_1);
	switch (update_type)
	{
	case DVFS_PT1:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] += 100 * cval;
		if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
			goto calc_td1_0;
		break;
	case DVFS_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] =
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
		break;
	case PERIODIC_TRAINING_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] =
			(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX])
			/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
		break;
	}

	tdelta = dst_emc_entry->current_dram_clktree_c0d0u1 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] / 100);
	if (tdelta < 0)
		tdelta *= -1;
	if ((u32)tdelta > adelta)
		adelta = tdelta;
	if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
		dst_emc_entry->current_dram_clktree_c0d0u1 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] / 100;

calc_td1_0:
	if (channel1_enabled)
	{
		cval = tval / (src_rate_mhz * temp_ch1_0);
		switch (update_type)
		{
		case DVFS_PT1:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] += 100 * cval;
			if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
				goto calc_td1_1;
			break;
		case DVFS_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] =
				dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
			break;
		case PERIODIC_TRAINING_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] =
				(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX])
				/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
			break;
		}

		tdelta = dst_emc_entry->current_dram_clktree_c1d0u0 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] / 100);
		if (tdelta < 0)
			tdelta *= -1;
		if ((u32)tdelta > adelta)
			adelta = tdelta;
		if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
			dst_emc_entry->current_dram_clktree_c1d0u0 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] / 100;

calc_td1_1:
		cval = tval / (src_rate_mhz * temp_ch1_1);
		switch (update_type)
		{
		case DVFS_PT1:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] += 100 * cval;
			if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
				goto calc_dev2;
			break;
		case DVFS_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] =
				dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
			break;
		case PERIODIC_TRAINING_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] =
				(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX])
				/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
			break;
		}

		tdelta = dst_emc_entry->current_dram_clktree_c1d0u1 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] / 100);
		if (tdelta < 0)
			tdelta *= -1;
		if ((u32)tdelta > adelta)
			adelta = tdelta;
		if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
			dst_emc_entry->current_dram_clktree_c1d0u1 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] / 100;
	}

calc_dev2:
	if (dram_dev_num != TWO_RANK)
		goto out;

	if (update_type <= PERIODIC_TRAINING_UPDATE && upd_type_bits & 0x5400)
	{
		_request_mmr_data(0x40130000, channel1_enabled); // Dev1 MRR 19.
		temp_ch0_0 = (EMC(EMC_MRR) & 0xFF) << 8;
		temp_ch0_1 = EMC(EMC_MRR) & 0xFF00;
		if (channel1_enabled)
		{
			temp_ch1_0 = (EMC_CH1(EMC_MRR) & 0xFF) << 8;
			temp_ch1_1 = EMC_CH1(EMC_MRR) & 0xFF00;
		}

		_request_mmr_data(0x40120000, channel1_enabled); // Dev1 MRR 18
		temp_ch0_0 |= EMC(EMC_MRR) & 0xFF;
		temp_ch0_1 |= ((EMC(EMC_MRR) & 0xFF00) >> 8);
		if (channel1_enabled)
		{
			temp_ch1_0 |= EMC_CH1(EMC_MRR) & 0xFF;
			temp_ch1_1 |= (EMC_CH1(EMC_MRR) & 0xFF00) >> 8;
		}
	}

	cval = tval / (src_rate_mhz * temp_ch0_0);
	switch (update_type )
	{
	case DVFS_PT1:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] += 100 * cval;
		if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
			goto calc_tmp_td0_1;
		break;
	case DVFS_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] =
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
		break;
	case PERIODIC_TRAINING_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] =
			(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX])
			/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
		break;
	}

	tdelta = dst_emc_entry->current_dram_clktree_c0d1u0 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] / 100);
	if (tdelta < 0)
		tdelta *= -1;
	if ((u32)tdelta > adelta)
		adelta = tdelta;
	if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
		dst_emc_entry->current_dram_clktree_c0d1u0 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] / 100;

calc_tmp_td0_1:
	cval = tval / (src_rate_mhz * temp_ch0_1);
	switch (update_type)
	{
	case DVFS_PT1:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] += 100 * cval;
		if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
			goto calc_tmp_td1_0;
		break;
	case DVFS_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] =
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
		break;
	case PERIODIC_TRAINING_UPDATE:
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] =
			(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX])
			/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
		break;
	}

	tdelta = dst_emc_entry->current_dram_clktree_c0d1u1 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] / 100);
	if (tdelta < 0)
		tdelta *= -1;
	if ((u32)tdelta > adelta)
		adelta = tdelta;
	if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
		dst_emc_entry->current_dram_clktree_c0d1u1 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] / 100;

calc_tmp_td1_0:
	if (channel1_enabled)
	{
		cval = tval / (src_rate_mhz * temp_ch1_0);
		switch (update_type)
		{
		case DVFS_PT1:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] += 100 * cval;
			if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
				goto calc_tmp_td1_1;
			break;
		case DVFS_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] =
				dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
			break;
		case PERIODIC_TRAINING_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] =
				(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX])
				/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
			break;
		}

		tdelta = dst_emc_entry->current_dram_clktree_c1d1u0 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] / 100);
		if (tdelta < 0)
			tdelta *= -1;
		if ((u32)tdelta > adelta)
			adelta = tdelta;
		if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
			dst_emc_entry->current_dram_clktree_c1d1u0 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] / 100;

calc_tmp_td1_1:
		cval = tval / (src_rate_mhz * temp_ch1_1);
		switch (update_type)
		{
		case DVFS_PT1:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] += 100 * cval;
			if (update_type > PERIODIC_TRAINING_UPDATE || !(upd_type_bits & 0x6800))
				goto out;
			break;
		case DVFS_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] =
				dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] / dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
			break;
		case PERIODIC_TRAINING_UPDATE:
			dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] =
				(100 * cval + dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] * dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX])
				/ (dst_emc_entry->ptfv_list[PTFV_MOVAVG_WEIGHT_INDEX] + 1);
			break;
		}

		tdelta = dst_emc_entry->current_dram_clktree_c1d1u1 - (dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] / 100);
		if (tdelta < 0)
			tdelta *= -1;
		if ((u32)tdelta > adelta)
			adelta = tdelta;
		if (((dst_rate_mhz * tdelta * 128) / 1000000) > dst_emc_entry->tree_margin)
			dst_emc_entry->current_dram_clktree_c1d1u1 = dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] / 100;
	}

out:

	return (u32)adelta;
}


static u32 _minerva_periodic_compensation_handler(struct emc_table *src_emc_entry,
	struct emc_table *dst_emc_entry, u32 dram_dev_num, u32 channel1_enabled)
{
	u32 delay;
	u32 i;

	if (!dst_emc_entry->periodic_training)
		return 0;

	delay = 1000 * tegra210_actual_osc_clocks(src_emc_entry->run_clocks) / src_emc_entry->rate + 2;

	if (src_emc_entry->periodic_training && dst_emc_entry->ptfv_list[PTFV_CONFIG_CTRL_INDEX] & PTFV_CONFIG_CTRL_USE_PREVIOUS_EMA)
	{
		u32 samples = dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX];
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] * samples;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] = src_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] * samples;
	}
	else
	{
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U0_INDEX] = 0;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U0_INDEX] = 0;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U0_INDEX] = 0;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U0_INDEX] = 0;
		udelay(1); // Avoid unaligned access on O2.
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D0U1_INDEX] = 0;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D0U1_INDEX] = 0;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C0D1U1_INDEX] = 0;
		dst_emc_entry->ptfv_list[PTFV_DQSOSC_MOVAVG_C1D1U1_INDEX] = 0;

		for (i = 0; i < dst_emc_entry->ptfv_list[PTFV_DVFS_SAMPLES_INDEX]; i++)
		{
			tegra210_start_periodic_compensation();
			udelay(delay);
			_minerva_update_clock_tree_delay(src_emc_entry, dst_emc_entry, dram_dev_num, channel1_enabled, DVFS_PT1);
		}
	}

	return _minerva_update_clock_tree_delay(src_emc_entry, dst_emc_entry, dram_dev_num, channel1_enabled, DVFS_UPDATE);
}

u32 __do_periodic_emc_compensation_icosa(
			struct emc_table *current_timing)
{
	u32 dram_dev_num, adel, i, pd_mask;
	u32 emc_cfg_o, emc_cfg_dig_dll_o, emc_cfg_update_o;
	bool channel1_enabled;

	if (current_timing->periodic_training) {
		dram_dev_num = (MC(MC_EMEM_ADR_CFG) & 1) + 1;
		pd_mask = (dram_dev_num == TWO_RANK) ? EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK : 0x10;
		channel1_enabled = (current_timing->burst_regs[EMC_FBIO_CFG7_INDEX] >> 2) & 1;

		emc_cc_dbg(PER_TRAIN, "Periodic training starting\n");
		emc_readl(EMC_DBG); /* Flush */

		/* Safekeep current config. */
		emc_cfg_o = EMC(EMC_CFG);
		emc_cfg_dig_dll_o = EMC(EMC_CFG_DIG_DLL);
		emc_cfg_update_o = EMC(EMC_CFG_UPDATE);

		/* 1. Disable digital DLL. */
		EMC(EMC_CFG_DIG_DLL) = emc_cfg_dig_dll_o & 0xFFFFFFFE;

		/* 1.2. Always update auto cal in clock change. */
		EMC(EMC_CFG_UPDATE) = (emc_cfg_update_o & 0xFFFFF9FF) | 0x400;

		/* 1.3. Disable other power features. */
		EMC(EMC_CFG) = emc_cfg_o & 0xFFFFFFF;

		/* Timing update and wait for everything to power down. */
		emc_timing_update(channel1_enabled);

		_wait_emc_status(EMC_EMC_STATUS, pd_mask, 0, EMC_CHANNEL0);
		if (channel1_enabled)
			_wait_emc_status(EMC_EMC_STATUS, pd_mask, 0, EMC_CHANNEL1);

		_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK, 0, EMC_CHANNEL0);
		if (channel1_enabled)
			_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK, 0, EMC_CHANNEL1);


		_wait_emc_status(EMC_CFG_DIG_DLL, 1, 0, EMC_CHANNEL0);
		if (channel1_enabled)
			_wait_emc_status(EMC_CFG_DIG_DLL, 1, 0, EMC_CHANNEL1);

		/*
		 * 2. osc kick off - this assumes training and dvfs have set
		 *    correct MR23.
		 */
		tegra210_start_periodic_compensation();

		/*
		 * 3. Let dram capture its clock tree delays.
		 */
		udelay(1000 * tegra210_actual_osc_clocks(current_timing->run_clocks) /
			current_timing->rate + 2);

		/*
		 * 4. Check delta wrt previous values (save value if margin
		 *    exceeds what is set in table).
		 */
		adel = _minerva_update_clock_tree_delay(current_timing, current_timing, dram_dev_num, channel1_enabled, PERIODIC_TRAINING_UPDATE);

		/*
		 * 5. Apply compensation w.r.t. trained values (if clock tree
		 *    has drifted more than the set margin).
		 */
		if (adel && ((current_timing->rate  / 1000) * 128) * adel / 1000000 >
			current_timing->tree_margin)
		{
			for (i = 0; i < 10; i++)
			{
				EMC(periodic_training_addr[i]) =
					tegra210_apply_periodic_compensation_trimmer(current_timing, periodic_training_addr[i]);
			}
		}

		/* 6. Restore other power features. */
		EMC(EMC_CFG) = emc_cfg_o;

		/* 6.1. Restore the DLL. */
		 EMC(EMC_CFG_DIG_DLL) = emc_cfg_dig_dll_o;

		/* 6.2. Timing update for applying the new trimmers. */
		emc_timing_update(channel1_enabled);

		/* 6.3. Restore the UPDATE_DLL_IN_UPDATE field. */
		EMC(EMC_CFG_UPDATE) = emc_cfg_update_o;

		/*
		 * 7. Copy over the periodic training registers that we updated
		 *    here to the corresponding derated/non-derated table.
		 */
		tegra210_update_emc_alt_timing(current_timing);
	}

	return 0;
}

/*
 * Do the clock change sequence.
 */
void emc_set_clock_icosa(struct emc_table *next_timing,
			  struct emc_table *last_timing,
			  int training, u32 clksrc)
{
	u32 adel;
	u32 emc_dbg_o;
	u32 emc_pin_o;
	u32 emc_cfg_pipe_clk_o;
	u32 emc_sel_dpd_ctrl;
	u32 emc_cfg;
	u32 emc_dbg_val;
	u32 emc_zq_cal = 0;
	u32 ramp_up_wait;
	u32 ramp_down_wait;
	u32 bg_regulator_mode_change;
	u32 mr13_flip_fspop = 0;
	u32 mr13_flip_fspwr = 0;
	u32 mr13_catr_enable = 0;
	u32 tFC_lpddr4;
	u32 R2P_war;
	u32 TRPab_war;
	u32 RP_war;
	u32 W2P_war;
	u32 deltaTWATM;
	u32 tRTM;
	u32 reg_addr;
	u32 reg_val;
	u32 reg_check;
	u32 tRP_src_timing;
	u32 ref_delay;
	u32 emc_pin_val_final;
	u32 i;
	u32 dll_out;
	u32 bg_regulator_switch_complete_wait_clks;

	bool compensate_trimmer_applicable;
	bool zcal_resistor_shared;
	bool enable_bg_regulator;
	bool channel1_enabled;
	bool dual_channel;

	u32 dram_type;
	u32 dram_dev_num;
	u32 tZQCAL_lpddr4;
	s32 tZQCAL_lpddr4_fc_adj;
	u32 nRTP;
	u32 tRPST;
	s32 zq_latch_dvfs_wait_time;
	s32 T_PDEX_timing_final;
	u32 T_PDEX_timing;

	u32 src_clock_period;
	u32 dst_clock_period;

	static bool fsp_for_src_freq;

	compensate_trimmer_applicable = false;
	emc_zq_cal = 0;
	mr13_flip_fspop = 0;
	mr13_flip_fspwr = 0;
	mr13_catr_enable = 0;

	emc_cc_dbg(INFO, "Running Minerva Training Cell mini.\n");

	zcal_resistor_shared = (last_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX] >> 31) & 1;
	enable_bg_regulator = (next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] & 1) ^ 1;
	channel1_enabled = (last_timing->burst_regs[EMC_FBIO_CFG7_INDEX] >> 2) & 1;
	dram_type = EMC(EMC_FBIO_CFG5) & 3;
	dram_dev_num = (MC(MC_EMEM_ADR_CFG) & 1) + 1;

	src_clock_period = 1000000000 / last_timing->rate ; // In picoseconds.
	dst_clock_period = 1000000000 / next_timing->rate ; // In picoseconds.

	if (WARN(dram_type != DRAM_TYPE_LPDDR4, "MTC Error: DRAM is not LPDDR4\n"))
		return;

	fsp_for_src_freq = !fsp_for_src_freq;

	tFC_lpddr4 = next_timing->dram_timings[T_FC_LPDDR4] * 1000;
	tZQCAL_lpddr4 = 1000000;
	if (dst_clock_period <= 2000)
		tZQCAL_lpddr4 -= tFC_lpddr4;
	tZQCAL_lpddr4_fc_adj = tZQCAL_lpddr4 / dst_clock_period;

	emc_cc_dbg(INFO, "Clock change version: %s\n",
		   DVFS_CLOCK_CHANGE_VERSION);
	emc_cc_dbg(INFO, "EMC table revision: %d\n",
		    next_timing->rev);
	emc_cc_dbg(INFO, "DVFS Version: %s\n",
		    next_timing->dvfs_ver);
	emc_cc_dbg(INFO, "DRAM type = %d\n", dram_type);
	emc_cc_dbg(INFO, "DRAM dev #: %d\n", dram_dev_num);
	emc_cc_dbg(INFO, "Next EMC clksrc: 0x%08x\n", clksrc);
	emc_cc_dbg(INFO, "DLL clksrc:      0x%08x\n", next_timing->dll_clk_src);
	emc_cc_dbg(INFO, "last rate: %u, next rate %u\n", last_timing->rate,
		   next_timing->rate);
	emc_cc_dbg(INFO, "last period: %u, next period: %u\n",
		   src_clock_period, dst_clock_period);
	emc_cc_dbg(INFO, "  shared_zq_resistor: %d\n", !!zcal_resistor_shared);
	emc_cc_dbg(INFO, "  channel_mode: %d\n", channel1_enabled);
	
	emc_readl(EMC_CFG); /* Flush */
	emc_readl(EMC_AUTO_CAL_CONFIG); /* Flush */

	/* Step 1:
	 *   Pre DVFS SW sequence.
	 */
	emc_cc_dbg(STEPS, "Step 1\n");
	emc_cc_dbg(STEPS, "Step 1.1: Disable DLL temporarily.\n");
	emc_dbg_o = EMC(EMC_DBG);
	emc_pin_o = EMC(EMC_PIN);
	emc_cfg = next_timing->burst_regs[EMC_CFG_INDEX] & 0xFFFFFFF;
	emc_sel_dpd_ctrl = next_timing->emc_sel_dpd_ctrl & 0xFFFFFEC3;
	emc_cfg_pipe_clk_o = EMC(EMC_CFG_PIPE_CLK);
	tegra210_dll_disable(channel1_enabled);

	emc_cc_dbg(STEPS, "Step 1.2: Disable AUTOCAL temporarily.\n");
	EMC(EMC_AUTO_CAL_CONFIG) = (next_timing->emc_auto_cal_config & 0x7FFFF9FF) | 0x600;
	emc_readl(EMC_AUTO_CAL_CONFIG); /* Flush write. */

	emc_cc_dbg(STEPS, "Step 1.3: Disable other power features.\n");
	EMC(EMC_DBG) = emc_dbg_o | 2;
	EMC(EMC_CFG) = emc_cfg;
	EMC(EMC_SEL_DPD_CTRL) = emc_sel_dpd_ctrl;
	EMC(EMC_DBG) = emc_dbg_o;

	if (next_timing->periodic_training) {
		if (dram_dev_num == TWO_RANK)
		{
			_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK, false, EMC_CHANNEL0);
			if (channel1_enabled)
				_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK, false, EMC_CHANNEL1);
		}
		else
		{
			_wait_emc_status(EMC_EMC_STATUS, 0x10, false, EMC_CHANNEL0);
			if (channel1_enabled)
				_wait_emc_status(EMC_EMC_STATUS, 0x10, false, EMC_CHANNEL1);
		}

		_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK, false, EMC_CHANNEL0);
		if (channel1_enabled)
			_wait_emc_status(EMC_EMC_STATUS, EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK, false, EMC_CHANNEL1);

		// Reset clock tree delays.
		next_timing->current_dram_clktree_c0d0u0 = next_timing->trained_dram_clktree_c0d0u0;
		next_timing->current_dram_clktree_c0d0u1 = next_timing->trained_dram_clktree_c0d0u1;
		next_timing->current_dram_clktree_c0d1u0 = next_timing->trained_dram_clktree_c0d1u0;
		next_timing->current_dram_clktree_c0d1u1 = next_timing->trained_dram_clktree_c0d1u1;
		next_timing->current_dram_clktree_c1d0u0 = next_timing->trained_dram_clktree_c1d0u0;
		next_timing->current_dram_clktree_c1d0u1 = next_timing->trained_dram_clktree_c1d0u1;
		next_timing->current_dram_clktree_c1d1u0 = next_timing->trained_dram_clktree_c1d1u0;
		next_timing->current_dram_clktree_c1d1u1 = next_timing->trained_dram_clktree_c1d1u1;

		adel = _minerva_periodic_compensation_handler(last_timing, next_timing, dram_dev_num, channel1_enabled);

		if (((next_timing->rate  / 1000) * 128) * adel / 1000000 > next_timing->tree_margin)
			compensate_trimmer_applicable = true;
	}

	EMC(EMC_INTSTATUS) = EMC_INTSTATUS_CLKCHANGE_COMPLETE;
	EMC(EMC_DBG) = emc_dbg_o | 2;
	EMC(EMC_CFG) = emc_cfg;
	EMC(EMC_SEL_DPD_CTRL) = emc_sel_dpd_ctrl;
	EMC(EMC_CFG_PIPE_CLK) = emc_cfg_pipe_clk_o | 1; // CLK_ALWAYS_ON.
	EMC(EMC_FDPD_CTRL_CMD_NO_RAMP) = next_timing->emc_fdpd_ctrl_cmd_no_ramp & 0xFFFFFFFE;

	bg_regulator_mode_change = last_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] ^
		next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX];
	bg_regulator_mode_change = (bg_regulator_mode_change | (bg_regulator_mode_change >> 2)) & 1;

	if (bg_regulator_mode_change)
	{
		EMC(EMC_DBG) = emc_dbg_o | 2;
		if (enable_bg_regulator)
			EMC(EMC_PMACRO_BG_BIAS_CTRL_0) = last_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] & 0xFFFFFFFE;
		else
			EMC(EMC_PMACRO_BG_BIAS_CTRL_0) = last_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] & 0xFFFFFFFB;
	}

	// Check if we need to turn on VREF generator.
	if ((!(last_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 0x100)
		&& (next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 0x100))
		|| (!(last_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 1)
		&& (next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 1)))
	{
		EMC(EMC_PMACRO_DATA_PAD_TX_CTRL) =
			(((next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 1)
				| (last_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] & 0xFFFFFFFE)) & 0xFFFFFEFF)
			| (((next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] >> 8) & 0x1) << 8);
	}

	udelay(1);

	EMC(EMC_DBG) = emc_dbg_o;

	/* Step 2:
	 *   Prelock the DLL.
	 */
	emc_cc_dbg(STEPS, "Step 2\n");
	if (next_timing->burst_regs[EMC_CFG_DIG_DLL_INDEX] & EMC_CFG_DIG_DLL_CFG_DLL_EN)
	{
		emc_cc_dbg(INFO, "Prelock enabled for target frequency.\n");
		dll_out = _digital_dll_prelock(next_timing, clksrc);
		emc_cc_dbg(INFO, "DLL out: 0x%03x\n", dll_out);
	}
	else
	{
		emc_cc_dbg(INFO, "Disabling DLL for target frequency.\n");
		tegra210_change_dll_src(next_timing, clksrc);
		tegra210_dll_disable(channel1_enabled);
	}

	/* Step 3:
	 *   Prepare autocal for the clock change.
	 */
	emc_cc_dbg(STEPS, "Step 3\n");
	EMC(EMC_AUTO_CAL_CONFIG) = (next_timing->emc_auto_cal_config & 0x7FFFF9FF) | 0x600;
	EMC(EMC_DBG) = emc_dbg_o | 2;
	EMC(EMC_AUTO_CAL_CONFIG2) = next_timing->emc_auto_cal_config2;
	EMC(EMC_AUTO_CAL_CONFIG3) = next_timing->emc_auto_cal_config3;
	EMC(EMC_AUTO_CAL_CONFIG4) = next_timing->emc_auto_cal_config4;
	EMC(EMC_AUTO_CAL_CONFIG5) = next_timing->emc_auto_cal_config5;
	EMC(EMC_AUTO_CAL_CONFIG6) = next_timing->emc_auto_cal_config6;
	EMC(EMC_AUTO_CAL_CONFIG7) = next_timing->emc_auto_cal_config7;
	EMC(EMC_AUTO_CAL_CONFIG8) = next_timing->emc_auto_cal_config8;
	EMC(EMC_DBG) = emc_dbg_o;
	EMC(EMC_AUTO_CAL_CONFIG) = (next_timing->emc_auto_cal_config & 0x7FFFF9FE) | 0x601;

	/* Step 4:
	 *   Update EMC_CFG. (??)
	 */
	emc_cc_dbg(STEPS, "Step 4\n");
	if (src_clock_period <= 50000)
		EMC(EMC_CFG_2) = next_timing->emc_cfg_2;
	else
		_ccfifo_write(EMC_SELF_REF, 1, 0);

	/* Step 5:
	 *   Prepare reference variables for ZQCAL regs - removed.
	 */

	/* Step 6:
	 *   Training code - removed.
	 */

	/* Step 7:
	 *   Program FSP reference registers and send MRWs to new FSPWR.
	 */
	emc_cc_dbg(STEPS, "Step 7\n");
	emc_cc_dbg(SUB_STEPS, "Step 7.1: Bug 200024907 - Patch RP R2P\n");
	R2P_war = 0;
	TRPab_war = 0;
	RP_war = 0;
	W2P_war = 0;

	nRTP = 8;  // <= 1066MHz.
	if (    src_clock_period < 1000000 /  266  // 266MHz  - 3759.39 ps.
		 && src_clock_period < 1000000 /  533  // 533MHz  - 1876.17 ps.
		 && src_clock_period < 1000000 /  800  // 800MHz  - 1250.00 ps.
		 && src_clock_period < 1000000 / 1066) // 1066MHz - 938.09 ps.
		nRTP = 10; // 1067MHz < x <= 1333MHz.
	if (src_clock_period < 1000000 / 1333)     // 1333MHz - 750.19 ps.
		nRTP = 12; // 1333MHz < x <= 1600MHz.
	if (src_clock_period < 1000000 / 1600)     // 1600MHz - 625.00 ps.
		nRTP = 14; // 1600MHz < x <= 1866MHz.
	if (src_clock_period < 1000000 / 1866)     // 1866MHz - 535.91 ps.
		nRTP = 16; // > 1866MHz

	tRPST = (last_timing->emc_mrw >> 7) & 1;

	deltaTWATM = max_t(u32, div_o3(7500, src_clock_period), 8);

	tRTM = last_timing->dram_timings[RL] + div_o3(3600, src_clock_period) + deltaTWATM + tRPST + nRTP + 1;

	emc_cc_dbg(INFO, "tRTM = %u, EMC_RP = %u\n", tRTM, last_timing->burst_regs[EMC_RP_INDEX]);

	if (tRTM <= last_timing->burst_regs[EMC_RP_INDEX] + last_timing->burst_regs[EMC_R2P_INDEX])
	{
		TRPab_war = last_timing->burst_regs[EMC_TRPAB_INDEX];
		R2P_war = last_timing->burst_regs[EMC_R2P_INDEX];
		RP_war = last_timing->burst_regs[EMC_RP_INDEX];
	}
	else
	{
		R2P_war = tRTM - last_timing->burst_regs[EMC_RP_INDEX];
		TRPab_war = last_timing->burst_regs[EMC_TRPAB_INDEX];
		RP_war = last_timing->burst_regs[EMC_RP_INDEX];
		if (R2P_war > 63)
		{
			RP_war = tRTM - 63;
			R2P_war = 63;
			if (last_timing->burst_regs[EMC_TRPAB_INDEX] < tRTM - 63)
				TRPab_war = tRTM - 63;
			else
				TRPab_war = last_timing->burst_regs[EMC_TRPAB_INDEX];
		}
	}

	if (RP_war >= deltaTWATM)
		W2P_war = last_timing->burst_regs[EMC_W2P_INDEX];
	else
	{
		u32 W2P_war_temp = deltaTWATM + last_timing->burst_regs[EMC_W2P_INDEX];
		W2P_war = W2P_war_temp - RP_war;
		if (W2P_war > 63)
		{
			RP_war = W2P_war_temp - 63;
			W2P_war = 63;
			if (TRPab_war < RP_war)
				TRPab_war = RP_war;
		}
	}

	if ( last_timing->burst_regs[EMC_W2P_INDEX] != W2P_war
		|| last_timing->burst_regs[EMC_RP_INDEX] != RP_war
		|| last_timing->burst_regs[EMC_R2P_INDEX] != R2P_war
		|| last_timing->burst_regs[EMC_TRPAB_INDEX] != TRPab_war)
	{
		EMC(EMC_DBG) = emc_dbg_o | 2;
		EMC(EMC_RP) = RP_war;
		EMC(EMC_R2P) = R2P_war;
		EMC(EMC_W2P) = W2P_war;
		EMC(EMC_TRPAB) = TRPab_war;
		EMC(EMC_DBG) = emc_dbg_o;
		emc_readl(EMC_TRPAB); /* Flush write. */
		udelay(1);
	}

	emc_cc_dbg(SUB_STEPS, "Step 7.2: Program FSP reference registers and send MRWs to new FSPWR\n");
	if (fsp_for_src_freq)
	{
		mr13_flip_fspop = next_timing->emc_mrw3 | 0xC0;
		mr13_flip_fspwr = (next_timing->emc_mrw3 & 0xFFFFFF3F) | 0x40;
	}
	else
	{
		mr13_flip_fspop = next_timing->emc_mrw3 & 0xFFFFFF3F;
		mr13_flip_fspwr = mr13_flip_fspop | 0x80;
	}

	if (dram_dev_num == TWO_RANK)
		mr13_catr_enable = (mr13_flip_fspwr & 0x3FFFFFFF) | 0x80000001;
	else
		mr13_catr_enable = mr13_flip_fspwr | 1;

	EMC(EMC_MRW3) = mr13_flip_fspwr;
	EMC(EMC_MRW) = next_timing->emc_mrw;
	EMC(EMC_MRW2) = next_timing->emc_mrw2;

	/* Step 8:
	 *   Program the shadow registers.
	 */
	emc_cc_dbg(STEPS, "Step 8\n");
	emc_cc_dbg(SUB_STEPS, "Writing burst_regs\n");
	reg_addr = 0;
	reg_val = 0;
	reg_check = false;

	for (i = 0; next_timing->num_burst > i; i++)
	{
		reg_check = false;
		reg_addr = burst_regs_emc_addr_table[i];
		reg_val = next_timing->burst_regs[i];

		if ((reg_addr & 0xFFF7) != EMC_MRW6
			&& (reg_addr - EMC_MRW7) & 0xFFFF7
			//&& (reg_addr & 0xEFF7) != 0x34B4 // EMC_MRW10.
			&& ((reg_addr & 0xEFFF) - 0x34B8) & 0xFFF7 // EMC_MRW11.
			&& reg_addr != EMC_TRAINING_CTRL
			&& reg_addr != EMC_MRW14
			&& reg_addr != EMC_MRW15)
		{
			reg_check = true;
		}

		if (reg_check && reg_addr == EMC_CFG)
		{
			reg_val &= 0xFFFFFFF;

			EMC(reg_addr) = reg_val;
			continue;
		}

		if (reg_addr != EMC_CFG)// EMC_CFG
		{
			if (reg_addr != EMC_ZCAL_INTERVAL)
			{
				switch ( reg_addr )
				{
				case EMC_PMACRO_AUTOCAL_CFG_COMMON:
					reg_val |= 0x10000;
					break;
				case EMC_PMACRO_DATA_PAD_TX_CTRL:
					reg_val &= 0xFEFEFDFD;
					break;
				case EMC_PMACRO_CMD_PAD_TX_CTRL:
					reg_val = (reg_val & 0xFAFEFDFD) | 0x4000000;
					break;
				case EMC_PMACRO_BRICK_CTRL_RFU1:
					reg_val &= 0xF800F800;
					break;
				case EMC_PMACRO_COMMON_PAD_TX_CTRL:
					reg_val &= 0xFFFFFFF0;
					break;
				}
			}
			else
				reg_val = 0;
		}
		else
			reg_val &= 0xFFFFFFF;

		EMC(reg_addr) = reg_val;
	}

	/* SW addition: do EMC refresh adjustment here. */
	set_over_temp_timing(next_timing, dram_over_temp_state);

	EMC(EMC_MRW) = (next_timing->run_clocks & 0xFF) | 0x170000;

	/* Per channel burst registers. */
	emc_cc_dbg(SUB_STEPS, "Writing burst_regs_per_ch\n");
	for (i = 0; next_timing->num_burst_per_ch > i; i++)
	{
		reg_addr = burst_reg_per_ch_emc01_addr_table[i];
		if (reg_addr && (channel1_enabled || ((reg_addr - 0x4000) > 0xFFF)))
		{
			emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n",
				i, next_timing->burst_reg_per_ch[i], reg_addr);
			if (reg_addr < 0x4000)
			{
				reg_addr -= 0x3000;
				EMC_CH0(reg_addr) = next_timing->burst_reg_per_ch[i];
			}
			else
			{
				reg_addr -= 0x4000;
				EMC_CH1(reg_addr) = next_timing->burst_reg_per_ch[i];
			}
		}
	}

	/* Vref regs. */
	emc_cc_dbg(SUB_STEPS, "Writing vref_regs\n");
	for (i = 0; next_timing->vref_num > i; i++)
	{
		reg_addr = vref_perch_regs_emc01_addr_table[i];
		if (reg_addr && (channel1_enabled || (reg_addr - 0x4000) > 0xFFF))
		{
			emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n",
				i, next_timing->vref_perch_regs[i], reg_addr);
			if (reg_addr < 0x4000)
			{
				reg_addr -= 0x3000;
				EMC_CH0(reg_addr) = next_timing->vref_perch_regs[i];
			}
			else
			{
				reg_addr -= 0x4000;
				EMC_CH1(reg_addr) = next_timing->vref_perch_regs[i];
			}
		}
	}

	/* Trimmers. */
	emc_cc_dbg(SUB_STEPS, "Writing trim_regs\n");
	reg_val = 0;
	for (i = 0; next_timing->num_trim > i; i++)
	{
		reg_addr = trim_regs_emc_addr_table[i];
		if (reg_addr)
		{
			if (((reg_addr & 0xFFFFFFF3) == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0
				 || (reg_addr & 0xFFFFFFF3) == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0
				 || (reg_addr & 0xFFFFFFFB) == EMC_DATA_BRLSHFT_0)
				&& compensate_trimmer_applicable)
			{
				reg_val  = tegra210_apply_periodic_compensation_trimmer(next_timing, reg_addr);
				emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n", i, reg_val, reg_addr);
				emc_cc_dbg(EMA_WRITES, "0x%08x <= 0x%08x\n", (u32)(u64)reg_addr, reg_val);
				EMC(reg_addr) = reg_val;
			}
			else
			{
				emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n",
					i, next_timing->trim_regs[i], reg_addr);
				EMC(reg_addr) = next_timing->trim_regs[i];
			}
		}
	}

	/* Per channel trimmers. */
	emc_cc_dbg(SUB_STEPS, "Writing trim_regs_per_ch\n");
	reg_val = 0;
	for (i = 0; next_timing->num_trim_per_ch > i; i++)
	{
		reg_addr = trim_perch_regs_emc01_addr_table[i];
		if (reg_addr && (channel1_enabled || reg_addr - 0x4000 > 0xFFF))
		{
			if (((reg_addr & 0xFFFFFFF3) == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0
				 || (reg_addr & 0xFFFFFFF3) == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0
				 || (reg_addr & 0xFFFFFFFB) == EMC_DATA_BRLSHFT_0)
				&& compensate_trimmer_applicable )
			{
				reg_val = tegra210_apply_periodic_compensation_trimmer(next_timing, reg_addr & 0xFFF);
			}
			else if (((reg_addr & 0xFFFFFFFB) == 0x3660
				 || (reg_addr & 0xFFFFFFDF) == 0x3648
				 || (reg_addr & 0xFFFFFFF7) == 0x3644
				 || reg_addr == 0x366C
				 || reg_addr == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0
				 || (reg_addr & 0xFFFFFFFB) == 0x3588)
				&& compensate_trimmer_applicable )
			{
				reg_val = tegra210_apply_periodic_compensation_trimmer(next_timing, reg_addr & 0xFFF);
			}
			else if (((reg_addr & 0xFFFFFFF3) == 0x4660
				 || (reg_addr & 0xFFFFFFF3) == 0x4640
				 || (reg_addr & 0xFFFFFFFB) == 0x4588)
				&& compensate_trimmer_applicable)
			{
				reg_val = tegra210_apply_periodic_compensation_trimmer(next_timing, reg_addr & 0xFFF);
			}
			else
			{
				reg_val = next_timing->trim_perch_regs[i];
			}

			emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n",
				i, reg_val, reg_addr);

			if (reg_addr < 0x4000)
			{
				reg_addr -= 0x3000;
				EMC_CH0(reg_addr) = reg_val;
			}
			else
			{
				reg_addr -= 0x4000;
				EMC_CH1(reg_addr) = reg_val;
			}
		}
	}

	emc_cc_dbg(SUB_STEPS, "Writing burst_mc_regs\n");
	for (i = 0; next_timing->num_mc_regs > i; i++)
	{
		emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n",
			i, next_timing->burst_mc_regs[i], burst_mc_regs_addr_table[i]);
		MC(burst_mc_regs_addr_table[i]) = next_timing->burst_mc_regs[i];
	}

	/* Registers to be programmed on the faster clock. */
	if (next_timing->rate  < last_timing->rate )
	{
		emc_cc_dbg(SUB_STEPS, "Writing la_scale_regs\n");
		for (i = 0; next_timing->num_up_down > i; i++)
		{
			emc_cc_dbg(REG_LISTS, "(%u) 0x%08x => 0x%08x\n",
				i, next_timing->la_scale_regs[i], la_scale_regs_mc_addr_table[i]);
			MC(la_scale_regs_mc_addr_table[i]) = next_timing->la_scale_regs[i];
		}
	}

	/* Flush all the burst register writes. */
	wmb();

	/* Step 9:
	 *   LPDDR4 section A.
	 */
	emc_cc_dbg(STEPS, "Step 9\n");
	EMC(EMC_ZCAL_INTERVAL) = last_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] & 0xFF000000;
	EMC(EMC_ZCAL_WAIT_CNT) = next_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX] & 0xFFFFF800;
	EMC(EMC_DBG) = emc_dbg_o | 0x40000002;
	EMC(EMC_ZCAL_INTERVAL) = last_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] & 0xFF000000;
	EMC(EMC_DBG) = emc_dbg_o;

	/* Step 10:
	 *   LPDDR4 and DDR3 common section.
	 */
	emc_cc_dbg(STEPS, "Step 10\n");
	_ccfifo_write(EMC_SELF_REF, 0x101, 0);

	if (dst_clock_period <= 2000)
	{
		_ccfifo_write(EMC_MRW3, mr13_flip_fspwr ^ 0x40, 0);
		_ccfifo_write(EMC_MRW6, (last_timing->burst_regs[EMC_MRW6_INDEX] & 0xC0C0) |
			(next_timing->burst_regs[EMC_MRW6_INDEX] & 0xFFFF3F3F), 0);
		_ccfifo_write(EMC_MRW14, (last_timing->burst_regs[EMC_MRW14_INDEX] & 0x3838) |
			(next_timing->burst_regs[EMC_MRW14_INDEX] & 0xFFFF0707), 0);
		if (dram_dev_num == TWO_RANK)
		{
			_ccfifo_write(EMC_MRW7, (last_timing->burst_regs[EMC_MRW7_INDEX] & 0xC0C0) |
				(next_timing->burst_regs[EMC_MRW7_INDEX] & 0xFFFF3F3F), 0);
			_ccfifo_write(EMC_MRW15, (last_timing->burst_regs[EMC_MRW15_INDEX] & 0x3838) |
				(next_timing->burst_regs[EMC_MRW15_INDEX] & 0xFFFF0707), 0);
		}

		if (dram_dev_num == ONE_RANK || zcal_resistor_shared)
			emc_zq_cal = 0x80000001;
		else
			emc_zq_cal = 1;

		_ccfifo_write(EMC_ZQ_CAL, emc_zq_cal, 0);
	}

	emc_dbg_val = emc_dbg_o;
	tRP_src_timing = last_timing->dram_timings[T_RP] * 1000 / src_clock_period;
	ref_delay = 0;

	_ccfifo_write(EMC_MRW3, mr13_flip_fspop | 8, tRP_src_timing);
	ref_delay = tFC_lpddr4 / src_clock_period;

	_ccfifo_write(EMC_INTSTATUS, 0, ref_delay);
	_ccfifo_write(EMC_PIN, emc_pin_o & 0xFFFFFFF8, 30);

	/* Step 11:
	 *   Ramp down.
	 */
	emc_cc_dbg(STEPS, "Step 11\n");
	_ccfifo_write(EMC_CFG_SYNC, 0, 0);
	_ccfifo_write(EMC_DBG, emc_dbg_val | 0x40000002, 0); // WRITE_MUX_ACTIVE | WRITE_ACTIVE_ONLY
	ramp_down_wait = _dvfs_power_ramp_down(false, last_timing, next_timing, src_clock_period);

	/* Step 12:
	 *   And finally - trigger the clock change.
	 */
	emc_cc_dbg(STEPS, "Step 12\n");
	_ccfifo_write(EMC_STALL_THEN_EXE_AFTER_CLKCHANGE, 1, 0);
	_ccfifo_write(EMC_DBG, (emc_dbg_val & 0xBFFFFFFF) | 2, 0);

	/* Step 13:
	 *   Ramp up.
	 */
	emc_cc_dbg(STEPS, "Step 13\n");
	ramp_up_wait = _dvfs_power_ramp_up(false, last_timing, next_timing, 0, dst_clock_period);
	_ccfifo_write(EMC_DBG, emc_dbg_val, 0);

	/* Step 14:
	 *   Bringup CKE pins.
	 */
	emc_cc_dbg(STEPS, "Step 14\n");
	emc_pin_val_final = 0;
	if (dram_dev_num == TWO_RANK)
		emc_pin_val_final = emc_pin_o | 7;
	else
		emc_pin_val_final = (emc_pin_o & 0xFFFFFFF8) | 1;

	_ccfifo_write(EMC_PIN, emc_pin_val_final, 0);

	/* Step 15: (two step 15s ??)
	 *   Calculate zqlatch wait time; has dependency on ramping times.
	 */
	emc_cc_dbg(STEPS, "Step 15\n");
	zq_latch_dvfs_wait_time = 0;
	T_PDEX_timing_final = 0;
	T_PDEX_timing = div_o3(next_timing->dram_timings[T_PDEX] * 1000, dst_clock_period);

	if (dst_clock_period > 2000)
		zq_latch_dvfs_wait_time = (s32)tZQCAL_lpddr4_fc_adj - (s32)T_PDEX_timing;
	else
		zq_latch_dvfs_wait_time =
			(s32)tZQCAL_lpddr4_fc_adj - (ramp_up_wait + ramp_down_wait) / dst_clock_period;

	emc_cc_dbg(INFO, "tZQCAL_lpddr4_fc_adj = %u\n", tZQCAL_lpddr4_fc_adj);
	emc_cc_dbg(INFO, "destination_clock_period = %u\n",
		   dst_clock_period);
	emc_cc_dbg(INFO, "next_timing->dram_timings[T_PDEX] = %u\n",
		   next_timing->dram_timings[T_PDEX]);
	emc_cc_dbg(INFO, "zq_latch_dvfs_wait_time = %d\n",
		   max_t(s32, 0, zq_latch_dvfs_wait_time));

	if (dram_dev_num == ONE_RANK)
	{
		if (dst_clock_period > 2000)
			_ccfifo_write(EMC_ZQ_CAL, 0x80000001, T_PDEX_timing);

		_ccfifo_write(EMC_MRW3, (mr13_flip_fspop & 0xF3FFFFF7) | 0xC000000, T_PDEX_timing);

		emc_zq_cal = 0x80000002;
	}
	else if (zcal_resistor_shared)
	{
		if (dst_clock_period > 2000)
			_ccfifo_write(EMC_ZQ_CAL, 0x80000001, T_PDEX_timing);

		T_PDEX_timing_final = zq_latch_dvfs_wait_time + (s32)T_PDEX_timing;

		if (T_PDEX_timing_final < 0)
			T_PDEX_timing_final = 0;

		_ccfifo_write(EMC_ZQ_CAL, 0x80000002, T_PDEX_timing_final);
		_ccfifo_write(EMC_ZQ_CAL, 0x40000001, 0);

		_ccfifo_write(EMC_MRW3, (mr13_flip_fspop & 0xF3FFFFF7) | 0xC000000, 0);

		emc_zq_cal = 0x40000002;
		zq_latch_dvfs_wait_time = 1000000 / dst_clock_period;
	}
	else
	{
		if (dst_clock_period > 2000)
			_ccfifo_write(EMC_ZQ_CAL, 1, T_PDEX_timing);

		_ccfifo_write(EMC_MRW3, (mr13_flip_fspop & 0xF3FFFFF7) | 0xC000000, T_PDEX_timing);

		emc_zq_cal = 2;	
	}

	// Disable self-refresh.
	_ccfifo_write(EMC_SELF_REF, 0, 0);
	_ccfifo_write(EMC_REF, 0, 0);

	if (zq_latch_dvfs_wait_time < 0)
		zq_latch_dvfs_wait_time = 0;

	_ccfifo_write(EMC_ZQ_CAL, emc_zq_cal, (u32)zq_latch_dvfs_wait_time);

	/* WAR: delay for zqlatch */
	_ccfifo_write(EMC_INTSTATUS, 0, 10);


	/* Step 16:
	 *   LPDDR4 Conditional Training Kickoff. Removed.
	 */

	/* Step 17:
	 *   MANSR exit self refresh. Removed.
	 */

	/* Step 18:
	 *   Send MRWs to LPDDR3/DDR3. Removed.
	 */

	/* Step 19:
	 *   ZQCAL for LPDDR3/DDR3
	 */
	emc_cc_dbg(STEPS, "Step 19.2\n");
	if (bg_regulator_mode_change)
	{
		_ccfifo_write(EMC_DBG, emc_dbg_o | 2, 0);

		bg_regulator_switch_complete_wait_clks = 0;

		if (ramp_up_wait <= 1250000)
			bg_regulator_switch_complete_wait_clks = (1250000 - ramp_up_wait) / dst_clock_period;
		_ccfifo_write(EMC_PMACRO_BG_BIAS_CTRL_0,
			next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX], bg_regulator_switch_complete_wait_clks);

		_ccfifo_write(EMC_DBG, emc_dbg_o, 0);
	}

	/* Step 20:
	 *   Issue ref and optional QRST. Removed.
	 */

	/* Step 21:
	 *   Restore ZCAL and ZCAL interval.
	 */
	emc_cc_dbg(STEPS, "Step 21\n");
	_ccfifo_write(EMC_DBG, emc_dbg_o | 2, 0);
	_ccfifo_write(EMC_CFG, next_timing->burst_regs[EMC_CFG_INDEX] & 0xEFFFFFFF, 0);

	/* Step 22:
	 *   Restore EMC_CFG_PIPE_CLK.
	 */
	emc_cc_dbg(STEPS, "Step 22\n");

	_ccfifo_write(EMC_DBG, emc_dbg_o, 0);
	_ccfifo_write(EMC_CFG_PIPE_CLK, emc_cfg_pipe_clk_o, 0);

	if (bg_regulator_mode_change)
	{
		if (enable_bg_regulator)
			EMC(EMC_PMACRO_BG_BIAS_CTRL_0) = next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] & 0xFFFFFFFB;
		else
			EMC(EMC_PMACRO_BG_BIAS_CTRL_0) = next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] & 0xFFFFFFFE;
	}

	/* Step 23:
	 */
	emc_cc_dbg(STEPS, "Step 23: Clock change\n");
	EMC(EMC_CFG_DIG_DLL) = (EMC(EMC_CFG_DIG_DLL) & 0xFFFFFF24) | 0x88;
	emc_readl(EMC_CFG_DIG_DLL); /* Flush write. */

	emc_readl(EMC_FBIO_CFG7); /* Flush */
	mc_readl(MC_EMEM_ADR_CFG); /* Flush */
	emc_readl(EMC_INTSTATUS); /* Flush */

	/* Clock change. Woot. BUG()s out if something fails. */
	do_clock_change(clksrc);

	/* Step 24:
	 *   Save training results. Removed.
	 */

	/* Step 25:
	 *   Program MC updown registers.
	 */
	emc_cc_dbg(STEPS, "Step 25\n");
	if (next_timing->rate  > last_timing->rate )
	{
		for (i = 0; next_timing->num_up_down > i; i++)
			MC(la_scale_regs_mc_addr_table[i]) = next_timing->la_scale_regs[i];

		dual_channel = (EMC(EMC_FBIO_CFG7) >> 1) & ((EMC(EMC_FBIO_CFG7) >> 2) & 1);
		emc_timing_update(dual_channel); //"MTC Error: MC UpDown reg timeout\n"
	}

	/* Step 26:
	 *   Restore ZCAL registers.
	 */
	emc_cc_dbg(STEPS, "Step 26\n");
	EMC(EMC_DBG) = emc_dbg_o | 2;
	EMC(EMC_ZCAL_WAIT_CNT) = next_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX];
	EMC(EMC_ZCAL_INTERVAL) = next_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX];
	EMC(EMC_DBG) = emc_dbg_o;

	/* Step 27:
	 *   Restore EMC_CFG, FDPD registers.
	 */
	emc_cc_dbg(STEPS, "Step 27\n");
	EMC(EMC_DBG) = emc_dbg_o | 2;
	EMC(EMC_CFG) = next_timing->burst_regs[EMC_CFG_INDEX];
	EMC(EMC_DBG) = emc_dbg_o;
	EMC(EMC_FDPD_CTRL_CMD_NO_RAMP) = next_timing->emc_fdpd_ctrl_cmd_no_ramp;
	EMC(EMC_SEL_DPD_CTRL) = next_timing->emc_sel_dpd_ctrl;

	/* Step 28:
	 *   Training recover. Removed.
	 */
	emc_cc_dbg(STEPS, "Step 28\n");
	EMC(EMC_DBG) = emc_dbg_o | 2;
	EMC(EMC_PMACRO_AUTOCAL_CFG_COMMON) = next_timing->burst_regs[EMC_PMACRO_AUTOCAL_CFG_COMMON_INDEX];
	EMC(EMC_DBG) = emc_dbg_o;

	/* Step 29:
	 *   Power fix WAR.
	 */
	emc_cc_dbg(STEPS, "Step 29\n");
	EMC(EMC_PMACRO_CFG_PM_GLOBAL_0) = 0xFF0000;
	EMC(EMC_PMACRO_TRAINING_CTRL_0) = EMC_PMACRO_TRAINING_CTRL_0_CH0_TRAINING_E_WRPTR;
	EMC(EMC_PMACRO_TRAINING_CTRL_1) = EMC_PMACRO_TRAINING_CTRL_1_CH1_TRAINING_E_WRPTR;
	EMC(EMC_PMACRO_CFG_PM_GLOBAL_0) = 0;

	/* Step 30:
	 *   Re-enable autocal and DLL .
	 */
	emc_cc_dbg(STEPS, "Step 30: Re-enable DLL and AUTOCAL\n");
	if (next_timing->burst_regs[EMC_CFG_DIG_DLL_INDEX] & EMC_CFG_DIG_DLL_CFG_DLL_EN)
		_digital_dll_enable_rs(channel1_enabled);
	EMC(EMC_AUTO_CAL_CONFIG) = next_timing->emc_auto_cal_config;

	/* Step 31:
	 *   Restore FSP to account for switch back. Only needed in training. Removed.
	 */

	/* Step 32:
	 *   [SW] Update the alternative timing (derated vs normal) table with
	 * the periodic training values computed during the clock change
	 * pre-amble.
	 */
	emc_cc_dbg(STEPS, "Step 32: Update alt timing\n");
	tegra210_update_emc_alt_timing(next_timing);

	/* Done! Pain and Suffering.. */
}
