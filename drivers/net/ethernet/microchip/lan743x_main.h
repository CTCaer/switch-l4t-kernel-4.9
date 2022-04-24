/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (C) 2018 Microchip Technology Inc. */

#ifndef _LAN743X_H
#define _LAN743X_H

#define DRIVER_AUTHOR   "Bryan Whitehead <Bryan.Whitehead@microchip.com>"
#define DRIVER_DESC "LAN743x PCIe Gigabit Ethernet Driver"
#define DRIVER_NAME "lan743x"

/* Register Definitions */
#define ID_REV				(0x00)
#define ID_REV_IS_VALID_CHIP_ID_(id_rev)	\
	(((id_rev) & 0xFFF00000) == 0x74300000)
#define ID_REV_CHIP_REV_MASK_		(0x0000FFFF)
#define ID_REV_CHIP_REV_A0_		(0x00000000)
#define ID_REV_CHIP_REV_B0_		(0x00000010)

#define FPGA_REV			(0x04)
#define FPGA_REV_GET_MINOR_(fpga_rev)	(((fpga_rev) >> 8) & 0x000000FF)
#define FPGA_REV_GET_MAJOR_(fpga_rev)	((fpga_rev) & 0x000000FF)

#define HW_CFG					(0x010)
#define HW_CFG_LRST_				BIT(1)

#define PMT_CTL					(0x014)
#define PMT_CTL_READY_				BIT(7)
#define PMT_CTL_ETH_PHY_RST_			BIT(4)

#define DP_SEL				(0x024)
#define DP_SEL_DPRDY_			BIT(31)
#define DP_SEL_MASK_			(0x0000001F)
#define DP_SEL_RFE_RAM			(0x00000001)

#define DP_SEL_VHF_HASH_LEN		(16)
#define DP_SEL_VHF_VLAN_LEN		(128)

#define DP_CMD				(0x028)
#define DP_CMD_WRITE_			(0x00000001)

#define DP_ADDR				(0x02C)

#define DP_DATA_0			(0x030)

#define FCT_RX_CTL			(0xAC)
#define FCT_RX_CTL_EN_(channel)		BIT(28 + (channel))
#define FCT_RX_CTL_DIS_(channel)	BIT(24 + (channel))
#define FCT_RX_CTL_RESET_(channel)	BIT(20 + (channel))

#define FCT_TX_CTL			(0xC4)
#define FCT_TX_CTL_EN_(channel)		BIT(28 + (channel))
#define FCT_TX_CTL_DIS_(channel)	BIT(24 + (channel))
#define FCT_TX_CTL_RESET_(channel)	BIT(20 + (channel))

#define FCT_FLOW(rx_channel)			(0xE0 + ((rx_channel) << 2))
#define FCT_FLOW_CTL_OFF_THRESHOLD_		(0x00007F00)
#define FCT_FLOW_CTL_OFF_THRESHOLD_SET_(value)	\
	((value << 8) & FCT_FLOW_CTL_OFF_THRESHOLD_)
#define FCT_FLOW_CTL_REQ_EN_			BIT(7)
#define FCT_FLOW_CTL_ON_THRESHOLD_		(0x0000007F)
#define FCT_FLOW_CTL_ON_THRESHOLD_SET_(value)	\
	((value << 0) & FCT_FLOW_CTL_ON_THRESHOLD_)

#define MAC_CR				(0x100)
#define MAC_CR_ADD_			BIT(12)
#define MAC_CR_ASD_			BIT(11)
#define MAC_CR_CNTR_RST_		BIT(5)
#define MAC_CR_RST_			BIT(0)

#define MAC_RX				(0x104)
#define MAC_RX_MAX_SIZE_SHIFT_		(16)
#define MAC_RX_MAX_SIZE_MASK_		(0x3FFF0000)
#define MAC_RX_RXD_			BIT(1)
#define MAC_RX_RXEN_			BIT(0)

#define MAC_TX				(0x108)
#define MAC_TX_TXD_			BIT(1)
#define MAC_TX_TXEN_			BIT(0)

#define MAC_FLOW			(0x10C)
#define MAC_FLOW_CR_TX_FCEN_		BIT(30)
#define MAC_FLOW_CR_RX_FCEN_		BIT(29)
#define MAC_FLOW_CR_FCPT_MASK_		(0x0000FFFF)

#define MAC_RX_ADDRH			(0x118)

#define MAC_RX_ADDRL			(0x11C)

#define MAC_MII_ACC			(0x120)
#define MAC_MII_ACC_PHY_ADDR_SHIFT_	(11)
#define MAC_MII_ACC_PHY_ADDR_MASK_	(0x0000F800)
#define MAC_MII_ACC_MIIRINDA_SHIFT_	(6)
#define MAC_MII_ACC_MIIRINDA_MASK_	(0x000007C0)
#define MAC_MII_ACC_MII_READ_		(0x00000000)
#define MAC_MII_ACC_MII_WRITE_		(0x00000002)
#define MAC_MII_ACC_MII_BUSY_		BIT(0)

#define MAC_MII_DATA			(0x124)

/* offset 0x400 - 0x500, x may range from 0 to 32, for a total of 33 entries */
#define RFE_ADDR_FILT_HI(x)		(0x400 + (8 * (x)))
#define RFE_ADDR_FILT_HI_VALID_		BIT(31)

/* offset 0x404 - 0x504, x may range from 0 to 32, for a total of 33 entries */
#define RFE_ADDR_FILT_LO(x)		(0x404 + (8 * (x)))

#define RFE_CTL				(0x508)
#define RFE_CTL_AB_			BIT(10)
#define RFE_CTL_AM_			BIT(9)
#define RFE_CTL_AU_			BIT(8)
#define RFE_CTL_MCAST_HASH_		BIT(3)
#define RFE_CTL_DA_PERFECT_		BIT(1)

#define INT_STS				(0x780)
#define INT_BIT_DMA_RX_(channel)	BIT(24 + (channel))
#define INT_BIT_ALL_RX_			(0x0F000000)
#define INT_BIT_DMA_TX_(channel)	BIT(16 + (channel))
#define INT_BIT_ALL_TX_			(0x000F0000)
#define INT_BIT_SW_GP_			BIT(9)
#define INT_BIT_ALL_OTHER_		(0x00000280)
#define INT_BIT_MAS_			BIT(0)

#define INT_SET				(0x784)

#define INT_EN_SET			(0x788)

#define INT_EN_CLR			(0x78C)

#define INT_STS_R2C			(0x790)

#define INT_VEC_EN_SET			(0x794)
#define INT_VEC_EN_CLR			(0x798)
#define INT_VEC_EN_AUTO_CLR		(0x79C)
#define INT_VEC_EN_(vector_index)	BIT(0 + vector_index)

#define INT_VEC_MAP0			(0x7A0)
#define INT_VEC_MAP0_RX_VEC_(channel, vector)	\
	(((u32)(vector)) << ((channel) << 2))

#define INT_VEC_MAP1			(0x7A4)
#define INT_VEC_MAP1_TX_VEC_(channel, vector)	\
	(((u32)(vector)) << ((channel) << 2))

#define INT_VEC_MAP2			(0x7A8)

#define INT_MOD_MAP0			(0x7B0)

#define INT_MOD_MAP1			(0x7B4)

#define INT_MOD_MAP2			(0x7B8)

#define INT_MOD_CFG0			(0x7C0)
#define INT_MOD_CFG1			(0x7C4)
#define INT_MOD_CFG2			(0x7C8)
#define INT_MOD_CFG3			(0x7CC)
#define INT_MOD_CFG4			(0x7D0)
#define INT_MOD_CFG5			(0x7D4)
#define INT_MOD_CFG6			(0x7D8)
#define INT_MOD_CFG7			(0x7DC)

#define DMAC_CFG				(0xC00)
#define DMAC_CFG_COAL_EN_			BIT(16)
#define DMAC_CFG_CH_ARB_SEL_RX_HIGH_		(0x00000000)
#define DMAC_CFG_MAX_READ_REQ_MASK_		(0x00000070)
#define DMAC_CFG_MAX_READ_REQ_SET_(val)	\
	((((u32)(val)) << 4) & DMAC_CFG_MAX_READ_REQ_MASK_)
#define DMAC_CFG_MAX_DSPACE_16_			(0x00000000)
#define DMAC_CFG_MAX_DSPACE_32_			(0x00000001)
#define DMAC_CFG_MAX_DSPACE_64_			BIT(1)
#define DMAC_CFG_MAX_DSPACE_128_		(0x00000003)

#define DMAC_COAL_CFG				(0xC04)
#define DMAC_COAL_CFG_TIMER_LIMIT_MASK_		(0xFFF00000)
#define DMAC_COAL_CFG_TIMER_LIMIT_SET_(val)	\
	((((u32)(val)) << 20) & DMAC_COAL_CFG_TIMER_LIMIT_MASK_)
#define DMAC_COAL_CFG_TIMER_TX_START_		BIT(19)
#define DMAC_COAL_CFG_FLUSH_INTS_		BIT(18)
#define DMAC_COAL_CFG_INT_EXIT_COAL_		BIT(17)
#define DMAC_COAL_CFG_CSR_EXIT_COAL_		BIT(16)
#define DMAC_COAL_CFG_TX_THRES_MASK_		(0x0000FF00)
#define DMAC_COAL_CFG_TX_THRES_SET_(val)	\
	((((u32)(val)) << 8) & DMAC_COAL_CFG_TX_THRES_MASK_)
#define DMAC_COAL_CFG_RX_THRES_MASK_		(0x000000FF)
#define DMAC_COAL_CFG_RX_THRES_SET_(val)	\
	(((u32)(val)) & DMAC_COAL_CFG_RX_THRES_MASK_)

#define DMAC_OBFF_CFG				(0xC08)
#define DMAC_OBFF_TX_THRES_MASK_		(0x0000FF00)
#define DMAC_OBFF_TX_THRES_SET_(val)	\
	((((u32)(val)) << 8) & DMAC_OBFF_TX_THRES_MASK_)
#define DMAC_OBFF_RX_THRES_MASK_		(0x000000FF)
#define DMAC_OBFF_RX_THRES_SET_(val)	\
	(((u32)(val)) & DMAC_OBFF_RX_THRES_MASK_)

#define DMAC_CMD				(0xC0C)
#define DMAC_CMD_SWR_				BIT(31)
#define DMAC_CMD_TX_SWR_(channel)		BIT(24 + (channel))
#define DMAC_CMD_START_T_(channel)		BIT(20 + (channel))
#define DMAC_CMD_STOP_T_(channel)		BIT(16 + (channel))
#define DMAC_CMD_RX_SWR_(channel)		BIT(8 + (channel))
#define DMAC_CMD_START_R_(channel)		BIT(4 + (channel))
#define DMAC_CMD_STOP_R_(channel)		BIT(0 + (channel))

#define DMAC_INT_STS				(0xC10)
#define DMAC_INT_EN_SET				(0xC14)
#define DMAC_INT_EN_CLR				(0xC18)
#define DMAC_INT_BIT_RXFRM_(channel)		BIT(16 + (channel))
#define DMAC_INT_BIT_TX_IOC_(channel)		BIT(0 + (channel))

#define RX_CFG_A(channel)			(0xC40 + ((channel) << 6))
#define RX_CFG_A_RX_WB_ON_INT_TMR_		BIT(30)
#define RX_CFG_A_RX_WB_THRES_MASK_		(0x1F000000)
#define RX_CFG_A_RX_WB_THRES_SET_(val)	\
	((((u32)(val)) << 24) & RX_CFG_A_RX_WB_THRES_MASK_)
#define RX_CFG_A_RX_PF_THRES_MASK_		(0x001F0000)
#define RX_CFG_A_RX_PF_THRES_SET_(val)	\
	((((u32)(val)) << 16) & RX_CFG_A_RX_PF_THRES_MASK_)
#define RX_CFG_A_RX_PF_PRI_THRES_MASK_		(0x00001F00)
#define RX_CFG_A_RX_PF_PRI_THRES_SET_(val)	\
	((((u32)(val)) << 8) & RX_CFG_A_RX_PF_PRI_THRES_MASK_)
#define RX_CFG_A_RX_HP_WB_EN_			BIT(5)

#define RX_CFG_B(channel)			(0xC44 + ((channel) << 6))
#define RX_CFG_B_TS_ALL_RX_			BIT(29)
#define RX_CFG_B_RX_PAD_MASK_			(0x03000000)
#define RX_CFG_B_RX_PAD_0_			(0x00000000)
#define RX_CFG_B_RX_PAD_2_			(0x02000000)
#define RX_CFG_B_RDMABL_512_			(0x00040000)
#define RX_CFG_B_RX_RING_LEN_MASK_		(0x0000FFFF)

#define RX_BASE_ADDRH(channel)			(0xC48 + ((channel) << 6))

#define RX_BASE_ADDRL(channel)			(0xC4C + ((channel) << 6))

#define RX_HEAD_WRITEBACK_ADDRH(channel)	(0xC50 + ((channel) << 6))

#define RX_HEAD_WRITEBACK_ADDRL(channel)	(0xC54 + ((channel) << 6))

#define RX_HEAD(channel)			(0xC58 + ((channel) << 6))

#define RX_TAIL(channel)			(0xC5C + ((channel) << 6))
#define RX_TAIL_SET_TOP_INT_EN_			BIT(30)
#define RX_TAIL_SET_TOP_INT_VEC_EN_		BIT(29)

#define RX_CFG_C(channel)			(0xC64 + ((channel) << 6))
#define RX_CFG_C_RX_TOP_INT_EN_AUTO_CLR_	BIT(6)
#define RX_CFG_C_RX_INT_EN_R2C_			BIT(4)
#define RX_CFG_C_RX_DMA_INT_STS_AUTO_CLR_	BIT(3)
#define RX_CFG_C_RX_INT_STS_R2C_MODE_MASK_	(0x00000007)

#define TX_CFG_A(channel)			(0xD40 + ((channel) << 6))
#define TX_CFG_A_TX_HP_WB_ON_INT_TMR_		BIT(30)
#define TX_CFG_A_TX_TMR_HPWB_SEL_IOC_		(0x10000000)
#define TX_CFG_A_TX_PF_THRES_MASK_		(0x001F0000)
#define TX_CFG_A_TX_PF_THRES_SET_(value)	\
	((((u32)(value)) << 16) & TX_CFG_A_TX_PF_THRES_MASK_)
#define TX_CFG_A_TX_PF_PRI_THRES_MASK_		(0x00001F00)
#define TX_CFG_A_TX_PF_PRI_THRES_SET_(value)	\
	((((u32)(value)) << 8) & TX_CFG_A_TX_PF_PRI_THRES_MASK_)
#define TX_CFG_A_TX_HP_WB_EN_			BIT(5)
#define TX_CFG_A_TX_HP_WB_THRES_MASK_		(0x0000000F)
#define TX_CFG_A_TX_HP_WB_THRES_SET_(value)	\
	(((u32)(value)) & TX_CFG_A_TX_HP_WB_THRES_MASK_)

#define TX_CFG_B(channel)			(0xD44 + ((channel) << 6))
#define TX_CFG_B_TDMABL_512_			(0x00040000)
#define TX_CFG_B_TX_RING_LEN_MASK_		(0x0000FFFF)

#define TX_BASE_ADDRH(channel)			(0xD48 + ((channel) << 6))

#define TX_BASE_ADDRL(channel)			(0xD4C + ((channel) << 6))

#define TX_HEAD_WRITEBACK_ADDRH(channel)	(0xD50 + ((channel) << 6))

#define TX_HEAD_WRITEBACK_ADDRL(channel)	(0xD54 + ((channel) << 6))

#define TX_HEAD(channel)			(0xD58 + ((channel) << 6))

#define TX_TAIL(channel)			(0xD5C + ((channel) << 6))
#define TX_TAIL_SET_DMAC_INT_EN_		BIT(31)
#define TX_TAIL_SET_TOP_INT_EN_			BIT(30)
#define TX_TAIL_SET_TOP_INT_VEC_EN_		BIT(29)

#define TX_CFG_C(channel)			(0xD64 + ((channel) << 6))
#define TX_CFG_C_TX_TOP_INT_EN_AUTO_CLR_	BIT(6)
#define TX_CFG_C_TX_DMA_INT_EN_AUTO_CLR_	BIT(5)
#define TX_CFG_C_TX_INT_EN_R2C_			BIT(4)
#define TX_CFG_C_TX_DMA_INT_STS_AUTO_CLR_	BIT(3)
#define TX_CFG_C_TX_INT_STS_R2C_MODE_MASK_	(0x00000007)

/* MAC statistics registers */
#define STAT_RX_FCS_ERRORS			(0x1200)
#define STAT_RX_ALIGNMENT_ERRORS		(0x1204)
#define STAT_RX_JABBER_ERRORS			(0x120C)
#define STAT_RX_UNDERSIZE_FRAME_ERRORS		(0x1210)
#define STAT_RX_OVERSIZE_FRAME_ERRORS		(0x1214)
#define STAT_RX_DROPPED_FRAMES			(0x1218)
#define STAT_RX_UNICAST_BYTE_COUNT		(0x121C)
#define STAT_RX_BROADCAST_BYTE_COUNT		(0x1220)
#define STAT_RX_MULTICAST_BYTE_COUNT		(0x1224)
#define STAT_RX_MULTICAST_FRAMES		(0x1230)
#define STAT_RX_TOTAL_FRAMES			(0x1254)

#define STAT_TX_FCS_ERRORS			(0x1280)
#define STAT_TX_EXCESS_DEFERRAL_ERRORS		(0x1284)
#define STAT_TX_CARRIER_ERRORS			(0x1288)
#define STAT_TX_SINGLE_COLLISIONS		(0x1290)
#define STAT_TX_MULTIPLE_COLLISIONS		(0x1294)
#define STAT_TX_EXCESSIVE_COLLISION		(0x1298)
#define STAT_TX_LATE_COLLISIONS			(0x129C)
#define STAT_TX_UNICAST_BYTE_COUNT		(0x12A0)
#define STAT_TX_BROADCAST_BYTE_COUNT		(0x12A4)
#define STAT_TX_MULTICAST_BYTE_COUNT		(0x12A8)
#define STAT_TX_MULTICAST_FRAMES		(0x12B4)
#define STAT_TX_TOTAL_FRAMES			(0x12D8)

/* End of Register definitions */

#define LAN743X_MAX_RX_CHANNELS		(4)
#define LAN743X_MAX_TX_CHANNELS		(1)
struct lan743x_adapter;

#define LAN743X_USED_RX_CHANNELS	(4)
#define LAN743X_USED_TX_CHANNELS	(1)
#define LAN743X_INT_MOD	(400)

#if (LAN743X_USED_RX_CHANNELS > LAN743X_MAX_RX_CHANNELS)
#error Invalid LAN743X_USED_RX_CHANNELS
#endif
#if (LAN743X_USED_TX_CHANNELS > LAN743X_MAX_TX_CHANNELS)
#error Invalid LAN743X_USED_TX_CHANNELS
#endif

/* PCI */
/* SMSC acquired EFAR late 1990's, MCHP acquired SMSC 2012 */
#define PCI_VENDOR_ID_SMSC		PCI_VENDOR_ID_EFAR
#define PCI_DEVICE_ID_SMSC_LAN7430	(0x7430)

#define PCI_CONFIG_LENGTH		(0x1000)

/* CSR */
#define CSR_LENGTH					(0x2000)

#define LAN743X_CSR_FLAG_IS_A0				BIT(0)
#define LAN743X_CSR_FLAG_IS_B0				BIT(1)
#define LAN743X_CSR_FLAG_SUPPORTS_INTR_AUTO_SET_CLR	BIT(8)

struct lan743x_csr {
	u32 flags;
	u8 __iomem *csr_address;
	u32 id_rev;
	u32 fpga_rev;
};

/* INTERRUPTS */
typedef void(*lan743x_vector_handler)(void *context, u32 int_sts, u32 flags);

#define LAN743X_VECTOR_FLAG_IRQ_SHARED			BIT(0)
#define LAN743X_VECTOR_FLAG_SOURCE_STATUS_READ		BIT(1)
#define LAN743X_VECTOR_FLAG_SOURCE_STATUS_R2C		BIT(2)
#define LAN743X_VECTOR_FLAG_SOURCE_STATUS_W2C		BIT(3)
#define LAN743X_VECTOR_FLAG_SOURCE_ENABLE_CHECK		BIT(4)
#define LAN743X_VECTOR_FLAG_SOURCE_ENABLE_CLEAR		BIT(5)
#define LAN743X_VECTOR_FLAG_SOURCE_ENABLE_R2C		BIT(6)
#define LAN743X_VECTOR_FLAG_MASTER_ENABLE_CLEAR		BIT(7)
#define LAN743X_VECTOR_FLAG_MASTER_ENABLE_SET		BIT(8)
#define LAN743X_VECTOR_FLAG_VECTOR_ENABLE_ISR_CLEAR	BIT(9)
#define LAN743X_VECTOR_FLAG_VECTOR_ENABLE_ISR_SET	BIT(10)
#define LAN743X_VECTOR_FLAG_VECTOR_ENABLE_AUTO_CLEAR	BIT(11)
#define LAN743X_VECTOR_FLAG_VECTOR_ENABLE_AUTO_SET	BIT(12)
#define LAN743X_VECTOR_FLAG_SOURCE_ENABLE_AUTO_CLEAR	BIT(13)
#define LAN743X_VECTOR_FLAG_SOURCE_ENABLE_AUTO_SET	BIT(14)
#define LAN743X_VECTOR_FLAG_SOURCE_STATUS_AUTO_CLEAR	BIT(15)

struct lan743x_vector {
	int			irq;
	u32			flags;
	struct lan743x_adapter	*adapter;
	int			vector_index;
	u32			int_mask;
	lan743x_vector_handler	handler;
	void			*context;
};

#define LAN743X_MAX_VECTOR_COUNT	(8)

struct lan743x_intr {
	int			flags;

	unsigned int		irq;

	struct lan743x_vector	vector_list[LAN743X_MAX_VECTOR_COUNT];
	int			number_of_vectors;
	bool			using_vectors;

	int			software_isr_flag;
};

#define LAN743X_MAX_FRAME_SIZE			(9 * 1024)

/* PHY */
struct lan743x_phy {
	bool	fc_autoneg;
	u8	fc_request_control;
};

/* TX */
struct lan743x_tx_descriptor;
struct lan743x_tx_buffer_info;

#define GPIO_QUEUE_STARTED		(0)
#define GPIO_TX_FUNCTION		(1)
#define GPIO_TX_COMPLETION		(2)
#define GPIO_TX_FRAGMENT		(3)

#define TX_FRAME_FLAG_IN_PROGRESS	BIT(0)

struct lan743x_tx {
	struct lan743x_adapter *adapter;
	u32	vector_flags;
	int	channel_number;

	int	ring_size;
	size_t	ring_allocation_size;
	struct lan743x_tx_descriptor *ring_cpu_ptr;
	dma_addr_t ring_dma_ptr;
	/* ring_lock: used to prevent concurrent access to tx ring */
	spinlock_t ring_lock;
	u32		frame_flags;
	u32		frame_first;
	u32		frame_data0;
	u32		frame_tail;

	struct lan743x_tx_buffer_info *buffer_info;

	u32		*head_cpu_ptr;
	dma_addr_t	head_dma_ptr;
	int		last_head;
	int		last_tail;

	struct napi_struct napi;

	struct sk_buff *overflow_skb;
};

/* RX */
struct lan743x_rx_descriptor;
struct lan743x_rx_buffer_info;

struct lan743x_rx {
	struct lan743x_adapter *adapter;
	u32	vector_flags;
	int	channel_number;

	int	ring_size;
	size_t	ring_allocation_size;
	struct lan743x_rx_descriptor *ring_cpu_ptr;
	dma_addr_t ring_dma_ptr;

	struct lan743x_rx_buffer_info *buffer_info;

	u32		*head_cpu_ptr;
	dma_addr_t	head_dma_ptr;
	u32		last_head;
	u32		last_tail;

	struct napi_struct napi;

	u32		frame_count;
};

struct lan743x_adapter {
	struct net_device       *netdev;
	struct mii_bus		*mdiobus;
	int                     msg_enable;
	struct pci_dev		*pdev;
	struct lan743x_csr      csr;
	struct lan743x_intr     intr;

	/* lock, used to prevent concurrent access to data port */
	struct mutex		dp_lock;

	u8			mac_address[ETH_ALEN];

	struct lan743x_phy      phy;
	struct lan743x_tx       tx[LAN743X_MAX_TX_CHANNELS];
	struct lan743x_rx       rx[LAN743X_MAX_RX_CHANNELS];
};

#define LAN743X_COMPONENT_FLAG_RX(channel)  BIT(20 + (channel))

#define INTR_FLAG_IRQ_REQUESTED(vector_index)	BIT(0 + vector_index)
#define INTR_FLAG_MSI_ENABLED			BIT(8)
#define INTR_FLAG_MSIX_ENABLED			BIT(9)

#define MAC_MII_READ            1
#define MAC_MII_WRITE           0

#define PHY_FLAG_OPENED     BIT(0)
#define PHY_FLAG_ATTACHED   BIT(1)

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
#define DMA_ADDR_HIGH32(dma_addr)   ((u32)(((dma_addr) >> 32) & 0xFFFFFFFF))
#else
#define DMA_ADDR_HIGH32(dma_addr)   ((u32)(0))
#endif
#define DMA_ADDR_LOW32(dma_addr) ((u32)((dma_addr) & 0xFFFFFFFF))
#define DMA_DESCRIPTOR_SPACING_16       (16)
#define DMA_DESCRIPTOR_SPACING_32       (32)
#define DMA_DESCRIPTOR_SPACING_64       (64)
#define DMA_DESCRIPTOR_SPACING_128      (128)
#define DEFAULT_DMA_DESCRIPTOR_SPACING  (L1_CACHE_BYTES)

#define DMAC_CHANNEL_STATE_SET(start_bit, stop_bit) \
	(((start_bit) ? 2 : 0) | ((stop_bit) ? 1 : 0))
#define DMAC_CHANNEL_STATE_INITIAL      DMAC_CHANNEL_STATE_SET(0, 0)
#define DMAC_CHANNEL_STATE_STARTED      DMAC_CHANNEL_STATE_SET(1, 0)
#define DMAC_CHANNEL_STATE_STOP_PENDING DMAC_CHANNEL_STATE_SET(1, 1)
#define DMAC_CHANNEL_STATE_STOPPED      DMAC_CHANNEL_STATE_SET(0, 1)

/* TX Descriptor bits */
#define TX_DESC_DATA0_DTYPE_MASK_		(0xC0000000)
#define TX_DESC_DATA0_DTYPE_DATA_		(0x00000000)
#define TX_DESC_DATA0_DTYPE_EXT_		(0x40000000)
#define TX_DESC_DATA0_FS_			(0x20000000)
#define TX_DESC_DATA0_LS_			(0x10000000)
#define TX_DESC_DATA0_EXT_			(0x08000000)
#define TX_DESC_DATA0_IOC_			(0x04000000)
#define TX_DESC_DATA0_ICE_			(0x00400000)
#define TX_DESC_DATA0_IPE_			(0x00200000)
#define TX_DESC_DATA0_TPE_			(0x00100000)
#define TX_DESC_DATA0_FCS_			(0x00020000)
#define TX_DESC_DATA0_BUF_LENGTH_MASK_		(0x0000FFFF)
#define TX_DESC_DATA0_EXT_LSO_			(0x00200000)
#define TX_DESC_DATA0_EXT_PAY_LENGTH_MASK_	(0x000FFFFF)
#define TX_DESC_DATA3_FRAME_LENGTH_MSS_MASK_	(0x3FFF0000)

struct lan743x_tx_descriptor {
	u32     data0;
	u32     data1;
	u32     data2;
	u32     data3;
} __aligned(DEFAULT_DMA_DESCRIPTOR_SPACING);

#define TX_BUFFER_INFO_FLAG_ACTIVE		BIT(0)
#define TX_BUFFER_INFO_FLAG_IGNORE_SYNC		BIT(2)
#define TX_BUFFER_INFO_FLAG_SKB_FRAGMENT	BIT(3)
struct lan743x_tx_buffer_info {
	int flags;
	struct sk_buff *skb;
	dma_addr_t      dma_ptr;
	unsigned int    buffer_length;
};

#define LAN743X_TX_RING_SIZE    (50)

/* OWN bit is set. ie, Descs are owned by RX DMAC */
#define RX_DESC_DATA0_OWN_                (0x00008000)
/* OWN bit is clear. ie, Descs are owned by host */
#define RX_DESC_DATA0_FS_                 (0x80000000)
#define RX_DESC_DATA0_LS_                 (0x40000000)
#define RX_DESC_DATA0_FRAME_LENGTH_MASK_  (0x3FFF0000)
#define RX_DESC_DATA0_FRAME_LENGTH_GET_(data0)	\
	(((data0) & RX_DESC_DATA0_FRAME_LENGTH_MASK_) >> 16)
#define RX_DESC_DATA0_EXT_                (0x00004000)
#define RX_DESC_DATA0_BUF_LENGTH_MASK_    (0x00003FFF)
#define RX_DESC_DATA2_TS_NS_MASK_         (0x3FFFFFFF)

#if ((NET_IP_ALIGN != 0) && (NET_IP_ALIGN != 2))
#error NET_IP_ALIGN must be 0 or 2
#endif

#define RX_HEAD_PADDING		NET_IP_ALIGN

struct lan743x_rx_descriptor {
	u32     data0;
	u32     data1;
	u32     data2;
	u32     data3;
} __aligned(DEFAULT_DMA_DESCRIPTOR_SPACING);

#define RX_BUFFER_INFO_FLAG_ACTIVE      BIT(0)
struct lan743x_rx_buffer_info {
	int flags;
	struct sk_buff *skb;

	dma_addr_t      dma_ptr;
	unsigned int    buffer_length;
};

#define LAN743X_RX_RING_SIZE        (65)

#define RX_PROCESS_RESULT_NOTHING_TO_DO     (0)
#define RX_PROCESS_RESULT_PACKET_RECEIVED   (1)
#define RX_PROCESS_RESULT_PACKET_DROPPED    (2)

#endif /* _LAN743X_H */
