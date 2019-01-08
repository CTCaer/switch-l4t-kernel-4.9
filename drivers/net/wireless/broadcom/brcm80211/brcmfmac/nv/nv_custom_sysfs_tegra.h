/*
 * nv_custom_sysfs_tegra.h
 *
 * NVIDIA Tegra Sysfs for brcmfmac driver
 *
 * Copyright (C) 2014-2019 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _nv_custom_sysfs_tegra_h_
#define _nv_custom_sysfs_tegra_h_

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/stat.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/spinlock_types.h>
#include <linux/errno.h>
#include <linux/system-wakeup.h>

#include "core.h"
#include "fwil.h"
#include "fwil_types.h"

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(x)	(void)(x)
#endif

#define ETHER_TYPE_BRCM_REV 0x6c88

/* wl defines */
#define	NFIFO			6	/* # tx/rx fifopairs */
#define NREINITREASONCOUNT	8
#define REINITREASONIDX(_x)	(((_x) < NREINITREASONCOUNT) ? (_x) : 0)

#define	WL_CNT_T_VERSION	10	/* current version of wl_cnt_t struct */
typedef struct {
	u16	version;	/* see definition of WL_CNT_T_VERSION */
	u16	length;		/* length of entire structure */

	/* transmit stat counters */
	u32	txframe;	/* tx data frames */
	u32	txbyte;		/* tx data bytes */
	u32	txretrans;	/* tx mac retransmits */
	u32	txerror;	/* tx data errors (derived: sum of others) */
	u32	txctl;		/* tx management frames */
	u32	txprshort;	/* tx short preamble frames */
	u32	txserr;		/* tx status errors */
	u32	txnobuf;	/* tx out of buffers errors */
	u32	txnoassoc;	/* tx discard because we're not associated */
	u32	txrunt;		/* tx runt frames */
	u32	txchit;		/* tx header cache hit (fastpath) */
	u32	txcmiss;	/* tx header cache miss (slowpath) */

	/* transmit chip error counters */
	u32	txuflo;		/* tx fifo underflows */
	u32	txphyerr;	/* tx phy errors (indicated in tx status) */
	u32	txphycrs;

	/* receive stat counters */
	u32	rxframe;	/* rx data frames */
	u32	rxbyte;		/* rx data bytes */
	u32	rxerror;	/* rx data errors (derived: sum of others) */
	u32	rxctl;		/* rx management frames */
	u32	rxnobuf;	/* rx out of buffers errors */
	u32	rxnondata;	/* rx non data frames in the data channel errors */
	u32	rxbadds;	/* rx bad DS errors */
	u32	rxbadcm;	/* rx bad control or management frames */
	u32	rxfragerr;	/* rx fragmentation errors */
	u32	rxrunt;		/* rx runt frames */
	u32	rxgiant;	/* rx giant frames */
	u32	rxnoscb;	/* rx no scb error */
	u32	rxbadproto;	/* rx invalid frames */
	u32	rxbadsrcmac;	/* rx frames with Invalid Src Mac */
	u32	rxbadda;	/* rx frames tossed for invalid da */
	u32	rxfilter;	/* rx frames filtered out */

	/* receive chip error counters */
	u32	rxoflo;		/* rx fifo overflow errors */
	u32	rxuflo[NFIFO];	/* rx dma descriptor underflow errors */

	u32	d11cnt_txrts_off;	/* d11cnt txrts value when reset d11cnt */
	u32	d11cnt_rxcrc_off;	/* d11cnt rxcrc value when reset d11cnt */
	u32	d11cnt_txnocts_off;	/* d11cnt txnocts value when reset d11cnt */

	/* misc counters */
	u32	dmade;		/* tx/rx dma descriptor errors */
	u32	dmada;		/* tx/rx dma data errors */
	u32	dmape;		/* tx/rx dma descriptor protocol errors */
	u32	reset;		/* reset count */
	u32	tbtt;		/* cnts the TBTT int's */
	u32	txdmawar;
	u32	pkt_callback_reg_fail;	/* callbacks register failure */

	/* MAC counters: 32-bit version of d11.h's macstat_t */
	u32	txallfrm;	/* total number of frames sent, incl. Data, ACK, RTS, CTS,
				 * Control Management (includes retransmissions)
				 */
	u32	txrtsfrm;	/* number of RTS sent out by the MAC */
	u32	txctsfrm;	/* number of CTS sent out by the MAC */
	u32	txackfrm;	/* number of ACK frames sent out */
	u32	txdnlfrm;	/* Not used */
	u32	txbcnfrm;	/* beacons transmitted */
	u32	txfunfl[6];	/* per-fifo tx underflows */
	u32	rxtoolate;	/* receive too late */
	u32	txfbw;		/* transmit at fallback bw (dynamic bw) */
	u32	txtplunfl;	/* Template underflows (mac was too slow to transmit ACK/CTS
				 * or BCN)
				 */
	u32	txphyerror;	/* Transmit phy error, type of error is reported in tx-status for
				 * driver enqueued frames
				 */
	u32	rxfrmtoolong;	/* Received frame longer than legal limit (2346 bytes) */
	u32	rxfrmtooshrt;	/* Received frame did not contain enough bytes for its frame type */
	u32	rxinvmachdr;	/* Either the protocol version != 0 or frame type not
				 * data/control/management
				 */
	u32	rxbadfcs;	/* number of frames for which the CRC check failed in the MAC */
	u32	rxbadplcp;	/* parity check of the PLCP header failed */
	u32	rxcrsglitch;	/* PHY was able to correlate the preamble but not the header */
	u32	rxstrt;		/* Number of received frames with a good PLCP
				 * (i.e. passing parity check)
				 */
	u32	rxdfrmucastmbss; /* Number of received DATA frames with good FCS and matching RA */
	u32	rxmfrmucastmbss; /* number of received mgmt frames with good FCS and matching RA */
	u32	rxcfrmucast;	/* number of received CNTRL frames with good FCS and matching RA */
	u32	rxrtsucast;	/* number of unicast RTS addressed to the MAC (good FCS) */
	u32	rxctsucast;	/* number of unicast CTS addressed to the MAC (good FCS) */
	u32	rxackucast;	/* number of ucast ACKS received (good FCS) */
	u32	rxdfrmocast;	/* number of received DATA frames (good FCS and not matching RA) */
	u32	rxmfrmocast;	/* number of received MGMT frames (good FCS and not matching RA) */
	u32	rxcfrmocast;	/* number of received CNTRL frame (good FCS and not matching RA) */
	u32	rxrtsocast;	/* number of received RTS not addressed to the MAC */
	u32	rxctsocast;	/* number of received CTS not addressed to the MAC */
	u32	rxdfrmmcast;	/* number of RX Data multicast frames received by the MAC */
	u32	rxmfrmmcast;	/* number of RX Management multicast frames received by the MAC */
	u32	rxcfrmmcast;	/* number of RX Control multicast frames received by the MAC
				 * (unlikely to see these)
				 */
	u32	rxbeaconmbss;	/* beacons received from member of BSS */
	u32	rxdfrmucastobss; /* number of unicast frames addressed to the MAC from
				  * other BSS (WDS FRAME)
				  */
	u32	rxbeaconobss;	/* beacons received from other BSS */
	u32	rxrsptmout;	/* Number of response timeouts for transmitted frames
				 * expecting a response
				 */
	u32	bcntxcancl;	/* transmit beacons canceled due to receipt of beacon (IBSS) */
	u32	rxf0ovfl;	/* Number of receive fifo 0 overflows */
	u32	rxf1ovfl;	/* Number of receive fifo 1 overflows (obsolete) */
	u32	rxf2ovfl;	/* Number of receive fifo 2 overflows (obsolete) */
	u32	txsfovfl;	/* Number of transmit status fifo overflows (obsolete) */
	u32	pmqovfl;	/* Number of PMQ overflows */
	u32	rxcgprqfrm;	/* Number of received Probe requests that made it into
				 * the PRQ fifo
				 */
	u32	rxcgprsqovfl;	/* Rx Probe Request Que overflow in the AP */
	u32	txcgprsfail;	/* Tx Probe Response Fail. AP sent probe response but did
				 * not get ACK
				 */
	u32	txcgprssuc;	/* Tx Probe Response Success (ACK was received) */
	u32	prs_timeout;	/* Number of probe requests that were dropped from the PRQ
				 * fifo because a probe response could not be sent out within
				 * the time limit defined in M_PRS_MAXTIME
				 */
	u32	rxnack;		/* obsolete */
	u32	frmscons;	/* obsolete */
	u32  txnack;		/* obsolete */
	u32	rxback;		/* blockack rxcnt */
	u32	txback;		/* blockack txcnt */

	/* 802.11 MIB counters, pp. 614 of 802.11 reaff doc. */
	u32	txfrag;		/* dot11TransmittedFragmentCount */
	u32	txmulti;	/* dot11MulticastTransmittedFrameCount */
	u32	txfail;		/* dot11FailedCount */
	u32	txretry;	/* dot11RetryCount */
	u32	txretrie;	/* dot11MultipleRetryCount */
	u32	rxdup;		/* dot11FrameduplicateCount */
	u32	txrts;		/* dot11RTSSuccessCount */
	u32	txnocts;	/* dot11RTSFailureCount */
	u32	txnoack;	/* dot11ACKFailureCount */
	u32	rxfrag;		/* dot11ReceivedFragmentCount */
	u32	rxmulti;	/* dot11MulticastReceivedFrameCount */
	u32	rxcrc;		/* dot11FCSErrorCount */
	u32	txfrmsnt;	/* dot11TransmittedFrameCount (bogus MIB?) */
	u32	rxundec;	/* dot11WEPUndecryptableCount */

	/* WPA2 counters (see rxundec for DecryptFailureCount) */
	u32	tkipmicfaill;	/* TKIPLocalMICFailures */
	u32	tkipcntrmsr;	/* TKIPCounterMeasuresInvoked */
	u32	tkipreplay;	/* TKIPReplays */
	u32	ccmpfmterr;	/* CCMPFormatErrors */
	u32	ccmpreplay;	/* CCMPReplays */
	u32	ccmpundec;	/* CCMPDecryptErrors */
	u32	fourwayfail;	/* FourWayHandshakeFailures */
	u32	wepundec;	/* dot11WEPUndecryptableCount */
	u32	wepicverr;	/* dot11WEPICVErrorCount */
	u32	decsuccess;	/* DecryptSuccessCount */
	u32	tkipicverr;	/* TKIPICVErrorCount */
	u32	wepexcluded;	/* dot11WEPExcludedCount */

	u32	txchanrej;	/* Tx frames suppressed due to channel rejection */
	u32	psmwds;		/* Count PSM watchdogs */
	u32	phywatchdog;	/* Count Phy watchdogs (triggered by ucode) */

	/* MBSS counters, AP only */
	u32	prq_entries_handled;	/* PRQ entries read in */
	u32	prq_undirected_entries;	/*    which were bcast bss & ssid */
	u32	prq_bad_entries;	/*    which could not be translated to info */
	u32	atim_suppress_count;	/* TX suppressions on ATIM fifo */
	u32	bcn_template_not_ready;	/* Template marked in use on send bcn ... */
	u32	bcn_template_not_ready_done; /* ...but "DMA done" interrupt rcvd */
	u32	late_tbtt_dpc;	/* TBTT DPC did not happen in time */

	/* per-rate receive stat counters */
	u32  rx1mbps;	/* packets rx at 1Mbps */
	u32  rx2mbps;	/* packets rx at 2Mbps */
	u32  rx5mbps5;	/* packets rx at 5.5Mbps */
	u32  rx6mbps;	/* packets rx at 6Mbps */
	u32  rx9mbps;	/* packets rx at 9Mbps */
	u32  rx11mbps;	/* packets rx at 11Mbps */
	u32  rx12mbps;	/* packets rx at 12Mbps */
	u32  rx18mbps;	/* packets rx at 18Mbps */
	u32  rx24mbps;	/* packets rx at 24Mbps */
	u32  rx36mbps;	/* packets rx at 36Mbps */
	u32  rx48mbps;	/* packets rx at 48Mbps */
	u32  rx54mbps;	/* packets rx at 54Mbps */
	u32  rx108mbps;	/* packets rx at 108mbps */
	u32  rx162mbps;	/* packets rx at 162mbps */
	u32  rx216mbps;	/* packets rx at 216 mbps */
	u32  rx270mbps;	/* packets rx at 270 mbps */
	u32  rx324mbps;	/* packets rx at 324 mbps */
	u32  rx378mbps;	/* packets rx at 378 mbps */
	u32  rx432mbps;	/* packets rx at 432 mbps */
	u32  rx486mbps;	/* packets rx at 486 mbps */
	u32  rx540mbps;	/* packets rx at 540 mbps */

	/* pkteng rx frame stats */
	u32	pktengrxducast; /* unicast frames rxed by the pkteng code */
	u32	pktengrxdmcast; /* multicast frames rxed by the pkteng code */

	u32	rfdisable;	/* count of radio disables */
	u32	bphy_rxcrsglitch;	/* PHY count of bphy glitches */
	u32  bphy_badplcp;

	u32	txexptime;	/* Tx frames suppressed due to timer expiration */

	u32	txmpdu_sgi;	/* count for sgi transmit */
	u32	rxmpdu_sgi;	/* count for sgi received */
	u32	txmpdu_stbc;	/* count for stbc transmit */
	u32	rxmpdu_stbc;	/* count for stbc received */

	u32	rxundec_mcst;	/* dot11WEPUndecryptableCount */

	/* WPA2 counters (see rxundec for DecryptFailureCount) */
	u32	tkipmicfaill_mcst;	/* TKIPLocalMICFailures */
	u32	tkipcntrmsr_mcst;	/* TKIPCounterMeasuresInvoked */
	u32	tkipreplay_mcst;	/* TKIPReplays */
	u32	ccmpfmterr_mcst;	/* CCMPFormatErrors */
	u32	ccmpreplay_mcst;	/* CCMPReplays */
	u32	ccmpundec_mcst;	/* CCMPDecryptErrors */
	u32	fourwayfail_mcst;	/* FourWayHandshakeFailures */
	u32	wepundec_mcst;	/* dot11WEPUndecryptableCount */
	u32	wepicverr_mcst;	/* dot11WEPICVErrorCount */
	u32	decsuccess_mcst;	/* DecryptSuccessCount */
	u32	tkipicverr_mcst;	/* TKIPICVErrorCount */
	u32	wepexcluded_mcst;	/* dot11WEPExcludedCount */

	u32	dma_hang;	/* count for dma hang */
	u32	reinit;		/* count for reinit */

	u32  pstatxucast;	/* count of ucast frames xmitted on all psta assoc */
	u32  pstatxnoassoc;	/* count of txnoassoc frames xmitted on all psta assoc */
	u32  pstarxucast;	/* count of ucast frames received on all psta assoc */
	u32  pstarxbcmc;	/* count of bcmc frames received on all psta */
	u32  pstatxbcmc;	/* count of bcmc frames transmitted on all psta */

	u32  cso_passthrough; /* hw cso required but passthrough */
	u32	cso_normal;	/* hw cso hdr for normal process */
	u32	chained;	/* number of frames chained */
	u32	chainedsz1;	/* number of chain size 1 frames */
	u32	unchained;	/* number of frames not chained */
	u32	maxchainsz;	/* max chain size so far */
	u32	currchainsz;	/* current chain size */
	u32	rxdrop20s;	/* drop secondary cnt */
	u32	pciereset;	/* Secondary Bus Reset issued by driver */
	u32	cfgrestore;	/* configspace restore by driver */
	u32	reinitreason[NREINITREASONCOUNT]; /* reinitreason counters; 0: Unknown reason */
	u32  rxrtry;		/* num of received packets with retry bit on */
	u32	txmpdu;		/* number of MPDUs txed.  */
	u32	rxnodelim;	/* number of occasions that no valid delimiter is detected
				 * by ampdu parser.
				 */
} wl_cnt_t;

#define WLC_CNTRY_BUF_SZ        4               /* Country string is 3 bytes + NUL */

/* initialization */
int
tegra_sysfs_register(struct device *dev);

void
tegra_sysfs_unregister(struct device *dev);

int
tegra_sysfs_bus_register(struct device *dev);

void
tegra_sysfs_bus_unregister(struct device *dev);

void
tegra_sysfs_on(void);

void
tegra_sysfs_off(void);

void
tegra_sysfs_suspend(void);

void
tegra_sysfs_resume(void);

#ifdef CPTCFG_NV_CUSTOM_CAP
#include "nv_cap.h"
#endif
#ifdef CPTCFG_BRCMFMAC_NV_IDS
#include "nv_logger.h"
#endif
#endif  /* _nv_custom_sysfs_tegra_h_ */
