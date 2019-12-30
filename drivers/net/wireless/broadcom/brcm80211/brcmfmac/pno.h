/*
 * Copyright (c) 2016 Broadcom
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
#ifndef _BRCMF_PNO_H
#define _BRCMF_PNO_H

#define BRCMF_PNO_SCAN_COMPLETE		1
#define BRCMF_PNO_MAX_PFN_COUNT		16

/**
 * brcmf_pno_clean - disable and clear pno in firmware.
 *
 * @ifp: interface object used.
 */
int brcmf_pno_clean(struct brcmf_if *ifp);

/**
 * brcmf_pno_config - configure pno parameters.
 *
 * @ifp: interface object used.
 * @scan_freq: scan frequency period in seconds.
 * @mscan: maximum number of scans stored in firmware.
 * @bestn: maximum number of APs per scan stored in firmware.
 */
int brcmf_pno_config(struct brcmf_if *ifp, u32 scan_freq,
		     u32 mscan, u32 bestn);

/**
 * brcmf_pno_set_random - setup randomisation mac address for pno.
 *
 * @ifp: interface object used.
 * @mac_addr: MAC address used with randomisation.
 * @mac_mask: MAC address mask used for randomisation.
 */
int brcmf_pno_set_random(struct brcmf_if *ifp, u8 *mac_addr, u8 *mac_mask);

/**
 * brcmf_pno_add_ssid - add ssid for pno in firmware.
 *
 * @ifp: interface object used.
 * @ssid: ssid information.
 * @active: indicate this ssid needs to be actively probed.
 */
int brcmf_pno_add_ssid(struct brcmf_if *ifp, struct cfg80211_ssid *ssid,
		       bool active);

#endif /* _BRCMF_PNO_H */
