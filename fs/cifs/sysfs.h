/*
 * sysfs header for cifs
 *
 * Copyright (c) 2018-2021, NVIDIA Corporation. All Rights Reserved.
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
#ifndef _CIFS_SYSFS_H
#define _CIFS_SYSFS_H

enum cifs_event_type {
	CONNECTED = 1,
	DISCONNECTED,
	RECONNECTING,
	RECONNECTED,
	SMB_ERROR,
	RECONNECT_ERROR
};

void cifs_sysfs_notify_change(const char *source, enum cifs_event_type type);
int cifs_sysfs_init(void);
void cifs_sysfs_exit(void);
#endif
