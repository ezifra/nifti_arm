/*
 * CPC-PCI CAN Interface Kernel Driver
 *
 * Copyright (C) 2000-2009 EMS Dr. Thomas Wuensche
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef CPCPCI_H
#define CPCPCI_H

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>

#include "../include/cpc.h"
#include "../include/cpc_int.h"

#define CPC_PCI_CARD_CNT      10
#define CPC_PCI_CHANNEL_CNT    4

#define CPC_PCI_CHANNEL_BASE  0x400
#define CPC_PCI_CHANNEL_WIDTH 0x200

enum {
	CPCPCI_V1 = 0,
	CPCPCI_V2
};

struct cpc_pci {
	void __iomem * ibase; // base address of PITA's internal registers
	void __iomem * base; // base address of CPC-PCI internal registers

	void (* clear_interrupts)(struct cpc_pci *);

	u16 irq;	// assigned irq
	spinlock_t slock;	// spinlock

	u32 idx;	// PCI number
	u32 busNo;	// Bus Number
	u32 slotNo;	// Slot Number
	u16 channels;	// available channels per card
	u32 serialNumber;

	u8 cardFullyInitialized;

	CPC_CHAN_T *chan[CPC_PCI_CHANNEL_CNT];
	struct miscdevice miscdev[CPC_PCI_CHANNEL_CNT];
	char deviceNames[CPC_PCI_CHANNEL_CNT][32];

	u32 revision;
};

#define CPCTable               CPCPCI_Table

#define CPC_INTERFACE_VERSION  "not applicable"
#define CPC_INTERFACE_SERIAL   "not applicable"

#define CPC_DRIVER_VERSION     "1.202"
#define CPC_DRIVER_SERIAL      "not applicable"

#endif /* CPCPCI_H */
