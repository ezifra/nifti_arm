/*
 * CPC-Card CAN Interface Kernel Driver
 *
 * Copyright (C) 2000-2008 EMS Dr. Thomas Wuensche
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
#ifndef CPCCARD_H
#define CPCCARD_H

#include <linux/interrupt.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ciscode.h>
#include <pcmcia/ds.h>

#define CPC_CARD_MAX_DEVICE  2	// only one device is supported at the moment
#define CPC_CARD_MAJOR       120
#define CPC_CARD_CHANNEL_CNT 2

#define CTRL_PCA82C200 0x04
#define CTRL_SJA1000   0x05

typedef struct CPC_CARD {
	void __iomem * base; // base address
	unsigned int irq; // assigned irq
	spinlock_t slock;
	dev_node_t node;
	unsigned int channels;
	unsigned int present;
	CPC_CHAN_T *chan[CPC_CARD_CHANNEL_CNT];
} CPC_CARD_T;

#define CPCTable      CPCCARD_Table

CPC_CARD_T *CPCCard_register(unsigned long base /*, unsigned int irq */ );
void        CPCCard_unregister(CPC_CARD_T * card);
irqreturn_t CPCCard_interrupt(int irq, void *dev_id, struct pt_regs *regs);

#define CPC_INTERFACE_VERSION "not applicable"
#define CPC_INTERFACE_SERIAL  "not applicable"

#define CPC_DRIVER_VERSION    "1.324"
#define CPC_DRIVER_SERIAL     "not applicable"

#define LOGIC_BASE            0x00	// LOGIC_BASE is where the card is found

#define CAN0_OFFSET_PELICAN	  0x100	// offset of canBase to LOGIC_BASE
#define CAN1_OFFSET_PELICAN	  0x180

#define CAN_OFFSET            0

#define RESET                 0x00	// Cmd to perform a reset of the card
#define MAP                   0x03	// Cmd to map can controller
#define UMAP                  0x02	// Cmd to unmap can controller

#endif				//CPCCARD_H
