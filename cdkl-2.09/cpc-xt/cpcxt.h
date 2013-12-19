/*
 * CPC-XT CAN Interface Kernel Driver
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
#ifndef CPCXT_H
#define CPCXT_H

#include <linux/miscdevice.h>

#define CPC_XT_CARD_CNT      3
#define CPC_XT_CHANNEL_CNT   4

typedef void (*card_write_byte_t) (void *card, unsigned int reg,
				   unsigned char val);
typedef unsigned char (*card_read_byte_t) (void *card, unsigned int reg);

typedef struct CPC_XT {
	void __iomem * base; /* base address of CPC-XT internal registers */
	unsigned long jumpered_address;
	card_read_byte_t read_byte;	/* card read access routine */
	card_write_byte_t write_byte;	/* card write access routine */
	int irq;		/* assigned irq */
	spinlock_t slock;	/* spinlock */
	unsigned int idx;	/* PCI number */
	unsigned char canControllerType;

	CPC_CHAN_T *chan[CPC_XT_CHANNEL_CNT];
	struct miscdevice miscdev[CPC_XT_CHANNEL_CNT];
	char deviceNames[CPC_XT_CHANNEL_CNT][32];

	int channel_count;
} CPC_XT_T;

#define CPCTable               CPCXT_Table

#define CPC_INTERFACE_VERSION  "not applicable"
#define CPC_INTERFACE_SERIAL   "not applicable"

#define CPC_DRIVER_VERSION     "1.202"
#define CPC_DRIVER_SERIAL      "not applicable"

#define CPC_XT_DRIVER_NUM      CPCXT

#define CAN_OFFSET              0x100

#define XT_START_ADDR           0xc0000
#define XT_END_ADDR             0xde000

#define XT_STEP_WIDTH           0x2000

#define XT_SIZE_OF_ADDR_SPACE   ((XT_END_ADDR-XT_START_ADDR)+0x2000)

#define XT_CONTR_PCA82C200      0x02
#define XT_CONTR_SJA1000        0x08
#define XT_CONTR_CC770          0x01

#define XT_CARD_REG_CONTR       4
#define XT_CARD_REG_STATUS      6

/* Only CPC-104MC */
#define XT_CARD_REG_IRQ_CTRL    7
#define XT_CARD_REG_IRQ_STATUS  8
#define XT_CARD_REG_VERSION     9

#define XT_CARD_REG_CONTROL     0

#define XT_CARD_CMD_HW_RESET    0
#define XT_CARD_CMD_MAP_CONTR   3
#define XT_CARD_CMD_UMAP_CONTR  2

/* Only Soudronic Cards */
#define XT_CARD_STEINHOFF_CHECK 5

#define XT_CARD_IRQ_NOT_ASSIGNED -1

#define XT_CLEANUP_IRQ(x)  do {\
										if(x->irq != XT_CARD_IRQ_NOT_ASSIGNED) {\
											disable_irq(x->irq);\
											free_irq(x->irq, x);\
											x->irq = XT_CARD_IRQ_NOT_ASSIGNED;\
										}\
									} while(0);

#endif				/* CPCXT_H */
