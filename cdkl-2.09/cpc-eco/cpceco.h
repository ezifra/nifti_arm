/*
 * CPC-ECO CAN Interface Kernel Driver
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
#ifndef CPCECO_H
#define CPCECO_H

#include <linux/miscdevice.h>

typedef void (*card_write_byte_t) (void *card, unsigned int reg,
				   unsigned char val);
typedef unsigned char (*card_read_byte_t) (void *card, unsigned int reg);

typedef struct CPC_ECO {
	unsigned long PA;	// base address of related printer port
	unsigned int irq;	// assigned irq
	unsigned int portnum;	// real port number
	int claimed;

	unsigned long irqCounter;
	CPC_CHAN_T *chan;
	struct pardevice *pdev;
	struct parport *pport;
	char procEntryName[64];
	char *procBuf;
	size_t procBufSize;
	spinlock_t slock;	// spinlock irqsave
	spinlock_t ilock;	// spinlock irq
	card_read_byte_t read_byte;	// card read access routine
	card_write_byte_t write_byte;	// card write access routine

	struct miscdevice miscdev;
	char deviceName[32];
} CPC_ECO_T;

#define CPCTable               CPCECO_Table

#define CAN_OFFSET             0

#define CPC_INTERFACE_VERSION  "not applicable"
#define CPC_INTERFACE_SERIAL   "not applicable"

#define CPC_DRIVER_VERSION     "1.502"
#define CPC_DRIVER_SERIAL      "not applicable"

#endif // CPCECO_H
