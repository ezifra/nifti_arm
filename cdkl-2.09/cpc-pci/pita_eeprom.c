/*
 * CPC-PCI CAN Interface Kernel Driver
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
#include <asm/io.h>

#include <linux/delay.h>

#include "pita_eeprom.h"

/* EEPROM stuff */
#define EE_READ    0x00030000
#define EE_START   0x01000000

#define eeprom_busy()   readl(ibase)&0x01000000

/*
 * Read 1 byte at given address from EEPROM
 */
static u8 eeprom_read(struct cpc_pci *card, u8 addr)
{
	u32 *ibase = (u32 *) (card->ibase + 0x24);

	int timeout;
	unsigned long ee_ctrl;

	ee_ctrl = EE_READ | (((u32)addr) << 8);
	writel(ee_ctrl, ibase);

	ee_ctrl = EE_READ | (((u32)addr) << 8) | EE_START;
	writel(ee_ctrl, ibase);

	timeout = 0;
	do {
		timeout++;
		udelay(5);

		if (timeout > 100000)
			break;

	} while (eeprom_busy());

	if (timeout > 99999)
		printk(KERN_INFO "read timeout %d\n", timeout);

	return readl(ibase);
}

/*
 * Get serialnumber of the board
 */
u32 pita_eeprom_get_serialnumber(struct cpc_pci *card)
{
	u32 serial = 0;

	serial = eeprom_read(card, 0x80);
	serial |= eeprom_read(card, 0x81) << 8;
	serial |= eeprom_read(card, 0x82) << 16;
	serial |= eeprom_read(card, 0x83) << 24;

	return serial;
}
