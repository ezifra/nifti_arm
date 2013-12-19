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

#include "plx9030_eeprom.h"

#define EE_CLK 24
#define EE_CS  25
#define EE_WR  26
#define EE_RD  27
#define EE_PR  28

#define cntrl *(volatile u32 *)(card->ibase + 0x50)

#define SET_BIT(x)   (cntrl |= (1 << x))
#define CLEAR_BIT(x) (cntrl &= ~(1 << x))

#define GET_BIT(x)   ((cntrl >> x) & 1)

#define TOGGLE_BIT(x) do { if (GET_BIT(x)) CLEAR_BIT(x); else SET_BIT(x); } while(0)
#define TICK() do { TOGGLE_BIT(EE_CLK); udelay(1); TOGGLE_BIT(EE_CLK); } while(0)

#define EE_OP_WRDS  0x00
#define EE_OP_WRITE 0x01
#define EE_OP_READ  0x02

/*
 * Send command to EEPROM
 */
static void eeprom_send_command(struct cpc_pci *card, u8 opcode, u16 val)
{
	int i;

	// Start bit
	SET_BIT(EE_WR);

	TICK();

	// Op-Code read
	if ((opcode >> 1) & 1)
		SET_BIT(EE_WR);
	else
		CLEAR_BIT(EE_WR);

	TICK();

	if (opcode & 1)
		SET_BIT(EE_WR);
	else
		CLEAR_BIT(EE_WR);

	TICK();

	for (i = 7; i >= 0; i--) {
		int bit = (val >> i) & 1;

		if (bit)
			SET_BIT(EE_WR);
		else
			CLEAR_BIT(EE_WR);

		TICK();
	}

	return;
}

/*
 * Read word from EEPROM
 */
static u16 eeprom_read(struct cpc_pci *card, u8 addr)
{
	int i;
	u16 val = 0;

	// Select chip
	SET_BIT(EE_CS);

	eeprom_send_command(card, EE_OP_READ, addr);

	for (i = 15; i >= 0; i--) {
		TICK();

		val |= GET_BIT(EE_RD) << i;
	}

	// Deselect chip
	CLEAR_BIT(EE_CS);

	return val;
}

/*
 * Enable write
 */
static void eeprom_write_enable(struct cpc_pci *card)
{
	// Select chip
	SET_BIT(EE_CS);

	eeprom_send_command(card, EE_OP_WRDS, 0xC0);

	// Deselect chip
	CLEAR_BIT(EE_CS);
}

/*
 * Disable write
 */
static void eeprom_write_disable(struct cpc_pci *card)
{
	// Select chip
	SET_BIT(EE_CS);

	eeprom_send_command(card, EE_OP_WRDS, 0x00);

	// Deselect chip
	CLEAR_BIT(EE_CS);
}

/*
 * Write word to EEPROM
 */
static void eeprom_write(struct cpc_pci *card, u8 addr, u16 val)
{
	int i;

	// Select chip
	SET_BIT(EE_CS);

	eeprom_send_command(card, EE_OP_WRITE, addr);

	for (i = 15; i >= 0; i--) {
		int bit = (val >> i) & 1;

		if (bit)
			SET_BIT(EE_WR);
		else
			CLEAR_BIT(EE_WR);

		TICK();
	}

	// Deselect chip
	CLEAR_BIT(EE_CS);

	TICK();

	SET_BIT(EE_CS);

	i = 0;
	do {
		i++;
		udelay(5);

		if (i > 100000)
			break;

	} while (!GET_BIT(EE_RD));

	printk(KERN_INFO "timeout %d\n", i);

	udelay(1000);

	CLEAR_BIT(EE_CS);
}

/*
 * Get serialnumber of the board
 */
u32 plx9030_eeprom_get_serialnumber(struct cpc_pci *card)
{
	u32 serial = 0;

	serial = eeprom_read(card, 0x44);
	serial |= eeprom_read(card, 0x45) << 16;

	return serial;
}
