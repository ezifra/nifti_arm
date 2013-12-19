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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/poll.h>

#include <linux/device.h>

#include <linux/proc_fs.h>

#ifndef PCI_DEVICE_ID_PLX_9030
#define PCI_DEVICE_ID_PLX_9030	0x9030
#endif

#ifdef CONFIG_DEVFS_FS
#   include <linux/devfs_fs_kernel.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>

#include "../include/cpc.h"

#include "../include/cpc_int.h"
#include "../include/cpc_common.h"

#include "cpcpci.h"

#include "../include/version.h"

#include "../include/sja1000.h"
#include "../include/cpcpassive.h"

#include "pita_eeprom.h"
#include "plx9030_eeprom.h"

/* Claimed major number */
#define CPC_PCI_MAJOR 125

MODULE_AUTHOR("Sebastian Haas <haas@ems-wuensche.com>");
MODULE_AUTHOR("Markus Plessing <plessing@ems-wuensche.com>");
MODULE_DESCRIPTION("CAN PC Interface driver for CPC-PCI/PCIe CAN cards");
MODULE_SUPPORTED_DEVICE("EMS CPC PCI CAN card");
MODULE_LICENSE("GPL v2");

/* SA_SHIRQ replaced by IRQF_SHARED */
#ifndef IRQF_SHARED
#define IRQF_SHARED SA_SHIRQ
#endif

/* Module debug parameter */
static int debug = 0;
module_param(debug, int, S_IRUGO);

static struct cpc_pci *CPCPCI_Table[CPC_PCI_CARD_CNT];
static unsigned CPCPciCnt = 0;

#ifdef CONFIG_PROC_FS
#   define CPC_PCI_PROC_DIR     CPC_PROC_DIR "cpc-pci"

static struct proc_dir_entry *procDir = NULL;
static struct proc_dir_entry *procEntry = NULL;
static struct proc_dir_entry *procDebugEntry = NULL;
#endif

/*
 * use to prevent kernel panic if driver is unloaded
 * while a programm has still open the device
 */
DECLARE_WAIT_QUEUE_HEAD(rmmodWq);
atomic_t useCount;

/*
 * CPC-PCI v1 functions and definitions (PSB4610 PITA-2 PCI Bridge)
 */

/*
 * PSB4610 PITA-2 bridge control registers
 */
#define PITA2_ICR           0x00	    /* Interrupt Control Register */
#define PITA2_ICR_INT0      0x00000002	/* [RC] INT0 Active/Clear */
#define PITA2_ICR_INT0_EN   0x00020000	/* [RW] Enable INT0 */

#define PITA2_MISC          0x1c	    /* Miscellaneous Register */
#define PITA2_MISC_CONFIG   0x04000000	/* Multiplexed parallel interface */

/*
 * Read from card specific register
 */
static u8 pita2_card_read_byte(struct cpc_pci *card, u32 reg)
{
	return readb(card->base + (reg * 4));
}

/*
 * Write to channel specific register
 */
static void pita2_chan_write_byte(void *chan, u32 reg,
								  u8 val)
{
	writeb(val, ((CPC_CHAN_T *) chan)->canBase + (reg * 4));
}

/*
 * Write from channel specific register
 */
static u8 pita2_chan_read_byte(void *chan, u32 reg)
{
	return readb(((CPC_CHAN_T *) chan)->canBase + (reg * 4));
}

/*
 * Clear pending interrupts
 */
static void pita2_clear_interrupts(struct cpc_pci *card)
{
	writel(0x20002, card->ibase);
}

/*
 * CPC-PCI v2 definitions and functions (PLX 9030 PCI-to-IO Bridge)
 */

/*
 * PLX 9030 PCI-to-IO Bridge control registers
 */
#define PLX9030_ICR             0x4c  /* Interrupt Control Register */
#define PLX9030_ICR_CLEAR_IRQ0  0x400
#define PLX9030_ICR_ENABLE_IRQ0 0x41
/*
 * Write to channel specific register
 */
static void plx9030_chan_write_byte(void *chan, u32 reg,
									u8 val)
{
	writeb(val, ((CPC_CHAN_T *) chan)->canBase + reg);
}

/*
 * Write from channel specific register
 */
static u8 plx9030_chan_read_byte(void *chan, u32 reg)
{
	return readb(((CPC_CHAN_T *) chan)->canBase + reg);
}

/*
 * Clear pending interrupts
 */
static void plx9030_clear_interrupts(struct cpc_pci *card)
{
	writel(PLX9030_ICR_CLEAR_IRQ0|PLX9030_ICR_ENABLE_IRQ0, card->ibase + PLX9030_ICR);
}

/* kernel PCI callbacks */
static void __devexit cpcpci_remove_device(struct pci_dev *dev);
static int __devinit cpcpci_probe_device(struct pci_dev *dev,
				      const struct pci_device_id *entr);

/*
 * List of supported EMS PCI cards
 */
static struct pci_device_id __devinitdata cpcpci_pci_ids[] = {
	{PCI_VENDOR_ID_SIEMENS, 0x2104, PCI_ANY_ID, PCI_ANY_ID}, /* CPC-PCI v1 */
	{PCI_VENDOR_ID_PLX,     PCI_DEVICE_ID_PLX_9030, PCI_VENDOR_ID_PLX, 0x4000}, /* CPC-PCI v2 */
	{0,}
};

/* structure describing this driver */
static struct pci_driver CPCPciDriver = {
	.name = "cpc-pci",
	.id_table = cpcpci_pci_ids,
	.probe = cpcpci_probe_device,
	.remove = cpcpci_remove_device,
};

static int cpcpci_create_debug_output(char *buf)
{
	int i = 0, j, c = 0;

	for (j = 0; j < CPC_PCI_CARD_CNT; j++) {
		if (CPCPCI_Table[j]) {
			struct cpc_pci *card = CPCPCI_Table[j];

			for (c = 0; c < card->channels; c++) {
				CPC_CHAN_T *chan = card->chan[c];

				i += sprintf(&buf[i], "Channel %d\n", chan->minor);
				i += sprintf(&buf[i], "\tHandled IRQs           %d\n",
										chan->handledIrqs);

				i += sprintf(&buf[i], "\tLost messages          %d\n\n",
									chan->lostMessages);

				i += sprintf(&buf[i], "\tSent Standard CAN      %d\n",
							chan->sentStdCan);
				i += sprintf(&buf[i], "\tSent Standard RTR      %d\n",
							chan->sentStdRtr);
				i += sprintf(&buf[i], "\tSent Extended XCAN     %d\n",
							chan->sentExtCan);
				i += sprintf(&buf[i], "\tSent Extended XRTR     %d\n\n",
							chan->sentExtRtr);

				i += sprintf(&buf[i], "\tReceived Standard CAN  %d\n",
							chan->recvStdCan);
				i += sprintf(&buf[i], "\tReceived Standard RTR  %d\n",
							chan->recvStdRtr);
				i += sprintf(&buf[i], "\tReceived Extended XCAN %d\n",
							chan->recvExtCan);
				i += sprintf(&buf[i], "\tReceived Extended XRTR %d\n\n",
							chan->recvExtRtr);
			}
		}
	}

	return i;
}

static int cpcpci_proc_read_debug(char *page, char **start, off_t off,
								  int count, int *eof, void *data)
{
	int len = cpcpci_create_debug_output(page);

	if (len <= off + count)
		*eof = 1;

	*start = page + off;
	len -= off;

	if (len > count)
		len = count;

	if (len < 0)
		len = 0;

	return len;
}

static int cpcpci_create_info_output(char *buf)
{
	int i = 0, j, c = 0;

	for (j = 0; j < CPC_PCI_CARD_CNT; j++) {
		if (CPCPCI_Table[j]) {
			struct cpc_pci *card = CPCPCI_Table[j];

			for (c = 0; c < card->channels; c++) {
				/* MINOR CHANNELNO BUSNO SLOTNO */
				i += sprintf(&buf[i], "%d %d %d %d\n", (j * CPC_PCI_CHANNEL_CNT) + c, c,
										card->busNo, card->slotNo);
			}
		}
	}

	return i;
}

static int cpcpci_proc_read_info(char *page, char **start, off_t off,
								 int count, int *eof, void *data)
{
	int len = cpcpci_create_info_output(page);

	if (len <= off + count)
		*eof = 1;

	*start = page + off;
	len -= off;

	if (len > count)
		len = count;

	if (len < 0)
		len = 0;

	return len;
}

static irqreturn_t cpcpci_interrupt(int irq, void *dev_id
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
				    , struct pt_regs *regs
#endif
)
{
	struct cpc_pci *card = (struct cpc_pci *) dev_id;
	u32 chan_idx = 0;

	int handled = 0;

	struct timeval now;

	spin_lock(&card->slock);

	/* for time stamp */
	do_gettimeofday(&now);

	for (chan_idx = 0; chan_idx < card->channels; chan_idx++) {
		CPC_CHAN_T *chan = card->chan[chan_idx];
		if (sja1000_interrupt(chan, now)) {
			handled = 1;
		}
	}

	card->clear_interrupts(card);

	spin_unlock(&card->slock);

	return IRQ_RETVAL(handled);
}

/*
 * File operations
 */
static ssize_t cpcpci_read(struct file *file, char *buf, size_t count, loff_t * offset);
static ssize_t cpcpci_write(struct file *file, const char *buf, size_t count,
						   loff_t * offset);

static u32 cpcpci_poll(struct file *file,
							   struct poll_table_struct *wait);
static int cpcpci_open(struct inode *inode, struct file *file);
static int cpcpci_release(struct inode *inode, struct file *file);

/* fops table */
static struct file_operations CPCPciFops = {
      read:    cpcpci_read,
      write:   cpcpci_write,
      poll:    cpcpci_poll,
      open:    cpcpci_open,
      release: cpcpci_release
};

static void can_reset(CPC_CHAN_T * chan)
{
	chan->write_byte(chan, SJA1000_MODE, 1);
	chan->write_byte(chan, SJA1000_MODE, 0);
}

static ssize_t cpcpci_read(struct file *file, char *buf, size_t count,
						   loff_t * offset)
{
	CPC_CHAN_T *chan = file->private_data;
	struct cpc_pci *card = chan->private;

	CPC_MSG_T *cpc = NULL;

	unsigned long flags;

	if (count < sizeof(CPC_MSG_T))
		return CPC_ERR_UNKNOWN;

	/* check if we can write to the address provided by the caller */
	if (!access_ok(VERIFY_WRITE, buf, count))
		return CPC_ERR_UNKNOWN;

	spin_lock_irqsave(&card->slock, flags);
	cpc = cpc_get_next_message(chan);
	spin_unlock_irqrestore(&card->slock, flags);

	if (!cpc)
		return 0;

	if (copy_to_user(buf, cpc, sizeof(CPC_MSG_T)) != 0)
		return CPC_ERR_IO_TRANSFER;

	return sizeof(CPC_MSG_T);
}

static ssize_t cpcpci_write(struct file *file, const char *buf,
							size_t count, loff_t * offset)
{
	CPC_CHAN_T *chan = file->private_data;
	struct cpc_pci *card = chan->private;

	int retval = 0;
	unsigned long flags;
	CPC_MSG_T cpcbuf;

	if (count > sizeof(CPC_MSG_T) || !card || !chan)
		return CPC_ERR_UNKNOWN;

	/* check if can read from the given address */
	if (!access_ok(VERIFY_READ, buf, count))
		return CPC_ERR_UNKNOWN;

	/* copy message from user to kernel space */
	if (copy_from_user(&cpcbuf, buf, sizeof(CPC_MSG_T)) != 0)
		return CPC_ERR_IO_TRANSFER;

	spin_lock_irqsave(&card->slock, flags);

	if(cpcbuf.type == CPC_CMD_T_INQ_INFO) {
		if(cpcbuf.msg.info.source == CPC_INFOMSG_T_INTERFACE
			&& cpcbuf.msg.info.type == CPC_INFOMSG_T_SERIAL
			&& card->serialNumber) {

			if (IsBufferFull(chan))
				return CPC_ERR_UNKNOWN;


			chan->buf[chan->iidx].type = CPC_MSG_T_INFO;
			chan->buf[chan->iidx].msg.info.source = CPC_INFOMSG_T_INTERFACE;
			chan->buf[chan->iidx].msg.info.type = CPC_INFOMSG_T_SERIAL;

			if(card->serialNumber == 0xFFFFFFFF) { // EEPROM not flashed
				chan->buf[chan->iidx].length = 13 + 2;
				sprintf(chan->buf[chan->iidx].msg.info.msg, "empty EEPROM");
			} else {
				chan->buf[chan->iidx].length = 8 + 2;
				sprintf(&chan->buf[chan->iidx].msg.info.msg[0], "%07d", card->serialNumber);
			}

			chan->WnR = 0;
			chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;
			spin_unlock_irqrestore(&card->slock, flags);

			return CPCMSG_HEADER_LEN + 2;
		}
	}

	retval = cpc_process_command(chan, &cpcbuf);

	spin_unlock_irqrestore(&card->slock, flags);

	return retval;
}

static u32 cpcpci_poll(struct file *file,
								struct poll_table_struct *wait)
{
	CPC_CHAN_T *chan = file->private_data;
	int retval = 0;

	poll_wait(file, chan->CPCWait_q, wait);

	if ((chan->oidx != chan->iidx) || !(chan->WnR))
		retval |= (POLLIN | POLLRDNORM);

	if (chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)
		retval |= (POLLOUT | POLLWRNORM);

	return retval;

}

static int cpcpci_open(struct inode *inode, struct file *file)
{
	u32 card_idx, chan_idx;

	for(card_idx = 0; card_idx < CPCPciCnt; card_idx++) {
		struct cpc_pci *card = CPCTable[card_idx];

		for(chan_idx = 0; chan_idx < card->channels; chan_idx++) {
			if(card->chan[chan_idx]->minor == MINOR(inode->i_rdev))
				break;
		}

		if(chan_idx < card->channels)
			break;
	}

	if (card_idx >= CPCPciCnt) {
		err("CPCPci[-]: only %d device found in this PC!", CPCPciCnt);

		return CPC_ERR_NO_INTERFACE_PRESENT;
	} else {
		struct cpc_pci *card = CPCTable[card_idx];

		if (chan_idx >= card->channels) {
			err("CPCPci[-]: only single channel card in this slot!");

			return CPC_ERR_NO_MATCHING_CHANNEL;
		} else {
			CPC_CHAN_T *chan = card->chan[chan_idx];

			if (chan->locked) {
				err("CPCPci[%d]: Device is locked!", card_idx + chan_idx);

				return CPC_ERR_CHANNEL_ALREADY_OPEN;
		} else {
				ResetBuffer(chan);

				chan->irqDisabled = 0;
				chan->locked = 1;
				chan->cpcCtrlCANMessage = 0;
				chan->cpcCtrlCANState = 0;
				chan->cpcCtrlBUSState = 0;

				chan->ovrLockedBuffer = 0;
				chan->ovr.event = 0;
				chan->ovr.count = 0;

				file->private_data = card->chan[chan_idx];

				atomic_inc(&useCount);

				dbg("CPCPci[%d]: open", card_idx + chan_idx);
				return 0;
			}
		}
	}
}

static int cpcpci_release(struct inode *inode, struct file *file)
{
	CPC_CHAN_T *chan = file->private_data;

	chan->cpcCtrlCANMessage = 0;
	chan->cpcCtrlCANState = 0;

	sja1000_reset(chan);

	file->private_data = NULL;
	chan->locked = 0;
	chan->irqDisabled = 1;

	atomic_dec(&useCount);

	/* last process detached */
	if (atomic_read(&useCount) == 0) {
		wake_up(&rmmodWq);
	}

	dbg("CPCPci[-]: closed");

	return 0;

}

/*
 * Probe plugged in device for EMS CAN Interface
 */
static int __devinit cpcpci_probe_device(struct pci_dev *dev,
										 const struct pci_device_id *entr)
{
	static u32 card_idx = 0;
	static u32 minor = 0;

	struct cpc_pci *card = NULL;
	CPC_CHAN_T *chan = NULL;

	int i;

	if (CPCPciCnt < 0 || CPCPciCnt >= CPC_PCI_CARD_CNT) {
		err("CPCPci[-]: Max. supported devices exceeded (max. %d)",
		    CPC_PCI_CARD_CNT);
		return -ENODEV;
	}

	if (pci_enable_device(dev) < 0) {
		err("CPCPci[-]: Enabling PCI device failed");
		return -ENODEV;
	}
	// allocate basic structure for this card
	CPCTable[card_idx] = (struct cpc_pci *) kzalloc(sizeof(struct cpc_pci), GFP_KERNEL);
	if (CPCTable[card_idx] == NULL) {
		err("CPCPci[-]: Unable to get enough memory for device structure!");
		return -ENODEV;
	}
	card = CPCTable[card_idx];
	pci_set_drvdata(dev, card);

	spin_lock_init(&card->slock);
	card->idx = card_idx;
	minor = card_idx * CPC_PCI_CHANNEL_CNT;

	card->revision = CPCPCI_V2;
	if(entr->vendor == PCI_VENDOR_ID_SIEMENS)
		card->revision = CPCPCI_V1;

	if (card->revision == CPCPCI_V2) {
		/* The adapter is accessable through memory-access read/write, not
		 * I/O read/write. Thus, we need to map it to some virtual address
		 * area in order to access the registers as normal memory.
		 */
		card->ibase = pci_iomap(dev, 0, 128);
		card->base = pci_iomap(dev, 2, 2048);

		/*
		 * Register clear interrupts callback
		 */
		card->clear_interrupts = plx9030_clear_interrupts;

		card->serialNumber = plx9030_eeprom_get_serialnumber(card);
	} else {
		/* The adapter is accessable through memory-access read/write, not
		 * I/O read/write. Thus, we need to map it to some virtual address
		 * area in order to access the registers as normal memory.
		 */
		card->ibase = pci_iomap(dev, 0, 4096);
		card->base = pci_iomap(dev, 1, 4096);

		/* set multiplexed mode on PITAs parallel bus */
		writel(PITA2_MISC_CONFIG, card->ibase + PITA2_MISC);

		if (pita2_card_read_byte(card, 0) != 0x55 ||
		    pita2_card_read_byte(card, 1) != 0xAA ||
		    pita2_card_read_byte(card, 2) != 0x01 ||
		    pita2_card_read_byte(card, 3) != 0xCB ||
		    pita2_card_read_byte(card, 4) != 0x11) {
			err("CPCPci[-]: No EMS interface: ");
			for (i = 0; i < 5; i++)
				printk("%2.2X ", pita2_card_read_byte(card, i));
			printk("\n");

			cpcpci_remove_device(dev);

			return -ENODEV;
		}

		card->serialNumber = pita_eeprom_get_serialnumber(card);

		/*
		 * Register clear interrupts callback
		 */
		card->clear_interrupts = pita2_clear_interrupts;
	}

	info("CPCPci[%d]  : Revision %d found", card->idx, card->revision + 1);

	if (card->serialNumber != 0xFFFFFFFF)
		info("CPCPci[%d]  : Serialnumber %07u", card->idx, card->serialNumber);
	else
		info("CPCPci[%d]  : No serialnumber set", card->idx);

	card->busNo = dev->bus->number;
	card->slotNo = PCI_SLOT(dev->devfn);

	card->channels = 0;

	info("CPCPci[%d] : Control base address: %p", card->idx, card->ibase);

	for(i = 0; i < CPC_PCI_CHANNEL_CNT; i++) {
		void *channelBase = card->base + CPC_PCI_CHANNEL_BASE + (i * CPC_PCI_CHANNEL_WIDTH);

		if(card->revision == CPCPCI_V1) {
			writeb(SJA1000_CLK_PELICAN, channelBase + (SJA1000_CLK_DIV * 4));

			if(readb(channelBase + (SJA1000_CLK_DIV * 4)) != SJA1000_CLK_PELICAN)
				break;
		} else {
			writeb(SJA1000_CLK_PELICAN, channelBase + SJA1000_CLK_DIV);

			if(readb(channelBase + SJA1000_CLK_DIV) != SJA1000_CLK_PELICAN) {
				info("CPCPci[%d] : Wrong %X", card_idx, readb(channelBase + SJA1000_CLK_DIV));

				break;
			}
		}

		info("CPCPci[%d] : Found channel #%d", card_idx, i);
		card->chan[i] = (CPC_CHAN_T *)kzalloc(sizeof(CPC_CHAN_T), GFP_KERNEL);
		if(!card->chan[i])
			goto err;

		chan = card->chan[i];

		memset(chan, 0, sizeof(CPC_CHAN_T));

		/* set CAN controller base addresses */
		chan->canBase = channelBase;

		info("CPCPci[%d:%d] : SJA1000 base address: %p", card->idx, i, card->chan[i]->canBase);

		if(card->revision == CPCPCI_V1) {
			/* Set read/write handlers */
			chan->read_byte = pita2_chan_read_byte;
			chan->write_byte = pita2_chan_write_byte;
		} else {
			/* Set read/write handlers */
			chan->read_byte = plx9030_chan_read_byte;
			chan->write_byte = plx9030_chan_write_byte;
		}

		chan->CPCWait_q = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
		if(!chan->CPCWait_q)
			goto err;

		init_waitqueue_head(card->chan[i]->CPCWait_q);

		/* allocate message buffer memory */
		chan->buf = kzalloc(sizeof(CPC_MSG_T) * CPC_MSG_BUF_CNT, GFP_KERNEL);
		if(!chan->buf)
			goto err;

#ifdef CONFIG_DEVFS_FS
		/* create device node */
		devfs_mk_cdev(MKDEV(CPC_PCI_MAJOR, chan->minor),
					  S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP
				      | S_IWGRP | S_IROTH | S_IWOTH,
				      "cpc_pci%d", chan->minor);
#endif
		snprintf(&card->deviceNames[i][0], 32, "cpc_pci%d", minor);

		memset(&card->miscdev[i], 0, sizeof(card->miscdev[i]));

		card->miscdev[i].minor = MISC_DYNAMIC_MINOR;
		card->miscdev[i].fops = &CPCPciFops;
		card->miscdev[i].name = &card->deviceNames[i][0];
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
		card->miscdev[i].parent = &dev->dev;
#endif
		misc_register(&card->miscdev[i]);

		chan->minor = card->miscdev[i].minor;

		chan->private = card;

		chan->locked = 0;	// So we can open the device
		chan->controllerType = SJA1000;

		dbg("CPCPci[%d:%d] : Message buffer allocated", card_idx, i);

		can_reset(card->chan[i]);

		card->channels++;
		minor++;
	}

	card->irq = dev->irq;

	/* setup interrupt */
	if (request_irq(card->irq, cpcpci_interrupt, IRQF_SHARED, "cpc_pci", card)) {
		err("CPCPci[%d] : Failed request interrupt IRQ%d!",
		    card->idx, card->irq);

		return -EIO;
	}
	info("CPCPci[%d] : BusNo. %d, SlotNo. %02X", card_idx, card->busNo,
	     card->slotNo);
	info("CPCPci[%d] : Assigned IRQ%d", card_idx, card->irq);

	disable_irq(card->irq);

	if(card->revision == CPCPCI_V1) {
		/* enable IRQ in PITA */
		writel(0x20000, (u32 *) card->ibase);
	} else {
		/* enable IRQ in PLX 9030 */
		writel(PLX9030_ICR_ENABLE_IRQ0, card->ibase + PLX9030_ICR);
	}

	enable_irq(card->irq);

	card_idx++;
	CPCPciCnt++;

	return 0;

err:
	err("CPCPci[-]: No memory available");

	cpcpci_remove_device(dev);

	return -ENOMEM;
}

/*
 * Remove EMS CAN PCI interface
 */
static void __devexit cpcpci_remove_device(struct pci_dev *dev)
{
	struct cpc_pci *card = pci_get_drvdata(dev);
	CPC_CHAN_T *chan = NULL;
	int i = 0;

	/* freeing card ressources */
	if (card->irq)
		free_irq(card->irq, card);

	/* unmap memory I/O addresses */
	pci_iounmap(dev, card->ibase);
	pci_iounmap(dev, card->base);

	for (i = 0; i < card->channels; i++) {
		if (card->chan[i]) {
			chan = card->chan[i];
#ifdef CONFIG_DEVFS_FS
			/* remove devfs entry */
			devfs_remove("cpc_pci%d", chan->minor);
#endif

			misc_deregister(&card->miscdev[i]);

			chan->read_byte = NULL;
			chan->write_byte = NULL;

			/* free waitqueue */
			if (chan->CPCWait_q)
				kfree(chan->CPCWait_q);

			/* free message buffer */
			if (chan->buf)
				kfree(chan->buf);

			/* free the channel */
			kfree(chan);
		}
	}

	if (card->cardFullyInitialized)
		CPCPciCnt--;

	/* mark table entry as unused */
	CPCPCI_Table[card->idx] = NULL;

	/* free card */
	kfree(card);
	/* remove the card from pci driver data field */
	pci_set_drvdata(dev, NULL);
	pci_disable_device(dev);
}

static int __init CPCPciInit(void)
{
	int i, ret;

	info("CPCPci[-] : Loading Driver v%s, build on %s at %s",
	     CPC_DRIVER_VERSION, __DATE__, __TIME__);
	info("CPCPci[-] : SJA1000 Driver v%s", SJA1000_VERSION);

	atomic_set(&useCount, 0);

	procDir = proc_mkdir(CPC_PCI_PROC_DIR, NULL);
	if (procDir) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
		procDir->owner = THIS_MODULE;
#endif

		procEntry = create_proc_read_entry("info", 0444, procDir,
						   cpcpci_proc_read_info, NULL);
		if (!procEntry) {
			err("CPCPci[-]: Could not create proc entry %s",
			    CPC_PCI_PROC_DIR "/info");
			remove_proc_entry(CPC_PCI_PROC_DIR, NULL);
			procDir = NULL;
		} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procEntry->owner = THIS_MODULE;
#endif
		}

		procDebugEntry =
		    create_proc_read_entry("debug", 0444, procDir,
					   cpcpci_proc_read_debug, NULL);
		if (!procDebugEntry) {
			err("CPCPci[-]: Could not create proc entry %s",
					CPC_PCI_PROC_DIR "/debug");
		} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procDebugEntry->owner = THIS_MODULE;
#endif
		}
	}

	for (i = 0; i < CPC_PCI_CARD_CNT; i++)
		CPCPCI_Table[i] = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
	ret = pci_register_driver(&CPCPciDriver);
#else
	pci_module_init(&CPCPciDriver);
#endif

	return 0;
}

static void __exit CPCPciExit(void)
{
	wait_event(rmmodWq, !atomic_read(&useCount));

	if (procDir) {
		if (procEntry)
			remove_proc_entry("info", procDir);

		if (procDebugEntry)
			remove_proc_entry("debug", procDir);
		remove_proc_entry(CPC_PCI_PROC_DIR, NULL);
	}

	pci_unregister_driver(&CPCPciDriver);

	info("CPCPci[-]   : Driver unloaded!");
}

module_init(CPCPciInit);
module_exit(CPCPciExit);
