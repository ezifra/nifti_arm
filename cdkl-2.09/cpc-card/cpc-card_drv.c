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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/interrupt.h>

#include <linux/device.h>

#include <linux/proc_fs.h>

#ifdef CONFIG_DEVFS_FS
#   include <linux/devfs_fs_kernel.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>

#include "../include/cpc.h"

#include "../include/cpc-class.h"

#include "../include/cpc_int.h"
#include "../include/cpc_common.h"

#include "../include/version.h"

#include "cpccard.h"

#include "../include/cpcpassive.h"

#include "../include/sja1000.h"
#include "../include/pca82c200.h"

#ifndef CONFIG_PROC_FS
#error "Need PROCFS support!"
#endif

MODULE_AUTHOR("Gerhard Uttenthaler <uttenthaler@ems-wuensche.com>, "\
		      "Sebastian Haas <haas@ems-wuensche.com>");
MODULE_DESCRIPTION("CPC-CARD Driver for Linux Kernel 2.6");
MODULE_VERSION(CPC_DRIVER_VERSION " (CDKL v" CDKL_VERSION ")");
MODULE_LICENSE("GPL");

#define CPC_CARD_PROC_DIR     CPC_PROC_DIR "cpc-card"

static struct proc_dir_entry *procDir = NULL;
static struct proc_dir_entry *procEntry = NULL;

static int debug = 0;
module_param(debug, int, S_IRUGO);

CPC_CARD_T CPCCARD_Table[CPC_CARD_MAX_DEVICE];

static int CPCCard_get_free_slot(void);
static int CPCCardCnt = 0;

static void CPCCard_delete(CPC_CARD_T * card);

/* use to prevent kernel panic if driver is unloaded
 * while a programm has still open the device
 */
DECLARE_WAIT_QUEUE_HEAD(rmmodWq);
atomic_t useCount;

static int CPCCard_create_info_output(char *buf)
{
	int i = 0, j, c;

	for (j = 0; j < CPC_CARD_MAX_DEVICE; j++) {
		if (CPCCARD_Table[j].present) {
			CPC_CARD_T *card = &CPCCARD_Table[j];
			for (c = 0; c < card->channels; c++) {
				CPC_CHAN_T *chan = card->chan[c];

				/* MINOR CHANNELNO PORTNO */
				i += sprintf(&buf[i], "%d %d %d\n", chan->minor, c, j);
			}
		}
	}

	return i;
}


static int CPCCard_proc_read_info(char *page, char **start, off_t off,
                                  int count, int *eof, void *data)
{
	int len = CPCCard_create_info_output(page);

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

static void chan_write_byte(void *chan, unsigned int reg, unsigned char val)
{
	writeb(val, ((CPC_CHAN_T *) chan)->canBase + reg);
}

static unsigned char chan_read_byte(void *chan, unsigned int reg)
{
	return readb(((CPC_CHAN_T *) chan)->canBase + reg);
}

/**
 * Device interrupt
 */
irqreturn_t CPCCard_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	unsigned int chan_idx = 0;
	volatile CPC_CARD_T *card = NULL;
	struct timeval now = { 0, 0 };
	int handled = 0;
	int try_again = 1;

	card = (CPC_CARD_T *) dev_id;

	if (!card) {
		info("CPC-CARD(%s:%d): card == NULL", __PRETTY_FUNCTION__, __LINE__);
		return IRQ_RETVAL(0);
	}

	/* card unplugged? */
	if (!card->present || readw((unsigned short *) card->base) != 0xAA55) {
		info("CPC-CARD: Spurious irq, ignoring");
		return IRQ_RETVAL(1);
	}

	do {
		try_again = 0;

		/* for time stamp */
		do_gettimeofday(&now);

		for (chan_idx = 0; chan_idx < card->channels; chan_idx++) {
			CPC_CHAN_T *chan = card->chan[chan_idx];

			if (chan->controllerType == PCA82C200) {
				if (pca82c200_interrupt(chan, now)) {
					handled = 1;
					try_again = 1;
				}
			} else if (chan->controllerType == SJA1000) {
				if (sja1000_interrupt(chan, now)) {
					handled = 1;
					try_again = 1;
				}
			}
		}
	} while (try_again != 0);

	return IRQ_RETVAL(handled);
}

EXPORT_SYMBOL_GPL(CPCCard_interrupt);

static ssize_t CPCCard_read(struct file *file, char *buf, size_t count,
                            loff_t * offset);
static ssize_t CPCCard_write(struct file *file, const char *buf,
                             size_t count, loff_t * offset);
static unsigned int CPCCard_poll(struct file *file,
                                 struct poll_table_struct *wait);
static int CPCCard_open(struct inode *inode, struct file *file);
static int CPCCard_release(struct inode *inode, struct file *file);

static struct file_operations CPCCardFops = {
	read:    CPCCard_read,
	write:   CPCCard_write,
	poll:    CPCCard_poll,
	open:    CPCCard_open,
	release: CPCCard_release
};

static ssize_t CPCCard_read(struct file *file, char *buf, size_t count,
                            loff_t * offset)
{
	unsigned int card_idx =
	    MINOR(file->f_dentry->d_inode->i_rdev) / CPC_CARD_CHANNEL_CNT;
	CPC_CHAN_T *chan = file->private_data;
	CPC_MSG_T *cpc = NULL;
	unsigned long flags;

	if (!CPCTable[card_idx].present)
		return CPC_ERR_NO_INTERFACE_PRESENT;

	if (count < sizeof(CPC_MSG_T))
		return CPC_ERR_UNKNOWN;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return CPC_ERR_UNKNOWN;

	spin_lock_irqsave(&CPCTable[card_idx].slock, flags);
	cpc = cpc_get_next_message(chan);
	spin_unlock_irqrestore(&CPCTable[card_idx].slock, flags);

	if (!cpc)
		return 0;

	if (copy_to_user(buf, cpc, sizeof(CPC_MSG_T)) != 0)
		return CPC_ERR_IO_TRANSFER;

	return sizeof(CPC_MSG_T);
}

/**
 * Command from application
 */
static ssize_t CPCCard_write(struct file *file, const char *buf,
		                     size_t count, loff_t * offset)
{
	unsigned int card_idx =
		MINOR(file->f_dentry->d_inode->i_rdev) / CPC_CARD_CHANNEL_CNT;
	CPC_CHAN_T *chan = file->private_data;
	int retval = 0;
	unsigned long flags;
	CPC_MSG_T cpcbuf;

	if (!CPCTable[card_idx].present)
		return CPC_ERR_NO_INTERFACE_PRESENT;

	if (count > sizeof(CPC_MSG_T))
		return CPC_ERR_UNKNOWN;

	/* check if can read from the given address */
	if (!access_ok(VERIFY_READ, buf, count))
		return CPC_ERR_UNKNOWN;

	if (copy_from_user(&cpcbuf, buf, sizeof(CPC_MSG_T)) != 0)
		return CPC_ERR_IO_TRANSFER;

	spin_lock_irqsave(&CPCTable[card_idx].slock, flags);
	retval = cpc_process_command(chan, &cpcbuf);
	spin_unlock_irqrestore(&CPCTable[card_idx].slock, flags);

	return retval;
}

static unsigned int CPCCard_poll(struct file *file,
		                         struct poll_table_struct *wait)
{
	CPC_CHAN_T *chan = file->private_data;
	unsigned int card_idx =
		MINOR(file->f_dentry->d_inode->i_rdev) / CPC_CARD_CHANNEL_CNT;
	int retval = 0;

	poll_wait(file, chan->CPCWait_q, wait);

	/* if the device was unplugged the fields are invalid */
	if (!CPCTable[card_idx].present)
		return (POLLIN | POLLRDNORM);

	if ((chan->oidx != chan->iidx) || !(chan->WnR))
		retval |= (POLLIN | POLLRDNORM);

	if (chan->controllerType == PCA82C200) {
		if (chan->
		    read_byte(chan, PCA82C200_STATUS) & PCA82C200_SR_TBS)
			retval |= (POLLOUT | POLLWRNORM);
	} else if (chan->controllerType == SJA1000) {
		if (chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)
			retval |= (POLLOUT | POLLWRNORM);
	} else {
		err("Wrong controller %d", chan->controllerType);
	}

	return retval;
}

/**
 * Application wants to open a channel
 */
static int CPCCard_open(struct inode *inode, struct file *file)
{
	unsigned int card_idx = MINOR(inode->i_rdev) / CPC_CARD_CHANNEL_CNT;
	unsigned int chan_idx = MINOR(inode->i_rdev) % CPC_CARD_CHANNEL_CNT;

	CPC_CARD_T *card = NULL;
	CPC_CHAN_T *chan = NULL;

	if (card_idx >= CPCCardCnt) {
		err("CPC-CARD - only %d device(s) found in this PC!", CPCCardCnt);
		return CPC_ERR_NO_INTERFACE_PRESENT;
	}

	card = &CPCTable[card_idx];
	if (!card->present)
		return CPC_ERR_NO_INTERFACE_PRESENT;

	if (chan_idx >= card->channels) {
		err("CPC-CARD - only single channel card in this slot!");
		return CPC_ERR_NO_MATCHING_CHANNEL;
	}

	chan = card->chan[chan_idx];

	if (chan->locked) {
		err("CPC-CARD[%d] - Device is locked!", card_idx + chan_idx);
		return CPC_ERR_CHANNEL_ALREADY_OPEN;
	}

	ResetBuffer(chan);

	chan->irqDisabled = 0;
	chan->locked = 1;

	chan->cpcCtrlCANMessage = 0;
	chan->cpcCtrlCANState = 0;
	chan->cpcCtrlBUSState = 0;

	chan->ovrLockedBuffer = 0;
	chan->ovr.event = 0;
	chan->ovr.count = 0;

	file->private_data = chan;

	/* To prevent the module is removed while
	 * an channel is opened
	 */
	atomic_inc(&useCount);
	dbg("CPC-CARD(DEBUG): INC useCount %d", atomic_read(&useCount));
	dbg("CPC-CARD[%d]: opened", card_idx + chan_idx);

	return 0;
}

/**
 * Application closed the channel
 */
static int CPCCard_release(struct inode *inode, struct file *file)
{
	unsigned int card_idx = MINOR(inode->i_rdev) / CPC_CARD_CHANNEL_CNT;
	unsigned int j = 0;
	unsigned int lockedCh = 0;
	CPC_CHAN_T *chan = file->private_data;

	if (!chan)
		return CPC_ERR_UNKNOWN;

	atomic_dec(&useCount);
	dbg("CPC-CARD(DEBUG): DEC useCount %d", atomic_read(&useCount));

	chan->cpcCtrlCANMessage = 0;
	chan->cpcCtrlCANState = 0;

	/* last process detached */
	if (atomic_read(&useCount) == 0) {
		wake_up(&rmmodWq);
	}

	chan->locked = 0;
	chan->irqDisabled = 1;

	/* cleanup, card has been already unplugged */
	if (!CPCTable[card_idx].present) {
		lockedCh = 0;
		/* check if we're closed the last opened channel
		 * (the last closes the door)
		 */
		for (j = 0; j < CPCTable[card_idx].channels; j++) {
			if (CPCTable[card_idx].chan[j]) {
				if (CPCTable[card_idx].chan[j]->locked) {
					lockedCh++;
				}
			}
		}

		/* if no channel is locked we could delete
		 * the card structure safety
		 */
		if (lockedCh == 0) {
			CPCCard_delete(&CPCTable[card_idx]);
		}
	} else {
		/* reset controller */
		if (chan->controllerType == PCA82C200) {
			pca82c200_reset(chan);
		} else {
			sja1000_reset(chan);
		}
	}

	file->private_data = NULL;

	return 0;
}

/**
 * Find free slot
 */
static inline int CPCCard_get_free_slot(void)
{
	unsigned int i;

	for (i = 0; i < CPC_CARD_MAX_DEVICE; i++)
		if (!CPCTable[i].present)
			return i;

	return -1;
}

/**
 * cpc-card_cs inform us about a newly plugged in card
 * @param base
 * @return card structure
 */
CPC_CARD_T *CPCCard_register(unsigned long base)
{
	CPC_CARD_T *card = NULL;
	unsigned int timeout = 0;
	int slot, i, minor;
	unsigned char contr;
	char devname[64];

	info("CPC-CARD: Base %8.8lXh", base);

	if ((slot = CPCCard_get_free_slot()) < 0) {
		err("CPC-CARD: No free slots anymore");
		return NULL;
	}

	card = &CPCTable[slot];
	memset(card, 0, sizeof(CPC_CARD_T));

	card->base = ioremap(base, 1024);

	/* init the spinlock for irqsave and irqrestore */
	spin_lock_init(&card->slock);

	/* Map the CAN controllers */
	writeb(MAP, card->base + LOGIC_BASE);

	/* check if this is an CPC-CARD */
	if (readw(card->base) != 0xAA55) {
		info("CPC-CARD: No EMS CPC-CARD found");
		goto err;
	}

	contr = readb(card->base + 4);
	if (contr == CTRL_PCA82C200) {
		info("CPC-CARD: Found valid card with PCA82C200 controller");
		writeb(0xFA, (card->base + PCA82C200_OFFSET_0 + PCA82C200_OUTP_CONTR));
		writeb(0xFA, (card->base + PCA82C200_OFFSET_1 + PCA82C200_OUTP_CONTR));

		udelay(100);

		card->channels = 0;
		if (readb(card->base + PCA82C200_OFFSET_0 + PCA82C200_OUTP_CONTR) != 0xFA) {
			err("CPC-CARD: First controller seems to be broken");
			goto err;
		}

		if (readb(card->base + PCA82C200_OFFSET_1 + PCA82C200_OUTP_CONTR) != 0xFA) {
			err("CPC-CARD: Second controller seems to be broken");
			goto err;
		}

		card->channels = 2;
	} else if (contr == CTRL_SJA1000) {
		info("CPC-CARD: Found valid card with SJA1000 controller");

		writeb(0x87, card->base + CAN0_OFFSET_PELICAN + SJA1000_CLK_DIV);
		writeb(0x87, card->base + CAN1_OFFSET_PELICAN + SJA1000_CLK_DIV);

		udelay(100);

		card->channels = 0;

		if (readb(card->base + CAN0_OFFSET_PELICAN + SJA1000_CLK_DIV) != 0x87) {
			info("CPC-CARD: First channel seems to be broken");
			goto err;
		}

		if (readb(card->base + CAN1_OFFSET_PELICAN + SJA1000_CLK_DIV) != 0x87) {
			info("CPC-CARD: Second channel seems to be broken");
			goto err;
		}

		card->channels = 2;
	} else {
		err("CPC-CARD: Unsupported controller signatur %2.2X", contr);
		goto err;
	}

	minor = slot * 2;
	for (i = 0; i < card->channels; i++) {
		CPC_CHAN_T *chan;

		chan = card->chan[i] = (CPC_CHAN_T *) vmalloc(sizeof(CPC_CHAN_T));
		if (card->chan[i]) {
			memset(card->chan[i], 0, sizeof(CPC_CHAN_T));

			card->chan[i]->buf = vmalloc(sizeof(CPC_MSG_T) * CPC_MSG_BUF_CNT);

			if (!card->chan[i]->buf) {
				err("CPC-CARD[%d:%d]: No memory left for buffer", slot, i);
				goto nomem_err;
			}

			chan->minor = minor++;

#ifdef CONFIG_DEVFS_FS
			/* create device node */
			devfs_mk_cdev(MKDEV(CPC_CARD_MAJOR, chan->minor),
				      S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP
				      | S_IWGRP | S_IROTH | S_IWOTH,
				      "cpc_card%d", chan->minor);
#endif

			snprintf(devname, sizeof(devname), "cpc_card%d", chan->minor);
			cpc_device_register(MKDEV(CPC_CARD_MAJOR, chan->minor), NULL, devname);

			card->chan[i]->locked = 0;
			card->chan[i]->irqDisabled = 0;
			ResetBuffer(card->chan[i]);

			card->chan[i]->CPCWait_q = vmalloc(sizeof(wait_queue_head_t));

			if (!card->chan[i]->CPCWait_q) {
				err("CPC-CARD[%d:%d]: No memory left for waitqueue", slot, i);
				goto nomem_err;
			}

			init_waitqueue_head(card->chan[i]->CPCWait_q);
			card->chan[i]->read_byte = chan_read_byte;
			card->chan[i]->write_byte = chan_write_byte;

			/* remember controller type */
			if (contr == CTRL_SJA1000)
				card->chan[i]->controllerType = SJA1000;
			else if (contr == CTRL_PCA82C200)
				card->chan[i]->controllerType = PCA82C200;
			else
				err("CPC-CARD[%d:%d] Unsupported controller.", slot, i);
		} else {
			err("CPC-CARD[%d:%d]: No memory left for channel", slot, i);
			goto nomem_err;
		}

		info("CPC-CARD[%d:%d]: Allocated memory", slot, i);
	}

	if (contr == CTRL_PCA82C200) {
		info("CPC-CARD: Reseting PCA82C200 controller");

		card->chan[0]->canBase = card->base + PCA82C200_OFFSET_0;
		info("CPC-CARD[%d]  : Control base address %p", slot, card->base);
		info("CPC-CARD[%d:0]: PCA82C200 base address %p", slot, card->chan[0]->canBase);
		pca82c200_reset(card->chan[0]);

		if (card->channels == 2) {
			card->chan[1]->canBase = card->base + PCA82C200_OFFSET_1;
			info("CPC-CARD[%d:0]: PCA82C200 base address %p", slot, card->chan[1]->canBase);
			pca82c200_reset(card->chan[1]);
		}
	} else if (contr == CTRL_SJA1000) {
		info("CPC-CARD: Reseting SJA1000 controller");
		card->chan[0]->canBase = card->base + CAN0_OFFSET_PELICAN;
		info("CPC-CARD[%d]  : Control base address %p", slot, card->base);
		info("CPC-CARD[%d:0]: SJA1000 base address %p", slot, card->chan[0]->canBase);

		if (card->channels == 2) {
			card->chan[1]->canBase = card->base + CAN1_OFFSET_PELICAN;
			info("CPC-CARD[%d:1]: SJA1000 base address %p", slot, card->chan[1]->canBase);
		}

		writeb(RESET, card->base + LOGIC_BASE);	// reset the controllers
		do {
			udelay(1);
			if (++timeout > 100000)
				break;
		} while (!(readb(card->base + LOGIC_BASE + 0) & 0x01));
	}

	info("CPCCard card found and installed!");

	CPCCardCnt++;

	card->present = 1;

	return card;

err:
nomem_err:
	CPCCard_unregister(card);
	return NULL;
}

EXPORT_SYMBOL_GPL(CPCCard_register);

/**
 * Delete specified card
 * @param card
 */
static void CPCCard_delete(CPC_CARD_T * card)
{
	int j;

	BUG_ON(!card);

	card->present = 0;

	/* Free allocated channels */
	for (j = 0; j < card->channels; j++) {
		if (card->chan[j]) {
			dbg("CPC-CARD: buf %p CPCWait_q %p chan %p",
			    card->chan[j]->buf, card->chan[j]->CPCWait_q,
			    card->chan[j]);

#ifdef CONFIG_DEVFS_FS
			/* remove devfs entry */
			devfs_remove("cpc_card%d", card->chan[j]->minor);
#endif

			cpc_device_unregister(MKDEV(CPC_CARD_MAJOR, card->chan[j]->minor));

			if (card->chan[j]->buf) {
				dbg("CPC-CARD: freeing circular buffer");
				vfree(card->chan[j]->buf);
				card->chan[j]->buf = NULL;
			}

			if (card->chan[j]->CPCWait_q) {
				dbg("CPC-CARD: freeing circular buffer");
				vfree(card->chan[j]->CPCWait_q);
				card->chan[j]->CPCWait_q = NULL;
			}

			dbg("CPC-CARD: Freeing channel structure");
			vfree(card->chan[j]);
			card->chan[j] = NULL;
		}
	}

	card->channels = 0;

	info("CPCCard[-]: Card finally removed");

	return;
}

/**
 * cpc-card_cs tells us the card was removed
 * @param card
 */
void CPCCard_unregister(CPC_CARD_T * card)
{
	int j;
	int lockedChannels = 0;

	/* assert */
	if (!card) {
		info("CPC-CARD(%s:%d): card == NULL", __PRETTY_FUNCTION__,
		     __LINE__);
		return;
	}

	/* prevent spurious irq that damage the ISR */
	card->present = 0;

	/* determine locked (opened) channels */
	for (j = 0; j < card->channels; j++) {
		if (card->chan[j]) {
			if (card->chan[j]->locked) {
				/* wake up all pending processes */
				wake_up_interruptible(card->chan[j]->CPCWait_q);
				lockedChannels++;
			}
		}
	}

	iounmap(card->base);

	CPCCardCnt--;

	if (!lockedChannels) {
		/* free card if channels not locked */
		CPCCard_delete(card);
	} else {
		dbg("CPC-CARD: %d channel(s) locked", lockedChannels);
	}
}

EXPORT_SYMBOL_GPL(CPCCard_unregister);

static int __init CPCCard_init(void)
{
	unsigned int i;

	atomic_set(&useCount, 0);

	info("CPCCard[-] : SJA1000 Driver v%s", SJA1000_VERSION);
	info("CPCCard[-] : PCA82C200 Driver v%s", PCA82C200_VERSION);

	/* register character device node */
	if (register_chrdev(CPC_CARD_MAJOR, "cpc_card", &CPCCardFops)) {
		info("%s - unable to get major: %d", __FILE__, CPC_CARD_MAJOR);
		return -EIO;
	}

	for (i = 0; i < CPC_CARD_MAX_DEVICE; i++) {
		memset(&CPCTable[i], 0, sizeof(CPC_CARD_T));
	}

	procDir = proc_mkdir(CPC_CARD_PROC_DIR, NULL);
	if (!procDir) {
		err("Could not create proc entry");
	} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
		procDir->owner = THIS_MODULE;
#endif
		procEntry = create_proc_read_entry("info", 0444, procDir,
						   CPCCard_proc_read_info,
						   NULL);
		if (!procEntry) {
			err("Could not create proc entry %s", CPC_CARD_PROC_DIR "/info");
			remove_proc_entry(CPC_CARD_PROC_DIR, NULL);
			procDir = NULL;
		} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procEntry->owner = THIS_MODULE;
#endif
		}
	}

	return 0;
}

static void __exit CPCCard_exit(void)
{
	/**
	 * Sleep here (incl. the calling process e.g. rmmod/modprobe -r)
	 * until every process has detached
	 */
	wait_event(rmmodWq, !atomic_read(&useCount));

	if (procDir) {
		if (procEntry)
			remove_proc_entry("info", procDir);
		remove_proc_entry(CPC_CARD_PROC_DIR, NULL);
	}

	unregister_chrdev(CPC_CARD_MAJOR, "cpc_card");
}

module_init(CPCCard_init);
module_exit(CPCCard_exit);
