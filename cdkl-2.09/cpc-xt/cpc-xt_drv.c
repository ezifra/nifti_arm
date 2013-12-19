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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/poll.h>

#include <linux/device.h>

#include <linux/proc_fs.h>

#ifdef CONFIG_DEVFS_FS
#   include <linux/devfs_fs_kernel.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>

#include "../include/cpc.h"

#include "../include/cpc_int.h"
#include "../include/cpc_common.h"

#include "cpcxt.h"

#include "../include/version.h"

#include "../include/cpcpassive.h"

/* Version Information */
#define DRIVER_VERSION CPC_DRIVER_VERSION " (CDKL v" CDKL_VERSION ")"

MODULE_AUTHOR("Sebastian Haas <haas@ems-wuensche.com>");
MODULE_DESCRIPTION("CAN PC Interface driver for CPC-XT/104/104M CAN cards");
MODULE_SUPPORTED_DEVICE("EMS CPC XT/104/104M CAN card");
MODULE_LICENSE("GPL v2");

/* SA_SHIRQ replaced by IRQF_SHARED */
#ifndef IRQF_SHARED
#define IRQF_SHARED SA_SHIRQ
#endif

#undef dbg
#undef err
#undef info

/* Use our own dbg macro */
#define dbg(format, arg...) do { if (debug) printk( KERN_INFO format "\n" , ## arg); } while (0)
#define err(format, arg...) do { printk( KERN_INFO "ERROR " format "\n" , ## arg); } while (0)
#define info(format, arg...) do { printk( KERN_INFO format "\n" , ## arg); } while (0)

/* support debugging of ISR (interrupt service routine) */
//#define _CPC_XT_DEBUG_ISR

#include "../include/sja1000.h"
#include "../include/pca82c200.h"
#include "../include/cc770.h"

/* Module parameters */
static int debug = 0;
module_param(debug, int, S_IRUGO);

static CPC_XT_T *CPCXT_Table[CPC_XT_CARD_CNT];
static unsigned CPCXtCnt = 0;

#ifdef CONFIG_PROC_FS
#   define CPC_XT_PROC_DIR     CPC_PROC_DIR "cpc-xt"

static struct proc_dir_entry *procDir = NULL;
static struct proc_dir_entry *procEntry = NULL;
static struct proc_dir_entry *procDebugEntry = NULL;

#endif

/* use to prevent kernel panic if driver is unloaded
 * while a programm has still open the device
 */
DECLARE_WAIT_QUEUE_HEAD(rmmodWq);
atomic_t useCount;

static void card_write_byte(void *card, unsigned int reg, unsigned char val)
{
	writeb(val, (unsigned char *)(((CPC_XT_T *) card)->base + reg));
}

static unsigned char card_read_byte(void *card, unsigned int reg)
{
	return readb(((unsigned char *)((CPC_XT_T *) card)->base + reg));
}

static void chan_write_byte(void *chan, unsigned int reg, unsigned char val)
{
	writeb(val, (unsigned char *)(((CPC_CHAN_T *)chan)->canBase + reg));
}

static unsigned char chan_read_byte(void *chan, unsigned int reg)
{
	return readb(((unsigned char *)((CPC_CHAN_T *) chan)->canBase + reg));
}

/* kernel XT callbacks */
static int CPCXt_AddDevice(unsigned long base);
static void CPCXt_RemoveDevice(CPC_XT_T * card);

static int cpcxt_create_debug_output(char *buf)
{
	int i = 0, j, k;

	i += sprintf(&buf[i], "Driver infos\n");
	i += sprintf(&buf[i], "\tUse count %d\n", atomic_read(&useCount));

	for (j = 0; j < CPC_XT_CARD_CNT; j++) {
		if (CPCTable[j]) {
			for(k = 0; k < CPCTable[j]->channel_count; k++) {
				CPC_CHAN_T *chan = CPCTable[j]->chan[k];

				if(chan) {
					i += sprintf(&buf[i], "Channel %d\n", k);
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
	}

	return i;
}

static int cpcxt_proc_read_debug(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = cpcxt_create_debug_output(page);

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

static int cpcxt_create_info_output(char *buf)
{
	int i = 0, j, k;

	for (j = 0; j < CPC_XT_CARD_CNT; j++) {
		if (CPCTable[j]) {
			for(k = 0; k < CPCTable[j]->channel_count; k++) {
				i += sprintf(&buf[i], "%d %d %d %lX\n",
					     (j * CPC_XT_CHANNEL_CNT) + k, k, CPCTable[j]->irq, CPCTable[j]->jumpered_address);
			}
		}
	}
	return i;
}

static int cpcxt_proc_read_info(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len = cpcxt_create_info_output(page);

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

/*
 * Interrupt used by all channels of one card
 */
static irqreturn_t CPCXt_interrupt(int irq, void *dev_id
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
				   , struct pt_regs *regs
#endif
)
{
	int handled = 0;
	int try_again = 1;
	int i;

	struct timeval now;
	CPC_XT_T *card = (CPC_XT_T *)dev_id;
	CPC_CHAN_T *chan = NULL;

	while (try_again) {
		try_again = 0;

		for(i = 0; i < card->channel_count; i++) {
			if(card->chan[i]) {
				chan = card->chan[i];

				// is channel used
				if(!chan->locked || chan->irqDisabled)
					continue;

				/* for time stamp */
				do_gettimeofday(&now);

				if (chan->controllerType == SJA1000) {
					if (sja1000_interrupt(chan, now)) {
						try_again = 1;
						handled = 1;
					}
				} else if (chan->controllerType == PCA82C200) {
					if (pca82c200_interrupt(chan, now)) {
						try_again = 1;
						handled = 1;
					}
				} else if (chan->controllerType == AN82527) {
					if (cc770_interrupt(chan, now)) {
						try_again = 1;
						handled = 1;
					}
				}
			}
		}
	}

	return IRQ_RETVAL(handled);
}

static ssize_t CPCXtRead(struct file *file, char *buf, size_t count,
			 loff_t * offset);
static ssize_t CPCXtWrite(struct file *file, const char *buf, size_t count,
			  loff_t * offset);
static unsigned int CPCXtPoll(struct file *file,
			      struct poll_table_struct *wait);
static int CPCXtOpen(struct inode *inode, struct file *file);
static int CPCXtRelease(struct inode *inode, struct file *file);

/* fops table */
static struct file_operations CPCXtFops = {
      read:    CPCXtRead,
      write:   CPCXtWrite,
      poll:    CPCXtPoll,
      open:    CPCXtOpen,
      release: CPCXtRelease
};

/*
 * Read a message
 */
static ssize_t CPCXtRead(struct file *file, char *buf, size_t count,
			 loff_t * offset)
{
	CPC_CHAN_T *chan = file->private_data;
	CPC_XT_T *card = chan->private;
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

/*
 * Process command
 */
static ssize_t CPCXtWrite(struct file *file, const char *buf, size_t count,
			  loff_t * offset)
{
	CPC_CHAN_T *chan = file->private_data;

	CPC_MSG_T cpcbuf;
	CPC_XT_T *card = chan->private;

	int retval = 0;
	unsigned long flags;

	/* Write access failed */
	if (!card || !chan)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	if (!chan->locked)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	if (count > sizeof(CPC_MSG_T))
		return CPC_ERR_UNKNOWN;

	dbg("CPCXt[%d]: Check read access %s %d", card->idx, buf, count);
	/* check if can read from the given address */
	if (!access_ok(VERIFY_READ, buf, count)) {
		err("CPCXt[%d]: Access denied", card->idx);
		return CPC_ERR_UNKNOWN;
	}

	/* copy message from user to kernel space */
	if (copy_from_user(&cpcbuf, buf, sizeof(CPC_MSG_T)) != 0)
		return CPC_ERR_IO_TRANSFER;

	if (cpcbuf.type == CPC_CMD_T_OPEN_CHAN) {
		if(card->irq == XT_CARD_IRQ_NOT_ASSIGNED) {
			XT_CLEANUP_IRQ(card);

			card->irq = (unsigned int) cpcbuf.msg.generic[0];

			/* setup interrupt */
			if (request_irq(card->irq, CPCXt_interrupt, IRQF_SHARED, "cpc_xt", card)) {
				err("CPCXt[%d] : failed request interrupt IRQ%d!",
						card->idx, card->irq);
				card->irq = XT_CARD_IRQ_NOT_ASSIGNED;
				retval = CPC_ERR_NO_INTERRUPT;
			} else {
				dbg("CPCXt[%d] : Assigned IRQ%d", card->idx, card->irq);
				retval = 0;
			}
		}
	} else {
		spin_lock_irqsave(&card->slock, flags);
		retval = cpc_process_command(chan, &cpcbuf);
		spin_unlock_irqrestore(&card->slock, flags);
	}

	return retval;
}

static unsigned int CPCXtPoll(struct file *file, struct poll_table_struct *wait)
{
	CPC_CHAN_T *chan = file->private_data;
	int retval = 0;

	poll_wait(file, chan->CPCWait_q, wait);

	if ((chan->oidx != chan->iidx) || !(chan->WnR))
		retval |= (POLLIN | POLLRDNORM);

	if (chan->controllerType == PCA82C200) {
		if (chan->read_byte(chan, PCA82C200_STATUS) & PCA82C200_SR_TBS)
			retval |= (POLLOUT | POLLWRNORM);
	} else if (chan->controllerType == SJA1000) {
		if (chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)
			retval |= (POLLOUT | POLLWRNORM);
	} else if (chan->controllerType == AN82527) {
			retval |= (POLLOUT | POLLWRNORM);
	} else {
		err("Wrong controller %d", chan->controllerType);
	}

	return retval;
}


/*
 * Called on open the device file
 */
static int CPCXtOpen(struct inode *inode, struct file *file)
{
	u32 card_idx, chan_idx = CPC_XT_CHANNEL_CNT;

	for(card_idx = 0; card_idx < CPCXtCnt; card_idx++) {
		struct CPC_XT *card = CPCTable[card_idx];

		for(chan_idx = 0; chan_idx < card->channel_count; chan_idx++) {
			if(card->chan[chan_idx]->minor == MINOR(inode->i_rdev))
				break;
		}

		if(chan_idx < card->channel_count)
			break;
	}

	dbg("CPCXt[%d:%d]: open called", card_idx, chan_idx);

	if (card_idx >= CPCXtCnt) {
		err("CPCXt[-]: only %d device found in this PC!", CPCXtCnt);

		return CPC_ERR_NO_INTERFACE_PRESENT;
	} else if(CPCTable[card_idx]->chan[chan_idx]){
		CPC_CHAN_T *chan = CPCTable[card_idx]->chan[chan_idx];
		CPC_XT_T   *card = CPCTable[card_idx];

		if (chan->locked) {
			err("CPCXt[%d]: Device is locked!", card_idx);
			return CPC_ERR_CHANNEL_ALREADY_OPEN;
		} else {
			ResetBuffer(chan);

			chan->locked = 1;

			// enable IRQ source
			card->write_byte(card, XT_CARD_REG_IRQ_CTRL, 0x3 << (chan_idx*2));
			dbg("CPCXt[%d]: IRQ Status %2.2X", card_idx, card->read_byte(card, XT_CARD_REG_IRQ_STATUS));
			chan->irqDisabled = 0;

			chan->cpcCtrlCANMessage = 0;
			chan->cpcCtrlCANState = 0;
			chan->cpcCtrlBUSState = 0;

			chan->ovrLockedBuffer = 0;
			chan->ovr.event = 0;
			chan->ovr.count = 0;

			file->private_data = chan;

			atomic_inc(&useCount);
		}
	} else {
		err("CPCXt[-]: Channel not active!");
		return CPC_ERR_CHANNEL_NOT_ACTIVE;
	}

	return 0;
}

/*
 * Close channel and release all resources
 */
static int CPCXtRelease(struct inode *inode, struct file *file)
{
	CPC_CHAN_T *chan = file->private_data;
	struct CPC_XT *card = chan->private;
	int i;
	int chan_idx = CPC_XT_CHANNEL_CNT;

	dbg("CPCXt[%d]: Closing", card->idx);

	chan->cpcCtrlCANMessage = 0;
	chan->cpcCtrlCANState = 0;

	if (chan->controllerType == PCA82C200) {
		pca82c200_reset(chan);
	} else if (chan->controllerType == SJA1000) {
		sja1000_reset(chan);
	} else if (chan->controllerType == AN82527) {
		cc770_reset(chan);
	}

	file->private_data = NULL;
	chan->locked = 0;

	for(i = 0; i < card->channel_count; i++) {
		if(card->chan[i] == chan)
			chan_idx = i;
	}

	BUG_ON(chan_idx >= CPC_XT_CHANNEL_CNT);

	card->write_byte(card, XT_CARD_REG_IRQ_CTRL, 0x2 << chan_idx);

	chan->irqDisabled = 1;

	for(i = 0; i < card->channel_count; i++) {
		if(card->chan[i]) {
			if(card->chan[i]->locked)
				break;
		}
	}

	/* last channel closed -> free the IRQ */
	if(i == card->channel_count ) {
		dbg("CPCXt[%d:%d]: Last channel closed.", card->idx, chan_idx);
		XT_CLEANUP_IRQ(card);
	}

	atomic_dec(&useCount);

	/* last process detached */
	if (atomic_read(&useCount) == 0) {
		wake_up(&rmmodWq);
	}

	dbg("CPCXt[-]: closed");

	return 0;
}

/*
 * Probe I/O memory to detect channels
 */
#define NOMEM_ASSERT(x) do { if(!x) { err("CPCXt[-]: No memory available (%s:%d)", __PRETTY_FUNCTION__, __LINE__); goto nomem; } } while(0)
static int CPCXt_AddDevice(unsigned long base)
{
	/* remember count of found boards */
	static unsigned int card_idx = 0;

	CPC_XT_T     *card = NULL;
	CPC_CHAN_T   *chan = NULL;
	void         *mem  = NULL;
	unsigned int  k, ck, minor;
	unsigned long signature;

	if(CPCXtCnt < 0 || CPCXtCnt >= CPC_XT_CARD_CNT) {
		err("CPCXt[-]: Max. supported devices exceeded (max. %d)", CPC_XT_CARD_CNT);
		return -ENODEV;
	}

	mem = ioremap(base, XT_STEP_WIDTH);
	if(!mem) {
		err("CPCXt[-]: Unable to map memory space");
		return -ENODEV;
	}

	signature = readl(mem);
	dbg("CPCXt[-]: %p signature %8.8lX", mem, signature);

	if(signature != 0xcb03aa55 && signature != 0xcb01aa55) {
		dbg("CPCXt[-]: Neither CPC-XT, CPC-104 nor CPC-104M");
		iounmap(mem);
		return -ENODEV;
	}
	dbg("CPCXt[-]: Found EMS board");

	/* allocate basic structure for this card */
	CPCTable[card_idx] = (CPC_XT_T *) vmalloc(sizeof(CPC_XT_T));
	NOMEM_ASSERT(CPCTable[card_idx]);
	memset((unsigned char *) CPCTable[card_idx], 0, sizeof(CPC_XT_T));
	card = CPCTable[card_idx];

	spin_lock_init(&card->slock);
	card->idx = card_idx;

	card->base       = mem;
	card->read_byte  = card_read_byte;
	card->write_byte = card_write_byte;
	card->jumpered_address = base;

	/* reset the CAN controller */
	card->write_byte(card, XT_CARD_REG_CONTROL, XT_CARD_CMD_HW_RESET);

	/* Wait for the reset to finish */
	do { } while(card->read_byte(card, XT_CARD_REG_STATUS) == 0x01);

	/* MAP CAN controller to XT address space */
	card->write_byte(card, XT_CARD_REG_CONTROL, XT_CARD_CMD_MAP_CONTR);
	card->write_byte(card, XT_CARD_REG_CONTROL, XT_CARD_CMD_MAP_CONTR);

	card->canControllerType = card->read_byte(card, XT_CARD_REG_CONTR);

	card->irq = XT_CARD_IRQ_NOT_ASSIGNED;

	/* check if the CAN controller is mapped by XT */
	if (!(card->read_byte(card, XT_CARD_REG_STATUS) & 0x02)) {
		err("CPCXt[%d] : CAN controller not mapped to XT address space", card_idx);
		goto err;
	}

	card->channel_count = 1;
	if(card->read_byte(card, XT_CARD_REG_VERSION) == 0x11) {
		info("CPCXt[%d] : Found CPC-104M card at %p", card_idx, card->base);

		if(card->read_byte(card, XT_CARD_STEINHOFF_CHECK) == 0x04
			&& card->read_byte(card, XT_CARD_STEINHOFF_CHECK*2) == 0x16)
		{
			info("CPCXt[%d] : Card is equipped with Steinhoff Firmware", card_idx);
		}

		info("CPCXt[%d] : Interrupt Control %2.2X", card_idx, card->read_byte(card, 7));
		info("CPCXt[%d] : Interrupt Status %2.2X", card_idx, card->read_byte(card, 8));

		card->channel_count = CPC_XT_CHANNEL_CNT;
	} else {
		info("CPCXt[%d] : Found XT card at %p", card_idx, card->base);
	}

	minor = 0;

	/* ck counts channels found inactive */
	for(k = 0, ck = 0; k < card->channel_count; k++) {
		card->chan[k-ck] = (CPC_CHAN_T *) vmalloc(sizeof(CPC_CHAN_T));
		NOMEM_ASSERT(card->chan[k-ck]);

		chan = card->chan[k-ck];

		memset(chan, 0, sizeof(CPC_CHAN_T));

		/* set CAN controller base addresses */
		chan->canBase = card->base + (CAN_OFFSET * (k+1));
		chan->read_byte = chan_read_byte;
		chan->write_byte = chan_write_byte;

		chan->CPCWait_q = (wait_queue_head_t *) vmalloc(sizeof(wait_queue_head_t));
		NOMEM_ASSERT(chan->CPCWait_q);

		init_waitqueue_head(chan->CPCWait_q);

		/* allocate message buffer memory */
		chan->buf = (CPC_MSG_T *) vmalloc(sizeof(CPC_MSG_T) * CPC_MSG_BUF_CNT);
		NOMEM_ASSERT(chan->buf);

		ResetBuffer(chan);

#ifdef CONFIG_DEVFS_FS
		/* create device node */
		devfs_mk_cdev(MKDEV(CPC_XT_MAJOR, chan->minor),
				S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP |
				S_IROTH | S_IWOTH, "cpc_xt%d", minor);
#endif

		snprintf(&card->deviceNames[k-ck][0], 32, "cpc_xt%d", minor);

		memset(&card->miscdev[k-ck], 0, sizeof(card->miscdev[k-ck]));

		card->miscdev[k-ck].minor = MISC_DYNAMIC_MINOR;
		card->miscdev[k-ck].fops = &CPCXtFops;
		card->miscdev[k-ck].name = &card->deviceNames[k-ck][0];

		misc_register(&card->miscdev[k-ck]);

		chan->minor = card->miscdev[k-ck].minor;

		chan->private = card;

		chan->locked = 0;	/* So we can open the device */

		dbg("CPCXt[%d] : Message buffer allocated", card_idx);

		/* set controller into reset mode and remember correct controller
		 * type of this channel.
		 */
		switch(card->canControllerType) {
				case XT_CONTR_SJA1000:
					sja1000_reset(chan);
					chan->controllerType = SJA1000;
					break;
				case XT_CONTR_PCA82C200:
					pca82c200_reset(chan);
					chan->controllerType = PCA82C200;
					break;
				case XT_CONTR_CC770:
					cc770_reset(chan);
					chan->controllerType = AN82527;
					break;
				default:
					err("No valid CAN controller type");
		}

		if(chan->controllerType == AN82527) {
			/* Enable configuration register access and set init state */
			chan->write_byte(chan, CC770_CONTROL, 0x1);

			chan->write_byte(chan, CC770_P2_CONF, 0); /* Port 2 is input */

			/* Set CPU-interface register to SCLK = XTAL/2 and clockout enabled */
			chan->write_byte(chan, CC770_CPU_IFACE, 0x41);

			if(chan->read_byte(chan, CC770_CPU_IFACE) == 0x41) {
				info("CC770 found. ");
				minor++;
				continue;
			}

			info("Control %02X", chan->read_byte(chan, CC770_CONTROL));
			info("CPU     %02X", chan->read_byte(chan, CC770_CPU_IFACE));
		} else {
			chan->write_byte(chan, SJA1000_CLK_DIV, SJA1000_CLK_PELICAN);
			dbg("CPCXt[%d]: CLK_DIV %2.2X", card_idx, chan->read_byte(chan, SJA1000_CLK_DIV));

			if(chan->read_byte(chan, SJA1000_CLK_DIV) == SJA1000_CLK_PELICAN) {
				info("CPCXt[%d:%d] : CAN controller at %p", card_idx, k-ck, chan->canBase);
				minor++;
				continue;
			}
		}
		/* channel not accessible, delete */
		vfree(chan->buf);
		misc_deregister(&card->miscdev[k-ck]);
		card->chan[k-ck] = NULL;
		vfree(chan);
		ck++; /* count inaccessible channels to decrement */
		minor++;
	}
	card->channel_count -= ck; /* sustract the inactive channels from channel count */
	info("CPCXt[%d] : Found %d active Channel(s)", card_idx, card->channel_count);
	CPCXtCnt++;
	card_idx++;

	return 0;

err:
	CPCXt_RemoveDevice(card);
	return -ENODEV;

nomem:
	CPCXt_RemoveDevice(card);
	return -ENOMEM;
}

/*
 * Remove board
 */
static void CPCXt_RemoveDevice(CPC_XT_T * card)
{
	CPC_CHAN_T *chan = NULL;
	unsigned int k;

	if (!card)
		return;

	XT_CLEANUP_IRQ(card);

	/* UNMAP CAN controller to XT address space */
	card->write_byte(card, XT_CARD_REG_CONTROL,
			 XT_CARD_CMD_UMAP_CONTR);

	card->read_byte = NULL;
	card->write_byte = NULL;

	for(k = 0; k < CPC_XT_CHANNEL_CNT; k++) {
		chan = card->chan[k];

		if(chan) {
#ifdef CONFIG_DEVFS_FS
			/* remove devfs entry */
			devfs_remove("cpc_xt%d", chan->minor);
#endif
			misc_deregister(&card->miscdev[k]);

			chan->read_byte = NULL;
			chan->write_byte = NULL;

			/* free waitqueue */
			if (chan->CPCWait_q)
				vfree(chan->CPCWait_q);

			/* free message buffer */
			if (chan->buf)
				vfree(chan->buf);

			/* free the channel */
			vfree(chan);
		}
	}

	CPCXtCnt--;

	/* mark table entry as unused */
	CPCTable[card->idx] = NULL;

	/* free card */
	vfree(card);
}

/*
 * Call probe function for all useable address ranges
 */
static int CPCXt_probe_address_space(void)
{
	unsigned long search = XT_START_ADDR;
	unsigned int count = 0;

	for (search = XT_START_ADDR; search <= XT_END_ADDR; search += XT_STEP_WIDTH) {
		if (CPCXt_AddDevice(search) == 0) {
			count++;
		}
	}

	return count;
}

static int __init CPCXtInit(void)
{
	int i;
	int countOfFoundInterfaces = 0;

	info("CPCXt[-] : Loading Driver v%s, build on %s at %s",
	     DRIVER_VERSION, __DATE__, __TIME__);
	info("CPCXt[-] : SJA1000 Driver v%s", SJA1000_VERSION);
	info("CPCXt[-] : PCA82C200 Driver v%s", PCA82C200_VERSION);

	//if(register_chrdev(126, "cpc_xt", &CPCXtFops)) {
	//	err("Unable to get major");
	//}

	for (i = 0; i < CPC_XT_CARD_CNT; i++)
		CPCTable[i] = NULL;

	atomic_set(&useCount, 0);

	countOfFoundInterfaces = CPCXt_probe_address_space();

	if (countOfFoundInterfaces == 0) {
		return -ENODEV;
	}

	procDir = proc_mkdir(CPC_XT_PROC_DIR, NULL);
	if (!procDir) {
		err("CPCXti[-]: Could not create proc entry");
	} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
		procDir->owner = THIS_MODULE;
#endif
		procEntry = create_proc_read_entry("info", 0444, procDir,
				cpcxt_proc_read_info, NULL);

		if (!procEntry) {
			err("CPCXti[-]: Could not create proc entry %s", CPC_XT_PROC_DIR "/info");
			remove_proc_entry(CPC_XT_PROC_DIR, NULL);
			procDir = NULL;
		} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procEntry->owner = THIS_MODULE;
#endif
		}

		procDebugEntry =
		    create_proc_read_entry("debug", 0444, procDir,
					   cpcxt_proc_read_debug, NULL);
		if (!procDebugEntry) {
			err("CPCXti[-]: Could not create proc entry %s",
			    CPC_XT_PROC_DIR "/debug");
		} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procDebugEntry->owner = THIS_MODULE;
#endif
		}
	}

	return 0;
}

static void __exit CPCXtExit(void)
{
	int i;

	wait_event(rmmodWq, !atomic_read(&useCount));

	if (procDir) {
		if (procEntry)
			remove_proc_entry("info", procDir);

		if (procDebugEntry)
			remove_proc_entry("debug", procDir);

		remove_proc_entry(CPC_XT_PROC_DIR, NULL);
	}

	for (i = 0; i < CPC_XT_CARD_CNT; i++)
		CPCXt_RemoveDevice(CPCTable[i]);

	info("CPCXt[-]: Driver unloaded!");
}

module_init(CPCXtInit);
module_exit(CPCXtExit);
