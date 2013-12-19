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
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/ioport.h>

#include <linux/proc_fs.h>

#ifdef CONFIG_DEVFS_FS
#   include <linux/devfs_fs_kernel.h>
#endif

#include <linux/parport.h>

#include <asm/io.h>
#include <asm/irq.h>

#include "../include/cpc.h"

#include "../include/cpc_int.h"
#include "../include/cpc_common.h"

#include "cpceco.h"
#include "../include/version.h"

#include "../include/cpcpassive.h"
#include "../include/sja1000.h"

#define CPC_ECO_MAX_DEVICE       2
#define CPC_ECO_MAJOR            122

#define DRIVER_NAME   "cpc_eco"
#define DRIVER_VERSION CPC_DRIVER_VERSION " (CDKL v" CDKL_VERSION ")"

MODULE_AUTHOR("Sebastian Haas <haas@ems-wuensche.com>");
MODULE_DESCRIPTION("CPC-ECO driver for Linux Kernel 2.6");
MODULE_VERSION(CPC_DRIVER_VERSION);
MODULE_LICENSE("GPL");

//#define _MANUAL_IRQ_CTRL

#undef dbg
#undef err
#undef info

/* Use our own dbg macro */
#define dbg(format, arg...) do { if (debug) printk( KERN_INFO "CPC-ECO: " format "\n" , ## arg); } while (0)
#define err(format, arg...) do { printk( KERN_INFO "CPC-ECO(ERROR): " format "\n" , ## arg); } while (0)
#define info(format, arg...) do { printk( KERN_INFO "CPC-ECO: " format "\n" , ## arg); } while (0)

static int debug = 0;
module_param(debug, int, S_IRUGO);

#ifdef CONFIG_PROC_FS
#   define CPC_ECO_PROC_DIR     CPC_PROC_DIR "cpc-eco"

static struct proc_dir_entry *procDir = NULL;
static struct proc_dir_entry *procEntry = NULL;

#endif

static void CPCEco_attach(struct parport *port);
static void CPCEco_delete(CPC_ECO_T ** peco);
static void CPCEco_detach(struct parport *port);

static unsigned char chan_read_byte(void *chan, unsigned int reg);
static void chan_write_byte(void *chan, unsigned int reg, unsigned char val);

static struct parport_driver CPCEco_driver = {
	.name = "cpc-eco",
	.attach = CPCEco_attach,
	.detach = CPCEco_detach,
	.list = {NULL},
};

/* use to prevent kernel panic if driver is unloaded
 * while a programm has still open the device
 */
DECLARE_WAIT_QUEUE_HEAD(rmmodWq);
atomic_t useCount;

static ssize_t CPCEcoRead(struct file *file, char *buf, size_t count,
			  loff_t * offset);
static ssize_t CPCEcoWrite(struct file *file, const char *buf,
			   size_t count, loff_t * offset);
static unsigned int CPCEcoPoll(struct file *file,
			       struct poll_table_struct *wait);
static int CPCEcoOpen(struct inode *inode, struct file *file);
static int CPCEcoRelease(struct inode *inode, struct file *file);

static struct file_operations CPCEcoFops = {
	read:    CPCEcoRead,
	write:   CPCEcoWrite,
	poll:    CPCEcoPoll,
	open:    CPCEcoOpen,
	release: CPCEcoRelease
};

static CPC_ECO_T *CPCECO_Table[CPC_ECO_MAX_DEVICE] = { 0 };
static unsigned CPCEcoCnt = 0;

/* functions and structures for hardware access *******************************************/
static unsigned char n_decode[32] = {
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7
};

static int CPCEco_create_info_output(char *buf)
{
	int i = 0, j;

	for (j = 0; j < CPC_ECO_MAX_DEVICE; j++) {
		if (CPCTable[j]) {
			CPC_ECO_T *card = CPCTable[j];
			CPC_CHAN_T *chan = card->chan;

			/* MINOR PORTNO */
			i += sprintf(&buf[i], "%d %d\n", chan->minor, card->portnum);
		}
	}

	return i;
}

static ssize_t CPCEco_proc_read_info(char *page, char **start, off_t off,
                                     int count, int *eof, void *data)
{
	int len = CPCEco_create_info_output(page);

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

static unsigned char chan_read_byte(void *chan, unsigned int reg)
{
	unsigned char n0, n1;

	unsigned int PA = (unsigned int)((CPC_CHAN_T *) chan)->canBase;
	unsigned int PB = PA + 1;
	unsigned int PC = PA + 2;

	outb(reg, PA);		// Address (0..31)
	outb(0x0b ^ 0x1b, PC);	// ALE H->L
	outb(0x0b ^ 0x19, PC);	// nRD H->L
	n1 = n_decode[(inb(PB) >> 3) & 0x1f];	// Get high Nibble
	outb(0x0b ^ 0x18, PC);	// select low Nibble
	n0 = n_decode[(inb(PB) >> 3) & 0x1f];	// Get low Nibble
	outb(0x0b ^ 0x1b, PC);	// nRD L->H
	outb(0x0b ^ 0x1f, PC);	// ALE L->H
	outb(0xff, PA);		// set high

	return (n1 << 4) | n0;
}

static void chan_write_byte(void *chan, unsigned int reg, unsigned char val)
{
	unsigned int PA = (unsigned int)((CPC_CHAN_T *) chan)->canBase;
	unsigned int PC = PA + 2;

	outb(reg, PA);		// Address
	outb(0x0b ^ 0x1b, PC);	// nRD H, ALE H->L, nWR H, Nib H
	outb(0x0b ^ 0x13, PC);	// nWR H->L
	outb(val, PA);		// Daten
	outb(0x0b ^ 0x1b, PC);	// nWR L->H
	outb(0x0b ^ 0x1f, PC);	// ALE L->H
	outb(0xff, PA);		// Set high

}

static inline void CPCEco_enable_irq(CPC_ECO_T * card)
{
	card->chan->write_byte(card->chan, SJA1000_INT_ENABLE, SJA1000_IER_RIE);
}

static inline void CPCEco_disable_irq(CPC_ECO_T * card)
{
	card->chan->write_byte(card->chan, SJA1000_INT_ENABLE, 0);
}

int can_reset(CPC_CHAN_T * chan)
{
	outb(0x0b ^ 0x15, (unsigned int)chan->canBase + 2);	/* Set nWR and nRD Low */
	mdelay(500);
	outb(0x0b ^ 0x1f, (unsigned int)chan->canBase + 2);

	return 0;
}

static void CPCEco_interrupt(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		int irq,
#endif
		void *handle
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
                             , struct pt_regs *regs
#endif
)
{
	volatile CPC_ECO_T *card = (CPC_ECO_T *)handle;
	int handled = 0;
	int try_again = 1;
	int count = 0;
	struct timeval now;

	/* On high CAN bus load this ISR could get several
	 * thousands of message within 1 IRQ that causes
	 * the system hangs due the time this ISR is taking.
	 *
	 * This probably cause data lost, but we have no
	 * other choice. We had to give time to the application!
	 */
	while (try_again && (count++) < 10) {
		try_again = 0;
		do_gettimeofday(&now);
		if (sja1000_interrupt(card->chan, now)) {
			try_again = 1;
			handled = 1;
		}
		if (IsBufferFull(card->chan))
			break;
	}

	/* !!!HACK!!!
	 * Make sure the receive buffer is empty to prevent not recognized IRQs.
	 * This could cause lost of CAN messages, which doesn't matter anyway
	 * cause the parallel port is to slow.
	 */
	card->chan->write_byte(card->chan, SJA1000_COMMAND,
			SJA1000_CMR_RRB | SJA1000_CMR_CDO);
}

static ssize_t CPCEcoRead(struct file *file, char *buf, size_t count,
                          loff_t * offset)
{
	CPC_ECO_T *card = file->private_data;
	CPC_CHAN_T *chan = card->chan;
	CPC_MSG_T *cpc = NULL;
	unsigned long flags;

	if (count < sizeof(CPC_MSG_T))
		return CPC_ERR_UNKNOWN;

	if(!access_ok(VERIFY_WRITE, buf, count))
		return CPC_ERR_UNKNOWN;

	spin_lock_irqsave(&card->slock, flags);

#ifdef _MANUAL_IRQ_CTRL
	CPCEco_disable_irq(card);
#endif
	cpc = cpc_get_next_message(chan);
#ifdef _MANUAL_IRQ_CTRL
	CPCEco_enable_irq(card);
#endif

	spin_unlock_irqrestore(&card->slock, flags);

	if (cpc) {
		if (copy_to_user(buf, cpc, sizeof(CPC_MSG_T)) != 0)
			return CPC_ERR_IO_TRANSFER;

		return sizeof(CPC_MSG_T);
	}

	return 0;
}

static int CPCEcoWrite(struct file *file, const char *buf, size_t count,
                       loff_t * offset)
{
	CPC_ECO_T *card = file->private_data;
	CPC_CHAN_T *chan = card->chan;
	int retval = 0;
	unsigned long flags = 0;
	CPC_MSG_T cpcbuf;

	if (count > sizeof(CPC_MSG_T))
		return CPC_ERR_UNKNOWN;

	/* check if can read from the given address */
	if(!access_ok(VERIFY_READ, buf, count))
		return CPC_ERR_UNKNOWN;

	if (copy_from_user(&cpcbuf, buf, sizeof(CPC_MSG_T)) != 0)
		return CPC_ERR_IO_TRANSFER;

	spin_lock_irqsave(&card->slock, flags);
#ifdef _MANUAL_IRQ_CTRL
	CPCEco_disable_irq(card);
#endif

	retval = cpc_process_command(chan, &cpcbuf);

#ifdef _MANUAL_IRQ_CTRL
	CPCEco_enable_irq(card);
#endif
	spin_unlock_irqrestore(&card->slock, flags);

	return retval;
}

static unsigned int CPCEcoPoll(struct file *file,
                               struct poll_table_struct *wait)
{
	CPC_CHAN_T *chan = ((CPC_ECO_T *) file->private_data)->chan;
	int retval = 0;

	poll_wait(file, chan->CPCWait_q, wait);

	if (IsBufferNotEmpty(chan))
		retval |= (POLLIN | POLLRDNORM);

	if (chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)
		retval |= (POLLOUT | POLLWRNORM);

	return retval;
}

static int CPCEcoOpen(struct inode *inode, struct file *file)
{
	u32 card_idx;

	for(card_idx = 0; card_idx < CPCEcoCnt; card_idx++) {
		if(CPCTable[card_idx]->chan->minor == MINOR(inode->i_rdev))
			break;
	}

	if (card_idx >= CPC_ECO_MAX_DEVICE) {
		dbg("CPCEco[-]: only %d device(s) supported by driver!",
		    CPC_ECO_MAX_DEVICE);
		return CPC_ERR_NO_MATCHING_INTERFACE;
	} else {
		CPC_ECO_T *card = CPCTable[card_idx];
		CPC_CHAN_T *chan = card->chan;

		file->private_data = card;

		if (chan->locked) {
			dbg("CPCEco[%d]: Device is locked!", card_idx);
			return CPC_ERR_CHANNEL_ALREADY_OPEN;
		} else {
			ResetBuffer(chan);

			parport_enable_irq(card->pport);

			chan->irqDisabled = 0;
			chan->locked = 1;

			chan->cpcCtrlCANMessage = 0;
			chan->cpcCtrlCANState = 0;
			chan->cpcCtrlBUSState = 0;

			chan->ovrLockedBuffer = 0;
			chan->ovr.event = 0;
			chan->ovr.count = 0;

			atomic_inc(&useCount);

			dbg("CPCEco[%d]: open", card_idx);

			return 0;
		}
	}
}

static int CPCEcoRelease(struct inode *inode, struct file *file)
{
	CPC_ECO_T *card = file->private_data;
	CPC_CHAN_T *chan = card->chan;

	CPC_SJA1000_PARAMS_T cinit =
	    { 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0, 0 };

	sja1000_init_contr(chan, &cinit);

	file->private_data = NULL;

	chan->locked = 0;
	parport_disable_irq(card->pport);

	chan->irqDisabled = 1;

	atomic_dec(&useCount);

	/* last process detached */
	if (atomic_read(&useCount) == 0) {
		wake_up(&rmmodWq);
	}

	dbg("CPCEco[%d]: closed", MINOR(file->f_dentry->d_inode->i_rdev));

	return 0;
}

static int CPCEco_preempt(void *handle)
{
	dbg("PREEMPT");
	return 1;
}

static void CPCEco_wakeup(void *handle)
{
	dbg("WAKEUP");
	return;
}

/**
 * New parallel port found, probe for CPC-ECO
 * @param port
 */
static void CPCEco_attach(struct parport *port)
{
	struct pardevice *pdev = NULL;
	CPC_ECO_T *card = NULL;
	CPC_CHAN_T *chan = NULL;

	if (CPCEcoCnt >= CPC_ECO_MAX_DEVICE) {
		err("CPCEco: No more devices supported");
		return;
	}

	CPCTable[CPCEcoCnt] = NULL;

	pdev = parport_register_device(port, "CPC-ECO", CPCEco_preempt,
			CPCEco_wakeup, CPCEco_interrupt, 0, NULL);

	if (!pdev) {
		err("Could not register device");
		goto error;
	}

	if (parport_claim(pdev) != 0) {
		err("Could not claim device");
		goto error;
	}

	CPCTable[CPCEcoCnt] = kzalloc(sizeof(CPC_ECO_T), GFP_KERNEL);
	if (!CPCTable[CPCEcoCnt]) {
		err("Out of memory");
		goto error;
	}

	card = CPCECO_Table[CPCEcoCnt];	/* next slot */
	memset(card, 0, sizeof(CPC_ECO_T));

	/* init new entry */
	card->pdev = pdev;
	card->pdev->private = card;

	card->PA = port->base;
	card->irq = port->irq;
	card->portnum = port->portnum;
	card->pport = port;

	card->chan = (CPC_CHAN_T *) vmalloc(sizeof(CPC_CHAN_T));
	if (!card->chan) {
		err("CPCEco[-]: Unable to get enough memory for device structures!");
		goto error;
	}

	chan = card->chan;
	memset(chan, 0, sizeof(CPC_CHAN_T));
	chan->canBase = (void *)card->PA;
	chan->read_byte = chan_read_byte;
	chan->write_byte = chan_write_byte;
	card->chan->minor = CPCEcoCnt;

#ifdef CONFIG_DEVFS_FS
	/* create device node */
	devfs_mk_cdev(MKDEV(CPC_ECO_MAJOR, chan->minor),
		      S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP |
		      S_IROTH | S_IWOTH, "cpc_eco%d", chan->minor);
#endif

	chan->CPCWait_q = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	if (!chan->CPCWait_q) {
		err("No memory left");
		goto error;
	}

	chan->buf = vmalloc(sizeof(CPC_MSG_T) * CPC_MSG_BUF_CNT);

	if (chan->buf == NULL) {
		err("CPCEco[%d]: Unable to get enough memory for buffering!", CPCEcoCnt);
		goto error;
	}

	ResetBuffer(chan);
	chan->controllerType = SJA1000;

	init_waitqueue_head(chan->CPCWait_q);

	/* Set CAN controller into reset mode */
	chan->write_byte(chan, SJA1000_CLK_DIV, SJA1000_CLK_PELICAN);
	mdelay(50);
	dbg("Read SJA1000_CLK_DIV: %2.2x ", chan->read_byte(chan, SJA1000_CLK_DIV));

	if (chan->read_byte(chan, SJA1000_CLK_DIV) != SJA1000_CLK_PELICAN) {
		err("No CPC-ECO connected to parport or no power supply!");
		goto error;
	}

	snprintf(card->deviceName, sizeof(card->deviceName), "cpc_eco%d", chan->minor);

	memset(&card->miscdev, 0, sizeof(card->miscdev));

	card->miscdev.minor = MISC_DYNAMIC_MINOR;
	card->miscdev.fops = &CPCEcoFops;
	card->miscdev.name = &card->deviceName[0];

	misc_register(&card->miscdev);

	chan->minor = card->miscdev.minor;

	chan->private = card;

	parport_disable_irq(card->pport);

	spin_lock_init(&card->slock);

	/* Route IRQ in parallel port controller */
	info("CPCEco: Device #%d successfully recognized!", CPCEcoCnt + 1);
	info("CPCEco: -> IRQ%d", card->irq);
	info("CPCEco: -> Base 0x%4.4lX", card->PA);
	info("CPCEco: -> Portno. %d", card->portnum);

	card->irqCounter = 0;

	CPCEcoCnt++;

	return;

error:
	CPCEco_delete(&CPCECO_Table[CPCEcoCnt]);

	return;
}

static void CPCEco_delete(CPC_ECO_T ** peco)
{
	CPC_ECO_T *card;

	if (!*peco)
		return;

	card = *peco;

	dbg("Deleting 0x%4.4lX", card->PA);

	if (card->pdev) {
		card->pdev->private = NULL;
		parport_release(card->pdev);
		parport_unregister_device(card->pdev);
	}

	/* cleanup all available channels */
	if (card->chan->CPCWait_q)
		kfree(card->chan->CPCWait_q);

	if (card->chan->buf)
		vfree(card->chan->buf);

#ifdef CONFIG_DEVFS_FS
	devfs_remove("cpc_eco%d", card->chan->minor);
#endif

	misc_deregister(&card->miscdev);

	if (card->chan)
		vfree(card->chan);

	kfree(card);

	*peco = NULL;

	return;
}

/**
 * Parallel port remove, remove CPC-ECO as well
 */
static void CPCEco_detach(struct parport *port)
{
	int i;

	for (i = 0; i < CPC_ECO_MAX_DEVICE; i++) {
		if (CPCECO_Table[i]) {
			if (CPCECO_Table[i]->PA == port->base) {
				CPCEco_delete(&CPCECO_Table[i]);
				break;
			}

			CPCEcoCnt--;
			CPCECO_Table[i] = NULL;
		}
	}
}

static int __init CPCEcoInit(void)
{
	int retval = 0;

	info("CPCEco[-] : Loading Driver v%s, build on %s at %s",
	     DRIVER_VERSION, __DATE__, __TIME__);
	info("CPCEco[-] : SJA1000 Driver v%s", SJA1000_VERSION);

	/* Register the parport driver:
	 * - all connected parports are probed
	 * - claim parport to prevent other parport driver to use it
	 */
	if (parport_register_driver(&CPCEco_driver)) {
		err("Could not register driver");
		retval = -EIO;
		goto err;
	}

	/* check if parport_register_driver has detect an CPC-ECO */
	if (!CPCEcoCnt) {
		err("No devices detected");
		parport_unregister_driver(&CPCEco_driver);
		retval = -ENODEV;
		goto err;
	}

	if (register_chrdev(CPC_ECO_MAJOR, "cpc-eco", &CPCEcoFops)) {
		err("Unable to get major: %d", CPC_ECO_MAJOR);
		parport_unregister_driver(&CPCEco_driver);
		retval = -EIO;
		goto err;
	}

	procDir = proc_mkdir(CPC_ECO_PROC_DIR, NULL);
	if (procDir) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procDir->owner = THIS_MODULE;
#endif
		procEntry = create_proc_read_entry("info", 0444, procDir,
						CPCEco_proc_read_info, NULL);

		if (!procEntry) {
			err("Could not create proc entry %s", CPC_ECO_PROC_DIR "/info");
			remove_proc_entry(CPC_ECO_PROC_DIR, NULL);
			procDir = NULL;
		} else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
			procEntry->owner = THIS_MODULE;
#endif
		}
	}

	return 0;

err:
	return retval;
}

static void __exit CPCEcoExit(void)
{
	info("CPCEco: unloading driver");

	wait_event(rmmodWq, !atomic_read(&useCount));

	parport_unregister_driver(&CPCEco_driver);

	unregister_chrdev(CPC_ECO_MAJOR, "cpc-eco");

	if (procDir) {
		if (procEntry)
			remove_proc_entry("info", procDir);
		remove_proc_entry(CPC_ECO_PROC_DIR, NULL);
	}

	return;
}

module_init(CPCEcoInit);
module_exit(CPCEcoExit);
