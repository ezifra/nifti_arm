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
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include "../include/cpc.h"
#include "../include/cpc_int.h"
#include "cpccard.h"

#include "../include/version.h"

#include <asm/io.h>
#include <asm/irq.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ciscode.h>
#include <pcmcia/ds.h>
#include <pcmcia/cisreg.h>

static int debug = 0;
module_param(debug, int, S_IRUGO);

#define CPC_CARD_ENABLER_VERSION  "0.933"
#define CPC_CARD_ENABLER_MAJOR    31

MODULE_AUTHOR("Sebastian Haas <haas@ems-wuensche.com>");
MODULE_DESCRIPTION("PC-CARD Service Handler for CPC-CARD Driver");
MODULE_VERSION(CPC_CARD_ENABLER_VERSION);
MODULE_LICENSE("GPL");

/* Bit map of interrupts to choose from
 * This means pick from 15, 14, 12, 11, 10, 9, 7, 5, 4, and 3
 */
static u_long irq_mask = 0xdeb8;
/***********************************************************/

static int get_tuple(client_handle_t handle, tuple_t * tuple,
		             cisparse_t * parse);
static int next_tuple(client_handle_t handle, tuple_t * tuple,
		              cisparse_t * parse);
static int first_tuple(client_handle_t handle, tuple_t * tuple,
		               cisparse_t * parse);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
static int CPCCardAttach(struct pcmcia_device *p_dev);
static void CPCCardDetach(struct pcmcia_device *p_dev);
#else
static dev_link_t *CPCCardAttach(void);
static void CPCCardDetach(dev_link_t * link);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
static void CPCCardConfig(struct pcmcia_device *);
static void CPCCardRelease(struct pcmcia_device *);
#else
static void CPCCardConfig(dev_link_t *);
static void CPCCardRelease(dev_link_t *);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
static int CPCCardEvent(event_t event, int priority,
			event_callback_args_t * args);
#endif

/* The dev_info variable is the "key" that is used to match up this
 * device driver with appropriate cards, through the card configuration
 * database.
 */
static dev_info_t dev_info = "cpc-card_cs";

/* Tuple related functions */
static int get_tuple(client_handle_t handle, tuple_t * tuple,
                     cisparse_t * parse)
{
	int i = pcmcia_get_tuple_data(handle, tuple);

	if (i != CS_SUCCESS)
		return i;

	return pcmcia_parse_tuple(handle, tuple, parse);
}

static int next_tuple(client_handle_t handle, tuple_t * tuple,
                      cisparse_t * parse)
{
	int i = pcmcia_get_next_tuple(handle, tuple);

	if (i != CS_SUCCESS)
		return i;

	return get_tuple(handle, tuple, parse);
}

static int first_tuple(client_handle_t handle, tuple_t * tuple,
                       cisparse_t * parse)
{
	int i = pcmcia_get_first_tuple(handle, tuple);

	if (i != CS_SUCCESS)
		return i;

	return get_tuple(handle, tuple, parse);
}

/**
 * Callback for PCMCIA subsystem, if card was plugged in
 * @return
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
static int CPCCardAttach(struct pcmcia_device *link)
#else
static dev_link_t *CPCCardAttach(void)
#endif
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16))
	client_reg_t client_reg;
	int ret;

	dev_link_t *link;

	enter_call();

	/* Initialize the dev_link_t structure */
	link = vmalloc(sizeof(struct dev_link_t));
	memset(link, 0, sizeof(struct dev_link_t));
#endif
	/* The io structure describes IO port mapping */
	link->io.NumPorts1 = 16;
	link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
	link->io.NumPorts2 = 16;
	link->io.Attributes2 = IO_DATA_PATH_WIDTH_16;
	link->io.IOAddrLines = 5;

	/* Interrupt setup */
	link->irq.Attributes =
	    IRQ_TYPE_DYNAMIC_SHARING | IRQ_HANDLE_PRESENT;
	link->irq.IRQInfo1 = IRQ_INFO2_VALID | IRQ_LEVEL_ID;
	link->irq.IRQInfo2 = irq_mask;
	link->irq.Handler = CPCCard_interrupt;

	/* General socket configuration */
	link->conf.Attributes = CONF_ENABLE_IRQ;
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	link->conf.Vcc = 50;
#endif
	link->conf.IntType = INT_MEMORY_AND_IO;
	link->conf.ConfigIndex = 1;
	link->conf.Present = PRESENT_OPTION;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
	/* Register with Card Services */
	link->handle = p_dev;
	p_dev->instance = link;

	link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
#endif
	CPCCardConfig(link);

	return 0;
#else

	/* Register with Card Services */
	client_reg.dev_info = &dev_info;
	client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
	client_reg.EventMask =
	    CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	    CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	    CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;

	client_reg.event_handler = CPCCardEvent;
	client_reg.Version = 0x0210;
	client_reg.event_callback_args.client_data = link;

	/* register new link */
	ret = pcmcia_register_client(&link->handle, &client_reg);
	if (ret != 0) {
		cs_error(link->handle, RegisterClient, ret);
		CPCCardDetach(link);
		return NULL;
	}

	return link;
#endif
}

/**
 * Device was unplugged
 * @param link
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
static void CPCCardDetach(struct pcmcia_device *link)
#else
static void CPCCardDetach(dev_link_t * link)
#endif
{
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
	dev_link_t *link = dev_to_instance(p_dev);
#endif

	if (!link) {
		info("CPC-CARD_CS: link was lost");
		return;
	}
#if !(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
	if (link->state & DEV_CONFIG) {
		info("CPC-CARD_CS: Linkstate is active, detach postponed");
		link->state |= DEV_STALE_LINK;
		return;
	}

	/* Break the link with Card Services */
	if (link->handle)
		pcmcia_deregister_client(link->handle);
#endif

	/* call CPC-CARD driver */
	CPCCard_unregister(link->priv);

#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
	if (link->state & DEV_CONFIG)
#endif
	{
		CPCCardRelease(link);
	}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	/* free */
	vfree(link);
#endif
	info("CPC-CARD_CS: Device was disconnected");

	return;
}

/**
 * Configure device
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
static void CPCCardConfig(struct pcmcia_device *link)
#else
static void CPCCardConfig(dev_link_t * link)
#endif
{
	client_handle_t handle;
	tuple_t tuple;
	cisparse_t parse;
	u_char buf[64];
	win_req_t req;
	memreq_t mem;
	int i;
	CPC_CARD_T *card = NULL;

#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
	dev_link_t *link = dev_to_instance(p_dev);
#endif

	handle = link;

	// This reads the card's CONFIG tuple to find its configuration registers.
	do {
		tuple.DesiredTuple = CISTPL_CONFIG;
		i = pcmcia_get_first_tuple(handle, &tuple);
		if (i != CS_SUCCESS)
			break;

		tuple.TupleData = buf;
		tuple.TupleDataMax = 64;
		tuple.TupleOffset = 0;
		i = pcmcia_get_tuple_data(handle, &tuple);
		if (i != CS_SUCCESS)
			break;

		i = pcmcia_parse_tuple(handle, &tuple, &parse);
		if (i != CS_SUCCESS)
			break;
		link->conf.ConfigBase = parse.config.base;
	} while (0);

	if (i != CS_SUCCESS) {
		cs_error(handle, ParseTuple, i);
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
		link->state &= ~DEV_CONFIG_PENDING;
#endif
		return;
	}
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	// Configure card
	link->state |= DEV_CONFIG;
#endif
	do {
		// try to get board information from CIS
		tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
		tuple.Attributes = 0;

		if (first_tuple(handle, &tuple, &parse) == CS_SUCCESS) {
			while (1) {
				//if this tuple has an IRQ info, keep it for later use */
				if (parse.cftable_entry.irq.
				    IRQInfo1 & IRQ_INFO2_VALID) {
					err("CPC-CARD_CS: irqmask=0x%x",
							parse.cftable_entry.irq.IRQInfo2);
					link->irq.IRQInfo2 = parse.cftable_entry.irq.IRQInfo2;
				}

				// card has a memory window
				if (parse.cftable_entry.mem.nwin > 0) {
					// Allocate a 4K memory window */
					req.Attributes = WIN_DATA_WIDTH_8 |
						WIN_MEMORY_TYPE_CM | WIN_ENABLE | WIN_USE_WAIT;
					req.Base = 0;
					req.Size = 0x1000;
					req.AccessSpeed = 0;
					link->win = (window_handle_t)handle;

					i = pcmcia_request_window(&link, &req, &link->win);
					if (i != CS_SUCCESS) {
						cs_error(handle, RequestWindow, i);
						return;
					}

					mem.Page = 0;
					mem.CardOffset = parse.cftable_entry.mem.win[0].card_addr;
					i = pcmcia_map_mem_page(link->win, &mem);
					if (i != CS_SUCCESS) {
						cs_error(handle, MapMemPage, i);
						return;
					}
				}
				if (next_tuple(handle, &tuple, &parse) !=  CS_SUCCESS)
					break;
			}
		} else {
			err("CPC-CARD_CS: can't get card information");
		}

		card = link->priv = CPCCard_register(req.Base);
		if (!card) {
			err("CPC-CARD_CS: registering failed");
			break;
		}
		link->irq.Instance = card;

		i = pcmcia_request_irq(handle, &link->irq);
		if (i != CS_SUCCESS) {
			err("CPC-CARD_CS: Can't request IRQ mapping");
			cs_error(handle, RequestIRQ, i);
			break;
		}
		info("CPC-CARD_CS: IRQ%d assigned", link->irq.AssignedIRQ);

		/* This actually configures the PCMCIA socket -- setting up
		 * the I/O windows and the interrupt mapping.
		 */
		i = pcmcia_request_configuration(handle, &link->conf);
		if (i != CS_SUCCESS) {
			cs_error(handle, RequestConfiguration, i);
			break;
		}
	} while (0);

	// If any step failed, release any partially configured state
	if (i != 0 || !card) {
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
		CPCCardRelease(p_dev);
#else
		CPCCardRelease(link);
#endif
		return;
	}

	/* At this point, the dev_node_t structure(s) should be
	 * initialized and arranged in a linked list at link->dev.
	 */
	sprintf(card->node.dev_name, "cpc-card0");
	card->node.major = CPC_CARD_ENABLER_MAJOR;
	card->node.minor = 0;
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	link->dev = &card->node;
	link->state &= ~DEV_CONFIG_PENDING;
#endif

	dbg("CPCCard_cs: device loaded\n");
}

/**
 * Final removal call
 * @param link
 */
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
static void CPCCardRelease(struct pcmcia_device *p_dev)
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,16))
static void CPCCardRelease(struct pcmcia_device *link)
#else
static void CPCCardRelease(dev_link_t * link)
#endif
{
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
	dev_link_t *link = dev_to_instance(p_dev);
#endif

	if (!link)
		return;

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	// Unlink the device chain
	link->dev = NULL;
	link->state &= ~DEV_CONFIG;
#endif

	// Don't bother checking to see if these succeed or not
	if (link->win)
		pcmcia_release_window(link->win);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,16))
	pcmcia_disable_device(link);
#else
	pcmcia_release_configuration(link->handle);
	pcmcia_release_io(link->handle, &link->io);
	pcmcia_release_irq(link->handle, &link->irq);
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	if (link->state & DEV_STALE_LINK) {
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16))
		CPCCardDetach(p_dev);
#else
		CPCCardDetach(link);
#endif
	}
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
/**
 * The card event handler
 */
static int CPCCardEvent(event_t event, int priority,
			event_callback_args_t * args)
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	dev_link_t *link = args->client_data;
#else
	pcmcia_device_t *link = arg->client_data;
#endif

	switch (event) {
	case CS_EVENT_REGISTRATION_COMPLETE:
		dbg("CPC-CARD_CS: registration complete");
		break;
	case CS_EVENT_CARD_REMOVAL:
		link->state &= ~DEV_PRESENT;
		if (link->state & DEV_CONFIG) {
			CPCCardRelease(link);
		}
		break;
	case CS_EVENT_CARD_INSERTION:
		link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
		CPCCardConfig(link);
		break;
	case CS_EVENT_PM_SUSPEND:
		link->state |= DEV_SUSPEND;
	case CS_EVENT_RESET_PHYSICAL:
		if (link->state & DEV_CONFIG)
			pcmcia_release_configuration(link->handle);
		break;
	case CS_EVENT_PM_RESUME:
		link->state &= ~DEV_SUSPEND;
	case CS_EVENT_CARD_RESET:
		if (link->state & DEV_CONFIG)
			pcmcia_request_configuration(link->handle, &link->conf);
		break;
	}
	return 0;
}
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12))
static struct pcmcia_device_id cpccard_ids[] = {
	PCMCIA_DEVICE_PROD_ID123("EMS_T_W", "CPC-Card", "V2.0", 0xeab1ea23,
				 0xa338573f, 0xe4575800),
	PCMCIA_DEVICE_NULL,
};

MODULE_DEVICE_TABLE(pcmcia, cpccard_ids);
#endif

static struct pcmcia_driver cpccard_cs_driver = {
	.owner = THIS_MODULE,
	.drv = {
		.name = dev_info,	/* important for database */
		},
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
	.probe = CPCCardAttach,
	.remove = CPCCardDetach,
#else
	.attach = CPCCardAttach,
	.detach = CPCCardDetach,
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12))
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16))
	.event = CPCCardEvent,
#endif
	.id_table = cpccard_ids,
#endif
};

static int __init CPCCard_cs_init(void)
{
	info("CPC-CARD_CS: Loaded v%s", CPC_CARD_ENABLER_VERSION);

	pcmcia_register_driver(&cpccard_cs_driver);

	return 0;
}


static void __exit CPCCard_cs_exit(void)
{
	pcmcia_unregister_driver(&cpccard_cs_driver);
	dbg("CPC-CARD_CS: pcmcia device unregistered");
}

module_init(CPCCard_cs_init);
module_exit(CPCCard_cs_exit);
