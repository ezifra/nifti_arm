/************************************************************************/
/* Kernel driver low level routines for all passive CPC interfaces      */
/*                                                                      */
/* Copyright 2000,2001,2002, 2003 Dr.-Ing. Thomas W?nsche               */
/*                                                                      */
/* Company:  EMS Dr. Thomas Wuensche                                    */
/*           Sonnenhang 3                                               */
/*           85304 Ilmmuenster                                          */
/*           Phone: +49-8441-490260                                     */
/*           Fax:   +49-8441-81860                                      */
/*           email: support@ems-wuensche.com                            */
/*           WWW:   www.ems-wuensche.com                                */
/*                                                                      */
/* All rights reserved                                                  */
/*                                                                      */
/* This code is provided "as is" without warranty of any kind, either   */
/* expressed or implied, including but not limited to the liability     */
/* concerning the freedom from material defects, the fitness for        */
/* particular purposes or the freedom of proprietary rights of third    */
/* parties.                                                             */
/************************************************************************/
/* Author:   Gerhard Uttenthaler, Sebastian Haas, Christian Schoett
 *           
 * Revision: 0.902 - 10.05.2002
 *                 - initial revision
 *
 * Revision: 1.000 - 03.01.03
 *                 - more functions
 *                 - adepted to new data structures
 +
 *           1.001 - 29.10.03
 *                 - added udelay(5) after CAN controller is set to
 *                   reset mode
 *
 *           1.002 - 30.11.03
 *                 - functions declared static
 *
 *           2.003 - CAN controller specific routines exported
 *                   to sja1000.h and pca82c200.h
 *                 - some return error codes corrected
 *                 - new function cpc_get_next_message()
 *
 *           2.004 - Bugfix: Buffer is changed before check if it is full
 *           
 *           2.005 - Sourced command processing out of driver
 *                 - Added BUS state control
 */

/****************************************************************************/
#ifndef _INLINE_
#define _INLINE_ inline
#endif

#include "sja1000.h"
#include "pca82c200.h"
#include "cc770.h"

/* --------------------------------------------------------------------------
 * cpc_examine_control
 * desc: examine control call and set/unset apropriate bits
 * params: chan - the channel
 *         val  - control value
 * return: 0 on success, otherwise -1
 * --------------------------------------------------------------------------
 * Date         Author           Comment
 * 11.02.2005   Sebastian Haas   Initial revision
 * -------------------------------------------------------------------------- */
static _INLINE_ int cpc_examine_control(CPC_CHAN_T * chan,
					unsigned char val)
{
	unsigned char subject = val & (~0x03);
	unsigned char command = val & (0x03);

//    printk(KERN_INFO "%s: Val %X Subject %X Command %X\n", __PRETTY_FUNCTION__, val, subject, command);

	/* sanity check */
	switch (command) {
	case CONTR_CONT_OFF:
	case CONTR_CONT_ON:
	case CONTR_SING_ON:
		break;
	default:
		return -1;
	}

	switch (subject) {
	case CONTR_CAN_State:
		chan->cpcCtrlCANState = command;
		break;
	case CONTR_CAN_Message:
		chan->cpcCtrlCANMessage = command;
		break;
	case CONTR_BusError:
		chan->cpcCtrlBUSState = command;
		break;
	default:
		return -1;
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * cpc_get_buffer_count
 * desc: get count of the buffer from a channel
 * params: chan - the channel
 * return: count of buffer
 * --------------------------------------------------------------------------
 * Date         Author           Comment
 * xx.xx.xxxx   Ch. Schoett       Initial revision
 * -------------------------------------------------------------------------- */
static _INLINE_ int cpc_get_buffer_count(CPC_CHAN_T * chan)
{
	/* check the buffer parameters */
	if (chan->iidx == chan->oidx) {
		if (!chan->WnR) {
			return CPC_MSG_BUF_CNT;
		} else {
			return 0;
		}
	} else if (chan->iidx >= chan->oidx)
		return (chan->iidx - chan->oidx) % CPC_MSG_BUF_CNT;
	else
		return (chan->iidx + CPC_MSG_BUF_CNT -
			chan->oidx) % CPC_MSG_BUF_CNT;

	return CPC_ERR_UNKNOWN;
}

/* --------------------------------------------------------------------------
 * cpc_get_next_message
 * desc: retrieve next message
 * params: chan
 * return: NULL if buffer is empty, otherwise the message
 * --------------------------------------------------------------------------
 * Date         Author           Comment
 * 17.06.2004   Sebastian Haas   Initial revision
 * 18.06.2004   Sebastian Haas   Changed name and prototype
 *                               Removed kernel stuff
 * -------------------------------------------------------------------------- */
static _INLINE_ CPC_MSG_T *cpc_get_next_message(CPC_CHAN_T * chan)
{
	CPC_MSG_T *cpc, *ovr;

	if (IsBufferEmpty(chan)) {
		return NULL;
	}

	cpc = &chan->buf[chan->oidx];

	chan->oidx = (chan->oidx + 1) % CPC_MSG_BUF_CNT;
	chan->WnR = 1;

	/* overrun active? */
	if (chan->ovr.event) {
		if (chan->ovrLockedBuffer > 0)
			chan->ovrLockedBuffer--;

		if (!chan->ovrLockedBuffer) {
			ovr = &chan->buf[chan->iidx];

			memset(ovr, 0, sizeof(CPC_MSG_T));

			ovr->type = CPC_MSG_T_OVERRUN;
			ovr->length = 2;

			ovr->ts_sec = chan->ovrTimeSec;
			ovr->ts_nsec = chan->ovrTimeNSec;

			ovr->msg.overrun.event = chan->ovr.event;
			ovr->msg.overrun.count = chan->ovr.count;

			/* reset overrun */
			chan->ovr.event = 0;
			chan->ovr.count = 0;
			chan->ovrLockedBuffer = 0;

			chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;
			chan->WnR = 0;
		}
	}

	return cpc;
}

/******************************************************************************************/
static int _INLINE_ putCPCInfo(CPC_CHAN_T * chan, unsigned char type)
{
	struct timeval now;

	if (IsBufferFull(chan))
		return CPC_ERR_UNKNOWN;

	do_gettimeofday(&now);

	chan->buf[chan->iidx].type = CPC_MSG_T_INFO;
	chan->buf[chan->iidx].ts_sec = now.tv_sec;
	chan->buf[chan->iidx].ts_nsec = now.tv_usec * 1000;
	chan->buf[chan->iidx].msg.info.source = CPC_INFOMSG_T_INTERFACE;
	chan->buf[chan->iidx].msg.info.type = type;

	switch (type) {
	case CPC_INFOMSG_T_VERSION:{
			chan->buf[chan->iidx].length =
			    strlen(CPC_INTERFACE_VERSION) + 2;
			sprintf(chan->buf[chan->iidx].msg.info.msg, "%s",
				CPC_INTERFACE_VERSION);
		}
		break;
	case CPC_INFOMSG_T_SERIAL:{
			chan->buf[chan->iidx].length =
			    strlen(CPC_INTERFACE_SERIAL) + 2;
			sprintf(chan->buf[chan->iidx].msg.info.msg, "%s",
				CPC_INTERFACE_SERIAL);
		}
		break;
	default:{
			chan->buf[chan->iidx].length = 2;
			chan->buf[chan->iidx].msg.info.type =
			    CPC_INFOMSG_T_UNKNOWN_TYPE;
		}
	}

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 2;
}

/******************************************************************************************/
static int _INLINE_ putDrvInfo(CPC_CHAN_T * chan, unsigned char type)
{
	struct timeval now;

	if (IsBufferFull(chan))
		return CPC_ERR_UNKNOWN;

	do_gettimeofday(&now);

	chan->buf[chan->iidx].type = CPC_MSG_T_INFO;
	chan->buf[chan->iidx].ts_sec = now.tv_sec;
	chan->buf[chan->iidx].ts_nsec = now.tv_usec * 1000;
	chan->buf[chan->iidx].msg.info.source = CPC_INFOMSG_T_DRIVER;
	chan->buf[chan->iidx].msg.info.type = type;

	switch (type) {
	case CPC_INFOMSG_T_VERSION:{
			chan->buf[chan->iidx].length =
			    strlen(CPC_DRIVER_VERSION) + 2;
			sprintf(chan->buf[chan->iidx].msg.info.msg, "%s\n",
				CPC_DRIVER_VERSION);
		}
		break;
	case CPC_INFOMSG_T_SERIAL:{
			chan->buf[chan->iidx].length =
			    strlen(CPC_DRIVER_SERIAL) + 2;
			sprintf(chan->buf[chan->iidx].msg.info.msg, "%s\n",
				CPC_DRIVER_SERIAL);
		}
		break;
	default:{
			chan->buf[chan->iidx].length = 2;
			chan->buf[chan->iidx].msg.info.type =
			    CPC_INFOMSG_T_UNKNOWN_TYPE;
		}
	}
	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 2;
}

/******************************************************************************************/
int _INLINE_ putCPCInfoError(CPC_CHAN_T * chan, unsigned char type)
{
	struct timeval now;

	if (IsBufferFull(chan))
		return CPC_ERR_UNKNOWN;

	do_gettimeofday(&now);

	chan->buf[chan->iidx].type = CPC_MSG_T_INFO;
	chan->buf[chan->iidx].ts_sec = now.tv_sec;
	chan->buf[chan->iidx].ts_nsec = now.tv_usec * 1000;
	chan->buf[chan->iidx].length = 2;
	chan->buf[chan->iidx].msg.info.source =
	    CPC_INFOMSG_T_UNKNOWN_SOURCE;

	switch (type) {
	case CPC_INFOMSG_T_VERSION:
	case CPC_INFOMSG_T_SERIAL:{
			chan->buf[chan->iidx].msg.info.type = type;
		}
		break;
	default:
		chan->buf[chan->iidx].msg.info.type =
		    CPC_INFOMSG_T_UNKNOWN_TYPE;
	}

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 2;

}

/* --------------------------------------------------------------------------
 * cpc_process_command
 * desc: Processes a CPC_CMD_T_*
 * params: chan - the channel
 *         msg  - message already copied to kernel space
 * return: 0 on success, otherwise CPC error codes
 * --------------------------------------------------------------------------
 * Date         Author           Comment
 * 11.02.2005   Sebastian Haas   Initial revision
 * -------------------------------------------------------------------------- */
static _INLINE_ int cpc_process_command(CPC_CHAN_T *chan, CPC_MSG_T * msg)
{
	int retval = 0;
	int handled = 1;

	switch (msg->type) {
		case CPC_CMD_T_CONTROL:
			if (cpc_examine_control(chan, msg->msg.generic[0])
			    < 0) {
				retval = CPC_ERR_SERVICE_NOT_SUPPORTED;
			} else {
				retval = sizeof(CPC_MSG_T);
			}
			break;

		case CPC_CMD_T_INQ_INFO:
			switch (msg->msg.info.source) {
			case CPC_INFOMSG_T_INTERFACE:
				retval = 
					putCPCInfo(chan, msg->msg.info.type);
				break;
			case CPC_INFOMSG_T_DRIVER:
				retval =
				    putDrvInfo(chan, msg->msg.info.type);
				break;
			default:
				retval = putCPCInfoError(chan, msg->msg.info.type);
			}
			break;

		case CPC_CMD_T_CLEAR_MSG_QUEUE:
			ResetBuffer(chan);
			chan->ovr.event = 0;
			chan->ovr.count = 0;
			chan->ovrLockedBuffer = 0;
			retval = 0;
			break;

		case CPC_CMD_T_INQ_MSG_QUEUE_CNT:
			retval = cpc_get_buffer_count(chan);
			break;

		default:
			handled = 0;
	}

	if(handled)
		return retval;

	if (chan->controllerType == PCA82C200) {
		switch (msg->type) {
		case CPC_CMD_T_CAN_EXIT:
			pca82c200_reset(chan);
			break;

		case CPC_CMD_T_XCAN:
		case CPC_CMD_T_XRTR:	/* PCA82C200 didn't support Extendend Frame Format */
			retval = CPC_ERR_UNKNOWN;
			break;

		case CPC_CMD_T_CAN:
			retval =
			    pca82c200_send_can(chan, &msg->msg.canmsg);
			break;
		case CPC_CMD_T_RTR:
			retval =
			    pca82c200_send_rtr(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_CANSTATE:
			retval = pca82c200_get_canstate(chan);
			break;

		case CPC_CMD_T_CAN_PRMS:
			if (msg->msg.canparams.cc_type == PCA82C200) {
				retval =
				    pca82c200_init_contr(chan,
							 &msg->msg.
							 canparams.
							 cc_params.
							 pca82c200);
			} else {
				retval = CPC_ERR_WRONG_CONTROLLER_TYPE;
			}
			break;

		case CPC_CMD_T_INQ_CAN_PARMS:
			retval = pca82c200_inq_params(chan);
			break;

		default:
			retval = CPC_ERR_SERVICE_NOT_SUPPORTED;
			break;
		}
	} else if (chan->controllerType == SJA1000) {
		switch (msg->type) {
		case CPC_CMD_T_CAN_EXIT:
			sja1000_reset(chan);
			break;

		case CPC_CMD_T_XCAN:
			retval = sja1000_send_xcan(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_XRTR:
			retval = sja1000_send_xrtr(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_CAN:
			retval = sja1000_send_can(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_RTR:
			retval = sja1000_send_rtr(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_CANSTATE:
			retval = sja1000_get_canstate(chan);
			break;

		case CPC_CMD_T_CAN_PRMS:
			if (msg->msg.canparams.cc_type == SJA1000) {
				retval =
				    sja1000_init_contr(chan,
						       &msg->msg.canparams.
						       cc_params.sja1000);
			} else {
				retval = CPC_ERR_WRONG_CONTROLLER_TYPE;
			}
			break;

		case CPC_CMD_T_INQ_CAN_PARMS:
			retval = sja1000_inq_params(chan);
			break;

		default:
			retval = CPC_ERR_SERVICE_NOT_SUPPORTED;
			break;
		}
	} else if (chan->controllerType == AN82527) {
		switch (msg->type) {
		case CPC_CMD_T_CAN_EXIT:
			cc770_reset(chan);
			break;

		case CPC_CMD_T_XCAN:
			retval = cc770_send_xcan(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_XRTR:
			retval = cc770_send_xrtr(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_CAN:
			retval = cc770_send_can(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_RTR:
			retval = cc770_send_rtr(chan, &msg->msg.canmsg);
			break;

		case CPC_CMD_T_CANSTATE:
			retval = cc770_get_canstate(chan);
			break;

		case CPC_CMD_T_CAN_PRMS:
			if (msg->msg.canparams.cc_type == SJA1000) {
				retval = cc770_init_contr(chan, &msg->msg.canparams.cc_params.sja1000);
			} else {
				retval = CPC_ERR_WRONG_CONTROLLER_TYPE;
			}
			break;

		case CPC_CMD_T_INQ_CAN_PARMS:
			retval = cc770_inq_params(chan);
			break;

		default:
			retval = CPC_ERR_SERVICE_NOT_SUPPORTED;
			break;
		}
	} else {
		info("DRIVER ERROR UNKNOWN CONTROLLER\n");
		retval = CPC_ERR_UNKNOWN;
	}

	return retval;
}

