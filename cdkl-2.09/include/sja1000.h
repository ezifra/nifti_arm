/*
 * SJA1000 Definitions
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
#ifndef SJA1000_H
#define SJA1000_H

#define SJA1000_OFFSET 0

#define SJA1000_VERSION "1.010"

/* SJA1000 register definitions *****************************************/
#define SJA1000_MODE          (SJA1000_OFFSET + 0)
#define SJA1000_COMMAND       (SJA1000_OFFSET + 1)
#define SJA1000_STATUS        (SJA1000_OFFSET + 2)
#define SJA1000_INT_REG       (SJA1000_OFFSET + 3)
#define SJA1000_INT_ENABLE    (SJA1000_OFFSET + 4)
#define SJA1000_BTR0          (SJA1000_OFFSET + 6)
#define SJA1000_BTR1          (SJA1000_OFFSET + 7)
#define SJA1000_OUTP_CONTR    (SJA1000_OFFSET + 8)
#define SJA1000_ALC           (SJA1000_OFFSET + 11)
#define SJA1000_ECC           (SJA1000_OFFSET + 12)
#define SJA1000_EWLR          (SJA1000_OFFSET + 13)
#define SJA1000_RXERR         (SJA1000_OFFSET + 14)
#define SJA1000_TXERR         (SJA1000_OFFSET + 15)
	/* RESET MODE */
#define SJA1000_ACC_CODE0     (SJA1000_OFFSET + 16)
#define SJA1000_ACC_CODE1     (SJA1000_OFFSET + 17)
#define SJA1000_ACC_CODE2     (SJA1000_OFFSET + 18)
#define SJA1000_ACC_CODE3     (SJA1000_OFFSET + 19)
#define SJA1000_ACC_MASK0     (SJA1000_OFFSET + 20)
#define SJA1000_ACC_MASK1     (SJA1000_OFFSET + 21)
#define SJA1000_ACC_MASK2     (SJA1000_OFFSET + 22)
#define SJA1000_ACC_MASK3     (SJA1000_OFFSET + 23)

/*STANDARD-FRAME-FORMAT  */
	/* OPERATING MODE = READ */
#define SJA1000_RX_FI         (SJA1000_OFFSET + 16)
#define SJA1000_SFF_RX_ID1    (SJA1000_OFFSET + 17)
#define SJA1000_SFF_RX_ID2    (SJA1000_OFFSET + 18)
#define SJA1000_SFF_RX_DATA0	(SJA1000_OFFSET + 19)
#define SJA1000_SFF_RX_DATA1	(SJA1000_OFFSET + 20)
#define SJA1000_SFF_RX_DATA2	(SJA1000_OFFSET + 21)
#define SJA1000_SFF_RX_DATA3	(SJA1000_OFFSET + 22)
#define SJA1000_SFF_RX_DATA4	(SJA1000_OFFSET + 23)
#define SJA1000_SFF_RX_DATA5	(SJA1000_OFFSET + 24)
#define SJA1000_SFF_RX_DATA6	(SJA1000_OFFSET + 25)
#define SJA1000_SFF_RX_DATA7	(SJA1000_OFFSET + 26)

	/* OPERATING MODE = WRITE */
#define SJA1000_TX_FI         (SJA1000_OFFSET + 16)
#define SJA1000_SFF_TX_ID1    (SJA1000_OFFSET + 17)
#define SJA1000_SFF_TX_ID2    (SJA1000_OFFSET + 18)
#define SJA1000_SFF_TX_DATA0	(SJA1000_OFFSET + 19)
#define SJA1000_SFF_TX_DATA1	(SJA1000_OFFSET + 20)
#define SJA1000_SFF_TX_DATA2	(SJA1000_OFFSET + 21)
#define SJA1000_SFF_TX_DATA3	(SJA1000_OFFSET + 22)
#define SJA1000_SFF_TX_DATA4	(SJA1000_OFFSET + 23)
#define SJA1000_SFF_TX_DATA5	(SJA1000_OFFSET + 24)
#define SJA1000_SFF_TX_DATA6	(SJA1000_OFFSET + 25)
#define SJA1000_SFF_TX_DATA7	(SJA1000_OFFSET + 26)


/*EXTENDED-FRAME-FORMAT*/
	/* OPERATING MODE = READ */
#define SJA1000_RX_FI         (SJA1000_OFFSET + 16)
#define SJA1000_EFF_RX_ID1    (SJA1000_OFFSET + 17)
#define SJA1000_EFF_RX_ID2    (SJA1000_OFFSET + 18)
#define SJA1000_EFF_RX_ID3    (SJA1000_OFFSET + 19)
#define SJA1000_EFF_RX_ID4    (SJA1000_OFFSET + 20)
#define SJA1000_EFF_RX_DATA0	(SJA1000_OFFSET + 21)
#define SJA1000_EFF_RX_DATA1	(SJA1000_OFFSET + 22)
#define SJA1000_EFF_RX_DATA2	(SJA1000_OFFSET + 23)
#define SJA1000_EFF_RX_DATA3	(SJA1000_OFFSET + 24)
#define SJA1000_EFF_RX_DATA4	(SJA1000_OFFSET + 25)
#define SJA1000_EFF_RX_DATA5	(SJA1000_OFFSET + 26)
#define SJA1000_EFF_RX_DATA6	(SJA1000_OFFSET + 27)
#define SJA1000_EFF_RX_DATA7	(SJA1000_OFFSET + 28)

       /* OPERATING MODE = WRITE */
#define SJA1000_TX_FI         (SJA1000_OFFSET + 16)
#define SJA1000_EFF_TX_ID1    (SJA1000_OFFSET + 17)
#define SJA1000_EFF_TX_ID2    (SJA1000_OFFSET + 18)
#define SJA1000_EFF_TX_ID3    (SJA1000_OFFSET + 19)
#define SJA1000_EFF_TX_ID4    (SJA1000_OFFSET + 20)
#define SJA1000_EFF_TX_DATA0	(SJA1000_OFFSET + 21)
#define SJA1000_EFF_TX_DATA1	(SJA1000_OFFSET + 22)
#define SJA1000_EFF_TX_DATA2	(SJA1000_OFFSET + 23)
#define SJA1000_EFF_TX_DATA3	(SJA1000_OFFSET + 24)
#define SJA1000_EFF_TX_DATA4	(SJA1000_OFFSET + 25)
#define SJA1000_EFF_TX_DATA5	(SJA1000_OFFSET + 26)
#define SJA1000_EFF_TX_DATA6	(SJA1000_OFFSET + 27)
#define SJA1000_EFF_TX_DATA7	(SJA1000_OFFSET + 28)

#define SJA1000_RX_MSG_COUNT	(SJA1000_OFFSET + 29)
#define SJA1000_RX_BUFF_ADR   (SJA1000_OFFSET + 30)
#define SJA1000_CLK_DIV       (SJA1000_OFFSET + 31)

#define SJA1000_CLK_PELICAN             0x87

/* command register */
#define SJA1000_CMR_GTS                 0x10
#define SJA1000_CMR_CDO                 0x08
#define SJA1000_CMR_RRB                 0x04
#define SJA1000_CMR_AT                  0x02
#define SJA1000_CMR_TR                  0x01

#define SJA1000_INIT_MODE               0x01

/* flags **************************/
/* status register                */
#define SJA1000_SR_RBS                  0x01
#define SJA1000_SR_DOS                  0x02
#define SJA1000_SR_TBS                  0x04
#define SJA1000_SR_TCS                  0x08
#define SJA1000_SR_RS                   0x10
#define SJA1000_SR_TS                   0x20
#define SJA1000_SR_ES                   0x40
#define SJA1000_SR_BS                   0x80

/* interrupt register             */
#define SJA1000_IR_RI                   0x01
#define SJA1000_IR_TI                   0x02
#define SJA1000_IR_EI                   0x04
#define SJA1000_IR_DOI                  0x08
#define SJA1000_IR_WUI                  0x10
#define SJA1000_IR_EPI                  0x20
#define SJA1000_IR_ALI                  0x40
#define SJA1000_IR_BEI                  0x80

/* interrupt enable register      */
#define SJA1000_IER_RIE                 0x01
#define SJA1000_IER_TIE                 0x02
#define SJA1000_IER_EIE                 0x04
#define SJA1000_IER_DOIE                0x08	/* Data overrun interrupt enable */
#define SJA1000_IER_WUIE                0x10
#define SJA1000_IER_EPIE                0x20
#define SJA1000_IER_ALIE                0x40
#define SJA1000_IER_BEIE                0x80


/* SJA1000 functions ****************************************************/
static inline void sja1000_reset(CPC_CHAN_T * chan)
{
	//printk(KERN_INFO "Sja1000 Reset: Called\n");

	chan->write_byte(chan, SJA1000_MODE, 1);

	chan->write_byte(chan, SJA1000_BTR0, 0x00);	/* Set bit timing register 0 */
	chan->write_byte(chan, SJA1000_BTR1, 0x00);	/* Set bit timing register 1 */
	chan->write_byte(chan, SJA1000_OUTP_CONTR, 0x00);	/* Set output control register */

	chan->write_byte(chan, SJA1000_INT_ENABLE, 0x00);	/* disable all irqs */

}

static inline int sja1000_interrupt(CPC_CHAN_T * chan, struct timeval now)
{
	unsigned int i;
	volatile unsigned char interrupt_source;
	volatile unsigned char dataInfo;
	int handled = 0;

	interrupt_source = chan->read_byte(chan, SJA1000_INT_REG);

	/* no interrupt occured by sja1000 */
	if (!interrupt_source
	    || (chan->read_byte(chan, SJA1000_MODE) & SJA1000_INIT_MODE))
		return handled;

	chan->handledIrqs++;

	if (interrupt_source & SJA1000_IR_RI) {	/* RECEIVE INTERRUPT ****************** */
		handled = 1;

//        printk(KERN_INFO "Sja1000 IRQ: Receive Interrupt\n");

		if (!chan->ovr.event) {
			/* Check whether we have a free slot in the circular buffer         */
			if (!(IsBufferFull(chan))
			    && (chan->
				cpcCtrlCANMessage & (CONTR_CONT_ON |
						     CONTR_SING_ON))) {
				CPC_MSG_T *recmsg = &chan->buf[chan->iidx];

				dataInfo =
				    chan->read_byte(chan, SJA1000_RX_FI);

				recmsg->ts_sec = now.tv_sec;
				recmsg->ts_nsec = now.tv_usec * 1000;

				if ((dataInfo & 0xc0) == 0x00) {	/* Standard-Data-Frame */
					recmsg->type = CPC_MSG_T_CAN;
					recmsg->length =
					    (dataInfo & 0x0f) + 5;
					recmsg->msg.canmsg.id =
					    ((chan->
					      read_byte(chan,
							SJA1000_SFF_RX_ID1)
					      << 3) | (chan->
						       read_byte(chan,
								 SJA1000_SFF_RX_ID2)
						       >> 5));
					recmsg->msg.canmsg.length =
					    dataInfo & 0x0f;
					for (i = 0;
					     i <
					     (recmsg->msg.canmsg.length);
					     i++)
						recmsg->msg.canmsg.msg[i] =
						    chan->read_byte(chan,
								    SJA1000_SFF_RX_DATA0
								    + i);
					chan->recvStdCan++;
				} else if ((dataInfo & 0xc0) == 0x40) {	/* Standard-RTR-Frame */
					recmsg->type = CPC_MSG_T_RTR;
					recmsg->length = 5;
					recmsg->msg.canmsg.id =
					    ((chan->
					      read_byte(chan,
							SJA1000_SFF_RX_ID1)
					      << 3) | (chan->
						       read_byte(chan,
								 SJA1000_SFF_RX_ID2)
						       >> 5));
					recmsg->msg.canmsg.length =
					    dataInfo & 0x0f;
					chan->recvStdRtr++;
				} else if ((dataInfo & 0xc0) == 0x80) {	/* Extended-Data-Frame */
					recmsg->type = CPC_MSG_T_XCAN;
					recmsg->length =
					    (dataInfo & 0x0f) + 5;
					recmsg->msg.canmsg.id =
					    (((unsigned long) chan->
					      read_byte(chan,
							SJA1000_EFF_RX_ID1)
					      << 21) | ((unsigned long)
							chan->
							read_byte(chan,
								  SJA1000_EFF_RX_ID2)
							<< 13) | ((unsigned
								   long)
								  chan->
								  read_byte
								  (chan,
								   SJA1000_EFF_RX_ID3)
								  << 5) |
					     ((unsigned long) chan->
					      read_byte(chan,
							SJA1000_EFF_RX_ID4)
					      >> 3));
					recmsg->msg.canmsg.length =
					    dataInfo & 0x0f;
					for (i = 0;
					     i < recmsg->msg.canmsg.length;
					     i++)
						recmsg->msg.canmsg.msg[i] =
						    chan->read_byte(chan,
								    SJA1000_EFF_RX_DATA0
								    + i);
					chan->recvExtCan++;
				} else if ((dataInfo & 0xc0) == 0xc0) {	/* Extended-RTR-Frame */
					recmsg->type = CPC_MSG_T_XRTR;
					recmsg->length = 5;
					recmsg->msg.canmsg.id =
					    (((unsigned long) chan->
					      read_byte(chan,
							SJA1000_EFF_RX_ID1)
					      << 21) | ((unsigned long)
							chan->
							read_byte(chan,
								  SJA1000_EFF_RX_ID2)
							<< 13) | ((unsigned
								   long)
								  chan->
								  read_byte
								  (chan,
								   SJA1000_EFF_RX_ID3)
								  << 5) |
					     ((unsigned long) chan->
					      read_byte(chan,
							SJA1000_EFF_RX_ID4)
					      >> 3));
					recmsg->msg.canmsg.length =
					    dataInfo & 0x0f;
					chan->recvExtRtr++;
				}
				chan->WnR = 0;
				chan->iidx =
				    (chan->iidx + 1) % CPC_MSG_BUF_CNT;
			} else if (IsBufferFull(chan)) {
				chan->ovrLockedBuffer = CPC_OVR_GAP;

				chan->ovr.event |= CPC_OVR_EVENT_CAN;
				chan->ovr.count = 1;

				chan->ovrTimeSec = now.tv_sec;
				chan->ovrTimeNSec = now.tv_usec * 1000;

				chan->lostMessages++;
			}
		} else {
			chan->ovr.event |= CPC_OVR_EVENT_CAN;
			if ((chan->ovr.count + 1) <= 127)
				chan->ovr.count++;

			/* count overall lost messages */
			chan->lostMessages++;
		}
		chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_RRB | SJA1000_CMR_CDO);	/* Release Receive Buffer */
		chan->cpcCtrlCANMessage &= ~CONTR_SING_ON;	/* clear single */

		wake_up_interruptible(chan->CPCWait_q);
	}

	if (interrupt_source & SJA1000_IR_TI) {	/* TRANSMIT INTERRUPT ***************** */
		handled = 1;

//      printk(KERN_INFO "Sja1000 IRQ: Transmit Interrupt\n");

		wake_up_interruptible(chan->CPCWait_q);
	}

	/* this bit is set on every change (set
	 * and clear) of either the error status
	 * or bus status bits...
	 */
	if (interrupt_source & SJA1000_IR_EI) {	/* ERROR WARNING INTERRUPT ************ */
		handled = 1;

//        printk(KERN_INFO "Sja1000 IRQ: Error Warning Interrupt\n");

		if (!chan->ovr.event) {
			/* Check whether we have a free slot in the circular buffer         */
			if (!(IsBufferFull(chan))
			    && (chan->
				cpcCtrlCANState & (CONTR_CONT_ON |
						   CONTR_SING_ON))) {
				CPC_MSG_T *recmsg = &chan->buf[chan->iidx];

				recmsg->type = CPC_MSG_T_CANSTATE;
				recmsg->length = 1;
				recmsg->ts_sec = now.tv_sec;
				recmsg->ts_nsec = now.tv_usec * 1000L;
				recmsg->msgid = 0;
				recmsg->msg.canstate =
				    chan->read_byte(chan,
						    SJA1000_STATUS) & 0xc0;

				chan->WnR = 0;
				chan->iidx =
				    (chan->iidx + 1) % CPC_MSG_BUF_CNT;

				wake_up_interruptible(chan->CPCWait_q);
			} else if (IsBufferFull(chan)) {
				chan->ovrLockedBuffer = CPC_OVR_GAP;

				chan->ovr.event |= CPC_OVR_EVENT_CANSTATE;
				chan->ovr.count = 0;

				chan->ovrTimeSec = now.tv_sec;
				chan->ovrTimeNSec = now.tv_usec * 1000;

				chan->lostMessages++;
			}
		} else {
			chan->ovr.event |= CPC_OVR_EVENT_CANSTATE;

			/* count overall lost messages */
			chan->lostMessages++;
		}

		chan->cpcCtrlCANState &= ~CONTR_SING_ON;	/* clear single */
	}
	if (interrupt_source & SJA1000_IR_DOI) {	/* RECEIVE INTERRUPT: Data overrun interrupt ************** */
		handled = 1;

//              printk(KERN_INFO "Sja1000 IRQ: Received Data overrun interrupt\n");


		if (!chan->ovr.event) {

			if (!IsBufferFull(chan)) {
				CPC_MSG_T *recmsg = &chan->buf[chan->iidx];

				recmsg->type = CPC_MSG_T_OVERRUN;
				recmsg->length = sizeof(CPC_OVERRUN_T);
				recmsg->ts_sec = now.tv_sec;
				recmsg->ts_nsec = now.tv_usec * 1000L;
				recmsg->msgid = 0;

				recmsg->msg.canstate =
				    chan->read_byte(chan,
						    SJA1000_STATUS) & 0xc0;
				recmsg->msg.overrun.count = CPC_OVR_HW;
				recmsg->msg.overrun.event =
				    CPC_OVR_EVENT_CAN;

				chan->WnR = 0;
				chan->iidx =
				    (chan->iidx + 1) % CPC_MSG_BUF_CNT;

				wake_up_interruptible(chan->CPCWait_q);

			} else if (IsBufferFull(chan)) {	/* buffer is full */
				chan->ovrLockedBuffer = CPC_OVR_GAP;

				chan->ovr.event = CPC_OVR_EVENT_CAN;
				chan->ovr.count = CPC_OVR_HW;

				chan->ovrTimeSec = now.tv_sec;
				chan->ovrTimeNSec = now.tv_usec * 1000;

				chan->lostMessages++;
			}
			/* end buffer full */
		} else {	/* ovr->event != 0 */
			chan->ovr.event |= CPC_OVR_EVENT_CAN;
			chan->ovr.count |= CPC_OVR_HW;
			chan->lostMessages++;
		}
	}
	/* this bit is set when the CAN controller
	 * detects an error on the CAN-bus...
	 */
	if (interrupt_source & SJA1000_IR_BEI) {	/* BUS ERROR INTERRUPT **************** */
		handled = 1;

//        printk(KERN_INFO "Sja1000 IRQ: Bus Error Interrupt\n");
		if (!chan->ovr.event) {
			/* Check whether we have a free slot in the circular buffer         */
			if (!(IsBufferFull(chan))
			    && (chan->
				cpcCtrlBUSState & (CONTR_CONT_ON |
						   CONTR_SING_ON))) {
				CPC_MSG_T *recmsg = &chan->buf[chan->iidx];

				recmsg->type = CPC_MSG_T_CANERROR;
				recmsg->length = sizeof(CPC_CAN_ERROR_T);
				recmsg->ts_sec = now.tv_sec;
				recmsg->ts_nsec = now.tv_usec * 1000L;
				recmsg->msgid = 0;
				recmsg->msg.error.ecode =
				    CPC_CAN_ECODE_ERRFRAME;
				recmsg->msg.error.cc.cc_type = SJA1000;
				recmsg->msg.error.cc.regs.sja1000.ecc =
				    chan->read_byte(chan, SJA1000_ECC);
				recmsg->msg.error.cc.regs.sja1000.rxerr =
				    chan->read_byte(chan, SJA1000_RXERR);
				recmsg->msg.error.cc.regs.sja1000.txerr =
				    chan->read_byte(chan, SJA1000_TXERR);

				chan->WnR = 0;
				chan->iidx =
				    (chan->iidx + 1) % CPC_MSG_BUF_CNT;

				wake_up_interruptible(chan->CPCWait_q);
			} else if (IsBufferFull(chan)) {
				chan->ovrLockedBuffer = CPC_OVR_GAP;
				chan->ovr.event |= CPC_OVR_EVENT_BUSERROR;
				chan->ovr.count = 0;

				chan->ovrTimeSec = now.tv_sec;
				chan->ovrTimeNSec = now.tv_usec * 1000;

				chan->lostMessages++;
			}
		} else {
			chan->ovr.event |= CPC_OVR_EVENT_BUSERROR;

			/* count overall lost messages */
			chan->lostMessages++;
		}

		chan->cpcCtrlBUSState &= ~CONTR_SING_ON;	/* clear single */
	}
	// add other interrupt sources here, if enabled
	if (!handled) {
		// hack
		handled = 1;
		printk(KERN_INFO "%s - unknown SJA1000 interrupt"
		       " occured %2.2X\n", __PRETTY_FUNCTION__,
		       interrupt_source);
	}

	return handled;

}

/******************************************************************************************/
/* Transmit Standard-Frame-Format-DATA-Message */
static inline int sja1000_send_can(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	unsigned char finfo, ident1, ident2;
	unsigned i;

	if (msg->id > 0x7ff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (!(chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	finfo = msg->length;
	ident1 = msg->id >> 3;
	ident2 = (msg->id << 5) & 0xe0;

	chan->write_byte(chan, SJA1000_TX_FI, finfo);
	chan->write_byte(chan, SJA1000_SFF_TX_ID1, ident1);
	chan->write_byte(chan, SJA1000_SFF_TX_ID2, ident2);

	for (i = 0; i < msg->length; i++)
		chan->write_byte(chan, SJA1000_SFF_TX_DATA0 + i,
				 msg->msg[i]);

	// Transmission Request
	chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_TR);

	chan->sentStdCan++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

/******************************************************************************************/
/* Transmit Standard-Frame-Format-REMOTE-Message */
static inline int sja1000_send_rtr(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	unsigned char finfo, ident1, ident2;

	if (msg->id > 0x7ff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (!(chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS))
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;

	finfo = msg->length | 0x40;
	ident1 = msg->id >> 3;
	ident2 = (msg->id << 5) & 0xe0;

	chan->write_byte(chan, SJA1000_TX_FI, finfo);
	chan->write_byte(chan, SJA1000_SFF_TX_ID1, ident1);
	chan->write_byte(chan, SJA1000_SFF_TX_ID2, ident2);

	// Transmission Request
	chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_TR);

	chan->sentStdRtr++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

/******************************************************************************************/
/* Transmit Extended-Frame-Format-DATA-Message       */
static inline int sja1000_send_xcan(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	unsigned char xfinfo, xident1, xident2, xident3, xident4;
	unsigned i;

	if (msg->id > 0x1fffffff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (!(chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	xfinfo = msg->length | 0x80;
	xident1 = msg->id >> 21;
	xident2 = msg->id >> 13;
	xident3 = msg->id >> 5;
	xident4 = (msg->id << 3) & 0xf8;

	chan->write_byte(chan, SJA1000_TX_FI, xfinfo);
	chan->write_byte(chan, SJA1000_EFF_TX_ID1, xident1);
	chan->write_byte(chan, SJA1000_EFF_TX_ID2, xident2);
	chan->write_byte(chan, SJA1000_EFF_TX_ID3, xident3);
	chan->write_byte(chan, SJA1000_EFF_TX_ID4, xident4);

	for (i = 0; i < msg->length; i++)
		chan->write_byte(chan, SJA1000_EFF_TX_DATA0 + i,
				 msg->msg[i]);

	// Transmission Request
	chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_TR);

	chan->sentExtCan++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}


/******************************************************************************************/
/* Transmit Extended-Frame-Format-REMOTE-Message */
static inline int sja1000_send_xrtr(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	unsigned char xfinfo, xident1, xident2, xident3, xident4;

	if (msg->id > 0x1fffffff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (!(chan->read_byte(chan, SJA1000_STATUS) & SJA1000_SR_TBS)) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	xfinfo = msg->length | 0xc0;
	xident1 = msg->id >> 21;
	xident2 = msg->id >> 13;
	xident3 = msg->id >> 5;
	xident4 = (msg->id << 3) & 0xf8;

	chan->write_byte(chan, SJA1000_TX_FI, xfinfo);
	chan->write_byte(chan, SJA1000_EFF_TX_ID1, xident1);
	chan->write_byte(chan, SJA1000_EFF_TX_ID2, xident2);
	chan->write_byte(chan, SJA1000_EFF_TX_ID3, xident3);
	chan->write_byte(chan, SJA1000_EFF_TX_ID4, xident4);

	// Transmission Request
	chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_TR);

	chan->sentExtRtr++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

/**************************************************************************************/
static inline int sja1000_get_canstate(CPC_CHAN_T * chan)
{
	struct timeval now;

	if (IsBufferFull(chan))
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;

	do_gettimeofday(&now);

	chan->buf[chan->iidx].type = CPC_MSG_T_CANSTATE;
	chan->buf[chan->iidx].ts_sec = now.tv_sec;
	chan->buf[chan->iidx].ts_nsec = now.tv_usec * 1000;
	chan->buf[chan->iidx].length = 1;
	chan->buf[chan->iidx].msg.canstate =
	    chan->read_byte(chan, SJA1000_STATUS);

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 1;
}

/****************************************************************************/
/* read CAN-Controller parameter */
static inline int sja1000_inq_params(CPC_CHAN_T * chan)
{
	struct timeval now;
	CPC_SJA1000_PARAMS_T *can =
	    &chan->buf[chan->iidx].msg.canparams.cc_params.sja1000;

	if (IsBufferFull(chan))
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;

	/* safe mode */
	can->mode = chan->read_byte(chan, SJA1000_MODE) & 0xfe;

	chan->write_byte(chan, SJA1000_MODE, SJA1000_INIT_MODE);	/* Clear RESET MODE, set Filter Mode   */
	udelay(10);

	if (!(chan->read_byte(chan, SJA1000_MODE) & SJA1000_INIT_MODE))	/* check, if controller is in init mode */
		return CPC_ERR_NO_RESET_MODE;

	do_gettimeofday(&now);

	chan->buf[chan->iidx].type = CPC_MSG_T_CAN_PRMS;
	chan->buf[chan->iidx].ts_sec = now.tv_sec;
	chan->buf[chan->iidx].ts_nsec = now.tv_usec * 1000;
	chan->buf[chan->iidx].length = sizeof(CPC_CAN_PARAMS_T);
	chan->buf[chan->iidx].msg.canparams.cc_type = SJA1000;

	can->acc_code0 = chan->read_byte(chan, SJA1000_ACC_CODE0);	/* get ACC0                            */
	can->acc_code1 = chan->read_byte(chan, SJA1000_ACC_CODE1);	/* get ACC1                            */
	can->acc_code2 = chan->read_byte(chan, SJA1000_ACC_CODE2);	/* get ACC2                            */
	can->acc_code3 = chan->read_byte(chan, SJA1000_ACC_CODE3);	/* get ACC3                            */
	can->acc_mask0 = chan->read_byte(chan, SJA1000_ACC_MASK0);	/* get ACM0                            */
	can->acc_mask1 = chan->read_byte(chan, SJA1000_ACC_MASK1);	/* get ACM1                            */
	can->acc_mask2 = chan->read_byte(chan, SJA1000_ACC_MASK2);	/* get ACM2                            */
	can->acc_mask3 = chan->read_byte(chan, SJA1000_ACC_MASK3);	/* get ACM3                            */
	can->btr0 = chan->read_byte(chan, SJA1000_BTR0);	/* get bit timing register 0           */
	can->btr1 = chan->read_byte(chan, SJA1000_BTR1);	/* get bit timing register 1           */
	can->outp_contr = chan->read_byte(chan, SJA1000_OUTP_CONTR);	/* get output control register         */

	chan->write_byte(chan, SJA1000_MODE, (can->mode & 0xfe));	/* Clear RESET MODE, set Filter Mode   */

	chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_RRB | SJA1000_CMR_CDO);	/* release receive buffer and clear Data Overrun */

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 1;
}

static inline int sja1000_init_contr(CPC_CHAN_T * chan,
				     CPC_SJA1000_PARAMS_T * can)
{
	sja1000_reset(chan);
	udelay(5);
	//printk(KERN_INFO "Sja1000 Init Contr\n");

	if (!(chan->read_byte(chan, SJA1000_MODE) & SJA1000_INIT_MODE))	/* check, if controller is in init mode */
		return CPC_ERR_NO_RESET_MODE;

	//printk(KERN_INFO "Sja1000 Init Contr: After InitMode check\n");

	chan->write_byte(chan, SJA1000_CLK_DIV, SJA1000_CLK_PELICAN);	/* PeliCAN-Mode                        */
	chan->write_byte(chan, SJA1000_ACC_CODE0, can->acc_code0);	/* Set ACC0                            */
	chan->write_byte(chan, SJA1000_ACC_CODE1, can->acc_code1);	/* Set ACC1                            */
	chan->write_byte(chan, SJA1000_ACC_CODE2, can->acc_code2);	/* Set ACC2                            */
	chan->write_byte(chan, SJA1000_ACC_CODE3, can->acc_code3);	/* Set ACC3                            */
	chan->write_byte(chan, SJA1000_ACC_MASK0, can->acc_mask0);	/* Set ACM0                            */
	chan->write_byte(chan, SJA1000_ACC_MASK1, can->acc_mask1);	/* Set ACM1                            */
	chan->write_byte(chan, SJA1000_ACC_MASK2, can->acc_mask2);	/* Set ACM2                            */
	chan->write_byte(chan, SJA1000_ACC_MASK3, can->acc_mask3);	/* Set ACM3                            */
	chan->write_byte(chan, SJA1000_BTR0, can->btr0);	/* Set bit timing register 0           */
	chan->write_byte(chan, SJA1000_BTR1, can->btr1);	/* Set bit timing register 1           */
	chan->write_byte(chan, SJA1000_OUTP_CONTR, can->outp_contr);	/* Set output control register         */
	chan->write_byte(chan, SJA1000_INT_ENABLE, SJA1000_IER_RIE | SJA1000_IER_TIE | SJA1000_IER_EIE | SJA1000_IER_BEIE | SJA1000_IER_DOIE);	/* Enable Rx/Tx/Error interrupt */
	chan->write_byte(chan, SJA1000_MODE, can->mode);	/* Clear RESET MODE, set Filter Mode   */
	chan->write_byte(chan, SJA1000_COMMAND, SJA1000_CMR_RRB | SJA1000_CMR_CDO);	/* release receive buffer and clear Data Overrun */

	return CPCMSG_HEADER_LEN + sizeof(CPC_SJA1000_PARAMS_T);
}

#endif
