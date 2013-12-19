/*
 * PCA82C200 Definitions
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
#ifndef PCA82C200_H
#define PCA82C200_H

#define PCA82C200_VERSION "1.007"

#define PCA82C200_OFFSET_0 0x40
#define PCA82C200_OFFSET_1 0x60

/*----- PCA82C200 Register definitions ----*/
#define PCA82C200_CONTROL    0
#define PCA82C200_COMMAND    1
#define PCA82C200_STATUS     2
#define PCA82C200_INTR       3
#define PCA82C200_ACC_CODE   4
#define PCA82C200_ACC_MASK   5
#define PCA82C200_BTR0       6
#define PCA82C200_BTR1       7
#define PCA82C200_OUTP_CONTR 8
//#define PCA82C200_CLOCK_DIV  31

#define PCA82C200_RTR       0x10

/*----- Transmit Buffer ----*/
#define PCA82C200_TX_ID     10
#define PCA82C200_TX_DLC    11
#define PCA82C200_TX_DATA0  12
#define PCA82C200_TX_DATA1  13
#define PCA82C200_TX_DATA2  14
#define PCA82C200_TX_DATA3  15
#define PCA82C200_TX_DATA4  16
#define PCA82C200_TX_DATA5  17
#define PCA82C200_TX_DATA6  18
#define PCA82C200_TX_DATA7  19

/*----- Receive Buffer ----*/
#define PCA82C200_RX_ID     20
#define PCA82C200_RX_DLC    21
#define PCA82C200_RX_DATA0  22
#define PCA82C200_RX_DATA1  23
#define PCA82C200_RX_DATA2  24
#define PCA82C200_RX_DATA3  25
#define PCA82C200_RX_DATA4  26
#define PCA82C200_RX_DATA5  27
#define PCA82C200_RX_DATA6  28
#define PCA82C200_RX_DATA7  29

/*----- Flags ----*/
/* interrupt */
#define PCA82C200_IER_OI   0x10
#define PCA82C200_IER_EI   0x08
#define PCA82C200_IER_TI   0x04
#define PCA82C200_IER_RI   0x02

#define PCA82C200_IR_WU   0x10
#define PCA82C200_IR_OI   0x08
#define PCA82C200_IR_EI   0x04
#define PCA82C200_IR_TI   0x02
#define PCA82C200_IR_RI   0x01

#define PCA82C200_CONTR_RR 0x01	/* reset request */

/* command */
#define PCA82C200_CMD_SLEEP      0x10
#define PCA82C200_CMD_CLEAR      0x08
#define PCA82C200_CMD_RELEASE_RX 0x04
#define PCA82C200_CMD_ABORT_TX   0x02
#define PCA82C200_CMD_TX_REQ     0x01

/* status register */
#define PCA82C200_SR_BS  0x80
#define PCA82C200_SR_ES  0x40
#define PCA82C200_SR_TS  0x20
#define PCA82C200_SR_RS  0x10
#define PCA82C200_SR_TCS 0x08
#define PCA82C200_SR_TBS 0x04
#define PCA82C200_SR_DO  0x02
#define PCA82C200_SR_RBS 0x01


static inline int pca82c200_interrupt(CPC_CHAN_T * chan,
				      struct timeval now)
{
	volatile unsigned char interrupt_source;
	volatile unsigned char data;

	int i;

	interrupt_source = chan->read_byte(chan, PCA82C200_INTR);
	interrupt_source &= ~0xE0;	// mask reserved bits out

	/* no interrupt occured by pca82c200 (e.g. in reset mode) */
	if (!interrupt_source
	    || (chan->
		read_byte(chan, PCA82C200_CONTROL) & PCA82C200_CONTR_RR))
		return 0;

	if (interrupt_source & PCA82C200_IR_RI) {	/* RECEIVE INTERRUPT ****************** */
		if (chan->
		    read_byte(chan, PCA82C200_STATUS) & PCA82C200_SR_RBS) {
			if (!chan->ovr.event) {
				if (!(IsBufferFull(chan))
				    && (chan->
					cpcCtrlCANMessage & (CONTR_CONT_ON
							     |
							     CONTR_SING_ON)))
				{
					CPC_MSG_T *recmsg =
					    &chan->buf[chan->iidx];

					recmsg->ts_sec = now.tv_sec;
					recmsg->ts_nsec =
					    now.tv_usec * 1000;
					data =
					    chan->read_byte(chan,
							    PCA82C200_RX_DLC);
					recmsg->length = (data & 0x0f) + 5;

					recmsg->msg.canmsg.id =
					    (chan->
					     read_byte(chan,
						       PCA82C200_RX_ID) <<
					     3) | ((data & 0xe0) >> 5);
					recmsg->msg.canmsg.length =
					    data & 0x0f;

					if (data & PCA82C200_RTR) {
						recmsg->type =
						    CPC_MSG_T_RTR;
						chan->recvStdRtr++;
					} else {
						recmsg->type =
						    CPC_MSG_T_CAN;
						chan->recvStdCan++;

						for (i = 0;
						     i <
						     (recmsg->msg.canmsg.
						      length); i++)
							recmsg->msg.canmsg.
							    msg[i] =
							    chan->
							    read_byte(chan,
								      PCA82C200_RX_DATA0
								      + i);
					}
					/* store message */
					chan->WnR = 0;
					chan->iidx =
					    (chan->iidx +
					     1) % CPC_MSG_BUF_CNT;

					/* wake up possible sleeping processes */
					wake_up_interruptible(chan->
							      CPCWait_q);
				} else if (IsBufferFull(chan)) {
					chan->ovrLockedBuffer =
					    CPC_OVR_GAP;

					chan->ovr.event |=
					    CPC_OVR_EVENT_CAN;
					chan->ovr.count = 0;

					chan->ovrTimeSec = now.tv_sec;
					chan->ovrTimeNSec =
					    now.tv_usec * 1000;

					chan->lostMessages++;
				}
			} else {
				chan->ovr.event |= CPC_OVR_EVENT_CAN;
				if ((chan->ovr.count + 1) <= 127)
					chan->ovr.count++;


				/* count overall lost messages */
				chan->lostMessages++;
			}
			chan->write_byte(chan, PCA82C200_COMMAND,
					 PCA82C200_CMD_CLEAR |
					 PCA82C200_CMD_RELEASE_RX);
			chan->cpcCtrlCANMessage &= ~CONTR_SING_ON;	/* clear single */
		}
	}

	/**/ if (interrupt_source & PCA82C200_IR_TI) {	/* TRANSMIT INTERRUPT ***************** */
		/* wake up possible sleeping processes */
		wake_up_interruptible(chan->CPCWait_q);
	}

	if (interrupt_source & PCA82C200_IR_EI) {	/* ERROR WARNING INTERRUPT ************ */
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
						    PCA82C200_STATUS) &
				    0xc0;

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

	return 1;
}

/*---- Functions for accessing the register ----*/
static inline void pca82c200_reset(CPC_CHAN_T * chan)
{
	/* reset mode */
	chan->write_byte(chan, PCA82C200_CONTROL, PCA82C200_CONTR_RR);
	udelay(10);

	chan->write_byte(chan, PCA82C200_BTR0, 0x00);	/* Set bit timing register 0           */
	chan->write_byte(chan, PCA82C200_BTR1, 0x00);	/* Set bit timing register 1           */
	chan->write_byte(chan, PCA82C200_OUTP_CONTR, 0x00);	/* Set output control register         */

	return;
}

static inline int pca82c200_send_can(CPC_CHAN_T * chan,
				     CPC_CAN_MSG_T * msg)
{
	unsigned char id, dlc;
	int i;

	if (msg->id > 0x7ff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (!(chan->read_byte(chan, PCA82C200_STATUS) & PCA82C200_SR_TBS)) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	id = (msg->id >> 3) & 0xff;
	dlc = ((msg->id & 0x07) << 5) & 0xe0;
	dlc |= (msg->length & 0x0f);

	chan->write_byte(chan, PCA82C200_TX_ID, id);
	chan->write_byte(chan, PCA82C200_TX_DLC, dlc);

	for (i = 0; i < msg->length; i++)
		chan->write_byte(chan, PCA82C200_TX_DATA0 + i,
				 msg->msg[i]);

	// Transmission Request
	chan->write_byte(chan, PCA82C200_COMMAND, PCA82C200_CMD_TX_REQ);

	chan->sentStdCan++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

static inline int pca82c200_send_rtr(CPC_CHAN_T * chan,
				     CPC_CAN_MSG_T * msg)
{
	unsigned char id, dlc;

	if (msg->id > 0x7ff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (!(chan->read_byte(chan, PCA82C200_STATUS) & PCA82C200_SR_TBS)) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	id = (msg->id >> 3) & 0xff;
	dlc = ((msg->id & 0x07) << 5) & 0xe0;
	dlc |= PCA82C200_RTR;
	dlc |= (msg->length & 0x0f);

	chan->write_byte(chan, PCA82C200_TX_ID, id);
	chan->write_byte(chan, PCA82C200_TX_DLC, dlc);

	// Transmission Request
	chan->write_byte(chan, PCA82C200_COMMAND, PCA82C200_CMD_TX_REQ);

	chan->sentStdRtr++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

static inline int pca82c200_inq_params(CPC_CHAN_T * chan)
{
	struct timeval now;
	volatile unsigned char control = 0;
	CPC_PCA82C200_PARAMS_T *can =
	    &chan->buf[chan->iidx].msg.canparams.cc_params.pca82c200;

	if (IsBufferFull(chan))
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;

	/* get control reg and clear reset mode flag if set */
	control =
	    chan->read_byte(chan,
			    PCA82C200_CONTROL) & (~PCA82C200_CONTR_RR);

	chan->write_byte(chan, PCA82C200_CONTROL, PCA82C200_CONTR_RR);	/* reset mode */
	udelay(10);

	do_gettimeofday(&now);

	chan->buf[chan->iidx].type = CPC_MSG_T_CAN_PRMS;
	chan->buf[chan->iidx].ts_sec = now.tv_sec;
	chan->buf[chan->iidx].ts_nsec = now.tv_usec * 1000;
	chan->buf[chan->iidx].length = sizeof(CPC_CAN_PARAMS_T);
	chan->buf[chan->iidx].msg.canparams.cc_type = PCA82C200;

	if (!(chan->read_byte(chan, PCA82C200_CONTROL) & PCA82C200_CONTR_RR)) {	/* check, if controller is in init mode */
		err("CPC-CARD: Controller not in reset mode");
		return CPC_ERR_NO_RESET_MODE;
	}

	can->acc_code = chan->read_byte(chan, PCA82C200_ACC_CODE);	/* get ACC0                      */
	can->acc_mask = chan->read_byte(chan, PCA82C200_ACC_MASK);	/* get ACM0                      */
	can->btr0 = chan->read_byte(chan, PCA82C200_BTR0);	/* get bit timing register 0     */
	can->btr1 = chan->read_byte(chan, PCA82C200_BTR1);	/* get bit timing register 1     */
	can->outp_contr = chan->read_byte(chan, PCA82C200_OUTP_CONTR);	/* get output control register   */

	chan->write_byte(chan, PCA82C200_CONTROL, control);	/* remember original control reg */

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_PARAMS_T);
}

static inline int pca82c200_init_contr(CPC_CHAN_T * chan,
				       CPC_PCA82C200_PARAMS_T * can)
{
	pca82c200_reset(chan);	/* Set CAN controller to reset mode  */
	udelay(5);

	if (!(chan->read_byte(chan, PCA82C200_CONTROL) & PCA82C200_CONTR_RR)) {	/* check, if controller is in init mode */
		err("CPC-CARD: Controller not in reset mode");
		return CPC_ERR_NO_RESET_MODE;
	}

	chan->write_byte(chan, PCA82C200_ACC_CODE, can->acc_code);	/* Set ACC0                            */
	chan->write_byte(chan, PCA82C200_ACC_MASK, can->acc_mask);	/* Set ACM0                            */
	chan->write_byte(chan, PCA82C200_BTR0, can->btr0);	/* Set bit timing register 0           */
	chan->write_byte(chan, PCA82C200_BTR1, can->btr1);	/* Set bit timing register 1           */
	chan->write_byte(chan, PCA82C200_OUTP_CONTR, can->outp_contr);	/* Set output control register         */

	chan->write_byte(chan, PCA82C200_CONTROL, PCA82C200_IER_RI | PCA82C200_IER_EI | PCA82C200_IER_TI);	/* Enable RX interrupt and disable RESET MODE */

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_PARAMS_T);
}

static inline unsigned char pca82c200_get_canstate(CPC_CHAN_T * chan)
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
	    chan->read_byte(chan, PCA82C200_STATUS);

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 1;
}

#endif
