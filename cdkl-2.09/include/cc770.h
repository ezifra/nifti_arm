/*
 * CC770 Definitions
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
#ifndef CC770_H
#define CC770_H

#define CC770_OFFSET 0

#define CC770_VERSION "1.000"

#define CC770_CLK_FAST	0x09

/* CC770 register definitions *********************************************/
#define CC770_CONTROL			(CC770_OFFSET + 0x00)
#define CC770_STATUS			(CC770_OFFSET + 0x01)
#define CC770_CPU_IFACE		(CC770_OFFSET + 0x02)

#define CC770_HSR_LB			(CC770_OFFSET + 0x04)	// High Speed Read
#define CC770_HSR_HB			(CC770_OFFSET + 0x05)

#define CC770_GMSTD_1			(CC770_OFFSET + 0x06)	// ACC STD FRAME
#define CC770_GMSTD_2			(CC770_OFFSET + 0x07)

#define CC770_GMEXT_1			(CC770_OFFSET + 0x08)	// ACC EXT FRAME
#define CC770_GMEXT_2			(CC770_OFFSET + 0x09)
#define CC770_GMEXT_3			(CC770_OFFSET + 0x0A)
#define CC770_GMEXT_4			(CC770_OFFSET + 0x0B)

#define CC770_MSG15_1			(CC770_OFFSET + 0x0C)	// ACC MSG15
#define CC770_MSG15_2			(CC770_OFFSET + 0x0D)
#define CC770_MSG15_3			(CC770_OFFSET + 0x0E)
#define CC770_MSG15_4			(CC770_OFFSET + 0x0F)

#define CC770_CLK_OUT			(CC770_OFFSET + 0x1F)
#define CC770_BUS_CONF			(CC770_OFFSET + 0x2F)
#define CC770_BTR0				(CC770_OFFSET + 0x3F)
#define CC770_BTR1				(CC770_OFFSET + 0x4F)
#define CC770_INT_REG			(CC770_OFFSET + 0x5F)
#define CC770_RXERR				(CC770_OFFSET + 0x6F)
#define CC770_TXERR				(CC770_OFFSET + 0x7F)
#define CC770_P1_CONF			(CC770_OFFSET + 0x9F)
#define CC770_P2_CONF			(CC770_OFFSET + 0xAF)
#define CC770_P1_IN				(CC770_OFFSET + 0xBF)
#define CC770_P2_IN				(CC770_OFFSET + 0xCF)
#define CC770_P1_OUT			(CC770_OFFSET + 0xDF)
#define CC770_P2_OUT			(CC770_OFFSET + 0xEF)
#define CC770_SERIAL_RESET		(CC770_OFFSET + 0xFF)
#define CC770_MESSAGE1			(CC770_OFFSET + 0x10)
#define CC770_MESSAGE2			(CC770_OFFSET + 0x20)
#define CC770_MESSAGE3			(CC770_OFFSET + 0x30)
#define CC770_MESSAGE4			(CC770_OFFSET + 0x40)
#define CC770_MESSAGE5			(CC770_OFFSET + 0x50)
#define CC770_MESSAGE6			(CC770_OFFSET + 0x50)
#define CC770_MESSAGE7			(CC770_OFFSET + 0x60)
#define CC770_MESSAGE8			(CC770_OFFSET + 0x70)
#define CC770_MESSAGE9			(CC770_OFFSET + 0x80)
#define CC770_MESSAGE10			(CC770_OFFSET + 0x90)
#define CC770_MESSAGE11			(CC770_OFFSET + 0xA0)
#define CC770_MESSAGE12			(CC770_OFFSET + 0xB0)
#define CC770_MESSAGE13			(CC770_OFFSET + 0xC0)
#define CC770_MESSAGE14			(CC770_OFFSET + 0xE0)
#define CC770_MESSAGE15			(CC770_OFFSET + 0xF0)

/* Message Register
 * Use the MESSAGE1-15 defines as offset */

#define CC770_MSG_CONTROL0		0x00
#define CC770_MSG_CONTROL1		0x01
#define CC770_MSG_ARB0			0x02
#define CC770_MSG_ARB1			0x03
#define CC770_MSG_ARB2			0x04
#define CC770_MSG_ARB3			0x05
#define CC770_MSG_CONFIG		0x06
#define CC770_MSG_DATA0			0x07
#define CC770_MSG_DATA1			0x08
#define CC770_MSG_DATA2			0x09
#define CC770_MSG_DATA3			0x0A
#define CC770_MSG_DATA4			0x0B
#define CC770_MSG_DATA5			0x0C
#define CC770_MSG_DATA6			0x0D
#define CC770_MSG_DATA7			0x0E

/* STANDARD FRAME FORMAT */
/* OPERATING MODE = READ */
#define CC770_SFF_RX_FI			(CC770_MESSAGE1 + CC770_MSG_CONFIG)
#define CC770_SFF_RX_ID1		(CC770_MESSAGE1 + CC770_MSG_ARB0)
#define CC770_SFF_RX_ID2		(CC770_MESSAGE1 + CC770_MSG_ARB1)
#define CC770_SFF_RX_CFG		(CC770_MESSAGE1 + CC770_MSG_CONFIG)
#define CC770_SFF_RX_DATA0		(CC770_MESSAGE1 + CC770_MSG_DATA0)
#define CC770_SFF_RX_DATA1		(CC770_MESSAGE1 + CC770_MSG_DATA1)
#define CC770_SFF_RX_DATA2		(CC770_MESSAGE1 + CC770_MSG_DATA2)
#define CC770_SFF_RX_DATA3		(CC770_MESSAGE1 + CC770_MSG_DATA3)
#define CC770_SFF_RX_DATA4		(CC770_MESSAGE1 + CC770_MSG_DATA4)
#define CC770_SFF_RX_DATA5		(CC770_MESSAGE1 + CC770_MSG_DATA5)
#define CC770_SFF_RX_DATA6		(CC770_MESSAGE1 + CC770_MSG_DATA6)
#define CC770_SFF_RX_DATA7		(CC770_MESSAGE1 + CC770_MSG_DATA7)

/* STANDARD FRAME FORMAT */
/* OPERATING MODE = WRITE */
#define CC770_SFF_TX_FI			(CC770_MESSAGE2 + CC770_MSG_CONFIG)
#define CC770_SFF_TX_ID1		(CC770_MESSAGE2 + CC770_MSG_ARB0)
#define CC770_SFF_TX_ID2		(CC770_MESSAGE2 + CC770_MSG_ARB1)
#define CC770_SFF_TX_CFG		(CC770_MESSAGE2 + CC770_MSG_CONFIG)
#define CC770_SFF_TX_DATA0		(CC770_MESSAGE2 + CC770_MSG_DATA0)
#define CC770_SFF_TX_DATA1		(CC770_MESSAGE2 + CC770_MSG_DATA1)
#define CC770_SFF_TX_DATA2		(CC770_MESSAGE2 + CC770_MSG_DATA2)
#define CC770_SFF_TX_DATA3		(CC770_MESSAGE2 + CC770_MSG_DATA3)
#define CC770_SFF_TX_DATA4		(CC770_MESSAGE2 + CC770_MSG_DATA4)
#define CC770_SFF_TX_DATA5		(CC770_MESSAGE2 + CC770_MSG_DATA5)
#define CC770_SFF_TX_DATA6		(CC770_MESSAGE2 + CC770_MSG_DATA6)
#define CC770_SFF_TX_DATA7		(CC770_MESSAGE2 + CC770_MSG_DATA7)

/*EXTENDED-FRAME-FORMAT*/
/* OPERATING MODE = READ */
#define CC770_EFF_RX_ID1		(CC770_MESSAGE3 + CC770_MSG_ARB0)
#define CC770_EFF_RX_ID2		(CC770_MESSAGE3 + CC770_MSG_ARB1)
#define CC770_EFF_RX_ID3		(CC770_MESSAGE3 + CC770_MSG_ARB2)
#define CC770_EFF_RX_ID4		(CC770_MESSAGE3 + CC770_MSG_ARB3)
#define CC770_EFF_RX_CFG		(CC770_MESSAGE3 + CC770_MSG_CONFIG)
#define CC770_EFF_RX_DATA0		(CC770_MESSAGE3 + CC770_MSG_DATA0)
#define CC770_EFF_RX_DATA1		(CC770_MESSAGE3 + CC770_MSG_DATA1)
#define CC770_EFF_RX_DATA2		(CC770_MESSAGE3 + CC770_MSG_DATA2)
#define CC770_EFF_RX_DATA3		(CC770_MESSAGE3 + CC770_MSG_DATA3)
#define CC770_EFF_RX_DATA4		(CC770_MESSAGE3 + CC770_MSG_DATA4)
#define CC770_EFF_RX_DATA5		(CC770_MESSAGE3 + CC770_MSG_DATA5)
#define CC770_EFF_RX_DATA6		(CC770_MESSAGE3 + CC770_MSG_DATA6)
#define CC770_EFF_RX_DATA7		(CC770_MESSAGE3 + CC770_MSG_DATA7)

/* EXTENDED FRAME FORMAT */
/* OPERATING MODE = WRITE */
#define CC770_EFF_TX_FI			(CC770_MESSAGE4 + CC770_MSG_CONFIG)
#define CC770_EFF_TX_ID1		(CC770_MESSAGE4 + CC770_MSG_ARB0)
#define CC770_EFF_TX_ID2		(CC770_MESSAGE4 + CC770_MSG_ARB1)
#define CC770_EFF_TX_ID3		(CC770_MESSAGE4 + CC770_MSG_ARB2)
#define CC770_EFF_TX_ID4		(CC770_MESSAGE4 + CC770_MSG_ARB3)
#define CC770_EFF_TX_CFG		(CC770_MESSAGE4 + CC770_MSG_CONFIG)
#define CC770_EFF_TX_DATA0		(CC770_MESSAGE4 + CC770_MSG_DATA0)
#define CC770_EFF_TX_DATA1		(CC770_MESSAGE4 + CC770_MSG_DATA1)
#define CC770_EFF_TX_DATA2		(CC770_MESSAGE4 + CC770_MSG_DATA2)
#define CC770_EFF_TX_DATA3		(CC770_MESSAGE4 + CC770_MSG_DATA3)
#define CC770_EFF_TX_DATA4		(CC770_MESSAGE4 + CC770_MSG_DATA4)
#define CC770_EFF_TX_DATA5		(CC770_MESSAGE4 + CC770_MSG_DATA5)
#define CC770_EFF_TX_DATA6		(CC770_MESSAGE4 + CC770_MSG_DATA6)
#define CC770_EFF_TX_DATA7		(CC770_MESSAGE4 + CC770_MSG_DATA7)

/* control register */
#define CC770_CTR_CCE			0x40
#define CC770_CTR_EAF			0x20
#define CC770_CTR_EIE			0x08
#define CC770_CTR_SIE			0x04
#define CC770_CTR_IE			0x02
#define CC770_CTR_INIT			0x01

#define CC770_INIT_MODE			0x01

/* status register */
#define CC770_SR_BOFF			0x80
#define CC770_SR_WARN			0x40
#define CC770_SR_WAKEUP			0x20
#define CC770_SR_RXOK			0x10
#define CC770_SR_TXOK			0x08
#define CC770_SR_LEC			0x07

/* CPU Interface Register */
#define CC770_IFR_RSTST			0x80
#define CC770_IFR_DSC			0x40	// Divide System Clock
#define CC770_IFR_DMC			0x20	// Divide Memory Clock
#define CC770_IFR_PWD			0x10
#define CC770_IFR_SLEEP			0x08
#define CC770_IFR_MUX			0x04
#define CC770_IFR_STEN			0x02	// Clockout Stretch Enable
#define CC770_IFR_CEN			0x01	// Clockout Enable
/* Message 15 Special Mask Bits */

#define CC770_M15_MDIR			0x04
#define CC770_M15_MXTD			0x07

#define CC770_IR_MSG15			0x02
#define CC770_IR_MSG1			0x03
#define CC770_IR_MSG2			0x04
#define CC770_IR_MSG3			0x05
#define CC770_IR_MSG4			0x06
#define CC770_IR_MSG5			0x07
#define CC770_IR_MSG6			0x08
#define CC770_IR_MSG7			0x09
#define CC770_IR_MSG8			0x0A
#define CC770_IR_MSG9			0x0B
#define CC770_IR_MSG10			0x0C
#define CC770_IR_MSG11			0x0D
#define CC770_IR_MSG12			0x0E
#define CC770_IR_MSG13			0x0F
#define CC770_IR_MSG14			0x10

/* Clock out Register */
#define CC770_CLK_SL1			0x20
#define CC770_CLK_SL0			0x10
#define CC770_CLK_DIV			0x0F

/* Message Control Registers */
/* Control 0 Masks */
#define CC770_MCTL_VALID		0xC0	// message object valid or not
#define CC770_MCTL_TXEN			0x30	// generate an interrupt after tx
#define CC770_MCTL_RXEN			0x0C	// generate an interrupt after rx
#define CC770_MCTL_INT			0x03	// Interrupt pending
/* Control 1 Masks */
#define CC770_MCTL_RMT			0xC0	// Remote request pending
#define CC770_MCTL_TXR			0x30	// Transmit request pending
#define CC770_MCTL_LOST			0x0C	// A Message got lost ( overwritten )
#define CC770_MCTL_NEW			0x03	// new data has been written
/* bit value definitions for control registers */
#define CC770_MCTL_SET			0x02	// set a bit in control registers
#define CC770_MCTL_RESET		0x01	// reset a bit in control registers
#define CC770_MCTL_HOLD			0x03	// hold the value in control registers
/* message configuration register masks*/
#define CC770_MCFG_DLC			0xF0	// Data length code of the message
#define CC770_MCFG_DIR			0x08	// direction of the message
#define CC770_MCFG_XTD			0x04	// extended flag of the message

/* CC770 functions ****************************************************/
static inline void cc770_reset(CPC_CHAN_T * chan)
{
	chan->write_byte(chan, CC770_CONTROL, CC770_CTR_INIT);

	/* Be silent */
	chan->write_byte(chan, CC770_BUS_CONF, 0x40);
}

static inline int cc770_interrupt(CPC_CHAN_T *chan, struct timeval now)
{
	volatile unsigned char interrupt_source;

	interrupt_source = chan->read_byte(chan, CC770_INT_REG);

	info("INT_REG %02X", interrupt_source);

	/* no interrupt occured by cc770 */
	if (!interrupt_source || (chan->read_byte(chan, CC770_CONTROL)
			& CC770_INIT_MODE)){
		printk("Not me\n");
		return 0;
	}

	chan->handledIrqs++;

	switch (interrupt_source) {
	case 1:
		if (!chan->ovr.event) {
			/* Check whether we have a free slot in the circular buffer */
			if (!(IsBufferFull(chan)) && (chan->cpcCtrlCANState
					& (CONTR_CONT_ON | CONTR_SING_ON))) {
				CPC_MSG_T *recmsg = &chan->buf[chan->iidx];

				recmsg->type = CPC_MSG_T_CANSTATE;
				recmsg->length = 1;
				recmsg->ts_sec = now.tv_sec;
				recmsg->ts_nsec = now.tv_usec * 1000L;
				recmsg->msgid = 0;
				recmsg->msg.canstate = chan->read_byte(chan, CC770_STATUS) & 0xc0;

				chan->WnR = 0;
				chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

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

		chan->cpcCtrlCANState &= ~CONTR_SING_ON; /* clear single */
		break;

	case 2:
		if (!chan->ovr.event) {
			/* Check whether we have a free slot in the circular buffer */
			if (!(IsBufferFull(chan)) && (chan->cpcCtrlCANMessage
					& (CONTR_CONT_ON | CONTR_SING_ON))) {
				CPC_MSG_T *recmsg = &chan->buf[chan->iidx];

				recmsg->ts_sec = now.tv_sec;
				recmsg->ts_nsec = now.tv_usec * 1000;

				recmsg->msg.canmsg.length =
					chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_CONFIG) >> 4;

				recmsg->msg.canmsg.msg[0] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA0);
				recmsg->msg.canmsg.msg[1] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA1);
				recmsg->msg.canmsg.msg[2] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA2);
				recmsg->msg.canmsg.msg[3] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA3);
				recmsg->msg.canmsg.msg[4] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA4);
				recmsg->msg.canmsg.msg[5] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA5);
				recmsg->msg.canmsg.msg[6] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA6);
				recmsg->msg.canmsg.msg[7] = chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_DATA7);

				if(chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_CONFIG) & CC770_MCFG_XTD) {
					recmsg->type = CPC_MSG_T_XCAN;
					chan->recvExtCan++;
				} else {
					recmsg->type = CPC_MSG_T_CAN;
					recmsg->msg.canmsg.id  = ((unsigned long)chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_ARB0) << 3);
					recmsg->msg.canmsg.id |= ((unsigned long)chan->read_byte(chan, CC770_MESSAGE15 + CC770_MSG_ARB1) >> 5);
					chan->recvStdCan++;
				}

				chan->WnR = 0;
				chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;
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

		chan->write_byte(chan, (CC770_MESSAGE15 + CC770_MSG_CONTROL0), 0xfd);
		chan->write_byte(chan, (CC770_MESSAGE15 + CC770_MSG_CONTROL1), 0x55);

		chan->cpcCtrlCANMessage &= ~CONTR_SING_ON; /* clear single */

		wake_up_interruptible(chan->CPCWait_q);
		break;

	case 3:{

		chan->write_byte(chan, (CC770_MESSAGE15 + CC770_MSG_CONTROL0), 0x55);
		chan->write_byte(chan, (CC770_MESSAGE15 + CC770_MSG_CONTROL1), 0x55);

	}break;

	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		chan->write_byte(chan, (CC770_OFFSET + ((interrupt_source - 3) * 0x10)), 0xfd);

		// Other message object, probably Tx interrupt
		wake_up_interruptible(chan->CPCWait_q);
	}

	return 1;
}

/******************************************************************************************/
/* Transmit Standard-Frame-Format-DATA-Message */
static inline int cc770_send_can(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	if (msg->id > 0x7ff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (chan->read_byte(chan, CC770_MESSAGE1 + CC770_MSG_CONTROL1) & 0x20) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	info("INTPEND %02X", chan->read_byte(chan, CC770_INT_REG));

	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONTROL1), 0x08); /* set CPUUpd */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB0), msg->id >> 3);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB1), msg->id << 5);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB2), 0x00);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB3), 0x00);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA0), msg->msg[0]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA1), msg->msg[1]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA2), msg->msg[2]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA3), msg->msg[3]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA4), msg->msg[4]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA5), msg->msg[5]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA6), msg->msg[6]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA7), msg->msg[7]); /* write data to msg[0] */

	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONFIG), (msg->length
			<< 4) | CC770_MCFG_DIR); /* transmit object */

	/* object is valid */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONTROL0),
			0x80 | 0x20 | 0x05);

	/* request transmission and set newdat avail */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONTROL1),
			0x20 | 0x04 | 0x02);

	info("Config %02X", chan->read_byte(chan, CC770_MESSAGE1 + CC770_MSG_CONFIG));
	info("Control1 %02X", chan->read_byte(chan, CC770_MESSAGE1 + CC770_MSG_CONTROL0));
	info("Control2 %02X", chan->read_byte(chan, CC770_MESSAGE1 + CC770_MSG_CONTROL1));

	chan->sentStdCan++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

/******************************************************************************************/
/* Transmit Standard-Frame-Format-REMOTE-Message */
static inline int cc770_send_rtr(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	return CPC_ERR_SERVICE_NOT_SUPPORTED;
}

/******************************************************************************************/
/* Transmit Extended-Frame-Format-DATA-Message       */
static inline int cc770_send_xcan(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	if (msg->id > 0x1fffffff)
		return CPC_ERR_CAN_WRONG_ID;
	if (msg->length > 8)
		return CPC_ERR_CAN_WRONG_LENGTH;

	if (chan->read_byte(chan, CC770_MESSAGE1 + CC770_MSG_CONTROL1)
			& CC770_MCTL_TXR) {
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;
	}

	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONTROL1), 0x08); /* set CPUUpd */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB0), msg->id >> 21);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB1), msg->id >> 13);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB2), msg->id >> 5);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_ARB3), msg->id << 3);
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA0), msg->msg[0]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA1), msg->msg[1]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA2), msg->msg[2]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA3), msg->msg[3]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA4), msg->msg[4]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA5), msg->msg[5]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA6), msg->msg[6]); /* write data to msg[0] */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_DATA7), msg->msg[7]); /* write data to msg[0] */

	/* transmit object */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONFIG), (msg->length
			<< 4) | CC770_MCFG_DIR | CC770_MCFG_XTD);

	/* object is valid */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONTROL0),
			CC770_MCTL_VALID | CC770_MCTL_TXEN);

	/* request transmission and set newdat avail */
	chan->write_byte(chan, (CC770_MESSAGE1 + CC770_MSG_CONTROL1),
			CC770_MCTL_TXR | CC770_MCTL_NEW);

	chan->sentExtCan++;

	return CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T);
}

/******************************************************************************************/
/* Transmit Extended-Frame-Format-REMOTE-Message */
static inline int cc770_send_xrtr(CPC_CHAN_T * chan, CPC_CAN_MSG_T * msg)
{
	return CPC_ERR_SERVICE_NOT_SUPPORTED;
}

/**************************************************************************************/
static inline int cc770_get_canstate(CPC_CHAN_T * chan)
{
	if (IsBufferFull(chan))
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;

	chan->buf[chan->iidx].type = CPC_MSG_T_CANSTATE;
	chan->buf[chan->iidx].length = 1;
	chan->buf[chan->iidx].msg.canstate = chan->read_byte(chan, CC770_STATUS);

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 1;
}

/****************************************************************************/
/* read CAN-Controller parameter */
static inline int cc770_inq_params(CPC_CHAN_T * chan)
{
	u8 control;

	CPC_SJA1000_PARAMS_T *can =
			&chan->buf[chan->iidx].msg.canparams.cc_params.sja1000;

	if (IsBufferFull(chan))
		return CPC_ERR_CAN_NO_TRANSMIT_BUF;

	/* safe mode */
	control = chan->read_byte(chan, CC770_CONTROL);

	chan->write_byte(chan, CC770_CONTROL, CC770_INIT_MODE); /* Clear RESET MODE, set Filter Mode   */
	udelay(10);

	if (!(chan->read_byte(chan, CC770_CONTROL) & CC770_INIT_MODE)) /* check, if controller is in init mode */
		return CPC_ERR_NO_RESET_MODE;

	chan->buf[chan->iidx].type = CPC_MSG_T_CAN_PRMS;
	chan->buf[chan->iidx].length = sizeof(CPC_CAN_PARAMS_T);
	chan->buf[chan->iidx].msg.canparams.cc_type = SJA1000;

	can->btr0 = chan->read_byte(chan, CC770_BTR0);
	can->btr1 = chan->read_byte(chan, CC770_BTR1);

	/* Restore control */
	chan->write_byte(chan, CC770_CONTROL, control);

	chan->WnR = 0;
	chan->iidx = (chan->iidx + 1) % CPC_MSG_BUF_CNT;

	return CPCMSG_HEADER_LEN + 1;
}

void dump_cc770(CPC_CHAN_T *chan)
{
	unsigned int i=0;

	printk( "\n%2.2X: ", i);
	for(; i<0x10; i++)
		printk( "%2.2X ", chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x20; i++)
		printk( "%2.2X ", chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x30; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x40; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x50; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x60; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x70; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x80; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x90; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0xa0; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0xb0; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0xc0; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0xd0; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0xe0; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0xf0; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n%2.2X: ", i);
	for(; i<0x100; i++)
		printk( "%2.2X ",chan->read_byte(chan, (CC770_CONTROL+i)));

	printk( "\n");
}

static inline int cc770_init_contr(CPC_CHAN_T *chan, CPC_SJA1000_PARAMS_T *can)
{
	int i;

	cc770_reset(chan);

	udelay(5);

	/* check, if controller is in init mode */
	if (!(chan->read_byte(chan, CC770_CONTROL) & CC770_CTR_INIT))
		return CPC_ERR_NO_RESET_MODE;

	/* Enable configuration register access and set init state */
	chan->write_byte(chan, CC770_CONTROL, 0x61);

	chan->write_byte(chan, CC770_P2_CONF, 0); /* Port 2 is input */

	/* Set CPU-interface register to SCLK = XTAL/2 and clockout enabled */
	chan->write_byte(chan, CC770_CPU_IFACE, 0x61);

	chan->write_byte(chan, CC770_BUS_CONF, 0x40); /* Set bus-configuration register */

	chan->write_byte(chan, CC770_BTR0, can->btr0); /* Set the bit timing register 0 */
	chan->write_byte(chan, CC770_BTR1, can->btr1); /* Set the bit timing register 1 */

	chan->write_byte(chan, CC770_GMSTD_1, 0xff); /* All bits valid */
	chan->write_byte(chan, CC770_GMSTD_2, 0xff); /* All bits valid */

	chan->write_byte(chan, CC770_GMEXT_1, 0xff); /* All bits valid */
	chan->write_byte(chan, CC770_GMEXT_2, 0xff); /* All bits valid */
	chan->write_byte(chan, CC770_GMEXT_3, 0xff); /* All bits valid */
	chan->write_byte(chan, CC770_GMEXT_4, 0xff); /* All bits valid */

	chan->write_byte(chan, CC770_MSG15_1, 0x00);
	chan->write_byte(chan, CC770_MSG15_2, 0x00);
	chan->write_byte(chan, CC770_MSG15_3, 0x00);
	chan->write_byte(chan, CC770_MSG15_4, 0x00);

	/* Clear all bits in the control registers */
	for (i = 1; i <= 15; i++)
		chan->write_byte(chan, ((CC770_OFFSET + 0x00) + 0x10 * i), 0x55);

	for (i = 1; i <= 15; i++)
		chan->write_byte(chan, ((CC770_OFFSET + 0x00) + 1 + 0x10 * i), 0x55);

	/* Prepare receiving object */
	chan->write_byte(chan, (CC770_MESSAGE15 + CC770_MSG_CONFIG), 0x80);
	chan->write_byte(chan, (CC770_MESSAGE15 + CC770_MSG_CONTROL0), 0x99);

	/* Reset init mode and enable interrupts */
	chan->write_byte(chan, CC770_CONTROL, 0x0A);

	dump_cc770(chan);

	return CPCMSG_HEADER_LEN + sizeof(CPC_SJA1000_PARAMS_T);
}

#endif
