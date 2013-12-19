/*
 * Serial client implementation CPCLIB
 *
 * Copyright (C) 2000-2010 EMS Dr. Thomas Wuensche
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <ctype.h>

// includes for serial bluetooth interface
#include <sys/types.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "../include/cpc.h"
#include "../include/cpclib.h"

#include "../include/cpc_common.h"
#include "../include/serial.h"

#define TRUE 1
#define FALSE 0

char * get_cpc_cmd_str(unsigned char type);

int setupSerialConnection(int handle, char *ucChannel)
{
	struct sockaddr_rc loc_dev = { 0 };
	int sfd, status;
	struct termios newmios;
	int serialBaudrate = B230400;
	
	sfd = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	if (sfd < 0) {
		perror("Can't open RFCOMM control socket\n");
		return -1;
	}
	
	fcntl(sfd, F_SETFL, O_RDWR | O_NOCTTY);
	
	printf("setupSerialConnection called %d\n", sfd);
	 
	loc_dev.rc_family = AF_BLUETOOTH;
	loc_dev.rc_channel = (uint8_t) 1;
	str2ba(ucChannel, &loc_dev.rc_bdaddr);

	status = connect(sfd, (struct sockaddr *)&loc_dev, sizeof(loc_dev));

	if(status == -1) {
		printf("%d: %s\n", status, strerror(errno));
		return -1;
	}

	tcgetattr(sfd, &CPCLibParams[handle].interface.serial.oldtermios);         // alte Einstellungen sichern

	bzero(&newmios, sizeof(newmios));
	newmios.c_cflag = CS8 | CLOCAL | CREAD ;   // CRTSCTS |
	newmios.c_iflag = IGNPAR | IGNBRK;
	newmios.c_oflag = 0;
	newmios.c_lflag = 0;
	newmios.c_cc[VTIME] = 0;        // keine Timeouts

	newmios.c_cc[VMIN] = 0;         // Zeichenanzahl auf die gerwartet werden soll
	cfsetspeed(&newmios,serialBaudrate) ;

	tcflush(sfd, TCIFLUSH);
	tcsetattr(sfd, TCSANOW, &newmios);

	fcntl(sfd, F_SETFL, O_NONBLOCK);

	printf("Channel opened\n");

	CPCLibParams[handle].interface.serial.fd = sfd;

	memcpy(CPCLibParams[handle].name, "CPC-BLUE/M3", sizeof("CPC-BLUE/M3"));

	CPCLibParams[handle].read = Serial_Read;
	CPCLibParams[handle].write = Serial_Write;

	// Initialize write-related variables (were static before)
	CPCLibParams[handle].interface.serial.write.sendBufFree  = TRUE;
	CPCLibParams[handle].interface.serial.write.bytesWritten = 0;

	// Initialize read-related variables (were static before)
	CPCLibParams[handle].interface.serial.read.i			= 0;
	CPCLibParams[handle].interface.serial.read.length		= 0;
	CPCLibParams[handle].interface.serial.read.state		= S_IDLE;
	CPCLibParams[handle].interface.serial.read.ready		= TRUE;
	CPCLibParams[handle].interface.serial.read.esc_state 	= ESC_NONE;

	return sfd;

}

/****************************************************************************/
unsigned short CalculateCheckSum(CPC_MSG_T * msg)
{
	unsigned short 	i, tmp = msg->type  + 
				msg->length + 
				msg->msgid  +
				msg->ts_sec + 
				msg->ts_nsec;

	for(i=0; i<msg->length; i++)
		tmp += msg->msg.generic[i];
		
	return tmp;
}

#define addToBuf(a)  	if((a) > 0xfb) {		\
				*ptr++ = CW_ESCAPE;	\
				*ptr++ =((a) & 0x7f);	\
			} else { 			\
				*ptr++ = (a);		\
			}

/******************************************************************************
* Function.....: Serial_Write																								 *
*																																						 *
* Task.........: sends a CPC_MSG via Serial																   *
*																																						 *
* Parameters...: 									  *
*																																						 *
* Return values: 0: controller was initialzed																 *
*								1: init mode could not be set																*
******************************************************************************/
ssize_t Serial_Write(int fd, const void *buf, size_t count)
{
	unsigned char *ptr;
	unsigned int i;
	ssize_t wb;
	unsigned int len;
	unsigned short tmp;

	struct serial_write *wr = NULL;

	CPC_MSG_T *cpc = (CPC_MSG_T *) buf;

	for(i = 0; i < MAX_CHANNEL_ENTRIES; i++) {
		if(fd == CPCLibParams[i].interface.serial.fd) {
			wr = &CPCLibParams[i].interface.serial.write;
			break;
		}
	}

	// Could not corelate fd to handle
	if(i == MAX_CHANNEL_ENTRIES || !wr)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

//	printf("Write called %s, %d times\n", get_cpc_cmd_str(((char *)buf)[0]), callCnt++);

	/* check if message is pending */
	if (wr->sendBufFree == FALSE) {
		len = strlen((char *)wr->sendBuf);

		wb = write(fd, &wr->sendBuf[wr->bytesWritten], len - wr->bytesWritten);

		if (wb >= 0) {

			wr->bytesWritten += wb;

			if (wr->bytesWritten == len) {	/* message complete */
				wr->sendBufFree = TRUE;
			} else {			/* message incomplete */
				return CPC_ERR_CAN_NO_TRANSMIT_BUF;
			}
		} else {
		  
			if (BE_INFORMATIVE) {
						perror("MESSAGE[CPCLIB->Serial_Write()]:");
			}
			return CPC_ERR_NO_INTERFACE_PRESENT;
		}
	}

	ptr = wr->sendBuf;
		
	tmp = CalculateCheckSum(cpc);

	*ptr++ = (unsigned char)CW_BEGINOFDATA;
 	addToBuf(cpc->type);
	addToBuf(cpc->length);
	addToBuf(cpc->msgid);

	addToBuf((unsigned char)(cpc->ts_sec>>0));
	addToBuf((unsigned char)(cpc->ts_sec>>8));
	addToBuf((unsigned char)(cpc->ts_sec>>16));
	addToBuf((unsigned char)(cpc->ts_sec>>24));

	addToBuf((unsigned char)(cpc->ts_sec>>0));
	addToBuf((unsigned char)(cpc->ts_sec>>8));
	addToBuf((unsigned char)(cpc->ts_sec>>16));
	addToBuf((unsigned char)(cpc->ts_sec>>24));

	for(i=0; i < cpc->length; i++){
 	  addToBuf(cpc->msg.generic[i]);
 	}

	addToBuf((unsigned char)(tmp>>0));
	addToBuf((unsigned char)(tmp>>8));

 	*ptr++ = (unsigned char)CW_ENDOFDATA;

	len = (unsigned char)(ptr - wr->sendBuf);
	
#if 0
	unsigned int k;
	printf("write: ");
	for(k=0; k<len;k++){
		if(isprint(wr->sendBuf[k]))
			printf("%c ", wr->sendBuf[k]);
		else
			printf("%2.2X ", wr->sendBuf[k]);
			if(wr->sendBuf[k] == CW_ENDOFDATA)
			printf("\n");
	}
#endif

	wb = write(fd, wr->sendBuf, len);

	if (wb < 0) {		/* error? */

		 	perror("Serial Write");

		if (BE_INFORMATIVE) {
			perror("MESSAGE[CPCLIB->Serial_Write()]:");
		}
		return CPC_ERR_NO_INTERFACE_PRESENT;
		  
	} else if (wb != len && wb >= 0) {	/* not all data could be sent */
		wr->sendBufFree = FALSE;
		wr->bytesWritten = wb;
	}
	return count;
}

/******************************************************************************
* Function.....: Serial_Read																								  *
*																																						 *
* Task.........: reads a CPC_MSG via Serial																   *
*																																						 *
* Parameters...:																														  *
*																																						 *
* Return values: <0: error																										*
*								sizeof(CPC_MSG_T on success																  *
******************************************************************************/
ssize_t Serial_Read(int fd, void *pCpcMsg, size_t count)
{
	int result, k;
	struct serial_read *rd = NULL;

	CPC_MSG_T *cpc = (CPC_MSG_T *) pCpcMsg;
		unsigned short chktmp;

	for(k = 0; k < MAX_CHANNEL_ENTRIES; k++) {
		if(fd == CPCLibParams[k].interface.serial.fd) {
			rd = &CPCLibParams[k].interface.serial.read;
			break;
		}
	}

	// Could not corelate fd to handle
	if(k == MAX_CHANNEL_ENTRIES || !rd){
		return -1;
  }

	/* check, if all messages has been processed */
	if (rd->ready == TRUE) {
		memset(&rd->recvbuf, 0, sizeof(rd->recvbuf)); 
		errno  = 0;
		result = 0;

		result = read(fd, rd->recvbuf, sizeof(rd->recvbuf)); 

		if (errno == ECONNRESET) {
			perror("\nClient has disconnected\n");
			return CPC_ERR_NO_INTERFACE_PRESENT;
		} else {
			/* check if there is data in the buffer */
			if (result > 0) {
				rd->length = result;
				rd->i = 0;
				rd->ready = FALSE;
			} else {
				return 0; // nothing read from interface
			}
		}
	}

	/* the "start" of the rd->state-machine */
	while (rd->i < rd->length) {
		
		unsigned char c = rd->recvbuf[rd->i];
		  rd->i++;
		// Process escape codes
		switch(rd->esc_state) {
			case  ESC_NONE:
				if(IS_ESCAPE(c)) {		// Received escape?
					rd->esc_state = ESC_ESC;
					continue;
				} else { // No escape code
					if(c >= 0xfc)				// Other control code ?
						rd->inchar = 0x100 | c;
					else {
						rd->inchar = c;
					}
				}
				break;

			case  ESC_ESC:
				if(c >= 0xfc)				  // Unexpected control code
					rd->inchar = 0x100 | c;
				else
					rd->inchar = REMOVE_ESCAPE(c);
				rd->esc_state = ESC_NONE;
				break;
		}
		  
		/* Here the received character without ESC is processed */
		switch(rd->state) {
			case S_DAT:  // this case first, because it's most often
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->msg.generic[rd->cnt++] = CONTENT(rd->inchar);
						if(rd->cnt == cpc->length)
							rd->state = S_CHK1;
						  			break;
				}
				break;
			case S_IDLE:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->state = S_TYPE;
						rd->errorcode = E_NONE;
						break;
					default:
						rd->errorcode |= E_FORMAT;
						break;
				}
				break;
			case S_TYPE:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						rd->errorcode = E_NONE;
						cpc->type = CONTENT(rd->inchar);
						rd->state = S_LEN;
						break;
				}
				break;
			case S_LEN:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->length = CONTENT(rd->inchar);
						rd->cnt   = 0;
						rd->state = S_MSGID;
						break;
				}
				break;
			case S_MSGID:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->msgid = CONTENT(rd->inchar);
						rd->state = S_TSSEC1;
						break;
				}
				break;
			case S_TSSEC1:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_sec = CONTENT(rd->inchar);
						rd->state = S_TSSEC2;
						break;
				}
				break;
			case S_TSSEC2:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_sec |= ((unsigned int)CONTENT(rd->inchar)<<8);
						rd->state = S_TSSEC3;
						break;
				}
				break;
			case S_TSSEC3:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_sec |= ((unsigned int)CONTENT(rd->inchar)<<16);
						rd->state = S_TSSEC4;
						break;
				}
				break;
			case S_TSSEC4:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_sec |= ((unsigned int)CONTENT(rd->inchar)<<24);
						rd->state = S_TSNSEC1;
						break;
				}
				break;
			case S_TSNSEC1:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_nsec = CONTENT(rd->inchar);
						rd->state = S_TSNSEC2;
						break;
				}
				break;
			case S_TSNSEC2:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_nsec |= ((unsigned int)CONTENT(rd->inchar)<<8);
						rd->state = S_TSNSEC3;
						break;
				}
				break;
			case S_TSNSEC3:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_nsec |= ((unsigned int)CONTENT(rd->inchar)<<16);
						rd->state = S_TSNSEC4;
						break;
				}
				break;
			case S_TSNSEC4:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						cpc->ts_nsec |= ((unsigned int)CONTENT(rd->inchar)<<24);
						rd->chk = 0;
						rd->state = S_DAT;
						break;
				}
				break;
			case S_CHK1:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						rd->chk   = CONTENT(rd->inchar);
						rd->state = S_CHK2;
						break;
				}
				break;
			case S_CHK2:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_IDLE;
						break;
					default:
						rd->chk |= (unsigned short)(CONTENT(rd->inchar))<<8;
						if((chktmp = CalculateCheckSum(cpc)) != rd->chk)
							rd->errorcode |= E_CHECKSUM;
						rd->state = S_END;
						break;
				}
				break;
			case S_END:
				switch(rd->inchar) {
					case CW_BEGINOFDATA:
						rd->errorcode |= E_FORMAT;
						rd->state = S_TYPE;
						break;
					case CW_ENDOFDATA:
						rd->state = S_IDLE;
						if(!rd->errorcode)
							return sizeof(CPC_MSG_T);
						else
							return -2;
						break;
					default:
						rd->errorcode |= E_LENGTH;
						break;
				}
				break; 
		} // switch(rd->state) 
	} // end state machine
	
	rd->i = 0;
	rd->length = 0;
	rd->recvbuf[0] = '\0';
	rd->ready = TRUE;

	return 0;
}

char * cpc_cmd_str[256] = { "None",
														"CPC_CMD_T_CAN", //									1
                            "None",
														"CPC_CMD_T_CONTROL", //             3
                            "None",
                            "None",
														"CPC_CMD_T_CAN_PRMS", //            6
                            "None",
														"CPC_CMD_T_CLEAR_MSG_QUEUE", //     8
                            "None",
                            "None",
														"CPC_CMD_T_INQ_CAN_PARMS", //      11
														"CPC_CMD_T_FILTER_PRMS", //        12
														"CPC_CMD_T_RTR", //                13
														"CPC_CMD_T_CANSTATE", //           14
														"CPC_CMD_T_XCAN", //               15
														"CPC_CMD_T_XRTR", //               16
														"CPC_CMD_T_RESET", //              17
														"CPC_CMD_T_INQ_INFO", //           18
														"CPC_CMD_T_OPEN_CHAN", //          19
														"CPC_CMD_T_CLOSE_CHAN", //         20
														"CPC_CMD_T_INQ_MSG_QUEUE_CNT", //  21
                            "None",
                            "None",
                            "None",
														"CPC_CMD_T_INQ_ERR_COUNTER", //    25
                            "None",
                            "None",
														"CPC_CMD_T_CLEAR_CMD_QUEUE", //    28
                            "None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
														"CPC_CMD_T_FIRMWARE", //          100
														"CPC_CMD_T_USB_RESET", //         101
														"CPC_CMD_T_WAIT_NOTIFY", //       102
														"CPC_CMD_T_WAIT_SETUP", //        103
                            "None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
														"CPC_CMD_T_CAN_EXIT", //          200
                            "None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None","None","None","None","None","None",
                            "None","None","None","None","None",
														"CPC_CMD_T_ABORT", //             255
                           };
                           
char * get_cpc_cmd_str(unsigned char type){

	return cpc_cmd_str[type];
  
}
