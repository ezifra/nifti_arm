/*
 * Sample application using CPCLIB
 *
 * Copyright (C) 2000-2008 EMS Dr. Thomas Wuensche
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#define PRG_VERSION "2.10"

#include <sys/types.h>
#include <sys/dir.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/file.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>

#include "../include/cpc.h"
#include "../include/cpclib.h"
#include "kbhit.h"
#include "cpctool.h"

/* CPCHandler */
void CPCPrintCANMessage(int handle, const CPC_MSG_T * cpcmsg);
void CPCPrintCPCMessage(int handle, const CPC_MSG_T * cpcmsg);
void CPCPrintInfo(int handle, const CPC_MSG_T * cpcmsg);
void CPCPrintCANState(int handle, const CPC_MSG_T * cpcmsg);
void CPCPrintCANError(int handle, const CPC_MSG_T * cpcmsg);
void CPCPrintCANParams(int handle, const CPC_MSG_T * cpcmsg);
void CPCCANMsgCounter(int handle, const CPC_MSG_T * cpcmsg);

void CPCPrintCANSendMessage(int handle, unsigned char type,
		const CPC_CAN_MSG_T * msg);

FILE * logFile;

unsigned int msgRxCnt = 0;
unsigned int rtrRxCnt = 0;
unsigned int msgTxCnt = 0;
unsigned int rtrTxCnt = 0;
unsigned int msgXRxCnt = 0;
unsigned int rtrXRxCnt = 0;
unsigned int msgXTxCnt = 0;
unsigned int rtrXTxCnt = 0;

/**
 * Main loop
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
	int zchn;
	CPC_INIT_PARAMS_T * CPCInitParamsPtr;
	unsigned char sendIdx = 0;
	char interface[32];
	unsigned char btr0, btr1;
	int handle;
	int baudrate = -1;

	extern char * optarg;
	char * options;
	int c;

	/* Some CAN Messages to get the Manufacturer-, Product-, Serial- and Versionstring
	 * of a EMS-CST CAN module.
	 */
	CPC_CAN_MSG_T smsg[8] = { { 0x1f1L, 8, { 1, 2, 3, 4, 5, 6, 7, 8 } }, { 20L,
			2, { 128, 0, 0, 0, 0, 0, 0, 0 } }, { 2021L, 8, { 4, 1, 0, 0, 0, 0,
			0, 0 } }, { 2021L, 1, { 0x24 } }, { 2021L, 1, { 0x25 } }, { 2021L,
			1, { 0x26 } }, { 2021L, 1, { 0x83 } }, { 2021L, 8, { 4, 0, 0, 0, 0,
			0, 0, 0 } } };

	CPC_CAN_MSG_T xmsg = { 0xB43ffeL, 8, { 1, 2, 3, 4, 5, 6, 7, 8 } };
	CPC_MSG_T *pMsg;

	options = "b:i:vh";

#ifdef CONFIG_LOGGING
	if((logFile = fopen("log.dat", "wt")) != NULL)
	printf("Logfile log.dat opened\n");
#endif

	strcpy(interface, "CHAN00");

	/* processing commandline ********************************/
	while ((c = getopt(argc, argv, options)) != EOF) {
		switch (c) {
		case 'h':
			printf("Usage: cpctool -b<baudrate> -i<interface>\n");
			printf("\nIf no interface is specified default is CHAN00.\n");
			exit(0);
			break;

		case 'v':
			printf(
					"cpctool v%s - Copyright 2000-2005 EMS Dr. Thomas Wuensche\n",
					PRG_VERSION);
			exit(0);
			break;

		case 'i': {
			unsigned char len = strlen(optarg);
			if (len > sizeof(interface)) {
				len = sizeof(interface) - 1;
				printf("device path length too long\n");
			}
			interface[len] = '\0';
			strncpy(interface, optarg, len);
		}
			break;

		case 'b': {
			baudrate = atoi(optarg);

			switch (baudrate) {
			case 1000:
				btr0 = 0x00;
				btr1 = 0x14;
				break;
			case 800:
				btr0 = 0x00;
				btr1 = 0x16;
				break;
			case 500:
				btr0 = 0x00;
				btr1 = 0x1c;
				break;
			case 250:
				btr0 = 0x01;
				btr1 = 0x1c;
				break;
			case 125:
				btr0 = 0x03;
				btr1 = 0x1c;
				break;
			case 100:
				btr0 = 0x04;
				btr1 = 0x1c;
				break;
			case 50:
				btr0 = 0x09;
				btr1 = 0x1c;
				break;
			case 25:
				btr0 = 0x13;
				btr1 = 0x1c;
				break;
			case 20:
				btr0 = 0x18;
				btr1 = 0x1c;
				break;
			case 10:
				btr0 = 0x31;
				btr1 = 0x1c;
				break;

			default:
				printf("ERROR: Unsupported Baudrate %d!\n", baudrate);
				exit(-1);

			}
			printf("Baudrate    : %dkBaud\n", baudrate);
			printf("BTR0 set to : %2.2Xh\n", btr0);
			printf("BTR1 set to : %2.2Xh\n", btr1);
		}
			break;
		}
	}

	if (baudrate == -1) {
		printf("ERROR: No Baudrate specified\n");
		printf("Try cpctool -h for usage information\n");
		exit(-1);
	}

	printf("\n------------- cpctool %s -------------\n", PRG_VERSION);
	printf("build with lib version %s \n\n", CPC_GetInfo(0,
			CPC_INFOMSG_T_LIBRARY, CPC_INFOMSG_T_VERSION));

	if ((handle = CPC_OpenChannel(interface)) < 0) {
		printf("ERROR: %s\n", CPC_DecodeErrorMsg(handle));
		exit(1);
	}

	printf("%s is CAN interface -> handle %d\n", interface, handle);

	// adding handlers, which will do the application job
	CPC_AddHandler(handle, CPCCANMsgCounter);
	CPC_AddHandler(handle, CPCPrintCANMessage);
	CPC_AddHandler(handle, CPCPrintInfo);
	CPC_AddHandler(handle, CPCPrintCANState);
	CPC_AddHandler(handle, CPCPrintCANError);
	CPC_AddHandler(handle, CPCPrintCANParams);

	printf("Press: <s> : to send a standard identifier data frame\n");
	printf("       <r> : to send a standard identifier remote frame\n");
	printf("       <x> : to send a extended identifier data frame\n");
	printf("       <y> : to send a extended identifier remote frame\n");
	printf("       <i> : to input a standard identifier message and send it\n");
	printf("       <e> : to inquire the CAN controller status register\n");
	printf("       <p> : to inquire the CAN controller parameter\n");
	printf("       <c> : to toggle ON/OFF of receiveing CAN messages\n");
	printf("       <q> : to quit programm\n");
	printf("\n");

	/* This sets up the parameters used to initialize the CAN controller */
	printf("Initializing CAN-Controller ... ");

	CPCInitParamsPtr = CPC_GetInitParamsPtr(handle);
	CPCInitParamsPtr->canparams.cc_type = SJA1000;
	CPCInitParamsPtr->canparams.cc_params.sja1000.btr0 = btr0;
	CPCInitParamsPtr->canparams.cc_params.sja1000.btr1 = btr1;
	CPCInitParamsPtr->canparams.cc_params.sja1000.outp_contr = 0xda;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code0 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code1 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code2 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code3 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask0 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask1 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask2 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask3 = 0xff;
	CPCInitParamsPtr->canparams.cc_params.sja1000.mode = 0;

	CPC_CANInit(handle, 0);
	printf("Done!\n\n");

	/* inquire of version, serial and driver version string **/
	CPC_RequestInfo(handle, 0, CPC_INFOMSG_T_DRIVER, CPC_INFOMSG_T_VERSION);
	CPC_RequestInfo(handle, 0, CPC_INFOMSG_T_INTERFACE, CPC_INFOMSG_T_VERSION);
	CPC_RequestInfo(handle, 0, CPC_INFOMSG_T_INTERFACE, CPC_INFOMSG_T_SERIAL);

	printf(
			"Switch ON transimssion of CAN and CAN state messages from CPC to PC\n");

	/* switch on transmission of CAN messages from CPC to PC */
	CPC_Control(handle, CONTR_CAN_Message | CONTR_CONT_ON);
	/* switch on transmission of CAN state messages from CPC to PC */
	CPC_Control(handle, CONTR_CAN_State | CONTR_CONT_ON);

	kbdinit();

	/* this is the main loop *********************************/
	do {
		if (CPC_WaitForEvent(handle, 10, EVENT_READ) & EVENT_READ) {
			do {
				pMsg = CPC_Handle(handle);

				if (pMsg) {
					if (pMsg->type == CPC_MSG_T_DISCONNECTED) {
						printf("Device unplugged\n");
						zchn = 'q';
						break;
					}
				}
			} while (pMsg);

		}

		/* check keyboard **************************************/
		if (kbhit()) {
			zchn = readch();

			switch (zchn) {
			case 'c': { /* config CPC_Control*/
				int toggle = -1;
				printf("Receiving of CAN_Messages\n");
				printf("OFF    :  0\n");
				printf("ON     :  1\n");
				printf("SINGLE :  2\n");
				read(0, &toggle, 1);
				if (toggle >= 0) {
					switch (toggle) {
					case 0:
						CPC_Control(handle, CONTR_CAN_Message | CONTR_CONT_OFF);
						break;
					case 1:
						CPC_Control(handle, CONTR_CAN_Message | CONTR_CONT_ON);
						break;
					case 2:
						CPC_Control(handle, CONTR_CAN_Message | CONTR_SING_ON);
						break;
					}
				}
			}
				break;

			case 'e': { /* check CAN state*/
				printf("inquire CAN state ... ");
				CPC_RequestCANState(handle, 0);
				printf("done!\n");
			}
				break;

			case 'p': { /* check CAN state*/
				printf("inquire CAN parameter ... ");
				CPC_RequestCANParams(handle, 0);
				printf("done!\n");
			}
				break;

			case 's': { /* check, if transmit buffer can be accessed, to send new messages */
				if (CPC_WaitForEvent(handle, 10, EVENT_WRITE) & EVENT_WRITE) {
					if (!CPC_SendMsg(handle, 0, &smsg[sendIdx])) {
						msgTxCnt++;
						CPCPrintCANSendMessage(handle, CPC_CMD_T_CAN,
								&smsg[sendIdx]);
					} else
						printf("\n");

					sendIdx = (sendIdx + 1) % 7;
				}
			}
				break;

			case 't': { /*  */
				unsigned int i, sendCnt = 100, bufOccupied = 0;

				for (i = 0; i < sendCnt;) {
					signed int ret;
					if ((ret = CPC_SendMsg(handle, 0, &smsg[0])) >= 0) {
						smsg[0].msg[0]++;
						msgTxCnt++;
						i++;
					} else {

						if (ret == CPC_ERR_CAN_NO_TRANSMIT_BUF) {
							if (!bufOccupied)
								printf("ERROR: %s\n", CPC_DecodeErrorMsg(ret));
							bufOccupied++;
						} else {
							printf("ERROR: %s\n", CPC_DecodeErrorMsg(ret));
							break;
						}
					}
				}
				printf(
						"%d sent, %d attempts failed due to occupied transmit buffer\n",
						sendCnt, bufOccupied);

			}
				break;

			case 'r': { /* check, if transmit buffer can be accessed, to send new messages */
				if (CPC_WaitForEvent(handle, 10, EVENT_WRITE) & EVENT_WRITE) {
					printf("send std rtr ... ");
					CPC_SendRTR(handle, 0, &smsg[sendIdx]);
					printf("done!\n");
					rtrTxCnt++;
					sendIdx = (sendIdx + 1) % 6;
				}
			}
				break;

			case 'x': { /* check, if transmit buffer can be accessed, to send new messages */
				if (CPC_WaitForEvent(handle, 10, EVENT_WRITE) & EVENT_WRITE) {
					if (!CPC_SendXMsg(handle, 0, &xmsg)) {
						msgXTxCnt++;
						CPCPrintCANSendMessage(handle, CPC_CMD_T_XCAN, &xmsg);
						xmsg.id++;
					} else
						printf("\n");
				}
			}
				break;

			case 'y': { /* check, if transmit buffer can be accessed, to send new messages */
				if (CPC_WaitForEvent(handle, 10, EVENT_WRITE) & EVENT_WRITE) {
					printf("send xtd rtr ... ");
					CPC_SendXRTR(handle, 0, &xmsg);
					printf("done!\n");
					rtrXTxCnt++;
				}
			}
				break;

			case 'i': { /* Send a single CAN-message  */
				static CPC_CAN_MSG_T cmsg = { 0x00L, 0, { 0, 0, 0, 0, 0, 0, 0,
						0 } };

				printf("Input and send a single CAN message ");

				get_can_msg(&cmsg);

				if (ask("Send Messages as data frame?", ASK_YES)) {
					if (CPC_WaitForEvent(handle, 10, EVENT_WRITE) & EVENT_WRITE) {
						msgTxCnt++;
						CPC_SendMsg(handle, 0, &cmsg);
					}
				}/*if(data frame)*/
				else if (ask("Send Messages as remote frame?", ASK_NO)) {
					if (CPC_WaitForEvent(handle, 10, EVENT_WRITE) & EVENT_WRITE) {
						rtrTxCnt++;
						CPC_SendRTR(handle, 0, &cmsg);
					}
				}
			}
				break;
			}
		}
	} while (zchn != 'q');

	CPC_Control(handle, CONTR_CAN_Message | CONTR_CONT_OFF);

	printf("\nTRANSMIT:\n");
	printf("standard data frames   : %d\n", msgTxCnt);
	printf("standard remote frames : %d\n", rtrTxCnt);
	printf("extended data frames   : %d\n", msgXTxCnt);
	printf("extended remote frames : %d\n", rtrXTxCnt);
	printf("RECEIVE:\n");
	printf("standard data frames   : %d\n", msgRxCnt);
	printf("standard remote frames : %d\n", rtrRxCnt);
	printf("extended data frames   : %d\n", msgXRxCnt);
	printf("extended remote frames : %d\n", rtrXRxCnt);
	printf("\n");

	kbdexit();
	CPC_CloseChannel(handle);

	if (logFile != NULL)
		fclose(logFile);

	return 0;

}

/**
 * calculates difference of two timestamps
 * out = in2 - in1
 */
void diffTS(unsigned long inSec1, unsigned long inNSec1, unsigned long inSec2,
		unsigned long inNSec2, unsigned long * outMSec)
{

	if (inSec1 == inSec2)
		*outMSec = (inNSec2 - inNSec1) / 100000;
	else
		*outMSec = (1000000000 - inNSec1 + inNSec2) / 100000;

	return;

}

void CPCCANMsgCounter(int handle, const CPC_MSG_T * cpcmsg) {
	switch (cpcmsg->type) {
	case CPC_MSG_T_CAN: /* checking CAN data messages     */
		msgRxCnt++;
		return;
	case CPC_MSG_T_XCAN: /* checking CAN data messages     */
		msgXRxCnt++;
		return;
	case CPC_MSG_T_RTR: /* checking RTR-CAN messages      */
		rtrRxCnt++;
		return;
	case CPC_MSG_T_XRTR: /* checking RTR-CAN messages      */
		rtrXRxCnt++;
		return;
	}
}

void CPCPrintCANMessage(int handle, const CPC_MSG_T * cpcmsg)
{
	unsigned int i;

	static unsigned long oldInterval_s;
	static unsigned long oldInterval_ns;
	static unsigned long actualInterval = 0;

	if (cpcmsg->type == CPC_MSG_T_CAN) { /* checking CAN data messages     */
		printf("\nTS[%u]: %8.8lu.%9.9lu\n", handle, cpcmsg->ts_sec,
				cpcmsg->ts_nsec);

		diffTS(oldInterval_s, oldInterval_ns, cpcmsg->ts_sec, cpcmsg->ts_nsec,
				&actualInterval);

		printf("SD[%u]: %2.2lu.%4.4lu %4.4u ID: %3.3lx, L:%2.2u Data: ",
				handle, actualInterval / 10000, actualInterval % 10000,
				msgRxCnt, cpcmsg->msg.canmsg.id, cpcmsg->msg.canmsg.length);
		oldInterval_s = cpcmsg->ts_sec;
		oldInterval_ns = cpcmsg->ts_nsec;

		for (i = 0; i < cpcmsg->msg.canmsg.length; i++)
			printf("%2.2x ", cpcmsg->msg.canmsg.msg[i]);
		printf("\n");
	} else if (cpcmsg->type == CPC_MSG_T_XCAN) { /* checking CAN data messages     */
		diffTS(oldInterval_s, oldInterval_ns, cpcmsg->ts_sec, cpcmsg->ts_nsec,
				&actualInterval);

		printf("XD[%u]: %2.2lu.%4.4lu %4.4u ID: %8.8lx, Len: %2.2u Data: ",
				handle, actualInterval / 10000, actualInterval % 10000,
				msgXRxCnt, cpcmsg->msg.canmsg.id, cpcmsg->msg.canmsg.length);
		for (i = 0; i < cpcmsg->msg.canmsg.length; i++)
			printf("%2.2x ", cpcmsg->msg.canmsg.msg[i]);
		printf("\n");

		oldInterval_s = cpcmsg->ts_sec;
		oldInterval_ns = cpcmsg->ts_nsec;
	} else if (cpcmsg->type == CPC_MSG_T_RTR) { /* checking RTR-CAN messages      */
		diffTS(oldInterval_s, oldInterval_ns, cpcmsg->ts_sec, cpcmsg->ts_nsec,
				&actualInterval);

		printf("SR[%u]: %2.2lu.%4.4lu %4.4u ID: %3.3lx, Len: %2.2u\n", handle,
				actualInterval / 10000, actualInterval % 10000, rtrRxCnt,
				cpcmsg->msg.canmsg.id, cpcmsg->msg.canmsg.length);
		oldInterval_s = cpcmsg->ts_sec;
		oldInterval_ns = cpcmsg->ts_nsec;
	} else if (cpcmsg->type == CPC_MSG_T_XRTR) { /* checking RTR-CAN messages      */
		diffTS(oldInterval_s, oldInterval_ns, cpcmsg->ts_sec, cpcmsg->ts_nsec,
				&actualInterval);

		printf("XR[%u]: %2.2lu.%4.4lu %4.4u ID: %8.8lx, Len: %2.2u\n", handle,
				actualInterval / 10000, actualInterval % 10000, rtrXRxCnt,
				cpcmsg->msg.canmsg.id, cpcmsg->msg.canmsg.length);
		oldInterval_s = cpcmsg->ts_sec;
		oldInterval_ns = cpcmsg->ts_nsec;
	}
}

#define PCA82C200 1
#define SJA1000   2
#define AN82527   3
#define M16C      4

const char *CPCCANContrStr[] = { "Unknown", "PCA82C200", "SJA1000", "AN82527",
		"M16C_BASIC" };

void CPCPrintCANParams(int handle, const CPC_MSG_T * cpcmsg)
{
	CPC_CAN_PARAMS_T * ParamsPtr = (CPC_CAN_PARAMS_T *) &cpcmsg->msg.canparams;

	if (cpcmsg->type == CPC_MSG_T_CAN_PRMS) {
		switch (ParamsPtr->cc_type) {
		case SJA1000: {

			printf("\nCAN controller SJA1000 parameters set:\n");
			printf("MODE       : %2.2Xh\n", ParamsPtr->cc_params.sja1000.mode);
			printf("BTR0       : %2.2Xh\n", ParamsPtr->cc_params.sja1000.btr0);
			printf("BTR1       : %2.2Xh\n", ParamsPtr->cc_params.sja1000.btr1);
			printf("OUTP_CONTR : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.outp_contr);
			printf("\n");
			printf("ACC_CODE0  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_code0);
			printf("ACC_CODE1  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_code1);
			printf("ACC_CODE2  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_code2);
			printf("ACC_CODE3  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_code3);
			printf("ACC_MASK0  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_mask0);
			printf("ACC_MASK1  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_mask1);
			printf("ACC_MASK2  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_mask2);
			printf("ACC_MASK3  : %2.2Xh\n",
					ParamsPtr->cc_params.sja1000.acc_mask3);

		}
			break;
		case M16C_BASIC: {
			printf("\nCAN controller M16C parameters set:\n");
			printf("CON0          : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.con0);
			printf("CON11         : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.con1);
			printf("CTLR0         : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.ctlr0);
			printf("CTLR1         : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.ctlr1);
			printf("CLK           : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.clk);
			printf("\n");
			printf("ACC_CODE_STD0 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_std_code0);
			printf("ACC_CODE_STD1 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_std_code1);
			printf("ACC_MASK_STD0 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_std_mask0);
			printf("ACC_MASK_STD1 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_std_mask1);
			printf("\n");
			printf("ACC_CODE_EXT0 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_code0);
			printf("ACC_CODE_EXT1 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_code1);
			printf("ACC_CODE_EXT2 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_code2);
			printf("ACC_CODE_EXT3 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_code3);
			printf("ACC_MASK_EXT0 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_mask0);
			printf("ACC_MASK_EXT1 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_mask1);
			printf("ACC_MASK_EXT2 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_mask2);
			printf("ACC_MASK_EXT3 : %2.2Xh\n",
					ParamsPtr->cc_params.m16c_basic.acc_ext_mask3);
		}
			break;
		}
	}
}

const char *CPCInfoSourceStr[] = {
		"UnknownSource", "Interface", "Driver", "Library" };

const char *CPCInfoTypeStr[] = { "UnknownType", "Version", "Serial" };

void CPCPrintInfo(int handle, const CPC_MSG_T * cpcmsg) {
	unsigned int i;

	if (cpcmsg->type == CPC_MSG_T_INFO) { /* checking the driver info string*/
		printf("CPC-Info[%u]: %s.%s: ", handle,
				CPCInfoSourceStr[cpcmsg->msg.info.source],
				CPCInfoTypeStr[cpcmsg->msg.info.type]);

		for (i = 0; i < cpcmsg->length - 2; i++)
			printf("%c", cpcmsg->msg.info.msg[i]);
		printf("\n");
	}

}

void CPCPrintCANState(int handle, const CPC_MSG_T * cpcmsg) {
	if (cpcmsg->type == CPC_MSG_T_CANSTATE) /* checking CAN state */
		printf("CAN State[%u]: %2.2Xh\n", handle, cpcmsg->msg.canstate);
}

const char *CPCErrStr_1[] = { "bit error  ", //   0
		"form error ", //   2
		"stuff error", //   3
		"other error" }; //   4

const char *CPCErrStr_2[] = { "", //   0
		"", //   1
		"ID28 to ID21", //   2
		"start of frame", //   3
		"bit SRTR", //   4
		"bit IDE", //   5
		"ID20 to ID18", //   6
		"ID17 to ID13", //   7
		"CRC sequence", //   8
		"reserved bit 0", //   9
		"data field", //  10
		"data length code", //  11
		"bit RTR", //  12
		"reserved bit 1", //  13
		"ID04 to ID00", //  14
		"ID12 to ID05", //  15
		"", //  16
		"active error flag", //  17
		"intermission", //  18
		"tolerate dominant bits",//  19
		"", //  20
		"", //  21
		"passive error flag", //  22
		"error delimiter", //  23
		"CRC delimiter", //  24
		"acknowledge slot", //  25
		"end of frame", //  26
		"acknowledge delimiter", //  27
		"overload flag" }; //  28

void CPCPrintCANError(int handle, const CPC_MSG_T * cpcmsg) {
	if (cpcmsg->type == CPC_MSG_T_CANERROR) /* checking CAN error */
		printf("CAN Error[%u]: %s, %s, %s\n", handle, (cpcmsg->msg.error.ecode
				& 0x04) ? "RX" : "TX", CPCErrStr_1[(cpcmsg->msg.error.ecode
				& 0xc0) >> 6], CPCErrStr_2[cpcmsg->msg.error.ecode & 0x1f]);
}

const char *CPCMsgStr[] = { "CPC_MSG_T_RESYNC", //  0
		"CPC_MSG_T_CAN", //  1
		"CPC_MSG_T_BUSLOAD", //  2
		"CPC_MSG_T_STRING", //  3
		"CPC_MSG_T_CONTI", //  4
		"not defined", //  5
		"not defined", //  6
		"CPC_MSG_T_MEM", //  7
		"CPC_MSG_T_RTR", //  8
		"CPC_MSG_T_TXACK", //  9
		"CPC_MSG_T_POWERUP", // 10
		"CPC_MSG_T_CMD_NO", // 11
		"CPC_MSG_T_CAN_PRMS", // 12
		"CPC_MSG_T_ABORTED", // 13
		"CPC_MSG_T_CANSTATE", // 14
		"CPC_MSG_T_RESET", // 15
		"CPC_MSG_T_XCAN", // 16
		"CPC_MSG_T_XRTR", // 17
		"CPC_MSG_T_INFO", // 18
		"CPC_MSG_T_CONTROL", // 19
		"CPC_MSG_T_CONFIRM", // 20
		"CPC_MSG_T_OVERRUN" // 21
		};

void CPCPrintCPCMessage(int handle, const CPC_MSG_T * cpcmsg)
{
	unsigned char i;

	printf("Chan[%u], %s, Len: %d, D: ", handle, CPCMsgStr[cpcmsg->type],
			cpcmsg->length);
	for (i = 0; i < cpcmsg->length; i++)
		printf("%2.2x ", cpcmsg->msg.generic[i]);
	printf("\n");
}

void CPCPrintCANSendMessage(int handle, unsigned char type, const CPC_CAN_MSG_T * msg)
{
	unsigned int i;

	if (type == CPC_CMD_T_CAN) { /* checking CAN data messages     */
		printf("Send SD[%u]: %4.4u ID: %3.3lx, L:%2.2u Data: ", handle,
				msgTxCnt, msg->id, msg->length);

		for (i = 0; i < msg->length; i++)
			printf("%2.2x ", msg->msg[i]);
		printf("\n");
	} else if (type == CPC_CMD_T_XCAN) { /* checking CAN data messages     */
		printf("Send XD[%u]: %4.4u ID: %8.8lx, L: %2.2u Data: ", handle,
				msgXTxCnt, msg->id, msg->length);
		for (i = 0; i < msg->length; i++)
			printf("%2.2x ", msg->msg[i]);
		printf("\n");
	} else if (type == CPC_CMD_T_RTR) { /* checking RTR-CAN messages      */
		printf("Send SR[%u]: %4.4u ID: %3.3lx, L: %2.2u\n", handle, rtrTxCnt,
				msg->id, msg->length);
	} else if (type == CPC_CMD_T_XRTR) { /* checking RTR-CAN messages      */
		printf("Send XR[%u]: %4.4u ID: %8.8lx, L: %2.2u\n", handle, rtrXTxCnt,
				msg->id, msg->length);
	}
}
