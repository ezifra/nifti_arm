/*
 * CPCLIB
 *
 * Copyright (C) 2000-2008 EMS Dr. Thomas Wuensche
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifndef CPC_COMMON_H
#define CPC_COMMON_H

#include "../include/cpc.h"
#include "../include/ethercan.h"

#ifdef SERIAL_SUPPORT
#include <termios.h>
#include "../include/serial.h"
#endif

#define CPCMSG_HEADER_LEN      12
#define CPC_CAN_MSG_HEADER_LEN 5

#define MAX_CHANNEL_ENTRIES  CAN_MAX_DEVICE

#define HIDE_SYMBOL __attribute__((visibility("hidden")))

/* LIBRARY specific *****************************************************/

typedef struct CPC_LIB_PARAMS {
	char name[32];		// this is to be nice
	ssize_t(*read) (int fd, void *buf, size_t count);
	ssize_t(*write) (int fd, const void *buf, size_t count);

	union {
		// We need this to remember the state of reading and writing to TCP sockets for EtherCAN
		struct ethercan {
			struct ethercan_write {
				unsigned char sendBuf[255];
				unsigned char sendBufFree;
				unsigned int bytesWritten;
			} write;

			struct ethercan_read {
				int i; /* important int ... */
				int j; /* ... for the state-machine */
				int tcpframelength; /* length of the received tcp frame */
				int state;          /* the state for the state-machine */
				unsigned char convertBuffer[MAX_CONV_MSG_BUF_SIZE]; /* the converted message */
				unsigned char recvbuf[MAX_RECV_TCPFRAME_BUF_SIZE];  /* the received messages */
				unsigned char ready;
			} read;

			int fd;
		} ethercan; // EtherCAN related variables

#ifdef SERIAL_SUPPORT
		// We need this to remember the state of reading and writing to serial interfaces
		struct serial {
			struct serial_write {
				unsigned char sendBuf[1000];
				unsigned char sendBufFree;
				unsigned int bytesWritten;
			} write;

			struct serial_read {
				int i;
				int length;
				unsigned char cnt;
				unsigned char esc_state;
				unsigned short inchar;
				unsigned short chk;
				unsigned char errorcode;
				int state;          /* the state for the state-machine */
				unsigned char recvbuf[MAX_RECV_SERIAL_BUF_SIZE];  /* the received messages */
				unsigned char ready;
			} read;
			
			struct termios oldtermios;
			int fd;
		} serial;
#endif
	} interface;

	CPC_MSG_T handleMessage; // This message is returned if CPC_Handle was successfully

} CPC_LIB_PARAMS_T;

extern CPC_LIB_PARAMS_T CPCLibParams[];

extern unsigned int CPCLIBverbose;

// debug
#define DEBUG_INFORMATIVE  5
#define DEBUG_DEBUG        10
#define BE_INFORMATIVE     (CPCLIBverbose >= DEBUG_INFORMATIVE)
#define BE_DEBUG           (CPCLIBverbose >= DEBUG_DEBUG)

#endif
