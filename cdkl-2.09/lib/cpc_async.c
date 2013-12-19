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
#include <sys/types.h>
#include <sys/dir.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>

#include <poll.h>

#include "../include/cpc.h"
#include "../include/cpclib.h"
#include "../include/cpc_common.h"

/*********************************************************************************/
/* Functions to handle CPC messages in an asynchronous programming model         */
/*********************************************************************************/
extern unsigned int CPCHandlerCnt[CAN_MAX_DEVICE];
extern CPC_INIT_PARAMS_T CPCInitParams[CAN_MAX_DEVICE];
extern CPC_LIB_PARAMS_T CPCLibParams[CAN_MAX_DEVICE];

#define CPC_HANDLER_CNT 50

typedef enum {
	HDLR_STANDARD = 0,
	HDLR_EXTENDED
} EN_HDLR_T;

struct CPC_HANDLER {
	EN_HDLR_T type;

	/* standard handler */
	void (*cpc_handler) (int, const CPC_MSG_T *);

	/* extended handler */
	void (*cpc_handlerEx) (int, const CPC_MSG_T *, void *);
	void *customPointer;
};

struct CPC_HANDLER CPCHandlers[CAN_MAX_DEVICE][CPC_HANDLER_CNT];

int CPC_WaitForEvent(int handle, int timeout, unsigned char event)
{
	struct pollfd fds;
	int result = 0;

	if (CPCInitParams[handle].chanparams.fd < 0)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	fds.fd = CPCInitParams[handle].chanparams.fd;

	fds.events  = event & EVENT_READ  ? POLLIN : 0;
	fds.events |= event & EVENT_WRITE ? POLLOUT : 0;

	fds.revents = 0;

	if(poll(&fds, 1, timeout) >= 0) {
		result  = fds.revents & POLLIN  ? EVENT_READ : 0;
		result |= fds.revents & POLLOUT ? EVENT_WRITE : 0;
	}

	return result; // No events
}

/*********************************************************************************/
/* add a handler to the list                                                     */
int CPC_AddHandlerEx(int handle,
		     void (*handlerEx) (int handle, const CPC_MSG_T *, void *customPointer),
		     void *customPointer)
{
	if (CPCInitParams[handle].chanparams.fd < 0)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	if (CPCHandlerCnt[handle] < CPC_HANDLER_CNT) {
		CPCHandlers[handle][CPCHandlerCnt[handle]].cpc_handlerEx =
		    handlerEx;
		CPCHandlers[handle][CPCHandlerCnt[handle]].customPointer =
		    customPointer;
		CPCHandlers[handle][CPCHandlerCnt[handle]].type =
		    HDLR_EXTENDED;
		CPCHandlerCnt[handle]++;

		return 0;
	}
	return -1;
}

/*********************************************************************************/
/* remove a handler from the list                                                */
int CPC_RemoveHandlerEx(int handle,
			void (*handlerEx) (int handle, const CPC_MSG_T *,
					   void *customPointer))
{
	signed int i;

	if (CPCInitParams[handle].chanparams.fd < 0)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	for (i = CPCHandlerCnt[handle] - 1; i >= 0; i--) {
		if (CPCHandlers[handle][i].cpc_handlerEx == handlerEx) {
			for (; i < CPCHandlerCnt[handle]; i++) {
				CPCHandlers[handle][i].cpc_handler = CPCHandlers[handle][i + 1].cpc_handler;
				CPCHandlers[handle][i].cpc_handlerEx = CPCHandlers[handle][i + 1].cpc_handlerEx;
				CPCHandlers[handle][i].type = CPCHandlers[handle][i + 1].type;
				CPCHandlers[handle][i].customPointer =
					CPCHandlers[handle][i + 1].customPointer;
			}
			CPCHandlerCnt[handle]--;

			return 0;
		}
	}
	return -1;
}

/*********************************************************************************/
/* add a handler to the list                                                     */
int CPC_AddHandler(int handle,
		   void (*handler) (int handle, const CPC_MSG_T *))
{
	if (CPCInitParams[handle].chanparams.fd < 0)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	if (CPCHandlerCnt[handle] < CPC_HANDLER_CNT) {
		CPCHandlers[handle][CPCHandlerCnt[handle]].cpc_handler =
		    handler;
		CPCHandlers[handle][CPCHandlerCnt[handle]].type =
		    HDLR_STANDARD;
		CPCHandlerCnt[handle]++;

		return 0;
	}
	return -1;
}

/*********************************************************************************/
/* remove a handler from the list                                                */
int CPC_RemoveHandler(int handle,
		      void (*handler) (int handle, const CPC_MSG_T *))
{
	signed int i;

	if (CPCInitParams[handle].chanparams.fd < 0)
		return CPC_ERR_CHANNEL_NOT_ACTIVE;

	for (i = CPCHandlerCnt[handle] - 1; i >= 0; i--) {
		if (CPCHandlers[handle][i].cpc_handler == handler) {
			for (; i < CPCHandlerCnt[handle]; i++) {
				CPCHandlers[handle][i].cpc_handler =
				    CPCHandlers[handle][i + 1].cpc_handler;
				CPCHandlers[handle][i].cpc_handlerEx =
				    CPCHandlers[handle][i +
							1].cpc_handlerEx;
				CPCHandlers[handle][i].type =
				    CPCHandlers[handle][i + 1].type;
				CPCHandlers[handle][i].customPointer =
				    CPCHandlers[handle][i +
							1].customPointer;
			}
			CPCHandlerCnt[handle]--;
			return 0;
		}
	}
	return -1;
}

/*********************************************************************************/
/* execute all handlers in the list                                              */
CPC_MSG_T *CPC_Handle(int handle)
{
	unsigned int i;
	ssize_t retval;
	int fd = CPCInitParams[handle].chanparams.fd;

	if (fd < 0)
		return NULL;

	errno = 0;

	retval = CPCLibParams[handle].read(fd, &CPCLibParams[handle].handleMessage, sizeof(CPC_MSG_T));

	if (-errno == CPC_ERR_NO_INTERFACE_PRESENT
	    || retval == CPC_ERR_NO_INTERFACE_PRESENT) {
		CPCLibParams[handle].handleMessage.type = CPC_MSG_T_DISCONNECTED;
		CPCLibParams[handle].handleMessage.length = 1;

		/* sh 07.03.2005
		 * device has been disconnected inform application
		 * via handlers
		 */
		retval = sizeof(CPC_MSG_T);
	}

	if (retval > 0) {
		for (i = 0; i < CPCHandlerCnt[handle]; i++) {
			if (CPCHandlers[handle][i].type == HDLR_STANDARD)
				CPCHandlers[handle][i].cpc_handler(handle,
								   &CPCLibParams[handle].handleMessage);
			else
				CPCHandlers[handle][i].
				    cpc_handlerEx(handle, &CPCLibParams[handle].handleMessage,
						  CPCHandlers[handle][i].
						  customPointer);
		}
		return &CPCLibParams[handle].handleMessage;
	}

	return NULL;
}
