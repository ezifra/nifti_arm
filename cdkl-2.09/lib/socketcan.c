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
#include <string.h>
#include <fcntl.h>

#include "socketcan.h"

#include "../include/cpc.h"
#include "../include/cpclib.h"

#include "../include/cpc_common.h"

/**
 * Setup a Socket-CAN channel
 */
int socketcan_SetupConnection(int slot, char *devFile)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int loopback = 1, prev_mode;

    int fd;

    if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    	if (BE_INFORMATIVE)
    		perror(devFile);

        return CPC_ERR_CHANNEL_NOT_ACTIVE;
    }

    addr.can_family = AF_CAN;

    strcpy(ifr.ifr_name, devFile);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    	if (BE_INFORMATIVE)
    		perror(devFile);

        return CPC_ERR_CHANNEL_NOT_ACTIVE;
    }
    addr.can_ifindex = ifr.ifr_ifindex;

    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    	if (BE_INFORMATIVE)
    		perror(devFile);

        return CPC_ERR_CHANNEL_NOT_ACTIVE;
    }

	if ((prev_mode = fcntl(fd, F_GETFL, 0)) != -1)
		fcntl(fd, F_SETFL, prev_mode | O_NONBLOCK);

	CPCLibParams[slot].read = socketcan_read;
	CPCLibParams[slot].write = socketcan_write;

	return fd;
}

/**
 * Receive messages (only CAN frames here)
 */
ssize_t socketcan_read(int fd, void *buf, size_t count)
{
	CPC_MSG_T *msg = buf;

	struct can_frame frame;

	ssize_t nbytes;
	struct timeval tv;

	nbytes = recv(fd, &frame, sizeof(struct can_frame), 0);

	if(nbytes <= 0)
		return -1;

	if (ioctl(fd, SIOCGSTAMP, &tv) < 0)
		gettimeofday(&tv, NULL);

	memset(msg, 0, sizeof(CPC_MSG_T));

	msg->ts_sec = tv.tv_sec;
	msg->ts_nsec = tv.tv_usec * 1000;

	if(frame.can_id & CAN_ERR_FLAG) {
		msg->type = CPC_MSG_T_CANERROR;

		return sizeof(CPC_MSG_T);
	}

	if(frame.can_id & CAN_EFF_FLAG) {
		if(frame.can_id & CAN_RTR_FLAG) {
			msg->type = CPC_MSG_T_XRTR;
		} else {
			msg->type = CPC_MSG_T_XCAN;
		}
	} else {
		if(frame.can_id & CAN_RTR_FLAG) {
			msg->type = CPC_MSG_T_RTR;
		} else {
			msg->type = CPC_MSG_T_CAN;
		}
	}

	msg->msg.canmsg.id = frame.can_id & 0x1FFFFFFFU;
	msg->msg.canmsg.length = frame.can_dlc;

	if(frame.can_id & CAN_RTR_FLAG) {
		msg->length = 4 + 1;
	} else {
		int i;

		msg->length = 4 + 1 + frame.can_dlc;

		for(i = 0; i < frame.can_dlc; i++) {
			msg->msg.canmsg.msg[i]  = frame.data[i];
		}
	}

	return sizeof(CPC_MSG_T);
}

/**
 * Send commands (only CAN frames at the moment)
 */
ssize_t socketcan_write(int fd, const void *buf, size_t count)
{
	struct can_frame frame;
	const CPC_MSG_T *msg = buf;

	int rtr, ext, i;

	memset(&frame, 0, sizeof(struct can_frame));

	switch(msg->type) {
		case CPC_CMD_T_CAN:
			rtr = 0;
			ext = 0;
			break;
		case CPC_CMD_T_XCAN:
			rtr = 0;
			ext = 1;
			break;
		case CPC_CMD_T_RTR:
			rtr = 1;
			ext = 0;
			break;
		case CPC_CMD_T_XRTR:
			rtr = 1;
			ext = 1;
			break;

		case CPC_CMD_T_CONTROL:
			return 0;

		case CPC_CMD_T_CAN_PRMS:
			return 0;

		default:
			return CPC_ERR_SERVICE_NOT_SUPPORTED;
	}

	frame.can_id = msg->msg.canmsg.id;

	frame.can_id |= rtr ? CAN_RTR_FLAG : 0;
	frame.can_id |= ext ? CAN_EFF_FLAG : 0;

	frame.can_dlc = msg->msg.canmsg.length;

	if(!rtr) {
		for(i = 0; i < frame.can_dlc; i++) {
			frame.data[i] = msg->msg.canmsg.msg[i];
		}
	}

	if(send(fd, &frame, sizeof(struct can_frame), 0) < 0)
		return CPC_ERR_TRANSMISSION_FAILED;

	return sizeof(CPC_CAN_MSG_T);
}

