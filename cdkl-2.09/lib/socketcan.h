#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define PF_CAN 29
#define AF_CAN PF_CAN

ssize_t socketcan_read(int fd, void *buf, size_t count);
ssize_t socketcan_write(int fd, const void *buf, size_t count);

#endif
