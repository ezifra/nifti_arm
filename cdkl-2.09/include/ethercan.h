#ifndef ETHERCAN_H
#define ETHERCAN_H

#define ETHERCAN_PORT              1500
#define MAX_CONV_MSG_BUF_SIZE      70
#define MAX_RECV_TCPFRAME_BUF_SIZE 500
#define FIND_BEGIN                 0
#define STORE_DATA                 1

#define ETHERCAN_VERSION "1.2"

int setupEtherCANConnection(int handle, char *ucChannel);
ssize_t TCP_Read(int fd, void *buf, size_t count);
ssize_t TCP_Write(int fd, const void *buf, size_t count);

#endif
