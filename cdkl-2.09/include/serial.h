#ifndef SERIAL_H
#define SERIAL_H

void uart_ProcessSend(void);
int  uart_GetNextMessage(CPC_MSG_T * msg);

#define MAX_RECV_SERIAL_BUF_SIZE	1000

#define	ESC_NONE        0
#define	ESC_ESC         1

#define S_IDLE          0
#define S_TYPE          1
#define S_LEN           2
#define S_MSGID         3
#define S_TSSEC1        4
#define S_TSSEC2        5
#define S_TSSEC3        6
#define S_TSSEC4        7
#define S_TSNSEC1       8
#define S_TSNSEC2       9
#define S_TSNSEC3      10
#define S_TSNSEC4      11
#define S_DAT          12
#define S_CHK1         13
#define S_CHK2         14
#define S_END          15

/* Escape sequences are characters with value <8 and most significant bit set.
 * i.e 0x05 will become a sequence of 0x02 and 0x85
 */
#define CW_BEGINOFDATA   0x1ff
#define CW_ENDOFDATA     0x1fe
#define CW_ESCAPE        0xfd

/* defines for errorcodes */
#define E_NONE           0x00
#define E_CHECKSUM       0x01
#define E_LENGTH         0x02
#define E_FORMAT         0x04
#define E_LENGTH_TOO_BIG 0x05

#define IS_ESCAPE(a)     ((a) == CW_ESCAPE ? 1 : 0)
#define REMOVE_ESCAPE(a) ((a) |= 0x80);

#define CONTENT(a) (unsigned char)(a&0xff)

int scanForDevice(char *device, char *serial, char *ucChannel);
ssize_t Serial_Read(int fd, void *buf, size_t count);
ssize_t Serial_Write(int fd, const void *buf, size_t count);

#endif
