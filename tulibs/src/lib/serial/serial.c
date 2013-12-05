/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include "serial.h"

const char* serial_errors[] = {
  "success",
  "error opening serial device",
  "error closing serial device",
  "error draining serial device",
  "error flushing serial device",
  "invalid baudrate",
  "invalid number of databits",
  "invalid number of stopbits",
  "invalid parity",
  "error setting serial device parameters",
  "serial device select timeout",
  "error reading from serial device",
  "error writing to serial device",
};

int serial_open(serial_device_p dev, const char* name) {
  dev->fd = open(name, O_RDWR | O_NDELAY);

  if (dev->fd > 0) {
    strcpy(dev->name, name);
    dev->num_read = 0;
    dev->num_written = 0;
  }
  else
    return SERIAL_ERROR_OPEN;

  return SERIAL_ERROR_NONE;
}

int serial_close(serial_device_p dev) {
  if (tcdrain(dev->fd) < 0)
    return SERIAL_ERROR_DRAIN;
  if (tcflush(dev->fd, TCIOFLUSH) < 0)
    return SERIAL_ERROR_FLUSH;
  if (!close(dev->fd)) {
    dev->name[0] = 0;
    dev->fd = -1;
  }
  else
    return SERIAL_ERROR_CLOSE;

  return SERIAL_ERROR_NONE;
}

int serial_setup(serial_device_p dev, int baudrate, int databits, int stopbits,
  serial_parity_t parity, double timeout) {
  struct termios tio;
  memset(&tio, 0, sizeof(struct termios));
  
  switch (baudrate) {
    case 50L    : tio.c_cflag |= B50;
                  break;
    case 75L    : tio.c_cflag |= B75;
                  break;
    case 110L   : tio.c_cflag |= B110;
                  break;
    case 134L   : tio.c_cflag |= B134;
                  break;
    case 150L   : tio.c_cflag |= B150;
                  break;
    case 200L   : tio.c_cflag |= B200;
                  break;
    case 300L   : tio.c_cflag |= B300;
                  break;
    case 600L   : tio.c_cflag |= B600;
                  break;
    case 1200L  : tio.c_cflag |= B1200;
                  break;
    case 1800L  : tio.c_cflag |= B1800;
                  break;
    case 2400L  : tio.c_cflag |= B2400;
                  break;
    case 4800L  : tio.c_cflag |= B4800;
                  break;
    case 9600L  : tio.c_cflag |= B9600;
                  break;
    case 19200L : tio.c_cflag |= B19200;
                  break;
    case 38400L : tio.c_cflag |= B38400;
                  break;
    case 57600L : tio.c_cflag |= B57600;
                  break;
    case 115200L: tio.c_cflag |= B115200;
                  break;
    case 230400L: tio.c_cflag |= B230400;
                  break;
    default     : return SERIAL_ERROR_INVALID_BAUDRATE;
  }
  dev->baudrate = baudrate;
  
  switch (databits) {
    case 5 : tio.c_cflag |= CS5;
             break;
    case 6 : tio.c_cflag |= CS6;
             break;
    case 7 : tio.c_cflag |= CS7;
             break;
    case 8 : tio.c_cflag |= CS8;
             break;
    default: return SERIAL_ERROR_INVALID_DATABITS;
  }
  dev->databits = databits;
  
  switch (stopbits) {
    case 1 : break;
    case 2 : tio.c_cflag |= CSTOPB;
             break;
    default: return SERIAL_ERROR_INVALID_STOPBITS;
  }
  dev->stopbits = stopbits;
  
  switch (parity) {
    case none: break;
    case even: tio.c_cflag |= PARENB;
               break;
    case odd : tio.c_cflag |= PARENB | PARODD;
               break;
    default  : return SERIAL_ERROR_INVALID_PARITY;
  }
  dev->parity = parity;
  
  dev->timeout = timeout;

  tio.c_cflag |= CLOCAL;
  tio.c_iflag = IGNPAR;
  
  if (tcflush(dev->fd, TCIOFLUSH) < 0)
    return SERIAL_ERROR_FLUSH;
  if (tcsetattr(dev->fd, TCSANOW, &tio) < 0)
    return SERIAL_ERROR_SETUP;
  
  return SERIAL_ERROR_NONE;
}

int serial_read(serial_device_p dev, unsigned char* data, ssize_t num) {
  ssize_t num_read = 0;
  struct timeval time;
  fd_set set;
  int error;

  while (num_read < num) {
    time.tv_sec = 0;
    time.tv_usec = dev->timeout*1e6;

    FD_ZERO(&set);
    FD_SET(dev->fd, &set);

    error = select(dev->fd+1, &set, NULL, NULL, &time);
    if (error == 0)
      return -SERIAL_ERROR_TIMEOUT;

    ssize_t n;
    n = read(dev->fd, &data[num_read], num-num_read);
    if ((n < 0) && (errno != EWOULDBLOCK))
      return -SERIAL_ERROR_READ;
    if (n > 0) {
      num_read += n;
      dev->num_read += n;
    }
  }
  
  return num_read;
}

int serial_write(serial_device_p dev, unsigned char* data, ssize_t num) {
  ssize_t num_written = 0;

  while (num_written < num) {
    ssize_t n;
    while ((n = write(dev->fd, &data[num_written], num-num_written)) == 0);
    if ((n < 0) && (errno != EWOULDBLOCK))
      return -SERIAL_ERROR_WRITE;
    if (n > 0) {
      num_written += n;
      dev->num_written += n;
    }
  }
  
  return num_written;
}
