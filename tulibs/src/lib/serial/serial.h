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

#ifndef SERIAL_H
#define SERIAL_H

/** \file serial.h
  * \brief POSIX serial communication library
  * \author Ralf Kaestner
  * A POSIX-compliant serial communication library providing very basic
  * functionality.
  */

#include <unistd.h>

/** Predefined serial error codes
  */
#define SERIAL_ERROR_NONE                 0
#define SERIAL_ERROR_OPEN                 1
#define SERIAL_ERROR_CLOSE                2
#define SERIAL_ERROR_DRAIN                3
#define SERIAL_ERROR_FLUSH                4
#define SERIAL_ERROR_INVALID_BAUDRATE     5
#define SERIAL_ERROR_INVALID_DATABITS     6
#define SERIAL_ERROR_INVALID_STOPBITS     7
#define SERIAL_ERROR_INVALID_PARITY       8
#define SERIAL_ERROR_SETUP                9
#define SERIAL_ERROR_TIMEOUT              10
#define SERIAL_ERROR_READ                 11
#define SERIAL_ERROR_WRITE                12

/** \brief Predefined serial error descriptions
  */
extern const char* serial_errors[];

/** \brief Parity enumeratable type
  */
typedef enum {
  none = 0,   //!< No parity.
  odd = 1,    //!< Odd parity.
  even = 2    //!< Even parity.
} serial_parity_t;

/** \brief Serial device structure
  */
typedef struct serial_device_t {
  int fd;                   //!< File descriptor.
  char name[256];           //!< Device name.

  int baudrate;             //!< Device baudrate.
  int databits;             //!< Number of databits.
  int stopbits;             //!< Number of stopbits.
  serial_parity_t parity;   //!< Device parity.

  double timeout;           //!< Device select timeout in [s].

  ssize_t num_read;         //!< Number of bytes read from device.
  ssize_t num_written;      //!< Number of bytes written to device.
} serial_device_t, *serial_device_p;

/** \brief Open the serial device with the specified name
  * \param[in] dev The serial device to be opened.
  * \param[in] name The name of the device to be opened.
  * \return The resulting error code.
  */
int serial_open(
  serial_device_p dev,
  const char* name);

/** \brief Close an open serial device
  * \param[in] dev The open serial device to be closed.
  * \return The resulting error code.
  */
int serial_close(
  serial_device_p dev);

/** \brief Setup an already opened serial device
  * \param[in] dev The open serial device to be set up.
  * \param[in] baudrate The device baudrate to be set.
  * \param[in] databits The device's number of databits to be set.
  * \param[in] stopbits The device's number of stopbits to be set.
  * \param[in] parity The device parity to be set.
  * \param[in] timeout The device select timeout to be set in [s].
  * \return The resulting error code.
  */
int serial_setup(
  serial_device_p dev,
  int baudrate,
  int databits,
  int stopbits,
  serial_parity_t parity,
  double timeout);

/** \brief Read data from open serial device
  * \param[in] dev The open serial device to read data from.
  * \param[out] data An array containing the data read from the device.
  * \param[in] num The number of data bytes to be read.
  * \return The number of bytes read from the serial device or the
  *   negative error code.
  */
int serial_read(
  serial_device_p dev,
  unsigned char* data,
  ssize_t num);

/** \brief Write data to open serial device
  * \param[in] dev The open serial device to write data to.
  * \param[out] data An array containing the data to be written to the device.
  * \param[in] num The number of data bytes to be written.
  * \return The number of bytes written to the serial device or the
  *   negative error code.
  */
int serial_write(
  serial_device_p dev,
  unsigned char* data,
  ssize_t num);

#endif
