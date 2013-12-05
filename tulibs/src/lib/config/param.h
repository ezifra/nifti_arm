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

#ifndef PARAM_H
#define PARAM_H

/** \file param.h
  * \brief Simple parameter implementation
  * \author Ralf Kaestner
  * A POSIX-compliant parameter implementation.
  */

#include <stdio.h>

/** Predefined parameter constants
  */
#define PARAM_KEY_LENGTH         256
#define PARAM_VALUE_LENGTH       256

/** \brief Parameter structure
  */
typedef struct param_t {
  char key[PARAM_KEY_LENGTH];         //!< The parameter's key.
  char value[PARAM_VALUE_LENGTH];     //!< The parameter's value.
} param_t, *param_p;

/** \brief Initialize a parameter by string value
  * \param[in] param The parameter to be initialized.
  * \param[in] key The key of the parameter to be initialized.
  * \param[in] value The string value of the parameter to be initialized.
  */
void param_init_string(
  param_p param,
  const char* key,
  const char* value);

/** \brief Initialize a parameter by integer value
  * \param[in] param The parameter to be initialized.
  * \param[in] key The key of the parameter to be initialized.
  * \param[in] value The integer value of the parameter to be initialized.
  */
void param_init_int(
  param_p param,
  const char* key,
  int value);

/** \brief Initialize a parameter by floating point value
  * \param[in] param The parameter to be initialized.
  * \param[in] key The key of the parameter to be initialized.
  * \param[in] value The floating point value of the parameter to be
  *   initialized.
  */
void param_init_float(
  param_p param,
  const char* key,
  double value);

/** \brief Print a parameter
  * \param[in] stream The output stream that will be used for printing the
  *   parameter.
  * \param[in] param The parameter that will be printed.
  */
void param_print(
  FILE* stream,
  param_p param);

/** \brief Print help for a parameter
  * \param[in] stream The output stream that will be used for printing the
  *   parameter help.
  * \param[in] param The parameter for which help will be printed.
  * \param[in] prefix An optional argument prefix that will be appended to
  *   the parameter key.
  */
void param_print_help(
  FILE* stream,
  param_p param,
  const char* prefix);

/** \brief Set a parameter's string value
  * \param[in] param The parameter to set the string value for.
  * \param[in] value The string value to be set.
  */
void param_set_string_value(
  param_p param,
  const char* value);

/** \brief Retrieve a parameter's string value
  * \param[in] param The parameter to retrieve the string value from.
  * \return The parameter's string value.
  */
const char* param_get_string_value(
  param_p param);

/** \brief Set a parameter's integer value
  * \param[in] param The parameter to set the integer value for.
  * \param[in] value The integer value to be set.
  */
void param_set_int_value(
  param_p param,
  int value);

/** \brief Retrieve a parameter's integer value
  * \param[in] param The parameter to retrieve the integer value from.
  * \return The parameter's integer value.
  */
int param_get_int_value(
  param_p param);

/** \brief Set a parameter's floating point value
  * \param[in] param The parameter to set the floating point value for.
  * \param[in] value The floating point value to be set.
  */
void param_set_float_value(
  param_p param,
  double value);

/** \brief Retrieve a parameter's floating point value
  * \param[in] param The parameter to retrieve the floating point value from.
  * \return The parameter's floating point value.
  */
double param_get_float_value(
  param_p param);

#endif
