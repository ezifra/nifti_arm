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

#ifndef CONFIG_H
#define CONFIG_H

#include <stdlib.h>

#include "param.h"

/** \file config.h
  * \brief Simple configuration implementation
  * \author Ralf Kaestner
  * A POSIX-compliant configuration implementation.
  */

/** Predefined configuration constants
  */
#define CONFIG_ARG_HELP                 "--help"

/** \brief Configuration structure
  */
typedef struct config_t {
  param_p params;         //!< The configuration parameters.
  ssize_t num_params;     //!< The number of configuration parameters.
} config_t, *config_p;

/** \brief Initialize an empty configuration
  * \param[in] config The configuration to be initialized.
  */
void config_init(
  config_p config);

/** \brief Initialize a configuration from default parameters
  * \param[in] config The configuration to be initialized.
  * \param[in] default_config The default configuration parameters used to
  *   initialize the configuration.
  */
void config_init_default(
  config_p config,
  config_p default_config);

/** \brief Initialize a configuration from command line arguments
  * \param[in] config The configuration to be initialized.
  * \param[in] argc The number of supplied command line arguments.
  * \param[in] argv The list of supplied command line arguments.
  * \param[in] prefix An optional argument prefix that will be stripped from 
  *   the parameter keys.
  * \return 1 if the help argument was given, 0 otherwise.
  */
int config_init_arg(
  config_p config,
  int argc,
  char **argv,
  const char* prefix);

/** \brief Destroy a configuration
  * \param[in] config The configuration to be destroyed.
  */
void config_destroy(
  config_p config);

/** \brief Print a configuration
  * \param[in] stream The output stream that will be used for printing the
  *   configuration.
  * \param[in] config The configuration that will be printed.
  */
void config_print(
  FILE* stream,
  config_p config);

/** \brief Print help for a configuration
  * \param[in] stream The output stream that will be used for printing the
  *   configuration help.
  * \param[in] config The configuration for which help will be printed.
  * \param[in] prefix An optional argument prefix that will be prepended to
  *   the parameter keys.
  */
void config_print_help(
  FILE* stream,
  config_p config,
  const char* prefix);

/** \brief Set configuration parameters from a source configuration.
  * \note If the source parameters contain the help argument, a help
  *   will be printed for the destination configuration and the program
  *   exits with return value 0.
  * \param[in] dst_config The configuration to set the parameters for.
  * \param[in] src_config The configuration containing the source parameters
  *    to be set.
  */
void config_set(
  config_p dst_config,
  config_p src_config);

/** \brief Set a configuration parameter
  * \note If a parameter with the same key already exists in the configuration,
  *   its value will be modified accordingly. Otherwise, a new parameter
  *   will be appended to the configuration.
  * \param[in] config The configuration to set the parameter for.
  * \param[in] param The parameter to be set.
  */
void config_set_param(
  config_p config,
  param_p param);

/** \brief Retrieve a configuration parameter
  * \param[in] config The configuration to retrieve the parameter from.
  * \param[in] key The key of the parameter to be retrieved.
  * \return The configuration parameter with the specified key or null,
  *   if no such parameter exists.
  */
param_p config_get_param(
  config_p config,
  const char* key);

/** \brief Set a configuration parameter's string value
  * \param[in] config The configuration to set the string value for.
  * \param[in] key The key of the string value to be set.
  * \param[in] value The string value to be set.
  */
void config_set_string(
  config_p config,
  const char* key,
  const char* value);

/** \brief Retrieve a configuration parameter's string value
  * \param[in] config The configuration to retrieve the string value from.
  * \param[in] key The key of the string value to be retrieved.
  * \return The parameter's string value.
  */
const char* config_get_string(
  config_p config,
  const char* key);

/** \brief Set a configuration parameter's integer value
  * \param[in] config The configuration to set the integer value for.
  * \param[in] key The key of the integer value to be set.
  * \param[in] value The integer value to be set.
  */
void config_set_int(
  config_p config,
  const char* key,
  int value);

/** \brief Retrieve a configuration parameter's integer value
  * \param[in] config The configuration to retrieve the integer value from.
  * \param[in] key The key of the integer value to be retrieved.
  * \return The parameter's integer value.
  */
int config_get_int(
  config_p config,
  const char* key);

/** \brief Set a configuration parameter's floating point value
  * \param[in] config The configuration to set the floating point value for.
  * \param[in] key The key of the floating point value to be set.
  * \param[in] value The floating point value to be set.
  */
void config_set_float(
  config_p config,
  const char* key,
  double value);

/** \brief Retrieve a configuration parameter's floating point value
  * \param[in] config The configuration to retrieve the floating point
  *   value from.
  * \param[in] key The key of the floating point value to be retrieved.
  * \return The parameter's floating point value.
  */
double config_get_float(
  config_p config,
  const char* key);

#endif
