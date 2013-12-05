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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "param.h"

void param_init_string(param_p param, const char* key, const char*
  value) {
  strcpy(param->key, key);
  param_set_string_value(param, value);
}

void param_init_int(param_p param, const char* key, int value) {
  strcpy(param->key, key);
  param_set_int_value(param, value);
}

void param_init_float(param_p param, const char* key, double value) {
  strcpy(param->key, key);
  param_set_float_value(param, value);
}

void param_print(FILE* stream, param_p param) {
  fprintf(stream, "%s = %s\n", param->key, param->value);
}

void param_print_help(FILE* stream, param_p param, const char* prefix) {
  char arg[prefix ? strlen(prefix)+strlen(param->key)+4 :
    strlen(param->key)+3];

  if (prefix)
    sprintf(arg, "--%s-%s", prefix, param->key);
  else
    sprintf(arg, "--%s", param->key);

  fprintf(stream, "  %-25s  [%s]\n", arg, param->value);
}

void param_set_string_value(param_p param, const char* value) {
  strcpy(param->value, value);
}

const char* param_get_string_value(param_p param) {
  return param->value;
}

void param_set_int_value(param_p param, int value) {
  sprintf(param->value, "%d", value);
}

int param_get_int_value(param_p param) {
  return atoi(param->value);
}

void param_set_float_value(param_p param, double value) {
  sprintf(param->value, "%lf", value);
}

double param_get_float_value(param_p param) {
  return atof(param->value);
}
