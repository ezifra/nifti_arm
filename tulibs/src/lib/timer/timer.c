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

#include <sys/time.h>

#include "timer.h"

const char* timer_errors[] = {
  "success",
  "timer fault"
};

void timer_start(
  double* timestamp) {
  struct timeval time;
  double million = 1e6;

  gettimeofday(&time, 0);

  *timestamp = time.tv_sec+time.tv_usec/million;
}

void timer_correct(
  double* timestamp) {
  struct timeval time;
  double million = 1e6;

  gettimeofday(&time, 0);

  *timestamp = 0.5*(*timestamp+time.tv_sec+time.tv_usec/million);
}

double timer_stop(
  double timestamp) {
  struct timeval time;
  double million = 1e6;

  gettimeofday(&time, 0);

  return time.tv_sec+time.tv_usec/million-timestamp;
}

double timer_get_frequency(
  double timestamp) {
  return 1.0/timer_stop(timestamp);
}

int timer_wait(
  double timestamp,
  double frequency) {
  if (frequency <= 0.0) return TIMER_ERROR_FAULT;

  return timer_sleep(1.0/frequency-timer_stop(timestamp));
}

int timer_sleep(
  double seconds) {
  if (seconds < 0.0) return TIMER_ERROR_FAULT;

  usleep((int)(seconds*1e6));

  return TIMER_ERROR_NONE;
}
