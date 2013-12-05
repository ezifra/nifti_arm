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

#include "thread.h"

#include "timer.h"

const char* thread_errors[] = {
  "success",
  "error creating thread",
};

void thread_empty_cleanup(void* arg);

int thread_start(thread_p thread, void* (*thread_routine)(void*), 
  void (*thread_cleanup)(void*), void* thread_arg, double frequency) {
  thread->routine = thread_routine;
  thread->cleanup = (thread_cleanup) ? thread_cleanup : thread_empty_cleanup;
  thread->arg = thread_arg;

  thread->frequency = frequency;
  thread->start_time = 0.0;

  thread->exit_request = 0;

  if (pthread_create(&thread->thread, NULL, thread_run, thread))
    return THREAD_ERROR_CREATE;
  else
    return THREAD_ERROR_NONE;
}

void thread_exit(thread_p thread, int wait) {
  thread_mutex_lock(&thread->mutex);
  thread->exit_request = 1;
  thread_mutex_unlock(&thread->mutex);

#ifdef HAVE_LIBGCC_S
  pthread_cancel(thread->thread);
#endif

  if (wait)
    thread_wait_exit(thread);
}

void thread_self_exit() {
  pthread_exit(0);
}

void* thread_run(void* arg) {
  thread_p thread = arg;
  void* result;

  pthread_cleanup_push(thread->cleanup, thread->arg);

#ifdef HAVE_LIBGCC_S
  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);
  pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, 0);
#else
  pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);
#endif 

  thread_mutex_init(&thread->mutex);
  timer_start(&thread->start_time);

  if (thread->frequency > 0.0) {
    while (!thread_test_exit(thread)) {
      double timestamp;
      timer_start(&timestamp);

      result = thread->routine(thread->arg);

      timer_wait(timestamp, thread->frequency);
    }
  }
  else
    result = thread->routine(thread->arg);

  thread_mutex_destroy(&thread->mutex);

  pthread_cleanup_pop(1);

  return result;
}

int thread_test_exit(thread_p thread) {
  int result = THREAD_ERROR_NONE;

  thread_mutex_lock(&thread->mutex);
  result = thread->exit_request;
  thread_mutex_unlock(&thread->mutex);

  return result;
}

void thread_self_test_exit() {
  pthread_testcancel();
}

void thread_wait_exit(thread_p thread) {
  pthread_join(thread->thread, NULL);
}

void thread_empty_cleanup(void* arg) {
}
