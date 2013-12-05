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

#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>

#include "mutex.h"

/** \file thread.h
  * \brief POSIX-compliant thread handling
  * \author Ralf Kaestner
  * A basic POSIX-compliant thread handling implementation.
  */

/** Predefined thread handling error codes
  */
#define THREAD_ERROR_NONE              0
#define THREAD_ERROR_CREATE            1

/** \brief Predefined thread handling error descriptions
  */
extern const char* thread_errors[];

/** \brief Structure defining the thread context
  */
typedef struct thread_t {
  pthread_t thread;         //!< The thread handle.
  void* (*routine)(void*);  //!< The thread routine.
  void (*cleanup)(void*);   //!< The thread cleanup handler.
  void* arg;                //!< The thread routine argument.

  thread_mutex_t mutex;     //!< The thread mutex.

  double frequency;         //!< The thread cycle frequency in [Hz].
  double start_time;        //!< The thread start timestamp.

  int exit_request;         //!< Flag signaling a pending exit request.
} thread_t, *thread_p;

/** \brief Start a thread
  * \param[in] thread The thread to be started.
  * \param[in] thread_routine The thread routine that will be executed
  *   within the thread.
  * \param[in] thread_cleanup The optional thread cleanup handler that will 
  *   be executed upon thread termination.
  * \param[in] thread_arg The argument to be passed on to the thread
  *   routine. The memory should be allocated by the caller and will be
  *   freed after thread termination.
  * \param[in] frequency The thread cycle frequency in [Hz]. If the frequency 
  *   is 0, the thread routine will be executed once.
  * \return The resulting error code.
  */
int thread_start(
  thread_p thread,
  void* (*thread_routine)(void*),
  void (*thread_cleanup)(void*),
  void* thread_arg,
  double frequency);

/** \brief Exit a thread
  * \param[in] thread The thread to be cancelled.
  * \param[in] wait If 0, return instantly, wait for thread termination
  *   otherwise.
  */
void thread_exit(
  thread_p thread,
  int wait);

/** \brief Exit the calling thread
  */
void thread_self_exit();

/** \brief Run the thread
  * This function is run within the thread and should never be called
  * directly.
  * \param[in] arg The arguments passed to the thread.
  * \return The result of the thread.
  */
void* thread_run(void* arg);

/** \brief Test thread for a pending exit request
  * \param[in] thread The thread to be tested for a pending exit request.
  * \return 1 if an exit request is pending, 0 otherwise.
  */
int thread_test_exit(
  thread_p thread);

/** \brief Test the calling thread for a pending cancellation request
  */
void thread_self_test_exit();

/** \brief Wait for thread termination
  * \param[in] thread The thread to wait for.
  */
void thread_wait_exit(
  thread_p thread);

#endif
