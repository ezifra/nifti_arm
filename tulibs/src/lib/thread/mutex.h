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

#ifndef THREAD_MUTEX_H
#define THREAD_MUTEX_H

/** \file mutex.h
  * \brief POSIX-compliant thread mutex implementation
  * \author Ralf Kaestner
  * A basic POSIX-compliant thread mutex implementation.
  */

#include <pthread.h>

/** Predefined thread handling error codes
  */
#define THREAD_MUTEX_ERROR_NONE        0
#define THREAD_MUTEX_ERROR_LOCK        1

/** \brief Predefined thread handling error descriptions
  */
extern const char* thread_mutex_errors[];

/** \brief Structure defining the thread mutex
  */
typedef struct thread_mutex_t {
  pthread_mutex_t mutex;    //!< The mutex handle.
} thread_mutex_t, *thread_mutex_p;

/** \brief Initialize a thread mutex
  * \param[in] mutex The thread mutex to be initialized.
  */
void thread_mutex_init(
  thread_mutex_p mutex);

/** \brief Destroy a thread mutex
  * \param[in] mutex The initialized thread mutex to be destroyed.
  */
void thread_mutex_destroy(
  thread_mutex_p mutex);

/** \brief Lock a thread mutex
  * \param[in] mutex The initialized thread mutex to be locked.
  */
void thread_mutex_lock(
  thread_mutex_p mutex);

/** \brief Unlock a thread mutex
  * \param[in] mutex The initialized thread mutex to be unlocked.
  */
void thread_mutex_unlock(
  thread_mutex_p mutex);

/** \brief Try to lock a thread mutex
  * \param[in] mutex The initialized thread mutex to be locked.
  * \return The resulting error code.
  */
int thread_mutex_try_lock(
  thread_mutex_p mutex);

#endif
