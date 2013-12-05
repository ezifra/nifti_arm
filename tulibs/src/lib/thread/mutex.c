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

#include "mutex.h"

const char* thread_mutex_errors[] = {
  "success",
  "failed to acquire mutex lock",
};

void thread_mutex_init(thread_mutex_p mutex) {
  pthread_mutex_init(&mutex->mutex, 0);
}

void thread_mutex_destroy(thread_mutex_p mutex) {
  pthread_mutex_destroy(&mutex->mutex);
}

void thread_mutex_lock(thread_mutex_p mutex) {
  pthread_mutex_lock(&mutex->mutex);
}

void thread_mutex_unlock(thread_mutex_p mutex) {
  pthread_mutex_unlock(&mutex->mutex);
}

int thread_mutex_try_lock(thread_mutex_p mutex) {
  if (!pthread_mutex_trylock(&mutex->mutex))
    return THREAD_MUTEX_ERROR_NONE;
  else
    return THREAD_MUTEX_ERROR_LOCK;
}
