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

#ifndef SPLINE_H
#define SPLINE_H

/** \file spline.h
  * \brief POSIX spline computation library
  * \author Ralf Kaestner
  * A POSIX-compliant bicubic spline computation library.
  */

#include <stdlib.h>
#include <stdio.h>

/** Predefined spline error codes
  */
#define SPLINE_ERROR_NONE                  0
#define SPLINE_ERROR_FILE_OPEN             1
#define SPLINE_ERROR_FILE_FORMAT           2
#define SPLINE_ERROR_FILE_CREATE           3
#define SPLINE_ERROR_FILE_WRITE            4
#define SPLINE_ERROR_UNDEFINED             5

/** \brief Spline evaluation type
  */
typedef enum {
  spline_base_function = 0,
  spline_first_derivative = 1,
  spline_second_derivative = 2,
} spline_eval_type_t;

/** \brief Predefined spline error descriptions
  */
extern const char* spline_errors[];

/** \brief Structure defining a spline segment
  * \note A spline segment is defined by f(x) = a*x^3+b*x^2+c*x+d.
  */
typedef struct spline_segment_t {
  double a;                    //!< The spline segments cubic coefficient.
  double b;                    //!< The spline segments quadratic coefficient.
  double c;                    //!< The spline segments linear coefficient.
  double d;                    //!< The spline segments constant.

  double arg_width;            //!< The argument interval width of the segment.
} spline_segment_t, *spline_segment_p;

/** \brief Structure defining the spline
  * \note The first spline segment is assumed to start at zero.
  */
typedef struct spline_t {
  ssize_t num_segments;        //!< The number of spline segments.

  spline_segment_p segments;   //!< The segments of the spline.
  double* arg_start;           //!< The start of the spline segments.
} spline_t, *spline_p;

/** \brief Initialize a spline segment
  * \param[in] segment The spline segment to be initialized with zero
  *   coefficients and width one.
  */
void spline_init_segment(
  spline_segment_p segment);

/** \brief Initialize an empty bicubic spline
  * \param[in] spline The bicubic spline to be initialized.
  */
void spline_init(
  spline_p spline);

/** \brief Destroy a bicubic spline
  * \param[in] spline The bicubic spline to be destroyed.
  */
void spline_destroy(
  spline_p spline);

/** \brief Print a spline segment
  * \param[in] stream The output stream that will be used for printing the
  *   spline segment.
  * \param[in] segment The spline segment that will be printed.
  */
void spline_print_segment(
  FILE* stream,
  spline_segment_p segment);

/** \brief Print a bicubic spline
  * \param[in] stream The output stream that will be used for printing the
  *   bicubic spline.
  * \param[in] spline The bicubic spline that will be printed.
  */
void spline_print(
  FILE* stream,
  spline_p spline);

/** \brief Read bicubic spline from file
  * \note A spline will be allocated and must be destroyed by the caller.
  * \param[in] filename The name of the file containing the bicubic spline.
  * \param[out] spline The read bicubic spline.
  * \return The number of bicubic spline segments read from the file
  *   or the negative error code.
  */
int spline_read(
  const char* filename,
  spline_p spline);

/** \brief Write bicubic spline to file
  * \param[in] filename The name of the file the bicubic spline will be 
  *   written to.
  * \param[in] spline The bicubic spline to be written.
  * \return The number of bicubic spline segments written to the file
  *   or the negative error code.
  */
int spline_write(
  const char* filename,
  spline_p spline);

/** \brief Add a spline segment
  * \param[in] spline The bicubic spline the segment will be added to.
  * \param[in] segment The spline segment to be added to the bicubic spline.
  */
void spline_add_segment(
  spline_p spline,
  spline_segment_p segment);

/** \brief Evaluate a spline segment for a given argument
  * \param[in] spline The bicubic spline containing the segment to be evaluated.
  * \param[in] type The evaluation type to be used.
  * \param[in] seg_index The index of the segment to be evaluated.
  * \param[in] argument The argument for which to evaluate the spline segment.
  * \return The value of the spline segment for the given argument.
  */
double spline_evaluate_segment(
  spline_p spline,
  spline_eval_type_t type,
  int seg_index,
  double argument);

/** \brief Evaluate the spline for a given argument using linear search
  * \note This method uses linear search on the spline to identify the 
  *   segment to be evaluated.
  * \param[in] spline The bicubic spline to be evaluated.
  * \param[in] type The evaluation type to be used.
  * \param[in] argument The argument for which to evaluate the bicubic spline.
  * \param[in] seg_index The index of the segment to start with the 
  *   linear search.
  * \param[out] value The value of the bicubic spline for the given argument.
  * \return The index of the evaluated bicubic spline segment or the negative 
  *   error code.
  */
int spline_evaluate_linear_search(
  spline_p spline,
  spline_eval_type_t type,
  double argument,
  int seg_index,
  double* value);

#endif
