/***************************************************************************
 *   Copyright (C) 2008 by Fritz Stoeckli, Ralf Kaestner                   *
 *   stfritz@ethz.ch, ralf.kaestner@gmail.com                              *
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

#include <string.h>

#include "spline.h"

#define sqr(a) ((a)*(a))
#define cub(a) ((a)*(a)*(a))

const char* spline_errors[] = {
  "success",
  "error opening file",
  "invalid file format",
  "error creating file",
  "error writing file",
  "value undefined",
};

void spline_init_segment(spline_segment_p segment) {
  segment->a = 0.0;
  segment->b = 0.0;
  segment->c = 0.0;
  segment->d = 0.0;

  segment->arg_width = 1.0;
}

void spline_init(spline_p spline) {
  memset(spline, 0, sizeof(spline_t));
}

void spline_destroy(spline_p spline) {
  if (spline->num_segments) {
    free(spline->segments);
    free(spline->arg_start);

    spline->num_segments = 0;
    spline->segments = 0;
    spline->arg_start = 0;
  }
}

void spline_print_segment(FILE* stream, spline_segment_p segment) {
  fprintf(stream, "%5s: %lg\n", "a", segment->a);
  fprintf(stream, "%5s: %lg\n", "b", segment->b);
  fprintf(stream, "%5s: %lg\n", "c", segment->c);
  fprintf(stream, "%5s: %lg\n", "d", segment->d);
  fprintf(stream, "%5s: %lg\n", "width", segment->arg_width);
}

void spline_print(FILE* stream, spline_p spline) {
  fprintf(stream, "%10s  %10s  %10s  %10s  %10s  %10s\n",
    "a",
    "b",
    "c",
    "d",
    "width",
    "start");

  int i;
  for (i = 0; i < spline->num_segments; i++) {
    fprintf(stream,
      "%10g  %10lg  %10lg  %10lg  %10lg  %10lg\n",
      spline->segments[i].a,
      spline->segments[i].b,
      spline->segments[i].c,
      spline->segments[i].d,
      spline->segments[i].arg_width,
      spline->arg_start[i]);
  }
}

int spline_read(const char* filename, spline_p spline) {
  int i, result;
  FILE* file;
  char buffer[1024];

  spline_segment_t segment;
  spline_init(spline);

  file = fopen(filename, "r");
  if (file == NULL)
    return -SPLINE_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      result = sscanf(buffer, "%lg %lg %lg %lg %lg",
        &segment.a,
        &segment.b,
        &segment.c,
        &segment.d,
        &segment.arg_width);

      if (result < 5) {
        fclose(file);
        return -SPLINE_ERROR_FILE_FORMAT;
      }

      spline_add_segment(spline, &segment);
    }
  }

  fclose(file);

  return spline->num_segments;
}

int spline_write(const char* filename, spline_p spline) {
  int i;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");
  if (file == NULL)
    return -SPLINE_ERROR_FILE_CREATE;

  for (i = 0; i < spline->num_segments; ++i) {
    sprintf(buffer, "%lg %lg %lg %lg %lg\n",
      spline->segments[i].a,
      spline->segments[i].b,
      spline->segments[i].c,
      spline->segments[i].d,
      spline->segments[i].arg_width);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -SPLINE_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return spline->num_segments;
}
 
void spline_add_segment(spline_p spline, spline_segment_p segment) {
  spline->segments = realloc(spline->segments,
    (spline->num_segments+1)*sizeof(spline_segment_t));
  spline->arg_start = realloc(spline->arg_start,
    (spline->num_segments+1)*sizeof(double));

  spline->segments[spline->num_segments] = *segment;
  spline->arg_start[spline->num_segments] = (spline->num_segments) ?
    spline->arg_start[spline->num_segments-1]+
    spline->segments[spline->num_segments-1].arg_width : 0.0;

  ++spline->num_segments;
}

double spline_evaluate_segment(spline_p spline, spline_eval_type_t type, 
  int seg_index, double argument) {
  spline_segment_p segment = &spline->segments[seg_index];
  double x = argument-spline->arg_start[seg_index];

  if (type == spline_first_derivative)
    return 3.0*segment->a*sqr(x)+2.0*segment->b*x+segment->c;
  else if (type == spline_second_derivative)
    return 6.0*segment->a*x+2.0*segment->b;
  else
    return segment->a*cub(x)+segment->b*sqr(x)+segment->c*x+segment->d;
}

int spline_evaluate_linear_search(spline_p spline, spline_eval_type_t type,
  double argument, int seg_index, double* value) {
  int i = seg_index;

  while ((i >= 0) && (i < spline->num_segments)) {
    if (argument >= spline->arg_start[i]) {
      if (argument <= spline->arg_start[i]+spline->segments[i].arg_width) {
        *value = spline_evaluate_segment(spline, type, i, argument);
        return i;
      }
      else
        ++i;
    }
    else
      --i;
  }
  
  return -SPLINE_ERROR_UNDEFINED;
}
