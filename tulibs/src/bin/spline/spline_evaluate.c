/***************************************************************************
 *   Copyright (C) 2004 by Ralf Kaestner                                   *
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

#include <stdio.h>

#include <spline.h>

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s FILE STEPSIZE [TYPE]\n", argv[0]);
    return -1;
  }
  const char* file = argv[1];
  double step_size = atof(argv[2]);
  spline_eval_type_t type = spline_base_function;
  if (argc == 4)
    type = atoi(argv[3]);

  spline_t spline;

  int result;
  if ((result = spline_read(file, &spline)) < 0)
    fprintf(stderr, "%s\n", spline_errors[-result]);
  double x = 0.0, f_x;
  int i = 0;
  while ((i = spline_evaluate_linear_search(&spline, type, x, i, &f_x)) >= 0) {
    fprintf(stdout, "%lf %lf\n", x, f_x);
    x += step_size;
  }

  spline_destroy(&spline);
  return 0;
}
