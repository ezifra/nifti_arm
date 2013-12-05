/*
 * tools.h
 *
 *  Created on: Feb 21, 2012
 *      Author: daniel
 */

#ifndef TOOLS_H_
#define TOOLS_H_

const char *byte_to_binary(short x)
{
  static char b[54];
  b[0] = '\0';

  int z;
  for (z = (1<<15); z > 0; z >>= 1)
  {
    strcat(b, ((x & z) == z) ? "1" : "0");
    strcat(b, " |");
  }

  return b;
}

#endif /* TOOLS_H_ */
