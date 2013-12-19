/********************************************************************
* filename: kbhit.c                                                 *
* purpose : simulate the kbhit() and getch() functions              *
*           known from DOS                                          *
* Author  : Christian Schoett                                       *
* Date    : 20.02.2001                                              *
* Version : 1.0                                                     *
********************************************************************/

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "kbhit.h"

static struct termios orig, tnew;
static int peek = -1;

void kbdinit(void)
{
  tcgetattr(0, &orig);
  tnew = orig;
  tnew.c_lflag &= ~ICANON;
  tnew.c_lflag &= ~ECHO;
  tnew.c_lflag &= ~ISIG;
  tnew.c_cc[VMIN] = 1;
  tnew.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &tnew);
}

void kbdexit(void)
{
  tcsetattr(0,TCSANOW, &orig);
}

int kbhit(void)
{
  char ch;
  int nread = -1;

  if(peek != -1) return 1;
  tnew.c_cc[VMIN]=0;
  tcsetattr(0, TCSANOW, &tnew);
  nread = read(0,&ch,1);
  tnew.c_cc[VMIN]=1;
  tcsetattr(0, TCSANOW, &tnew);

  if(nread == 1) {
   peek = ch;
   return nread;
  }

  return 0;
}

int readch(void)
{
  char ch;

  if(peek != -1) {
    ch = peek;
    peek = -1;
    return ch;
  }

  read(0,&ch,1);
  return ch;
}


#if 0
/* a 'main' routine to test the above functions */
int main()
{
  int ch  = ' ';
  
  kbdinit();
  while(ch != 'q') {
   if(kbhit()){
      ch = readch();
      if(ch == 0x1B){
        ch  = readch();
        ch  = readch();
        ch  = readch();
        if(ch == '1')
        printf("you hit F1\n");
          else if(ch == '2')
        printf("you hit F2\n");
          else if(ch == '3')
        printf("you hit F3\n");
          else if(ch == '4')
        printf("you hit F4\n");
          else if(ch == '5')
        printf("you hit F5\n");
          else if(ch == '7')
        printf("you hit F6\n");
          else if(ch == '8')
        printf("you hit F7\n");
          else if(ch == '9')
        printf("you hit F8\n");
        ch  = readch();
      }
      else 
        printf("you hit %2.2X %d, %c\n", ch, ch, ch);
    }
  }
  kbdexit();
  exit(0);
}
#endif

