#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/time.h>
#include <unistd.h>
#include <ctype.h>

#include "cpc.h"
#include "cpctool.h"
#include "kbhit.h"

/***********************************************************/
void get_can_msg(CPC_CAN_MSG_T * cmsg)
{ char zchn;
  unsigned int i, tmp;

  printf("Hex or Dec? (h/d)?\n");
  do{
    zchn=toupper(readch());
  }while(zchn!='H' && zchn!='D');

  kbdexit();

  if(zchn=='D'){
    printf("\nId: ");
    scanf("%lu",&cmsg->id);
    printf("Len: ");
    scanf("%u", &tmp);
    cmsg->length = (unsigned char)tmp;
    for(i=0;i<cmsg->length;i++){
     printf("D[%d]:",i);
     scanf("%d",&tmp);
     cmsg->msg[i] = (unsigned char)tmp;
    }
  }
  else{
    printf("\nId: ");
    scanf("%lx",&cmsg->id);
    printf("Len: ");
    scanf("%x",&tmp);
    cmsg->length = (unsigned char)tmp;
    for(i=0;i<cmsg->length;i++){
     printf("D[%x]:",i);
     scanf("%x",&tmp);
     cmsg->msg[i] = (unsigned char)tmp;
    }
  }
  
  kbdinit();

}


/***********************************************************/
unsigned char ask(char * ask_what, unsigned char mode)
{
   char zchn;

   if(mode == ASK_NO)
     printf("%s (y/N)\n",ask_what);
   else
     if(mode == ASK_YES)
       printf("%s (Y/n)\n",ask_what);

   do{
    zchn = getchar();
    zchn = toupper(zchn);
   }while(!((zchn == 'Y') || (zchn == 'N') || (zchn==0x0d)));

   if(zchn == 'Y')
    return ASK_YES;
   if(zchn == 'N')
    return ASK_NO;
   if(zchn == 0x0d)
    return mode;
    
   return mode;
   
}
