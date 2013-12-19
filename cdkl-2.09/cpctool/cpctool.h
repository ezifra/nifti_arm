#ifndef CPCTOOL_H
#define CPCTOOL_H

#define CPCMLEN  (cpcmsg->length)
#define CPCMTYPE (cpcmsg->type)

// can message
#define ID   (cpcmsg->msg.canmsg.id)
#define LEN  (cpcmsg->msg.canmsg.len)
#define MSG0 (cpcmsg->msg.canmsg.msg[0])
#define MSG1 (cpcmsg->msg.canmsg.msg[1])
#define MSG2 (cpcmsg->msg.canmsg.msg[2])
#define MSG3 (cpcmsg->msg.canmsg.msg[3])
#define MSG4 (cpcmsg->msg.canmsg.msg[4])
#define MSG5 (cpcmsg->msg.canmsg.msg[5])
#define MSG6 (cpcmsg->msg.canmsg.msg[6])
#define MSG7 (cpcmsg->msg.canmsg.msg[7])

// generic message
#define G_MSG0 (cpcmsg->msg.genericmsg[0])
#define G_MSG1 (cpcmsg->msg.genericmsg[1])
#define G_MSG2 (cpcmsg->msg.genericmsg[2])
#define G_MSG3 (cpcmsg->msg.genericmsg[3])
#define G_MSG4 (cpcmsg->msg.genericmsg[4])
#define G_MSG5 (cpcmsg->msg.genericmsg[5])
#define G_MSG6 (cpcmsg->msg.genericmsg[6])
#define G_MSG7 (cpcmsg->msg.genericmsg[7])

/* utility.c */
void get_can_msg(CPC_CAN_MSG_T * cmsg);

#define ASK_YES 1
#define ASK_NO  0
unsigned char ask(char * ask_what, unsigned char mode);

#endif
