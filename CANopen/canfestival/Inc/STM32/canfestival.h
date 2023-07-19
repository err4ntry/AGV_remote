#ifndef __CAN_FESTIVAL_H
#define __CAN_FESTIVAL_H
#include "applicfg.h"
#include "data.h"

void initTimer(void);
void clearTimer(void);

unsigned char canSend(CAN_PORT notused, Message *m);
unsigned char canInit(CO_Data * d, uint32_t bitrate);
void InitNodes(CO_Data* d, UNS32 id);
void canClose(void);

void disable_it(void);
void enable_it(void);

#endif  /*__CAN_FESTIVAL_H*/
