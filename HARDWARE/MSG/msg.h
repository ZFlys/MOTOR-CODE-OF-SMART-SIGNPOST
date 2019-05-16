#ifndef _MSG_H
#define _MSG_H

#include <stdlib.h>
extern int GM_Queue_Enqueue( int speed,float angle);
extern int GM_Queue_Dequeue( int* speed,float* angle );
void GM_Queue_Clear(void);
int GM_Queue_Length(void);

#endif 
