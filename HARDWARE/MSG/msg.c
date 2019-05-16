#include "msg.h"
#include "motor.h"
#include "can.h"
typedef struct Queue
{
    int speed;
	  float angle;
    struct Queue* next;
}Queue_Struct;

static Queue_Struct* head = NULL;
static Queue_Struct* tail = NULL;
static int count = 0;
int hhh;
int GM_Queue_Enqueue( int speed,float angle)
{
    Queue_Struct* tmp = (Queue_Struct*)malloc(sizeof(Queue_Struct));

    if (NULL == tmp)
    {
       
			return -1;
    }

    tmp->speed = speed;
		tmp->angle=angle;
    tmp->next = NULL;

    if (NULL == tail)
    {
        head = tmp;
    }
    else
    {
        tail->next = tmp;
    }

    tail = tmp;
    ++count;
    Prameter.flag++;
    return 1;
}

int GM_Queue_Dequeue( int* speed,float* angle )
{
    Queue_Struct* tmp = NULL;

    if ((NULL == head) )
    {
					hhh=89; 
        return -1;
	
    }

    *speed = head->speed;
		*angle=head->angle;
    tmp = head;

    if (head == tail)
    {
        head = NULL;
        tail = NULL;
    }
    else
    {
        head = head->next;
    }

    free(tmp);
    tmp = NULL;

    --count;
    Prameter.flag--;
    return 1;
}

void GM_Queue_Clear()
{
    while (count > 0)
    {
       // GM_Queue_Dequeue(&value);
    }
}

int GM_Queue_Length()
{
    return count;
}

