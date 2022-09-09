/*
 * PID.h
 *
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

typedef struct _command{
	int direction; // 0-ST,
	int dist;
} Cmd;

typedef struct _listnode{
   Cmd item;
   struct _listnode *next;
} ListNode;

typedef struct _linkedlist{
   int size;
   ListNode *head;
   ListNode *tail;
} LinkedList;

typedef struct _queue{
	LinkedList ll;
} Queue;

ListNode * findNode(LinkedList *ll, int index);
int insertNode(LinkedList *ll, int index, Cmd value);
int removeNode(LinkedList *ll, int index);
void queue_init(Queue *q);
void enqueue(Queue *q, Cmd item);
Cmd dequeue(Queue *q);
int isEmptyQueue(Queue *s);

#endif /* INC_QUEUE_H_ */
