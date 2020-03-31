/*
 * fifo.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#include "fifo.h"
#include "gpio.h"
#include <stddef.h>
#include <stdlib.h>

//----- FIFO Append -----
void FIFO_Append(FIFO_SetptFIFO_t* fifo, GPIO_BTNState_t btn0, GPIO_BTNState_t btn1){
	struct FIFO_SetptNode_t* newNode = (struct FIFO_SetptNode_t*) malloc(sizeof(struct FIFO_SetptNode_t)); //Allocate a new node on the heap

	if(newNode != NULL) {								//Ensure memory successfully allocated
		newNode->btn0_state = btn0;						//Initialize new node
		newNode->btn1_state = btn1;
		newNode->next = NULL;

		if(FIFO_IsEmpty(fifo)) {						//List empty?
			fifo->head = newNode;						//First entry, tail and head point to same node
			fifo->tail = newNode;
		}
		else {
			fifo->tail->next = newNode;					//Set current tail node to point to new node
			fifo->tail = newNode;						//Set tail to new node
	}
	}
}


//----- FIFO Peek -----
struct FIFO_SetptNode_t* FIFO_Peek(FIFO_SetptFIFO_t* fifo) {
	if(FIFO_IsEmpty(fifo)) return NULL;				//If fifo is empty, nothing to pop
	else return fifo->head;							//Return pointer to head of the list
}


//----- FIFO Pop ------
void FIFO_Pop(FIFO_SetptFIFO_t* fifo) {
	if(FIFO_IsEmpty(fifo)) return;							//If fifo is empty, nothing to pop return

	if(fifo->head == fifo->tail) {							//If only one entry in fifo
		free(fifo->head);									//Free node
		fifo->head = NULL;									//Set head and tail to NULL (list empty)
		fifo->tail = NULL;
	}
	else {
		struct FIFO_SetptNode_t* next = fifo->head->next;			//Save next node in the lust (new head)
		free(fifo->head);									//Free current head of the list
		fifo->head = next;									//Set new head
	}
}


//----- FIFO empty? -----
bool FIFO_IsEmpty(FIFO_SetptFIFO_t* fifo) {
	if(fifo->head == NULL && fifo->tail == NULL) {        	//Check if fifo head and tail null
		return true;										//head and tail null -> empty
	}
	else return false;										//Head and tail != NULL -> not empty
}
