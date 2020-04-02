/*
 * fifo.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#include "fifo.h"
#include "gpio.h"
#include "game.h"
#include <stddef.h>
#include <stdlib.h>

//----- FIFO Append -----
void FIFO_Append(WayPtFIFO_t* fifo, int xDiff){
	struct WayPt_t* newNode = (struct WayPt_t*) malloc(sizeof(struct WayPt)); //Allocate a new node on the heap

	if(newNode != NULL) {								//Ensure memory successfully allocated

		newNode->next = NULL;							//Node is at end of the list
		newNode->xPos = fifo->tail->xPos + xDiff;		//Add x delta to previous waypoint to get this x
		newNode->yPos = fifo->tail->yPos + WAYPT_YDIFF;	//Next way point is 5m

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
struct WayPt_t* FIFO_Peek(WayPtFIFO_t* fifo) {
	if(FIFO_IsEmpty(fifo)) return NULL;				//If fifo is empty, nothing to pop
	else return fifo->head;							//Return pointer to head of the list
}


//----- FIFO Pop ------
void FIFO_Pop(WayPtFIFO_t* fifo) {
	if(FIFO_IsEmpty(fifo)) return;							//If fifo is empty, nothing to pop return

	if(fifo->head == fifo->tail) {							//If only one entry in fifo
		free(fifo->head);									//Free node
		fifo->head = NULL;									//Set head and tail to NULL (list empty)
		fifo->tail = NULL;
	}
	else {
		struct WayPt_t* next = fifo->head->next;			//Save next node in the lust (new head)
		free(fifo->head);									//Free current head of the list
		fifo->head = next;									//Set new head
	}
}


//----- FIFO empty? -----
bool FIFO_IsEmpty(WayPtFIFO_t* fifo) {
	if(fifo->head == NULL && fifo->tail == NULL) {        	//Check if fifo head and tail null
		return true;										//head and tail null -> empty
	}
	else return false;										//Head and tail != NULL -> not empty
}
