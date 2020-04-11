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
void FIFO_Append(WayPtFIFO_t* fifo, int xDiff, bool useXDiff){
	struct WayPt_t* newNode = (struct WayPt_t*) malloc(sizeof(struct WayPt_t)); //Allocate a new node on the heap

	if(newNode != NULL) {									//Ensure memory successfully allocated

		newNode->next = NULL;								//Node is at end of the list

		if(FIFO_IsEmpty(fifo)) {							//List empty?
			newNode->xPos = 0;
			newNode->yPos = 0;
			fifo->head = newNode;							//First entry, tail and head point to same node
			fifo->tail = newNode;
		}
		else {
			if(useXDiff) {
				newNode->xPos = fifo->tail->xPos + xDiff;	//Add x delta to previous waypoint to get this x
			}
			else {
				newNode->xPos = xDiff;						//Use x value passed in as true x value, not difference.
			}
			newNode->yPos = fifo->tail->yPos + WAYPT_YDIFF;	//Next way point is 5m
			fifo->tail->next = newNode;						//Set current tail node to point to new node
			fifo->tail = newNode;							//Set tail to new node
		}

		fifo->currWayPts++;
		fifo->totalWayPts--;
	}
}


//----- FIFO Peek -----
struct WayPt_t* FIFO_Peek(WayPtFIFO_t* fifo) {
	if(FIFO_IsEmpty(fifo)) return NULL;						//If fifo is empty, nothing to pop
	else return fifo->head;									//Return pointer to head of the list
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
	fifo->currWayPts--;
}


//----- FIFO empty? -----
bool FIFO_IsEmpty(WayPtFIFO_t* fifo) {
	if(fifo->head == NULL && fifo->tail == NULL) {        	//Check if fifo head and tail null
		return true;										//head and tail null -> empty
	}
	else return false;										//Head and tail != NULL -> not empty
}
