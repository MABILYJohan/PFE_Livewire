/*
  Intelligent Snakes

  FILE: pq.cc
  DESCRIPTION: defines PQ class,
               a simple priority queue
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 7/13/2000
*/

#include "pq.h"
#include <stdio.h>

PQ::PQ(long numpoints) {
  long i;

  this->numpoints = numpoints + 1;

  queue = new Point *[numpoints * 2];
  if(!queue)
    fprintf(stderr, "Mayday!\n");
  for(i = 0; i < numpoints * 2; i++)
    queue[i] = NULL;
  numelements = 1;
}

PQ::~PQ() {
  delete[] queue;
}

int PQ::empty() {
  if(numelements==1)
    return 1;
  else
    return 0;
}

Point *PQ::deleteMin() {
  Point *answer = queue[1];
  answer->accessor = NONE;

  if(--numelements > 1) {
    queue[1] = queue[numelements];
    queue[1]->accessor = 1;
    queue[numelements] = NULL;
    percolateDown(1);
  }
  return answer;
}

void PQ::addPoint(Point *newpoint) {
  if(queue[0])
    fprintf(stderr, "error");
  newpoint->accessor = numelements;
  queue[numelements] = newpoint;
  percolateUp(numelements++);
  if(queue[newpoint->accessor] != newpoint)
    fprintf(stderr, "fuckme");
}

void PQ::setCost(Point *current, float newcost) {
  current->cost = newcost;
  if(current->accessor == NONE)
    addPoint(current);
  else {
    if(!queue[current->accessor]) {
      fprintf(stderr, "FUCK YOU! %d\n", numelements);
      for(int i = 0; i < numpoints; i++)
	if(queue[i] == current)
	  fprintf(stderr, "Got it %d\n", i);
    }
    if(queue[current->accessor] != current)
      fprintf(stderr, "Well, fuck you too, %d.", current->accessor);
    percolateUp(current->accessor);
    percolateDown(current->accessor);
    if(queue[current->accessor] != current)
      fprintf(stderr, "fuck me here");
  }
}

void PQ::percolateUp(int accessor) {
  Point *temp;
  float parentcost;

  parentcost = (accessor/2)?queue[accessor/2]->cost:-1;

  if(!queue[accessor])
    fprintf(stderr, "error0");

  while(parentcost > queue[accessor]->cost) {
    if(accessor / 2 < 1)
      fprintf(stderr, "error1");
    if(!queue[accessor/2])
      fprintf(stderr, "error2");
    temp = queue[accessor/2];
    queue[accessor/2] = queue[accessor];
    queue[accessor] = temp;
    queue[accessor/2]->accessor = accessor/2;
    queue[accessor]->accessor = accessor;
    accessor /= 2;
    parentcost = (accessor/2)?queue[accessor/2]->cost:-1;
  }
}

void PQ::percolateDown(int accessor) {
  Point *temp;
  float leftcost, rightcost;

  leftcost = queue[accessor * 2]?queue[accessor * 2]->cost:INFINITY;
  rightcost = queue[accessor * 2 + 1]?queue[accessor * 2 + 1]->cost:INFINITY;

  while(leftcost < queue[accessor]->cost ||
	rightcost < queue[accessor]->cost) {
    if(leftcost < rightcost) {
      temp = queue[accessor*2];
      queue[accessor*2] = queue[accessor];
      queue[accessor] = temp;
      queue[accessor*2]->accessor = accessor * 2;
      queue[accessor]->accessor = accessor;
      accessor *= 2;
    }
    else {
      temp = queue[accessor*2+1];
      queue[accessor*2+1] = queue[accessor];
      queue[accessor] = temp;
      queue[accessor*2+1]->accessor = accessor * 2 + 1;
      queue[accessor]->accessor = accessor;
      accessor = accessor * 2 + 1;
    }
    leftcost = queue[accessor * 2]?queue[accessor * 2]->cost:INFINITY;
    rightcost = queue[accessor * 2 + 1]?queue[accessor * 2 + 1]->cost:INFINITY;
  }
}








