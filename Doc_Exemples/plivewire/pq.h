/*
  Intelligent Snakes

  FILE: pq.h
  DESCRIPTION: prototypes for PQ class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 7/7/2000
*/

#ifndef PQ_H
#define PQ_H

#include "point.h"

class PQ {
 public:
  Point **queue;
  long numpoints, numelements;

  PQ(long numpoints);
  ~PQ();

  int empty();
  Point *deleteMin();
  void addPoint(Point *newpoint);
  void setCost(Point *current, float cost);
  void percolateUp(int accessor);
  void percolateDown(int accessor);
};

#endif  
