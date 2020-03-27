/*
  Intelligent Snakes

  FILE: point.cc
  DESCRIPTION: defines Point class,
               a glorified point
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 7/11/2000
*/

#include "point.h"

Point::Point() {
  x = y = INFINITY;
  cost = INFINITY;
  accessor = NONE;
  previous = next = NULL;
}

Point::Point(int x, int y) {
  this->x = x;
  this->y = y;
  cost = INFINITY;
  accessor = NONE;
  previous = next = NULL;
}

Point::Point(int x, int y, Point *previous, float cost) {
  this->x = x;
  this->y = y;
  this->previous = previous;
  this->cost = cost;
  accessor = NONE;
  next = NULL;
}
