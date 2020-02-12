/*
  Intelligent Snakes

  FILE: point.h
  DESCRIPTION: prototypes for Point class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 7/7/2000
*/

#ifndef POINT_H
#define POINT_H

#include <stdlib.h>

#define INFINITY 32000
#define NONE -1
#define DONE -2

class Point {
 public:
  int x, y, accessor;
  Point *previous, *next;
  float cost;

  Point();
  Point(int x, int y);
  Point(int x, int y, Point *previous, float cost);
};

#endif
