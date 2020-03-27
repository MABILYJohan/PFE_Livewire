/*
  Intelligent Snakes

  FILE: isnake.h
  DESCRIPTION: prototypes for ISnake class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/07/2000
*/

#ifndef ISNAKE_H
#define ISNAKE_H

#include "snake.h"
#include "image.h"
#include "liveWire.h"

class ISnake : public Snake {
 public:
  LiveWire *livewire;
  int resolution;

  ISnake();
  ISnake(Image *image);
  ISnake(Contour *contour);
  ISnake(Contour *contour, Image *image);
  ISnake(Contour *contour, Image *image, LiveWire *livewire, int resolution);

  void minimize();
  double firstderiv(Point *a, Point *b);
  double seconderiv(Point *a, Point *b, Point *c);
  void draw();
};

#endif


