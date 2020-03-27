/*
  Intelligent Snakes

  FILE: snake.h
  DESCRIPTION: prototypes for Snake class,
               a generic active contour,
	       abstract
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/07/2000
*/

#ifndef SNAKE_H
#define SNAKE_H

#include "contour.h"
#include "image.h"

class Snake : public Contour {
 public:
  Image *image;
  int minimized;

  Snake() {}
  Snake(Contour *contour) : Contour(contour->start) {};
  Snake(Image *image) {
    this->image = image;
  }

  void setImage(Image *image) {
    this->image = image;
  }
  virtual void minimize() = 0;
};

#endif
