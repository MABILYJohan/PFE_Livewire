/*
  Intelligent Snakes

  FILE: contour.h
  DESCRIPTION: prototypes for Contour class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/01/2000
*/

#ifndef CONTOUR_H
#define CONTOUR_H

#include "point.h"
#include "glutMaster.h"
#include <stdio.h>
#include <math.h>

class Contour {
 public:
  Point *start, *end;
  int numpoints, visible;
  int closed;
  
  Contour();
  Contour(Contour *contour);
  Contour(Point *start);
  Contour(char *filename);
  virtual ~Contour();

  Point *closestPoint(int x, int y);
  void writeCNT(char *filename);
  virtual void draw();
  virtual void reset();
  virtual void addPointAt(int x, int y);
  void addPoint(Point *newpoint);
  void deletePointAt(int x, int y);
  void deletePoint(Point *toremove);
  Contour *highCurvature(int resolution, int threshold);
  void setVisible(int visible);
  char *getName();
  virtual void closeContour();
};

#endif

