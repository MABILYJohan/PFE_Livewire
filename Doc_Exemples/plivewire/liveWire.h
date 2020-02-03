/*
  Intelligent Snakes

  FILE: liveWire.h
  DESCRIPTION: prototypes for LiveWire class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/01/2000
*/

#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <math.h>
#include "image.h"
#include "point.h"
#include "pq.h"
#include "contour.h"
#include "glutMaster.h"

class LiveWire : public Contour {
 public:
  Point **points, *cursor;
  int width, height;
  unsigned char *raster;
  PQ Q;
  Contour controlpoints;
  double average, sdev;
  double averageb, sdevb;
  int costcap;
  float gausstable[256];
  float gausstableb[256];

  LiveWire(Image *image);
  ~LiveWire();

  void resetGauss();
  void deleteControl();
  void addPointAt(int x, int y);
  Image *getCostImage();
  float f6(unsigned char t, unsigned char p, unsigned char v,
	   unsigned char u, unsigned char q, unsigned char w);
  float fb(unsigned char t, unsigned char p, unsigned char v,
	   unsigned char u, unsigned char q, unsigned char w);
  float gauss(float x, float mean, float sd);
  float gaussb(float x, float mean, float sd);
  float partialCost(Point *from, Point *to);
  float partialCostb(Point *from, Point *to);
  float cost(Point *from, Point *to);
  float getCost(int x, int y);
  void train(Contour *boundary);
  void train();
  void initPoints();
  void shortestPathsAt(int x, int y);
  void shortestPaths(Point *start);
  void draw();
  void moveCursor(int x, int y);
  void closeContour();
};

#endif






