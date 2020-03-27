/*
  Intelligent Snakes

  FILE: image.h
  DESCRIPTION: prototypes for Image class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 6/28/2000
*/

#ifndef IMAGE_H
#define IMAGE_H

class Image {
 public:
  int width, height;
  unsigned char *raster;
  char *filename;
  //  char filename[300];

  Image(int width, int height, unsigned char *raster);
  Image(char *filename);
  int getWidth();
  int getHeight();
  unsigned char *getRaster();
  void medianFilter(int size);
};

#endif
