/*
  Intelligent Snakes

  FILE: image.cc
  DESCRIPTION: defines Image class,
               stores and reads PGM files
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 7/17/2000
*/

#include "image.h"
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>

Image::Image(int width, int height, unsigned char *raster) {
  this->width = width;
  this->height = height;
  this->raster = raster;
}

Image::Image(char *filename) {
  this->filename = new char[300];
  strcpy(this->filename, filename);
  FILE *pgm = fopen(this->filename, "rb");
  fscanf(pgm, "P5\n%ld %ld 255\n", &width, &height);
  raster = new unsigned char[width * height];
  fread(raster, width * height, 1, pgm);
  fclose(pgm);
}

int Image::getWidth() {
  return width;
}

int Image::getHeight() {
  return height;
}

unsigned char *Image::getRaster() {
  return raster;
}

void Image::medianFilter(int size) {
  unsigned char filter[size*size], temp;
  unsigned char *newraster = new unsigned char[width * height];
  int i, j, x, y, done;

  for(x = 0; x < size/2; x++)
    for(y = 0; y < height; y++)
      newraster[x + y * width] = raster[x + y * width];
  for(x = width - size/2; x < width; x++)
    for(y = 0; y < height; y++)
      newraster[x + y * width] = raster[x + y * width];
  for(x = 0; x < width; x++)
    for(y = 0; y < size/2; y++)
      newraster[x + y * width] = raster[x + y * width];
  for(x = 0; x < width; x++)
    for(y = height - size/2; y < height; y++)
      newraster[x + y * width] = raster[x + y * width];

  for(x = size/2; x < width - size/2; x++)
    for(y = size/2; y < height - size/2; y++) {
      for(i = -size/2; i <= size/2; i++)
	for(j = -size/2; j <= size/2; j++)
	  filter[(i + size/2) * size + j + size/2] = raster[x + i + (y + j) * width];
      done = 0;
      while(!done) {
	done = 1;
	for(i = 0; i < size * size - 1; i++)
	  if(filter[i + 1] < filter[i]) {
	    done = 0;
	    temp = filter[i];
	    filter[i] = filter[i + 1];
	    filter[i + 1] = temp;
	  }
      }
      newraster[x + y * width] = filter[(size * size)/2];
    }
  delete raster;
  raster = newraster;
}







