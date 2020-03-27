/*
  Intelligent Snake

  FILE: displayWindow.h
  DESCRIPTION: prototypes for DisplayWindow class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/07/2000
*/

#ifndef DISPLAYWINDOW_H
#define DISPLAYWINDOW_H

#include "glutMaster.h"
#include "image.h"
#include <stdio.h>
#include <memory.h>
#include <stdlib.h>

class DisplayWindow : public GlutWindow {
 public:
  Image *image;

  DisplayWindow(GlutMaster *glutmaster, Image *image);
  ~DisplayWindow();

  void CallBackDisplayFunc(void);
  void CallBackReshapeFunc(int w, int h);
  void CallBackMenuFunc(int value);
  void CallBackKeyboardFunc(unsigned char key, int x, int y);
  void setImage(Image *image);
  void writePPM();
};

#endif
