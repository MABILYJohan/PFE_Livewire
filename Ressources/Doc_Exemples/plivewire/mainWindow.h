/*
  Intelligent Snake

  FILE: mainWindow.h
  DESCRIPTION: prototypes for MainWindow class
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/07/2000
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory.h>
#include "glutMaster.h"
#include "image.h"
#include "snake.h"
#include "isnake.h"
#include "point.h"
#include "liveWire.h"
#include "displayWindow.h"

class MainWindow : public GlutWindow {
 public:
  DisplayWindow *display;
  GlutMaster *glutmaster;
  Image *image;
  int width, height, painting;
  Snake *snake;
  LiveWire *livewire;
  Contour *paint, *highcurvature;

  MainWindow(GlutMaster *glutmaster, Image *image);
  ~MainWindow();

  void next(int dir);
  void newLiveWire();
  void CallBackDisplayFunc(void);
  void writePPM();
  void CallBackMenuFunc(int value);
  void CallBackReshapeFunc(int w, int h);
  void CallBackMouseFunc(int button, int state, int x, int y);
  void CallBackMotionFunc(int x, int y);
  void CallBackPassiveMotionFunc(int x, int y);
  void CallBackKeyboardFunc(unsigned char key, int x, int y);

  void reset();
};

#endif

