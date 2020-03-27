/*
  Intelligent Snakes

  FILE: main.cc
  DESCRIPTION: main function,
               initializes image and snake and displays main window
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 7/17/2000
*/

#include "glutMaster.h"
#include "main.h"
#include <stdio.h>

GlutMaster *glutmaster;
MainWindow *mainwindow;
Image *image;
ISnake *isnake;

int main(int argc, char *argv[]) {
  if(argc > 1)
    image = new Image(argv[1]);
  else
    image = new Image("00080.pgm");
  glutmaster = new GlutMaster();
  mainwindow = new MainWindow(glutmaster, image);
  glutmaster->CallGlutMainLoop();
}
