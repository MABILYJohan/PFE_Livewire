/*
  Intelligent Snake

  FILE: displayWindow.cc
  DESCRIPTION: defines DisplayWindow class,
               displays an Image and takes screenshots
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/07/2000
*/

#include "displayWindow.h"

void DisplayWindow::writePPM() {
  int i = 0;
  FILE *output;
  int width = this->width * 2;
  int height = this->height * 2;
  char *filename = new char[15];

  while((i == 0 || output != NULL) && i < 10000) {
    fclose(output);
    sprintf(filename, "output%d.ppm", i++);
    output = fopen(filename, "r");
  }

  sprintf(filename, "output%d.ppm", i-1);
  output = fopen(filename, "wb");

  unsigned char *pixmap = (unsigned char *)malloc(width * height * 3);
  glReadBuffer(GL_FRONT_LEFT);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  for(i = 0; i < height; i++)
    glReadPixels(0, i, width, 1, GL_RGB, GL_UNSIGNED_BYTE, pixmap + width * (height - i - 1) * 3);

  fprintf(output, "P6\n%d %d 255\n", width, height);
  fwrite(pixmap, 1, width * height * 3, output);

  fclose(output);
}

DisplayWindow::DisplayWindow(GlutMaster *glutmaster, Image *image) {
  this->image = image;
  this->width = image->getWidth();
  this->height = image->getHeight();

  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(width * 2, height * 2);
  glViewport(0, 0, width * 2, height * 2);
  glClearColor(0, 0, 0, 0);

  glutmaster->CallGlutCreateWindow("Cost Function", this);
  glMatrixMode(GL_PROJECTION);
  gluOrtho2D(0, width, height, 0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glRasterPos2i(0, 0);
  glPixelZoom(2, -2);

  glutAddMenuEntry("Screen Capture [s]", 's');
  glutAttachMenu(GLUT_RIGHT_BUTTON);
}

DisplayWindow::~DisplayWindow() {
  glutDestroyWindow(windowID);
}

void DisplayWindow::CallBackKeyboardFunc(unsigned char key, int x, int y) {
  switch(key) { 
    case 's':
    case 'S':
      writePPM();
      break;
  }
}

void DisplayWindow::CallBackDisplayFunc(void) {
  glClear(GL_COLOR_BUFFER_BIT);
  glDrawPixels(image->width, image->height, GL_LUMINANCE, GL_UNSIGNED_BYTE,
	       image->getRaster());

  glutSwapBuffers();
}

void DisplayWindow::CallBackReshapeFunc(int w, int h) {
  glutReshapeWindow(width * 2, height * 2);
  glutPostRedisplay();
}

void DisplayWindow::CallBackMenuFunc(int value) {
  CallBackKeyboardFunc((unsigned char)value, 0, 0);
}

void DisplayWindow::setImage(Image *image) {
  int oldwin = glutGetWindow();

  this->image = image;
  glutSetWindow(windowID);
  glutPostRedisplay();
  glutSetWindow(oldwin);
}

