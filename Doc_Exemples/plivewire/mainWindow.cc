/*
  Intelligent Snake

  FILE: mainWindow.cc
  DESCRIPTION: defines MainWindow class,
               handles display and user interface
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/01/2000
*/

#include <math.h>
#include "mainWindow.h"
#include <stdio.h>
#include <strings.h>

void MainWindow::CallBackMenuFunc(int value) {
    CallBackKeyboardFunc((unsigned char)value, 0, 0);
}

void MainWindow::writePPM() 
{
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

MainWindow::MainWindow(GlutMaster *glutmaster, Image *image) {
    long numpoints, i;

    this->glutmaster = glutmaster;
    this->image = image;
    this->width = image->getWidth();
    this->height = image->getHeight();
    this->paint = NULL;
    this->livewire = new LiveWire(image);
    this->snake = NULL;
    this->highcurvature = NULL;
    this->display = NULL;

    livewire->setVisible(1);

    painting = 0;

    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(width * 2, height * 2);
    glViewport(0, 0, width * 2, height * 2);
    glClearColor(0, 0, 0, 0);

    glutmaster->CallGlutCreateWindow("Intelligent Snake", this);
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, width, height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRasterPos2i(0, 0);
    glPixelZoom(2, -2);

    glutAddMenuEntry("Train Livewire [t]", 't');
    glutAddMenuEntry("Delete Last Control Point [DEL]", 127);
    glutAddMenuEntry("Snakify [s]", 's');
    glutAddMenuEntry("Minimize Snake [m]", 'm');
    glutAddMenuEntry("Livewirify [l]", 'l');
    glutAddMenuEntry("3x3 Median Filter [3]", '3');
    glutAddMenuEntry("5x5 Median Filter [5]", '5');
    glutAddMenuEntry("Toggle Cost Image Display [c]", 'c');
    glutAddMenuEntry("Previous Slice [p]", 'p');
    glutAddMenuEntry("Next Slice [n]", 'n');
    glutAddMenuEntry("Reset Contours [r]", 'r');
    glutAddMenuEntry("Write Screenshot [w]", 'w');
    glutAddMenuEntry("Exit [ESC]", 27);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

MainWindow::~MainWindow() {
    delete snake;
    delete paint;
    delete livewire;
    delete image;
    image = NULL;
    delete display;
    glutDestroyWindow(windowID);
}

void MainWindow::reset() {
    if(snake) {
        delete snake;
        snake = NULL;
    }
    if(livewire) {
        delete livewire;
        livewire = new LiveWire(image);
        livewire->setVisible(1);
    }
    if(highcurvature) {
        delete highcurvature;
        highcurvature = NULL;
    }
    if(paint) {
        delete paint;
        paint = NULL;
        painting = 0;
    }
    if(display) {
        delete display;
        display = NULL;
    }
    glutPostRedisplay();
}

void MainWindow::newLiveWire() {
    double average = livewire->average, sdev = livewire->sdev;
    double averageb = livewire->averageb, sdevb = livewire->sdevb;
    double cost;
    int i;
    Point *current = snake->end;

    delete livewire;
    delete highcurvature;
    highcurvature = NULL;
    livewire = new LiveWire(image);
    livewire->average = average;
    livewire->sdev = sdev;
    livewire->averageb = averageb;
    livewire->sdevb = sdevb;
    livewire->setVisible(1);
    for(i = 0; i < snake->numpoints; i++) {
        if(i) {
            livewire->costcap = 40;
            while(livewire->getCost(current->x, current->y) == INFINITY && livewire->costcap < INFINITY) {
                livewire->costcap *= 2;
                livewire->shortestPathsAt(current->next->x, current->next->y);
            }
            if(cost < INFINITY) {
                cost += livewire->getCost(current->x, current->y);
                livewire->addPointAt(current->x, current->y);
            }
            else fprintf(stderr, "skipping one\n");
        }
        else
            livewire->addPointAt(current->x, current->y);
        current = current->previous;
    }
    livewire->closeContour();
    delete snake;
    snake = NULL;
    fprintf(stderr, "%d\n", (int)cost);
}

void MainWindow::next(int dir) {
    char str[300];

    sprintf(str, "%.5d.pgm", atoi(image->filename) + dir);
    fprintf(stderr, "%s\n", str);
    delete image;
    image = new Image(str);
    livewire->raster = image->getRaster();
    livewire->resetGauss();
    if(display)
        display->setImage(livewire->getCostImage());
    glutPostRedisplay();
}

void MainWindow::CallBackKeyboardFunc(unsigned char key, int x, int y) {
    LiveWire *newlivewire;

    switch(key) {
    case 'n':
    case 'N':
        next(1);
        break;
    case 'p':
    case 'P':
        next(-1);
        break;
    case 's':
    case 'S':
        if(highcurvature)
            delete highcurvature;
        highcurvature = livewire->highCurvature(16, -1); //(used to be 10,20)
        highcurvature->setVisible(1);
        newlivewire = new LiveWire(image);
        newlivewire->average = livewire->average;
        newlivewire->averageb = livewire->averageb;
        newlivewire->sdev = livewire->sdev;
        newlivewire->sdevb = livewire->sdevb;
        livewire->setVisible(0);
        snake = new ISnake(highcurvature, image, newlivewire, 15);
        snake->setVisible(1);
        glutPostRedisplay();
        break;
    case 'r':
    case 'R':
        reset();
        break;
    case 'w':
    case 'W':
        writePPM();
        break;
    case 't':
    case 'T':
        if(livewire->start) {
            livewire->train();
            if(display)
                display->setImage(livewire->getCostImage());
        }
        else
            painting = 1;
        glutPostRedisplay();
        break;
    case 'c':
    case 'C':
        if(display) {
            delete display;
            display = NULL;
        }
        else if(livewire)
            display = new DisplayWindow(glutmaster, livewire->getCostImage());
        break;
    case 'l':
    case 'L':
        newLiveWire();
        glutPostRedisplay();
        break;
    case 'm':
    case 'M':
        if(snake)
            snake->minimize();
        glutPostRedisplay();
        break;
    case '3':
        image->medianFilter(3);
        livewire->raster = image->getRaster();
        livewire->resetGauss();
        image = new Image(image->filename);
        if(display)
            display->setImage(livewire->getCostImage());
        glutPostRedisplay();
        break;
    case '5':
        image->medianFilter(5);
        livewire->raster = image->getRaster();
        livewire->resetGauss();
        image = new Image(image->filename);
        if(display)
            display->setImage(livewire->getCostImage());
        glutPostRedisplay();
        break;
    case 'e':
    case 'E':
        livewire->writeCNT("livewire.cnt");
        break;
    case 127:
        livewire->deleteControl();
        glutPostRedisplay();
        break;
    case 27:
        delete this;
        exit(0);
        break;
    }
}

void MainWindow::CallBackDisplayFunc(void) {
    int i;

    glClear(GL_COLOR_BUFFER_BIT);
    glDrawPixels(image->width, image->height, GL_LUMINANCE, GL_UNSIGNED_BYTE,
                 image->getRaster());

    if(paint)
        paint->draw();
    if(livewire)
        livewire->draw();
    if(highcurvature)
        highcurvature->draw();
    if(snake)
        snake->draw();
    glutSwapBuffers();
}

void MainWindow::CallBackReshapeFunc(int w, int h) {
    glutReshapeWindow(width * 2, height * 2);
    glutPostRedisplay();
}

void MainWindow::CallBackMouseFunc(int button, int state, int x, int y) {
    x /= 2;
    y /= 2;

    if(button == GLUT_LEFT_BUTTON)
        if(state == GLUT_DOWN) {
            if(painting) {
                paint = new Contour();
                paint->setVisible(1);
            }
            else if(livewire)
                livewire->addPointAt(x, y);
        }
        else if(state == GLUT_UP && painting) {
            livewire->train(paint);
            if(display)
                display->setImage(livewire->getCostImage());
            painting = 0;
            delete paint;
            paint = NULL;
        }
    if(button == GLUT_MIDDLE_BUTTON)
        if(state == GLUT_DOWN && !painting)
            livewire->closeContour();

    glutPostRedisplay();
}

void MainWindow::CallBackMotionFunc(int x, int y) {
    x /= 2;
    y /= 2;

    if(x > 0 && x < width - 1 && y > 0 && y < height - 1)
        if(paint)
            paint->addPointAt(x, y);
        else
            livewire->moveCursor(x, y);

    glutPostRedisplay();
}

void MainWindow::CallBackPassiveMotionFunc(int x, int y) {
    CallBackMotionFunc(x, y);
}















