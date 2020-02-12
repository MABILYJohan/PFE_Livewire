/*
  Intelligent Snakes

  FILE: isnake.cc
  DESCRIPTION: defines ISnake class,
               stores a snake and calculates its behavior at each iteration
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/07/2000
*/

#include "isnake.h"

ISnake::ISnake() {
  this->image = NULL;
  this->livewire = NULL;
  minimized = 0;
}

ISnake::ISnake(Image *image) {
  this->image = image;
  this->livewire = NULL;
  minimized = 0;
}

ISnake::ISnake(Contour *contour) : Snake(contour) {
  closeContour();
  this->image = NULL;
  this->livewire = NULL;
  minimized = 0;
}

ISnake::ISnake(Contour *contour, Image *image) : Snake(contour) {
  closeContour();
  this->image = image;
  this->livewire = NULL;
  minimized  = 0;
}

ISnake::ISnake(Contour *contour, Image *image, LiveWire *livewire, int resolution) : Snake(contour) {
  closeContour();
  this->image = image;
  this->livewire = livewire;
  this->resolution = resolution;
  minimized = 0;
}

void ISnake::minimize() {
  Point *matrix[numpoints][resolution * resolution];
  Point *positions[numpoints][resolution];
  Point *a, *b, *c, *min, *temp;
  int backzero, backone;
  Point *current = start;
  double lcost, firstd, newcost;
  int i, j, k, l;
  double x1, y1, x2, y2;
  double centroidx, centroidy;
  double ai, atmp = 0, xtmp = 0, ytmp = 0, x, y, step;
  int howmany;

  fprintf(stderr, "Minimizing...");

  livewire->costcap = 15;

  for(i = 0; i < numpoints; i++) {
    x1 = current->x - current->previous->x;
    y1 = current->y - current->previous->y;
    x2 = current->next->x - current->x;
    y2 = current->next->y - current->y;
    x = (y1/(sqrt(x1*x1+y1*y1)) + y2/(sqrt(x2*x2+y2*y2)))/2;
    y = -(x1/(sqrt(x1*x1+y1*y1)) + x2/(sqrt(x2*x2+y2*y2)))/2;
    step = 1/sqrt(x*x + y*y);
    for(j = 0; j < resolution; j++)
      positions[i][j] = new Point(current->x+(int)(x *
						   (j - resolution/2) * step),
				  current->y+(int)(y *
						   (j - resolution/2) * step));
    current = current->next;
  }

  /*  for(i = 0; i < numpoints; i++) {
    ai = current->x * current->previous->y - current->y * current->previous->x;
    atmp += ai;
    xtmp += (current->x + current->previous->x) * ai;
    ytmp += (current->y + current->previous->y) * ai;
    current = current->next;
  }

  x = xtmp / (3 * atmp);
  y = ytmp / (3 * atmp);

  current = start;
  for(i = 0; i < numpoints; i++) {
    step = 1/sqrt((x - current->x)*(x - current->x)
		  + (y - current->y)*(y - current->y));
    for(j = 0; j < resolution; j++)
      positions[i][j] = new Point(current->x+(int)((x-current->x) *
						   (j - resolution/2) * step),
				  current->y+(int)((y-current->y) *
						   (j - resolution/2) * step));
      current = current->next;
    }*/

  fprintf(stderr, "snaxel 1 of %d\n", numpoints);

  fprintf(stderr, "snaxel 2 of %d\n", numpoints);
  for(i = 0; i < resolution * resolution; i++) {
    b = new Point(positions[0][i/resolution]->x,
		  positions[0][i/resolution]->y);
    c = new Point(positions[1][i%resolution]->x,
		  positions[1][i%resolution]->y);

    livewire->shortestPathsAt(b->x, b->y);
    lcost = livewire->getCost(c->x, c->y);

    while(lcost == INFINITY && livewire->costcap < INFINITY) {
      livewire->costcap *= 2;
      livewire->shortestPathsAt(b->x, b->y);
      lcost = livewire->getCost(c->x, c->y);
    }

    firstd = firstderiv(b, c);
    
    matrix[1][i] = new Point();
    matrix[1][i]->cost = lcost + firstd;
  }


  for(i = 2; i < numpoints; i++) {
    fprintf(stderr, "snaxel %d of %d\n", i + 1, numpoints);
    livewire->costcap = 15;
    for(j = 0; j < resolution; j++) {
      b = new Point(positions[i - 1][j]->x, positions[i - 1][j]->y);
      livewire->shortestPathsAt(b->x, b->y);
      for(k = 0; k < resolution; k++) {
	c = new Point(positions[i][k]->x, positions[i][k]->y);

	lcost = livewire->getCost(c->x, c->y);

	while(lcost == INFINITY && livewire->costcap < INFINITY) {
	  fprintf(stderr, "bumping\n");
	  livewire->costcap *= 2;
	  livewire->shortestPathsAt(b->x, b->y);
	  lcost = livewire->getCost(c->x, c->y);
	}

	firstd = firstderiv(b, c);
	a = new Point();
	min = new Point();
	min->cost = INFINITY * 2;

	for(l = 0; l < resolution; l++) {
	  a->x = positions[i - 2][l]->x;
	  a->y = positions[i - 2][l]->y;
	  if((newcost = seconderiv(a, b, c) +
	      matrix[i - 1][l * resolution + j]->cost) <= min->cost) {
	    min->x = a->x;
	    min->y = a->y;
	    min->cost = newcost;
	    min->accessor = l;
	  }
	}

	min->cost += firstd + lcost;
	matrix[i][j * resolution + k] = min;
	delete a;
	delete c;
      }
      delete b;
    }
  }

  howmany = numpoints;
  reset();

  min = matrix[howmany - 1][0];
  backone = 0;

  for(i = 1; i < resolution * resolution; i++)
    if(matrix[howmany - 1][i]->cost < min->cost) {
      min = matrix[howmany - 1][i];
      backzero = i % resolution;
      backone = i / resolution;
    }

  addPointAt(positions[howmany - 1][backzero]->x, positions[howmany - 1][backzero]->y);
  addPointAt(positions[howmany - 2][backone]->x, positions[howmany - 2][backone]->y);
  addPointAt(min->x, min->y);

  fprintf(stderr, "%d\n", (int)min->cost);

  for(i = howmany - 2; i >= 2; i--) {
    backzero = backone;
    backone = min->accessor;
    min = matrix[i][backone * resolution + backzero];
    addPointAt(min->x, min->y);
  }

  minimized = 1;

  for(i = 0; i < howmany; i++)
    for(j = 0; j < resolution; j++)
      delete positions[i][j];
  for(i = 1; i < howmany; i++)
    for(j = 0; j < resolution * resolution; j++)
      delete matrix[i][j];
}

double ISnake::firstderiv(Point *a, Point *b) {
  return 0;
}

double ISnake::seconderiv(Point *a, Point *b, Point *c) {
  return 0;
}

/*void ISnake::draw() {
  int i, j;
  Point *current = start;
  double ai, atmp = 0, xtmp = 0, ytmp = 0, x, y, step;

  if(visible) {
    if(!minimized) {
      for(i = 0; i < numpoints; i++) {
	ai = current->x * current->previous->y - current->y * current->previous->x;
	atmp += ai;
	xtmp += (current->x + current->previous->x) * ai;
	ytmp += (current->y + current->previous->y) * ai;
	current = current->next;
      }
      x = xtmp / (3 * atmp);
      y = ytmp / (3 * atmp);
      
      glPointSize(5);
      glColor3ub(0, 255, 255);
      glBegin(GL_POINTS);
      glVertex2i((int)x, (int)y);
      glEnd();
      glPointSize(2);
    
      current = start;
      for(i = 0; i < numpoints; i++) {
	step = 1/sqrt((x - current->x)*(x - current->x) + (y - current->y)*(y - current->y));
	glBegin(GL_LINES);
  	glVertex2i(current->x+(int)((x-current->x) * (-resolution/2) * step),
		   current->y+(int)((y-current->y) * (-resolution/2) * step));
	glVertex2i(current->x+(int)((x-current->x) * (resolution/2) * step),
		   current->y+(int)((y-current->y) * (resolution/2) * step));
	glEnd();
	glBegin(GL_POINTS);
  	  glVertex2i(current->x, current->y);
	glEnd();
	current = current->next;
      }
    }
    else {
      glPointSize(2);
      glColor3ub(0, 255, 255);
      current = start;
      glBegin(GL_POINTS);
      for(i = 0; i < numpoints; i++) {
	glVertex2i(current->x, current->y);
	current = current->next;
      }
      glEnd();
    }
  }
}*/

/*void ISnake::draw() {
  int i, j;
  Point *current = start;
  int resolution = 20;
  long xtotal = 0, ytotal = 0;
  double x, y, step;

  glPointSize(2);
  glColor3ub(0, 0, 255);

  for(i = 0; i < numpoints; i++) {
    xtotal += current->x;
    ytotal += current->y;
    current = current->next;
  }
  x = (double)xtotal/numpoints;
  y = (double)ytotal/numpoints;
  glPointSize(5);
  glColor3ub(0, 255, 255);
  glBegin(GL_POINTS);
  glVertex2i((int)x, (int)y);
  glEnd();
  glColor3ub(0, 0, 255);
  glPointSize(2);

  current = start;
  for(i = 0; i < numpoints; i++) {
    step = 1/sqrt((x - current->x)*(x - current->x) + (y - current->y)*(y - current->y));
    glBegin(GL_LINE_STRIP);
    for(j = -resolution/2; j < resolution/2; j++)
      glVertex2i(current->x + (int)((x-current->x) * j * step), current->y + (int)((y-current->y) * j * step));
    glEnd();
    current = current->next;
  }
  }*/

void ISnake::draw() {
  int i, j;
  Point *current = start;
  double x, y, x1, y1, x2, y2, step;

  if(visible)
    if(!minimized) {
      glPointSize(2);
      glColor3ub(0, 255, 255);

      for(i = 0; i < numpoints; i++) {
	x1 = current->x - current->previous->x;
	y1 = current->y - current->previous->y;
	x2 = current->next->x - current->x;
	y2 = current->next->y - current->y;
	x = (y1/(sqrt(x1*x1+y1*y1)) + y2/(sqrt(x2*x2+y2*y2)))/2;
	y = -(x1/(sqrt(x1*x1+y1*y1)) + x2/(sqrt(x2*x2+y2*y2)))/2;
	step = 1/sqrt(x*x + y*y);
	glBegin(GL_LINES);
	glVertex2i(current->x + (int)(x * -resolution/2 * step), current->y + (int)(y * -resolution/2 * step));
	glVertex2i(current->x + (int)(x * resolution/2 * step), current->y + (int)(y * resolution/2 * step));
	glEnd();
	current = current->next;
      }
    }
    else {
      glPointSize(2);
      glColor3ub(0, 255, 255);
      current = start;
      glBegin(GL_POINTS);
      for(i = 0; i < numpoints; i++) {
	glVertex2i(current->x, current->y);
	current = current->next;
      }
      glEnd();
    }
}

/*void ISnake::draw() {
  int i, j, x, y;
  int width = image->getWidth();
  Point *current = start;
  double step;
  int resolution = 50;
  unsigned char *raster = image->getRaster();

  glPointSize(2);
  glColor3ub(0, 0, 255);
  for(i = 0; i < numpoints; i++) {
    x = raster[current->x + 1 + current->y * width] -
      raster[current->x + current->y * width];
    y = raster[current->x + (current->y + 1) * width] -
      raster[current->x + current->y * width];
    step = 1/sqrt(x*x + y*y);
    glBegin(GL_LINE_STRIP);
    for(j = -resolution/2; j < resolution/2; j++)
      glVertex2i(current->x + (int)(x * j * step), current->y + (int)(y * j * step));
    glEnd();
    current = current->next;
  }
  }*/





