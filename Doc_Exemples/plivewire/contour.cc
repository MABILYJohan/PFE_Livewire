/*
  Intelligent Snakes

  FILE: countour.cc
  DESCRIPTION: defines Contour class,
               stores a list of points
  AUTHOR: Steven Marx, Univeristy of Rochester
  LAST UPDATE: 8/01/2000
*/

#include "contour.h"

Contour::Contour() {
  start = end = NULL;
  numpoints = 0;
  visible = 0;
  closed = 0;
}

Contour::Contour(Point *start) {
  Point *current = start;

  this->start = end = NULL;
  numpoints = 0;
  closed = 0;

  while(current && (!numpoints || current != start)) {
    addPointAt(current->x, current->y);
    current = current->next;
  }
}

Contour::Contour(char *filename) {
  FILE *input = fopen(filename, "r");
  int x, y;

  numpoints = 0;

  while(!feof(input)) {
    fscanf(input, "%d, %d\n", &x, &y);
    addPointAt(x, y);
  }
}

Contour::~Contour() {
  Point *current = start, *temp;

  while(current && current != end) {
    temp = current;
    current = current->next;
    delete temp;
  }
  delete end;
}

void Contour::reset() {
  Point *current = start, *temp;
  int i;

  for(i = 0; i < numpoints; i++) {
    temp = current;
    current = current->next;
    delete temp;
  }

  numpoints = 0;
  start = end = NULL;
  closed = 0;
}

Point *Contour::closestPoint(int x, int y) {
  Point *current = start, *best = NULL;
  long min = INFINITY, distance;

  while(current) {
    if((distance = (current->x - x) * (current->x - x) +
	           (current->y - y) * (current->y - y)) < min) {
      best = current;
      min = distance;
    }
    current = current->next;
  }
  return best;
}

void Contour::writeCNT(char *filename) {
  FILE *output = fopen(filename, "a");
  Point *current = start;
  int count;

  for(count = 0; count < numpoints; count++) {
    fprintf(output, "%d, %d\n", current->x, current->y);
    current = current->next;
  }
}

void Contour::draw() {
  Point *current = start;
  int i;

  if(visible) {
    glColor3ub(255, 0, 0);
    glBegin(GL_LINE_STRIP);
      for(i = 0; i < numpoints; i++) {
        glVertex2i(current->x, current->y);
        current = current->next;
      }
      if(closed)
	glVertex2i(start->x, start->y);
    glEnd();

    current = start;
    glPointSize(4);
    glBegin(GL_POINTS);
      for(i = 0; i < numpoints; i++) {
        glVertex2i(current->x, current->y);
        current = current->next;
      }
    glEnd();
  }
}

void Contour::addPointAt(int x, int y) {
  addPoint(new Point(x, y));
}

void Contour::addPoint(Point *newpoint) {
  if(start) {
    end->next = newpoint;
    newpoint->previous = end;
    end = newpoint;
  }
  else
    start = end = newpoint;
  numpoints++;
}

void Contour::deletePointAt(int x, int y) {
  deletePoint(closestPoint(x, y));
}

void Contour::deletePoint(Point *toremove) {
  if(toremove->next)
    toremove->next->previous = toremove->previous;
  if(toremove->previous)
    toremove->previous->next = toremove->next;

  if(toremove == end)
    end = toremove->previous;

  if(toremove == start)
    start = toremove->next;

  delete toremove;

  numpoints--;
}
  
Contour *Contour::highCurvature(int resolution, int threshold) {
  Point *current = start, *memory[resolution + 1], *maxpoint;
  int max, i;
  int count = 0;
  Contour *newCont = new Contour();
  int last = 0;
  int first = 0;

  for(i = 0; i < numpoints + resolution - 1; i++) {
    if(count == resolution + 1) {
      memcpy(&memory[0], &memory[1], (resolution-1) * sizeof(memory[0]));
      memory[resolution] = current;
      memory[resolution/2]->cost = (int)(pow(memory[resolution]->x - 2 * memory[resolution/2]->x
			  + memory[0]->x, 2) +
	              pow(memory[resolution]->y - 2 * memory[resolution/2]->y
			  + memory[0]->y, 2));
      memory[resolution/2]->accessor = NONE;
    }
    else
      memory[count++] = current;
    current = current->next;
  }

  current = start;

  for(i = 0; i < numpoints; i++) {
    if(i && !(i % resolution) || (i == numpoints - 1)) {
      maxpoint->accessor = DONE;
      max = 0;
      maxpoint = NULL;
    }
    if(current->cost > max) {
      max = (int)current->cost;
      maxpoint = current;
    }
    current = current->next;
  }

  current = start;

  for(i = 0; i < 5; i++)
    current = current->next;

  maxpoint = NULL;
  max = 0;
  last = 0;

  for(i = 0; i < numpoints; i++) {
    if(!(i % resolution) && maxpoint) {
      fprintf(stderr, "%d\n", first - last);
      newCont->addPointAt(maxpoint->x, maxpoint->y);
      max = 0;
      maxpoint = NULL;
      last = first;
    }
    if(current->cost > max && current->accessor == DONE) {
      max = (int)current->cost;
      maxpoint = current;
      first = i;
    }
    current = current->next;
  }

  newCont->closeContour();
  return newCont;
}

/*Contour *Contour::highCurvature(int resolution, int threshold) {
  Point *current = start, *memory[2 * resolution + 1], *buffer[resolution + 2];
  double curvatures[resolution + 2][2], curvature;
  int i, j, numcurves = 0, count = 0, distance = 0, numhigh = 0, maxpoint;
  double max;
  Contour *newCont = new Contour();

  for(i = 0; i < 2 * resolution + 1; i++)
    memory[i] = NULL;

  for(i = 0; i < numpoints + resolution * 2 - 1; i++) {
    if(count == 2 * resolution + 1) {
      memcpy(&memory[0], &memory[1], 2 * resolution * sizeof(memory[0]));
      memory[count - 1] = current;
      if(numcurves || distance)
	distance++;
      curvature = pow(memory[2 * resolution]->x - 2 * memory[resolution]->x
		      + memory[0]->x, 2) +
                  pow(memory[2 * resolution]->y - 2 * memory[resolution]->y
		      + memory[0]->y, 2);
      if(curvature > threshold || i == numpoints + resolution * 2 - 2) {
	if(distance > resolution || i == numpoints + resolution * 2 - 2) {
	  max = -1;
	  for(j = 0; j < numcurves; j++)
	    if(curvatures[j][0] > max) {
	      max = curvatures[j][0];
	      maxpoint = j;
	    }
	  if(maxpoint == 0) {
	    newCont->addPointAt(buffer[maxpoint]->x, buffer[maxpoint]->y);
	    numcurves = 0;
	    distance = 1;
	  }
	  else {
	    buffer[0] = buffer[maxpoint];
	    curvatures[0][0] = curvatures[j][0];
	    curvatures[0][1] = curvatures[j][1];
	    numcurves = 1;
	  //	  distance = resolution - (int)curvatures[maxpoint][1];
	    distance = 1;
	  }
	}
        else if(curvature > threshold) {
	  buffer[numcurves] = memory[resolution];
	  curvatures[numcurves][0] = curvature;
	  curvatures[numcurves++][1] = distance;
	}
      }
    }
    else
      memory[count++] = current;
    current = current->next;
  }
  newCont->closeContour();
  return newCont;
}*/

/*Contour *Contour::highCurvature(int resolution, int threshold) {
  Point *current = start, *memory[2 * resolution + 1], *snake[2000];
  double curvatures[resolution + 2][2], curvature;
  int i, j, numcurves = 0, count = 0, distance = 0, last = NONE, numhigh = 0;
  double max;
  int maxpoint;
  Contour *newCont = new Contour();

  for(i = 0; i < 2 * resolution + 1; i++)
    memory[i] = NULL;

  for(i = 0; i < numpoints + resolution * 2 - 1; i++) {
    if(count == 2 * resolution + 1) {
      memcpy(&memory[0], &memory[1], 2 * resolution * sizeof(memory[0]));
      memory[count - 1] = current;
      if(last != NONE)
	distance++;
      curvature = pow(memory[2 * resolution]->x - 2 * memory[resolution]->x
		      + memory[0]->x, 2) +
	          pow(memory[2 * resolution]->y - 2 * memory[resolution]->y
		      + memory[0]->y, 2);
      if(curvature > threshold || i == numpoints + resolution * 2 - 2) {
	if(distance > resolution || i == numpoints + resolution * 2 - 2) {
	  max = 0;
	  for(j = last; j < numhigh; j++)
	    if(curvatures[i - last][0] > max) {
	      max = curvatures[i - last][0];
	      maxpoint = i;
	    }
	  snake[last] = snake[maxpoint];
	  curvatures[0][0] = curvatures[maxpoint - last][0];
	  curvatures[0][1] = curvatures[maxpoint - last][1];
	  numhigh = last++;
	  numcurves = 1;
	  distance = 0;
	}
	if(last == NONE)
	  last = numhigh;
	if(curvature > threshold) {
	  snake[numhigh++] = memory[resolution];
	  curvatures[numcurves][0] = curvature;
	  curvatures[numcurves++][1] = distance;
	}
      }
    }
    else
      memory[count++] = current;
    current = current->next;
  }

  for(i = 0; i < numhigh; i++)
    newCont->addPointAt(snake[i]->x, snake[i]->y);
  if(numhigh)
    newCont->addPointAt(snake[0]->x, snake[0]->y);

  return newCont;
  }*/

void Contour::setVisible(int visible) {
  this->visible = visible;
}

void Contour::closeContour() {
  if(!closed) {
    start->previous = end;
    end->next = start;
    closed = 1;
  }
}







