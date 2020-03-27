/*
  Intelligent Snakes

  FILE: liveWire.cc
  DESCRIPTION: defines LiveWire class,
               handles graph searching
  AUTHOR: Steven Marx, University of Rochester
  LAST UPDATE: 8/01/2000
*/

#include "liveWire.h"
#include <stdio.h>

LiveWire::LiveWire(Image *image) : Q(image->getWidth() * image->getHeight()) {
    int i;

    this->raster = image->getRaster();
    width = image->getWidth();
    height = image->getHeight();
    cursor = NULL;
    numpoints = 0;
    points = new Point *[width * height];
    for(i = 0; i < width * height; i++)
        points[i] = new Point(i % width, i / width);
    closed = 0;
    average = sdev = 0;
    averageb = sdevb = 0;
    costcap = INFINITY + 1;
    resetGauss();
}

LiveWire::~LiveWire() {
    int i;

    for(i = 0; i < width * height; i++)
        delete points[i];

    delete[] points;

    start = end = NULL;
}

void LiveWire::resetGauss() {
    int i;

    for(i = 0; i < 256; i++)
        gausstable[i] = gausstableb[i] = -1;
}

void LiveWire::addPointAt(int x, int y) {
    Point *current = points[x + y * width];

    if(x > 0 && x < width - 1 && y > 0 && y < height - 1 && !closed &&
            (current == start || !current->next) && (!start || current->previous)) {
        while(current && current->previous && current != end) {
            current->previous->next = current;
            current = current->previous;
            numpoints++;
        }

        if(!start)
            start = points[x + y * width];

        end = points[x + y * width];

        controlpoints.addPointAt(x, y);
        shortestPaths(end);
    }
}

void LiveWire::deleteControl() {
    Point *current, *temp;

    if(controlpoints.end) {
        if(controlpoints.end->previous) {
            current = points[controlpoints.end->previous->x + controlpoints.end->previous->y * width];
            while(current) {
                temp = current->next;
                current->next = NULL;
                current = temp;
                numpoints--;
            }
        }

        controlpoints.deletePoint(controlpoints.end);

        if(controlpoints.end) {
            end = points[controlpoints.end->x + controlpoints.end->y * width];
            numpoints++;
            shortestPaths(end);
        }
        else {
            start = NULL;
            initPoints();
        }
    }
}

void LiveWire::train(Contour *boundary) {
    float temp, tempb;
    int count = 0;
    float sqsum = 0, sqsumb = 0;
    Point *current = boundary->start;

    average = 0;
    averageb = 0;

    while(current && (count == 0 || current != boundary->start)) {
        if(current->next) {
            count++;
            temp = partialCost(current, current->next);
            tempb = partialCostb(current, current->next);
            average += temp;
            averageb += tempb;
            sqsum += temp * temp;
            sqsumb += tempb * tempb;
        }
        current = current->next;
    }
    average /= (float)count;
    averageb /= (float)count;
    sdev = sqrt((sqsum - 2 * average * average * (float)count + average * average * (float)count)/(float)(count-1));
    sdevb = sqrt((sqsumb - 2 * averageb * averageb * (float)count + averageb * averageb * (float)count)/(float)(count-1));
    resetGauss();
    if(end)
        shortestPaths(end);
}

void LiveWire::train() {
    float temp, tempb;
    int count = 0;
    float sqsum = 0, sqsumb = 0;
    Point *current = start;

    average = 0;
    averageb = 0;

    while(current && (count == 0 || current != start)) {
        if(current->next) {
            count++;
            temp = partialCost(current, current->next);
            tempb = partialCostb(current, current->next);
            average += temp;
            averageb += tempb;
            sqsum += temp * temp;
            sqsumb += tempb * tempb;
        }
        current = current->next;
    }
    average /= (float)count;
    averageb /= (float)count;
    sdev = sqrt((sqsum - 2 * average * average * (float)count + average * average * (float)count)/(float)(count-1));
    sdevb = sqrt((sqsumb - 2 * averageb * averageb * (float)count + averageb * averageb * (float)count)/(float)(count-1));
    resetGauss();
    if(end)
        shortestPaths(end);
}

Image *LiveWire::getCostImage() {
    int x, y, i, best, current;
    static int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    unsigned char *costi = new unsigned char[width * height];

    for(y = 1; y < height - 1; y++)
        for(x = 1; x < width - 1; x++) {
            best = 0;
            for(i = 0; i < 4; i++) {
                current = (unsigned char)(255 - cost(points[x + y * width], points[(x + neighbors[i][0]) + ((y + neighbors[i][1]) * width)]) * 255);
                if(current > best)
                    best = current;
            }
            costi[x + y * width] = best;
        }
    return new Image(width, height, costi);
}

float LiveWire::getCost(int x, int y) {
    return points[x + y * width]->cost;
}

/*float LiveWire::f6(unsigned char t, unsigned char p, unsigned char v,
                   unsigned char u, unsigned char q, unsigned char w) {
  return 1.0/3.0*abs(p + t + v - q - u - w);
}*/



/*float LiveWire::f6(unsigned char t, unsigned char p, unsigned char v,
                   unsigned char u, unsigned char q, unsigned char w) {
  return 0.25*(abs(p - u) + abs(t - q) + abs(p - w) + abs(v - q));
  }*/

float LiveWire::f6(unsigned char t, unsigned char p, unsigned char v,
                   unsigned char u, unsigned char q, unsigned char w) {
    return q;
}

float LiveWire::fb(unsigned char t, unsigned char p, unsigned char v,
                   unsigned char u, unsigned char q, unsigned char w) {
    return p;
}

/*float LiveWire::f6(unsigned char t, unsigned char p, unsigned char v,
                   unsigned char u, unsigned char q, unsigned char w) {
  return (255 + (q - p))/2.0;
  }*/

float LiveWire::gauss(float x, float mean, float sd) {
    if(gausstable[(int)x] == -1)
        gausstable[(int)x] = exp((-0.5*((x-mean)/sd)*((x-mean)/sd)));
    return gausstable[(int)x];
}

float LiveWire::gaussb(float x, float mean, float sd) {
    if(gausstableb[(int)x] == -1)
        gausstableb[(int)x] = exp((-0.5*((x-mean)/sd)*((x-mean)/sd)));
    return gausstableb[(int)x];
}

float LiveWire::partialCostb(Point *from, Point *to) {
    int x1 = from->x;
    int y1 = from->y;
    int x2 = to->x;
    int y2 = to->y;
    unsigned char t, p, v, u, q, w;

    if(y1 > y2) {
        t = raster[x1 - 1 + (y1 - 2) * width];
        p = raster[x1 - 1 + (y1 - 1) * width];
        v = raster[x1 - 1 + y1 * width];
        u = raster[x1 + (y1 - 2) * width];
        q = raster[x1 + (y1 - 1) * width];
        w = raster[x1 + y1 * width];
    }
    else if(y1 < y2) {
        w = raster[x1 - 1 + (y2 - 2) * width];
        q = raster[x1 - 1 + (y2 - 1) * width];
        u = raster[x1 - 1 + y2 * width];
        v = raster[x1 + (y2 - 2) * width];
        p = raster[x1 + (y2 - 1) * width];
        t = raster[x1 + y2 * width];
    }
    else if(x1 < x2) {
        v = raster[x1 - 1 + (y1 - 1) * width];
        p = raster[x1 + (y1 - 1) * width];
        t = raster[x1 + 1 + (y1 - 1) * width];
        w = raster[x1 - 1 + y1 * width];
        q = raster[x1 + y1 * width];
        u = raster[x1 + 1 + y1 * width];
    }
    else {
        u = raster[x1 - 1 + (y1 - 1) * width];
        q = raster[x1 + (y1 - 1) * width];
        w = raster[x1 + 1 + (y1 - 1) * width];
        t = raster[x1 - 1 + y1 * width];
        p = raster[x1 + y1 * width];
        v = raster[x1 + 1 + y1 * width];
    }
    return 255-fb(t, p, v, u, q, w);
}

float LiveWire::partialCost(Point *from, Point *to) {
    int x1 = from->x;
    int y1 = from->y;
    int x2 = to->x;
    int y2 = to->y;
    unsigned char t, p, v, u, q, w;

    if(y1 > y2) {
        t = raster[x1 - 1 + (y1 - 2) * width];
        p = raster[x1 - 1 + (y1 - 1) * width];
        v = raster[x1 - 1 + y1 * width];
        u = raster[x1 + (y1 - 2) * width];
        q = raster[x1 + (y1 - 1) * width];
        w = raster[x1 + y1 * width];
    }
    else if(y1 < y2) {
        w = raster[x1 - 1 + (y2 - 2) * width];
        q = raster[x1 - 1 + (y2 - 1) * width];
        u = raster[x1 - 1 + y2 * width];
        v = raster[x1 + (y2 - 2) * width];
        p = raster[x1 + (y2 - 1) * width];
        t = raster[x1 + y2 * width];
    }
    else if(x1 < x2) {
        v = raster[x1 - 1 + (y1 - 1) * width];
        p = raster[x1 + (y1 - 1) * width];
        t = raster[x1 + 1 + (y1 - 1) * width];
        w = raster[x1 - 1 + y1 * width];
        q = raster[x1 + y1 * width];
        u = raster[x1 + 1 + y1 * width];
    }
    else {
        u = raster[x1 - 1 + (y1 - 1) * width];
        q = raster[x1 + (y1 - 1) * width];
        w = raster[x1 + 1 + (y1 - 1) * width];
        t = raster[x1 - 1 + y1 * width];
        p = raster[x1 + y1 * width];
        v = raster[x1 + 1 + y1 * width];
    }
    return 255-f6(t, p, v, u, q, w);
}

float LiveWire::cost(Point *from, Point *to) {
    //  return 1 - gauss(partialCost(from, to), average, sdev);
    return 1-0.5*(gauss(partialCost(from, to), average, sdev) +
                  gaussb(partialCostb(from, to), averageb, sdevb));
}

/*float LiveWire::cost(Point *from, Point *to) {
  return 1 - gauss(partialCost(from, to), average, sdev);
  }*/

void LiveWire::initPoints() {
    long i;

    for(i = 0; i < width * height; i++) {
        points[i]->accessor = NONE;
        points[i]->cost = INFINITY;
        points[i]->previous = NULL;
    }
}

void LiveWire::shortestPathsAt(int x, int y) {
    shortestPaths(points[x + y * width]);
}

void LiveWire::shortestPaths(Point *start) {
    Point *current, *v;
    static int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    int newx, newy, i;
    float tempcost;

    initPoints();
    start->cost = 0;
    Q.addPoint(start);

    while(!Q.empty()) {
        current = Q.deleteMin();
        current->accessor = DONE;
        if(!current->next && current->cost <= costcap)
            for(i = 0; i < 4; i++) {
                newx = current->x + neighbors[i][0];
                newy = current->y + neighbors[i][1];
                if(newx > 0 && newx < width - 1 && newy > 0 && newy < height - 1) {
                    v = points[newx + newy * width];
                    if(v->accessor != DONE) {
                        tempcost = current->cost + cost(current, v);
                        if(tempcost < v->cost) {
                            v->previous = current;
                            Q.setCost(v, tempcost);
                        }
                    }
                }
            }
    }
}

void LiveWire::draw() {
    Point *current;
    int i;

    if(visible) {
        glPointSize(1);
        glColor3ub(0, 255, 0);

        current = start;
        glBegin(GL_POINTS);
        for(i = 0; i < numpoints; i++) {
            glVertex2i(current->x, current->y);
            current = current->next;
        }
        glEnd();

        current = cursor;
        glBegin(GL_POINTS);
        while(current && current->next != start) {
            glVertex2i(current->x, current->y);
            current = current->previous;
        }
        glEnd();
    }

    controlpoints.draw();
}

void LiveWire::moveCursor(int x, int y) {
    if(x > 1 && x < width - 1 && y > 1 && y < height - 1 && !closed && !points[x + y * width]->next && points[x + y * width]->previous)
        cursor = points[x + y * width];
}

void LiveWire::closeContour() {
    addPointAt(start->x, start->y);
    if(start == end) {
        closed = 1;
        cursor = NULL;
    }
}  
