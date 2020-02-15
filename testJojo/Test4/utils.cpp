#include "utils.h"

#include <QObject>
#include <QDebug>
#include <ctime>

Utils::Utils()
{

}

int Utils::randInt(int low, int high)
{
    return qrand() % ((high + 1) - low) + low;
}

double Utils::distance_euclidienne(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

double Utils::distance_euclidienne(double x1, double y1, double x2, double y2, double x3, double y3)
{
    return sqrt((x1-x2-x3)*(x1-x2-x3) + (y1-y2-y3)*(y1-y2-y3));
}
