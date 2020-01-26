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
