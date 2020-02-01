#include "contour.h"


Contour::Contour()
{
    ;
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return contour; }
