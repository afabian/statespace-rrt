#ifndef STATEFLOATERMATH_H
#define STATEFLOATERMATH_H

#include "StateFloater.h"

class StateFloaterMath {
  
public:
    double distance(StateFloater* a, StateFloater* b);
    bool connectable(StateFloater* a, StateFloater* b);
    void setRandomStateConstraints(StateFloater _minimums, StateFloater _maximums);
    StateFloater getRandomState();

protected:
    StateFloater minimums, maximums;
    StateFloater scale, shift;

};

#endif