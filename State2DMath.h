#ifndef STATE2DMATH_H
#define STATE2DMATH_H

#include "State2D.h"

class State2DMath {
  
public:
    double distance(State2D* a, State2D* b);
    bool connectable(State2D* a, State2D* b);
    void setRandomStateConstraints(State2D _minimums, State2D _maximums);
    State2D getRandomState();

protected:
    State2D minimums, maximums;
    State2D scale, shift;

};

#endif