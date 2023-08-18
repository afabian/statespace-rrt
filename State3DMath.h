#ifndef STATE3DMATH_H
#define STATE3DMATH_H

#include "State3D.h"

class State3DMath {
  
public:
    double distance(State3D* a, State3D* b);
    bool connectable(State3D* a, State3D* b);
    void setRandomStateConstraints(State3D _minimums, State3D _maximums);
    State3D getRandomState();

protected:
    State3D minimums, maximums;
    State3D scale, shift;
    
};

#endif