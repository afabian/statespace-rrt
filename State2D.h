#ifndef STATE2D_H
#define STATE2D_H

#include <string>

class State2D {

friend class State2DMath;
friend class Map2D;

public:
    State2D();
    State2D(double _x, double _y);    
    State2D get();
    void set(State2D source);
    void set(State2D* source);
    void set(double _x, double _y);
    std::string toString();
    bool operator==(const State2D &other);

protected:
    double x;
    double y;

};

#endif