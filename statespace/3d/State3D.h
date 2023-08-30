#ifndef STATE3D_H
#define STATE3D_H

#include <string>

class State3D {
  
friend class State3DMath;
friend class Map3D;

public:
    State3D();
    State3D(double _x, double _y, double _z);
    State3D get();
    void set(State3D source);
    void set(State3D* source);
    void set(double _x, double _y, double _z);
    std::string toString();
    bool operator==(const State3D &other);

protected:
    double x;
    double y;
    double z;

};

#endif