#ifndef STATEFLOATER_H
#define STATEFLOATER_H

#include <string>

class StateFloater {

friend class StateFloaterMath;
friend class MapFloater;

public:
    StateFloater();
    StateFloater(double _t, double _y, double _vy);    
    StateFloater get();
    void set(StateFloater source);
    void set(StateFloater* source);
    void set(double _t, double _y, double _vy);
    std::string toString();

protected:
    double t;
    double y;
    double vy;

};

#endif