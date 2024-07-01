#ifndef STATERACER_H
#define STATERACER_H

#include <string>

class StateRacer {

    friend class StateRacerMath;
    friend class MapRacer;
    friend class ModelRacer;

public:
    StateRacer();
    StateRacer(double _x, double _y);
    StateRacer(double _x, double _y, double _v, double _h);
    StateRacer get();
    void set(StateRacer source);
    void set(StateRacer* source);
    void set(double _x, double _y, double _v, double _h);
    std::string toString();
    bool operator==(const StateRacer &other);

protected:
    double x;
    double y;
    double v;
    double h;

};

#endif