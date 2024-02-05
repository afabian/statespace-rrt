#include "StateRacer.h"

StateRacer::StateRacer() { }

StateRacer::StateRacer(double _x, double _y) {
    x = _x;
    y = _y;
}

StateRacer::StateRacer(double _t, double _x, double _y, double _v, double _h) {
    t = _t;
    x = _x;
    y = _y;
    v = _v;
    h = _h;
}

StateRacer StateRacer::get() {
    StateRacer output;
    output.t = t;
    output.x = x;
    output.y = y;
    output.v = v;
    output.h = h;
    return output;
}

void StateRacer::set(StateRacer source) {
    t = source.t;
    x = source.x;
    y = source.y;
    v = source.v;
    h = source.h;
}

void StateRacer::set(StateRacer* source) {
    t = source->t;
    x = source->x;
    y = source->y;
    v = source->v;
    h = source->h;
}

void StateRacer::set(double _t, double _x, double _y, double _v, double _h) {
    t = _t;
    x = _x;
    y = _y;
    v = _v;
    h = _h;
}

std::string StateRacer::toString() {
    return std::string(std::string("(")
           + "t=" + std::to_string(t) + ","
           + "x=" + std::to_string(x) + ","
           + "y=" + std::to_string(y) + ","
           + "v=" + std::to_string(v) + ","
           + "h=" + std::to_string(h) + ","
           + std::string(")"));
}

bool StateRacer::operator==(const StateRacer &other) {
    return t == other.t
        && x == other.x
        && y == other.y
        && v == other.v
        && h == other.h
        ;
}
