#include "StateFloater.h"

StateFloater::StateFloater() { }

StateFloater::StateFloater(double _t, double _y, double _vy) {
    t = _t;
    y = _y;
    vy = _vy;
}

StateFloater StateFloater::get() {
    StateFloater output;
    output.t = t;
    output.y = y;
    output.vy = vy;
    return output;
}

void StateFloater::set(StateFloater source) {
    t = source.t;
    y = source.y;
    vy = source.vy;
}

void StateFloater::set(StateFloater* source) {
    t = source->t;
    y = source->y;
    vy = source->vy;
}

void StateFloater::set(double _t, double _y, double _vy) {
    t = _t;
    y = _y;
    vy = _vy;
}

std::string StateFloater::toString() {
    return std::string("(" + std::to_string(t) + ":" + std::to_string(y)  + "@" + std::to_string(vy) + ")");
}

bool StateFloater::operator==(const StateFloater &other) {
    return t == other.t && y == other.y && vy == other.vy;
}
