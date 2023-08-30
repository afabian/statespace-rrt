#include "State2D.h"

State2D::State2D() { }

State2D::State2D(double _x, double _y) {
    x = _x;
    y = _y;
}

State2D State2D::get() {
    State2D output;
    output.x = x;
    output.y = y;
    return output;
}

void State2D::set(State2D source) {
    x = source.x;
    y = source.y;
}

void State2D::set(State2D* source) {
    x = source->x;
    y = source->y;
}

void State2D::set(double _x, double _y) {
    x = _x;
    y = _y;
}

std::string State2D::toString() {
    return std::string("(" + std::to_string(x) + "," + std::to_string(y) + ")");
}

bool State2D::operator==(const State2D &other) {
    return x == other.x && y == other.y;
}