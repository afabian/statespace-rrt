#include "State3D.h"

State3D::State3D() { }

State3D::State3D(double _x, double _y, double _z) {
    x = _x;
    y = _y;
    z = _z;
}

State3D State3D::get() {
    State3D output;
    output.x = x;
    output.y = y;
    output.z = z;
    return output;
}

void State3D::set(State3D source) {
    x = source.x;
    y = source.y;
    z = source.z;
}

void State3D::set(State3D* source) {
    x = source->x;
    y = source->y;
    z = source->z;
}

void State3D::set(double _x, double _y, double _z) {
    x = _x;
    y = _y;
    z = _z;
}

std::string State3D::toString() {
    return std::string("(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")");
}

bool State3D::operator==(const State3D &other) {
    return x == other.x && y == other.y && z == other.z;
}