#include "Map3D.h"

Map3D::Map3D(std::string datafile) {

}

bool Map3D::pointInObstacle(State3D *point) {
    return false;
}

bool Map3D::edgeInObstacle(State3D *pointA, State3D *pointB) {
    return false;
}

float Map3D::pointCost(State3D *point) {
    return 0;
}

float Map3D::edgeCost(State3D *pointA, State3D *pointB) {
    return 0;
}

void Map3D::resetVis() {

}

void Map3D::addVisPoint(State3D *point, int color) {

}

void Map3D::addVisLine(State3D *pointA, State3D *pointB, int color) {

}

void Map3D::renderVis(std::string pngfile) {

}

void Map3D::getBounds(State3D *minimums, State3D *maximums) {

}
