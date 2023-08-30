#include "Map3D.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

Map3D::Map3D(std::string datafile) {
    FILE* fp;
    fp = fopen(datafile.c_str(), "r");
    if (fp != NULL) {
        fscanf(fp, "%lf %lf %lf", &border.bound_lower.x, &border.bound_lower.y, &border.bound_lower.z);
        fscanf(fp, "%lf %lf %lf", &border.bound_upper.x, &border.bound_upper.y, &border.bound_upper.z);
        while (!feof(fp)) {
            fscanf(fp, "%lf %lf %lf", &objects[object_count].bound_lower.x, &objects[object_count].bound_lower.y, &objects[object_count].bound_lower.z);
            State3D size;
            fscanf(fp, "%lf %lf %lf", &size.x, &size.y, &size.z);
            objects[object_count].bound_upper.x = objects[object_count].bound_lower.x + size.x;
            objects[object_count].bound_upper.y = objects[object_count].bound_lower.y + size.y;
            objects[object_count].bound_upper.z = objects[object_count].bound_lower.z + size.z;
            object_count++;
        }
        fclose(fp);
    }
}

bool Map3D::pointInObstacle(State3D *point) {
    for (int i=0; i<object_count; i++) {
        bool x_inside = point->x > objects[i].bound_lower.x && point->x < objects[i].bound_upper.x;
        bool y_inside = point->y > objects[i].bound_lower.y && point->y < objects[i].bound_upper.y;
        bool z_inside = point->z > objects[i].bound_lower.z && point->z < objects[i].bound_upper.z;
        if (x_inside || y_inside || z_inside) {
            return true;
        }
    }
    return false;
}

bool Map3D::edgeInObstacle(State3D *pointA, State3D *pointB) {
    State3D diff(pointB->x - pointA->x, pointB->y - pointA->y, pointB->z - pointA->z);
    float step = EDGE_WALK_SCALE / sqrtf(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
    for (float progress = 0; progress < 1; progress += step) {
        State3D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress, pointA->z + diff.z * progress);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

float Map3D::pointCost(State3D *point) {
    if (point->x < border.bound_lower.x || point->x > border.bound_upper.x) return INFINITY;
    if (point->y < border.bound_lower.y || point->x > border.bound_upper.y) return INFINITY;
    if (point->z < border.bound_lower.z || point->x > border.bound_upper.z) return INFINITY;
    return pointInObstacle(point) ? INFINITY : 1;
}

float Map3D::edgeCost(State3D *pointA, State3D *pointB) {
    float sum = 0;
    State3D diff(pointB->x - pointA->x, pointB->y - pointA->y, pointB->z - pointA->z);
    int iterations = 0;
    float length = sqrtf(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
    for (float progress = 0; progress < 1; progress += EDGE_WALK_SCALE / length) {
        State3D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress, pointA->z + diff.z * progress);
        sum += pointCost(&point);
        iterations++;
    }
    return iterations == 0 ? 0 : sum / iterations * length;
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
    *minimums = border.bound_lower;
    *maximums = border.bound_upper;
}
