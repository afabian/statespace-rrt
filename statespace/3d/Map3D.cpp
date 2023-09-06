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
