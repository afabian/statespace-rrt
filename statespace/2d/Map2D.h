#ifndef RRT_MAP2D_H
#define RRT_MAP2D_H

#include "State2D.h"
#include <string>
#include <png.h>

class Map2D {

public:
    Map2D(std::string pngfile);

    void setCostScale(float scale);

    void getBounds(State2D* minimums, State2D* maximums);

    bool pointInObstacle(State2D* point);
    bool edgeInObstacle(State2D* pointA, State2D* pointB);
    float pointCost(State2D* point);
    float edgeCost(State2D* pointA, State2D* pointB);

    void resetVis();
    void addVisPoint(State2D* point, unsigned int color);
    void addVisLine(State2D* pointA, State2D* pointB, unsigned int color);
    void renderVis(std::string filename_prefix);

private:
    void load_png(std::string pngfile);
    void write_png(std::string pngfile);
    void make_grayscale();
    inline int grayoffset(int width_pos, int height_pos) { return height_pos * image_width + width_pos; }

    int image_width = 0;
    int image_height = 0;
    png_bytep *image_rows = NULL;
    png_bytep *vis_rows = NULL;

    float* grayscale = nullptr;
    const float EDGE_WALK_SCALE = 1.0f;

    float cost_scale = 1;
};


#endif
