#ifndef RRT_MAP2D_H
#define RRT_MAP2D_H

#include "State2D.h"
#include <string>
#include <png.h>

class Map2D {

public:
    Map2D(std::string pngfile);
    void getBounds(State2D* minimums, State2D* maximums);
    void resetVis();
    void addVisPoint(State2D* point, unsigned int color);
    void addVisLine(State2D* pointA, State2D* pointB, unsigned int color);
    void renderVis(std::string filename_prefix);
    float getGrayscalePixel(int width_pos, int height_pos);

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

};


#endif
