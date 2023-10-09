#ifndef RRT_MAPFLOATER_H
#define RRT_MAPFLOATER_H

#include "StateFloater.h"
#include <string>
#include <png.h>

class MapFloater {

public:
    MapFloater(std::string pngfile);
    void getBounds(StateFloater* minimums, StateFloater* maximums);
    void configureVis(int width, int height);
    void resetVis();
    void addVisPoint(StateFloater* point, unsigned int color);
    void addVisLine(StateFloater* pointA, StateFloater* pointB, unsigned int color);
    void renderVis(std::string filename_prefix);
    void renderFinalVis(std::string filename_prefix);
    float getGrayscalePixel(int width_pos, int height_pos);

private:
    void load_png(std::string pngfile);
    void write_png(std::string pngfile);
    void add_image_to_list(std::string filename_prefix);
    void write_video(std::string filename_prefix);
    void make_grayscale();
    inline int grayoffset(int width_pos, int height_pos) { return height_pos * image_width + width_pos; }

    int image_width = 0;
    int image_height = 0;
    png_bytep *image_rows = NULL;
    png_bytep *vis_rows = NULL;

    float* grayscale = nullptr;
    std::string filelist = "";

};


#endif
