#ifndef RRT_MAPRACER_H
#define RRT_MAPRACER_H

#include "StateRacer.h"
#include <string>
#include <png.h>

class MapRacer {

public:
    MapRacer(std::string pngfile);
    void getBounds(StateRacer* minimums, StateRacer* maximums);
    bool getPixelIsObstacle(int width_pos, int height_pos);

    void configureVis(int width, int height);
    void resetVis();
    void addVisPoint(StateRacer* point, unsigned int color, bool big=false);
    void addVisLine(StateRacer* pointA, StateRacer* pointB, unsigned int color);
    void addGoalDetail(StateRacer* source, StateRacer* dest);
    void renderVis(std::string filename_prefix);
    void renderFinalVis(std::string filename_prefix);

private:
    void load_png(std::string pngfile);
    void make_grayscale();

    void write_png(std::string pngfile);
    void add_image_to_list(std::string filename_prefix);
    void write_video(std::string filename_prefix);
    inline int grayoffset(int width_pos, int height_pos) { return height_pos * image_width + width_pos; }

    int image_width = 0;
    int image_height = 0;
    png_bytep *image_rows = NULL;
    png_bytep *vis_rows = NULL;

    float* grayscale = nullptr;
    std::string filelist = "";

};


#endif
