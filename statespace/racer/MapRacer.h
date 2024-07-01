#ifndef RRT_MAPRACER_H
#define RRT_MAPRACER_H

#include "StateRacerMath.h"
#include "StateRacer.h"
#include <string>
#include <png.h>

class StateRacerMath;

class MapRacer {

public:
    MapRacer(std::string pngfile, float _output_scale=1, float _output_crop_x_min=0, float _output_crop_y_min=0, float _output_crop_x_max=1, float _output_crop_y_max=1);
    void setStateRacerMath(StateRacerMath* _stateRacerMath);
    void getBounds(StateRacer* minimums, StateRacer* maximums);
    bool getPixelIsObstacle(int width_pos, int height_pos);

    void configureVis(float _vmax);
    void resetVis();
    void addVisPoint(StateRacer* point, unsigned int color, bool big=false);
    void addVisLine(StateRacer* pointA, StateRacer* pointB, unsigned int color);
    void addGoalDetail(StateRacer* source, StateRacer* dest);
    void renderVis(std::string filename_prefix);
    void renderFinalVis(std::string filename_prefix);
    void addDebugText(std::string text);

private:
    void load_png(std::string pngfile);
    void make_grayscale();

    void write_png(std::string pngfile);
    void add_image_to_list(std::string filename_prefix);
    void write_video(std::string filename_prefix);
    inline int grayoffset(int width_pos, int height_pos) { return height_pos * image_width + width_pos; }

    StateRacerMath* stateRacerMath = nullptr;

    float vmax = 1;
    int image_width = 0;
    int image_height = 0;
    float output_scale = 1;
    float output_crop_x_min = 0;
    float output_crop_y_min = 0;
    float output_crop_x_max = 1;
    float output_crop_y_max = 1;
    int output_width = 0;
    int output_height = 0;
    int output_width_cropped = 0;
    int output_height_cropped = 0;
    int output_shift_x = 0;
    int output_shift_y = 0;
    png_bytep *image_rows = NULL;
    png_bytep *vis_rows = NULL;

    float* grayscale = nullptr;
    std::string filelist = "";

};


#endif
