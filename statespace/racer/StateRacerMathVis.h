#ifndef RRT_STATERACERMATHVIS_H
#define RRT_STATERACERMATHVIS_H

#include "ModelRacerEdgeCost.h"
#include <string>
#include <png.h>

class StateRacerMath;

class StateRacerMathVis {

public:
    void setOutputPath(std::string _outputPath);
    void renderLUT(ModelRacerEdgeCost lut[], int vres, int xres, int yres, float vmax);

private:
    void write_png(std::string filename_prefix);
    int lutindex(int v0idx, int dforwardidx, int drightidx);

    std::string outputPath = "";
    bool configured = false;

    int vres = 0;
    int xres = 0;
    int yres = 0;
    float vmax = 0;
    png_bytep *vis_rows = NULL;

};


#endif //RRT_STATERACERMATHVIS_H
