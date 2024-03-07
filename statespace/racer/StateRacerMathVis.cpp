#include <cmath>
#include "StateRacerMathVis.h"
#include "Utils.h"

void StateRacerMathVis::setOutputPath(std::string _outputPath) {
    outputPath = _outputPath;
    mkpath(outputPath.c_str(), S_IRWXU);
    configured = true;
}

void StateRacerMathVis::renderLUT(ModelRacerEdgeCost *lut, int _vres, int _xres, int _yres, float _vmax) {
    if (!configured) return;

    vres = _vres;
    xres = _xres;
    yres = _yres;
    vmax = _vmax;

    // allocate memory for the image
    vis_rows = (png_bytep*)malloc(sizeof(png_bytep) * yres);
    for(int y = 0; y < yres; y++) {
        vis_rows[y] = (png_byte*)malloc(sizeof(png_bytep) * xres);
    }

    // find the max cost anywhere in the data
    float max_cost = 0;
    for (int i=0; i<vres*xres*yres; i++) {
        if (lut[i].cost > max_cost) max_cost = lut[i].cost;
    }

    // for each velocity
    for (int vidx=0; vidx < vres; vidx++) {

        // paint the x-y picture
        for (int xidx=0; xidx < xres; xidx++) {
            for (int yidx=0; yidx < yres; yidx++) {
                float cost = lut[lutindex(vidx, xidx, yidx)].cost;
                png_bytep row = vis_rows[yidx];
                if (cost > 0) {
                    int value = cost / max_cost * 255;
                    row[xidx * 4 + 0] = value;
                    row[xidx * 4 + 1] = value;
                    row[xidx * 4 + 2] = 128;
                    row[xidx * 4 + 3] = 255;
                }
                else {
                    row[xidx * 4 + 0] = 0;
                    row[xidx * 4 + 1] = 0;
                    row[xidx * 4 + 2] = 0;
                    row[xidx * 4 + 3] = 255;
                }
            }
        }

        // write out image
        float v = vidx * vmax / vres;
        write_png(outputPath + "lut_v" + std::to_string((int)floor(v)));
    }

}

int StateRacerMathVis::lutindex(int v, int x, int y) {
    int idx = v * xres * yres
              + x * yres
              + y;
    return idx;
}

void StateRacerMathVis::write_png(std::string filename_prefix) {
    int y;

    FILE *fp = fopen((filename_prefix + ".png").c_str(), "wb");
    if(!fp) abort();

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) abort();

    png_infop info = png_create_info_struct(png);
    if (!info) abort();

    if (setjmp(png_jmpbuf(png))) abort();

    png_init_io(png, fp);

    // Output is 8bit depth, RGBA format.
    png_set_IHDR(
            png,
            info,
            xres, yres,
            8,
            PNG_COLOR_TYPE_RGBA,
            PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_DEFAULT,
            PNG_FILTER_TYPE_DEFAULT
    );
    png_write_info(png, info);

    if (!vis_rows) abort();

    png_write_image(png, vis_rows);
    png_write_end(png, NULL);

    fclose(fp);

    png_destroy_write_struct(&png, &info);
}
