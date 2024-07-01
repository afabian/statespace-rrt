#include <cmath>
#include "StateRacerMathVis.h"
#include "utils.h"

void StateRacerMathVis::setOutputPath(std::string _outputPath) {
    outputPath = _outputPath;
    mkpath(outputPath.c_str(), S_IRWXU);
    configured = true;
}

int StateRacerMathVis::lutindex(int v0idx, int dforwardidx, int drightidx) {
    bool v0idx_ok = v0idx >= 0 && v0idx < vres;
    bool dforwardidx_ok = dforwardidx >= 0 && dforwardidx < xres;
    bool drightidx_ok = drightidx >= 0 && drightidx < yres;

    int idx = v0idx * xres * yres
              + dforwardidx * yres
              + drightidx;

    return v0idx_ok && dforwardidx_ok && drightidx_ok ? idx : -1;
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

    // for each v0
    for (int v0idx=0; v0idx < vres; v0idx++) {

        // paint the x-y picture
        for (int dforwardidx = 0; dforwardidx < xres; dforwardidx++) {
            for (int drightidx = 0; drightidx < yres; drightidx++) {
                float cost = lut[lutindex(v0idx, dforwardidx, drightidx)].cost;
                png_bytep row = vis_rows[yres - drightidx - 1];
                if (cost > 0) {
                    int value = cost / max_cost * 255;
                    row[dforwardidx * 4 + 0] = value;
                    row[dforwardidx * 4 + 1] = 255 - value;
                    row[dforwardidx * 4 + 2] = 0;
                    row[dforwardidx * 4 + 3] = 255;
                } else {
                    row[dforwardidx * 4 + 0] = 0;
                    row[dforwardidx * 4 + 1] = 0;
                    row[dforwardidx * 4 + 2] = 0;
                    row[dforwardidx * 4 + 3] = 255;
                }
            }
        }

        // write out image
        float v0 = v0idx * vmax / vres;
        write_png(outputPath + "lut_v" + std::to_string((int) floor(v0)));
    }

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
