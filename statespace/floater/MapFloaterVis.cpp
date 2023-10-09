#include <cmath>
#include <cstdint>
#include <libgen.h>
#include "MapFloater.h"
#include <iostream>
#include <fstream>


void MapFloater::configureVis(int width, int height) {
    // MapFloater doesn't have a configurable output size
}

void MapFloater::resetVis() {
    for (int height_pos = 0; height_pos < image_height; height_pos++) {
        png_bytep row = vis_rows[height_pos];
        for (int width_pos = 0; width_pos < image_width; width_pos++) {
            uint8_t value = grayscale[grayoffset(width_pos, height_pos)] * 255;
            row[width_pos * 4 + 0] = value;
            row[width_pos * 4 + 1] = value;
            row[width_pos * 4 + 2] = value;
            row[width_pos * 4 + 3] = 255;
        }
    }
}

void MapFloater::addVisPoint(StateFloater *point, unsigned int color) {
    png_bytep row = vis_rows[(int)point->y];
    row[(int)point->t * 4 + 0] = (color >> 0) & 0x000000ff;
    row[(int)point->t * 4 + 1] = (color >> 8) & 0x000000ff;
    row[(int)point->t * 4 + 2] = (color >> 16) & 0x000000ff;
    row[(int)point->t * 4 + 3] = 255;
}

void MapFloater::addVisLine(StateFloater *pointA, StateFloater *pointB, unsigned int color) {
    if (pointA == pointB) return;
    StateFloater diff(pointB->t - pointA->t, pointB->y - pointA->y, 0);
    float step = 1.0f / hypotf(diff.t, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        StateFloater point(pointA->t + diff.t * progress, pointA->y + diff.y * progress, 0);
        addVisPoint(&point, color);
    }
}

void MapFloater::renderVis(std::string pngfile) {
    write_png(pngfile);
}

void MapFloater::write_png(std::string filename_prefix) {
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
            image_width, image_height,
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

    add_image_to_list(filename_prefix);
}

void MapFloater::add_image_to_list(std::string filename_prefix) {
    std::string base_filename = filename_prefix.substr(filename_prefix.find_last_of("/\\") + 1) + ".png";
    filelist += "file '" + base_filename + "'\n";
    bool is_sample = base_filename.find("sample") != std::string::npos;
    float duration = is_sample ? 0.5 : 1.0;
    filelist += "duration " + std::to_string(duration) + "\n";
}

void MapFloater::renderFinalVis(std::string filename_prefix) {
    write_video(filename_prefix);
}

void MapFloater::write_video(std::string filename_prefix) {
    std::ofstream outfile(filename_prefix + ".txt");
    outfile << filelist;
    outfile.close();

    std::string cmd = "ffmpeg -y -f concat -i " + filename_prefix + ".txt -vf format=yuv420p -movflags +faststart " + filename_prefix + ".mp4";
    system(cmd.c_str());
}