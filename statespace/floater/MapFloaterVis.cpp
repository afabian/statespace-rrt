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

void MapFloater::addVisPoint(StateFloater *point, unsigned int color, bool big) {
    if (point->t < 0 || point->t >= image_width) return;
    if (point->y < 0 || point->y >= image_height) return;

    if (big) {
        StateFloater point2 = *point; point2.t++;
        addVisPoint(&point2, color);
        point2 = *point; point2.t--;
        addVisPoint(&point2, color);
        point2 = *point; point2.y++;
        addVisPoint(&point2, color);
        point2 = *point; point2.y--;
        addVisPoint(&point2, color);
    }

    else {
        png_bytep row = vis_rows[(int) point->y];
        uint8_t newpoint[4];
        newpoint[0] = (color >> 0) & 0x000000ff;
        newpoint[1] = (color >> 8) & 0x000000ff;
        newpoint[2] = (color >> 16) & 0x000000ff;
        newpoint[3] = 255;

        row[(int) point->t * 4 + 0] = row[(int) point->t * 4 + 0] / 2 + newpoint[0] / 2;
        row[(int) point->t * 4 + 1] = row[(int) point->t * 4 + 1] / 2 + newpoint[1] / 2;
        row[(int) point->t * 4 + 2] = row[(int) point->t * 4 + 2] / 2 + newpoint[2] / 2;
        row[(int) point->t * 4 + 3] = row[(int) point->t * 4 + 3] / 2 + newpoint[3] / 2;
    }
}

void MapFloater::addVisLine(StateFloater *source, StateFloater *dest, unsigned int color) {
    if (source == dest) return;

    int points = (int)ceilf(dest->t - source->t) + 1;
    float* p = (float*)malloc(sizeof(float) * points);
    float* a = (float*)malloc(sizeof(float) * points);
    float* t = (float*)malloc(sizeof(float) * points);
    stateFloaterMath->edgePath(source, dest, t, p, a, points);

    StateFloater last_point, point;

    last_point.t = t[0];
    last_point.y = p[0];

    for (int i=1; i<points; i++) {
        point.t = t[i];
        point.y = p[i];
        addStraightLine(last_point, point, color);
        last_point = point;
    }

    free(p);
    free(a);
    free(t);
}

void MapFloater::addGoalDetail(StateFloater *source, StateFloater *dest) {
    int points = (int)ceilf(dest->t - source->t) + 1;
    float* p = (float*)malloc(sizeof(float) * points);
    float* a = (float*)malloc(sizeof(float) * points);
    float* t = (float*)malloc(sizeof(float) * points);
    stateFloaterMath->edgePath(source, dest, t, p, a, points);

    for (int step=0; step<points; step++) {
        StateFloater state;
        state.t = t[step];
        state.y = p[step];
        state.vy = a[step];
        add_state_display(state);
    }

    free(p);
    free(a);
    free(t);
}

void MapFloater::add_state_display(StateFloater state) {

    unsigned int color = 0x000000ff;

    int t = state.t;
    int y0 = image_height - 50;
    int y1 = image_height - 50 - (state.vy * accel_scale);

    png_bytep row = vis_rows[y0];
    row[(int) t * 4 + 0] = (color >> 0) & 0x000000ff;
    row[(int) t * 4 + 1] = (color >> 8) & 0x000000ff;
    row[(int) t * 4 + 2] = (color >> 16) & 0x000000ff;
    row[(int) t * 4 + 3] = 255;

    for (int y=y0; y!=y1; y+=sign(y1-y0)) {
        if (y >= 0 && y < image_height) {
            png_bytep row = vis_rows[y];
            row[(int) t * 4 + 0] = (color >> 0) & 0x000000ff;
            row[(int) t * 4 + 1] = (color >> 8) & 0x000000ff;
            row[(int) t * 4 + 2] = (color >> 16) & 0x000000ff;
            row[(int) t * 4 + 3] = 255;
        }
    }

}

void MapFloater::addStraightLine(StateFloater a, StateFloater b, unsigned int color) {
    StateFloater point;
    float dist = hypotf(b.t - a.t, b.y - a.y);
    float dist_per_px = 1;
    int steps = (int)ceilf(dist / dist_per_px);
    for (int i=0; i<steps; i++) {
        float scale = (float)i / steps;
        point.t = a.t + (b.t - a.t) * scale;
        point.y = a.y + (b.y - a.y) * scale;
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