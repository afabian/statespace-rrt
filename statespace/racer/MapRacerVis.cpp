#include <cmath>
#include <cstdint>
#include <libgen.h>
#include "MapRacer.h"
#include <iostream>
#include <fstream>


void MapRacer::configureVis(float _vmax) {
    vmax = _vmax;
}

void MapRacer::resetVis() {
    for (int height_pos = 0; height_pos < output_height_cropped; height_pos++) {
        png_bytep row = vis_rows[height_pos];
        for (int width_pos = 0; width_pos < output_width_cropped; width_pos++) {
            int x = (width_pos - output_shift_x) / output_scale;
            int y = (height_pos + output_shift_y) / output_scale;
            uint8_t value = grayscale[grayoffset(x,y)] * 255;
            if (value < 30) value = 30;
            if (value > 200) value = 200;
            row[width_pos * 4 + 0] = value;
            row[width_pos * 4 + 1] = value;
            row[width_pos * 4 + 2] = value;
            row[width_pos * 4 + 3] = 255;
        }
    }
}

void MapRacer::addVisPoint(StateRacer *point, unsigned int color, bool big) {
    if (point->x < 0 || point->x >= image_width) return;
    if (point->y < 0 || point->y >= image_height) return;

    if (big) {
        StateRacer point2 = *point; point2.x += 1.0/output_scale;
        addVisPoint(&point2, color);
        point2 = *point; point2.x -= 1.0/output_scale;
        addVisPoint(&point2, color);
        point2 = *point; point2.y += 1.0/output_scale;
        addVisPoint(&point2, color);
        point2 = *point; point2.y -= 1.0/output_scale;
        addVisPoint(&point2, color);
    }

    else {
        int x = int(point->x * output_scale) + output_shift_x;
        int y = int((image_height - point->y - 1) * output_scale) - output_shift_y;
        if (x >= 0 && y >= 0 && x < output_width_cropped && y < output_height_cropped) {
            png_bytep row = vis_rows[y];
            row[x * 4 + 0] = (color >> 0) & 0x000000ff;
            row[x * 4 + 1] = (color >> 8) & 0x000000ff;
            row[x * 4 + 2] = (color >> 16) & 0x000000ff;
            row[x * 4 + 3] = 255;
        }
    }
}

void MapRacer::addVisLine(StateRacer *pointA, StateRacer *pointB, unsigned int color) {
    if (pointA == pointB) return;
    StateRacer diff(pointB->x - pointA->x, pointB->y - pointA->y);
    int dist = int(hypotf(diff.x, diff.y));

    float step = 1.0f / dist / output_scale;
    for (float progress = 0; progress < 1; progress += step) {
        StateRacer point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        addVisPoint(&point, 0xffcccccc);
    }

    bool falseColor = color == 0xffffffff;

    StateRacer* points = (StateRacer*)malloc(sizeof(StateRacer) * dist * output_scale * 1.5);
    bool edgepath_ok = stateRacerMath->edgePath(pointA, pointB, points, dist * output_scale * 1.5);

    if (edgepath_ok) {
        for (int i = 0; i < dist * output_scale * 1.5; i++) {
            if (falseColor) {
                uint8_t brightness = fmin(255, points[i].v / vmax * 255);
                *((uint8_t *) &color + 0) = 255 - brightness;
                *((uint8_t *) &color + 1) = brightness;
                *((uint8_t *) &color + 2) = 0;
            }
            addVisPoint(&points[i], color);
        }
    }

    else {
        std::cout << "Cannot draw path from " << pointA->toString() << " to " << pointB->toString() << std::endl;
    }

    free(points);
}

void MapRacer::addDebugText(std::string text) {
    // TODO
}

void MapRacer::addGoalDetail(StateRacer* source, StateRacer* dest) {
}

void MapRacer::renderVis(std::string pngfile) {
    write_png(pngfile);
}

void MapRacer::write_png(std::string filename_prefix) {
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
            output_width_cropped, output_height_cropped,
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

void MapRacer::add_image_to_list(std::string filename_prefix) {
    std::string base_filename = filename_prefix.substr(filename_prefix.find_last_of("/\\") + 1) + ".png";
    filelist += "file '" + base_filename + "'\n";
    bool is_sample = base_filename.find("sample") != std::string::npos;
    float duration = is_sample ? 0.5 : 1.0;
    filelist += "duration " + std::to_string(duration) + "\n";
}

void MapRacer::renderFinalVis(std::string filename_prefix) {
    write_video(filename_prefix);
}

void MapRacer::write_video(std::string filename_prefix) {
    std::ofstream outfile(filename_prefix + ".txt");
    outfile << filelist;
    outfile.close();

    std::string cmd = "ffmpeg -y -f concat -i " + filename_prefix + ".txt -vf format=yuv420p -movflags +faststart " + filename_prefix + ".mp4";
    system(cmd.c_str());
}