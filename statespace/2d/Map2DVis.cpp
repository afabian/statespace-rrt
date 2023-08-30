#include <cmath>
#include <cstdint>
#include "Map2D.h"

void Map2D::resetVis() {
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

void Map2D::addVisPoint(State2D *point, unsigned int color) {
    png_bytep row = vis_rows[(int)point->y];
    row[(int)point->x * 4 + 0] = (color >> 0) & 0x000000ff;
    row[(int)point->x * 4 + 1] = (color >> 8) & 0x000000ff;
    row[(int)point->x * 4 + 2] = (color >> 16) & 0x000000ff;
    row[(int)point->x * 4 + 3] = 255;
}

void Map2D::addVisLine(State2D *pointA, State2D *pointB, unsigned int color) {
    if (pointA == pointB) return;
    State2D diff(pointB->x - pointA->x, pointB->y - pointA->y);
    float step = 1.0f / hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        State2D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        addVisPoint(&point, color);
    }
}

void Map2D::renderVis(std::string pngfile) {
    write_png(pngfile);
}

void Map2D::write_png(std::string pngfile) {
    int y;

    FILE *fp = fopen(pngfile.c_str(), "wb");
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

    // To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
    // Use png_set_filler().
    //png_set_filler(png, 0, PNG_FILLER_AFTER);

    if (!vis_rows) abort();

    png_write_image(png, vis_rows);
    png_write_end(png, NULL);

    fclose(fp);

    png_destroy_write_struct(&png, &info);
}
