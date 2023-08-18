#include <cmath>
#include <cstdint>
#include "Map2D.h"

Map2D::Map2D(std::string pngfile) {
    load_png(pngfile);
    make_grayscale();
    resetVis();
}

void Map2D::load_png(std::string pngfile) {
    FILE *fp = fopen(pngfile.c_str(), "rb");

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!png) abort();

    png_infop info = png_create_info_struct(png);
    if(!info) abort();

    if(setjmp(png_jmpbuf(png))) abort();

    png_init_io(png, fp);

    png_read_info(png, info);

    image_width = png_get_image_width(png, info);
    image_height = png_get_image_height(png, info);
    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth  = png_get_bit_depth(png, info);

    // Read any color_type into 8bit depth, RGBA format.
    // See http://www.libpng.org/pub/png/libpng-manual.txt

    if(bit_depth == 16)
        png_set_strip_16(png);

    if(color_type == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);

    // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
    if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
        png_set_expand_gray_1_2_4_to_8(png);

    if(png_get_valid(png, info, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png);

    // These color_type don't have an alpha channel then fill it with 0xff.
    if(color_type == PNG_COLOR_TYPE_RGB ||
       color_type == PNG_COLOR_TYPE_GRAY ||
       color_type == PNG_COLOR_TYPE_PALETTE)
        png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

    if(color_type == PNG_COLOR_TYPE_GRAY ||
       color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png);

    png_read_update_info(png, info);

    if (image_rows) abort();

    image_rows = (png_bytep*)malloc(sizeof(png_bytep) * image_height);
    for(int y = 0; y < image_height; y++) {
        image_rows[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
    }

    vis_rows = (png_bytep*)malloc(sizeof(png_bytep) * image_height);
    for(int y = 0; y < image_height; y++) {
        vis_rows[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
    }

    png_read_image(png, image_rows);

    fclose(fp);

    png_destroy_read_struct(&png, &info, NULL);
}

void Map2D::make_grayscale() {
    grayscale = (float*)malloc(image_width * image_height * sizeof(float));
    for (int height_pos = 0; height_pos < image_height; height_pos++) {
        png_bytep row = image_rows[height_pos];
        for (int width_pos = 0; width_pos < image_width; width_pos++) {
            png_bytep px = &(row[width_pos * 4]);
            grayscale[grayoffset(width_pos, height_pos)] = ((int)px[0] + (int)px[1] + (int)px[2]) / 765.0f;
        }
    }
}

void Map2D::getBounds(State2D *minimums, State2D *maximums) {
    minimums->set(0, 0);
    maximums->set(image_width, image_height);
}

bool Map2D::pointInObstacle(State2D *point) {
    return pointCost(point) > 0.25f;
}

bool Map2D::edgeInObstacle(State2D *pointA, State2D *pointB) {
    State2D diff(pointB->x - pointA->x, pointB->y - pointA->y);
    float step = EDGE_WALK_SCALE / hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        State2D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

float Map2D::pointCost(State2D *point) {
    int x = point->x;
    int y = point->y;
    if (x < 0 || x >= image_width || y < 0 || y >= image_height) {
        return 1.0f;
    }
    else {
        return 1.0f - grayscale[grayoffset(x, y)];
    }
}

float Map2D::edgeCost(State2D *pointA, State2D *pointB) {
    float sum = 0;
    State2D diff(pointB->x - pointA->x, pointB->y - pointA->y);
    int iterations = 0;
    float length = hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += EDGE_WALK_SCALE / length) {
        State2D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        sum += pointCost(&point);
        iterations++;
    }
    return iterations == 0 ? 0 : sum / iterations * length;
}

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
