#include <cmath>
#include <cstdint>
#include "MapRacer.h"

MapRacer::MapRacer(std::string pngfile, float _output_scale, float _output_crop_x_min, float _output_crop_y_min, float _output_crop_x_max, float _output_crop_y_max) {
    output_scale = _output_scale;
    output_crop_x_min = _output_crop_x_min;
    output_crop_y_min = _output_crop_y_min;
    output_crop_x_max = _output_crop_x_max;
    output_crop_y_max = _output_crop_y_max;
    load_png(pngfile);
    make_grayscale();
    resetVis();
}

void MapRacer::setStateRacerMath(StateRacerMath *_stateRacerMath) {
    stateRacerMath = _stateRacerMath;
}

void MapRacer::load_png(std::string pngfile) {
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

    output_width = image_width * output_scale;
    output_height = image_height * output_scale;

    output_width_cropped = output_width * (output_crop_x_max - output_crop_x_min);
    output_height_cropped = output_height * (output_crop_y_max - output_crop_y_min);

    output_shift_x = -(output_crop_x_min * output_width);
    output_shift_y = ((1.0 - output_crop_y_max) * output_height);

    vis_rows = (png_bytep*)malloc(sizeof(png_bytep) * output_height_cropped);
    for(int y = 0; y < output_height_cropped; y++) {
        vis_rows[y] = (png_byte*)malloc(png_get_rowbytes(png,info) * output_scale * (output_crop_x_max - output_crop_x_min));
    }

    png_read_image(png, image_rows);

    fclose(fp);

    png_destroy_read_struct(&png, &info, NULL);
}

void MapRacer::make_grayscale() {
    grayscale = (float*)malloc(image_width * image_height * sizeof(float));
    for (int height_pos = 0; height_pos < image_height; height_pos++) {
        png_bytep row = image_rows[height_pos];
        for (int width_pos = 0; width_pos < image_width; width_pos++) {
            png_bytep px = &(row[width_pos * 4]);
            grayscale[grayoffset(width_pos, height_pos)] = ((int)px[0] + (int)px[1] + (int)px[2]) / 765.0f;
        }
    }
}

void MapRacer::getBounds(StateRacer *minimums, StateRacer *maximums) {
    minimums->x = 0;
    minimums->y = 0;
    maximums->x = image_width;
    maximums->y = image_height;
    // this does not provide a velocity limit.  that is part of the StateMath configuration.
    // this does not provide heading limits, since they are fixed at [0, 360]
}

bool MapRacer::getPixelIsObstacle(int width_pos, int height_pos) {
    // coordinate system is +x right, +y up
    return grayscale[grayoffset(width_pos, image_height - height_pos - 1)] < 0.5f;
}
