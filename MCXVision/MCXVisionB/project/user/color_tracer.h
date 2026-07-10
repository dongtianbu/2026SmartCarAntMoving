#ifndef color_tracer_H
#define color_tracer_H

#include "zf_common_headfile.h"

#define COLOR_GRID_STRIDE       8u
#define COLOR_GRID_W            ((SCC8660_W + COLOR_GRID_STRIDE - 1u) / COLOR_GRID_STRIDE)
#define COLOR_GRID_H            ((SCC8660_H + COLOR_GRID_STRIDE - 1u) / COLOR_GRID_STRIDE)

typedef struct
{
    unsigned char           h_min;
    unsigned char           h_max;
    unsigned char           s_min;
    unsigned char           s_max;
    unsigned char           l_min;
    unsigned char           l_max;
    unsigned char           hue_ref;
    unsigned char           hue_wrap;
    unsigned char           valid;
    unsigned char           reserved;
    unsigned int            width_min;
    unsigned int            hight_min;
    unsigned int            width_max;
    unsigned int            hight_max;
    unsigned int            area_min;
    unsigned int            area_max;
    unsigned int            aspect_min_x100;
    unsigned int            aspect_max_x100;
    unsigned int            fill_min_x100;
} target_condi_struct;

typedef struct
{
    unsigned int            x;
    unsigned int            y;
    unsigned int            w;
    unsigned int            h;
    unsigned int            pixels;
    unsigned int            score;
} result_struct;

typedef struct
{
    unsigned char           red;
    unsigned char           green;
    unsigned char           blue;
} color_rgb_struct;

typedef struct
{
    unsigned char           hue;
    unsigned char           saturation;
    unsigned char           luminance;
} color_hsl_struct;

extern target_condi_struct target_color_condi;
extern result_struct target_pos_out;

int     color_trace             (const target_condi_struct *condition, result_struct *resu);
void    set_color_target_condi  (uint16 rgb565_data, target_condi_struct *condition);
void    color_trace_reset       (void);
uint8   color_trace_is_ready    (const target_condi_struct *condition);

#endif
