#ifndef COLOR_TRACER_H
#define COLOR_TRACER_H

#include "zf_common_headfile.h"

// ==================== 可调试参数区 ====================
// 亮点检测网格边长，值越小越容易保留小光点，但计算量会增大。
#define LIGHT_GRID_STRIDE                4u
// 网格列数，由网格步长自动推导。
#define LIGHT_GRID_W                     ((SCC8660_W + LIGHT_GRID_STRIDE - 1u) / LIGHT_GRID_STRIDE)
// 网格行数，由网格步长自动推导。
#define LIGHT_GRID_H                     ((SCC8660_H + LIGHT_GRID_STRIDE - 1u) / LIGHT_GRID_STRIDE)
// 需要保留的最强亮点数量，当前场景固定为两个红外光点。
#define LIGHT_POINT_MAX_COUNT            2u
// ==================== 可调试参数区 ====================

typedef struct
{
    uint8                   valid;
    uint8                   reserved0;
    uint16                  reserved1;
    unsigned int            x;
    unsigned int            y;
    unsigned int            w;
    unsigned int            h;
    unsigned int            pixels;
    unsigned int            peak_value;
    unsigned int            score;
} light_point_result_struct;

typedef struct
{
    uint8                   count;
    uint8                   reserved0;
    uint16                  reserved1;
    light_point_result_struct points[LIGHT_POINT_MAX_COUNT];
    light_point_result_struct merged;
} light_trace_result_struct;

extern light_trace_result_struct light_trace_out;

int     color_trace             (light_trace_result_struct *result);
void    color_trace_reset       (void);

#endif
