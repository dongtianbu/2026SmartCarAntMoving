#ifndef color_tracer_H
#define color_tracer_H

#include "zf_common_headfile.h"
// 颜色识别算法参考链接
//  https://blog.csdn.net/niruxi0401/article/details/119685347

#define IMG_X               0               // 图片x偏移
#define IMG_Y               0               // 图片y偏移
#define IMG_W               SCC8660_W - 1   // 图片宽度
#define IMG_H               SCC8660_H - 1   // 图片高度

#define ALLOW_FAIL_PER      10              // 容错率，越大1<<ALLOW_FAIL_PER，越容易误识别，容错率越大越宽松
#define ITERATE_NUM         5               // 迭代次数，迭代次数越多，识别越准确，但速度越慢

#define CONDI_H_RANGE       30              // 设定颜色标定的色相范围（set_color_target_condi函数使用）
#define CONDI_S_RANGE       50              // 设定颜色标定的对比度范围（set_color_target_condi函数使用）
#define CONDI_L_RANGE       80              // 设定颜色标定的亮度范围（set_color_target_condi函数使用）

typedef struct{
    unsigned char           h_min;          // 目标最小色相
    unsigned char           h_max;          // 目标最大色相

    unsigned char           s_min;          // 目标最小饱和度
    unsigned char           s_max;          // 目标最大饱和度

    unsigned char           l_min;          // 目标最小亮度
    unsigned char           l_max;          // 目标最大亮度

    unsigned int            width_min;      // 目标最小宽度
    unsigned int            hight_min;      // 目标最小高度

    unsigned int            width_max;      // 目标最大宽度
    unsigned int            hight_max;      // 目标最大高度
}target_condi_struct;                       // 判定为目标的条件

typedef struct{
    unsigned int            x;              // 目标x坐标
    unsigned int            y;              // 目标y坐标
    unsigned int            w;              // 目标宽度
    unsigned int            h;              // 目标高度
}result_struct;                             // 识别结果

typedef struct{
    unsigned char           red;            // [0,255]
    unsigned char           green;          // [0,255]
    unsigned char           blue;           // [0,255]
}color_rgb_struct;                          // RGB格式颜色

typedef struct{
    unsigned char           hue;            // [0,240]
    unsigned char           saturation;     // [0,240]
    unsigned char           luminance;      // [0,240]
}color_hsl_struct;                          // HSL格式颜色

typedef struct{
    unsigned int            x_start;        // 搜索x起始位置
    unsigned int            x_end;          // 搜索x结束位置
    unsigned int            y_start;        // 搜索y起始位置
    unsigned int            y_end;          // 搜索y结束位置
}search_area_struct;                        // 搜索区域

extern target_condi_struct target_color_condi;  // 颜色目标阈值信息
extern result_struct target_pos_out;            // 目标位置信息

// 颜色追踪函数
int     color_trace             (const target_condi_struct *condition,result_struct *resu);
// 获取颜色目标阈值参数（通过图像中心点颜色获取）
void    set_color_target_condi  (uint16 rgb565_data, target_condi_struct* condition);
#endif
