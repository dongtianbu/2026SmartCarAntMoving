#include "color_tracer.h"

#define min3v(v1, v2, v3)   ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v1)))
#define max3v(v1, v2, v3)   ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))
#define SWAPBYTE(h) ((((uint16_t)h << 8)&0xFF00) | ((uint16_t)h >> 8))

// ============================================
// 深蓝色沙包优化参数 (基于HSL颜色空间)
// ============================================
// H(色相): 125-175  蓝色范围
// S(饱和度): 80-240  中高饱和度  
// L(亮度): 15-140    较暗到中等亮度
// 尺寸: 15-180像素 (适应不同距离)
target_condi_struct target_color_condi = {125, 175, 80, 240, 15, 140, 15, 15, 180, 180};
result_struct target_pos_out = {0};

static void readcolor(unsigned int x, unsigned int y, color_rgb_struct* rgb)
{
    unsigned short c16;
    c16 = SWAPBYTE(*((uint16*)g_camera_buffer + y * SCC8660_W + x));
    rgb->red   = (unsigned char)((c16 & 0xf800) >> 8);
    rgb->green = (unsigned char)((c16 & 0x07e0) >> 3);
    rgb->blue  = (unsigned char)((c16 & 0x001f) << 3);
}

static void rgbtohsl(const color_rgb_struct* rgb, color_hsl_struct* hsl)
{
    int h, s, l, maxval, minval, difval;
    int r  = rgb->red;
    int g  = rgb->green;
    int b  = rgb->blue;

    maxval = max3v(r, g, b);
    minval = min3v(r, g, b);

    difval = maxval - minval;

    // 计算亮度
    l = (maxval + minval) * 240 / 255 / 2;

    if(maxval == minval)
    {
        h = 0;
        s = 0;
    }
    else
    {
        // 计算色相
        if(maxval == r)
        {
            if(g >= b)
            {
                h = 40 * (g - b) / (difval);
            }
            else
            {
                h = 40 * (g - b) / (difval) + 240;
            }
        }
        else if(maxval == g)
        {
            h = 40 * (b - r) / (difval) + 80;
        }
        else if(maxval == b)
        {
            h = 40 * (r - g) / (difval) + 160;
        }
        // 计算饱和度
        if(l == 0)
        {
            s = 0;
        }
        else if(l <= 120)
        {
            s = (difval) * 240 / (maxval + minval);
        }
        else
        {
            s = (difval) * 240 / (480 - (maxval + minval));
        }
    }
    hsl->hue = (unsigned char)(((h > 240) ? 240 : ((h < 0) ? 0 : h)));
    hsl->saturation = (unsigned char)(((s > 240) ? 240 : ((s < 0) ? 0 : s)));
    hsl->luminance = (unsigned char)(((l > 240) ? 240 : ((l < 0) ? 0 : l)));
}

static int colormatch(const color_hsl_struct* hsl, const target_condi_struct* condition)
{
    if(
        hsl->hue        >=  condition->h_min &&
        hsl->hue        <=  condition->h_max &&
        hsl->saturation >=  condition->s_min &&
        hsl->saturation <=  condition->s_max &&
        hsl->luminance  >=  condition->l_min &&
        hsl->luminance  <=  condition->l_max
    )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static int searchcentre(unsigned int* x, unsigned int* y, const target_condi_struct* condition, const search_area_struct* area)
{
    unsigned int spacex, spacey, i, j, k, failcount = 0;
    color_rgb_struct rgb;
    color_hsl_struct hsl;

    spacex = condition->width_min / 3;
    spacey = condition->hight_min / 3;

    for(i = area->y_start; i < area->y_end; i += spacey)
    {
        for(j = area->x_start; j < area->x_end; j += spacex)
        {
            failcount = 0;
            for(k = 0; k < spacex + spacey; k++)
            {
                if(k < spacex)
                {
                    readcolor(j + k, i + spacey / 2, &rgb);
                }
                else
                {
                    readcolor(j + spacex / 2, i + (k - spacex), &rgb);
                }
                rgbtohsl(&rgb, &hsl);

                if(!colormatch(&hsl, condition))
                {
                    failcount++;
                }
                if(failcount > ((spacex + spacey) >> ALLOW_FAIL_PER))
                {
                    break;
                }
            }
            if(k == spacex + spacey)
            {
                *x = j + spacex / 2;
                *y = i + spacey / 2;
                return 1;
            }
        }
    }
    return 0;
}

static int corrode(unsigned int oldx, unsigned int oldy, const target_condi_struct* condition, result_struct* resu)
{
    unsigned int xmin, xmax, ymin, ymax, i, failcount = 0;
    color_rgb_struct rgb;
    color_hsl_struct hsl;

    for(i = oldx; i > IMG_X; i--)
    {
        readcolor(i, oldy, &rgb);
        rgbtohsl(&rgb, &hsl);
        if(!colormatch(&hsl, condition))
        {
            failcount++;
        }
        if(failcount > (((condition->width_min + condition->width_max) >> 2) >> ALLOW_FAIL_PER))
        {
            break;
        }
    }
    xmin = i;
    failcount = 0;

    for(i = oldx; i < IMG_X + IMG_W; i++)
    {
        readcolor(i, oldy, &rgb);
        rgbtohsl(&rgb, &hsl);
        if(!colormatch(&hsl, condition))
        {
            failcount++;
        }
        if(failcount > (((condition->width_min + condition->width_max) >> 2) >> ALLOW_FAIL_PER))
        {
            break;
        }
    }
    xmax = i;
    failcount = 0;

    for(i = oldy; i > IMG_Y; i--)
    {
        readcolor(oldx, i, &rgb);
        rgbtohsl(&rgb, &hsl);
        if(!colormatch(&hsl, condition))
        {
            failcount++;
        }
        if(failcount > (((condition->hight_min + condition->hight_max) >> 2) >> ALLOW_FAIL_PER))
        {
            break;
        }
    }
    ymin = i;
    failcount = 0;

    for(i = oldy; i < IMG_Y + IMG_H; i++)
    {
        readcolor(oldx, i, &rgb);
        rgbtohsl(&rgb, &hsl);
        if(!colormatch(&hsl, condition))
        {
            failcount++;
        }
        if(failcount > (((condition->hight_min + condition->hight_max) >> 2) >> ALLOW_FAIL_PER))
        {
            break;
        }
    }
    ymax = i;
    failcount = 0;

    resu->x = (xmin + xmax) / 2;
    resu->y = (ymin + ymax) / 2;
    resu->w = xmax - xmin;
    resu->h = ymax - ymin;

    if(((xmax - xmin) > (condition->width_min)) && ((ymax - ymin) > (condition->hight_min)) && \
            ((xmax - xmin) < (condition->width_max)) && ((ymax - ymin) < (condition->hight_max)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称   通过rgb565数据获取目标颜色识别颜色范围
// 参数说明   rgb565_data                     rgb565数据，可直接从摄像头获取
// 参数说明   condition                       颜色条件颜色阈值结构体
// 使用示例   set_color_target_condi(scc8660_image[60][80], &target_color_condi);
//-------------------------------------------------------------------------------------------------------------------
void set_color_target_condi(uint16 rgb565_data, target_condi_struct* condition)
{
    color_rgb_struct rgb;
    color_hsl_struct hsl;
    rgb.red   = (unsigned char)((SWAPBYTE(rgb565_data) & 0xf800) >> 8);
    rgb.green = (unsigned char)((SWAPBYTE(rgb565_data) & 0x07e0) >> 3);
    rgb.blue  = (unsigned char)((SWAPBYTE(rgb565_data) & 0x001f) << 3);

    rgbtohsl(&rgb, &hsl);


    if(hsl.hue > CONDI_H_RANGE)
    {
        condition->h_min = hsl.hue - CONDI_H_RANGE;
    }
    else
    {
        condition->h_min = 0;
    }
    if(hsl.hue < (240 - CONDI_H_RANGE))
    {
        condition->h_max = hsl.hue + CONDI_H_RANGE;
    }
    else
    {
        condition->h_max = 240;
    }

    if(hsl.saturation > CONDI_S_RANGE)
    {
        condition->s_min = hsl.saturation - CONDI_S_RANGE;
    }
    else
    {
        condition->s_min = 0;
    }
    if(hsl.saturation < (240 - CONDI_S_RANGE))
    {
        condition->s_max = hsl.saturation + CONDI_S_RANGE;
    }
    else
    {
        condition->s_max = 240;
    }


    if(hsl.luminance > CONDI_L_RANGE)
    {
        condition->l_min = hsl.luminance - CONDI_L_RANGE;
    }
    else
    {
        condition->l_min = 0;
    }
    if(hsl.luminance < (240 - CONDI_L_RANGE))
    {
        condition->l_max = hsl.luminance + CONDI_L_RANGE;
    }
    else
    {
        condition->l_max = 240;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称   颜色识别
// 参数说明   target_condi_struct             颜色条件颜色阈值结构体
// 参数说明   resu                            目标位置信息结构体
// 返回参数   int                             返回1表示识别目标，返回0表示未识别
// 使用示例   color_trace(&target_color_condi, &target_pos_out)
//-------------------------------------------------------------------------------------------------------------------
int color_trace(const target_condi_struct* condition, result_struct* resu)
{
    unsigned int i;
    unsigned int x0 = 0, y0 = 0, flag = 0;
    search_area_struct area = {IMG_X, IMG_X + IMG_W, IMG_Y, IMG_Y + IMG_H};
    result_struct result;
    if(flag == 0)
    {
        if(searchcentre(&x0, &y0, condition, &area))
        {
            flag = 1;
        }
        else
        {

            if(searchcentre(&x0, &y0, condition, &area))
            {
                flag = 0;
                return 0;
            }
        }
    }
    result.x = x0;
    result.y = y0;

    for(i = 0; i < ITERATE_NUM; i++)
    {
        corrode(result.x, result.y, condition, &result);
    }

    if(corrode(result.x, result.y, condition, &result))
    {
        x0 = result.x;
        y0 = result.y;
        resu->x = result.x;
        resu->y = result.y;
        resu->w = result.w;
        resu->h = result.h;
        flag = 1;
        area.x_start = result.x - ((result.w) >> 1);
        area.x_end   = result.x + ((result.w) >> 1);
        area.y_start = result.y - ((result.h) >> 1);
        area.y_end   = result.y + ((result.h) >> 1);
        return 1;
    }
    else
    {
        flag = 0;
        return 0;
    }

}
