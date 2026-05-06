#include "color_tracer.h"

#define SWAPBYTE(h)                 ((((uint16_t)(h) << 8) & 0xFF00u) | ((uint16_t)(h) >> 8))
#define COLOR_SAMPLE_RADIUS         4
#define COLOR_CELL_MATCH_COUNT      3
#define COLOR_TRACK_KEEP_FRAMES     2

#define COLOR_DEFAULT_WIDTH_MIN     14u
#define COLOR_DEFAULT_HEIGHT_MIN    14u
#define COLOR_DEFAULT_AREA_MIN      120u
#define COLOR_DEFAULT_ASPECT_MIN    45u
#define COLOR_DEFAULT_ASPECT_MAX    240u
#define COLOR_DEFAULT_FILL_MIN      28u

typedef struct
{
    uint16 min_x;
    uint16 max_x;
    uint16 min_y;
    uint16 max_y;
    uint16 cells;
} component_info_t;

target_condi_struct target_color_condi = {0};
result_struct target_pos_out = {0};

static uint8 s_grid_mask[COLOR_GRID_H][COLOR_GRID_W] = {{0}};
static uint8 s_grid_visit[COLOR_GRID_H][COLOR_GRID_W] = {{0}};
static uint16 s_queue[COLOR_GRID_W * COLOR_GRID_H] = {0};
static result_struct s_last_result = {0};
static uint8 s_last_result_valid = 0u;
static uint8 s_lost_frames = 0u;

static unsigned int umin(unsigned int a, unsigned int b)
{
    return (a < b) ? a : b;
}

static unsigned int umax(unsigned int a, unsigned int b)
{
    return (a > b) ? a : b;
}

static unsigned int clampu(unsigned int value, unsigned int low, unsigned int high)
{
    if(value < low)
    {
        return low;
    }
    if(value > high)
    {
        return high;
    }
    return value;
}

static unsigned int sat_sub(unsigned int value, unsigned int delta)
{
    return (value > delta) ? (value - delta) : 0u;
}

static int absi(int value)
{
    return (value < 0) ? -value : value;
}

static void readcolor(unsigned int x, unsigned int y, color_rgb_struct *rgb)
{
    uint16 c16;
    c16 = SWAPBYTE(*((uint16 *)g_camera_buffer + y * SCC8660_W + x));
    rgb->red   = (unsigned char)((c16 & 0xF800u) >> 8);
    rgb->green = (unsigned char)((c16 & 0x07E0u) >> 3);
    rgb->blue  = (unsigned char)((c16 & 0x001Fu) << 3);
}

static void rgbtohsl(const color_rgb_struct *rgb, color_hsl_struct *hsl)
{
    int h = 0;
    int s = 0;
    int l;
    int maxval;
    int minval;
    int difval;
    int r = rgb->red;
    int g = rgb->green;
    int b = rgb->blue;

    maxval = r;
    if(g > maxval) maxval = g;
    if(b > maxval) maxval = b;

    minval = r;
    if(g < minval) minval = g;
    if(b < minval) minval = b;

    difval = maxval - minval;
    l = (maxval + minval) * 240 / 255 / 2;

    if(difval != 0)
    {
        if(maxval == r)
        {
            h = (g >= b) ? (40 * (g - b) / difval) : (40 * (g - b) / difval + 240);
        }
        else if(maxval == g)
        {
            h = 40 * (b - r) / difval + 80;
        }
        else
        {
            h = 40 * (r - g) / difval + 160;
        }

        if(l <= 120)
        {
            s = (maxval + minval == 0) ? 0 : (difval * 240 / (maxval + minval));
        }
        else
        {
            s = (480 - (maxval + minval) == 0) ? 0 : (difval * 240 / (480 - (maxval + minval)));
        }
    }

    if(h < 0) h = 0;
    if(h > 240) h = 240;
    if(s < 0) s = 0;
    if(s > 240) s = 240;
    if(l < 0) l = 0;
    if(l > 240) l = 240;

    hsl->hue = (unsigned char)h;
    hsl->saturation = (unsigned char)s;
    hsl->luminance = (unsigned char)l;
}

static int hue_in_range(uint8 hue, const target_condi_struct *condition)
{
    if(condition->hue_wrap)
    {
        return (hue >= condition->h_min) || (hue <= condition->h_max);
    }
    return (hue >= condition->h_min) && (hue <= condition->h_max);
}

static int hue_delta(uint8 hue, uint8 ref)
{
    int delta = (int)hue - (int)ref;

    while(delta > 120)
    {
        delta -= 240;
    }
    while(delta < -120)
    {
        delta += 240;
    }

    return delta;
}

static uint8 wrap_hue(int hue)
{
    while(hue < 0)
    {
        hue += 240;
    }
    while(hue > 240)
    {
        hue -= 240;
    }
    return (uint8)hue;
}

static int pixel_match_hsl(const color_hsl_struct *hsl, const target_condi_struct *condition)
{
    if(!condition->valid)
    {
        return 0;
    }

    if(!hue_in_range(hsl->hue, condition))
    {
        return 0;
    }
    if(hsl->saturation < condition->s_min || hsl->saturation > condition->s_max)
    {
        return 0;
    }
    if(hsl->luminance < condition->l_min || hsl->luminance > condition->l_max)
    {
        return 0;
    }

    return 1;
}

static int pixel_match_xy(unsigned int x, unsigned int y, const target_condi_struct *condition)
{
    color_rgb_struct rgb;
    color_hsl_struct hsl;

    readcolor(x, y, &rgb);
    rgbtohsl(&rgb, &hsl);
    return pixel_match_hsl(&hsl, condition);
}

static int cell_match(unsigned int grid_x, unsigned int grid_y, const target_condi_struct *condition)
{
    unsigned int x0 = grid_x * COLOR_GRID_STRIDE;
    unsigned int y0 = grid_y * COLOR_GRID_STRIDE;
    unsigned int x1 = umin(x0 + COLOR_GRID_STRIDE / 2u, SCC8660_W - 1u);
    unsigned int y1 = umin(y0 + COLOR_GRID_STRIDE / 2u, SCC8660_H - 1u);
    unsigned int x2 = umin(x0 + COLOR_GRID_STRIDE - 1u, SCC8660_W - 1u);
    unsigned int y2 = umin(y0 + COLOR_GRID_STRIDE - 1u, SCC8660_H - 1u);
    unsigned int match_count = 0u;

    if(pixel_match_xy(x0, y0, condition)) match_count++;
    if(pixel_match_xy(x1, y0, condition)) match_count++;
    if(pixel_match_xy(x0, y1, condition)) match_count++;
    if(pixel_match_xy(x1, y1, condition)) match_count++;
    if(pixel_match_xy(x2, y2, condition)) match_count++;

    return (match_count >= COLOR_CELL_MATCH_COUNT);
}

static void build_grid_mask(const target_condi_struct *condition)
{
    unsigned int y;
    unsigned int x;

    for(y = 0; y < COLOR_GRID_H; y++)
    {
        for(x = 0; x < COLOR_GRID_W; x++)
        {
            s_grid_visit[y][x] = 0u;
            s_grid_mask[y][x] = (uint8)cell_match(x, y, condition);
        }
    }
}

static unsigned int component_score(const component_info_t *info, const target_condi_struct *condition)
{
    unsigned int bbox_w_cells = (unsigned int)(info->max_x - info->min_x + 1u);
    unsigned int bbox_h_cells = (unsigned int)(info->max_y - info->min_y + 1u);
    unsigned int bbox_w = bbox_w_cells * COLOR_GRID_STRIDE;
    unsigned int bbox_h = bbox_h_cells * COLOR_GRID_STRIDE;
    unsigned int bbox_area = bbox_w_cells * bbox_h_cells;
    unsigned int fill_ratio;
    unsigned int aspect_x100;
    unsigned int center_x;
    unsigned int center_y;
    unsigned int score;
    int dist_x;
    int dist_y;

    if(info->cells < 2u || bbox_area == 0u || bbox_h == 0u)
    {
        return 0u;
    }

    fill_ratio = (unsigned int)(info->cells * 100u / bbox_area);
    aspect_x100 = (unsigned int)(bbox_w * 100u / bbox_h);

    if(bbox_w < condition->width_min / 2u || bbox_h < condition->hight_min / 2u)
    {
        return 0u;
    }
    if(fill_ratio + 8u < condition->fill_min_x100)
    {
        return 0u;
    }
    if(aspect_x100 + 20u < condition->aspect_min_x100 || aspect_x100 > condition->aspect_max_x100 + 20u)
    {
        return 0u;
    }

    center_x = (info->min_x + info->max_x + 1u) * COLOR_GRID_STRIDE / 2u;
    center_y = (info->min_y + info->max_y + 1u) * COLOR_GRID_STRIDE / 2u;
    score = info->cells * 80u + fill_ratio * 12u;

    if(s_last_result_valid)
    {
        dist_x = absi((int)center_x - (int)s_last_result.x);
        dist_y = absi((int)center_y - (int)s_last_result.y);
        score += (dist_x < (int)SCC8660_W && dist_y < (int)SCC8660_H) ? sat_sub(400u, (unsigned int)(dist_x + dist_y)) : 0u;
    }
    else
    {
        dist_x = absi((int)center_x - (int)(SCC8660_W / 2u));
        dist_y = absi((int)center_y - (int)(SCC8660_H / 2u));
        score += sat_sub(260u, (unsigned int)(dist_x + dist_y));
    }

    return score;
}

static int find_best_component(const target_condi_struct *condition, component_info_t *best_info)
{
    unsigned int y;
    unsigned int x;
    unsigned int best_score = 0u;

    for(y = 0; y < COLOR_GRID_H; y++)
    {
        for(x = 0; x < COLOR_GRID_W; x++)
        {
            component_info_t info;
            unsigned int head = 0u;
            unsigned int tail = 0u;
            unsigned int score;

            if(!s_grid_mask[y][x] || s_grid_visit[y][x])
            {
                continue;
            }

            info.min_x = (uint16)x;
            info.max_x = (uint16)x;
            info.min_y = (uint16)y;
            info.max_y = (uint16)y;
            info.cells = 0u;

            s_grid_visit[y][x] = 1u;
            s_queue[tail++] = (uint16)(y * COLOR_GRID_W + x);

            while(head < tail)
            {
                uint16 index = s_queue[head++];
                uint16 cy = (uint16)(index / COLOR_GRID_W);
                uint16 cx = (uint16)(index % COLOR_GRID_W);
                int ny;
                int nx;

                info.cells++;
                if(cx < info.min_x) info.min_x = cx;
                if(cx > info.max_x) info.max_x = cx;
                if(cy < info.min_y) info.min_y = cy;
                if(cy > info.max_y) info.max_y = cy;

                for(ny = (int)cy - 1; ny <= (int)cy + 1; ny++)
                {
                    for(nx = (int)cx - 1; nx <= (int)cx + 1; nx++)
                    {
                        if(nx < 0 || ny < 0 || nx >= (int)COLOR_GRID_W || ny >= (int)COLOR_GRID_H)
                        {
                            continue;
                        }
                        if(!s_grid_mask[ny][nx] || s_grid_visit[ny][nx])
                        {
                            continue;
                        }

                        s_grid_visit[ny][nx] = 1u;
                        s_queue[tail++] = (uint16)(ny * COLOR_GRID_W + nx);
                    }
                }
            }

            score = component_score(&info, condition);
            if(score > best_score)
            {
                best_score = score;
                *best_info = info;
            }
        }
    }

    return (best_score > 0u);
}

static int refine_component_bbox(const component_info_t *info, const target_condi_struct *condition, result_struct *resu)
{
    unsigned int x_start = (info->min_x > 0u) ? ((unsigned int)info->min_x * COLOR_GRID_STRIDE - COLOR_GRID_STRIDE) : 0u;
    unsigned int y_start = (info->min_y > 0u) ? ((unsigned int)info->min_y * COLOR_GRID_STRIDE - COLOR_GRID_STRIDE) : 0u;
    unsigned int x_end = umin(((unsigned int)info->max_x + 1u) * COLOR_GRID_STRIDE + COLOR_GRID_STRIDE, SCC8660_W) - 1u;
    unsigned int y_end = umin(((unsigned int)info->max_y + 1u) * COLOR_GRID_STRIDE + COLOR_GRID_STRIDE, SCC8660_H) - 1u;
    unsigned int min_x = SCC8660_W - 1u;
    unsigned int min_y = SCC8660_H - 1u;
    unsigned int max_x = 0u;
    unsigned int max_y = 0u;
    unsigned int match_count = 0u;
    unsigned int sum_x = 0u;
    unsigned int sum_y = 0u;
    unsigned int x;
    unsigned int y;
    unsigned int width;
    unsigned int height;
    unsigned int bbox_area;
    unsigned int fill_ratio;
    unsigned int aspect_x100;

    for(y = y_start; y <= y_end; y++)
    {
        for(x = x_start; x <= x_end; x++)
        {
            if(pixel_match_xy(x, y, condition))
            {
                if(x < min_x) min_x = x;
                if(x > max_x) max_x = x;
                if(y < min_y) min_y = y;
                if(y > max_y) max_y = y;
                match_count++;
                sum_x += x;
                sum_y += y;
            }
        }
    }

    if(match_count == 0u)
    {
        return 0;
    }

    width = max_x - min_x + 1u;
    height = max_y - min_y + 1u;
    bbox_area = width * height;
    fill_ratio = (bbox_area == 0u) ? 0u : (match_count * 100u / bbox_area);
    aspect_x100 = (height == 0u) ? 0u : (width * 100u / height);

    if(width < condition->width_min || height < condition->hight_min)
    {
        return 0;
    }
    if(width > condition->width_max || height > condition->hight_max)
    {
        return 0;
    }
    if(match_count < condition->area_min || match_count > condition->area_max)
    {
        return 0;
    }
    if(fill_ratio < condition->fill_min_x100)
    {
        return 0;
    }
    if(aspect_x100 < condition->aspect_min_x100 || aspect_x100 > condition->aspect_max_x100)
    {
        return 0;
    }

    resu->x = sum_x / match_count;
    resu->y = sum_y / match_count;
    resu->w = width;
    resu->h = height;
    resu->pixels = match_count;
    resu->score = match_count + fill_ratio * 8u;

    return 1;
}

static void smooth_result(result_struct *resu)
{
    if(!s_last_result_valid)
    {
        s_last_result = *resu;
        s_last_result_valid = 1u;
        s_lost_frames = 0u;
        return;
    }

    if(absi((int)resu->x - (int)s_last_result.x) <= (int)(resu->w + COLOR_GRID_STRIDE) &&
       absi((int)resu->y - (int)s_last_result.y) <= (int)(resu->h + COLOR_GRID_STRIDE))
    {
        resu->x = (resu->x * 70u + s_last_result.x * 30u) / 100u;
        resu->y = (resu->y * 70u + s_last_result.y * 30u) / 100u;
        resu->w = (resu->w * 65u + s_last_result.w * 35u) / 100u;
        resu->h = (resu->h * 65u + s_last_result.h * 35u) / 100u;
    }

    s_last_result = *resu;
    s_last_result_valid = 1u;
    s_lost_frames = 0u;
}

void set_color_target_condi(uint16 rgb565_data, target_condi_struct *condition)
{
    int center_x = (int)(SCC8660_W / 2u);
    int center_y = (int)(SCC8660_H / 2u);
    int sx;
    int sy;
    color_rgb_struct rgb;
    color_hsl_struct hsl;
    uint8 ref_hue = 0u;
    uint8 ref_found = 0u;
    int min_delta = 0;
    int max_delta = 0;
    uint8 min_s = 240u;
    uint8 max_s = 0u;
    uint8 min_l = 240u;
    uint8 max_l = 0u;
    unsigned int valid_samples = 0u;
    unsigned int hue_margin;
    unsigned int sat_margin;
    unsigned int lum_margin;

    (void)rgb565_data;

    readcolor((unsigned int)center_x, (unsigned int)center_y, &rgb);
    rgbtohsl(&rgb, &hsl);
    if(hsl.saturation >= 20u)
    {
        ref_hue = hsl.hue;
        ref_found = 1u;
    }

    for(sy = center_y - COLOR_SAMPLE_RADIUS; sy <= center_y + COLOR_SAMPLE_RADIUS; sy++)
    {
        for(sx = center_x - COLOR_SAMPLE_RADIUS; sx <= center_x + COLOR_SAMPLE_RADIUS; sx++)
        {
            if(sx < 0 || sx >= (int)SCC8660_W || sy < 0 || sy >= (int)SCC8660_H)
            {
                continue;
            }

            readcolor((unsigned int)sx, (unsigned int)sy, &rgb);
            rgbtohsl(&rgb, &hsl);

            if(hsl.saturation < 20u)
            {
                continue;
            }

            if(!ref_found)
            {
                ref_hue = hsl.hue;
                ref_found = 1u;
            }

            if(valid_samples == 0u)
            {
                min_delta = hue_delta(hsl.hue, ref_hue);
                max_delta = min_delta;
            }
            else
            {
                int delta = hue_delta(hsl.hue, ref_hue);
                if(delta < min_delta) min_delta = delta;
                if(delta > max_delta) max_delta = delta;
            }

            if(hsl.saturation < min_s) min_s = hsl.saturation;
            if(hsl.saturation > max_s) max_s = hsl.saturation;
            if(hsl.luminance < min_l) min_l = hsl.luminance;
            if(hsl.luminance > max_l) max_l = hsl.luminance;
            valid_samples++;
        }
    }

    if(!ref_found || valid_samples == 0u)
    {
        memset(condition, 0, sizeof(*condition));
        return;
    }

    hue_margin = clampu((unsigned int)(absi(max_delta - min_delta) / 2 + 8), 8u, 28u);
    sat_margin = clampu((unsigned int)(max_s - min_s) / 2u + 18u, 18u, 52u);
    lum_margin = clampu((unsigned int)(max_l - min_l) / 2u + 18u, 18u, 54u);

    min_delta -= (int)hue_margin;
    max_delta += (int)hue_margin;
    if(min_delta < -120) min_delta = -120;
    if(max_delta > 120) max_delta = 120;

    condition->hue_ref = ref_hue;
    condition->h_min = wrap_hue((int)ref_hue + min_delta);
    condition->h_max = wrap_hue((int)ref_hue + max_delta);
    condition->hue_wrap = (condition->h_min > condition->h_max) ? 1u : 0u;
    condition->s_min = (uint8)((min_s > sat_margin) ? (min_s - sat_margin) : 0u);
    condition->s_max = (uint8)umin(240u, max_s + sat_margin);
    condition->l_min = (uint8)((min_l > lum_margin) ? (min_l - lum_margin) : 0u);
    condition->l_max = (uint8)umin(240u, max_l + lum_margin);
    condition->width_min = COLOR_DEFAULT_WIDTH_MIN;
    condition->hight_min = COLOR_DEFAULT_HEIGHT_MIN;
    condition->width_max = SCC8660_W;
    condition->hight_max = SCC8660_H;
    condition->area_min = COLOR_DEFAULT_AREA_MIN;
    condition->area_max = SCC8660_W * SCC8660_H * 9u / 10u;
    condition->aspect_min_x100 = COLOR_DEFAULT_ASPECT_MIN;
    condition->aspect_max_x100 = COLOR_DEFAULT_ASPECT_MAX;
    condition->fill_min_x100 = COLOR_DEFAULT_FILL_MIN;
    condition->valid = 1u;
}

int color_trace(const target_condi_struct *condition, result_struct *resu)
{
    component_info_t best_info;
    result_struct result;

    if(!condition->valid)
    {
        return 0;
    }

    build_grid_mask(condition);
    if(find_best_component(condition, &best_info) && refine_component_bbox(&best_info, condition, &result))
    {
        smooth_result(&result);
        *resu = result;
        target_pos_out = result;
        return 1;
    }

    if(s_last_result_valid && s_lost_frames < COLOR_TRACK_KEEP_FRAMES)
    {
        s_lost_frames++;
        *resu = s_last_result;
        target_pos_out = s_last_result;
        return 1;
    }

    s_last_result_valid = 0u;
    s_lost_frames = 0u;
    memset(resu, 0, sizeof(*resu));
    memset(&target_pos_out, 0, sizeof(target_pos_out));
    return 0;
}

void color_trace_reset(void)
{
    memset(&target_pos_out, 0, sizeof(target_pos_out));
    memset(&s_last_result, 0, sizeof(s_last_result));
    s_last_result_valid = 0u;
    s_lost_frames = 0u;
}

uint8 color_trace_is_ready(const target_condi_struct *condition)
{
    return condition->valid;
}
