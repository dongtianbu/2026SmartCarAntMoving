#include "color_tracer.h"

#define SWAPBYTE(h)                         ((((uint16_t)(h) << 8) & 0xFF00u) | ((uint16_t)(h) >> 8))

// ==================== 可调试参数区 ====================
// 单个像素被视为亮点的最小强度阈值，值越大越能压制弱杂点。
#define LIGHT_PIXEL_THRESHOLD              168u
// 一个网格内至少有多少个亮像素，才认为该网格有效。
#define LIGHT_CELL_PIXEL_MIN               1u
// 对候选连通域向外扩展的像素数，避免细小亮点被截断。
#define LIGHT_COMPONENT_EXPAND_PIXELS      2u

// 单个亮点的最小宽度，单位像素。
#define LIGHT_POINT_MIN_WIDTH              2u
// 单个亮点的最小高度，单位像素。
#define LIGHT_POINT_MIN_HEIGHT             2u
// 单个亮点的最大宽度，单位像素。
#define LIGHT_POINT_MAX_WIDTH              24u
// 单个亮点的最大高度，单位像素。
#define LIGHT_POINT_MAX_HEIGHT             24u
// 单个亮点的最小亮像素数。
#define LIGHT_POINT_MIN_PIXELS             2u
// 单个亮点的最大亮像素数。
#define LIGHT_POINT_MAX_PIXELS             180u
// 亮点框内部填充率下限，按 100 倍存储。
#define LIGHT_POINT_MIN_FILL_X100          8u
// 亮点宽高比下限，按 100 倍存储。
#define LIGHT_POINT_MIN_ASPECT_X100        25u
// 亮点宽高比上限，按 100 倍存储。
#define LIGHT_POINT_MAX_ASPECT_X100        400u
// 单个亮点峰值强度下限，避免把很暗的杂点也选进来。
#define LIGHT_POINT_MIN_PEAK_VALUE         180u
// 单个亮点总强度下限，避免只有一两个偶然噪点通过。
#define LIGHT_POINT_MIN_SUM_VALUE          360u

// 峰值强度在评分中的权重。
#define LIGHT_SCORE_PEAK_WEIGHT            10u
// 亮像素数量在评分中的权重。
#define LIGHT_SCORE_PIXEL_WEIGHT           6u
// 总强度在评分中的缩放因子，值越小则总强度影响越大。
#define LIGHT_SCORE_SUM_DIV                4u

// 当前帧未检出时，保留上一帧结果的帧数。
#define LIGHT_TRACK_KEEP_FRAMES            1u
// ==================== 可调试参数区 ====================

typedef struct
{
    uint16                  min_x;
    uint16                  max_x;
    uint16                  min_y;
    uint16                  max_y;
    uint16                  cells;
    uint8                   peak_value;
} component_info_t;

typedef struct
{
    uint8                   red;
    uint8                   green;
    uint8                   blue;
} color_rgb_struct;

light_trace_result_struct light_trace_out = {0};

static uint8 s_grid_mask[LIGHT_GRID_H][LIGHT_GRID_W] = {{0}};
static uint8 s_grid_visit[LIGHT_GRID_H][LIGHT_GRID_W] = {{0}};
static uint8 s_grid_peak[LIGHT_GRID_H][LIGHT_GRID_W] = {{0}};
static uint16 s_queue[LIGHT_GRID_W * LIGHT_GRID_H] = {0};
static light_trace_result_struct s_last_trace_out = {0};
static uint8 s_last_trace_valid = 0u;
static uint8 s_lost_frames = 0u;

static unsigned int umin(unsigned int a, unsigned int b)
{
    return (a < b) ? a : b;
}

static int absi(int value)
{
    return (value < 0) ? -value : value;
}

static void readcolor(unsigned int x, unsigned int y, color_rgb_struct *rgb)
{
    uint16 c16;

    c16 = SWAPBYTE(*((uint16 *)g_camera_buffer + y * SCC8660_W + x));
    rgb->red   = (uint8)((c16 & 0xF800u) >> 8);
    rgb->green = (uint8)((c16 & 0x07E0u) >> 3);
    rgb->blue  = (uint8)((c16 & 0x001Fu) << 3);
}

static uint8 pixel_value_xy(unsigned int x, unsigned int y)
{
    color_rgb_struct rgb;
    uint8 peak_value;

    readcolor(x, y, &rgb);
    peak_value = rgb.red;
    if(rgb.green > peak_value) peak_value = rgb.green;
    if(rgb.blue > peak_value) peak_value = rgb.blue;
    return peak_value;
}

static void build_grid_mask(void)
{
    unsigned int grid_y;
    unsigned int grid_x;

    for(grid_y = 0; grid_y < LIGHT_GRID_H; grid_y++)
    {
        for(grid_x = 0; grid_x < LIGHT_GRID_W; grid_x++)
        {
            unsigned int x_start = grid_x * LIGHT_GRID_STRIDE;
            unsigned int y_start = grid_y * LIGHT_GRID_STRIDE;
            unsigned int x_end = umin(x_start + LIGHT_GRID_STRIDE, SCC8660_W);
            unsigned int y_end = umin(y_start + LIGHT_GRID_STRIDE, SCC8660_H);
            unsigned int x;
            unsigned int y;
            unsigned int bright_pixels = 0u;
            uint8 peak_value = 0u;

            for(y = y_start; y < y_end; y++)
            {
                for(x = x_start; x < x_end; x++)
                {
                    uint8 value = pixel_value_xy(x, y);

                    if(value > peak_value)
                    {
                        peak_value = value;
                    }
                    if(value >= LIGHT_PIXEL_THRESHOLD)
                    {
                        bright_pixels++;
                    }
                }
            }

            s_grid_visit[grid_y][grid_x] = 0u;
            s_grid_peak[grid_y][grid_x] = peak_value;
            s_grid_mask[grid_y][grid_x] = (bright_pixels >= LIGHT_CELL_PIXEL_MIN) ? 1u : 0u;
        }
    }
}

static unsigned int component_score(const component_info_t *info)
{
    unsigned int bbox_w_cells = (unsigned int)(info->max_x - info->min_x + 1u);
    unsigned int bbox_h_cells = (unsigned int)(info->max_y - info->min_y + 1u);
    unsigned int area_cells = bbox_w_cells * bbox_h_cells;

    if(info->cells == 0u || area_cells == 0u)
    {
        return 0u;
    }

    return info->cells * LIGHT_SCORE_PIXEL_WEIGHT + (unsigned int)info->peak_value * LIGHT_SCORE_PEAK_WEIGHT;
}

static int refine_component_bbox(const component_info_t *info, light_point_result_struct *point)
{
    unsigned int x_start;
    unsigned int y_start;
    unsigned int x_end;
    unsigned int y_end;
    unsigned int min_x = SCC8660_W - 1u;
    unsigned int min_y = SCC8660_H - 1u;
    unsigned int max_x = 0u;
    unsigned int max_y = 0u;
    unsigned int sum_x = 0u;
    unsigned int sum_y = 0u;
    unsigned int sum_value = 0u;
    unsigned int pixels = 0u;
    unsigned int width;
    unsigned int height;
    unsigned int bbox_area;
    unsigned int fill_x100;
    unsigned int aspect_x100;
    unsigned int x;
    unsigned int y;
    uint8 peak_value = 0u;

    x_start = (info->min_x > 0u) ? (unsigned int)info->min_x * LIGHT_GRID_STRIDE : 0u;
    y_start = (info->min_y > 0u) ? (unsigned int)info->min_y * LIGHT_GRID_STRIDE : 0u;
    x_start = (x_start > LIGHT_COMPONENT_EXPAND_PIXELS) ? (x_start - LIGHT_COMPONENT_EXPAND_PIXELS) : 0u;
    y_start = (y_start > LIGHT_COMPONENT_EXPAND_PIXELS) ? (y_start - LIGHT_COMPONENT_EXPAND_PIXELS) : 0u;
    x_end = umin(((unsigned int)info->max_x + 1u) * LIGHT_GRID_STRIDE + LIGHT_COMPONENT_EXPAND_PIXELS, SCC8660_W);
    y_end = umin(((unsigned int)info->max_y + 1u) * LIGHT_GRID_STRIDE + LIGHT_COMPONENT_EXPAND_PIXELS, SCC8660_H);

    for(y = y_start; y < y_end; y++)
    {
        for(x = x_start; x < x_end; x++)
        {
            uint8 value = pixel_value_xy(x, y);

            if(value < LIGHT_PIXEL_THRESHOLD)
            {
                continue;
            }

            if(x < min_x) min_x = x;
            if(x > max_x) max_x = x;
            if(y < min_y) min_y = y;
            if(y > max_y) max_y = y;
            if(value > peak_value) peak_value = value;

            pixels++;
            sum_x += x;
            sum_y += y;
            sum_value += value;
        }
    }

    if(pixels == 0u)
    {
        return 0;
    }

    width = max_x - min_x + 1u;
    height = max_y - min_y + 1u;
    bbox_area = width * height;
    fill_x100 = (bbox_area == 0u) ? 0u : (pixels * 100u / bbox_area);
    aspect_x100 = (height == 0u) ? 0u : (width * 100u / height);

    if(width < LIGHT_POINT_MIN_WIDTH || width > LIGHT_POINT_MAX_WIDTH)
    {
        return 0;
    }
    if(height < LIGHT_POINT_MIN_HEIGHT || height > LIGHT_POINT_MAX_HEIGHT)
    {
        return 0;
    }
    if(pixels < LIGHT_POINT_MIN_PIXELS || pixels > LIGHT_POINT_MAX_PIXELS)
    {
        return 0;
    }
    if(fill_x100 < LIGHT_POINT_MIN_FILL_X100)
    {
        return 0;
    }
    if(aspect_x100 < LIGHT_POINT_MIN_ASPECT_X100 || aspect_x100 > LIGHT_POINT_MAX_ASPECT_X100)
    {
        return 0;
    }
    if(peak_value < LIGHT_POINT_MIN_PEAK_VALUE)
    {
        return 0;
    }
    if(sum_value < LIGHT_POINT_MIN_SUM_VALUE)
    {
        return 0;
    }

    memset(point, 0, sizeof(*point));
    point->valid = 1u;
    point->x = sum_x / pixels;
    point->y = sum_y / pixels;
    point->w = width;
    point->h = height;
    point->pixels = pixels;
    point->peak_value = peak_value;
    point->score = peak_value * LIGHT_SCORE_PEAK_WEIGHT + pixels * LIGHT_SCORE_PIXEL_WEIGHT + sum_value / LIGHT_SCORE_SUM_DIV;
    return 1;
}

static void insert_best_point(light_point_result_struct *dst, const light_point_result_struct *candidate)
{
    if(!candidate->valid)
    {
        return;
    }

    if(!dst[0].valid || candidate->score > dst[0].score)
    {
        dst[1] = dst[0];
        dst[0] = *candidate;
        return;
    }

    if(!dst[1].valid || candidate->score > dst[1].score)
    {
        dst[1] = *candidate;
    }
}

static void build_merged_result(light_trace_result_struct *result)
{
    unsigned int i;
    unsigned int min_x;
    unsigned int min_y;
    unsigned int max_x;
    unsigned int max_y;
    unsigned int sum_pixels = 0u;
    unsigned int peak_value = 0u;
    unsigned int score = 0u;
    unsigned int center_x;
    unsigned int center_y;

    if(result->count == 0u)
    {
        memset(&result->merged, 0, sizeof(result->merged));
        return;
    }

    min_x = result->points[0].x - result->points[0].w / 2u;
    min_y = result->points[0].y - result->points[0].h / 2u;
    max_x = min_x + result->points[0].w - 1u;
    max_y = min_y + result->points[0].h - 1u;

    for(i = 0; i < result->count; i++)
    {
        unsigned int x1 = result->points[i].x - result->points[i].w / 2u;
        unsigned int y1 = result->points[i].y - result->points[i].h / 2u;
        unsigned int x2 = x1 + result->points[i].w - 1u;
        unsigned int y2 = y1 + result->points[i].h - 1u;

        if(x1 < min_x) min_x = x1;
        if(y1 < min_y) min_y = y1;
        if(x2 > max_x) max_x = x2;
        if(y2 > max_y) max_y = y2;

        sum_pixels += result->points[i].pixels;
        if(result->points[i].peak_value > peak_value)
        {
            peak_value = result->points[i].peak_value;
        }
        score += result->points[i].score;
    }

    // 当前场景固定为两个红外光点，控制时需要使用两个亮点中心坐标的中值。
    // 检测到两个亮点时直接取两点中心坐标的平均值；若暂时只剩一个亮点，则退化为单点中心。
    if(result->count >= 2u)
    {
        center_x = (result->points[0].x + result->points[1].x) / 2u;
        center_y = (result->points[0].y + result->points[1].y) / 2u;
    }
    else
    {
        center_x = result->points[0].x;
        center_y = result->points[0].y;
    }

    memset(&result->merged, 0, sizeof(result->merged));
    result->merged.valid = 1u;
    result->merged.x = center_x;
    result->merged.y = center_y;
    result->merged.w = max_x - min_x + 1u;
    result->merged.h = max_y - min_y + 1u;
    result->merged.pixels = sum_pixels;
    result->merged.peak_value = peak_value;
    result->merged.score = score;
}

int color_trace(light_trace_result_struct *result)
{
    unsigned int grid_y;
    unsigned int grid_x;

    build_grid_mask();
    memset(result, 0, sizeof(*result));

    for(grid_y = 0; grid_y < LIGHT_GRID_H; grid_y++)
    {
        for(grid_x = 0; grid_x < LIGHT_GRID_W; grid_x++)
        {
            component_info_t info;
            unsigned int head = 0u;
            unsigned int tail = 0u;
            light_point_result_struct candidate;

            if(!s_grid_mask[grid_y][grid_x] || s_grid_visit[grid_y][grid_x])
            {
                continue;
            }

            memset(&info, 0, sizeof(info));
            info.min_x = (uint16)grid_x;
            info.max_x = (uint16)grid_x;
            info.min_y = (uint16)grid_y;
            info.max_y = (uint16)grid_y;
            info.peak_value = s_grid_peak[grid_y][grid_x];

            s_grid_visit[grid_y][grid_x] = 1u;
            s_queue[tail++] = (uint16)(grid_y * LIGHT_GRID_W + grid_x);

            while(head < tail)
            {
                uint16 index = s_queue[head++];
                uint16 cy = (uint16)(index / LIGHT_GRID_W);
                uint16 cx = (uint16)(index % LIGHT_GRID_W);
                int ny;
                int nx;

                info.cells++;
                if(cx < info.min_x) info.min_x = cx;
                if(cx > info.max_x) info.max_x = cx;
                if(cy < info.min_y) info.min_y = cy;
                if(cy > info.max_y) info.max_y = cy;
                if(s_grid_peak[cy][cx] > info.peak_value)
                {
                    info.peak_value = s_grid_peak[cy][cx];
                }

                for(ny = (int)cy - 1; ny <= (int)cy + 1; ny++)
                {
                    for(nx = (int)cx - 1; nx <= (int)cx + 1; nx++)
                    {
                        if(nx < 0 || ny < 0 || nx >= (int)LIGHT_GRID_W || ny >= (int)LIGHT_GRID_H)
                        {
                            continue;
                        }
                        if(!s_grid_mask[ny][nx] || s_grid_visit[ny][nx])
                        {
                            continue;
                        }

                        s_grid_visit[ny][nx] = 1u;
                        s_queue[tail++] = (uint16)(ny * LIGHT_GRID_W + nx);
                    }
                }
            }

            if(component_score(&info) == 0u)
            {
                continue;
            }

            memset(&candidate, 0, sizeof(candidate));
            if(refine_component_bbox(&info, &candidate))
            {
                insert_best_point(result->points, &candidate);
            }
        }
    }

    if(result->points[0].valid)
    {
        result->count = 1u;
    }
    if(result->points[1].valid)
    {
        result->count = 2u;
    }

    if(result->count > 0u)
    {
        build_merged_result(result);
        light_trace_out = *result;
        s_last_trace_out = *result;
        s_last_trace_valid = 1u;
        s_lost_frames = 0u;
        return (int)result->count;
    }

    if(s_last_trace_valid && s_lost_frames < LIGHT_TRACK_KEEP_FRAMES)
    {
        s_lost_frames++;
        *result = s_last_trace_out;
        light_trace_out = s_last_trace_out;
        return (int)result->count;
    }

    s_last_trace_valid = 0u;
    s_lost_frames = 0u;
    memset(&light_trace_out, 0, sizeof(light_trace_out));
    return 0;
}

void color_trace_reset(void)
{
    memset(&light_trace_out, 0, sizeof(light_trace_out));
    memset(&s_last_trace_out, 0, sizeof(s_last_trace_out));
    s_last_trace_valid = 0u;
    s_lost_frames = 0u;
}
