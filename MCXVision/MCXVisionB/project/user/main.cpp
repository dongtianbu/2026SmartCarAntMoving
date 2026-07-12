#if defined(__cplusplus)
extern "C" {
#endif

#include "zf_common_headfile.h"
#include "color_tracer.h"

// ==================== 可调试参数区 ====================
// 串口发送频率，单位 Hz。
#define UART_SEND_FREQ                   30u
// 视觉主循环目标帧率，单位 Hz。
#define CAMERA_FPS                       30u
// 每隔多少帧发送一次串口数据。
#define SEND_INTERVAL                    ((CAMERA_FPS + UART_SEND_FREQ - 1u) / UART_SEND_FREQ)
// 屏幕中心绿色十字的半边长度，单位像素。
#define CENTER_MARK_HALF_SIZE            6
// 检测框颜色。
#define DETECT_BOX_COLOR                 RGB565_WHITE
// ==================== 可调试参数区 ====================

#if (SCC8660_W > 160)
#define UART_VIEW_W                      160u
#define UART_VIEW_H                      120u
#else
#define UART_VIEW_W                      SCC8660_W
#define UART_VIEW_H                      SCC8660_H
#endif

typedef struct __attribute__((packed))
{
    uint8 idx;
    uint16 x1;
    uint16 y1;
    uint16 x2;
    uint16 y2;
} uart_data_t;

static uart_data_t uart_data = {255u, 0u, 0u, (uint16)(UART_VIEW_W - 1u), (uint16)(UART_VIEW_H - 1u)};
static uint32_t send_counter = 0u;

static uint16_t scale_to_uart_x(unsigned int x)
{
    return (uint16_t)((uint32_t)x * (UART_VIEW_W - 1u) / (SCC8660_W - 1u));
}

static uint16_t scale_to_uart_y(unsigned int y)
{
    return (uint16_t)((uint32_t)y * (UART_VIEW_H - 1u) / (SCC8660_H - 1u));
}

static void draw_center_marker(void)
{
    int center_x = (int)(SCC8660_W / 2u);
    int center_y = (int)(SCC8660_H / 2u);

    ips200_draw_line(center_x - CENTER_MARK_HALF_SIZE, center_y, center_x + CENTER_MARK_HALF_SIZE, center_y, RGB565_GREEN);
    ips200_draw_line(center_x, center_y - CENTER_MARK_HALF_SIZE, center_x, center_y + CENTER_MARK_HALF_SIZE, RGB565_GREEN);
}

static void draw_result_box(const light_point_result_struct *result)
{
    int x1;
    int y1;
    int x2;
    int y2;

    if(!result->valid)
    {
        return;
    }

    x1 = (int)result->x - (int)result->w / 2;
    y1 = (int)result->y - (int)result->h / 2;
    x2 = x1 + (int)result->w - 1;
    y2 = y1 + (int)result->h - 1;

    if(x1 < 0) x1 = 0;
    if(y1 < 0) y1 = 0;
    if(x2 >= (int)SCC8660_W) x2 = (int)SCC8660_W - 1;
    if(y2 >= (int)SCC8660_H) y2 = (int)SCC8660_H - 1;

    ips200_draw_line(x1, y1, x2, y1, DETECT_BOX_COLOR);
    ips200_draw_line(x1, y1, x1, y2, DETECT_BOX_COLOR);
    ips200_draw_line(x1, y2, x2, y2, DETECT_BOX_COLOR);
    ips200_draw_line(x2, y1, x2, y2, DETECT_BOX_COLOR);
}

static void fill_uart_data_from_result(const light_point_result_struct *result)
{
    int x1;
    int y1;
    int x2;
    int y2;

    if(!result->valid)
    {
        uart_data.idx = 255u;
        uart_data.x1 = 0u;
        uart_data.y1 = 0u;
        uart_data.x2 = (uint16_t)(UART_VIEW_W - 1u);
        uart_data.y2 = (uint16_t)(UART_VIEW_H - 1u);
        return;
    }

    x1 = (int)result->x - (int)result->w / 2;
    y1 = (int)result->y - (int)result->h / 2;
    x2 = x1 + (int)result->w - 1;
    y2 = y1 + (int)result->h - 1;

    if(x1 < 0) x1 = 0;
    if(y1 < 0) y1 = 0;
    if(x2 >= (int)SCC8660_W) x2 = (int)SCC8660_W - 1;
    if(y2 >= (int)SCC8660_H) y2 = (int)SCC8660_H - 1;

    // 串口继续发送一个框，但这个框的中心点已经对齐到两个红外光点中心坐标的中值。
    // 因此下位机继续用框中心闭环时，控制目标就是“两点中值位于屏幕中心”。
    uart_data.idx = 0u;
    uart_data.x1 = scale_to_uart_x((unsigned int)x1);
    uart_data.y1 = scale_to_uart_y((unsigned int)y1);
    uart_data.x2 = scale_to_uart_x((unsigned int)x2);
    uart_data.y2 = scale_to_uart_y((unsigned int)y2);
}

int main(void)
{
    zf_board_init();
    user_uart_init();
    system_delay_ms(300);

    zf_debug_printf("debug_uart_init_finish\r\n");
    zf_user_printf("user_uart_init_finish\r\n");
    zf_user_printf("========================================\r\n");
    zf_user_printf("MCXVision IR Double Point Detector\r\n");
    zf_user_printf("Detect two bright points on dark background\r\n");
    zf_user_printf("UART Box Scale=%ux%u, Camera=%ux%u, Send=%uHz\r\n", UART_VIEW_W, UART_VIEW_H, SCC8660_W, SCC8660_H, UART_SEND_FREQ);
    zf_user_printf("========================================\r\n");

    color_trace_reset();
    ips200_init();
    scc8660_init();
    fill_uart_data_from_result(&light_trace_out.merged);

    while(1)
    {
        if(scc8660_finish)
        {
            light_trace_result_struct trace_result;
            int detected_count;
            unsigned int i;

            scc8660_finish = 0;
            ips200_show_scc8660((uint16_t *)g_camera_buffer);
            draw_center_marker();

            detected_count = color_trace(&trace_result);
            if(detected_count > 0)
            {
                for(i = 0u; i < trace_result.count; i++)
                {
                    draw_result_box(&trace_result.points[i]);
                }
            }

            fill_uart_data_from_result(&trace_result.merged);

            send_counter++;
            if(send_counter >= SEND_INTERVAL)
            {
                send_counter = 0u;
                user_uart_putchar(0xAA);
                user_uart_write_buffer((const uint8_t *)&uart_data, 9u);
                user_uart_putchar(0xFF);
            }
        }
    }
}

#if defined(__cplusplus)
}
#endif
