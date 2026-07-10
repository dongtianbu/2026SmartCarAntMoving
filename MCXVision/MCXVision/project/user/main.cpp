#if defined(__cplusplus)
extern "C" {
#endif
#include "zf_common_headfile.h"
#include "color_tracer.h"

#define UART_SEND_FREQ           30u
#define CAMERA_FPS               30u
#define SEND_INTERVAL            ((CAMERA_FPS + UART_SEND_FREQ - 1u) / UART_SEND_FREQ)
#define CALIB_MARK_HALF_SIZE     6
#define CALIB_FEEDBACK_FRAMES    12u

#if (SCC8660_W > 160)
#define UART_VIEW_W              160u
#define UART_VIEW_H              120u
#else
#define UART_VIEW_W              SCC8660_W
#define UART_VIEW_H              SCC8660_H
#endif

gpio_struct gpio_key_1 = {GPIO4, 2u};

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
static uint8_t key1_last_level = 1u;
static uint8_t calib_feedback_frames = 0u;

static uint8_t key_pressed_event(gpio_struct key, uint8_t *last_level)
{
    uint8_t current_level = (uint8_t)gpio_get_level(key);
    uint8_t pressed = 0u;

    if((*last_level != 0u) && (current_level == 0u))
    {
        pressed = 1u;
    }

    *last_level = current_level;
    return pressed;
}

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

    ips200_draw_line(center_x - CALIB_MARK_HALF_SIZE, center_y, center_x + CALIB_MARK_HALF_SIZE, center_y, RGB565_GREEN);
    ips200_draw_line(center_x, center_y - CALIB_MARK_HALF_SIZE, center_x, center_y + CALIB_MARK_HALF_SIZE, RGB565_GREEN);
}

static void draw_calibration_feedback(void)
{
    int center_x = (int)(SCC8660_W / 2u);
    int center_y = (int)(SCC8660_H / 2u);
    int half_size = CALIB_MARK_HALF_SIZE + 4;

    ips200_draw_line(center_x - half_size, center_y - half_size, center_x + half_size, center_y - half_size, RGB565_RED);
    ips200_draw_line(center_x - half_size, center_y - half_size, center_x - half_size, center_y + half_size, RGB565_RED);
    ips200_draw_line(center_x - half_size, center_y + half_size, center_x + half_size, center_y + half_size, RGB565_RED);
    ips200_draw_line(center_x + half_size, center_y - half_size, center_x + half_size, center_y + half_size, RGB565_RED);
}

static void update_uart_data_from_result(const result_struct *result)
{
    int x1 = (int)result->x - (int)result->w / 2;
    int y1 = (int)result->y - (int)result->h / 2;
    int x2 = x1 + (int)result->w - 1;
    int y2 = y1 + (int)result->h - 1;

    if(x1 < 0) x1 = 0;
    if(y1 < 0) y1 = 0;
    if(x2 >= (int)SCC8660_W) x2 = (int)SCC8660_W - 1;
    if(y2 >= (int)SCC8660_H) y2 = (int)SCC8660_H - 1;

    ips200_draw_line(x1, y1, x2, y1, RGB565_WHITE);
    ips200_draw_line(x1, y1, x1, y2, RGB565_WHITE);
    ips200_draw_line(x1, y2, x2, y2, RGB565_WHITE);
    ips200_draw_line(x2, y1, x2, y2, RGB565_WHITE);

    uart_data.idx = 0u;
    uart_data.x1 = scale_to_uart_x((unsigned int)x1);
    uart_data.y1 = scale_to_uart_y((unsigned int)y1);
    uart_data.x2 = scale_to_uart_x((unsigned int)x2);
    uart_data.y2 = scale_to_uart_y((unsigned int)y2);
}

static void set_no_target_uart_data(void)
{
    uart_data.idx = 255u;
    uart_data.x1 = 0u;
    uart_data.y1 = 0u;
    uart_data.x2 = (uint16_t)(UART_VIEW_W - 1u);
    uart_data.y2 = (uint16_t)(UART_VIEW_H - 1u);
}

int main(void)
{
    zf_board_init();
    user_uart_init();
    system_delay_ms(300);

    zf_debug_printf("debug_uart_init_finish\r\n");
    zf_user_printf("user_uart_init_finish\r\n");
    zf_user_printf("========================================\r\n");
    zf_user_printf("MCXVision Pure Color Sandbag Detector\r\n");
    zf_user_printf("KEY1=Calibrate Center Target\r\n");
    zf_user_printf("UART Box Scale=%ux%u, Camera=%ux%u, Send=%uHz\r\n", UART_VIEW_W, UART_VIEW_H, SCC8660_W, SCC8660_H, UART_SEND_FREQ);
    zf_user_printf("========================================\r\n");

    gpio_init(gpio_key_1, GPI, 0, PULL_UP);

    color_trace_reset();
    memset(&target_color_condi, 0, sizeof(target_color_condi));
    ips200_init();
    scc8660_init();
    set_no_target_uart_data();

    while(1)
    {
        if(scc8660_finish)
        {
            uint8_t key1_pressed;
            int detected = 0;

            scc8660_finish = 0;
            ips200_show_scc8660((uint16_t *)g_camera_buffer);
            draw_center_marker();

            key1_pressed = key_pressed_event(gpio_key_1, &key1_last_level);
            if(key1_pressed)
            {
                set_color_target_condi(
                    *((uint16 *)g_camera_buffer + SCC8660_H / 2u * SCC8660_W + SCC8660_W / 2u),
                    &target_color_condi
                );
                color_trace_reset();
                calib_feedback_frames = CALIB_FEEDBACK_FRAMES;

                zf_user_printf("[Calib] H:%d-%d wrap:%d S:%d-%d L:%d-%d\r\n",
                               target_color_condi.h_min, target_color_condi.h_max, target_color_condi.hue_wrap,
                               target_color_condi.s_min, target_color_condi.s_max,
                               target_color_condi.l_min, target_color_condi.l_max);
            }

            if(color_trace_is_ready(&target_color_condi))
            {
                detected = color_trace(&target_color_condi, &target_pos_out);
            }

            if(detected)
            {
                update_uart_data_from_result(&target_pos_out);
            }
            else
            {
                set_no_target_uart_data();
            }

            if(calib_feedback_frames > 0u)
            {
                draw_calibration_feedback();
                calib_feedback_frames--;
            }

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
