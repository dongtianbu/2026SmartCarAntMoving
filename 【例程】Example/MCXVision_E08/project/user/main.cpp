/*********************************************************************************************************************
* MCX Vision - 深蓝色沙包检测 (基于E08 - 最终正确版)
********************************************************************************************************************/
#include "zf_model_process.h"
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */ 
#include "zf_common_headfile.h"

// ============================================
// 我们自己的串口数据结构体（全新名称，不与E08冲突）
// ============================================
typedef struct{
    uint16 x1;
    uint16 y1;
    uint16 x2;
    uint16 y2;
}my_uart_data_t;

// 我们自己的全局变量
my_uart_data_t my_uart_data = {0};

// ============================================
// 简化的颜色检测
// ============================================
#define IMG_W   160
#define IMG_H   120

typedef struct{ unsigned char red, green, blue; }color_rgb_struct;

static int simple_color_detect(void)
{
    uint16_t center_pixel = *((uint16_t*)g_camera_buffer + (IMG_H/2) * IMG_W + IMG_W/2);
    
    color_rgb_struct rgb;
    rgb.red   = ((((uint16_t)center_pixel << 8)&0xFF00) | ((uint16_t)center_pixel >> 8)) & 0xf800 >> 8;
    rgb.green = ((((uint16_t)center_pixel << 8)&0xFF00) | ((uint16_t)center_pixel >> 8)) & 0x07e0 >> 3;
    rgb.blue  = ((((uint16_t)center_pixel << 8)&0xFF00) | ((uint16_t)center_pixel >> 8)) & 0x001f << 3;
    
    if(rgb.blue > 150 && rgb.green < 180 && rgb.red < 100)
    {
        return 1;
    }
    
    return 0;
}

int main(void)
{
    zf_board_init();
    user_uart_init();
    
    system_delay_ms(300);
    
    zf_debug_printf("debug_uart_init_finish\r\n");
    zf_user_printf("user_uart_init_finish\r\n");
    zf_user_printf("MCXVision Final Version Start\r\n");
    
    ips200_init();
    scc8660_init();
    zf_model_init();
    
    while (1)
    {
        if(scc8660_finish)
        {
            scc8660_finish = 0;
            
            // E08的关键：调用zf_model_run()
            zf_model_run();
            
            // 额外：我们的颜色检测
            if(simple_color_detect())
            {
                // 使用我们自己的变量
                my_uart_data.x1 = (uint16_t)(80 - 15);
                my_uart_data.y1 = (uint16_t)(60 - 15);
                my_uart_data.x2 = (uint16_t)(80 + 15);
                my_uart_data.y2 = (uint16_t)(60 + 15);
                
                user_uart_putchar(0xAA);
                user_uart_write_buffer((const uint8_t*)&my_uart_data, sizeof(my_uart_data));
                user_uart_putchar(0xFF);
            }
        }
    }
}
#if defined(__cplusplus)
}
#endif /* __cplusplus */
