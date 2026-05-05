/*********************************************************************************************************************
* MCX Vision - 深蓝色沙包检测系统 V3.8 (基于E09色块追踪例程)
* 
* V3.8 改进:
* - 保持V3.6稳定的边界框格式 (确保通信正常)
* - 在上位机端转换为中心坐标+尺寸格式
********************************************************************************************************************/
#include "zf_model_process.h"
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */ 
#include "zf_common_headfile.h"
#include "color_tracer.h"

// ============================================
// 功能配置
// ============================================
#define ENABLE_SD_CARD    0

// 发送频率控制 (Hz)
#define UART_SEND_FREQ    10   // 串口发送频率 (Hz) - 每秒10次
#define CAMERA_FPS        30   // 摄像头帧率 (FPS) - 检测速度不变!
#define SEND_INTERVAL     (CAMERA_FPS / UART_SEND_FREQ)

// ============================================
// 引脚定义
// ============================================
gpio_struct gpio_key_1 = {GPIO4, 2u};      // KEY1: 颜色标定
gpio_struct gpio_key_2 = {GPIO4, 3u};      // KEY2: 补光灯开关
gpio_struct gpio_led_white = {GPIO2, 11u}; // 白色LED (补光灯)

// ============================================
// 串口通信协议 V3.8 (保持V3.6的稳定格式!)
// 
// 格式: [0xAA][idx(1B)][x1(2B)][y1(2B)][x2(2B)][y2(2B)][0xFF]
// 总长: 11字节 (与V3.6完全一致!)
//
// 上位机收到后会转换为: MCXVISION:center_x,center_y,width,height
// ============================================
typedef struct __attribute__((packed)){
    uint8  idx;       // 目标索引 (0=有目标, 255=无目标)
    uint16 x1;        // 左上角 X
    uint16 y1;        // 左上角 Y
    uint16 x2;        // 右下角 X
    uint16 y2;        // 右下角 Y
} uart_data_t;

uart_data_t uart_data = {0};

static uint8_t fill_light_on = 0;
static uint32_t frame_count = 0;
static uint32_t send_counter = 0;

#if ENABLE_SD_CARD
#include "zf_driver_sd.h"
extern sd_card_t g_sd;
bool sd_card_ready = false;
#endif

int main(void)
{
    zf_board_init();
    
    user_uart_init();
    system_delay_ms(300);
    
    zf_debug_printf("debug_uart_init_finish\r\n");
    zf_user_printf("user_uart_init_finish\r\n");
    zf_user_printf("========================================\r\n");
    zf_user_printf("MCXVision V3.8 Sandbag Detector\r\n");
    zf_user_printf("KEY1=Calibrate  KEY2=FillLight\r\n");
    zf_user_printf("========================================\r\n");
    
    // 按键初始化
    gpio_init(gpio_key_1, GPI, 0, PULL_UP);
    gpio_init(gpio_key_2, GPI, 0, PULL_UP);
    
    // LED初始化
    gpio_init(gpio_led_white, GPO, 1, PULL_UP);
    gpio_set_level(gpio_led_white, 1);  // 默认关闭
    
    ips200_init();
    scc8660_init();
    
    zf_user_printf("[Init] System ready!\r\n");
    
    while (1)
    {
        frame_count++;
        
        if(scc8660_finish)
        {
            scc8660_finish = 0;
            
            // ⭐ 检测和显示始终以30FPS运行
            ips200_show_scc8660((uint16_t*)g_camera_buffer);
            
            // ===== KEY1: 颜色标定 =====
            if(!gpio_get_level(gpio_key_1))
            {
                set_color_target_condi(
                    (*((uint16*)g_camera_buffer + SCC8660_H/2 * SCC8660_W + SCC8660_W/2)), 
                    &target_color_condi
                );
                
                system_delay_ms(200);
                
                zf_user_printf("[Calib] H:%d-%d S:%d-%d L:%d-%d\r\n",
                             target_color_condi.h_min, target_color_condi.h_max,
                             target_color_condi.s_min, target_color_condi.s_max,
                             target_color_condi.l_min, target_color_condi.l_max);
            }
            
            // ===== KEY2: 补光灯切换 =====
            if(!gpio_get_level(gpio_key_2))
            {
                fill_light_on = !fill_light_on;
                gpio_set_level(gpio_led_white, !fill_light_on);
                
                system_delay_ms(200);
                
                if(fill_light_on)
                    zf_user_printf("[Light] ON\r\n");
                else
                    zf_user_printf("[Light] OFF\r\n");
            }
            
            // ===== 颜色检测 (30FPS) =====
            int detected = color_trace(&target_color_condi, &target_pos_out);
            
            if(detected)
            {
                int x1 = target_pos_out.x - target_pos_out.w/2;
                int y1 = target_pos_out.y - target_pos_out.h/2;
                int x2 = target_pos_out.x + target_pos_out.w/2;
                int y2 = target_pos_out.y + target_pos_out.h/2;
                
                // 绘制检测框 (30FPS实时更新)
                ips200_draw_line(x1, y1, x2, y1, 0xffff);
                ips200_draw_line(x1, y1, x1, y2, 0xffff);
                ips200_draw_line(x1, y2, x2, y2, 0xffff);
                ips200_draw_line(x2, y1, x2, y2, 0xffff);
                
                uart_data.idx = 0;
                uart_data.x1 = (uint16_t)x1;
                uart_data.y1 = (uint16_t)y1;
                uart_data.x2 = (uint16_t)x2;
                uart_data.y2 = (uint16_t)y2;
            }
            else
            {
                uart_data.idx = 255;
                uart_data.x1 = 0;
                uart_data.y1 = 0;
                uart_data.x2 = 159;
                uart_data.y2 = 119;
            }
            
            // ⭐ 串口发送频率控制 (10Hz)
            send_counter++;
            if(send_counter >= SEND_INTERVAL)
            {
                send_counter = 0;
                
                user_uart_putchar(0xAA);                                    // 帧头
                user_uart_write_buffer((const uint8_t*)&uart_data, 9);       // 数据 (9字节)
                user_uart_putchar(0xFF);                                    // 帧尾
                
                // 调试信息
                if(frame_count % (UART_SEND_FREQ * 3) == 0)  // 每3秒打印一次
                {
                    if(detected)
                        zf_user_printf("[SEND] %d,%d,%d,%d\r\n",
                                      uart_data.x1, uart_data.y1,
                                      uart_data.x2, uart_data.y2);
                    else
                        zf_user_printf("[SEND] No target\r\n");
                }
            }
        }
    }
}

#if defined(__cplusplus)
}
#endif /* __cplusplus */
