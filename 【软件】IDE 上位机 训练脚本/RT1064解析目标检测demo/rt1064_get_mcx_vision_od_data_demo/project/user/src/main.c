/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP | ARM 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 
// 使用 MCX_Vision 模块连接 推荐使用1064学习板
//      模块管脚            单片机管脚
//      MCX_Vision_RX         C17
//      MCX_Vision_TX         C16
//      MCX_Vision_GND        GND
//      MCX_Vision_5V         电源
//      模块管脚            单片机管脚

//      双排排针 并口两寸屏 硬件引脚
//      RD                  查看 zf_device_ips200.h 中 IPS200_RD_PIN_PARALLEL8     宏定义 B0 
//      WR                  查看 zf_device_ips200.h 中 IPS200_WR_PIN_PARALLEL8     宏定义 B1 
//      RS                  查看 zf_device_ips200.h 中 IPS200_RS_PIN_PARALLEL8     宏定义 B2 
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_PARALLEL8    宏定义 C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_PARALLEL8     宏定义 B3 
//      BL                  查看 zf_device_ips200.h 中 IPS200_BL_PIN_PARALLEL8     宏定义 C18
//      D0-D7               查看 zf_device_ips200.h 中 IPS200_Dx_PIN_PARALLEL8     宏定义 B16/B17/B18/B19/D12/D13/D14/D15
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//      单排排针 SPI 两寸屏 硬件引脚
//      SCL                 查看 zf_device_ips200.h 中 IPS200_SCL_PIN_SPI  宏定义  B0
//      SDA                 查看 zf_device_ips200.h 中 IPS200_SDA_PIN_SPI  宏定义  B1
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_SPI  宏定义  B2
//      DC                  查看 zf_device_ips200.h 中 IPS200_DC_PIN_SPI   宏定义  C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_SPI   宏定义  B3 
//      BL                  查看 zf_device_ips200.h 中 IPS200_BLk_PIN_SPI  宏定义  C18
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// *************************** 例程测试说明 ***************************
// 1.MCX Vision下载好目标检测的例程
// 
// 2.使用MCX Vision连接到主板或核心板的串口4，插上IPS2.0寸屏幕，在断电情况下完成连接
// 
// 3.核心板烧录完成本例程
// 
// 4.复位后，可以再屏幕上看到目标检测返回的信息，如果没有屏幕，可以进入调试查看od_result数据

// **************************** 代码区域 ****************************
#define UART_INDEX              (UART_4   )                             
#define UART_BAUDRATE           (115200)                                
#define UART_TX_PIN             (UART4_TX_C16  )                        
#define UART_RX_PIN             (UART4_RX_C17  )                        
#define UART_PRIORITY           (LPUART4_IRQn)                                  

#define IPS200_TYPE             (IPS200_TYPE_SPI)   // 双排排针 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
                                                    // 单排排针 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI

uint8 uart_get_data[64];                            // 串口接收数据缓冲区
uint8 fifo_get_data[64];                            // fifo 输出读出缓冲区
                    
uint32 fifo_data_count = 0;                         // fifo 数据个数
fifo_struct uart_data_fifo;

typedef struct
{
    uint16 res_x1;
    uint16 res_y1;
    uint16 res_x2;
    uint16 res_y2;
}od_result_t;
volatile od_result_t od_result[10];

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);                  // 不可删除
    debug_init();                                   // 调试端口初始化
    
    system_delay_ms(300);
    // 此处编写用户代码 例如外设初始化代码等
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

    ips200_init(IPS200_TYPE);                                                   // 初始化屏幕
    
    // 显示固定的表格框架
    for(uint8 i = 0; i < 8; i++)
    {
        ips200_show_string(0, i * 40, "od_res_ :  x1   y1   x2   y2");
        ips200_show_uint(56, i * 40, i, 1);
        ips200_draw_line(0, 40 * (i + 1) - 2, 239, 40 * (i + 1) - 2, RGB565_BLACK);
    }
        
    while(1)
    {
        // 显示目标数据
        for(uint8 i = 0; i < 8; i++)
        {
            ips200_show_uint(85, i * 40 + 20, od_result[i].res_x1, 3);
            ips200_show_uint(125, i * 40 + 20, od_result[i].res_y1, 3);
            ips200_show_uint(165, i * 40 + 20, od_result[i].res_x2, 3);
            ips200_show_uint(205, i * 40 + 20, od_result[i].res_y2, 3);
        }
        system_delay_ms(10);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{ 
    uint8 get_data = 0;                                                             // 接收数据变量
    uint32 temp_length = 0;
    uint8 od_num = 0;
    uart_query_byte(UART_INDEX, &get_data);  
    {
        fifo_write_buffer(&uart_data_fifo, &get_data, 1);   
    }
    
    if(0xFF == get_data)
    {
        // 读取第1个数据，用于判断帧头，使用完清除此数据
        temp_length = 1;
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
        if(0xAA == fifo_get_data[0])
        {
            // 读取第1个数据，用于获取目标序号，使用完清除此数据
            temp_length = 1;
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
            od_num = fifo_get_data[0];
            // 读取8个数据，用于获取目标数据，然后转移到结构体数组中
            temp_length = 8;
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
            memcpy((uint8*)(&od_result[od_num]), fifo_get_data, 8);
        }
        fifo_clear(&uart_data_fifo);
    }
}
// **************************** 代码区域 ****************************
