/*********************************************************************************************************************
* MCX Vision Opensourec Library 即（MCX Vision 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2024 SEEKFREE 逐飞科技
* 
* 本文件是 MCX Vision 开源库的一部分
* 
* MCX Vision 开源库 是免费软件
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
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          zf_device_scc8660
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.38a
* 适用平台          MCX Vision
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-04-21        ZSY            first version
********************************************************************************************************************/
#include "zf_device_scc8660.h"

#if (FRAME_SIZE == SCC8660_QVGA)
__attribute__((section (".camera"))) volatile uint32_t g_camera_buffer[320*240/2]; __attribute__ ((aligned (32)))
#elif (FRAME_SIZE == SCC8660_QQVGA)
__attribute__((section (".camera"))) volatile uint32_t g_camera_buffer[120*160/2]; __attribute__ ((aligned (32)))
#endif

uint16_t *scc8660_image;    // 实际使用的摄像头数组头指针
uint32 scc8660_frame_size = FRAME_SIZE;

gpio_struct scc8660_iic_scl_gpio = {GPIO3, 3u};
gpio_struct scc8660_iic_sda_gpio = {GPIO3, 2u};
gpio_struct scc8660_rst_gpio = {GPIO1, 18u};

// 需要配置到摄像头的数据 不允许在这修改参数
static int16 scc8660_set_confing_buffer[SCC8660_CONFIG_FINISH][2]=
{
    {SCC8660_INIT,              0},                                             // 摄像头开始初始化

    {SCC8660_AUTO_EXP,          SCC8660_AUTO_EXP_DEF},                          // 自动曝光
    {SCC8660_BRIGHT,            SCC8660_BRIGHT_DEF},                            // 亮度设置
    {SCC8660_FPS,               SCC8660_FPS_DEF},                               // 图像帧率
    {SCC8660_SET_COL,           SCC8660_W},                                     // 图像列数
    {SCC8660_SET_ROW,           SCC8660_H},                                     // 图像行数
    {SCC8660_PCLK_DIV,          SCC8660_PCLK_DIV_DEF},                          // PCLK分频系数
    {SCC8660_PCLK_MODE,         SCC8660_PCLK_MODE_DEF},                         // PCLK模式
    {SCC8660_COLOR_MODE,        SCC8660_COLOR_MODE_DEF},                        // 图像色彩模式
    {SCC8660_DATA_FORMAT,       SCC8660_DATA_FORMAT_DEF},                       // 输出数据格式
#if SCC8660_IS_WB_AUTO
    {SCC8660_WB_R,              0},                                             // 自动白平衡
    {SCC8660_WB_G,              0},                                             // 自动白平衡
    {SCC8660_WB_B,              0},                                             // 自动白平衡
#else   
    {SCC8660_WB_R,              SCC8660_MANUAL_WB_R},                           // 自动白平衡
    {SCC8660_WB_G,              SCC8660_MANUAL_WB_G},                           // 自动白平衡
    {SCC8660_WB_B,              SCC8660_MANUAL_WB_B},                           // 自动白平衡
#endif
};

// 从摄像头内部获取到的配置数据 不允许在这修改参数
static int16 scc8660_get_confing_buffer[SCC8660_CONFIG_FINISH - 1][2]=
{
    {SCC8660_AUTO_EXP,          0},
    {SCC8660_BRIGHT,            0},                                             // 亮度设置          
    {SCC8660_FPS,               0},                                             // 图像帧率           
    {SCC8660_SET_COL,           0},                                             // 图像列数           
    {SCC8660_SET_ROW,           0},                                             // 图像行数          
    {SCC8660_PCLK_DIV,          0},                                             // PCLK分频系数      
    {SCC8660_PCLK_MODE,         0},                                             // PCLK模式      
    {SCC8660_COLOR_MODE,        0},                                             // 图像色彩模式
    {SCC8660_DATA_FORMAT,       0},                                             // 输出数据格式    
    {SCC8660_WB_R,              0},                                             // 白平衡设置
    {SCC8660_WB_G,              0},                                             // 白平衡设置
    {SCC8660_WB_B,              0},                                             // 白平衡设置
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单独设置图像亮度
// 参数说明     data            需要设置的亮度值
// 返回参数     uint8           1-失败 0-成功
// 使用示例     scc8660_set_bright(data);                                       
// 备注信息     调用该函数前请先初始化摄像头配置串口
//-------------------------------------------------------------------------------------------------------------------
uint8 scc8660_set_brightness (uint16 data)
{
    uint8 return_state = 0;
    return_state = scc8660_set_brightness_sccb(data);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单独设置白平衡
// 参数说明     wb_r    手动模式，此参数设置白平衡红色参数，值越大，画面越红， 自动模式，此设置失效
// 参数说明     wb_g    手动模式，此参数设置白平衡绿色参数，值越大，画面越绿， 自动模式，此设置失效
// 参数说明     wb_b    手动模式，此参数设置白平衡蓝色参数，值越大，画面越蓝， 自动模式，此设置失效
// 参数说明     wb_r和wb_g和wb_b都为0，则设置为自动模式
// 返回参数     uint8           1-失败 0-成功
// 使用示例     scc8660_set_white_balance (wb_r, wb_g, wb_b);          
//-------------------------------------------------------------------------------------------------------------------
uint8 scc8660_set_white_balance (uint16 wb_r, uint16 wb_g, uint16 wb_b)
{
    uint8 return_state = 0;
    return_state = scc8660_set_manual_wb_sccb(wb_r, wb_g, wb_b);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对摄像头内部寄存器进行写操作
// 参数说明     addr            摄像头内部寄存器地址
// 参数说明     data            需要写入的数据
// 返回参数     uint8           1-失败 0-成功
// 使用示例     scc8660_set_reg(addr, data);                    // 调用该函数前请先初始化串口
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 scc8660_set_reg (uint8 addr, uint16 data)
{
    uint8 return_state = 0;
    return_state = scc8660_set_reg_sccb(addr, data);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SCC8660 摄像头初始化
// 参数说明     void
// 返回参数     uint8           1-失败 0-成功
// 使用示例     scc8660_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 scc8660_init (void)
{
    uint8 return_state = 0;
     
    CLOCK_EnableClock(kCLOCK_InputMux0);
    /* Enables the clock for PORT0 controller: Enables clock */
    CLOCK_EnableClock(kCLOCK_Port3);
    PORT_SetPinMux(PORT3, 6U, kPORT_MuxAsGpio);

    PORT3->PCR[6] = ((PORT3->PCR[6] &
                     /* Mask bits to zero which are setting */
                     (~(PORT_PCR_IBE_MASK)))

                    /* Input Buffer Enable: Enables. */
                    | PORT_PCR_IBE(0x01u));
    gpio_init(scc8660_rst_gpio, GPO, 1, PULL_UP);
    soft_iic_info_struct scc8660_iic_struct;
    do
    {
        system_delay_ms(200);
        soft_iic_init(&scc8660_iic_struct, 0, SCC8660_COF_IIC_DELAY, &scc8660_iic_scl_gpio, &scc8660_iic_sda_gpio);

        if(scc8660_set_config_sccb(&scc8660_iic_struct, scc8660_set_confing_buffer))
        {
            zf_debug_printf("camera_init_error\r\n");
        }
        else
        {
            zf_debug_printf("camera_init_finish\r\n");
            break;
        }
    }while(0);

    memset((uint8*)g_camera_buffer, 0xff, sizeof(g_camera_buffer));
    zf_debug_printf("SCC8660_SIZE:%d\r\n", sizeof(g_camera_buffer));
    ezh_start();
    return return_state;
}
