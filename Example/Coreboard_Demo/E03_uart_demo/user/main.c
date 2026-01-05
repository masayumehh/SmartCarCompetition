/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
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
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// *************************** 例程硬件连接说明 ***************************
// 使用 type-c 连接
//      直接将type-c插入核心板，即可使用USB-CDC
//
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 main.c 文件中 UART_TX_PIN 宏定义的引脚
//      USB-TTL-TX          查看 main.c 文件中 UART_RX_PIN 宏定义的引脚
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用 type-c接口 和 USB-TTL 模块，在断电情况下完成连接
//
// 2.将 type-c接口 和 USB-TTL 模块连接电脑，完成上电
//
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
//
// 4.可以在串口助手上看到如下串口信息：
//      UART Text.
//
// 5.通过串口助手发送数据，会收到相同的反馈数据
//      UART get data:.......
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

#define UART_INDEX              ( UART_5   )                // 默认 UART_5
#define UART_BAUDRATE           ( 115200 )                  // 默认 115200
#define UART_TX_PIN             ( UART5_TX_P05 )            // 默认 UART5_TX_P05
#define UART_RX_PIN             ( UART5_RX_P04 )            // 默认 UART5_RX_P04


uint8       uart_get_data[64] = {0};                        // 串口接收数据缓冲区
uint8       fifo_get_data[64] = {0};                        // fifo 输出读出缓冲区


uint32      fifo_data_count = 0;                            // fifo 数据个数

fifo_struct uart_data_fifo = {0};


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (uint8 dat)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_INDEX, &dat);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &dat, 1);                           // 将数据写入 fifo 中
}

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

	
	// 此处编写用户代码 例如外设初始化代码等
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    
	// UART1的中断优先级不能设置，为最低优先级值0
	// UART1的中断优先级不能设置，为最低优先级值0
	// UART1的中断优先级不能设置，为最低优先级值0
	uart_rx_interrupt(UART_INDEX, ZF_ENABLE, uart_rx_interrupt_handler);                                   // 开启 UART_INDEX 的接收中断

	uart_write_string(UART_INDEX, "UART Text.");                                // 输出测试信息
    uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    uart_write_byte(UART_INDEX, '\n');                                          // 输出换行

	
	// 此处编写用户代码 例如外设初始化代码等
	
	while(1)
	{
        // 此处编写需要循环执行的代码
        fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
        if(fifo_data_count != 0)                                                // 读取到数据了
        {
            // 为了防止在读取FIFO的时候，又写入FIFO，这里关闭总中断。
            interrupt_global_disable();
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
            interrupt_global_enable();
            
            uart_write_string(UART_INDEX, "UART get data:");                     // 输出测试信息
            uart_write_buffer(UART_INDEX, fifo_get_data, (uint16)fifo_data_count);       // 将读取到的数据发送出去
            uart_write_string(UART_INDEX, "\r\n");                               // 输出测试信息
        }
     
        system_delay_ms(10);
        // 此处编写需要循环执行的代码
	}
}

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
//
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的type-c的USB_CDC
//      如果是type-c的USB_CDC连接，那么检查下载器线是否松动
//