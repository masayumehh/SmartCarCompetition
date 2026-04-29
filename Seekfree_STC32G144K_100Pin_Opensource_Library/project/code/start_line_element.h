#ifndef __START_LINE_ELEMENT_H__
#define __START_LINE_ELEMENT_H__

#include "img_processing.h"
#include "tuning_params.h"
#include "zf_common_typedef.h"

/**
 * @file start_line_element.h
 * @brief 发车线识别与单圈停车模块接口
 * @details
 * 本模块专门用于识别赛道中的双黑带发车线，并在“小车已经离开发车区后，
 * 再次识别到同一组发车线”时向上层返回停车请求。
 *
 * 设计目标如下：
 * 1. 不打断原有的图像处理中线提取流程；
 * 2. 不直接接管电机、舵机输出，只负责给出“是否完赛停车”的判断结果；
 * 3. 与十字、环岛、路障、坡道等已有元素模块并行工作，避免相互污染状态；
 * 4. 起步阶段自动忽略脚下的发车线，防止一发车就被误判为“已经跑完一圈”。
 */

/**
 * @brief 发车线识别状态机枚举
 * @details
 * START_LINE_STATE_WAITING_CLEAR
 * 刚发车时，车辆本身就停在发车线附近，因此此阶段不会立刻判定停车，
 * 而是先等待发车线从当前画面中连续消失若干帧。
 *
 * START_LINE_STATE_ARMED
 * 说明车辆已经基本驶离起点区域，此时重新进入“布防”状态，
 * 只要后续再次稳定识别到发车线，就认为已经回到起点附近。
 *
 * START_LINE_STATE_FINISHED
 * 说明已经确认完成一圈。该状态一旦进入，就会持续保持，
 * 直到上层重新初始化模块为止。
 */
typedef enum
{
    START_LINE_STATE_WAITING_CLEAR = 0,
    START_LINE_STATE_ARMED,
    START_LINE_STATE_FINISHED
} Start_Line_State;

/**
 * @brief 发车线调试信息结构体
 * @details
 * 该结构体用于上位机实时观察发车线识别内部过程，便于现场调参。
 * 其中各成员含义如下：
 * state
 * 当前状态机状态。
 *
 * pattern_detected
 * 当前这一帧是否已经满足“双黑带发车线”的几何特征。
 *
 * near_band_rows
 * 靠近车辆一侧的黑带厚度，单位为图像行数。
 *
 * gap_rows
 * 两条黑带之间白色间隔的厚度，单位为图像行数。
 *
 * far_band_rows
 * 远离车辆一侧的黑带厚度，单位为图像行数。
 *
 * approach_rows
 * 第一条黑带之前，连续保持“正常直道外观”的有效行数。
 * 该值越大，说明识别结果越像真正的发车线而不是其它元素噪声。
 *
 * guide_center
 * 本帧识别时所使用的引导中心列号。模块会优先参考普通循线中线，
 * 用它来约束发车线识别只在赛道中心附近进行。
 */
typedef struct
{
    Start_Line_State state;
    uint8_t pattern_detected;
    uint16_t near_band_rows;
    uint16_t gap_rows;
    uint16_t far_band_rows;
    uint16_t approach_rows;
    int16_t guide_center;
} Start_Line_Debug_Info;

// 函数简介     初始化发车线识别状态机
// 参数说明     void
// 返回参数     void
// 使用示例     start_line_element_init();
// 备注信息     每次正式发车前都应调用一次，用于清空上一轮识别状态与调试缓存
void start_line_element_init(void);

// 函数简介     直接把发车线状态机切到布防态
// 参数说明     void
// 返回参数     void
// 使用示例     start_line_element_arm();
// 备注信息     适用于车辆已经明显离开起点后，再按时间延迟启用完赛检测的场景
void start_line_element_arm(void);

// 函数简介     处理当前帧发车线识别逻辑，并返回是否已经完成一圈
// 参数说明     image       当前二值图像
// 参数说明     mid_line    当前普通循线模块输出的中线数组
// 返回参数     uint8_t     1-再次识别到发车线，应停车 0-继续正常运行
// 使用示例     if(start_line_element_process(image, mid_line)) { car_emergency_stop(); }
// 备注信息     本函数只给出“是否停车”的判定，不直接修改底盘输出
uint8_t start_line_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT]);

// 函数简介     获取当前发车线状态机状态
// 参数说明     void
// 返回参数     Start_Line_State 当前状态机枚举值
// 使用示例     Start_Line_State state = start_line_element_get_state();
Start_Line_State start_line_element_get_state(void);

// 函数简介     获取当前发车线调试信息快照
// 参数说明     void
// 返回参数     Start_Line_Debug_Info 当前调试信息
// 使用示例     Start_Line_Debug_Info debug = start_line_element_get_debug_info();
Start_Line_Debug_Info start_line_element_get_debug_info(void);

#endif
