#ifndef __CROSS_ELEMENT_H__
#define __CROSS_ELEMENT_H__

#include "img_processing.h"
#include "tuning_params.h"
#include "stdint.h"

/**
 * @file cross_element.h
 * @brief 十字交叉路口元素状态机接口
 * @details
 * 本模块负责：
 * 1. 根据当前图像和中线识别十字交叉路口；
 * 2. 在识别到十字后切换到十字专用状态；
 * 3. 在十字专用状态下输出“只允许直行”的专用控制中线和目标速度。
 */

/**
 * @brief 十字元素状态枚举
 */
typedef enum
{
    CROSS_STATE_NONE = 0,  // 未识别到十字，使用普通巡线控制。
    CROSS_STATE_ENTERING,  // 已确认进入十字入口，准备切换到专用控制。
    CROSS_STATE_INSIDE,    // 当前位于十字内部，强制保持直行。
    CROSS_STATE_EXITING    // 已离开十字主体，但仍保持短时间直行控制。
} Cross_State;

/**
 * @brief 十字状态机调试信息
 */
typedef struct
{
    Cross_State state;        // 当前十字状态。
    uint16_t lower_width;     // 下采样行的赛道宽度。
    uint16_t middle_width;    // 中采样行的赛道宽度。
    uint16_t upper_width;     // 上采样行的赛道宽度。
    int16_t guide_center;     // 十字专用控制使用的引导中心列。
    uint8_t pattern_detected; // 当前帧是否检测到十字宽度模式。
} Cross_Debug_Info;

void cross_element_init(void);  // 初始化十字元素状态机。
uint8_t cross_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                              const int16_t mid_line[IMAGE_HEIGHT],
                              int16_t override_mid_line[IMAGE_HEIGHT],
                              float *override_speed);  // 处理当前帧并输出十字专用控制结果。
Cross_State cross_element_get_state(void);  // 获取当前十字状态。
Cross_Debug_Info cross_element_get_debug_info(void);  // 获取十字调试信息。

#endif