#ifndef __ROUNDABOUT_ELEMENT_H__
#define __ROUNDABOUT_ELEMENT_H__

#include "img_processing.h"
#include "stdint.h"
#include "tuning_params.h"

/**
 * @file roundabout_element.h
 * @brief 环岛元素识别与专用控制接口
 * @details
 * 本模块负责：
 * 1. 根据识线图中的环岛特征识别顺时针或逆时针环岛；
 * 2. 在识别到环岛后切换到环岛专用状态；
 * 3. 在环岛专用状态下输出补线后的覆盖中线和保守目标速度。
 */

/**
 * @brief 环岛方向枚举
 * @details
 * 约定如下：
 * 1. 逆时针环岛表示左侧先丢线、右侧保持稳定；
 * 2. 顺时针环岛表示右侧先丢线、左侧保持稳定。
 */
typedef enum
{
    ROUNDABOUT_DIRECTION_NONE = 0,   // 当前未识别到环岛方向。
    ROUNDABOUT_DIRECTION_CLOCKWISE,  // 顺时针环岛。
    ROUNDABOUT_DIRECTION_COUNTERCLOCKWISE  // 逆时针环岛。
} Roundabout_Direction;

/**
 * @brief 环岛状态枚举
 */
typedef enum
{
    ROUNDABOUT_STATE_NONE = 0,  // 未识别到环岛。
    ROUNDABOUT_STATE_ENTERING,  // 正在确认进入环岛。
    ROUNDABOUT_STATE_INSIDE,    // 当前位于环岛识别与补线控制阶段。
    ROUNDABOUT_STATE_EXITING    // 环岛模式消失后的退出保持阶段。
} Roundabout_State;

/**
 * @brief 环岛调试信息
 */
typedef struct
{
    Roundabout_State state;          // 当前环岛状态。
    Roundabout_Direction direction;  // 当前识别出的环岛方向。
    uint16_t lower_width;            // 图像下部参考赛道宽度。
    uint16_t stable_edge;            // 稳定侧边线参考列号。
    uint16_t missing_rows;           // 连续丢线行数。
    int16_t arc_peak;                // 重现弧线的弧顶列号。
    uint16_t supplement_width;       // 由弧顶估算出的补线距离。
    int16_t guide_center;            // 环岛专用控制使用的引导中心列。
    uint8_t candidate_detected;      // 当前帧是否命中环岛候选模式。
} Roundabout_Debug_Info;

void roundabout_element_init(void);  // 初始化环岛元素状态机。
uint8_t roundabout_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT],
                                   int16_t override_mid_line[IMAGE_HEIGHT],
                                   float *override_speed);  // 处理当前帧环岛识别并输出专用控制结果。
Roundabout_State roundabout_element_get_state(void);  // 获取当前环岛状态。
Roundabout_Direction roundabout_element_get_direction(void);  // 获取当前环岛方向。
Roundabout_Debug_Info roundabout_element_get_debug_info(void);  // 获取环岛调试信息。

#endif
