#ifndef __SLOPE_ELEMENT_H__
#define __SLOPE_ELEMENT_H__

#include "img_processing.h"
#include "zf_common_typedef.h"
#include "tuning_params.h"

/**
 * @file slope_element.h
 * @brief 坡道识别与坡道专用控制模块接口
 * @details
 * 纯视觉方案下，本模块主要完成三件事：
 * 1. 从二值图中识别“中间收窄、左右近似对称”的坡道梯形外观；
 * 2. 按“上坡接近 -> 上坡 -> 顶部平台 -> 下坡 -> 出坡”顺序切换速度策略；
 * 3. 在坡道阶段输出更稳定的覆盖中线，减小俯仰变化对普通巡线的干扰。
 */

/**
 * @brief 坡道状态机枚举
 */
typedef enum
{
    SLOPE_STATE_NONE = 0,       // 当前未启用坡道专用控制。
    SLOPE_STATE_APPROACHING_UP, // 已识别到上坡梯形，正在提前补速接近。
    SLOPE_STATE_CLIMBING,       // 已开始进入上坡段。
    SLOPE_STATE_PLATFORM,       // 已到坡顶平台，准备观察下坡趋势。
    SLOPE_STATE_DESCENDING,     // 已确认进入下坡段。
    SLOPE_STATE_EXITING         // 已过下坡，正在回到普通巡线。
} Slope_State;

/**
 * @brief 坡道调试信息
 */
typedef struct
{
    Slope_State state;          // 当前坡道状态机状态。
    uint8_t trapezoid_detected; // 当前帧是否命中坡道梯形候选。
    uint8_t flat_detected;      // 当前帧是否更像平台或普通平直赛道。
    uint8_t reference_ready;    // 是否已经建立普通直道参考宽度。
    uint16_t lower_width;       // 下采样带平均赛道宽度。
    uint16_t middle_width;      // 中采样带平均赛道宽度。
    uint16_t upper_width;       // 上采样带平均赛道宽度。
    uint16_t base_lower_width;  // 普通直道参考下宽度。
    uint16_t close_rows;        // 下部已经明显收窄的连续行数。
    int16_t guide_center;       // 当前坡道覆盖中线使用的引导中心。
} Slope_Debug_Info;

/**
 * @brief 初始化坡道状态机
 */
void slope_element_init(void);

/**
 * @brief 处理当前帧的坡道识别与坡道专用控制
 * @param image 当前二值图
 * @param mid_line 普通巡线中线
 * @param override_mid_line 坡道模式下输出的覆盖中线
 * @param override_speed 坡道模式下建议使用的目标速度
 * @return 返回 1 表示当前应启用坡道专用控制，返回 0 表示继续普通巡线
 */
uint8_t slope_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                              const int16_t mid_line[IMAGE_HEIGHT],
                              int16_t override_mid_line[IMAGE_HEIGHT],
                              float *override_speed);

/**
 * @brief 获取当前坡道状态
 * @return 当前状态机状态
 */
Slope_State slope_element_get_state(void);

/**
 * @brief 获取当前坡道调试信息
 * @return 当前帧的调试快照
 */
Slope_Debug_Info slope_element_get_debug_info(void);

#endif
