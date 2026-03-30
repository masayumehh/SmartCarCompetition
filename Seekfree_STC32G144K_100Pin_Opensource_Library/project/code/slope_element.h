#ifndef __SLOPE_ELEMENT_H__
#define __SLOPE_ELEMENT_H__

#include "img_processing.h"
#include "zf_common_typedef.h"
#include "tuning_params.h"

/**
 * @file slope_element.h
 * @brief 坡道元素状态机接口
 * @details
 * 纯视觉方案下，坡道模块主要完成三件事：
 * 1. 从二值图中识别“居中、对称、逐渐收窄”的梯形直道外观；
 * 2. 按“上坡接近 -> 上坡中 -> 顶部平台 -> 下坡 -> 出坡”顺序驱动速度策略；
 * 3. 在坡道过程中输出一条更稳的覆盖中线，降低俯仰变化对普通中线的扰动。
 */

/**
 * @brief 坡道状态枚举
 */
typedef enum
{
    SLOPE_STATE_NONE = 0,          /**< 当前未启用坡道专用控制。 */
    SLOPE_STATE_APPROACHING_UP,    /**< 已识别到前方上坡梯形，正在提前补速接近。 */
    SLOPE_STATE_CLIMBING,          /**< 已经开始进入上坡段，继续保持适度上坡动力。 */
    SLOPE_STATE_PLATFORM,          /**< 已经上到顶部平台，速度回收，准备观察下坡。 */
    SLOPE_STATE_DESCENDING,        /**< 已经识别到下坡段，切换到更保守的下坡速度。 */
    SLOPE_STATE_EXITING            /**< 已经过完下坡，短时间平滑回到普通巡线。 */
} Slope_State;

/**
 * @brief 坡道调试信息
 *
 * 串口调试时可以借助这些量观察：
 * 1. 当前是否命中“梯形候选”；
 * 2. 当前是否已经回到“平台/平直直道”外观；
 * 3. 三个采样高度的赛道宽度是否符合预期；
 * 4. 坡道模块当前给出的引导中心和参考宽度是否稳定。
 */
typedef struct
{
    Slope_State state;          /**< 当前坡道状态机状态。 */
    uint8_t trapezoid_detected; /**< 当前帧是否命中坡道梯形候选。 */
    uint8_t flat_detected;      /**< 当前帧是否命中平台/平直直道候选。 */
    uint8_t reference_ready;    /**< 是否已经建立普通直道参考宽度。 */
    uint16_t lower_width;       /**< 下采样高度的平均赛道宽度。 */
    uint16_t middle_width;      /**< 中采样高度的平均赛道宽度。 */
    uint16_t upper_width;       /**< 上采样高度的平均赛道宽度。 */
    uint16_t base_lower_width;  /**< 普通直道参考下宽度。 */
    uint16_t close_rows;        /**< 下部已经明显收窄的行数，用来判断是否贴近坡脚。 */
    int16_t guide_center;       /**< 当前覆盖中线使用的引导中心列。 */
} Slope_Debug_Info;

void slope_element_init(void);  /**< 初始化坡道元素状态机。 */

/**
 * @brief 坡道识别与专用控制主函数
 *
 * @param image 当前帧二值图。
 * @param mid_line 普通巡线模块提取出的中线。
 * @param override_mid_line 坡道模式下生成的覆盖中线输出缓冲区。
 * @param override_speed 坡道模式下建议使用的目标速度输出地址。
 * @return uint8_t 返回 1 表示当前启用坡道专用控制，返回 0 表示未启用。
 */
uint8_t slope_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                              const int16_t mid_line[IMAGE_HEIGHT],
                              int16_t override_mid_line[IMAGE_HEIGHT],
                              float *override_speed);

Slope_State slope_element_get_state(void);            /**< 获取当前坡道状态。 */
Slope_Debug_Info slope_element_get_debug_info(void);  /**< 获取当前坡道调试信息。 */

#endif
