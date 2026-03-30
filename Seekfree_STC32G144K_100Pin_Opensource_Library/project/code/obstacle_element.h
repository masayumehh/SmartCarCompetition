#ifndef __OBSTACLE_ELEMENT_H__
#define __OBSTACLE_ELEMENT_H__

#include "img_processing.h"
#include "zf_common_typedef.h"
#include "tuning_params.h"

/**
 * @brief 路障状态机枚举
 *
 * 路障处理分为靠近、绕行和出障三个阶段，
 * 这样可以让车辆在不同距离下使用不同的目标速度和中线策略。
 */
typedef enum
{
    OBSTACLE_STATE_NONE = 0,     /**< 当前未识别到有效路障。 */
    OBSTACLE_STATE_APPROACHING,  /**< 已经识别到路障，正在提前减速接近。 */
    OBSTACLE_STATE_AVOIDING,     /**< 路障已接近到触发阈值，正在绕行。 */
    OBSTACLE_STATE_EXITING       /**< 路障候选消失后，保持一段时间完成回正。 */
} Obstacle_State;

/**
 * @brief 路障所在侧枚举
 *
 * 这里既用于描述障碍物本身位于赛道左/右哪一侧，
 * 也用于描述车辆应该从哪一侧绕行通过。
 */
typedef enum
{
    OBSTACLE_SIDE_NONE = 0,  /**< 当前无有效左右侧信息。 */
    OBSTACLE_SIDE_LEFT,      /**< 位于赛道左侧。 */
    OBSTACLE_SIDE_RIGHT      /**< 位于赛道右侧。 */
} Obstacle_Side;

/**
 * @brief 路障调试信息结构体
 *
 * 串口调试时可以通过该结构体观察：
 * 1. 是否检测到路障候选；
 * 2. 路障位于左侧还是右侧；
 * 3. 当前是否已经进入绕行状态；
 * 4. 补线中线的引导中心是否合理。
 */
typedef struct
{
    Obstacle_State state;          /**< 当前路障状态机状态。 */
    Obstacle_Side obstacle_side;   /**< 识别到的障碍物所在侧。 */
    Obstacle_Side pass_side;       /**< 规划得到的绕行侧。 */
    int16_t obstacle_center;       /**< 障碍物包围盒中心列号。 */
    uint16_t obstacle_width;       /**< 障碍物包围盒宽度。 */
    uint16_t obstacle_height;      /**< 障碍物包围盒高度。 */
    uint16_t obstacle_bottom_row;  /**< 障碍物包围盒最靠下的行号。 */
    int16_t guide_center;          /**< 当前覆盖中线所使用的引导中心。 */
    uint8_t candidate_detected;    /**< 当前帧是否检测到有效路障候选。 */
} Obstacle_Debug_Info;

/**
 * @brief 初始化路障模块
 *
 * 清空路障状态机和调试信息，恢复为未识别状态。
 */
void obstacle_element_init(void);

/**
 * @brief 路障识别与绕行主函数
 *
 * 输入当前帧二值图与普通巡线中线，若识别到赛道内部障碍物，
 * 则输出绕障使用的覆盖中线和减速后的目标速度。
 *
 * @param image 当前帧二值图。
 * @param mid_line 普通巡线模块生成的中线。
 * @param override_mid_line 路障模式下的覆盖中线输出缓冲区。
 * @param override_speed 路障模式下建议使用的目标速度输出地址。
 * @return uint8_t 返回 1 表示当前应启用路障控制，返回 0 表示未启用。
 */
uint8_t obstacle_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                 const int16_t mid_line[IMAGE_HEIGHT],
                                 int16_t override_mid_line[IMAGE_HEIGHT],
                                 float *override_speed);

/**
 * @brief 获取当前路障状态机状态
 * @return Obstacle_State 当前路障状态。
 */
Obstacle_State obstacle_element_get_state(void);

/**
 * @brief 获取当前路障调试信息
 * @return Obstacle_Debug_Info 当前一帧对应的调试信息。
 */
Obstacle_Debug_Info obstacle_element_get_debug_info(void);

#endif
