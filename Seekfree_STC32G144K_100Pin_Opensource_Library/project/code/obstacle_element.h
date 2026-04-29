#ifndef __OBSTACLE_ELEMENT_H__
#define __OBSTACLE_ELEMENT_H__

#include "img_processing.h"
#include "zf_common_typedef.h"
#include "tuning_params.h"

/**
 * @file obstacle_element.h
 * @brief 路障识别与绕行模块接口
 * @details
 * 本模块负责在赛道内部寻找“连续黑块”路障候选，
 * 然后按“接近 -> 绕行 -> 出障”三个阶段输出：
 * 1. 专用覆盖中线；
 * 2. 更保守的目标速度；
 * 3. 供调参与调试查看的状态快照。
 */

/**
 * @brief 路障状态机枚举
 */
typedef enum
{
    OBSTACLE_STATE_NONE = 0,    // 当前未启用路障专用控制。
    OBSTACLE_STATE_APPROACHING, // 已识别到路障，正在提前减速并规划通道。
    OBSTACLE_STATE_AVOIDING,    // 路障已接近到触发线，正在执行正式绕行。
    OBSTACLE_STATE_EXITING      // 路障主体已离开视野，短时间保持回正过渡。
} Obstacle_State;

/**
 * @brief 路障侧别枚举
 * @details
 * 既表示障碍物位于赛道左侧还是右侧，
 * 也表示车辆最终选择从哪一侧绕行。
 */
typedef enum
{
    OBSTACLE_SIDE_NONE = 0, // 当前没有可靠的左右侧信息。
    OBSTACLE_SIDE_LEFT,     // 位于赛道左侧。
    OBSTACLE_SIDE_RIGHT     // 位于赛道右侧。
} Obstacle_Side;

/**
 * @brief 路障调试信息
 */
typedef struct
{
    Obstacle_State state;         // 当前路障状态机状态。
    Obstacle_Side obstacle_side;  // 当前识别到的障碍物所在侧。
    Obstacle_Side pass_side;      // 当前规划的绕行侧。
    int16_t obstacle_center;      // 路障包围盒中心列号。
    uint16_t obstacle_width;      // 路障包围盒宽度。
    uint16_t obstacle_height;     // 路障包围盒高度。
    uint16_t obstacle_bottom_row; // 路障包围盒最靠近车体的底部行号。
    int16_t guide_center;         // 当前绕行中线使用的引导中心。
    uint8_t candidate_detected;   // 当前帧是否命中了单帧路障候选。
} Obstacle_Debug_Info;

/**
 * @brief 初始化路障状态机和调试快照
 */
void obstacle_element_init(void);

/**
 * @brief 处理当前帧的路障识别与绕行控制
 * @param image 当前二值图
 * @param mid_line 普通巡线中线
 * @param override_mid_line 路障模式下输出的覆盖中线
 * @param override_speed 路障模式下建议使用的目标速度
 * @return 返回 1 表示当前应启用路障专用控制，返回 0 表示继续普通巡线
 */
uint8_t obstacle_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                 const int16_t mid_line[IMAGE_HEIGHT],
                                 int16_t override_mid_line[IMAGE_HEIGHT],
                                 float *override_speed);

/**
 * @brief 获取当前路障状态
 * @return 当前状态机状态
 */
Obstacle_State obstacle_element_get_state(void);

/**
 * @brief 获取当前路障调试信息
 * @return 当前帧对应的调试快照
 */
Obstacle_Debug_Info obstacle_element_get_debug_info(void);

#endif
