#ifndef __ROUNDABOUT_ELEMENT_H__
#define __ROUNDABOUT_ELEMENT_H__

#include "img_processing.h"
#include "zf_common_typedef.h"
#include "tuning_params.h"

/**
 * @file roundabout_element.h
 * @brief 环岛识别与补线控制模块接口
 * @details
 * 模块会根据稳定边、异常边以及 A/B/C 关键点判断是否进入环岛，
 * 并在入环、环内、出环三个阶段输出更适合环岛的覆盖中线与速度建议。
 */

/**
 * @brief 环岛方向枚举
 * @details
 * 方向与“哪一侧边线稳定”对应：
 * 1. 顺时针环岛：左边线更稳定，右边线失真更明显；
 * 2. 逆时针环岛：右边线更稳定，左边线失真更明显。
 */
typedef enum
{
    ROUNDABOUT_DIRECTION_NONE = 0,         // 当前未确认出有效环岛方向。
    ROUNDABOUT_DIRECTION_CLOCKWISE,        // 顺时针环岛。
    ROUNDABOUT_DIRECTION_COUNTERCLOCKWISE  // 逆时针环岛。
} Roundabout_Direction;

/**
 * @brief 环岛稳定边枚举
 */
typedef enum
{
    ROUNDABOUT_SIDE_NONE = 0, // 当前没有可靠稳定边。
    ROUNDABOUT_SIDE_LEFT,     // 左边线作为稳定参考边。
    ROUNDABOUT_SIDE_RIGHT     // 右边线作为稳定参考边。
} Roundabout_Side;

/**
 * @brief 环岛状态机枚举
 */
typedef enum
{
    ROUNDABOUT_STATE_NONE = 0, // 普通巡线状态，未启用环岛控制。
    ROUNDABOUT_STATE_ENTERING, // 已确认进入环岛入口，正在执行入环补线。
    ROUNDABOUT_STATE_INSIDE,   // 当前位于环岛内部，使用稳定边补线。
    ROUNDABOUT_STATE_EXITING   // 已检测到出环趋势，正在切回普通巡线。
} Roundabout_State;

/**
 * @brief 环岛关键点
 * @details
 * A/B/C 三点用于组织环岛入口与环内补线：
 * 1. A 点：入口开始失真的起点；
 * 2. B 点：入口补线过渡点；
 * 3. C 点：环内弧线的高可信拐点。
 */
typedef struct
{
    int16_t row;   // 关键点所在行号。
    int16_t col;   // 关键点所在列号。
    uint8_t valid; // 当前关键点是否有效。
} Roundabout_Key_Point;

/**
 * @brief 环岛调试信息
 */
typedef struct
{
    Roundabout_State state;         // 当前环岛状态机状态。
    Roundabout_Direction direction; // 当前确认的环岛方向。
    Roundabout_Side stable_side;    // 当前确认的稳定边侧。
    uint16_t lower_width;           // 下部参考区域平均赛道宽度。
    uint16_t stable_edge;           // 稳定边参考位置。
    uint16_t missing_rows;          // 异常边连续缺失行数。
    int16_t arc_peak;               // 当前弧顶列号。
    uint16_t supplement_width;      // 当前补线宽度估计。
    int16_t guide_center;           // 当前覆盖中线使用的引导中心。
    uint8_t candidate_detected;     // 当前帧是否命中环岛候选。
    Roundabout_Key_Point point_a;   // A 点快照。
    Roundabout_Key_Point point_b;   // B 点快照。
    Roundabout_Key_Point point_c;   // C 点快照。
} Roundabout_Debug_Info;

/**
 * @brief 初始化环岛状态机
 */
void roundabout_element_init(void);

/**
 * @brief 处理当前帧的环岛识别与补线控制
 * @param image 当前二值图
 * @param mid_line 普通巡线中线
 * @param override_mid_line 环岛模式下输出的覆盖中线
 * @param override_speed 环岛模式下建议使用的目标速度
 * @return 返回 1 表示启用环岛专用控制，返回 0 表示继续普通巡线
 */
uint8_t roundabout_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT],
                                   int16_t override_mid_line[IMAGE_HEIGHT],
                                   float *override_speed);

/**
 * @brief 获取当前环岛状态
 * @return 当前状态机状态
 */
Roundabout_State roundabout_element_get_state(void);

/**
 * @brief 获取当前环岛方向
 * @return 当前确认方向
 */
Roundabout_Direction roundabout_element_get_direction(void);

/**
 * @brief 获取当前环岛调试信息
 * @return 当前帧的调试快照
 */
Roundabout_Debug_Info roundabout_element_get_debug_info(void);

#endif
