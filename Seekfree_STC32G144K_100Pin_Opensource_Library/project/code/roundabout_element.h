#ifndef __ROUNDABOUT_ELEMENT_H__
#define __ROUNDABOUT_ELEMENT_H__

#include "img_processing.h"
#include "stdint.h"
#include "tuning_params.h"

/**
 * @brief 环岛方向枚举
 *
 * 这里的方向定义与“哪一侧边线稳定”对应：
 * 1. 顺时针：左侧边线稳定，右侧边线异常；
 * 2. 逆时针：右侧边线稳定，左侧边线异常。
 */
typedef enum
{
    ROUNDABOUT_DIRECTION_NONE = 0,          /**< 当前未识别到有效环岛方向。 */
    ROUNDABOUT_DIRECTION_CLOCKWISE,         /**< 顺时针环岛，左边稳定、右边异常。 */
    ROUNDABOUT_DIRECTION_COUNTERCLOCKWISE   /**< 逆时针环岛，右边稳定、左边异常。 */
} Roundabout_Direction;

/**
 * @brief 环岛稳定边侧枚举
 *
 * 新版环岛模块不再只看“哪一边丢线”，
 * 而是明确记录哪一边是稳定参考边。
 */
typedef enum
{
    ROUNDABOUT_SIDE_NONE = 0,  /**< 当前没有有效稳定边信息。 */
    ROUNDABOUT_SIDE_LEFT,      /**< 左侧边线作为稳定参考边。 */
    ROUNDABOUT_SIDE_RIGHT      /**< 右侧边线作为稳定参考边。 */
} Roundabout_Side;

/**
 * @brief 环岛状态机枚举
 *
 * 1. NONE：未处于环岛模式；
 * 2. ENTERING：已确认入口特征，正在按 A-B-C 点做进环补线；
 * 3. INSIDE：已进入环岛内部，优先使用稳定边拾边；
 * 4. EXITING：已检测到出环趋势，正在平滑切回普通巡线。
 */
typedef enum
{
    ROUNDABOUT_STATE_NONE = 0,  /**< 当前未启用环岛控制。 */
    ROUNDABOUT_STATE_ENTERING,  /**< 当前正在进入环岛。 */
    ROUNDABOUT_STATE_INSIDE,    /**< 当前正在环岛内部行驶。 */
    ROUNDABOUT_STATE_EXITING    /**< 当前正在退出环岛。 */
} Roundabout_State;

/**
 * @brief 环岛关键点结构体
 *
 * 环岛按 A/B/C 三个关键点组织补线：
 * 1. A：入口开始失真的起点；
 * 2. B：入口补线结束点；
 * 3. C：弧线最高可信点。
 */
typedef struct
{
    int16_t row;     /**< 点所在行号。 */
    int16_t col;     /**< 点所在列号。 */
    uint8_t valid;   /**< 该点当前是否有效。 */
} Roundabout_Key_Point;

/**
 * @brief 环岛调试信息结构体
 *
 * 用于串口调试查看：
 * 1. 环岛是否触发；
 * 2. 当前方向和稳定边是哪一侧；
 * 3. A/B/C 点是否正确找到；
 * 4. 当前补线宽度和引导中心是否合理。
 */
typedef struct
{
    Roundabout_State state;          /**< 当前环岛状态机状态。 */
    Roundabout_Direction direction;  /**< 当前确认的环岛方向。 */
    Roundabout_Side stable_side;     /**< 当前确认的稳定边侧。 */
    uint16_t lower_width;            /**< 图像下部参考区域的平均赛道宽度。 */
    uint16_t stable_edge;            /**< 稳定边参考位置。 */
    uint16_t missing_rows;           /**< 异常边连续缺失的行数。 */
    int16_t arc_peak;                /**< C 点对应的弧顶列号。 */
    uint16_t supplement_width;       /**< 当前环岛内部补线宽度。 */
    int16_t guide_center;            /**< 当前覆盖中线的引导中心。 */
    uint8_t candidate_detected;      /**< 当前帧是否命中环岛候选。 */
    Roundabout_Key_Point point_a;    /**< 当前识别到的 A 点。 */
    Roundabout_Key_Point point_b;    /**< 当前识别到的 B 点。 */
    Roundabout_Key_Point point_c;    /**< 当前识别到的 C 点。 */
} Roundabout_Debug_Info;

/**
 * @brief 初始化环岛模块
 */
void roundabout_element_init(void);

/**
 * @brief 环岛识别与补线主函数
 *
 * 输入当前帧二值图和普通中线，
 * 输出是否启用环岛覆盖中线和环岛专用速度。
 *
 * @param image 当前帧二值图。
 * @param mid_line 普通巡线模块提取出的中线。
 * @param override_mid_line 环岛模式下生成的覆盖中线输出缓冲区。
 * @param override_speed 环岛模式下建议使用的目标速度输出地址。
 * @return uint8_t 返回 1 表示启用环岛控制，返回 0 表示未启用。
 */
uint8_t roundabout_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT],
                                   int16_t override_mid_line[IMAGE_HEIGHT],
                                   float *override_speed);

/**
 * @brief 获取当前环岛状态
 * @return Roundabout_State 当前状态机状态。
 */
Roundabout_State roundabout_element_get_state(void);

/**
 * @brief 获取当前环岛方向
 * @return Roundabout_Direction 当前确认方向。
 */
Roundabout_Direction roundabout_element_get_direction(void);

/**
 * @brief 获取当前环岛调试信息
 * @return Roundabout_Debug_Info 当前调试信息快照。
 */
Roundabout_Debug_Info roundabout_element_get_debug_info(void);

#endif
