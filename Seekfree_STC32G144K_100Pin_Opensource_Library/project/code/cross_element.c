#include "cross_element.h"

/**
 * 十字识别思路：
 * 1. 图像下部仍然保持普通直道宽度；
 * 2. 图像中部因为横向道路接入而明显变宽；
 * 3. 图像上部重新收回到接近普通直道宽度；
 * 4. 上下两处中心偏差不能过大，避免把普通大弯误识别成十字。
 */

#define CROSS_SAMPLE_ROW_LOWER        ((IMAGE_HEIGHT * TUNE_CROSS_SAMPLE_ROW_LOWER_RATIO_NUM) / TUNE_CROSS_SAMPLE_ROW_LOWER_RATIO_DEN)    // 十字检测下采样行。
#define CROSS_SAMPLE_ROW_MIDDLE       ((IMAGE_HEIGHT * TUNE_CROSS_SAMPLE_ROW_MIDDLE_RATIO_NUM) / TUNE_CROSS_SAMPLE_ROW_MIDDLE_RATIO_DEN)  // 十字检测中采样行。
#define CROSS_SAMPLE_ROW_UPPER        ((IMAGE_HEIGHT * TUNE_CROSS_SAMPLE_ROW_UPPER_RATIO_NUM) / TUNE_CROSS_SAMPLE_ROW_UPPER_RATIO_DEN)    // 十字检测上采样行。

#define CROSS_NORMAL_MIN_WIDTH        TUNE_CROSS_NORMAL_MIN_WIDTH         // 普通直道最小宽度阈值。
#define CROSS_NORMAL_MAX_WIDTH        TUNE_CROSS_NORMAL_MAX_WIDTH         // 普通直道最大宽度阈值。
#define CROSS_WIDE_MIN_WIDTH          TUNE_CROSS_WIDE_MIN_WIDTH           // 十字中部至少扩展到的宽度。
#define CROSS_WIDTH_EXPAND_THRESHOLD  TUNE_CROSS_WIDTH_EXPAND_THRESHOLD   // 中部相对直道至少增加的宽度。
#define CROSS_CENTER_DIFF_LIMIT       TUNE_CROSS_CENTER_DIFF_LIMIT        // 上下采样行中心允许偏差。

#define CROSS_DETECT_CONFIRM_FRAMES   TUNE_CROSS_DETECT_CONFIRM_FRAMES    // 连续多少帧满足模式才确认进入十字。
#define CROSS_LOST_CONFIRM_FRAMES     TUNE_CROSS_LOST_CONFIRM_FRAMES      // 连续多少帧不满足模式才判定离开十字主体。
#define CROSS_EXIT_HOLD_FRAMES        TUNE_CROSS_EXIT_HOLD_FRAMES         // 离开十字后继续保持直行控制的帧数。
#define CROSS_SPECIAL_TARGET_SPEED    TUNE_CROSS_SPECIAL_TARGET_SPEED     // 十字专用目标速度。

/**
 * @brief 十字状态机内部上下文
 */
typedef struct
{
    Cross_State state;       // 当前十字状态。
    uint8_t detect_count;    // 连续检测到十字的帧数。
    uint8_t lost_count;      // 连续未检测到十字的帧数。
    uint8_t exit_hold_count; // 离开十字后继续直行的保持帧数。
    int16_t guide_center;    // 十字专用控制使用的引导中心列。
} Cross_Context;

static Cross_Context g_cross_ctx = {CROSS_STATE_NONE, 0U, 0U, 0U, (int16_t)(IMAGE_WIDTH / 2U)};  // 全局十字状态机上下文。
static Cross_Debug_Info g_cross_debug = {CROSS_STATE_NONE, 0U, 0U, 0U, (int16_t)(IMAGE_WIDTH / 2U), 0U};  // 当前帧十字调试快照。

/**
 * @brief 统计某一行非黑像素的左右边界和宽度
 * @param image 当前处理图像
 * @param row 需要统计的行号
 * @param left 输出左边界列号
 * @param right 输出右边界列号
 * @return 该行赛道宽度，未找到时返回 0
 */
static uint16_t get_row_nonzero_width(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                      uint16_t row,
                                      uint16_t *left,
                                      uint16_t *right)
{
    uint16_t x;           // 当前扫描列号。
    uint16_t found = 0U;  // 当前行是否已经找到非黑像素。

    *left = 0U;   // 先清左边界输出。
    *right = 0U;  // 先清右边界输出。

    for (x = 0U; x < IMAGE_WIDTH; x++)
    {
        if (image[row][x] > 0U)
        {
            if (!found)
            {
                *left = x;   // 第一次遇到非黑像素时记录左边界。
                found = 1U;  // 标记该行已经出现赛道区域。
            }

            *right = x;  // 持续更新右边界到最后一个非黑像素。
        }
    }

    if (!found)
    {
        return 0U;  // 整行都没有赛道区域时返回 0 宽度。
    }

    return (uint16_t)(*right - *left + 1U);  // 返回该行赛道跨度。
}

/**
 * @brief 根据图像下部中线估算十字直行引导中心
 * @param mid_line 当前普通巡线中线
 * @return 十字专用控制使用的引导中心列
 */
static int16_t calculate_guide_center(const int16_t mid_line[IMAGE_HEIGHT])
{
    uint16_t y;         // 当前统计的行号。
    int32_t sum = 0;    // 有效中线列号总和。
    uint16_t count = 0; // 有效中线点数量。

    for (y = (uint16_t)(IMAGE_HEIGHT * 2U / 3U); y < IMAGE_HEIGHT; y++)
    {
        if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH)
        {
            sum += mid_line[y];  // 累加图像下部有效中线。
            count++;             // 记录有效点数量。
        }
    }

    if (count == 0U)
    {
        return (int16_t)(IMAGE_WIDTH / 2U);  // 没有可靠中线时退回图像中心。
    }

    return (int16_t)(sum / (int32_t)count);  // 返回下部中线平均列号。
}

/**
 * @brief 生成十字专用直行中线
 * @param guide_center 当前引导中心列
 * @param override_mid_line 输出中线数组
 */
static void build_straight_override_mid_line(int16_t guide_center, int16_t override_mid_line[IMAGE_HEIGHT])
{
    uint16_t y;  // 当前写入的行号。

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        override_mid_line[y] = guide_center;  // 整条中线都强制设为同一中心，实现只允许直行。
    }
}

/**
 * @brief 判断当前帧是否满足十字宽度模式
 * @param image 当前图像
 * @param mid_line 当前普通巡线中线
 * @param guide_center 输出引导中心列
 * @return 1 表示检测到十字模式，0 表示未检测到
 */
static uint8_t detect_cross_pattern(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                    const int16_t mid_line[IMAGE_HEIGHT],
                                    int16_t *guide_center)
{
    uint16_t lower_left;    // 下采样行左边界。
    uint16_t lower_right;   // 下采样行右边界。
    uint16_t middle_left;   // 中采样行左边界。
    uint16_t middle_right;  // 中采样行右边界。
    uint16_t upper_left;    // 上采样行左边界。
    uint16_t upper_right;   // 上采样行右边界。
    uint16_t lower_width;   // 下采样行赛道宽度。
    uint16_t middle_width;  // 中采样行赛道宽度。
    uint16_t upper_width;   // 上采样行赛道宽度。
    int16_t lower_center;   // 下采样行中线中心。
    int16_t upper_center;   // 上采样行中线中心。

    lower_width = get_row_nonzero_width(image, CROSS_SAMPLE_ROW_LOWER, &lower_left, &lower_right);      // 统计图像下部宽度。
    middle_width = get_row_nonzero_width(image, CROSS_SAMPLE_ROW_MIDDLE, &middle_left, &middle_right);  // 统计图像中部宽度。
    upper_width = get_row_nonzero_width(image, CROSS_SAMPLE_ROW_UPPER, &upper_left, &upper_right);      // 统计图像上部宽度。

    g_cross_debug.lower_width = lower_width;    // 保存下采样行宽度供调试观察。
    g_cross_debug.middle_width = middle_width;  // 保存中采样行宽度供调试观察。
    g_cross_debug.upper_width = upper_width;    // 保存上采样行宽度供调试观察。

    lower_center = mid_line[CROSS_SAMPLE_ROW_LOWER];  // 读取下采样行中线列号。
    upper_center = mid_line[CROSS_SAMPLE_ROW_UPPER];  // 读取上采样行中线列号。

    if (lower_center < 0 || upper_center < 0)
    {
        return 0U;  // 上下中线无效时不认为是十字。
    }

    if (lower_width < CROSS_NORMAL_MIN_WIDTH || lower_width > CROSS_NORMAL_MAX_WIDTH)
    {
        return 0U;  // 下方不像正常直道入口时不认为是十字。
    }

    if (upper_width < CROSS_NORMAL_MIN_WIDTH || upper_width > CROSS_NORMAL_MAX_WIDTH)
    {
        return 0U;  // 上方不能重新收回到普通宽度时不认为是十字。
    }

    if (middle_width < CROSS_WIDE_MIN_WIDTH)
    {
        return 0U;  // 中部没有明显变宽时不认为是十字。
    }

    if (middle_width < (uint16_t)(lower_width + CROSS_WIDTH_EXPAND_THRESHOLD))
    {
        return 0U;  // 中部相对直道扩展不明显时不认为是十字。
    }

    if ((lower_center - upper_center > CROSS_CENTER_DIFF_LIMIT) ||
        (upper_center - lower_center > CROSS_CENTER_DIFF_LIMIT))
    {
        return 0U;  // 上下中心偏差过大时更像弯道，不像十字直行。
    }

    *guide_center = calculate_guide_center(mid_line);  // 根据图像下部中线更新直行引导中心。
    return 1U;  // 当前帧满足十字宽度模式。
}

/**
 * @brief 初始化十字元素状态机和调试信息
 */
void cross_element_init(void)
{
    g_cross_ctx.state = CROSS_STATE_NONE;                    // 初始状态设为普通巡线。
    g_cross_ctx.detect_count = 0U;                           // 清连续检测计数。
    g_cross_ctx.lost_count = 0U;                             // 清连续丢失计数。
    g_cross_ctx.exit_hold_count = 0U;                        // 清退出保持计数。
    g_cross_ctx.guide_center = (int16_t)(IMAGE_WIDTH / 2U);  // 初始引导中心放在图像中心。

    g_cross_debug.state = CROSS_STATE_NONE;                   // 同步复位调试状态。
    g_cross_debug.lower_width = 0U;                           // 清下部宽度调试值。
    g_cross_debug.middle_width = 0U;                          // 清中部宽度调试值。
    g_cross_debug.upper_width = 0U;                           // 清上部宽度调试值。
    g_cross_debug.guide_center = (int16_t)(IMAGE_WIDTH / 2U); // 清调试引导中心。
    g_cross_debug.pattern_detected = 0U;                      // 清当前模式识别结果。
}

/**
 * @brief 处理当前帧十字识别并输出专用控制结果
 * @param image 当前处理图像
 * @param mid_line 普通巡线模块输出的中线
 * @param override_mid_line 输出十字专用控制中线
 * @param override_speed 输出十字专用目标速度
 * @return 1 表示当前应启用十字专用控制，0 表示继续使用普通巡线控制
 */
uint8_t cross_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                              const int16_t mid_line[IMAGE_HEIGHT],
                              int16_t override_mid_line[IMAGE_HEIGHT],
                              float *override_speed)
{ 
    int16_t detected_center = g_cross_ctx.guide_center;  // 先沿用上一拍引导中心作为默认值。
    uint8_t pattern_detected = detect_cross_pattern(image, mid_line, &detected_center);  // 检查当前帧是否满足十字模式。

    g_cross_debug.pattern_detected = pattern_detected;  // 保存当前帧识别结果供调试查看。

    switch (g_cross_ctx.state)
    {
        case CROSS_STATE_NONE:
        {
            if (pattern_detected)
            {
                g_cross_ctx.detect_count++;                  // 连续命中十字模式时递增确认计数。
                g_cross_ctx.guide_center = detected_center;  // 命中时更新引导中心。

                if (g_cross_ctx.detect_count >= CROSS_DETECT_CONFIRM_FRAMES)
                {
                    g_cross_ctx.state = CROSS_STATE_ENTERING;  // 确认进入十字入口。
                    g_cross_ctx.lost_count = 0U;               // 清离开计数。
                    g_cross_ctx.exit_hold_count = 0U;          // 清退出保持计数。
                }
            }
            else
            {
                g_cross_ctx.detect_count = 0U;  // 模式中断时清确认计数。
            }
            break;
        }

        case CROSS_STATE_ENTERING:
        {
            if (pattern_detected)
            {
                g_cross_ctx.guide_center = detected_center;  // 入口阶段继续刷新引导中心。
                g_cross_ctx.state = CROSS_STATE_INSIDE;      // 确认后切到十字内部状态。
                g_cross_ctx.lost_count = 0U;                 // 清离开计数。
            }
            else
            {
                g_cross_ctx.state = CROSS_STATE_INSIDE;  // 入口阶段即使短暂抖动也按已经进入十字处理。
                g_cross_ctx.lost_count = 1U;             // 记录一次模式消失。
            }
            break;
        }

        case CROSS_STATE_INSIDE:
        {
            if (pattern_detected)
            {
                g_cross_ctx.guide_center = detected_center;  // 十字内部持续更新直行引导中心。
                g_cross_ctx.lost_count = 0U;                 // 只要模式仍在就清离开计数。
            }
            else
            {
                g_cross_ctx.lost_count++;  // 模式消失后累加离开计数。

                if (g_cross_ctx.lost_count >= CROSS_LOST_CONFIRM_FRAMES)
                {
                    g_cross_ctx.state = CROSS_STATE_EXITING;  // 确认离开十字主体。
                    g_cross_ctx.exit_hold_count = 0U;         // 开始退出保持计数。
                }
            }
            break;
        }

        case CROSS_STATE_EXITING:
        {
            g_cross_ctx.exit_hold_count++;  // 退出阶段继续保持短时间直行。

            if (g_cross_ctx.exit_hold_count >= CROSS_EXIT_HOLD_FRAMES)
            {
                g_cross_ctx.state = CROSS_STATE_NONE;  // 保持结束后恢复普通巡线。
                g_cross_ctx.detect_count = 0U;         // 清确认计数。
                g_cross_ctx.lost_count = 0U;           // 清离开计数。
            }
            break;
        }

        default:
        {
            g_cross_ctx.state = CROSS_STATE_NONE;  // 出现异常状态值时回到普通巡线。
            g_cross_ctx.detect_count = 0U;         // 清确认计数。
            g_cross_ctx.lost_count = 0U;           // 清离开计数。
            g_cross_ctx.exit_hold_count = 0U;      // 清退出保持计数。
            break;
        }
    }

    g_cross_debug.state = g_cross_ctx.state;                // 同步调试状态。
    g_cross_debug.guide_center = g_cross_ctx.guide_center;  // 同步调试引导中心。

    if (g_cross_ctx.state == CROSS_STATE_NONE)
    {
        return 0U;  // 普通巡线状态下不启用十字专用控制。
    }

    build_straight_override_mid_line(g_cross_ctx.guide_center, override_mid_line);  // 生成十字专用直行中线。
    *override_speed = CROSS_SPECIAL_TARGET_SPEED;  // 输出十字专用保守目标速度。
    return 1U;  // 通知上层启用十字专用控制。
}

/**
 * @brief 获取当前十字状态机状态
 * @return 当前十字状态枚举值
 */
Cross_State cross_element_get_state(void)
{
    return g_cross_ctx.state;  // 返回当前十字状态。
}

/**
 * @brief 获取当前十字调试信息
 * @return 当前帧对应的十字调试信息结构体
 */
Cross_Debug_Info cross_element_get_debug_info(void)
{
    return g_cross_debug;  // 返回当前十字调试信息。
}
