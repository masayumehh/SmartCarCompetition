#include "roundabout_element.h"

/**
 * 环岛识别思路：
 * 1. 先在图像下部建立普通直道参考；
 * 2. 自下向上寻找“单侧丢线、对侧稳定”的入口特征；
 * 3. 继续向上检查丢线侧是否重新出现并形成单峰弧线；
 * 4. 根据先丢线的一侧区分顺时针或逆时针环岛；
 * 5. 进入环岛状态后，根据补线距离构造一条覆盖中线交给底盘控制。
 */

#define ROUNDABOUT_LOWER_START_ROW         ((IMAGE_HEIGHT * TUNE_ROUNDABOUT_LOWER_START_RATIO_NUM) / TUNE_ROUNDABOUT_LOWER_START_RATIO_DEN)  // 环岛下部参考区域起始行。
#define ROUNDABOUT_NORMAL_MIN_WIDTH        TUNE_ROUNDABOUT_NORMAL_MIN_WIDTH        // 下部直道最小宽度阈值。
#define ROUNDABOUT_NORMAL_MAX_WIDTH        TUNE_ROUNDABOUT_NORMAL_MAX_WIDTH        // 下部直道最大宽度阈值。
#define ROUNDABOUT_REF_MIN_ROWS            TUNE_ROUNDABOUT_REF_MIN_ROWS            // 建立下部直道参考所需最少有效行数。
#define ROUNDABOUT_CENTER_DIFF_LIMIT       TUNE_ROUNDABOUT_CENTER_DIFF_LIMIT       // 下部中心允许最大波动量。
#define ROUNDABOUT_STABLE_EDGE_TOLERANCE   TUNE_ROUNDABOUT_STABLE_EDGE_TOLERANCE   // 稳定侧边线允许波动量。
#define ROUNDABOUT_MIN_MISSING_ROWS        TUNE_ROUNDABOUT_MIN_MISSING_ROWS        // 单侧至少连续丢线多少行。
#define ROUNDABOUT_MIN_RISING_ROWS         TUNE_ROUNDABOUT_MIN_RISING_ROWS         // 弧线发展阶段至少持续多少行。
#define ROUNDABOUT_MIN_FALLING_ROWS        TUNE_ROUNDABOUT_MIN_FALLING_ROWS        // 弧线回摆阶段至少持续多少行。
#define ROUNDABOUT_ARC_PEAK_MIN_OFFSET     TUNE_ROUNDABOUT_ARC_PEAK_MIN_OFFSET     // 弧顶相对原边线最小偏移量。
#define ROUNDABOUT_SUPPLEMENT_MIN_WIDTH    TUNE_ROUNDABOUT_SUPPLEMENT_MIN_WIDTH    // 补线距离最小阈值。
#define ROUNDABOUT_SUPPLEMENT_MAX_WIDTH    TUNE_ROUNDABOUT_SUPPLEMENT_MAX_WIDTH    // 补线距离最大阈值。
#define ROUNDABOUT_DETECT_CONFIRM_FRAMES   TUNE_ROUNDABOUT_DETECT_CONFIRM_FRAMES   // 连续多少帧命中才确认进入环岛。
#define ROUNDABOUT_LOST_CONFIRM_FRAMES     TUNE_ROUNDABOUT_LOST_CONFIRM_FRAMES     // 连续多少帧未命中才判定退出主体。
#define ROUNDABOUT_EXIT_HOLD_FRAMES        TUNE_ROUNDABOUT_EXIT_HOLD_FRAMES        // 退出后继续保持状态的帧数。
#define ROUNDABOUT_EDGE_JITTER_TOLERANCE   TUNE_ROUNDABOUT_EDGE_JITTER_TOLERANCE   // 判断弧线单调趋势时允许的小抖动。
#define ROUNDABOUT_SPECIAL_TARGET_SPEED    TUNE_ROUNDABOUT_SPECIAL_TARGET_SPEED    // 环岛专用目标速度。

/**
 * @brief 单帧环岛候选结果
 */
typedef struct
{
    uint8_t detected;                 // 当前方向是否命中候选模式。
    Roundabout_Direction direction;   // 当前候选方向。
    uint16_t lower_width;             // 下部直道参考宽度。
    uint16_t stable_edge;             // 稳定侧边线参考位置。
    uint16_t missing_rows;            // 连续丢线行数。
    int16_t arc_peak;                 // 重现弧线的弧顶位置。
    uint16_t supplement_width;        // 估算出的补线距离。
} Roundabout_Candidate;

/**
 * @brief 环岛状态机内部上下文
 */
typedef struct
{
    Roundabout_State state;          // 当前环岛状态。
    Roundabout_Direction direction;  // 当前环岛方向。
    uint8_t detect_count;            // 连续命中候选模式的帧数。
    uint8_t lost_count;              // 连续未命中候选模式的帧数。
    uint8_t exit_hold_count;         // 退出保持计数。
    uint16_t supplement_width;       // 最近一次确认得到的补线距离。
    int16_t guide_center;            // 最近一次确认得到的引导中心列。
} Roundabout_Context;

static Roundabout_Context g_roundabout_ctx =  // 环岛状态机内部上下文。
{
    ROUNDABOUT_STATE_NONE,
    ROUNDABOUT_DIRECTION_NONE,
    0U,
    0U,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U)
};

static Roundabout_Debug_Info g_roundabout_debug =  // 当前帧环岛调试信息快照。
{
    ROUNDABOUT_STATE_NONE,
    ROUNDABOUT_DIRECTION_NONE,
    0U,
    0U,
    0U,
    -1,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U),
    0U
};

/**
 * @brief 提取每一行的左右边界
 * @param image 当前处理图像
 * @param left_edges 输出左边界数组
 * @param right_edges 输出右边界数组
 * @param valid_rows 输出该行是否同时存在左右边界
 */
static void build_row_edges(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                            int16_t left_edges[IMAGE_HEIGHT],
                            int16_t right_edges[IMAGE_HEIGHT],
                            uint8_t valid_rows[IMAGE_HEIGHT])
{
    uint16_t y;  // 当前处理的行号。

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        uint16_t x;          // 当前扫描列号。
        uint8_t found = 0U;  // 当前行是否已经找到赛道区域。

        left_edges[y] = -1;   // 默认该行左边界无效。
        right_edges[y] = -1;  // 默认该行右边界无效。
        valid_rows[y] = 0U;   // 默认该行边界不完整。

        for (x = 0U; x < IMAGE_WIDTH; x++)
        {
            if (image[y][x] > 0U)
            {
                if (!found)
                {
                    left_edges[y] = (int16_t)x;  // 第一次命中时记录左边界。
                    found = 1U;                  // 标记当前行已经找到赛道区域。
                }

                right_edges[y] = (int16_t)x;  // 持续更新到最后一个非黑像素作为右边界。
            }
        }

        if (found && right_edges[y] > left_edges[y])
        {
            valid_rows[y] = 1U;  // 左右边界都有效时记为有效行。
        }
    }
}

/**
 * @brief 计算两个无符号数的绝对差
 * @param a 输入值 a
 * @param b 输入值 b
 * @return 绝对差
 */
static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);  // 返回两个输入值的无符号绝对差。
}

/**
 * @brief 根据图像下部中线估算环岛引导中心
 * @param mid_line 当前普通巡线中线
 * @return 环岛专用控制使用的引导中心列
 */
static int16_t calculate_guide_center(const int16_t mid_line[IMAGE_HEIGHT])
{
    uint16_t y;         // 当前统计的行号。
    int32_t sum = 0;    // 有效中线列号总和。
    uint16_t count = 0; // 有效中线点数量。

    for (y = ROUNDABOUT_LOWER_START_ROW; y < IMAGE_HEIGHT; y++)
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
 * @brief 建立图像下部直道参考
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param valid_rows 有效行标记数组
 * @param mid_line 普通巡线中线
 * @param left_ref 输出下部左边界参考值
 * @param right_ref 输出下部右边界参考值
 * @param width_ref 输出下部赛道宽度参考值
 * @return 1 表示参考有效，0 表示参考无效
 */
static uint8_t get_lower_reference(const int16_t left_edges[IMAGE_HEIGHT],
                                   const int16_t right_edges[IMAGE_HEIGHT],
                                   const uint8_t valid_rows[IMAGE_HEIGHT],
                                   const int16_t mid_line[IMAGE_HEIGHT],
                                   int16_t *left_ref,
                                   int16_t *right_ref,
                                   uint16_t *width_ref)
{
    uint16_t y;                 // 当前统计行号。
    int32_t left_sum = 0;       // 左边界累加值。
    int32_t right_sum = 0;      // 右边界累加值。
    int32_t width_sum = 0;      // 宽度累加值。
    uint16_t count = 0U;        // 有效参考行数。
    int16_t center_min = (int16_t)IMAGE_WIDTH;  // 下部中心最小值。
    int16_t center_max = -1;                  // 下部中心最大值。

    for (y = ROUNDABOUT_LOWER_START_ROW; y < IMAGE_HEIGHT; y++)
    {
        if (valid_rows[y])
        {
            int16_t center;  // 当前行中心列号。
            uint16_t width = (uint16_t)(right_edges[y] - left_edges[y] + 1);  // 当前行赛道宽度。

            left_sum += left_edges[y];  // 累加下部左边界。
            right_sum += right_edges[y];  // 累加下部右边界。
            width_sum += width;  // 累加下部赛道宽度。
            count++;  // 记录有效参考行数量。

            if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH)
            {
                center = mid_line[y];  // 优先使用已有中线结果。
            }
            else
            {
                center = (int16_t)((left_edges[y] + right_edges[y]) / 2);  // 中线无效时退回边界中点。
            }

            if (center < center_min) center_min = center;  // 更新中心最小值。
            if (center > center_max) center_max = center;  // 更新中心最大值。
        }
    }

    if (count < ROUNDABOUT_REF_MIN_ROWS)
    {
        return 0U;  // 下部有效参考行太少时，不建立环岛入口参考。
    }

    *left_ref = (int16_t)(left_sum / (int32_t)count);  // 计算下部左边界平均值。
    *right_ref = (int16_t)(right_sum / (int32_t)count);  // 计算下部右边界平均值。
    *width_ref = (uint16_t)(width_sum / (int32_t)count);  // 计算下部赛道平均宽度。

    if (*width_ref < ROUNDABOUT_NORMAL_MIN_WIDTH || *width_ref > ROUNDABOUT_NORMAL_MAX_WIDTH)
    {
        return 0U;  // 下部宽度不像普通直道时，不认为是环岛入口。
    }

    if (center_max < center_min)
    {
        return 0U;  // 没有有效中心信息时放弃识别。
    }

    if ((uint16_t)(center_max - center_min) > ROUNDABOUT_CENTER_DIFF_LIMIT)
    {
        return 0U;  // 下部中心偏移过大时更像普通弯道。
    }

    return 1U;  // 当前帧下部参考满足普通直道入口特征。
}

/**
 * @brief 识别“左侧先丢线”的环岛模式
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param valid_rows 有效行标记数组
 * @param left_ref 下部左边界参考值
 * @param right_ref 下部右边界参考值
 * @param width_ref 下部宽度参考值
 * @return 当前候选结果
 */
static Roundabout_Candidate detect_counterclockwise_candidate(const int16_t left_edges[IMAGE_HEIGHT],
                                                              const int16_t right_edges[IMAGE_HEIGHT],
                                                              const uint8_t valid_rows[IMAGE_HEIGHT],
                                                              int16_t left_ref,
                                                              int16_t right_ref,
                                                              uint16_t width_ref)
{
    Roundabout_Candidate candidate = {0U, ROUNDABOUT_DIRECTION_COUNTERCLOCKWISE, width_ref, 0U, 0U, -1, 0U};
    int16_t y;               // 自下向上扫描的行号。
    uint8_t phase = 0U;      // 0=寻找丢线，1=连续丢线，2=弧线发展，3=弧线回摆。
    uint16_t missing_rows = 0U;  // 连续丢线行数。
    uint16_t rising_rows = 0U;   // 弧线发展阶段持续行数。
    uint16_t falling_rows = 0U;  // 弧线回摆阶段持续行数。
    int16_t prev_edge = -1;      // 上一个有效弧线点位置。
    int16_t peak_edge = -1;      // 当前记录到的最大左边界，也就是弧顶。

    candidate.stable_edge = (uint16_t)right_ref;  // 当前稳定侧为右边线。

    for (y = (int16_t)(ROUNDABOUT_LOWER_START_ROW - 1U); y >= 0; y--)
    {
        uint8_t right_stable = 0U;  // 当前行右边线是否仍稳定。
        uint8_t left_present = 0U;  // 当前行左边线是否存在。

        if (right_edges[y] >= 0 &&
            abs_diff_u16((uint16_t)right_edges[y], (uint16_t)right_ref) <= ROUNDABOUT_STABLE_EDGE_TOLERANCE)
        {
            right_stable = 1U;  // 右边线仍在参考位置附近，认为稳定。
        }

        if (left_edges[y] >= 0 && valid_rows[y])
        {
            left_present = 1U;  // 当前行左边线存在且边界完整。
        }

        switch (phase)
        {
            case 0U:
            {
                if (!left_present && right_stable)
                {
                    phase = 1U;       // 找到左侧开始丢线的起点。
                    missing_rows = 1U;  // 记录第一行丢线。
                }
                break;
            }

            case 1U:
            {
                if (!left_present && right_stable)
                {
                    missing_rows++;  // 左侧继续丢线，右侧仍然稳定。
                }
                else if (left_present && right_stable)
                {
                    if (missing_rows >= ROUNDABOUT_MIN_MISSING_ROWS)
                    {
                        phase = 2U;              // 左侧弧线重新出现。
                        rising_rows = 1U;
                        prev_edge = left_edges[y];
                        peak_edge = left_edges[y];
                    }
                    else
                    {
                        phase = 0U;
                        missing_rows = 0U;
                    }
                }
                else
                {
                    phase = 0U;
                    missing_rows = 0U;
                }
                break;
            }

            case 2U:
            {
                if (!(left_present && right_stable))
                {
                    y = -1;
                    break;
                }

                if (left_edges[y] + (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE < prev_edge)
                {
                    phase = 3U;
                    falling_rows = 1U;
                    prev_edge = left_edges[y];
                }
                else
                {
                    if (left_edges[y] > peak_edge) peak_edge = left_edges[y];
                    prev_edge = left_edges[y];
                    rising_rows++;
                }
                break;
            }

            default:
            {
                if (!(left_present && right_stable))
                {
                    y = -1;
                    break;
                }

                if (left_edges[y] <= prev_edge + (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE)
                {
                    if (left_edges[y] < prev_edge) falling_rows++;
                    prev_edge = left_edges[y];
                }
                else
                {
                    y = -1;
                }
                break;
            }
        }
    }

    if (missing_rows < ROUNDABOUT_MIN_MISSING_ROWS ||
        rising_rows < ROUNDABOUT_MIN_RISING_ROWS ||
        falling_rows < ROUNDABOUT_MIN_FALLING_ROWS)
    {
        return candidate;
    }

    if (peak_edge < 0 || peak_edge - left_ref < ROUNDABOUT_ARC_PEAK_MIN_OFFSET)
    {
        return candidate;
    }

    if (right_ref <= peak_edge)
    {
        return candidate;
    }

    candidate.missing_rows = missing_rows;
    candidate.arc_peak = peak_edge;
    candidate.supplement_width = (uint16_t)(right_ref - peak_edge);

    if (candidate.supplement_width < ROUNDABOUT_SUPPLEMENT_MIN_WIDTH ||
        candidate.supplement_width > ROUNDABOUT_SUPPLEMENT_MAX_WIDTH)
    {
        return candidate;
    }

    candidate.detected = 1U;
    return candidate;
}

/**
 * @brief 识别“右侧先丢线”的环岛模式
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param valid_rows 有效行标记数组
 * @param left_ref 下部左边界参考值
 * @param right_ref 下部右边界参考值
 * @param width_ref 下部宽度参考值
 * @return 当前候选结果
 */
static Roundabout_Candidate detect_clockwise_candidate(const int16_t left_edges[IMAGE_HEIGHT],
                                                       const int16_t right_edges[IMAGE_HEIGHT],
                                                       const uint8_t valid_rows[IMAGE_HEIGHT],
                                                       int16_t left_ref,
                                                       int16_t right_ref,
                                                       uint16_t width_ref)
{
    Roundabout_Candidate candidate = {0U, ROUNDABOUT_DIRECTION_CLOCKWISE, width_ref, 0U, 0U, -1, 0U};
    int16_t y;
    uint8_t phase = 0U;
    uint16_t missing_rows = 0U;
    uint16_t rising_rows = 0U;
    uint16_t falling_rows = 0U;
    int16_t prev_edge = -1;
    int16_t peak_edge = -1;

    candidate.stable_edge = (uint16_t)left_ref;

    for (y = (int16_t)(ROUNDABOUT_LOWER_START_ROW - 1U); y >= 0; y--)
    {
        uint8_t left_stable = 0U;
        uint8_t right_present = 0U;

        if (left_edges[y] >= 0 &&
            abs_diff_u16((uint16_t)left_edges[y], (uint16_t)left_ref) <= ROUNDABOUT_STABLE_EDGE_TOLERANCE)
        {
            left_stable = 1U;
        }

        if (right_edges[y] >= 0 && valid_rows[y])
        {
            right_present = 1U;
        }

        switch (phase)
        {
            case 0U:
            {
                if (!right_present && left_stable)
                {
                    phase = 1U;
                    missing_rows = 1U;
                }
                break;
            }

            case 1U:
            {
                if (!right_present && left_stable)
                {
                    missing_rows++;
                }
                else if (right_present && left_stable)
                {
                    if (missing_rows >= ROUNDABOUT_MIN_MISSING_ROWS)
                    {
                        phase = 2U;
                        rising_rows = 1U;
                        prev_edge = right_edges[y];
                        peak_edge = right_edges[y];
                    }
                    else
                    {
                        phase = 0U;
                        missing_rows = 0U;
                    }
                }
                else
                {
                    phase = 0U;
                    missing_rows = 0U;
                }
                break;
            }

            case 2U:
            {
                if (!(right_present && left_stable))
                {
                    y = -1;
                    break;
                }

                if (right_edges[y] > prev_edge + (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE)
                {
                    phase = 3U;
                    falling_rows = 1U;
                    prev_edge = right_edges[y];
                }
                else
                {
                    if (peak_edge < 0 || right_edges[y] < peak_edge) peak_edge = right_edges[y];
                    prev_edge = right_edges[y];
                    rising_rows++;
                }
                break;
            }

            default:
            {
                if (!(right_present && left_stable))
                {
                    y = -1;
                    break;
                }

                if (right_edges[y] + (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE >= prev_edge)
                {
                    if (right_edges[y] > prev_edge) falling_rows++;
                    prev_edge = right_edges[y];
                }
                else
                {
                    y = -1;
                }
                break;
            }
        }
    }

    if (missing_rows < ROUNDABOUT_MIN_MISSING_ROWS ||
        rising_rows < ROUNDABOUT_MIN_RISING_ROWS ||
        falling_rows < ROUNDABOUT_MIN_FALLING_ROWS)
    {
        return candidate;
    }

    if (peak_edge < 0 || right_ref - peak_edge < ROUNDABOUT_ARC_PEAK_MIN_OFFSET)
    {
        return candidate;
    }

    if (peak_edge <= left_ref)
    {
        return candidate;
    }

    candidate.missing_rows = missing_rows;
    candidate.arc_peak = peak_edge;
    candidate.supplement_width = (uint16_t)(peak_edge - left_ref);

    if (candidate.supplement_width < ROUNDABOUT_SUPPLEMENT_MIN_WIDTH ||
        candidate.supplement_width > ROUNDABOUT_SUPPLEMENT_MAX_WIDTH)
    {
        return candidate;
    }

    candidate.detected = 1U;
    return candidate;
}

/**
 * @brief 检测当前帧最可信的环岛候选结果
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param valid_rows 有效行标记数组
 * @param mid_line 当前普通巡线中线
 * @return 当前帧最可信的候选结果
 */
static Roundabout_Candidate detect_roundabout_candidate(const int16_t left_edges[IMAGE_HEIGHT],
                                                        const int16_t right_edges[IMAGE_HEIGHT],
                                                        const uint8_t valid_rows[IMAGE_HEIGHT],
                                                        const int16_t mid_line[IMAGE_HEIGHT])
{
    int16_t left_ref;
    int16_t right_ref;
    uint16_t width_ref;
    Roundabout_Candidate ccw_candidate;
    Roundabout_Candidate cw_candidate;
    Roundabout_Candidate empty_candidate = {0U, ROUNDABOUT_DIRECTION_NONE, 0U, 0U, 0U, -1, 0U};

    if (!get_lower_reference(left_edges, right_edges, valid_rows, mid_line, &left_ref, &right_ref, &width_ref))
    {
        return empty_candidate;
    }

    ccw_candidate = detect_counterclockwise_candidate(left_edges, right_edges, valid_rows, left_ref, right_ref, width_ref);
    cw_candidate = detect_clockwise_candidate(left_edges, right_edges, valid_rows, left_ref, right_ref, width_ref);

    if (ccw_candidate.detected && cw_candidate.detected)
    {
        if (ccw_candidate.missing_rows >= cw_candidate.missing_rows)
        {
            return ccw_candidate;
        }

        return cw_candidate;
    }

    if (ccw_candidate.detected) return ccw_candidate;
    if (cw_candidate.detected) return cw_candidate;

    empty_candidate.lower_width = width_ref;
    return empty_candidate;
}

/**
 * @brief 构造环岛专用覆盖中线
 * @param left_edges 当前帧左边界数组
 * @param right_edges 当前帧右边界数组
 * @param valid_rows 当前帧有效行标记数组
 * @param mid_line 当前普通巡线中线
 * @param direction 当前环岛方向
 * @param supplement_width 当前补线距离
 * @param guide_center 当前引导中心列
 * @param override_mid_line 输出覆盖中线
 */
static void build_roundabout_override_mid_line(const int16_t left_edges[IMAGE_HEIGHT],
                                               const int16_t right_edges[IMAGE_HEIGHT],
                                               const uint8_t valid_rows[IMAGE_HEIGHT],
                                               const int16_t mid_line[IMAGE_HEIGHT],
                                               Roundabout_Direction direction,
                                               uint16_t supplement_width,
                                               int16_t guide_center,
                                               int16_t override_mid_line[IMAGE_HEIGHT])
{
    uint16_t y;

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        if (y >= ROUNDABOUT_LOWER_START_ROW && mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH)
        {
            override_mid_line[y] = mid_line[y];
            continue;
        }

        if (direction == ROUNDABOUT_DIRECTION_COUNTERCLOCKWISE)
        {
            if (left_edges[y] >= 0)
            {
                int16_t synthetic_right = (int16_t)(left_edges[y] + (int16_t)supplement_width);
                if (synthetic_right >= (int16_t)IMAGE_WIDTH) synthetic_right = (int16_t)(IMAGE_WIDTH - 1);
                override_mid_line[y] = (int16_t)((left_edges[y] + synthetic_right) / 2);
            }
            else if (right_edges[y] >= 0 && valid_rows[y])
            {
                int16_t synthetic_left = (int16_t)(right_edges[y] - (int16_t)supplement_width);
                if (synthetic_left < 0) synthetic_left = 0;
                override_mid_line[y] = (int16_t)((synthetic_left + right_edges[y]) / 2);
            }
            else
            {
                override_mid_line[y] = guide_center;
            }
        }
        else if (direction == ROUNDABOUT_DIRECTION_CLOCKWISE)
        {
            if (right_edges[y] >= 0)
            {
                int16_t synthetic_left = (int16_t)(right_edges[y] - (int16_t)supplement_width);
                if (synthetic_left < 0) synthetic_left = 0;
                override_mid_line[y] = (int16_t)((synthetic_left + right_edges[y]) / 2);
            }
            else if (left_edges[y] >= 0 && valid_rows[y])
            {
                int16_t synthetic_right = (int16_t)(left_edges[y] + (int16_t)supplement_width);
                if (synthetic_right >= (int16_t)IMAGE_WIDTH) synthetic_right = (int16_t)(IMAGE_WIDTH - 1);
                override_mid_line[y] = (int16_t)((left_edges[y] + synthetic_right) / 2);
            }
            else
            {
                override_mid_line[y] = guide_center;
            }
        }
        else
        {
            override_mid_line[y] = guide_center;
        }
    }
}

/**
 * @brief 初始化环岛元素状态机
 */
void roundabout_element_init(void)
{
    g_roundabout_ctx.state = ROUNDABOUT_STATE_NONE;
    g_roundabout_ctx.direction = ROUNDABOUT_DIRECTION_NONE;
    g_roundabout_ctx.detect_count = 0U;
    g_roundabout_ctx.lost_count = 0U;
    g_roundabout_ctx.exit_hold_count = 0U;
    g_roundabout_ctx.supplement_width = 0U;
    g_roundabout_ctx.guide_center = (int16_t)(IMAGE_WIDTH / 2U);

    g_roundabout_debug.state = ROUNDABOUT_STATE_NONE;
    g_roundabout_debug.direction = ROUNDABOUT_DIRECTION_NONE;
    g_roundabout_debug.lower_width = 0U;
    g_roundabout_debug.stable_edge = 0U;
    g_roundabout_debug.missing_rows = 0U;
    g_roundabout_debug.arc_peak = -1;
    g_roundabout_debug.supplement_width = 0U;
    g_roundabout_debug.guide_center = (int16_t)(IMAGE_WIDTH / 2U);
    g_roundabout_debug.candidate_detected = 0U;
}

/**
 * @brief 处理当前帧环岛识别并输出专用控制结果
 * @param image 当前处理图像
 * @param mid_line 当前普通巡线中线
 * @param override_mid_line 输出环岛专用覆盖中线
 * @param override_speed 输出环岛专用目标速度
 * @return 1 表示当前应启用环岛专用控制，0 表示继续使用普通巡线控制
 */
uint8_t roundabout_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT],
                                   int16_t override_mid_line[IMAGE_HEIGHT],
                                   float *override_speed)
{
    int16_t left_edges[IMAGE_HEIGHT];
    int16_t right_edges[IMAGE_HEIGHT];
    uint8_t valid_rows[IMAGE_HEIGHT];
    int16_t detected_center = g_roundabout_ctx.guide_center;
    Roundabout_Candidate candidate;

    build_row_edges(image, left_edges, right_edges, valid_rows);
    detected_center = calculate_guide_center(mid_line);
    candidate = detect_roundabout_candidate(left_edges, right_edges, valid_rows, mid_line);

    g_roundabout_debug.candidate_detected = candidate.detected;
    g_roundabout_debug.lower_width = candidate.lower_width;
    g_roundabout_debug.stable_edge = candidate.stable_edge;
    g_roundabout_debug.missing_rows = candidate.missing_rows;
    g_roundabout_debug.arc_peak = candidate.arc_peak;
    g_roundabout_debug.supplement_width = candidate.supplement_width;

    switch (g_roundabout_ctx.state)
    {
        case ROUNDABOUT_STATE_NONE:
        {
            if (candidate.detected)
            {
                if (g_roundabout_ctx.direction != candidate.direction)
                {
                    g_roundabout_ctx.detect_count = 0U;
                }

                g_roundabout_ctx.direction = candidate.direction;
                g_roundabout_ctx.detect_count++;
                g_roundabout_ctx.supplement_width = candidate.supplement_width;
                g_roundabout_ctx.guide_center = detected_center;

                if (g_roundabout_ctx.detect_count >= ROUNDABOUT_DETECT_CONFIRM_FRAMES)
                {
                    g_roundabout_ctx.state = ROUNDABOUT_STATE_ENTERING;
                    g_roundabout_ctx.lost_count = 0U;
                    g_roundabout_ctx.exit_hold_count = 0U;
                }
            }
            else
            {
                g_roundabout_ctx.detect_count = 0U;
                g_roundabout_ctx.direction = ROUNDABOUT_DIRECTION_NONE;
            }
            break;
        }

        case ROUNDABOUT_STATE_ENTERING:
        {
            if (candidate.detected && candidate.direction == g_roundabout_ctx.direction)
            {
                g_roundabout_ctx.state = ROUNDABOUT_STATE_INSIDE;
                g_roundabout_ctx.lost_count = 0U;
                g_roundabout_ctx.supplement_width = candidate.supplement_width;
                g_roundabout_ctx.guide_center = detected_center;
            }
            else
            {
                g_roundabout_ctx.state = ROUNDABOUT_STATE_INSIDE;
                g_roundabout_ctx.lost_count = 1U;
            }
            break;
        }

        case ROUNDABOUT_STATE_INSIDE:
        {
            if (candidate.detected && candidate.direction == g_roundabout_ctx.direction)
            {
                g_roundabout_ctx.lost_count = 0U;
                g_roundabout_ctx.supplement_width = candidate.supplement_width;
                g_roundabout_ctx.guide_center = detected_center;
            }
            else
            {
                g_roundabout_ctx.lost_count++;

                if (g_roundabout_ctx.lost_count >= ROUNDABOUT_LOST_CONFIRM_FRAMES)
                {
                    g_roundabout_ctx.state = ROUNDABOUT_STATE_EXITING;
                    g_roundabout_ctx.exit_hold_count = 0U;
                }
            }
            break;
        }

        case ROUNDABOUT_STATE_EXITING:
        {
            g_roundabout_ctx.exit_hold_count++;

            if (g_roundabout_ctx.exit_hold_count >= ROUNDABOUT_EXIT_HOLD_FRAMES)
            {
                g_roundabout_ctx.state = ROUNDABOUT_STATE_NONE;
                g_roundabout_ctx.direction = ROUNDABOUT_DIRECTION_NONE;
                g_roundabout_ctx.detect_count = 0U;
                g_roundabout_ctx.lost_count = 0U;
                g_roundabout_ctx.supplement_width = 0U;
            }
            break;
        }

        default:
        {
            g_roundabout_ctx.state = ROUNDABOUT_STATE_NONE;
            g_roundabout_ctx.direction = ROUNDABOUT_DIRECTION_NONE;
            g_roundabout_ctx.detect_count = 0U;
            g_roundabout_ctx.lost_count = 0U;
            g_roundabout_ctx.exit_hold_count = 0U;
            g_roundabout_ctx.supplement_width = 0U;
            break;
        }
    }

    g_roundabout_debug.state = g_roundabout_ctx.state;
    g_roundabout_debug.direction = g_roundabout_ctx.direction;
    g_roundabout_debug.guide_center = g_roundabout_ctx.guide_center;
    if (g_roundabout_ctx.supplement_width > 0U)
    {
        g_roundabout_debug.supplement_width = g_roundabout_ctx.supplement_width;
    }

    if (g_roundabout_ctx.state == ROUNDABOUT_STATE_NONE)
    {
        return 0U;
    }

    build_roundabout_override_mid_line(left_edges,
                                       right_edges,
                                       valid_rows,
                                       mid_line,
                                       g_roundabout_ctx.direction,
                                       g_roundabout_ctx.supplement_width,
                                       g_roundabout_ctx.guide_center,
                                       override_mid_line);
    *override_speed = ROUNDABOUT_SPECIAL_TARGET_SPEED;
    return 1U;
}

/**
 * @brief 获取当前环岛状态
 * @return 当前环岛状态枚举值
 */
Roundabout_State roundabout_element_get_state(void)
{
    return g_roundabout_ctx.state;
}

/**
 * @brief 获取当前环岛方向
 * @return 当前环岛方向枚举值
 */
Roundabout_Direction roundabout_element_get_direction(void)
{
    return g_roundabout_ctx.direction;
}

/**
 * @brief 获取当前环岛调试信息
 * @return 当前帧对应的环岛调试信息结构体
 */
Roundabout_Debug_Info roundabout_element_get_debug_info(void)
{
    return g_roundabout_debug;
}
