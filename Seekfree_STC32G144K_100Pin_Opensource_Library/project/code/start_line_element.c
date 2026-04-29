#include "start_line_element.h"

/**
 * @file start_line_element.c
 * @brief 发车线识别与单圈停车模块
 * @details
 * 赛道中的发车线由两条横向黑胶带组成，中间留有一段白色间隔。
 * 对二值化图像来说，小车从下往上看时，发车线会在赛道中心附近呈现出：
 * “黑带 -> 白间隔 -> 黑带”的纵向条纹结构。
 *
 * 本模块的核心思路如下：
 * 1. 先利用普通循线模块给出的中线，估算当前图像中赛道中心所在列号；
 * 2. 只在赛道中心附近的小窗口内做纵向扫描，不进行整幅模板匹配，
 *    这样可以降低弯道透视、噪声和边缘元素带来的干扰；
 * 3. 扫描时先检查黑带前方是否存在足够长的正常直道，
 *    用于避免把十字、环岛入口、障碍物阴影等误当成发车线；
 * 4. 状态机先等待车辆真正驶离发车区，再在后续帧中重新识别发车线；
 * 5. 一旦确认重新回到发车点，立即保持“完赛”状态，由上层锁定停车。
 */

#define START_LINE_TRACK_MIN_WIDTH         TUNE_START_LINE_TRACK_MIN_WIDTH
#define START_LINE_TRACK_MAX_WIDTH         TUNE_START_LINE_TRACK_MAX_WIDTH
#define START_LINE_CENTER_DIFF_LIMIT       TUNE_START_LINE_CENTER_DIFF_LIMIT
#define START_LINE_SAMPLE_HALF_WIDTH       TUNE_START_LINE_SAMPLE_HALF_WIDTH
#define START_LINE_DARK_PIXEL_MIN_COUNT    TUNE_START_LINE_DARK_PIXEL_MIN_COUNT
#define START_LINE_BAND_MIN_ROWS           TUNE_START_LINE_BAND_MIN_ROWS
#define START_LINE_BAND_MAX_ROWS           TUNE_START_LINE_BAND_MAX_ROWS
#define START_LINE_GAP_MIN_ROWS            TUNE_START_LINE_GAP_MIN_ROWS
#define START_LINE_GAP_MAX_ROWS            TUNE_START_LINE_GAP_MAX_ROWS
#define START_LINE_BAND_MAX_DIFF_ROWS      TUNE_START_LINE_BAND_MAX_DIFF_ROWS
#define START_LINE_APPROACH_MIN_ROWS       TUNE_START_LINE_APPROACH_MIN_ROWS
#define START_LINE_ARM_CLEAR_FRAMES        TUNE_START_LINE_ARM_CLEAR_FRAMES
#define START_LINE_DETECT_CONFIRM_FRAMES   TUNE_START_LINE_DETECT_CONFIRM_FRAMES

/**
 * @brief 单帧发车线候选结果
 * @details
 * 该结构体只描述“当前这一帧看起来像不像发车线”，
 * 不负责保存跨帧状态，也不直接决定是否停车。
 * 真正的完赛判定由后续状态机结合连续多帧确认来完成。
 */
typedef struct
{
    uint8_t detected;         // 当前帧是否满足双黑带发车线模式。
    uint16_t near_band_rows;  // 靠近车辆一侧黑带的厚度。
    uint16_t gap_rows;        // 两条黑带之间白色间隔的厚度。
    uint16_t far_band_rows;   // 远离车辆一侧黑带的厚度。
    uint16_t approach_rows;   // 第一条黑带前方有效直道的连续行数。
    int16_t guide_center;     // 本帧识别时采用的赛道中心列号。
} Start_Line_Candidate;

/**
 * @brief 发车线识别状态机上下文
 * @details
 * state
 * 当前状态机状态。
 *
 * clear_count
 * 在 WAITING_CLEAR 状态下，连续多少帧已经看不到发车线。
 * 只有达到阈值后，才认为车辆确实离开发车区。
 *
 * detect_count
 * 在 ARMED 状态下，连续多少帧再次稳定识别到发车线。
 * 只有达到阈值后，才认为车辆已经跑完一圈。
 *
 * guide_center
 * 最近一次有效识别所使用的赛道中心，用于调试观察。
 */
typedef struct
{
    Start_Line_State state;
    uint8_t clear_count;
    uint8_t detect_count;
    int16_t guide_center;
} Start_Line_Context;

static Start_Line_Context g_start_line_ctx =  // 全局发车线状态机上下文，负责跨帧记住“是否已经离开发车区”。
{
    START_LINE_STATE_WAITING_CLEAR,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U)
};

static Start_Line_Debug_Info g_start_line_debug =  // 当前帧发车线调试快照，供完赛判定调试使用。
{
    START_LINE_STATE_WAITING_CLEAR,
    0U,
    0U,
    0U,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U)
};

/**
 * @brief 计算两个无符号整数的绝对差
 * @param a 输入值 A
 * @param b 输入值 B
 * @return 两者差值的绝对值
 */
static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

/**
 * @brief 统计某一行白色赛道区域的左右边界与宽度
 * @param image 当前二值图像
 * @param row 当前行号
 * @param left 输出左边界列号
 * @param right 输出右边界列号
 * @return 当前行白色赛道宽度；若本行没有找到白色赛道则返回 0
 * @details
 * 发车线识别前，需要先确认该区域整体仍然像“正常直道”。
 * 因此这里先扫描整行，找到白色赛道的大致左右边界与宽度。
 * 后续会结合宽度范围和中心偏差，判断这行是否具备直道特征。
 */
static uint16_t get_row_nonzero_width(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                      uint16_t row,
                                      uint16_t *left,
                                      uint16_t *right)
{
    uint16_t x;
    uint8_t found;

    found = 0U;
    *left = 0U;
    *right = 0U;

    for (x = 0U; x < IMAGE_WIDTH; x++)
    {
        if (image[row][x] > 0U)
        {
            if (!found)
            {
                *left = x;  // 第一个白点视为左边界。
                found = 1U;
            }

            *right = x;  // 最后一个白点视为右边界。
        }
    }

    if (!found)
    {
        return 0U;
    }

    return (uint16_t)(*right - *left + 1U);
}

/**
 * @brief 根据普通循线中线估算当前赛道中心
 * @param mid_line 普通循线模块输出的中线数组
 * @return 当前帧发车线识别采用的引导中心列号
 * @details
 * 这里只统计图像下部的有效中线点，是因为下部区域更接近车辆当前位置，
 * 受到远处透视和复杂元素的影响更小，更适合用来约束发车线识别窗口。
 * 如果当前帧中线整体无效，则退回图像中心作为保底值。
 */
static int16_t calculate_guide_center(const int16_t mid_line[IMAGE_HEIGHT])
{
    uint16_t y;
    int32_t sum;
    uint16_t count;

    sum = 0;
    count = 0U;

    for (y = (uint16_t)(IMAGE_HEIGHT * 2U / 3U); y < IMAGE_HEIGHT; y++)
    {
        if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH)
        {
            sum += mid_line[y];
            count++;
        }
    }

    if (count == 0U)
    {
        return (int16_t)(IMAGE_WIDTH / 2U);
    }

    return (int16_t)(sum / (int32_t)count);
}

/**
 * @brief 判断当前行在赛道中心附近是否更像黑带
 * @param image 当前二值图像
 * @param row 当前行号
 * @param guide_center 当前赛道中心列号
 * @return 1 表示该行中心附近更像黑带，0 表示更像白色赛道
 * @details
 * 发车线本质上是横向黑胶带。当车辆压到发车线附近时，
 * 赛道中心附近会出现连续黑像素堆积。
 * 这里不查看整行，只在赛道中心附近开一个小窗口进行统计，
 * 以提升对弯道、边缘缺损和局部噪声的鲁棒性。
 */
static uint8_t row_center_is_dark(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                  uint16_t row,
                                  int16_t guide_center)
{
    int16_t x_start;
    int16_t x_end;
    int16_t x;
    uint16_t black_count;

    x_start = (int16_t)(guide_center - (int16_t)START_LINE_SAMPLE_HALF_WIDTH);
    x_end = (int16_t)(guide_center + (int16_t)START_LINE_SAMPLE_HALF_WIDTH);
    black_count = 0U;

    if (x_start < 0)
    {
        x_start = 0;
    }
    if (x_end >= (int16_t)IMAGE_WIDTH)
    {
        x_end = (int16_t)(IMAGE_WIDTH - 1);
    }

    for (x = x_start; x <= x_end; x++)
    {
        if (image[row][x] == 0U)
        {
            black_count++;
        }
    }

    return (black_count >= START_LINE_DARK_PIXEL_MIN_COUNT) ? 1U : 0U;
}

/**
 * @brief 判断当前行是否仍然满足“正常直道”特征
 * @param image 当前二值图像
 * @param mid_line 普通循线模块输出的中线数组
 * @param row 当前行号
 * @param guide_center 当前赛道中心列号
 * @return 1 表示该行像正常直道，0 表示不像正常直道
 * @details
 * 为了避免把其它元素误识别成发车线，这里要求黑带前方必须先有一段
 * 足够稳定的正常直道。判断标准包括：
 * 1. 当前行赛道宽度在合理范围内；
 * 2. 当前行中心位置与估算赛道中心偏差不能过大；
 * 3. 当前行中心附近不能已经被黑带覆盖。
 */
static uint8_t row_looks_straight(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                  const int16_t mid_line[IMAGE_HEIGHT],
                                  uint16_t row,
                                  int16_t guide_center)
{
    uint16_t left;
    uint16_t right;
    uint16_t width;
    int16_t row_center;

    width = get_row_nonzero_width(image, row, &left, &right);
    if (width < START_LINE_TRACK_MIN_WIDTH || width > START_LINE_TRACK_MAX_WIDTH)
    {
        return 0U;
    }

    if (mid_line[row] >= 0 && mid_line[row] < IMAGE_WIDTH)
    {
        row_center = mid_line[row];  // 优先采用已有中线结果。
    }
    else
    {
        row_center = (int16_t)((left + right) / 2U);  // 中线失效时退回左右边界中点。
    }

    if (abs_diff_u16((uint16_t)row_center, (uint16_t)guide_center) > START_LINE_CENTER_DIFF_LIMIT)
    {
        return 0U;
    }

    return row_center_is_dark(image, row, guide_center) ? 0U : 1U;
}

/**
 * @brief 在当前帧中检测发车线双黑带模式
 * @param image 当前二值图像
 * @param mid_line 普通循线模块输出的中线数组
 * @return 当前帧对应的发车线候选结果
 * @details
 * 扫描步骤如下：
 * 1. 从图像底部往上扫描，先统计第一条黑带之前连续正常直道的行数；
 * 2. 第一次遇到中心附近明显变黑时，统计靠近车辆一侧黑带厚度；
 * 3. 黑带结束后继续统计中间白色间隔厚度；
 * 4. 再继续统计远离车辆一侧黑带厚度；
 * 5. 最后结合厚度范围、双黑带一致性和前方直道长度做综合过滤。
 */
static Start_Line_Candidate detect_start_line_pattern(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                                      const int16_t mid_line[IMAGE_HEIGHT])
{
    Start_Line_Candidate candidate;
    int16_t y;

    candidate.detected = 0U;
    candidate.near_band_rows = 0U;
    candidate.gap_rows = 0U;
    candidate.far_band_rows = 0U;
    candidate.approach_rows = 0U;
    candidate.guide_center = calculate_guide_center(mid_line);

    y = (int16_t)(IMAGE_HEIGHT - 1U);
    while (y >= 0 && !row_center_is_dark(image, (uint16_t)y, candidate.guide_center))
    {
        if (row_looks_straight(image, mid_line, (uint16_t)y, candidate.guide_center))
        {
            candidate.approach_rows++;  // 黑带前方仍然像正常直道。
        }
        y--;
    }

    if (y < 0)
    {
        return candidate;  // 整帧都没找到中心黑带，说明本帧不像发车线。
    }

    while (y >= 0 && row_center_is_dark(image, (uint16_t)y, candidate.guide_center))
    {
        candidate.near_band_rows++;
        y--;
    }

    while (y >= 0 && !row_center_is_dark(image, (uint16_t)y, candidate.guide_center))
    {
        candidate.gap_rows++;
        y--;
    }

    while (y >= 0 && row_center_is_dark(image, (uint16_t)y, candidate.guide_center))
    {
        candidate.far_band_rows++;
        y--;
    }

    if (candidate.approach_rows < START_LINE_APPROACH_MIN_ROWS)
    {
        return candidate;
    }

    if (candidate.near_band_rows < START_LINE_BAND_MIN_ROWS || candidate.near_band_rows > START_LINE_BAND_MAX_ROWS)
    {
        return candidate;
    }

    if (candidate.gap_rows < START_LINE_GAP_MIN_ROWS || candidate.gap_rows > START_LINE_GAP_MAX_ROWS)
    {
        return candidate;
    }

    if (candidate.far_band_rows < START_LINE_BAND_MIN_ROWS || candidate.far_band_rows > START_LINE_BAND_MAX_ROWS)
    {
        return candidate;
    }

    if (abs_diff_u16(candidate.near_band_rows, candidate.far_band_rows) > START_LINE_BAND_MAX_DIFF_ROWS)
    {
        return candidate;
    }

    candidate.detected = 1U;
    return candidate;
}

/**
 * @brief 初始化发车线识别状态机与调试缓存
 * @details
 * 每次正式发车前都应调用本函数，使模块重新回到“等待驶离发车区”的初始状态。
 * 这样可以避免上一轮运行留下的状态影响下一轮发车。
 */
void start_line_element_init(void)
{
    g_start_line_ctx.state = START_LINE_STATE_WAITING_CLEAR;
    g_start_line_ctx.clear_count = 0U;
    g_start_line_ctx.detect_count = 0U;
    g_start_line_ctx.guide_center = (int16_t)(IMAGE_WIDTH / 2U);

    g_start_line_debug.state = START_LINE_STATE_WAITING_CLEAR;
    g_start_line_debug.pattern_detected = 0U;
    g_start_line_debug.near_band_rows = 0U;
    g_start_line_debug.gap_rows = 0U;
    g_start_line_debug.far_band_rows = 0U;
    g_start_line_debug.approach_rows = 0U;
    g_start_line_debug.guide_center = (int16_t)(IMAGE_WIDTH / 2U);
}

/**
 * @brief 直接把发车线识别状态机切到布防态
 * @details
 * 适用于上层已经通过时间或里程逻辑确认车辆离开发车区，
 * 不再需要经过 WAITING_CLEAR 逐帧清空过程的场景。
 */
void start_line_element_arm(void)
{
    g_start_line_ctx.state = START_LINE_STATE_ARMED;
    g_start_line_ctx.clear_count = 0U;
    g_start_line_ctx.detect_count = 0U;
    g_start_line_ctx.guide_center = (int16_t)(IMAGE_WIDTH / 2U);

    g_start_line_debug.state = START_LINE_STATE_ARMED;
    g_start_line_debug.pattern_detected = 0U;
    g_start_line_debug.near_band_rows = 0U;
    g_start_line_debug.gap_rows = 0U;
    g_start_line_debug.far_band_rows = 0U;
    g_start_line_debug.approach_rows = 0U;
    g_start_line_debug.guide_center = (int16_t)(IMAGE_WIDTH / 2U);
}

/**
 * @brief 处理当前帧发车线识别并判断是否已经完成一圈
 * @param image 当前二值图像
 * @param mid_line 普通循线模块输出的中线数组
 * @return 1 表示应停车，0 表示继续正常运行
 * @details
 * 状态流转说明如下：
 * 1. WAITING_CLEAR
 *    刚发车时先忽略脚下的发车线，只有连续多帧都看不到发车线，
 *    才认为车辆真正离开发车区并转入 ARMED 状态。
 * 2. ARMED
 *    车辆已经驶离起点区域，此时如果连续多帧再次稳定识别到发车线，
 *    就判定车辆已经回到发车点，完成一圈。
 * 3. FINISHED
 *    完赛状态保持锁定，持续返回 1，交给上层维持停车。
 */
uint8_t start_line_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT])
{
    Start_Line_Candidate candidate;

    candidate = detect_start_line_pattern(image, mid_line);

    g_start_line_debug.pattern_detected = candidate.detected;
    g_start_line_debug.near_band_rows = candidate.near_band_rows;
    g_start_line_debug.gap_rows = candidate.gap_rows;
    g_start_line_debug.far_band_rows = candidate.far_band_rows;
    g_start_line_debug.approach_rows = candidate.approach_rows;
    g_start_line_debug.guide_center = candidate.guide_center;

    switch (g_start_line_ctx.state)
    {
        case START_LINE_STATE_WAITING_CLEAR:
        {
            if (candidate.detected)
            {
                g_start_line_ctx.clear_count = 0U;  // 仍能看到发车线，说明车辆还没真正驶离起点区域。
            }
            else
            {
                if (g_start_line_ctx.clear_count < 255U)
                {
                    g_start_line_ctx.clear_count++;
                }

                if (g_start_line_ctx.clear_count >= START_LINE_ARM_CLEAR_FRAMES)
                {
                    g_start_line_ctx.state = START_LINE_STATE_ARMED;
                    g_start_line_ctx.detect_count = 0U;
                }
            }
            break;
        }

        case START_LINE_STATE_ARMED:
        {
            if (candidate.detected)
            {
                if (g_start_line_ctx.detect_count < 255U)
                {
                    g_start_line_ctx.detect_count++;
                }

                g_start_line_ctx.guide_center = candidate.guide_center;

                if (g_start_line_ctx.detect_count >= START_LINE_DETECT_CONFIRM_FRAMES)
                {
                    g_start_line_ctx.state = START_LINE_STATE_FINISHED;
                }
            }
            else
            {
                g_start_line_ctx.detect_count = 0U;  // 中途丢失模式时重新累计，避免单帧误停。
            }
            break;
        }

        case START_LINE_STATE_FINISHED:
        {
            return 1U;
        }

        default:
        {
            start_line_element_init();  // 异常状态下回到安全初始态。
            break;
        }
    }

    g_start_line_debug.state = g_start_line_ctx.state;
    return (g_start_line_ctx.state == START_LINE_STATE_FINISHED) ? 1U : 0U;
}

/**
 * @brief 获取当前发车线状态机状态
 * @return 当前状态机枚举值
 */
Start_Line_State start_line_element_get_state(void)
{
    return g_start_line_ctx.state;
}

/**
 * @brief 获取当前发车线调试信息快照
 * @return 当前发车线调试信息
 */
Start_Line_Debug_Info start_line_element_get_debug_info(void)
{
    return g_start_line_debug;
}
