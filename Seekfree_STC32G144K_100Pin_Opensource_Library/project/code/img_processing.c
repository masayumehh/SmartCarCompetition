#include "img_processing.h"

#include <string.h>

#include "zf_device_mt9v03x.h"

/**
 * @file img_processing.c
 * @brief 赛道图像处理模块
 * @details
 * 本模块负责完成：
 * 1. 图像滤波；
 * 2. OTSU 自适应阈值分割；
 * 3. 赛道中线提取；
 * 4. 输出纯二值图供后续元素识别。
 *
 * 本次优化重点是中线提取：
 * - 不再按列提取“横向中线”；
 * - 改为按行搜索赛道左右边界并取中点；
 * - 如果当前行找不到稳定左右边界，则回退到该行白点重心；
 * - 最后做一次小窗口平滑，减少直道抖动和弯道毛刺。
 */

/** 每帧动态阈值 */
static uint8_t dynamic_threshold = OTSU_THRESHOLD_INIT;  // 当前帧动态阈值。

static uint8_t tune_binary_threshold(uint8_t threshold)
{
    int16_t adjusted = (int16_t)threshold + (int16_t)TUNE_IMAGE_DEBUG_THRESHOLD_OFFSET;

    if (adjusted < (int16_t)TUNE_IMAGE_DEBUG_THRESHOLD_MIN)
    {
        adjusted = (int16_t)TUNE_IMAGE_DEBUG_THRESHOLD_MIN;
    }
    else if (adjusted > (int16_t)TUNE_IMAGE_DEBUG_THRESHOLD_MAX)
    {
        adjusted = (int16_t)TUNE_IMAGE_DEBUG_THRESHOLD_MAX;
    }

    return (uint8_t)adjusted;
}

/**
 * @brief 判断某行从指定起点开始是否存在连续白色区段
 * @param src 二值图
 * @param y 当前行
 * @param x_start 起始列
 * @param width 图像宽度
 * @param run_length 需要满足的最短连续白点长度
 * @return 1 表示存在，0 表示不存在
 */
static uint8_t has_white_run_from(const uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t y, uint16_t x_start, uint16_t width, uint16_t run_length)
{
    uint16_t x;  // 当前检查到的列号。
    uint16_t count = 0;  // 当前连续白点计数。

    for (x = x_start; x < width; x += TUNE_MIDLINE_SEARCH_STEP)
    {
        if (src[y][x] == 255U)
        {
            count++;
            if (count >= run_length) return 1U;
        }
        else
        {
            count = 0;
        }
    }

    return 0U;
}

/**
 * @brief 查找当前行赛道白块的左边界
 * @param src 二值图
 * @param y 当前行
 * @param width 图像宽度
 * @return 左边界列号，未找到返回 -1
 */
static int16_t find_left_track_edge(const uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t y, uint16_t width)
{
    uint16_t x;  // 从左向右扫描的列索引。

    for (x = 0; x < width; x += TUNE_MIDLINE_SEARCH_STEP)
    {
        if (src[y][x] == 255U && has_white_run_from(src, y, x, width, TUNE_MIDLINE_MIN_WHITE_RUN))
        {
            return (int16_t)x;
        }
    }

    return -1;
}

/**
 * @brief 查找当前行赛道白块的右边界
 * @param src 二值图
 * @param y 当前行
 * @param width 图像宽度
 * @return 右边界列号，未找到返回 -1
 */
static int16_t find_right_track_edge(const uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t y, uint16_t width)
{
    int16_t x;  // 从右向左扫描的列索引。
    uint16_t count = 0;  // 当前连续白点计数。
    int16_t right_edge_offset = (int16_t)(((uint16_t)TUNE_MIDLINE_MIN_WHITE_RUN - 1U) * (uint16_t)TUNE_MIDLINE_SEARCH_STEP);  // 右边界位置修正量。

    for (x = (int16_t)(width - 1U); x >= 0; x -= (int16_t)TUNE_MIDLINE_SEARCH_STEP)
    {
        if (src[y][x] == 255U)
        {
            count++;
            if (count >= TUNE_MIDLINE_MIN_WHITE_RUN)
            {
                int16_t edge = x + right_edge_offset;
                if (edge >= (int16_t)width) edge = (int16_t)(width - 1U);
                return edge;
            }
        }
        else
        {
            count = 0;
        }
    }

    return -1;
}

/**
 * @brief 计算当前行白点重心
 * @param src 二值图
 * @param y 当前行
 * @param width 图像宽度
 * @return 重心列号，未找到返回 -1
 */
static int16_t get_row_centroid(const uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t y, uint16_t width)
{
    uint16_t x;  // 当前扫描列号。
    int32_t sum_x = 0;  // 当前行白点列号总和。
    uint16_t count = 0;  // 当前行白点数量。

    for (x = 0; x < width; x++)
    {
        if (src[y][x] == 255U)
        {
            sum_x += x;
            count++;
        }
    }

    if (count == 0U) return -1;

    return (int16_t)(sum_x / (int32_t)count);
}

static int16_t abs_i16_local(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

/**
 * @brief 对中线做滑动平均平滑
 * @param mid_line 中线数组
 * @param length 中线数组长度
 */
static void smooth_mid_line(int16_t mid_line[IMAGE_HEIGHT], uint16_t length)
{
    int16_t smoothed[IMAGE_HEIGHT];  // 平滑后的中线临时数组。
    uint16_t i;  // 当前处理中线行号。

    for (i = 0; i < length; i++)
    {
        int32_t sum = 0;  // 平滑窗口内有效中线值总和。
        uint16_t count = 0;  // 平滑窗口内有效中线点数量。
        int16_t start = (int16_t)i - (int16_t)TUNE_MIDLINE_SMOOTH_WINDOW;  // 平滑窗口起始行。
        int16_t end = (int16_t)i + (int16_t)TUNE_MIDLINE_SMOOTH_WINDOW;  // 平滑窗口结束行。
        int16_t j;  // 遍历窗口的行索引。

        for (j = start; j <= end; j++)
        {
            if (j >= 0 && j < (int16_t)length && mid_line[j] >= 0)
            {
                sum += mid_line[j];
                count++;
            }
        }

        if (count > 0U) smoothed[i] = (int16_t)(sum / (int32_t)count);
        else smoothed[i] = mid_line[i];
    }

    for (i = 0; i < length; i++)
    {
        mid_line[i] = smoothed[i];
    }
}

/**
 * @brief 计算 OTSU 自适应阈值
 * @param src 输入灰度图
 * @param width 图像宽度
 * @param height 图像高度
 * @return 最佳阈值（0~255）
 */
uint8_t calculate_otsu_threshold(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height)
{
    uint32_t hist[256] = {0};  // 0~255 灰度直方图。
    uint32_t total = 0U;  // 参与阈值统计的像素总数。
    uint32_t sum_all = 0;  // 全图灰度值总和。
    uint32_t w_b = 0;  // 当前阈值下一侧像素数。
    uint32_t sum_b = 0;  // 当前阈值下一侧灰度总和。
    float max_var = -1.0f;  // 当前找到的最大类间方差。
    uint8_t threshold = OTSU_THRESHOLD_INIT;  // 当前最优阈值。
    uint16_t x;  // 当前列索引。
    uint16_t y;  // 当前行索引。
    uint16_t t;  // 当前尝试的阈值。
    uint16_t y_start;
    uint16_t y_end;
    uint16_t x_start;
    uint16_t x_end;
    uint16_t x_step;
    uint16_t y_step;

    if (width == 0U || height == 0U) return OTSU_THRESHOLD_INIT;

    // OTSU 只统计中下部赛道主视野，尽量避开顶部灯光、远端墙面和边缘背景。
    y_start = height / 3U;
    y_end = height - (height / 15U);
    x_start = width / 12U;
    x_end = width - (width / 12U);
    if (y_end <= y_start) y_end = height;
    if (x_end <= x_start) x_end = width;
    x_step = (TUNE_OTSU_SAMPLE_X_STEP == 0U) ? 1U : TUNE_OTSU_SAMPLE_X_STEP;
    y_step = (TUNE_OTSU_SAMPLE_Y_STEP == 0U) ? 1U : TUNE_OTSU_SAMPLE_Y_STEP;

    for (y = y_start; y < y_end; y += y_step)
    {
        for (x = x_start; x < x_end; x += x_step)
        {
            hist[src[y][x]]++;
            total++;
        }
    }

    if (total == 0U) return OTSU_THRESHOLD_INIT;

    for (t = 0; t < 256; t++)
    {
        sum_all += (uint32_t)t * hist[t];
    }

    for (t = 0; t < 256; t++)
    {
        float m_b;  // 一侧灰度均值。
        float m_f;  // 另一侧灰度均值。
        float diff;  // 两侧均值差。
        float var_between;  // 当前阈值对应的类间方差。
        uint32_t w_f;  // 另一侧像素数。

        w_b += hist[t];
        if (w_b == 0U) continue;

        w_f = total - w_b;
        if (w_f == 0U) break;

        sum_b += (uint32_t)t * hist[t];

        m_b = (float)sum_b / (float)w_b;
        m_f = (float)(sum_all - sum_b) / (float)w_f;
        diff = m_b - m_f;
        var_between = (float)w_b * (float)w_f * diff * diff;

        if (var_between > max_var)
        {
            max_var = var_between;
            threshold = (uint8_t)t;
        }
    }

    return threshold;
}

/**
 * @brief 3x3 中值滤波
 * @param src 输入图像
 * @param dst 输出图像
 * @param width 图像宽度
 * @param height 图像高度
 * @param filter_size 滤波窗口大小
 */
void simple_median_filter(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height, uint8_t filter_size)
{
    uint16_t x;  // 当前列坐标。
    uint16_t y;  // 当前行坐标。

    if (filter_size != 3U)
    {
        for (y = 0; y < height; y++)
        {
            for (x = 0; x < width; x++)
            {
                dst[y][x] = src[y][x];  // 如果不是 3x3，就直接原样拷贝。
            }
        }
        return;  // 结束本次滤波。
    }

    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            uint8_t win[9];  // 3x3 邻域窗口像素值。
            uint8_t idx = 0;  // 当前已写入窗口的元素个数。
            int8_t dx;  // 邻域列方向偏移。
            int8_t dy;  // 邻域行方向偏移。
            uint8_t i;  // 排序外层索引。
            uint8_t j;  // 排序内层索引。

            for (dy = -1; dy <= 1; dy++)
            {
                for (dx = -1; dx <= 1; dx++)
                {
                    int16_t nx = (int16_t)x + dx;  // 邻域点列坐标。
                    int16_t ny = (int16_t)y + dy;  // 邻域点行坐标。

                    if (nx < 0 || ny < 0 || nx >= (int16_t)width || ny >= (int16_t)height) win[idx++] = src[y][x];  // 越界时退回当前像素值。
                    else win[idx++] = src[ny][nx];  // 未越界时读取邻域像素。
                }
            }

            for (i = 0; i < 8; i++)
            {
                uint8_t min_i = i;
                for (j = i + 1; j < 9; j++)
                {
                    if (win[j] < win[min_i]) min_i = j;  // 找到当前更小的元素下标。
                }
                if (min_i != i)
                {
                    uint8_t tmp = win[i];  // 排序交换时的临时变量。
                    win[i] = win[min_i];  // 把更小值换到前面。
                    win[min_i] = tmp;  // 原位置放回旧值。
                }
            }

            dst[y][x] = win[4];  // 排序后中间值就是 3x3 中值。
        }
    }
}

/**
 * @brief 单像素阈值二值化
 * @param value 输入灰度值
 * @param threshold 阈值
 * @return 二值化结果
 */
uint8_t correct_threshold(uint8_t value, uint8_t threshold)
{
    return (value >= threshold) ? 255U : 0U;  // 大于等于阈值记为白，否则记为黑。
}

/**
 * @brief 全图阈值二值化
 * @param src 输入图像
 * @param dst 输出图像
 * @param width 图像宽度
 * @param height 图像高度
 * @param threshold 阈值
 */
void binarize_with_threshold(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height, uint8_t threshold)
{
    uint16_t x;  // 当前列坐标。
    uint16_t y;  // 当前行坐标。

    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            dst[y][x] = correct_threshold(src[y][x], threshold);  // 对每个像素执行单点阈值分割。
        }
    }
}

/**
 * @brief 提取赛道中线
 * @param src 二值图
 * @param width 图像宽度
 * @param height 图像高度
 * @param mid_line 输出中线数组
 * @details
 * 输出数组的含义是：mid_line[y] = 第 y 行赛道中心所在的列号。
 * 处理顺序为：
 * 1. 先在当前行找左边界和右边界，取中点；
 * 2. 如果左右边界不完整，则退回到当前行白点重心；
 * 3. 全部行处理完成后，再对整条中线做平滑。
 */
void get_mid_line(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height, int16_t mid_line[IMAGE_HEIGHT])
{
    int16_t y;  // 当前处理的行号。
    int16_t prev_mid = (int16_t)(width / 2U);  // 上一行已接受的中线。
    int16_t prev_width = (int16_t)(width / 3U);  // 上一行已接受的赛道宽度估计。
    uint8_t prev_valid = 0U;  // 是否已经建立连续跟踪参考。

    for (y = (int16_t)(height - 1U); y >= 0; y--)
    {
        int16_t left_edge = find_left_track_edge(src, (uint16_t)y, width);  // 当前行检测到的左边界。
        int16_t right_edge = find_right_track_edge(src, (uint16_t)y, width);  // 当前行检测到的右边界。
        int16_t candidate_mid = -1;  // 当前行候选中线。
        int16_t candidate_width = -1;  // 当前行候选赛道宽度。
        uint8_t edge_valid = 0U;  // 当前行是否找到了可信左右边界。
        int16_t allowed_center_jump = TUNE_MIDLINE_MAX_CENTER_JUMP;  // 当前行允许的中线跳变阈值。
        int16_t allowed_width_jump = TUNE_MIDLINE_MAX_WIDTH_JUMP;  // 当前行允许的宽度跳变阈值。

        if (left_edge >= 0 && right_edge >= 0 && right_edge > left_edge)
        {
            candidate_width = (int16_t)(right_edge - left_edge);
            candidate_mid = (int16_t)((left_edge + right_edge) / 2);  // 左右边界都有效时，取中点作为候选中心。
            edge_valid = 1U;
        }
        else
        {
            candidate_mid = get_row_centroid(src, (uint16_t)y, width);  // 边界不完整时，退回到该行白点重心。
        }

        if (height > 1U)
        {
            uint16_t upper_ratio = (uint16_t)(((height - 1U) - (uint16_t)y) * 255U / (height - 1U));  // 越靠上数值越大。
            allowed_center_jump = (int16_t)(allowed_center_jump +
                                  (int16_t)((TUNE_MIDLINE_UPPER_CENTER_JUMP_BONUS * upper_ratio) / 255U));
            allowed_width_jump = (int16_t)(allowed_width_jump +
                                 (int16_t)((TUNE_MIDLINE_UPPER_WIDTH_JUMP_BONUS * upper_ratio) / 255U));
        }

        if (prev_valid)
        {
            if (edge_valid)
            {
                if (abs_i16_local((int16_t)(candidate_mid - prev_mid)) > allowed_center_jump ||
                    abs_i16_local((int16_t)(candidate_width - prev_width)) > allowed_width_jump)
                {
                    edge_valid = 0U;  // 当前行虽然有白块，但和近场连续轨迹不一致，判为误检。
                }
            }

            if (!edge_valid)
            {
                if (candidate_mid < 0 || abs_i16_local((int16_t)(candidate_mid - prev_mid)) > allowed_center_jump)
                {
                    candidate_mid = prev_mid;  // 重心也不可信时，直接沿用近场连续轨迹，避免被反光或远端白块带偏。
                }
            }
        }

        mid_line[y] = candidate_mid;
        if (candidate_mid >= 0)
        {
            prev_mid = candidate_mid;
            if (edge_valid && candidate_width > 0)
            {
                prev_width = candidate_width;
            }
            prev_valid = 1U;
        }
    }

    smooth_mid_line(mid_line, height);  // 对整条中线做一次平滑。
}

#if 0
static int16_t abs_i16(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

/**
 * @brief 边缘检测
 * @param src 输入图像
 * @param dst 输出边缘图
 * @param width 图像宽度
 * @param height 图像高度
 */
void detect_edge_with_boundary(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height)
{
    uint16_t x;  // 当前列坐标。
    uint16_t y;  // 当前行坐标。

    memset(dst, 0, (uint16_t)(width * height));  // 先把整张边缘图清零。

    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            int16_t gx;  // 横向梯度。
            int16_t gy;  // 纵向梯度。
            int16_t magnitude;  // 边缘强度近似值。

            if (x == 0U) gx = (int16_t)src[y][x + 1U] - src[y][x];  // 左边界用单边差分。
            else if (x == width - 1U) gx = (int16_t)src[y][x] - src[y][x - 1U];  // 右边界用单边差分。
            else gx = (int16_t)src[y][x + 1U] - src[y][x - 1U];  // 中间区域用左右差分。

            if (y == 0U) gy = (int16_t)src[y + 1U][x] - src[y][x];  // 顶部边界用单边差分。
            else if (y == height - 1U) gy = (int16_t)src[y][x] - src[y - 1U][x];  // 底部边界用单边差分。
            else gy = (int16_t)src[y + 1U][x] - src[y - 1U][x];  // 中间区域用上下差分。

            magnitude = abs_i16(gx) + abs_i16(gy);  // 用横向和纵向变化估计边缘强度。
            dst[y][x] = (magnitude > 30) ? 255U : 0U;  // 强度超过阈值就标记为边缘。
        }
    }
}
#endif

/**
 * @brief 图像处理主流程
 * @param output_buffer 输出图像缓冲
 * @param mid_line_buffer 输出中线缓冲
 * @details
 * 流程为：
 * 1. 对摄像头灰度图做中值滤波；
 * 2. 计算当前帧的 OTSU 阈值；
 * 3. 按阈值做二值化；
 * 4. 从二值图中逐行提取中线；
 * 5. 输出保持为纯二值图，直接供后续元素识别使用。
 */
void image_processing(uint8_t output_buffer[IMAGE_HEIGHT][IMAGE_WIDTH], int16_t mid_line_buffer[IMAGE_HEIGHT])
{
    uint16_t mask_y;
    uint16_t top_mask_rows;
    uint16_t bottom_mask_rows;

    dynamic_threshold = calculate_otsu_threshold(output_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);  // 直接在已拷贝的静态帧上算阈值，避免读取 DMA 活帧。
    dynamic_threshold = tune_binary_threshold(dynamic_threshold);  // 对 OTSU 阈值做小范围修正，更适合当前白赛道场地。
    binarize_with_threshold(output_buffer, output_buffer, IMAGE_WIDTH, IMAGE_HEIGHT, dynamic_threshold);  // 在同一张图上原地二值化，继续节省内存并提升处理速度。

    top_mask_rows = (uint16_t)((IMAGE_HEIGHT * TUNE_IMAGE_MASK_TOP_RATIO_NUM) / TUNE_IMAGE_MASK_TOP_RATIO_DEN);
    bottom_mask_rows = (uint16_t)((IMAGE_HEIGHT * TUNE_IMAGE_MASK_BOTTOM_RATIO_NUM) / TUNE_IMAGE_MASK_BOTTOM_RATIO_DEN);

    // 只做更轻的上下遮罩，保留元素识别需要的远端和中上部赛道信息。
    for (mask_y = 0U; mask_y < top_mask_rows; mask_y++)
    {
        memset(output_buffer[mask_y], 0, IMAGE_WIDTH);
    }
    for (mask_y = (uint16_t)(IMAGE_HEIGHT - bottom_mask_rows); mask_y < IMAGE_HEIGHT; mask_y++)
    {
        memset(output_buffer[mask_y], 0, IMAGE_WIDTH);
    }

    get_mid_line(output_buffer, IMAGE_WIDTH, IMAGE_HEIGHT, mid_line_buffer);  // 从二值图提取赛道中线。
}

/**
 * @brief 读取指定行的中线值
 * @param y 行号
 * @param mid_line_buffer 中线数组，满足 mid_line_buffer[y] = 该行中心列号
 * @param height 图像高度
 * @return 中线值，超界返回 -1
 */
int16_t get_mid_line_at(uint16_t y, const int16_t mid_line_buffer[IMAGE_HEIGHT], uint16_t height)
{
    if (y < height) return mid_line_buffer[y];  // 行号合法时返回这一行的中线值。
    return -1;  // 越界时返回无效值。
}

#if 0
/**
 * @brief 判断某点是否为边缘点
 * @param x 列坐标
 * @param y 行坐标
 * @param edge_buffer_in 边缘缓冲
 * @param width 图像宽度
 * @param height 图像高度
 * @return 1 表示边缘，0 表示非边缘
 */
uint8_t is_edge(uint16_t x, uint16_t y, const uint8_t edge_buffer_in[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height)
{
    if (x < width && y < height) return edge_buffer_in[y][x];  // 坐标合法时返回边缘图中的结果。
    return 0U;  // 越界时默认不是边缘。
}
#endif
