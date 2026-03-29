#include "slope_element.h"

/* 三个宽度采样高度的行号。 */
#define SLOPE_SAMPLE_ROW_LOWER      ((IMAGE_HEIGHT * TUNE_SLOPE_SAMPLE_ROW_LOWER_RATIO_NUM) / TUNE_SLOPE_SAMPLE_ROW_LOWER_RATIO_DEN)
#define SLOPE_SAMPLE_ROW_MIDDLE     ((IMAGE_HEIGHT * TUNE_SLOPE_SAMPLE_ROW_MIDDLE_RATIO_NUM) / TUNE_SLOPE_SAMPLE_ROW_MIDDLE_RATIO_DEN)
#define SLOPE_SAMPLE_ROW_UPPER      ((IMAGE_HEIGHT * TUNE_SLOPE_SAMPLE_ROW_UPPER_RATIO_NUM) / TUNE_SLOPE_SAMPLE_ROW_UPPER_RATIO_DEN)
/* 每个采样高度上下各取多少行做均值，避免单行抖动过大。 */
#define SLOPE_SAMPLE_BAND_HALF      TUNE_SLOPE_SAMPLE_BAND_HALF_HEIGHT
/* 从这一行往上更容易受俯仰影响，因此覆盖中线在这里以上逐渐向引导中心拉直。 */
#define SLOPE_BLEND_START_ROW       ((IMAGE_HEIGHT * TUNE_SLOPE_BLEND_START_RATIO_NUM) / TUNE_SLOPE_BLEND_START_RATIO_DEN)

/* 普通直道参考和候选判定的基础阈值。 */
#define SLOPE_REF_MIN_ROWS          TUNE_SLOPE_REF_MIN_ROWS
#define SLOPE_NORMAL_MIN_WIDTH      TUNE_SLOPE_NORMAL_MIN_WIDTH
#define SLOPE_NORMAL_MAX_WIDTH      TUNE_SLOPE_NORMAL_MAX_WIDTH
#define SLOPE_CENTER_DIFF_LIMIT     TUNE_SLOPE_CENTER_DIFF_LIMIT
#define SLOPE_EDGE_SYMMETRY_LIMIT   TUNE_SLOPE_EDGE_SYMMETRY_LIMIT
#define SLOPE_BASE_WIDTH_TOLERANCE  TUNE_SLOPE_BASE_WIDTH_TOLERANCE
#define SLOPE_FLAT_WIDTH_TOLERANCE  TUNE_SLOPE_FLAT_WIDTH_TOLERANCE

/* 梯形候选阈值。 */
#define SLOPE_TRAPEZOID_MID_SHRINK  TUNE_SLOPE_TRAPEZOID_MIDDLE_SHRINK
#define SLOPE_TRAPEZOID_UP_SHRINK   TUNE_SLOPE_TRAPEZOID_UPPER_SHRINK
#define SLOPE_CLOSE_ROW_SHRINK      TUNE_SLOPE_CLOSE_ROW_SHRINK
#define SLOPE_CLOSE_MIN_ROWS        TUNE_SLOPE_CLOSE_MIN_ROWS

/* 状态机确认与保持参数。 */
#define SLOPE_DETECT_CONFIRM_FRAMES    TUNE_SLOPE_DETECT_CONFIRM_FRAMES
#define SLOPE_LOST_CONFIRM_FRAMES      TUNE_SLOPE_LOST_CONFIRM_FRAMES
#define SLOPE_PLATFORM_CONFIRM_FRAMES  TUNE_SLOPE_PLATFORM_CONFIRM_FRAMES
#define SLOPE_PLATFORM_MIN_HOLD_FRAMES TUNE_SLOPE_PLATFORM_MIN_HOLD_FRAMES
#define SLOPE_EXIT_CONFIRM_FRAMES      TUNE_SLOPE_EXIT_CONFIRM_FRAMES
#define SLOPE_EXIT_HOLD_FRAMES         TUNE_SLOPE_EXIT_HOLD_FRAMES
#define SLOPE_MAX_PHASE_FRAMES         TUNE_SLOPE_MAX_PHASE_FRAMES

/* 各阶段的中线拉直比例。 */
#define SLOPE_APPROACH_BLEND_PERCENT  TUNE_SLOPE_APPROACH_BLEND_PERCENT
#define SLOPE_CLIMB_BLEND_PERCENT     TUNE_SLOPE_CLIMB_BLEND_PERCENT
#define SLOPE_PLATFORM_BLEND_PERCENT  TUNE_SLOPE_PLATFORM_BLEND_PERCENT
#define SLOPE_DOWNHILL_BLEND_PERCENT  TUNE_SLOPE_DOWNHILL_BLEND_PERCENT
#define SLOPE_EXIT_BLEND_PERCENT      TUNE_SLOPE_EXIT_BLEND_PERCENT

/* 各阶段的目标速度。 */
#define SLOPE_APPROACH_TARGET_SPEED   TUNE_SLOPE_APPROACH_TARGET_SPEED
#define SLOPE_CLIMB_TARGET_SPEED      TUNE_SLOPE_CLIMB_TARGET_SPEED
#define SLOPE_PLATFORM_TARGET_SPEED   TUNE_SLOPE_PLATFORM_TARGET_SPEED
#define SLOPE_DOWNHILL_TARGET_SPEED   TUNE_SLOPE_DOWNHILL_TARGET_SPEED
#define SLOPE_EXIT_TARGET_SPEED       TUNE_SLOPE_EXIT_TARGET_SPEED

/**
 * @brief 单帧坡道特征
 *
 * 这个结构体只描述“当前这一帧看起来像不像坡道相关图像”，
 * 不负责跨帧状态保持，跨帧的确认逻辑交给后面的状态机完成。
 */
typedef struct
{
    uint8_t valid;               /* 当前帧是否已经提取到足够稳定的几何特征。 */
    uint8_t trapezoid_detected;  /* 当前帧是否命中“居中对称梯形”候选。 */
    uint8_t flat_detected;       /* 当前帧是否更像平台/普通平直直道。 */
    uint16_t lower_width;        /* 下采样高度的平均赛道宽度。 */
    uint16_t middle_width;       /* 中采样高度的平均赛道宽度。 */
    uint16_t upper_width;        /* 上采样高度的平均赛道宽度。 */
    int16_t lower_center;        /* 下采样高度的中心列。 */
    int16_t middle_center;       /* 中采样高度的中心列。 */
    int16_t upper_center;        /* 上采样高度的中心列。 */
    int16_t lower_left;          /* 下采样高度的左边线位置。 */
    int16_t lower_right;         /* 下采样高度的右边线位置。 */
    int16_t upper_left;          /* 上采样高度的左边线位置。 */
    int16_t upper_right;         /* 上采样高度的右边线位置。 */
    uint16_t close_rows;         /* 图像下部已经明显收窄的行数。 */
    int16_t guide_center;        /* 当前帧估计出的引导中心。 */
} Slope_Frame_Features;

/**
 * @brief 坡道状态机上下文
 *
 * 这里同时保存：
 * 1. 当前处于坡道的哪一个阶段；
 * 2. 普通直道的参考宽度与参考中心；
 * 3. 各种确认计数器，避免单帧抖动导致误切状态。
 */
typedef struct
{
    Slope_State state;         /* 当前坡道状态。 */
    uint8_t reference_ready;   /* 是否已经建立普通直道参考。 */
    uint8_t detect_count;      /* 梯形候选连续命中计数。 */
    uint8_t transition_count;  /* 当前阶段向下一阶段切换的确认计数。 */
    uint8_t lost_count;        /* 当前阶段连续看不到有效特征的计数。 */
    uint8_t hold_count;        /* 平台/出坡阶段的保持计数。 */
    uint16_t base_lower_width; /* 普通直道参考下宽度。 */
    uint16_t base_middle_width;/* 普通直道参考中宽度。 */
    uint16_t base_upper_width; /* 普通直道参考上宽度。 */
    int16_t base_center;       /* 普通直道参考中心。 */
    int16_t guide_center;      /* 当前覆盖中线使用的引导中心。 */
    uint16_t lower_width;      /* 当前帧下宽度缓存。 */
    uint16_t middle_width;     /* 当前帧中宽度缓存。 */
    uint16_t upper_width;      /* 当前帧上宽度缓存。 */
    uint16_t close_rows;       /* 当前帧下部收窄行数缓存。 */
    uint8_t trapezoid_detected;/* 当前帧梯形命中标志缓存。 */
    uint8_t flat_detected;     /* 当前帧平台命中标志缓存。 */
} Slope_Context;

/* 全局坡道状态机对象。 */
static Slope_Context g_slope_ctx =
{
    SLOPE_STATE_NONE,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U),
    (int16_t)(IMAGE_WIDTH / 2U),
    0U,
    0U,
    0U,
    0U,
    0U,
    0U
};

/* 全局坡道调试信息对象。 */
static Slope_Debug_Info g_slope_debug =
{
    SLOPE_STATE_NONE,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U)
};

/**
 * @brief 计算两个有符号数的绝对差
 */
static uint16_t abs_diff_i16(int16_t a, int16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

/**
 * @brief 将列号限制在图像范围内
 */
static int16_t clamp_column(int16_t value)
{
    if (value < 0) return 0;
    if (value >= (int16_t)IMAGE_WIDTH) return (int16_t)(IMAGE_WIDTH - 1);
    return value;
}

/**
 * @brief 逐行提取赛道左右边线、宽度和中心
 *
 * 对坡道模块来说，真正有用的是“赛道几何形状”：
 * 1. 左边线是否整体向右收；
 * 2. 右边线是否整体向左收；
 * 3. 中心是否一直稳定在图像中央附近。
 */
static void build_row_geometry(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                               int16_t left_edges[IMAGE_HEIGHT],
                               int16_t right_edges[IMAGE_HEIGHT],
                               uint16_t widths[IMAGE_HEIGHT],
                               int16_t centers[IMAGE_HEIGHT],
                               uint8_t valid_rows[IMAGE_HEIGHT])
{
    uint16_t y;

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        uint16_t x;
        uint8_t found = 0U;

        left_edges[y] = -1;
        right_edges[y] = -1;
        widths[y] = 0U;
        centers[y] = -1;
        valid_rows[y] = 0U;

        for (x = 0U; x < IMAGE_WIDTH; x++)
        {
            if (image[y][x] > 0U)
            {
                if (!found)
                {
                    left_edges[y] = (int16_t)x;  /* 第一个白点作为左边线。 */
                    found = 1U;
                }

                right_edges[y] = (int16_t)x;     /* 最后一个白点作为右边线。 */
            }
        }

        if (found && right_edges[y] > left_edges[y])
        {
            widths[y] = (uint16_t)(right_edges[y] - left_edges[y] + 1);
            centers[y] = (int16_t)((left_edges[y] + right_edges[y]) / 2);
            valid_rows[y] = 1U;  /* 只有左右边线都完整时，这一行才用于几何判定。 */
        }
    }
}

/**
 * @brief 统计数组在采样带中的平均值
 *
 * 坡道识别不直接看单行，因为单行容易抖；
 * 因此每个采样高度都会取“中心行上下若干行”的平均值。
 */
static uint8_t average_geometry_band(uint16_t center_row,
                                     const int16_t left_edges[IMAGE_HEIGHT],
                                     const int16_t right_edges[IMAGE_HEIGHT],
                                     const uint16_t widths[IMAGE_HEIGHT],
                                     const int16_t centers[IMAGE_HEIGHT],
                                     const uint8_t valid_rows[IMAGE_HEIGHT],
                                     int16_t *avg_left,
                                     int16_t *avg_right,
                                     uint16_t *avg_width,
                                     int16_t *avg_center)
{
    int32_t sum_left = 0;
    int32_t sum_right = 0;
    int32_t sum_width = 0;
    int32_t sum_center = 0;
    uint16_t count = 0U;
    int16_t row_start = (int16_t)center_row - (int16_t)SLOPE_SAMPLE_BAND_HALF;
    int16_t row_end = (int16_t)center_row + (int16_t)SLOPE_SAMPLE_BAND_HALF;
    int16_t row;

    if (row_start < 0) row_start = 0;
    if (row_end >= (int16_t)IMAGE_HEIGHT) row_end = (int16_t)(IMAGE_HEIGHT - 1);

    for (row = row_start; row <= row_end; row++)
    {
        uint16_t y = (uint16_t)row;

        if (valid_rows[y])
        {
            sum_left += left_edges[y];
            sum_right += right_edges[y];
            sum_width += widths[y];
            sum_center += centers[y];
            count++;
        }
    }

    if (count == 0U)
    {
        *avg_left = -1;
        *avg_right = -1;
        *avg_width = 0U;
        *avg_center = -1;
        return 0U;
    }

    *avg_left = (int16_t)(sum_left / (int32_t)count);
    *avg_right = (int16_t)(sum_right / (int32_t)count);
    *avg_width = (uint16_t)(sum_width / (int32_t)count);
    *avg_center = (int16_t)(sum_center / (int32_t)count);
    return 1U;
}

/**
 * @brief 更新普通直道参考
 *
 * 这个参考代表“没遇到坡道时，正常直道在三个高度上的宽度应该长什么样”。
 * 后面所有“梯形是否明显收窄”的判断，都要和这个参考做对比。
 */
static void update_straight_reference(const Slope_Frame_Features *features)
{
    if (!g_slope_ctx.reference_ready)
    {
        g_slope_ctx.base_lower_width = features->lower_width;
        g_slope_ctx.base_middle_width = features->middle_width;
        g_slope_ctx.base_upper_width = features->upper_width;
        g_slope_ctx.base_center = features->guide_center;
        g_slope_ctx.reference_ready = 1U;  /* 第一帧合格的普通直道直接拿来建参考。 */
        return;
    }

    /* 后续用 3:1 的低通更新，避免参考值被单帧噪声带偏。 */
    g_slope_ctx.base_lower_width = (uint16_t)(((uint32_t)g_slope_ctx.base_lower_width * 3U + features->lower_width + 2U) / 4U);
    g_slope_ctx.base_middle_width = (uint16_t)(((uint32_t)g_slope_ctx.base_middle_width * 3U + features->middle_width + 2U) / 4U);
    g_slope_ctx.base_upper_width = (uint16_t)(((uint32_t)g_slope_ctx.base_upper_width * 3U + features->upper_width + 2U) / 4U);
    g_slope_ctx.base_center = (int16_t)(((int32_t)g_slope_ctx.base_center * 3 + features->guide_center + 2) / 4);
}

/**
 * @brief 统计下部已经明显收窄的行数
 *
 * 越靠近坡脚，图像下部越多行会开始比“普通直道参考下宽度”更窄，
 * 因此这个计数可以用来判断是不是已经贴近坡道入口。
 */
static uint16_t count_close_rows(const uint16_t widths[IMAGE_HEIGHT],
                                 const uint8_t valid_rows[IMAGE_HEIGHT],
                                 uint16_t base_lower_width)
{
    uint16_t y;
    uint16_t close_rows = 0U;

    for (y = SLOPE_SAMPLE_ROW_MIDDLE; y < IMAGE_HEIGHT; y++)
    {
        if (valid_rows[y] && widths[y] + SLOPE_CLOSE_ROW_SHRINK <= base_lower_width)
        {
            close_rows++;
        }
    }

    return close_rows;
}

/**
 * @brief 分析当前帧的坡道几何特征
 *
 * 这里的核心判断是：
 * 1. 中心是否稳定，避免把普通大弯误识别成坡道；
 * 2. 左右边线是否对称向内收，符合“梯形直道”外观；
 * 3. 中上部是否比普通直道参考明显变窄。
 */
static Slope_Frame_Features analyze_slope_frame(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                                const int16_t mid_line[IMAGE_HEIGHT])
{
    int16_t left_edges[IMAGE_HEIGHT];
    int16_t right_edges[IMAGE_HEIGHT];
    uint16_t widths[IMAGE_HEIGHT];
    int16_t centers[IMAGE_HEIGHT];
    uint8_t valid_rows[IMAGE_HEIGHT];
    Slope_Frame_Features features;
    uint16_t valid_count = 0U;
    uint16_t y;

    features.valid = 0U;
    features.trapezoid_detected = 0U;
    features.flat_detected = 0U;
    features.lower_width = 0U;
    features.middle_width = 0U;
    features.upper_width = 0U;
    features.lower_center = -1;
    features.middle_center = -1;
    features.upper_center = -1;
    features.lower_left = -1;
    features.lower_right = -1;
    features.upper_left = -1;
    features.upper_right = -1;
    features.close_rows = 0U;
    features.guide_center = (int16_t)(IMAGE_WIDTH / 2U);

    build_row_geometry(image, left_edges, right_edges, widths, centers, valid_rows);

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        if (valid_rows[y]) valid_count++;
    }

    if (valid_count < SLOPE_REF_MIN_ROWS)
    {
        return features;  /* 有效赛道行太少时，这一帧不适合做坡道判断。 */
    }

    if (!average_geometry_band(SLOPE_SAMPLE_ROW_LOWER,
                               left_edges,
                               right_edges,
                               widths,
                               centers,
                               valid_rows,
                               &features.lower_left,
                               &features.lower_right,
                               &features.lower_width,
                               &features.lower_center))
    {
        return features;
    }

    if (!average_geometry_band(SLOPE_SAMPLE_ROW_MIDDLE,
                               left_edges,
                               right_edges,
                               widths,
                               centers,
                               valid_rows,
                               &features.upper_left,
                               &features.upper_right,
                               &features.middle_width,
                               &features.middle_center))
    {
        return features;
    }

    if (!average_geometry_band(SLOPE_SAMPLE_ROW_UPPER,
                               left_edges,
                               right_edges,
                               widths,
                               centers,
                               valid_rows,
                               &features.upper_left,
                               &features.upper_right,
                               &features.upper_width,
                               &features.upper_center))
    {
        return features;
    }

    /* 这里重新取一次 lower/upper 边线，是为了后面判断左右边是否对称向内收。 */
    average_geometry_band(SLOPE_SAMPLE_ROW_LOWER,
                          left_edges,
                          right_edges,
                          widths,
                          centers,
                          valid_rows,
                          &features.lower_left,
                          &features.lower_right,
                          &features.lower_width,
                          &features.lower_center);
    average_geometry_band(SLOPE_SAMPLE_ROW_UPPER,
                          left_edges,
                          right_edges,
                          widths,
                          centers,
                          valid_rows,
                          &features.upper_left,
                          &features.upper_right,
                          &features.upper_width,
                          &features.upper_center);

    if (features.lower_width < SLOPE_NORMAL_MIN_WIDTH || features.lower_width > SLOPE_NORMAL_MAX_WIDTH)
    {
        return features;  /* 下部宽度本身就不像正常直道，则不把它作为坡道入口。 */
    }

    if (abs_diff_i16(features.lower_center, features.middle_center) > SLOPE_CENTER_DIFF_LIMIT ||
        abs_diff_i16(features.middle_center, features.upper_center) > SLOPE_CENTER_DIFF_LIMIT ||
        abs_diff_i16(features.lower_center, features.upper_center) > SLOPE_CENTER_DIFF_LIMIT)
    {
        return features;  /* 三个高度中心差太大，说明图像更像弯道，不像直线坡道。 */
    }

    features.valid = 1U;
    features.guide_center = (int16_t)((features.lower_center + features.middle_center + features.upper_center) / 3);

    if (g_slope_ctx.reference_ready)
    {
        uint16_t left_inward = (features.upper_left > features.lower_left) ? (uint16_t)(features.upper_left - features.lower_left) : 0U;
        uint16_t right_inward = (features.lower_right > features.upper_right) ? (uint16_t)(features.lower_right - features.upper_right) : 0U;

        features.close_rows = count_close_rows(widths, valid_rows, g_slope_ctx.base_lower_width);

        if (features.lower_width > features.middle_width &&
            features.middle_width > features.upper_width &&
            features.middle_width + SLOPE_TRAPEZOID_MID_SHRINK <= g_slope_ctx.base_middle_width &&
            features.upper_width + SLOPE_TRAPEZOID_UP_SHRINK <= g_slope_ctx.base_upper_width &&
            abs_diff_i16((int16_t)left_inward, (int16_t)right_inward) <= SLOPE_EDGE_SYMMETRY_LIMIT)
        {
            features.trapezoid_detected = 1U;  /* 当前帧已经呈现出“对称向内收”的梯形直道。 */
        }

        if (abs_diff_i16((int16_t)features.lower_width, (int16_t)g_slope_ctx.base_lower_width) <= SLOPE_FLAT_WIDTH_TOLERANCE &&
            abs_diff_i16((int16_t)features.middle_width, (int16_t)g_slope_ctx.base_middle_width) <= SLOPE_FLAT_WIDTH_TOLERANCE &&
            abs_diff_i16((int16_t)features.upper_width, (int16_t)g_slope_ctx.base_upper_width) <= SLOPE_FLAT_WIDTH_TOLERANCE &&
            abs_diff_i16(features.guide_center, g_slope_ctx.base_center) <= SLOPE_CENTER_DIFF_LIMIT)
        {
            features.flat_detected = 1U;  /* 当前帧更像平台或普通直道。 */
        }
    }
    else
    {
        features.flat_detected = 1U;  /* 还没有参考时，把第一段稳定直道当成参考候选。 */
    }

    if (mid_line[SLOPE_SAMPLE_ROW_LOWER] >= 0 && mid_line[SLOPE_SAMPLE_ROW_LOWER] < IMAGE_WIDTH)
    {
        /* 普通中线如果在下部仍然可靠，就优先把它融入引导中心，避免完全丢掉实时信息。 */
        features.guide_center = (int16_t)((features.guide_center + mid_line[SLOPE_SAMPLE_ROW_LOWER]) / 2);
    }

    return features;
}

/**
 * @brief 获取不同坡道阶段对应的中线拉直比例
 */
static uint16_t get_state_blend_percent(Slope_State state)
{
    if (state == SLOPE_STATE_APPROACHING_UP) return SLOPE_APPROACH_BLEND_PERCENT;
    if (state == SLOPE_STATE_CLIMBING) return SLOPE_CLIMB_BLEND_PERCENT;
    if (state == SLOPE_STATE_PLATFORM) return SLOPE_PLATFORM_BLEND_PERCENT;
    if (state == SLOPE_STATE_DESCENDING) return SLOPE_DOWNHILL_BLEND_PERCENT;
    if (state == SLOPE_STATE_EXITING) return SLOPE_EXIT_BLEND_PERCENT;
    return 0U;
}

/**
 * @brief 获取不同坡道阶段对应的目标速度
 */
static float get_state_target_speed(Slope_State state)
{
    if (state == SLOPE_STATE_APPROACHING_UP) return SLOPE_APPROACH_TARGET_SPEED;
    if (state == SLOPE_STATE_CLIMBING) return SLOPE_CLIMB_TARGET_SPEED;
    if (state == SLOPE_STATE_PLATFORM) return SLOPE_PLATFORM_TARGET_SPEED;
    if (state == SLOPE_STATE_DESCENDING) return SLOPE_DOWNHILL_TARGET_SPEED;
    if (state == SLOPE_STATE_EXITING) return SLOPE_EXIT_TARGET_SPEED;
    return TUNE_DEFAULT_TARGET_SPEED;
}

/**
 * @brief 生成坡道专用覆盖中线
 *
 * 坡道本身位于直道上，因此不需要像环岛或路障那样大幅改轨迹，
 * 这里只需要把上半部分逐渐拉向稳定中心，减轻俯仰变化造成的中线抖动。
 */
static void build_slope_override_mid_line(const int16_t mid_line[IMAGE_HEIGHT],
                                          int16_t guide_center,
                                          Slope_State state,
                                          int16_t override_mid_line[IMAGE_HEIGHT])
{
    uint16_t y;
    uint16_t base_blend = get_state_blend_percent(state);

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        int16_t raw_center = mid_line[y];
        uint16_t effective_blend = 0U;

        if (raw_center < 0 || raw_center >= IMAGE_WIDTH)
        {
            raw_center = guide_center;  /* 普通中线无效时，直接回退到坡道引导中心。 */
        }

        if (y < SLOPE_BLEND_START_ROW)
        {
            /* 越靠上，受俯仰影响越明显，因此越需要向稳定中心靠拢。 */
            effective_blend = (uint16_t)(((uint32_t)base_blend * (SLOPE_BLEND_START_ROW - y)) / (uint32_t)SLOPE_BLEND_START_ROW);
        }
        else
        {
            effective_blend = (uint16_t)(base_blend / 4U);  /* 图像下部仍尽量保留真实中线。 */
        }

        override_mid_line[y] = clamp_column((int16_t)(((int32_t)raw_center * (100 - effective_blend) +
                                                       (int32_t)guide_center * effective_blend + 50) / 100));
    }
}

/**
 * @brief 把当前帧特征同步到上下文和调试信息
 */
static void update_runtime_snapshot(const Slope_Frame_Features *features)
{
    g_slope_ctx.lower_width = features->lower_width;
    g_slope_ctx.middle_width = features->middle_width;
    g_slope_ctx.upper_width = features->upper_width;
    g_slope_ctx.close_rows = features->close_rows;
    g_slope_ctx.trapezoid_detected = features->trapezoid_detected;
    g_slope_ctx.flat_detected = features->flat_detected;

    if (features->valid)
    {
        g_slope_ctx.guide_center = features->guide_center;
    }

    g_slope_debug.state = g_slope_ctx.state;
    g_slope_debug.trapezoid_detected = features->trapezoid_detected;
    g_slope_debug.flat_detected = features->flat_detected;
    g_slope_debug.reference_ready = g_slope_ctx.reference_ready;
    g_slope_debug.lower_width = features->lower_width;
    g_slope_debug.middle_width = features->middle_width;
    g_slope_debug.upper_width = features->upper_width;
    g_slope_debug.base_lower_width = g_slope_ctx.base_lower_width;
    g_slope_debug.close_rows = features->close_rows;
    g_slope_debug.guide_center = g_slope_ctx.guide_center;
}

/**
 * @brief 初始化坡道元素状态机
 */
void slope_element_init(void)
{
    g_slope_ctx.state = SLOPE_STATE_NONE;
    g_slope_ctx.detect_count = 0U;
    g_slope_ctx.transition_count = 0U;
    g_slope_ctx.lost_count = 0U;
    g_slope_ctx.hold_count = 0U;
    g_slope_ctx.guide_center = g_slope_ctx.reference_ready ? g_slope_ctx.base_center : (int16_t)(IMAGE_WIDTH / 2U);
    g_slope_ctx.lower_width = 0U;
    g_slope_ctx.middle_width = 0U;
    g_slope_ctx.upper_width = 0U;
    g_slope_ctx.close_rows = 0U;
    g_slope_ctx.trapezoid_detected = 0U;
    g_slope_ctx.flat_detected = 0U;

    g_slope_debug.state = SLOPE_STATE_NONE;
    g_slope_debug.trapezoid_detected = 0U;
    g_slope_debug.flat_detected = 0U;
    g_slope_debug.reference_ready = g_slope_ctx.reference_ready;
    g_slope_debug.lower_width = 0U;
    g_slope_debug.middle_width = 0U;
    g_slope_debug.upper_width = 0U;
    g_slope_debug.base_lower_width = g_slope_ctx.base_lower_width;
    g_slope_debug.close_rows = 0U;
    g_slope_debug.guide_center = g_slope_ctx.guide_center;
}

/**
 * @brief 坡道识别与专用控制主函数
 *
 * 状态机顺序严格按真实通过过程组织：
 * 1. 先看到上坡前的梯形直道，进入 APPROACHING_UP；
 * 2. 下部也开始明显收窄后，进入 CLIMBING；
 * 3. 图像重新恢复为平台/普通直道外观后，进入 PLATFORM；
 * 4. 再次看到梯形外观时，判定为下坡段，进入 DESCENDING；
 * 5. 出坡后恢复为普通直道，再进入 EXITING 并最终释放。
 */
uint8_t slope_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                              const int16_t mid_line[IMAGE_HEIGHT],
                              int16_t override_mid_line[IMAGE_HEIGHT],
                              float *override_speed)
{
    Slope_Frame_Features features;

    features = analyze_slope_frame(image, mid_line);

    if (features.valid && features.flat_detected && !features.trapezoid_detected)
    {
        /* 只有在“明显更像普通直道/平台”时，才允许更新普通直道参考。 */
        update_straight_reference(&features);
    }

    update_runtime_snapshot(&features);

    if (!features.valid)
    {
        g_slope_ctx.lost_count++;
    }
    else
    {
        g_slope_ctx.lost_count = 0U;
    }

    switch (g_slope_ctx.state)
    {
        case SLOPE_STATE_NONE:
        {
            if (!g_slope_ctx.reference_ready)
            {
                return 0U;  /* 还没建立参考之前，不贸然启用坡道控制。 */
            }

            if (features.trapezoid_detected)
            {
                g_slope_ctx.detect_count++;
                if (g_slope_ctx.detect_count >= SLOPE_DETECT_CONFIRM_FRAMES)
                {
                    g_slope_ctx.state = SLOPE_STATE_APPROACHING_UP;
                    g_slope_ctx.detect_count = 0U;
                    g_slope_ctx.transition_count = 0U;
                    g_slope_ctx.hold_count = 0U;
                }
            }
            else
            {
                g_slope_ctx.detect_count = 0U;
            }

            if (g_slope_ctx.state == SLOPE_STATE_NONE)
            {
                g_slope_debug.state = g_slope_ctx.state;
                return 0U;  /* 只有真的还处于 NONE 时，才继续走普通巡线。 */
            }

            break;  /* 已经确认进入上坡接近阶段时，这一帧就立刻输出坡道控制。 */
        }

        case SLOPE_STATE_APPROACHING_UP:
        {
            g_slope_ctx.hold_count++;

            if (features.trapezoid_detected)
            {
                g_slope_ctx.lost_count = 0U;

                if (features.close_rows >= SLOPE_CLOSE_MIN_ROWS ||
                    features.lower_width + SLOPE_BASE_WIDTH_TOLERANCE <= g_slope_ctx.base_lower_width)
                {
                    g_slope_ctx.state = SLOPE_STATE_CLIMBING;  /* 坡脚已经靠近，切到上坡阶段。 */
                    g_slope_ctx.transition_count = 0U;
                    g_slope_ctx.hold_count = 0U;
                }
            }
            else if (g_slope_ctx.lost_count >= SLOPE_LOST_CONFIRM_FRAMES)
            {
                slope_element_init();  /* 上坡候选消失太久，说明刚才是误触发。 */
                return 0U;
            }

            if (g_slope_ctx.hold_count >= SLOPE_MAX_PHASE_FRAMES)
            {
                slope_element_init();  /* 接近上坡太久仍没有真正进入，说明当前识别链不可靠，直接释放。 */
                return 0U;
            }
            break;
        }

        case SLOPE_STATE_CLIMBING:
        {
            g_slope_ctx.hold_count++;

            if (features.flat_detected)
            {
                g_slope_ctx.transition_count++;
                if (g_slope_ctx.transition_count >= SLOPE_PLATFORM_CONFIRM_FRAMES)
                {
                    g_slope_ctx.state = SLOPE_STATE_PLATFORM;  /* 宽度恢复正常，说明已经上到平台。 */
                    g_slope_ctx.transition_count = 0U;
                    g_slope_ctx.hold_count = 0U;
                }
            }
            else
            {
                g_slope_ctx.transition_count = 0U;
            }

            if (g_slope_ctx.hold_count >= SLOPE_MAX_PHASE_FRAMES)
            {
                g_slope_ctx.state = SLOPE_STATE_PLATFORM;  /* 上坡阶段过久仍未判到平台时，先回收速度，避免一直顶着上坡速度。 */
                g_slope_ctx.transition_count = 0U;
                g_slope_ctx.hold_count = 0U;
            }
            break;
        }

        case SLOPE_STATE_PLATFORM:
        {
            g_slope_ctx.hold_count++;  /* 平台上先保持若干帧，避免刚到平台就把回稳误判成下坡。 */

            if (g_slope_ctx.hold_count >= SLOPE_PLATFORM_MIN_HOLD_FRAMES && features.trapezoid_detected)
            {
                g_slope_ctx.transition_count++;
                if (g_slope_ctx.transition_count >= SLOPE_DETECT_CONFIRM_FRAMES)
                {
                    g_slope_ctx.state = SLOPE_STATE_DESCENDING;  /* 第二次看到梯形，判定为下坡段。 */
                    g_slope_ctx.transition_count = 0U;
                    g_slope_ctx.hold_count = 0U;
                }
            }
            else if (!features.trapezoid_detected)
            {
                g_slope_ctx.transition_count = 0U;
            }

            if (g_slope_ctx.hold_count >= SLOPE_MAX_PHASE_FRAMES)
            {
                g_slope_ctx.state = SLOPE_STATE_EXITING;  /* 平台阶段过久仍未看到下坡时，按正常出坡流程平滑释放控制。 */
                g_slope_ctx.transition_count = 0U;
                g_slope_ctx.hold_count = 0U;
            }
            break;
        }

        case SLOPE_STATE_DESCENDING:
        {
            g_slope_ctx.hold_count++;

            if (features.flat_detected)
            {
                g_slope_ctx.transition_count++;
                if (g_slope_ctx.transition_count >= SLOPE_EXIT_CONFIRM_FRAMES)
                {
                    g_slope_ctx.state = SLOPE_STATE_EXITING;  /* 出坡后恢复到普通直道，开始平滑释放控制。 */
                    g_slope_ctx.transition_count = 0U;
                    g_slope_ctx.hold_count = 0U;
                }
            }
            else
            {
                g_slope_ctx.transition_count = 0U;
            }

            if (g_slope_ctx.hold_count >= SLOPE_MAX_PHASE_FRAMES)
            {
                g_slope_ctx.state = SLOPE_STATE_EXITING;  /* 下坡阶段过久仍未恢复直道时，直接进入平滑释放，避免长时间低速锁死。 */
                g_slope_ctx.transition_count = 0U;
                g_slope_ctx.hold_count = 0U;
            }
            break;
        }

        case SLOPE_STATE_EXITING:
        {
            g_slope_ctx.hold_count++;

            if (features.valid && features.flat_detected && !features.trapezoid_detected)
            {
                update_straight_reference(&features);  /* 出坡阶段顺手刷新一下直道参考，便于后续再次识别坡道。 */
            }

            if (g_slope_ctx.hold_count >= SLOPE_EXIT_HOLD_FRAMES)
            {
                slope_element_init();
                return 0U;
            }
            break;
        }

        default:
        {
            slope_element_init();
            return 0U;
        }
    }

    g_slope_debug.state = g_slope_ctx.state;
    g_slope_debug.reference_ready = g_slope_ctx.reference_ready;
    g_slope_debug.base_lower_width = g_slope_ctx.base_lower_width;
    g_slope_debug.guide_center = g_slope_ctx.guide_center;

    build_slope_override_mid_line(mid_line,
                                  g_slope_ctx.guide_center,
                                  g_slope_ctx.state,
                                  override_mid_line);  /* 生成坡道专用覆盖中线。 */

    *override_speed = get_state_target_speed(g_slope_ctx.state);  /* 输出对应阶段的目标速度。 */
    return 1U;
}

/**
 * @brief 获取当前坡道状态
 */
Slope_State slope_element_get_state(void)
{
    return g_slope_ctx.state;
}

/**
 * @brief 获取当前坡道调试信息
 */
Slope_Debug_Info slope_element_get_debug_info(void)
{
    return g_slope_debug;
}
