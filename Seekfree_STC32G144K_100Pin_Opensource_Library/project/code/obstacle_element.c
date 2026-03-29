#include "obstacle_element.h"

/* 路障搜索区域的上边界。 */
#define OBSTACLE_SCAN_TOP_ROW           ((IMAGE_HEIGHT * TUNE_OBSTACLE_SCAN_TOP_RATIO_NUM) / TUNE_OBSTACLE_SCAN_TOP_RATIO_DEN)
/* 路障搜索区域的下边界。 */
#define OBSTACLE_SCAN_BOTTOM_ROW        ((IMAGE_HEIGHT * TUNE_OBSTACLE_SCAN_BOTTOM_RATIO_NUM) / TUNE_OBSTACLE_SCAN_BOTTOM_RATIO_DEN)
/* 开始明显把中线拉向绕行通道的行号。 */
#define OBSTACLE_BLEND_START_ROW        ((IMAGE_HEIGHT * TUNE_OBSTACLE_BLEND_START_RATIO_NUM) / TUNE_OBSTACLE_BLEND_START_RATIO_DEN)
/* 障碍物靠近到该行后，切换到正式绕行状态。 */
#define OBSTACLE_AVOID_TRIGGER_ROW      ((IMAGE_HEIGHT * TUNE_OBSTACLE_AVOID_TRIGGER_RATIO_NUM) / TUNE_OBSTACLE_AVOID_TRIGGER_RATIO_DEN)

/* 参与路障检测的赛道宽度范围。 */
#define OBSTACLE_TRACK_MIN_WIDTH        TUNE_OBSTACLE_TRACK_MIN_WIDTH
#define OBSTACLE_TRACK_MAX_WIDTH        TUNE_OBSTACLE_TRACK_MAX_WIDTH
/* 单行内部黑块宽度范围。 */
#define OBSTACLE_MIN_RUN_WIDTH          TUNE_OBSTACLE_MIN_RUN_WIDTH
#define OBSTACLE_MAX_RUN_WIDTH          TUNE_OBSTACLE_MAX_RUN_WIDTH
/* 命中路障候选所需的最少行数、最小高度和最小面积。 */
#define OBSTACLE_MIN_ROWS               TUNE_OBSTACLE_MIN_ROWS
#define OBSTACLE_MIN_HEIGHT             TUNE_OBSTACLE_MIN_HEIGHT
#define OBSTACLE_MIN_AREA               TUNE_OBSTACLE_MIN_AREA
/* 障碍物距离赛道边界至少要保留的原始空间。 */
#define OBSTACLE_MIN_SIDE_GAP           TUNE_OBSTACLE_MIN_SIDE_GAP
/* 障碍物中心相对赛道中心最小偏移。 */
#define OBSTACLE_SIDE_OFFSET_MIN        TUNE_OBSTACLE_SIDE_OFFSET_MIN
/* 连续命中行之间允许的中心跳变量。 */
#define OBSTACLE_CLUSTER_CENTER_LIMIT   TUNE_OBSTACLE_CLUSTER_CENTER_LIMIT
#define OBSTACLE_MAX_WIDTH_SPAN         TUNE_OBSTACLE_MAX_WIDTH_SPAN
#define OBSTACLE_MAX_EDGE_DRIFT         TUNE_OBSTACLE_MAX_EDGE_DRIFT

/* 状态机确认和保持参数。 */
#define OBSTACLE_DETECT_CONFIRM_FRAMES  TUNE_OBSTACLE_DETECT_CONFIRM_FRAMES
#define OBSTACLE_LOST_CONFIRM_FRAMES    TUNE_OBSTACLE_LOST_CONFIRM_FRAMES
#define OBSTACLE_EXIT_HOLD_FRAMES       TUNE_OBSTACLE_EXIT_HOLD_FRAMES

/* 绕障规划时的安全余量。 */
#define OBSTACLE_CLEARANCE_PIXELS       TUNE_OBSTACLE_CLEARANCE_PIXELS
#define OBSTACLE_OUTSIDE_ALLOWANCE      TUNE_OBSTACLE_OUTSIDE_ALLOWANCE_PIXELS

/* 不同阶段的中线偏置比例。 */
#define OBSTACLE_APPROACH_BLEND_PERCENT TUNE_OBSTACLE_APPROACH_BLEND_PERCENT
#define OBSTACLE_AVOID_BLEND_PERCENT    TUNE_OBSTACLE_AVOID_BLEND_PERCENT
#define OBSTACLE_EXIT_BLEND_PERCENT     TUNE_OBSTACLE_EXIT_BLEND_PERCENT

/* 不同阶段的目标速度。 */
#define OBSTACLE_APPROACH_TARGET_SPEED  TUNE_OBSTACLE_APPROACH_TARGET_SPEED
#define OBSTACLE_AVOID_TARGET_SPEED     TUNE_OBSTACLE_AVOID_TARGET_SPEED

/**
 * @brief 单帧路障候选结构体
 *
 * 该结构体只描述“当前这一帧像不像路障”，
 * 不负责跨帧状态保持。
 */
typedef struct
{
    uint8_t detected;             /* 当前帧是否命中路障候选。 */
    Obstacle_Side obstacle_side;  /* 路障位于赛道哪一侧。 */
    Obstacle_Side pass_side;      /* 车辆应该从哪一侧绕行。 */
    uint16_t bbox_left;           /* 路障包围盒左边界。 */
    uint16_t bbox_right;          /* 路障包围盒右边界。 */
    uint16_t bbox_top;            /* 路障包围盒上边界。 */
    uint16_t bbox_bottom;         /* 路障包围盒下边界。 */
    uint16_t bbox_width;          /* 路障包围盒宽度。 */
    uint16_t bbox_height;         /* 路障包围盒高度。 */
    int16_t bbox_center;          /* 路障包围盒中心列号。 */
    uint16_t area;                /* 累计命中面积。 */
    int16_t track_center;         /* 候选区域内的平均赛道中心。 */
} Obstacle_Candidate;

/**
 * @brief 路障状态机上下文
 *
 * 用于跨帧保存当前已经确认的绕行方向、
 * 包围盒位置、引导中心和出障计数等信息。
 */
typedef struct
{
    Obstacle_State state;         /* 当前路障状态机状态。 */
    uint8_t detect_count;         /* 连续命中计数。 */
    uint8_t lost_count;           /* 连续丢失计数。 */
    uint8_t exit_hold_count;      /* 出障保持计数。 */
    Obstacle_Side obstacle_side;  /* 当前确认的障碍侧。 */
    Obstacle_Side pass_side;      /* 当前确认的绕行侧。 */
    int16_t guide_center;         /* 当前覆盖中线使用的引导中心。 */
    uint16_t bbox_left;           /* 当前障碍物左边界。 */
    uint16_t bbox_right;          /* 当前障碍物右边界。 */
    uint16_t bbox_bottom;         /* 当前障碍物最靠下的行号。 */
    uint16_t bbox_width;          /* 当前障碍物宽度。 */
    uint16_t bbox_height;         /* 当前障碍物高度。 */
    int16_t bbox_center;          /* 当前障碍物中心列号。 */
} Obstacle_Context;

/* 全局路障状态机对象。 */
static Obstacle_Context g_obstacle_ctx =
{
    OBSTACLE_STATE_NONE,
    0U,
    0U,
    0U,
    OBSTACLE_SIDE_NONE,
    OBSTACLE_SIDE_NONE,
    (int16_t)(IMAGE_WIDTH / 2U),
    0U,
    0U,
    0U,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U)
};

/* 全局路障调试信息对象。 */
static Obstacle_Debug_Info g_obstacle_debug =
{
    OBSTACLE_STATE_NONE,
    OBSTACLE_SIDE_NONE,
    OBSTACLE_SIDE_NONE,
    (int16_t)(IMAGE_WIDTH / 2U),
    0U,
    0U,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U),
    0U
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
 * @brief 获取当前路障阶段对应的基础偏置比例
 */
static uint16_t get_phase_blend_percent(Obstacle_State state)
{
    /* 接近阶段先中等力度拉向自由通道，避免过早猛打方向。 */
    if (state == OBSTACLE_STATE_APPROACHING) return OBSTACLE_APPROACH_BLEND_PERCENT;
    /* 正式绕行阶段使用最大的偏置比例，优先保证不碰障碍。 */
    if (state == OBSTACLE_STATE_AVOIDING) return OBSTACLE_AVOID_BLEND_PERCENT;
    /* 出障阶段保留一部分偏置，避免刚绕过就过快回中。 */
    if (state == OBSTACLE_STATE_EXITING) return OBSTACLE_EXIT_BLEND_PERCENT;
    /* 其余状态不额外施加避障偏置。 */
    return 0U;
}

/**
 * @brief 根据障碍物下边界估算其接近程度
 *
 * 返回 0~100，越接近正式绕行触发线，值越大。
 */
static uint16_t get_proximity_percent(uint16_t bbox_bottom)
{
    /* 障碍物底部还没到达开始偏置线时，认为“还不够近”。 */
    if (bbox_bottom <= OBSTACLE_BLEND_START_ROW) return 0U;
    /* 障碍物底部一旦越过正式绕行触发线，就认为“已经非常近”。 */
    if (bbox_bottom >= OBSTACLE_AVOID_TRIGGER_ROW) return 100U;

    /* 其余区间按线性比例换算成 0~100 的接近程度。 */
    return (uint16_t)(((uint32_t)(bbox_bottom - OBSTACLE_BLEND_START_ROW) * 100U) /
                      (uint32_t)(OBSTACLE_AVOID_TRIGGER_ROW - OBSTACLE_BLEND_START_ROW));
}

/**
 * @brief 逐行提取赛道左右边界
 *
 * 这里沿用赛道白区的最左和最右外包边界，
 * 对路障模块来说已经足够用来限定“赛道内部搜索区域”。
 */
static void build_row_edges(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                            int16_t left_edges[IMAGE_HEIGHT],
                            int16_t right_edges[IMAGE_HEIGHT],
                            uint8_t valid_rows[IMAGE_HEIGHT])
{
    uint16_t y;

    /* 逐行提取赛道白区的左右外边界，供后续“只在赛道内部搜黑块”使用。 */
    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        uint16_t x;
        uint8_t found = 0U;

        left_edges[y] = -1;
        right_edges[y] = -1;
        valid_rows[y] = 0U;

        for (x = 0U; x < IMAGE_WIDTH; x++)
        {
            if (image[y][x] > 0U)
            {
                if (!found)
                {
                    left_edges[y] = (int16_t)x;
                    found = 1U;
                }

                right_edges[y] = (int16_t)x;
            }
        }

        if (found && right_edges[y] > left_edges[y])
        {
            /* 只有能形成有效宽度时，才把这一行当成有效赛道行。 */
            valid_rows[y] = 1U;
        }
    }
}

/**
 * @brief 计算普通巡线状态下的默认引导中心
 */
static int16_t calculate_default_guide_center(const int16_t mid_line[IMAGE_HEIGHT])
{
    uint16_t y;
    int32_t sum = 0;
    uint16_t count = 0U;

    /* 默认只统计图像下部，避免上半部噪声和元素形变把整体方向带偏。 */
    for (y = (uint16_t)(IMAGE_HEIGHT * 2U / 3U); y < IMAGE_HEIGHT; y++)
    {
        if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH)
        {
            sum += mid_line[y];
            count++;
        }
    }

    if (count == 0U) return (int16_t)(IMAGE_WIDTH / 2U);
    return (int16_t)(sum / (int32_t)count);
}

/**
 * @brief 在单行内寻找最长连续黑块
 *
 * 这里的黑块位于赛道内部，代表可能的障碍投影区域。
 */
static uint16_t find_row_dark_run(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                  uint16_t y,
                                  int16_t left_edge,
                                  int16_t right_edge,
                                  uint16_t *run_left,
                                  uint16_t *run_right)
{
    int16_t x;
    int16_t current_start = -1;
    uint16_t best_width = 0U;

    /* 这里找的是“赛道内部最长的连续黑块”，不是找所有黑点。 */
    *run_left = 0U;
    *run_right = 0U;

    for (x = (int16_t)(left_edge + 1); x <= (int16_t)(right_edge - 1); x++)
    {
        if (image[y][x] == 0U)
        {
            if (current_start < 0) current_start = x;
        }
        else if (current_start >= 0)
        {
            uint16_t width = (uint16_t)(x - current_start);

            if (width > best_width)
            {
                /* 始终保留当前行内部最长的那一段黑块。 */
                best_width = width;
                *run_left = (uint16_t)current_start;
                *run_right = (uint16_t)(x - 1);
            }

            current_start = -1;
        }
    }

    if (current_start >= 0)
    {
        /* 处理一直延伸到本行末尾的黑块。 */
        uint16_t width = (uint16_t)(right_edge - current_start);

        if (width > best_width)
        {
            best_width = width;
            *run_left = (uint16_t)current_start;
            *run_right = (uint16_t)(right_edge - 1);
        }
    }

    return best_width;
}

/**
 * @brief 计算当前行的赛道中心
 */
static int16_t get_track_center_at_row(const int16_t mid_line[IMAGE_HEIGHT],
                                       const int16_t left_edges[IMAGE_HEIGHT],
                                       const int16_t right_edges[IMAGE_HEIGHT],
                                       const uint8_t valid_rows[IMAGE_HEIGHT],
                                       uint16_t y)
{
    /* 优先相信普通巡线已经提出来的中线。 */
    if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH) return mid_line[y];
    /* 中线缺失时，退回到左右边界几何中点。 */
    if (valid_rows[y]) return (int16_t)((left_edges[y] + right_edges[y]) / 2);
    /* 再不行就退回图像中心，避免返回无效值。 */
    return (int16_t)(IMAGE_WIDTH / 2U);
}

/**
 * @brief 根据障碍物位置和绕行侧，计算当前行最安全的目标中心
 *
 * 这一步不是简单“往旁边偏一点”，而是明确计算障碍物之外的可通行区：
 * 1. 如果空间足够，就把目标中心放在自由通道的中间；
 * 2. 如果空间不足，就允许中线稍微压出赛道边界，优先保证不碰障碍。
 */
static int16_t calculate_row_pass_target(int16_t left_edge,
                                         int16_t right_edge,
                                         int16_t base_center,
                                         Obstacle_Side pass_side,
                                         int16_t obstacle_center,
                                         uint16_t obstacle_width)
{
    int16_t obstacle_left;
    int16_t obstacle_right;

    /* 当前行如果连有效赛道都没有，就只能先保留基础中心。 */
    if (left_edge < 0 || right_edge < 0 || right_edge <= left_edge) return base_center;

    /* 先把障碍物中心和宽度换算成当前行的障碍物左右边缘。 */
    obstacle_left = (int16_t)(obstacle_center - (int16_t)(obstacle_width / 2U));
    obstacle_right = (int16_t)(obstacle_center + (int16_t)(obstacle_width / 2U));

    if (pass_side == OBSTACLE_SIDE_RIGHT)
    {
        int16_t safe_left = (int16_t)(obstacle_right + (int16_t)OBSTACLE_CLEARANCE_PIXELS);
        int16_t safe_right = (int16_t)(right_edge + (int16_t)OBSTACLE_OUTSIDE_ALLOWANCE);

        /* 如果右侧可通行空间已经被压得很窄，就把目标中心顶到最右安全位置。 */
        if (safe_left >= safe_right)
        {
            return clamp_column(safe_right);
        }

        /* 否则把目标中心放到“右侧自由通道”的中间。 */
        return clamp_column((int16_t)((safe_left + safe_right) / 2));
    }

    if (pass_side == OBSTACLE_SIDE_LEFT)
    {
        int16_t safe_left = (int16_t)(left_edge - (int16_t)OBSTACLE_OUTSIDE_ALLOWANCE);
        int16_t safe_right = (int16_t)(obstacle_left - (int16_t)OBSTACLE_CLEARANCE_PIXELS);

        /* 左绕时镜像处理。 */
        if (safe_left >= safe_right)
        {
            return clamp_column(safe_left);
        }

        /* 否则把目标中心放到“左侧自由通道”的中间。 */
        return clamp_column((int16_t)((safe_left + safe_right) / 2));
    }

    /* 没有明确绕行方向时，就不要擅自偏移。 */
    return clamp_column(base_center);
}

/**
 * @brief 计算避障模式下的引导中心
 *
 * 用图像下部几行的自由通道目标中心做平均，
 * 得到一个更稳定的避障“总方向”。
 */
static int16_t calculate_obstacle_guide_center(const int16_t left_edges[IMAGE_HEIGHT],
                                               const int16_t right_edges[IMAGE_HEIGHT],
                                               const uint8_t valid_rows[IMAGE_HEIGHT],
                                               const int16_t mid_line[IMAGE_HEIGHT],
                                               Obstacle_Side pass_side,
                                               int16_t obstacle_center,
                                               uint16_t obstacle_width)
{
    uint16_t y;
    int32_t sum = 0;
    uint16_t count = 0U;

    /* 用下部多行的自由通道中心做平均，得到更稳的总体绕行方向。 */
    for (y = OBSTACLE_BLEND_START_ROW; y < IMAGE_HEIGHT; y++)
    {
        if (valid_rows[y])
        {
            int16_t base_center = get_track_center_at_row(mid_line, left_edges, right_edges, valid_rows, y);
            int16_t target_center = calculate_row_pass_target(left_edges[y],
                                                              right_edges[y],
                                                              base_center,
                                                              pass_side,
                                                              obstacle_center,
                                                              obstacle_width);

            sum += target_center;
            count++;
        }
    }

    /* 如果下部拿不到稳定统计，就退回普通巡线的默认引导中心。 */
    if (count == 0U) return calculate_default_guide_center(mid_line);
    return (int16_t)(sum / (int32_t)count);
}

/**
 * @brief 检测当前帧是否存在有效路障候选
 *
 * 识别逻辑是：
 * 1. 只在赛道内部搜索；
 * 2. 逐行寻找最长黑块；
 * 3. 要求黑块在纵向上大致连续，而不是东一块西一块；
 * 4. 再用高度、面积和偏心量过滤误检。
 */
static Obstacle_Candidate detect_obstacle_candidate(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                                    const int16_t mid_line[IMAGE_HEIGHT],
                                                    const int16_t left_edges[IMAGE_HEIGHT],
                                                    const int16_t right_edges[IMAGE_HEIGHT],
                                                    const uint8_t valid_rows[IMAGE_HEIGHT])
{
    /* 默认返回“空候选”，只有全部几何条件都成立才置 detected=1。 */
    Obstacle_Candidate candidate = {0U, OBSTACLE_SIDE_NONE, OBSTACLE_SIDE_NONE, 0U, 0U, 0U, 0U, 0U, 0U, -1, 0U, 0};
    uint16_t y;
    uint16_t hit_rows = 0U;
    int32_t center_sum = 0;
    int32_t track_center_sum = 0;
    uint16_t min_x = IMAGE_WIDTH;
    uint16_t max_x = 0U;
    uint16_t min_y = IMAGE_HEIGHT;
    uint16_t max_y = 0U;
    uint32_t area_sum = 0U;
    uint16_t min_run_width = 0xFFFFU;
    uint16_t max_run_width = 0U;
    uint16_t min_run_left = IMAGE_WIDTH;
    uint16_t max_run_left = 0U;
    uint16_t min_run_right = IMAGE_WIDTH;
    uint16_t max_run_right = 0U;
    uint8_t cluster_started = 0U;
    int16_t cluster_center = -1;

    /* 在限定的搜索窗口内逐行寻找“纵向连续”的内部黑块。 */
    for (y = OBSTACLE_SCAN_TOP_ROW; y <= OBSTACLE_SCAN_BOTTOM_ROW; y++)
    {
        if (!valid_rows[y]) continue;

        {
            uint16_t track_width = (uint16_t)(right_edges[y] - left_edges[y] + 1);

            /* 赛道宽度明显异常时，不在这一行上做路障判断。 */
            if (track_width < OBSTACLE_TRACK_MIN_WIDTH || track_width > OBSTACLE_TRACK_MAX_WIDTH) continue;
        }

        {
            uint16_t run_left;
            uint16_t run_right;
            uint16_t run_width = find_row_dark_run(image, y, left_edges[y], right_edges[y], &run_left, &run_right);

            if (run_width >= OBSTACLE_MIN_RUN_WIDTH && run_width <= OBSTACLE_MAX_RUN_WIDTH)
            {
                uint16_t left_gap = (uint16_t)(run_left - (uint16_t)left_edges[y]);
                uint16_t right_gap = (uint16_t)((uint16_t)right_edges[y] - run_right);

                /* 黑块两侧都要还留有赛道空间，否则更像赛道断裂而不是砖块。 */
                if (left_gap >= OBSTACLE_MIN_SIDE_GAP && right_gap >= OBSTACLE_MIN_SIDE_GAP)
                {
                    int16_t obstacle_center = (int16_t)((run_left + run_right) / 2U);
                    int16_t track_center = get_track_center_at_row(mid_line, left_edges, right_edges, valid_rows, y);

                    /* 黑块如果几乎压在赛道中心上，就不具备明确“左绕/右绕”意义。 */
                    if (abs_diff_i16(obstacle_center, track_center) >= OBSTACLE_SIDE_OFFSET_MIN)
                    {
                        if (cluster_started &&
                            abs_diff_i16(obstacle_center, cluster_center) > OBSTACLE_CLUSTER_CENTER_LIMIT)
                        {
                            /* 同一个障碍在连续多行里，中心不应该突然大跳。 */
                            continue;
                        }

                        /* 通过连续多行累积，得到一个稳定的障碍包围盒。 */
                        hit_rows++;
                        center_sum += obstacle_center;
                        track_center_sum += track_center;
                        area_sum += run_width;
                        cluster_started = 1U;
                        cluster_center = (int16_t)(center_sum / (int32_t)hit_rows);

                        /* 砖块在多行里应当近似矩形，因此宽度和左右边缘都不应大幅摆动。 */
                        if (run_width < min_run_width) min_run_width = run_width;
                        if (run_width > max_run_width) max_run_width = run_width;
                        if (run_left < min_run_left) min_run_left = run_left;
                        if (run_left > max_run_left) max_run_left = run_left;
                        if (run_right < min_run_right) min_run_right = run_right;
                        if (run_right > max_run_right) max_run_right = run_right;

                        if (run_left < min_x) min_x = run_left;
                        if (run_right > max_x) max_x = run_right;
                        if (y < min_y) min_y = y;
                        if (y > max_y) max_y = y;
                    }
                }
            }
        }
    }

    /* 命中行数太少，说明还不像一个稳定实体。 */
    if (hit_rows < OBSTACLE_MIN_ROWS) return candidate;

    /* 把所有命中行累积成一个整体包围盒。 */
    candidate.bbox_left = min_x;
    candidate.bbox_right = max_x;
    candidate.bbox_top = min_y;
    candidate.bbox_bottom = max_y;
    candidate.bbox_width = (uint16_t)(max_x - min_x + 1U);
    candidate.bbox_height = (uint16_t)(max_y - min_y + 1U);
    candidate.bbox_center = (int16_t)(center_sum / (int32_t)hit_rows);
    candidate.area = (uint16_t)area_sum;
    candidate.track_center = (int16_t)(track_center_sum / (int32_t)hit_rows);

    /* 再用高度和面积过滤太小的噪声目标。 */
    if (candidate.bbox_height < OBSTACLE_MIN_HEIGHT) return candidate;
    if (candidate.area < OBSTACLE_MIN_AREA) return candidate;
    /* 环岛内黑洞虽然也会形成赛道内部黑块，但它的宽度和左右边缘会随行号明显弧形变化。
     * 真正的砖块更接近矩形，因此这里要求多行之间的宽度和边缘漂移都不能太大。 */
    if ((uint16_t)(max_run_width - min_run_width) > OBSTACLE_MAX_WIDTH_SPAN) return candidate;
    if ((uint16_t)(max_run_left - min_run_left) > OBSTACLE_MAX_EDGE_DRIFT) return candidate;
    if ((uint16_t)(max_run_right - min_run_right) > OBSTACLE_MAX_EDGE_DRIFT) return candidate;

    /* 障碍在赛道中心左边，就从右侧绕；反之同理。 */
    if (candidate.bbox_center < candidate.track_center)
    {
        candidate.obstacle_side = OBSTACLE_SIDE_LEFT;
        candidate.pass_side = OBSTACLE_SIDE_RIGHT;
    }
    else
    {
        candidate.obstacle_side = OBSTACLE_SIDE_RIGHT;
        candidate.pass_side = OBSTACLE_SIDE_LEFT;
    }

    candidate.detected = 1U;
    return candidate;
}

/**
 * @brief 构造路障模式下的覆盖中线
 *
 * 这版中线的核心不是“轻轻偏一下”，而是：
 * 1. 先算出障碍外侧的自由通道；
 * 2. 再按状态阶段和接近程度逐步把中线拉向该通道；
 * 3. 绕行阶段允许中线少量偏出赛道边界，以优先保证不碰撞。
 */
static void build_obstacle_override_mid_line(const int16_t left_edges[IMAGE_HEIGHT],
                                             const int16_t right_edges[IMAGE_HEIGHT],
                                             const uint8_t valid_rows[IMAGE_HEIGHT],
                                             const int16_t mid_line[IMAGE_HEIGHT],
                                             int16_t obstacle_center,
                                             uint16_t obstacle_width,
                                             uint16_t bbox_bottom,
                                             Obstacle_State state,
                                             Obstacle_Side pass_side,
                                             int16_t guide_center,
                                             int16_t override_mid_line[IMAGE_HEIGHT])
{
    uint16_t y;
    uint16_t phase_percent = get_phase_blend_percent(state);
    uint16_t proximity_percent = get_proximity_percent(bbox_bottom);

    /* 逐行构造覆盖中线：近处偏得更多，远处偏得更缓。 */
    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        int16_t base_center = get_track_center_at_row(mid_line, left_edges, right_edges, valid_rows, y);
        int16_t target_center = guide_center;
        uint16_t row_percent;
        uint16_t total_percent;

        if (valid_rows[y])
        {
            /* 当前行如果仍有有效赛道，就按这一行自己的自由通道去算目标中心。 */
            target_center = calculate_row_pass_target(left_edges[y],
                                                      right_edges[y],
                                                      base_center,
                                                      pass_side,
                                                      obstacle_center,
                                                      obstacle_width);
        }

        /* 图像上部更靠近前方，优先偏过去；图像下部更靠近车体，偏移要更稳。 */
        if (y < OBSTACLE_BLEND_START_ROW) row_percent = 100U;
        else if (y <= bbox_bottom) row_percent = 70U;
        else if (state == OBSTACLE_STATE_AVOIDING) row_percent = 45U;
        else if (state == OBSTACLE_STATE_EXITING) row_percent = 25U;
        else row_percent = 0U;

        /* 先叠加状态机阶段强度，再叠加当前行的前视重要性。 */
        total_percent = (uint16_t)(((uint32_t)phase_percent * (uint32_t)row_percent) / 100U);

        if (state == OBSTACLE_STATE_APPROACHING)
        {
            uint16_t approach_gain = (uint16_t)(20U + (uint16_t)(((uint32_t)proximity_percent * 80U) / 100U));
            /* 接近阶段再根据障碍离车多近，逐步放大偏置。 */
            total_percent = (uint16_t)(((uint32_t)total_percent * (uint32_t)approach_gain) / 100U);
        }

        if (total_percent == 0U)
        {
            /* 这一行如果还不需要明显避障，就保持普通中心。 */
            override_mid_line[y] = clamp_column(base_center);
        }
        else
        {
            int32_t delta = (int32_t)target_center - (int32_t)base_center;
            int16_t blended = (int16_t)(base_center + (int16_t)(delta * (int32_t)total_percent / 100));
            /* 把普通中心按比例平滑拉向绕行目标中心。 */
            override_mid_line[y] = clamp_column(blended);
        }
    }
}

/**
 * @brief 初始化路障模块
 */
void obstacle_element_init(void)
{
    /* 清空路障状态机的全部上下文，回到普通巡线状态。 */
    g_obstacle_ctx.state = OBSTACLE_STATE_NONE;
    g_obstacle_ctx.detect_count = 0U;
    g_obstacle_ctx.lost_count = 0U;
    g_obstacle_ctx.exit_hold_count = 0U;
    g_obstacle_ctx.obstacle_side = OBSTACLE_SIDE_NONE;
    g_obstacle_ctx.pass_side = OBSTACLE_SIDE_NONE;
    g_obstacle_ctx.guide_center = (int16_t)(IMAGE_WIDTH / 2U);
    g_obstacle_ctx.bbox_left = 0U;
    g_obstacle_ctx.bbox_right = 0U;
    g_obstacle_ctx.bbox_bottom = 0U;
    g_obstacle_ctx.bbox_width = 0U;
    g_obstacle_ctx.bbox_height = 0U;
    g_obstacle_ctx.bbox_center = (int16_t)(IMAGE_WIDTH / 2U);

    g_obstacle_debug.state = OBSTACLE_STATE_NONE;
    g_obstacle_debug.obstacle_side = OBSTACLE_SIDE_NONE;
    g_obstacle_debug.pass_side = OBSTACLE_SIDE_NONE;
    g_obstacle_debug.obstacle_center = (int16_t)(IMAGE_WIDTH / 2U);
    g_obstacle_debug.obstacle_width = 0U;
    g_obstacle_debug.obstacle_height = 0U;
    g_obstacle_debug.obstacle_bottom_row = 0U;
    g_obstacle_debug.guide_center = (int16_t)(IMAGE_WIDTH / 2U);
    g_obstacle_debug.candidate_detected = 0U;
}

/**
 * @brief 路障识别与绕行主流程
 */
uint8_t obstacle_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                 const int16_t mid_line[IMAGE_HEIGHT],
                                 int16_t override_mid_line[IMAGE_HEIGHT],
                                 float *override_speed)
{
    /* 当前帧赛道边界。 */
    int16_t left_edges[IMAGE_HEIGHT];
    int16_t right_edges[IMAGE_HEIGHT];
    uint8_t valid_rows[IMAGE_HEIGHT];
    /* 当前帧单帧路障候选。 */
    Obstacle_Candidate candidate;

    /* 先提边界，再找路障候选。 */
    build_row_edges(image, left_edges, right_edges, valid_rows);
    candidate = detect_obstacle_candidate(image, mid_line, left_edges, right_edges, valid_rows);

    /* 先把单帧检测结果写进调试信息，方便串口观察。 */
    g_obstacle_debug.candidate_detected = candidate.detected;
    g_obstacle_debug.obstacle_side = candidate.obstacle_side;
    g_obstacle_debug.pass_side = candidate.pass_side;
    g_obstacle_debug.obstacle_center = candidate.bbox_center;
    g_obstacle_debug.obstacle_width = candidate.bbox_width;
    g_obstacle_debug.obstacle_height = candidate.bbox_height;
    g_obstacle_debug.obstacle_bottom_row = candidate.bbox_bottom;

    switch (g_obstacle_ctx.state)
    {
        case OBSTACLE_STATE_NONE:
            if (candidate.detected)
            {
                /* 绕行方向变了，说明前后两次候选不一致，重新累计确认帧。 */
                if (g_obstacle_ctx.pass_side != candidate.pass_side) g_obstacle_ctx.detect_count = 0U;

                /* 在空闲态里先累计几帧，避免一帧噪声就误触发。 */
                g_obstacle_ctx.detect_count++;
                g_obstacle_ctx.lost_count = 0U;
                g_obstacle_ctx.obstacle_side = candidate.obstacle_side;
                g_obstacle_ctx.pass_side = candidate.pass_side;
                g_obstacle_ctx.guide_center = calculate_obstacle_guide_center(left_edges,
                                                                               right_edges,
                                                                               valid_rows,
                                                                               mid_line,
                                                                               candidate.pass_side,
                                                                               candidate.bbox_center,
                                                                               candidate.bbox_width);
                g_obstacle_ctx.bbox_left = candidate.bbox_left;
                g_obstacle_ctx.bbox_right = candidate.bbox_right;
                g_obstacle_ctx.bbox_bottom = candidate.bbox_bottom;
                g_obstacle_ctx.bbox_width = candidate.bbox_width;
                g_obstacle_ctx.bbox_height = candidate.bbox_height;
                g_obstacle_ctx.bbox_center = candidate.bbox_center;

                if (g_obstacle_ctx.detect_count >= OBSTACLE_DETECT_CONFIRM_FRAMES)
                {
                    /* 连续命中足够帧后，进入“提前减速并预偏置”的接近阶段。 */
                    g_obstacle_ctx.state = OBSTACLE_STATE_APPROACHING;
                    g_obstacle_ctx.exit_hold_count = 0U;
                }
            }
            else
            {
                /* 空闲态下没命中就继续保持普通巡线。 */
                g_obstacle_ctx.detect_count = 0U;
                g_obstacle_ctx.obstacle_side = OBSTACLE_SIDE_NONE;
                g_obstacle_ctx.pass_side = OBSTACLE_SIDE_NONE;
            }
            break;

        case OBSTACLE_STATE_APPROACHING:
            if (candidate.detected && candidate.pass_side == g_obstacle_ctx.pass_side)
            {
                /* 接近阶段如果持续看到同一障碍，就持续刷新其位置和绕行方向。 */
                g_obstacle_ctx.lost_count = 0U;
                g_obstacle_ctx.obstacle_side = candidate.obstacle_side;
                g_obstacle_ctx.guide_center = calculate_obstacle_guide_center(left_edges,
                                                                               right_edges,
                                                                               valid_rows,
                                                                               mid_line,
                                                                               candidate.pass_side,
                                                                               candidate.bbox_center,
                                                                               candidate.bbox_width);
                g_obstacle_ctx.bbox_left = candidate.bbox_left;
                g_obstacle_ctx.bbox_right = candidate.bbox_right;
                g_obstacle_ctx.bbox_bottom = candidate.bbox_bottom;
                g_obstacle_ctx.bbox_width = candidate.bbox_width;
                g_obstacle_ctx.bbox_height = candidate.bbox_height;
                g_obstacle_ctx.bbox_center = candidate.bbox_center;

                if (candidate.bbox_bottom >= OBSTACLE_AVOID_TRIGGER_ROW)
                {
                    /* 障碍已经离车够近，切入正式绕行。 */
                    g_obstacle_ctx.state = OBSTACLE_STATE_AVOIDING;
                }
            }
            else
            {
                /* 接近阶段丢失候选时，先给几帧容错。 */
                g_obstacle_ctx.lost_count++;
                if (g_obstacle_ctx.lost_count >= OBSTACLE_LOST_CONFIRM_FRAMES)
                {
                    /* 连续丢失足够久，说明大概率不是有效障碍，直接回普通巡线。 */
                    obstacle_element_init();
                }
            }
            break;

        case OBSTACLE_STATE_AVOIDING:
            if (candidate.detected && candidate.pass_side == g_obstacle_ctx.pass_side)
            {
                /* 正式绕行阶段持续刷新障碍位置，防止中线仍按旧包围盒规划。 */
                g_obstacle_ctx.lost_count = 0U;
                g_obstacle_ctx.obstacle_side = candidate.obstacle_side;
                g_obstacle_ctx.guide_center = calculate_obstacle_guide_center(left_edges,
                                                                               right_edges,
                                                                               valid_rows,
                                                                               mid_line,
                                                                               candidate.pass_side,
                                                                               candidate.bbox_center,
                                                                               candidate.bbox_width);
                g_obstacle_ctx.bbox_left = candidate.bbox_left;
                g_obstacle_ctx.bbox_right = candidate.bbox_right;
                g_obstacle_ctx.bbox_bottom = candidate.bbox_bottom;
                g_obstacle_ctx.bbox_width = candidate.bbox_width;
                g_obstacle_ctx.bbox_height = candidate.bbox_height;
                g_obstacle_ctx.bbox_center = candidate.bbox_center;
            }
            else
            {
                /* 绕行中看不到障碍后，不立刻回正，而是先等确认。 */
                g_obstacle_ctx.lost_count++;
                if (g_obstacle_ctx.lost_count >= OBSTACLE_LOST_CONFIRM_FRAMES)
                {
                    /* 连续丢失说明车头大概率已经越过障碍，进入出障阶段。 */
                    g_obstacle_ctx.state = OBSTACLE_STATE_EXITING;
                    g_obstacle_ctx.exit_hold_count = 0U;
                }
            }
            break;

        case OBSTACLE_STATE_EXITING:
            if (candidate.detected && candidate.pass_side == g_obstacle_ctx.pass_side)
            {
                /* 出障期间如果又重新看到同方向障碍，说明还没真正绕过去，回到正式绕行。 */
                g_obstacle_ctx.state = OBSTACLE_STATE_AVOIDING;
                g_obstacle_ctx.lost_count = 0U;
                g_obstacle_ctx.guide_center = calculate_obstacle_guide_center(left_edges,
                                                                               right_edges,
                                                                               valid_rows,
                                                                               mid_line,
                                                                               candidate.pass_side,
                                                                               candidate.bbox_center,
                                                                               candidate.bbox_width);
                g_obstacle_ctx.bbox_left = candidate.bbox_left;
                g_obstacle_ctx.bbox_right = candidate.bbox_right;
                g_obstacle_ctx.bbox_bottom = candidate.bbox_bottom;
                g_obstacle_ctx.bbox_width = candidate.bbox_width;
                g_obstacle_ctx.bbox_height = candidate.bbox_height;
                g_obstacle_ctx.bbox_center = candidate.bbox_center;
            }
            else
            {
                /* 出障阶段额外保持几帧偏置，让车更平顺地回到普通巡线。 */
                g_obstacle_ctx.exit_hold_count++;
                if (g_obstacle_ctx.exit_hold_count >= OBSTACLE_EXIT_HOLD_FRAMES)
                {
                    /* 保持结束后，完整退出避障模块。 */
                    obstacle_element_init();
                }
            }
            break;

        default:
            obstacle_element_init();
            break;
    }

    /* 输出当前已经确认的状态机信息，而不是只输出单帧候选。 */
    g_obstacle_debug.state = g_obstacle_ctx.state;
    g_obstacle_debug.obstacle_side = g_obstacle_ctx.obstacle_side;
    g_obstacle_debug.pass_side = g_obstacle_ctx.pass_side;
    g_obstacle_debug.obstacle_center = g_obstacle_ctx.bbox_center;
    g_obstacle_debug.obstacle_width = g_obstacle_ctx.bbox_width;
    g_obstacle_debug.obstacle_height = g_obstacle_ctx.bbox_height;
    g_obstacle_debug.obstacle_bottom_row = g_obstacle_ctx.bbox_bottom;
    g_obstacle_debug.guide_center = g_obstacle_ctx.guide_center;

    if (g_obstacle_ctx.state == OBSTACLE_STATE_NONE)
    {
        /* 没进入路障状态时，不覆盖普通巡线控制。 */
        return 0U;
    }

    /* 进入路障状态后，生成本帧专用覆盖中线。 */
    build_obstacle_override_mid_line(left_edges,
                                     right_edges,
                                     valid_rows,
                                     mid_line,
                                     g_obstacle_ctx.bbox_center,
                                     g_obstacle_ctx.bbox_width,
                                     g_obstacle_ctx.bbox_bottom,
                                     g_obstacle_ctx.state,
                                     g_obstacle_ctx.pass_side,
                                     g_obstacle_ctx.guide_center,
                                     override_mid_line);

    if (g_obstacle_ctx.state == OBSTACLE_STATE_APPROACHING)
    {
        uint16_t proximity_percent = get_proximity_percent(g_obstacle_ctx.bbox_bottom);
        float delta_speed = (float)(OBSTACLE_APPROACH_TARGET_SPEED - OBSTACLE_AVOID_TARGET_SPEED);
        /* 接近阶段速度随距离线性下滑，越靠近障碍越接近正式绕行速度。 */
        *override_speed = OBSTACLE_APPROACH_TARGET_SPEED - delta_speed * ((float)proximity_percent / 100.0f);
    }
    else if (g_obstacle_ctx.state == OBSTACLE_STATE_EXITING)
    {
        /* 出障阶段速度先回到一个中间值，避免刚绕开就猛加速。 */
        *override_speed = (OBSTACLE_APPROACH_TARGET_SPEED + OBSTACLE_AVOID_TARGET_SPEED) * 0.5f;
    }
    else
    {
        /* 正式绕行阶段使用最保守速度。 */
        *override_speed = OBSTACLE_AVOID_TARGET_SPEED;
    }

    return 1U;
}

/**
 * @brief 获取当前路障状态
 * @return Obstacle_State 当前状态机状态
 */
Obstacle_State obstacle_element_get_state(void)
{
    return g_obstacle_ctx.state;
}

/**
 * @brief 获取当前路障调试信息
 * @return Obstacle_Debug_Info 当前调试信息快照
 */
Obstacle_Debug_Info obstacle_element_get_debug_info(void)
{
    return g_obstacle_debug;
}
