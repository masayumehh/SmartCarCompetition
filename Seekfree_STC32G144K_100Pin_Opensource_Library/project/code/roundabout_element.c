#include "roundabout_element.h"

/* 图像下部参考区域的起始行。 */
#define ROUNDABOUT_LOWER_START_ROW       ((IMAGE_HEIGHT * TUNE_ROUNDABOUT_LOWER_START_RATIO_NUM) / TUNE_ROUNDABOUT_LOWER_START_RATIO_DEN)
/* 正常直道宽度下限。 */
#define ROUNDABOUT_NORMAL_MIN_WIDTH      TUNE_ROUNDABOUT_NORMAL_MIN_WIDTH
/* 正常直道宽度上限。 */
#define ROUNDABOUT_NORMAL_MAX_WIDTH      TUNE_ROUNDABOUT_NORMAL_MAX_WIDTH
/* 下部参考区域至少需要的有效行数。 */
#define ROUNDABOUT_REF_MIN_ROWS          TUNE_ROUNDABOUT_REF_MIN_ROWS
/* 下部中心允许的最大波动。 */
#define ROUNDABOUT_CENTER_DIFF_LIMIT     TUNE_ROUNDABOUT_CENTER_DIFF_LIMIT
/* 稳定边允许的最大抖动。 */
#define ROUNDABOUT_STABLE_EDGE_TOLERANCE TUNE_ROUNDABOUT_STABLE_EDGE_TOLERANCE
/* 稳定边成立所需的最少稳定行数。 */
#define ROUNDABOUT_MIN_STABLE_ROWS       TUNE_ROUNDABOUT_MIN_STABLE_ROWS
/* 判定异常边开始失真的最小偏移量。 */
#define ROUNDABOUT_ENTRY_OFFSET_MIN      TUNE_ROUNDABOUT_ENTRY_OFFSET_MIN
/* 找 B 点时要求连续恢复的最少行数。 */
#define ROUNDABOUT_RECOVER_MIN_ROWS      TUNE_ROUNDABOUT_RECOVER_MIN_ROWS
/* B 点相对 A 点或参考边的最小偏移。 */
#define ROUNDABOUT_B_POINT_MIN_OFFSET    TUNE_ROUNDABOUT_B_POINT_MIN_OFFSET
/* C 点相对 B 点的最小偏移。 */
#define ROUNDABOUT_C_POINT_MIN_OFFSET    TUNE_ROUNDABOUT_C_POINT_MIN_OFFSET
/* 异常边连续缺失的最少行数。 */
#define ROUNDABOUT_MIN_MISSING_ROWS      TUNE_ROUNDABOUT_MIN_MISSING_ROWS
/* 补线宽度最小值。 */
#define ROUNDABOUT_SUPPLEMENT_MIN_WIDTH  TUNE_ROUNDABOUT_SUPPLEMENT_MIN_WIDTH
/* 补线宽度最大值。 */
#define ROUNDABOUT_SUPPLEMENT_MAX_WIDTH  TUNE_ROUNDABOUT_SUPPLEMENT_MAX_WIDTH
/* 进入环岛前要求连续命中的帧数。 */
#define ROUNDABOUT_DETECT_CONFIRM_FRAMES TUNE_ROUNDABOUT_DETECT_CONFIRM_FRAMES
/* 环岛过程中连续丢失候选的允许帧数。 */
#define ROUNDABOUT_LOST_CONFIRM_FRAMES   TUNE_ROUNDABOUT_LOST_CONFIRM_FRAMES
/* 连续恢复正常直道后，确认开始出环的帧数。 */
#define ROUNDABOUT_EXIT_CONFIRM_FRAMES   TUNE_ROUNDABOUT_EXIT_CONFIRM_FRAMES
/* 出环状态额外保持的帧数。 */
#define ROUNDABOUT_EXIT_HOLD_FRAMES      TUNE_ROUNDABOUT_EXIT_HOLD_FRAMES
/* 判定已经恢复正常直道所需的有效行数。 */
#define ROUNDABOUT_EXIT_NORMAL_ROWS      TUNE_ROUNDABOUT_EXIT_NORMAL_ROWS
/* 判断趋势时允许的小抖动。 */
#define ROUNDABOUT_EDGE_JITTER_TOLERANCE TUNE_ROUNDABOUT_EDGE_JITTER_TOLERANCE
/* 环岛专用目标速度。 */
#define ROUNDABOUT_SPECIAL_TARGET_SPEED  TUNE_ROUNDABOUT_SPECIAL_TARGET_SPEED

/**
 * @brief 单帧环岛候选结构体
 *
 * 该结构体只描述“当前这一帧像不像环岛”，
 * 不负责跨帧状态保持。
 */
typedef struct
{
    uint8_t detected;               /* 当前帧是否命中环岛候选。 */
    Roundabout_Direction direction; /* 当前候选方向。 */
    Roundabout_Side stable_side;    /* 当前候选中的稳定边侧。 */
    uint16_t lower_width;           /* 下部参考区域平均宽度。 */
    uint16_t stable_edge;           /* 稳定边参考位置。 */
    uint16_t missing_rows;          /* 异常边连续缺失的行数。 */
    int16_t arc_peak;               /* C 点所在弧顶列号。 */
    uint16_t supplement_width;      /* 当前环岛使用的补线宽度。 */
    int16_t reference_left;         /* 下部参考左边界。 */
    int16_t reference_right;        /* 下部参考右边界。 */
    int16_t guide_center;           /* 下部平均引导中心。 */
    Roundabout_Key_Point point_a;   /* A 点。 */
    Roundabout_Key_Point point_b;   /* B 点。 */
    Roundabout_Key_Point point_c;   /* C 点。 */
} Roundabout_Candidate;

/**
 * @brief 环岛状态机上下文
 *
 * 用于跨帧保存当前已确认的环岛方向、关键点、
 * 补线宽度和退出计数等信息。
 */
typedef struct
{
    Roundabout_State state;         /* 当前环岛状态机状态。 */
    Roundabout_Direction direction; /* 当前确认方向。 */
    Roundabout_Side stable_side;    /* 当前确认稳定边侧。 */
    uint8_t detect_count;           /* 连续命中环岛候选的帧数。 */
    uint8_t lost_count;             /* 连续未命中候选的帧数。 */
    uint8_t normal_count;           /* 连续恢复正常直道的帧数。 */
    uint8_t exit_hold_count;        /* 出环保持帧数。 */
    uint16_t lower_width;           /* 已确认的下部参考宽度。 */
    uint16_t supplement_width;      /* 已确认的补线宽度。 */
    int16_t reference_left;         /* 已确认的下部参考左边界。 */
    int16_t reference_right;        /* 已确认的下部参考右边界。 */
    int16_t guide_center;           /* 当前引导中心。 */
    Roundabout_Key_Point point_a;   /* 已确认的 A 点。 */
    Roundabout_Key_Point point_b;   /* 已确认的 B 点。 */
    Roundabout_Key_Point point_c;   /* 已确认的 C 点。 */
} Roundabout_Context;

/* 全局环岛状态机对象。 */
static Roundabout_Context g_roundabout_ctx =
{
    ROUNDABOUT_STATE_NONE,
    ROUNDABOUT_DIRECTION_NONE,
    ROUNDABOUT_SIDE_NONE,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0,
    0,
    (int16_t)(IMAGE_WIDTH / 2U),
    {-1, -1, 0U},
    {-1, -1, 0U},
    {-1, -1, 0U}
};

/* 全局环岛调试信息对象。 */
static Roundabout_Debug_Info g_roundabout_debug =
{
    ROUNDABOUT_STATE_NONE,
    ROUNDABOUT_DIRECTION_NONE,
    ROUNDABOUT_SIDE_NONE,
    0U,
    0U,
    0U,
    -1,
    0U,
    (int16_t)(IMAGE_WIDTH / 2U),
    0U,
    {-1, -1, 0U},
    {-1, -1, 0U},
    {-1, -1, 0U}
};

/**
 * @brief 构造一个无效关键点
 * @return Roundabout_Key_Point 无效关键点
 */
static Roundabout_Key_Point make_invalid_point(void)
{
    Roundabout_Key_Point point = {-1, -1, 0U};
    return point;
}

/**
 * @brief 计算两个无符号数的绝对差
 * @param a 数值 a
 * @param b 数值 b
 * @return uint16_t 绝对差值
 */
static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

/**
 * @brief 将列号限制在图像范围内
 * @param value 输入列号
 * @return int16_t 裁剪后的合法列号
 */
static int16_t clamp_column(int16_t value)
{
    if (value < 0) return 0;
    if (value >= (int16_t)IMAGE_WIDTH) return (int16_t)(IMAGE_WIDTH - 1);
    return value;
}

/**
 * @brief 在指定行中，从种子列附近寻找最近的白色像素
 *
 * 这一步的意义是：
 * 1. 不再全行扫“最左白点、最右白点”；
 * 2. 而是围绕中线或上一行中心，锁定当前真正的赛道连通区域。
 *
 * @param image 当前帧二值图
 * @param row 当前行号
 * @param seed 搜索种子列
 * @param center_pixel 输出：找到的白色像素列号
 * @return uint8_t 返回 1 表示找到，返回 0 表示本行未找到连通白区
 */
static uint8_t find_nearest_white_from_seed(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                            uint16_t row,
                                            int16_t seed,
                                            int16_t *center_pixel)
{
    int16_t offset;

    if (seed < 0) seed = 0;
    if (seed >= (int16_t)IMAGE_WIDTH) seed = (int16_t)(IMAGE_WIDTH - 1);

    if (image[row][seed] > 0U)
    {
        *center_pixel = seed;
        return 1U;
    }

    for (offset = 1; offset < (int16_t)IMAGE_WIDTH; offset++)
    {
        int16_t left = (int16_t)(seed - offset);
        int16_t right = (int16_t)(seed + offset);

        if (left >= 0 && image[row][left] > 0U)
        {
            *center_pixel = left;
            return 1U;
        }

        if (right < (int16_t)IMAGE_WIDTH && image[row][right] > 0U)
        {
            *center_pixel = right;
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 构造“真实连通赛道区域”的左右边线
 *
 * 新版环岛识别不再使用整行最外包边界，
 * 而是围绕当前中线或上一行中心，寻找当前真正的赛道连通白区，
 * 再从这个白区向左右扩展得到真实左右边线。
 *
 * @param image 当前帧二值图
 * @param mid_line 普通巡线中线
 * @param left_edges 输出：每行真实左边线
 * @param right_edges 输出：每行真实右边线
 * @param centers 输出：每行真实中心
 * @param valid_rows 输出：每行是否有效
 */
static void build_connected_track_edges(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                        const int16_t mid_line[IMAGE_HEIGHT],
                                        int16_t left_edges[IMAGE_HEIGHT],
                                        int16_t right_edges[IMAGE_HEIGHT],
                                        int16_t centers[IMAGE_HEIGHT],
                                        uint8_t valid_rows[IMAGE_HEIGHT])
{
    uint8_t last_center_valid = 0U;                     /* 是否已经建立起可持续跟踪的上一行中心。 */
    int16_t y;  /* 当前处理行号，按从下到上的顺序扫描。 */
    int16_t last_center = (int16_t)(IMAGE_WIDTH / 2U);  /* 上一条有效赛道中心。 */

    for (y = (int16_t)(IMAGE_HEIGHT - 1); y >= 0; y--)
    {
        int16_t seed = last_center;  /* 当前行搜索白区的起始种子。 */
        int16_t center_pixel;        /* 当前行找到的白区中心像素。 */

        left_edges[y] = -1;   /* 默认当前行左边界无效。 */
        right_edges[y] = -1;  /* 默认当前行右边界无效。 */
        centers[y] = -1;      /* 默认当前行中心无效。 */
        valid_rows[y] = 0U;   /* 默认当前行不是有效赛道行。 */

        if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH) seed = mid_line[y];  /* 优先使用普通中线作为搜索种子。 */

        if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH && last_center_valid &&
            abs_diff_u16((uint16_t)mid_line[y], (uint16_t)last_center) > ROUNDABOUT_NORMAL_MAX_WIDTH)
        {
            seed = last_center;  /* 普通中线跳变过大时，继续沿用上一行真实中心。 */
        }

        if (find_nearest_white_from_seed(image, (uint16_t)y, seed, &center_pixel))
        {
            last_center_valid = 1U;  /* 一旦找到当前行连通白区，就允许后续行继续沿用该中心跟踪。 */
            int16_t left = center_pixel;   /* 从中心向左扩展的游标。 */
            int16_t right = center_pixel;  /* 从中心向右扩展的游标。 */

            while (left > 0 && image[y][left - 1] > 0U) left--;  /* 一直向左走到赛道白区边界。 */
            while (right < (int16_t)(IMAGE_WIDTH - 1) && image[y][right + 1] > 0U) right++;  /* 一直向右走到赛道白区边界。 */

            if (right > left)
            {
                left_edges[y] = left;                            /* 保存真实左边界。 */
                right_edges[y] = right;                          /* 保存真实右边界。 */
                centers[y] = (int16_t)((left + right) / 2);      /* 保存当前行真实中心。 */
                valid_rows[y] = 1U;                              /* 标记当前行为有效赛道行。 */
                last_center = centers[y];                        /* 更新下一行使用的种子中心。 */
            }
        }
    }
}

/**
 * @brief 计算图像下部引导中心
 *
 * 当某些行补线失败时，用这个下部平均中心作为兜底中心。
 */
static int16_t calculate_guide_center(const int16_t centers[IMAGE_HEIGHT])
{
    uint16_t y;
    int32_t sum = 0;
    uint16_t count = 0U;

    for (y = ROUNDABOUT_LOWER_START_ROW; y < IMAGE_HEIGHT; y++)
    {
        if (centers[y] >= 0 && centers[y] < IMAGE_WIDTH)
        {
            sum += centers[y];
            count++;
        }
    }

    if (count == 0U) return (int16_t)(IMAGE_WIDTH / 2U);
    return (int16_t)(sum / (int32_t)count);
}

/**
 * @brief 统计图像下部的正常直道参考量
 *
 * 只有当前帧下部看起来像一段正常直道，
 * 才继续往上判定是否进入环岛。
 */
static uint8_t get_lower_reference(const int16_t left_edges[IMAGE_HEIGHT],
                                   const int16_t right_edges[IMAGE_HEIGHT],
                                   const int16_t centers[IMAGE_HEIGHT],
                                   const uint8_t valid_rows[IMAGE_HEIGHT],
                                   int16_t *left_ref,
                                   int16_t *right_ref,
                                   uint16_t *width_ref,
                                   int16_t *guide_center)
{
    uint16_t y;              /* 当前统计行号。 */
    int32_t left_sum = 0;    /* 左边界总和。 */
    int32_t right_sum = 0;   /* 右边界总和。 */
    int32_t width_sum = 0;   /* 宽度总和。 */
    int32_t center_sum = 0;  /* 中心总和。 */
    uint16_t count = 0U;     /* 有效参考行数。 */
    int16_t center_min = (int16_t)IMAGE_WIDTH;  /* 下部中心最小值。 */
    int16_t center_max = -1;                    /* 下部中心最大值。 */

    for (y = ROUNDABOUT_LOWER_START_ROW; y < IMAGE_HEIGHT; y++)
    {
        if (valid_rows[y])
        {
            uint16_t width = (uint16_t)(right_edges[y] - left_edges[y] + 1);

            left_sum += left_edges[y];   /* 累加左边界。 */
            right_sum += right_edges[y]; /* 累加右边界。 */
            width_sum += width;          /* 累加赛道宽度。 */
            center_sum += centers[y];    /* 累加赛道中心。 */
            count++;

            if (centers[y] < center_min) center_min = centers[y];  /* 更新最小中心。 */
            if (centers[y] > center_max) center_max = centers[y];  /* 更新最大中心。 */
        }
    }

    if (count < ROUNDABOUT_REF_MIN_ROWS) return 0U;  /* 有效参考行太少，不继续判环岛。 */

    *left_ref = (int16_t)(left_sum / (int32_t)count);       /* 计算平均左边界。 */
    *right_ref = (int16_t)(right_sum / (int32_t)count);     /* 计算平均右边界。 */
    *width_ref = (uint16_t)(width_sum / (int32_t)count);    /* 计算平均宽度。 */
    *guide_center = (int16_t)(center_sum / (int32_t)count); /* 计算平均中心。 */

    if (*width_ref < ROUNDABOUT_NORMAL_MIN_WIDTH || *width_ref > ROUNDABOUT_NORMAL_MAX_WIDTH) return 0U;  /* 宽度不像正常直道。 */
    if (center_max < center_min) return 0U;  /* 没有形成有效中心统计。 */
    if ((uint16_t)(center_max - center_min) > ROUNDABOUT_CENTER_DIFF_LIMIT) return 0U;  /* 下部中心抖动过大。 */

    return 1U;
}

/**
 * @brief 读取稳定边在当前行的边界值
 * @param stable_side 稳定边侧
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param row 当前行号
 * @return int16_t 当前行稳定边位置
 */
static int16_t get_stable_edge_at(Roundabout_Side stable_side,
                                  const int16_t left_edges[IMAGE_HEIGHT],
                                  const int16_t right_edges[IMAGE_HEIGHT],
                                  uint16_t row)
{
    if (stable_side == ROUNDABOUT_SIDE_LEFT) return left_edges[row];
    if (stable_side == ROUNDABOUT_SIDE_RIGHT) return right_edges[row];
    return -1;
}

/**
 * @brief 读取异常边在当前行的边界值
 * @param stable_side 稳定边侧
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param row 当前行号
 * @return int16_t 当前行异常边位置
 */
static int16_t get_abnormal_edge_at(Roundabout_Side stable_side,
                                    const int16_t left_edges[IMAGE_HEIGHT],
                                    const int16_t right_edges[IMAGE_HEIGHT],
                                    uint16_t row)
{
    if (stable_side == ROUNDABOUT_SIDE_LEFT) return right_edges[row];
    if (stable_side == ROUNDABOUT_SIDE_RIGHT) return left_edges[row];
    return -1;
}

/**
 * @brief 判断当前行稳定边是否仍然可靠
 * @param edge_value 当前行稳定边位置
 * @param edge_ref 下部参考稳定边位置
 * @return uint8_t 返回 1 表示当前行稳定边仍可靠
 */
static uint8_t is_stable_edge_ok(int16_t edge_value, int16_t edge_ref)
{
    if (edge_value < 0) return 0U;
    if (abs_diff_u16((uint16_t)edge_value, (uint16_t)edge_ref) > ROUNDABOUT_STABLE_EDGE_TOLERANCE) return 0U;
    return 1U;
}

/**
 * @brief 判断当前帧是否已恢复为正常直道
 *
 * 当环岛结束后，图像上部和中部会重新出现：
 * 1. 左右边线都完整；
 * 2. 宽度回到正常范围；
 * 3. 中心波动明显减小。
 *
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param centers 中心数组
 * @param valid_rows 有效行标记
 * @return uint8_t 返回 1 表示当前帧更像正常直道
 */
static uint8_t is_track_back_to_normal(const int16_t left_edges[IMAGE_HEIGHT],
                                       const int16_t right_edges[IMAGE_HEIGHT],
                                       const int16_t centers[IMAGE_HEIGHT],
                                       const uint8_t valid_rows[IMAGE_HEIGHT])
{
    uint16_t y;
    uint16_t normal_rows = 0U;                   /* 上部/中部恢复正常的有效行数。 */
    int16_t center_min = (int16_t)IMAGE_WIDTH;   /* 恢复区域中心最小值。 */
    int16_t center_max = -1;                     /* 恢复区域中心最大值。 */

    for (y = 0U; y < ROUNDABOUT_LOWER_START_ROW; y++)
    {
        if (valid_rows[y])
        {
            uint16_t width = (uint16_t)(right_edges[y] - left_edges[y] + 1);

            if (width >= ROUNDABOUT_NORMAL_MIN_WIDTH && width <= ROUNDABOUT_NORMAL_MAX_WIDTH)
            {
                normal_rows++;

                if (centers[y] < center_min) center_min = centers[y];
                if (centers[y] > center_max) center_max = centers[y];
            }
        }
    }

    if (normal_rows < ROUNDABOUT_EXIT_NORMAL_ROWS) return 0U;  /* 恢复正常的有效行数还不够。 */
    if (center_max < center_min) return 0U;                    /* 没有形成有效中心统计。 */
    if ((uint16_t)(center_max - center_min) > (uint16_t)(ROUNDABOUT_CENTER_DIFF_LIMIT * 2U)) return 0U;  /* 中心仍然抖动较大。 */

    return 1U;
}

/**
 * @brief 基于稳定边侧检测单帧环岛候选
 *
 * 识别逻辑按你的思路拆成三步：
 * 1. 先确定一侧边线稳定、单调，可作为参考边；
 * 2. 再在另一侧寻找 A 点和 B 点；
 * 3. 最后继续向上寻找 C 点，得到完整入口几何。
 *
 * @param stable_side 假设的稳定边侧
 * @param left_edges 左边界数组
 * @param right_edges 右边界数组
 * @param valid_rows 有效行标记
 * @param left_ref 下部参考左边界
 * @param right_ref 下部参考右边界
 * @param width_ref 下部参考宽度
 * @param guide_center 下部平均中心
 * @return Roundabout_Candidate 当前稳定边假设下的候选结果
 */
static Roundabout_Candidate detect_candidate_by_stable_side(Roundabout_Side stable_side,
                                                            const int16_t left_edges[IMAGE_HEIGHT],
                                                            const int16_t right_edges[IMAGE_HEIGHT],
                                                            const uint8_t valid_rows[IMAGE_HEIGHT],
                                                            int16_t left_ref,
                                                            int16_t right_ref,
                                                            uint16_t width_ref,
                                                            int16_t guide_center)
{
    Roundabout_Candidate candidate;       /* 当前方向下的候选结果。 */
    int16_t stable_ref;                   /* 当前稳定边参考位置。 */
    int16_t abnormal_ref;                 /* 当前异常边参考位置。 */
    int16_t y;                            /* 从下往上扫描的行号。 */
    uint16_t stable_rows = 0U;            /* 稳定边成立的总行数。 */
    uint8_t phase = 0U;                   /* 0=找 A，1=找 B，2=找 C。 */
    int16_t last_normal_row = -1;         /* 异常边还正常时的最后一行。 */
    int16_t last_normal_edge = -1;        /* 异常边还正常时的最后一个位置。 */
    uint16_t recover_streak = 0U;         /* 连续恢复行数。 */
    int16_t recover_start_row = -1;       /* 恢复段的起始行。 */
    int16_t recover_start_edge = -1;      /* 恢复段的起始边线位置。 */

    candidate.detected = 0U;
    candidate.direction = ROUNDABOUT_DIRECTION_NONE;
    candidate.stable_side = stable_side;
    candidate.lower_width = width_ref;
    candidate.stable_edge = 0U;
    candidate.missing_rows = 0U;
    candidate.arc_peak = -1;
    candidate.supplement_width = 0U;
    candidate.reference_left = left_ref;
    candidate.reference_right = right_ref;
    candidate.guide_center = guide_center;
    candidate.point_a = make_invalid_point();
    candidate.point_b = make_invalid_point();
    candidate.point_c = make_invalid_point();

    if (stable_side == ROUNDABOUT_SIDE_LEFT)
    {
        stable_ref = left_ref;
        abnormal_ref = right_ref;
        candidate.direction = ROUNDABOUT_DIRECTION_CLOCKWISE;
        candidate.stable_edge = (uint16_t)left_ref;
    }
    else if (stable_side == ROUNDABOUT_SIDE_RIGHT)
    {
        stable_ref = right_ref;
        abnormal_ref = left_ref;
        candidate.direction = ROUNDABOUT_DIRECTION_COUNTERCLOCKWISE;
        candidate.stable_edge = (uint16_t)right_ref;
    }
    else
    {
        return candidate;
    }

    for (y = (int16_t)(ROUNDABOUT_LOWER_START_ROW - 1U); y >= 0; y--)
    {
        int16_t stable_edge = get_stable_edge_at(stable_side, left_edges, right_edges, (uint16_t)y);      /* 当前行稳定边位置。 */
        int16_t abnormal_edge = get_abnormal_edge_at(stable_side, left_edges, right_edges, (uint16_t)y);  /* 当前行异常边位置。 */
        uint8_t stable_ok = is_stable_edge_ok(stable_edge, stable_ref);                                    /* 当前行稳定边是否可信。 */
        uint8_t abnormal_valid = (abnormal_edge >= 0 && valid_rows[y]);                                    /* 当前行异常边是否存在。 */
        uint8_t abnormal_far = 0U;                                                                          /* 当前行异常边是否已明显偏离参考。 */

        if (!stable_ok)
        {
            if (phase == 0U) continue;  /* 在真正进入环岛前，稳定边偶发不可靠时先跳过这一行。 */
            break;                      /* 一旦进入候选阶段，稳定边失效就终止本次候选。 */
        }

        stable_rows++;  /* 统计稳定边成立的总行数。 */

        if (abnormal_valid)
        {
            if (abs_diff_u16((uint16_t)abnormal_edge, (uint16_t)abnormal_ref) >= ROUNDABOUT_ENTRY_OFFSET_MIN)
            {
                abnormal_far = 1U;  /* 异常边已经明显偏离直道参考。 */
            }
        }

        if (phase == 0U)
        {
            if (abnormal_valid && !abnormal_far)
            {
                last_normal_row = y;       /* 记录异常边仍然正常时的最后一行。 */
                last_normal_edge = abnormal_edge;  /* 记录异常边仍然正常时的位置。 */
            }
            else
            {
                if (last_normal_row >= 0)
                {
                    candidate.point_a.row = last_normal_row;    /* A 点取异常开始前的最后正常点。 */
                    candidate.point_a.col = last_normal_edge;
                }
                else
                {
                    candidate.point_a.row = y;                  /* 若没有找到正常段，就以当前行作为 A 点。 */
                    candidate.point_a.col = abnormal_valid ? abnormal_edge : abnormal_ref;
                }

                candidate.point_a.valid = 1U;
                phase = 1U;  /* 开始寻找 B 点。 */

                if (!abnormal_valid)
                {
                    candidate.missing_rows = 1U;  /* A 点以上第一行就缺失时，先记一行缺失。 */
                }
            }

            continue;
        }

        if (phase == 1U)
        {
            if (!abnormal_valid)
            {
                candidate.missing_rows++;  /* 统计异常边缺失的行数。 */
                recover_streak = 0U;       /* 一旦又缺失，恢复段统计清零。 */
                continue;
            }

            if (abs_diff_u16((uint16_t)abnormal_edge, (uint16_t)candidate.point_a.col) >= ROUNDABOUT_B_POINT_MIN_OFFSET ||
                abs_diff_u16((uint16_t)abnormal_edge, (uint16_t)abnormal_ref) >= ROUNDABOUT_B_POINT_MIN_OFFSET)
            {
                if (recover_streak == 0U)
                {
                    recover_start_row = y;      /* 记录恢复段起始行。 */
                    recover_start_edge = abnormal_edge;  /* 记录恢复段起始边线。 */
                }

                recover_streak++;

                if (recover_streak >= ROUNDABOUT_RECOVER_MIN_ROWS)
                {
                    candidate.point_b.row = recover_start_row;     /* B 点取连续恢复段的起点。 */
                    candidate.point_b.col = recover_start_edge;
                    candidate.point_b.valid = 1U;
                    candidate.point_c = candidate.point_b;         /* C 点先从 B 点起步。 */
                    phase = 2U;                                    /* 开始继续向上找 C 点。 */
                }
            }
            else
            {
                recover_streak = 0U;  /* 偏移还不够时，不算真正恢复。 */
            }

            continue;
        }

        if (abnormal_valid)
        {
            if (stable_side == ROUNDABOUT_SIDE_LEFT)
            {
                if (abnormal_edge >= candidate.point_c.col - (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE)
                {
                    if (abnormal_edge > candidate.point_c.col)
                    {
                        candidate.point_c.row = y;      /* 顺时针时，右边异常边越往右说明越接近弧顶。 */
                        candidate.point_c.col = abnormal_edge;
                        candidate.point_c.valid = 1U;
                    }
                }
                else if (candidate.point_c.valid &&
                         candidate.point_c.col - abnormal_edge > (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE)
                {
                    break;  /* 一旦明显回摆，说明已经越过 C 点。 */
                }
            }
            else
            {
                if (abnormal_edge <= candidate.point_c.col + (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE)
                {
                    if (abnormal_edge < candidate.point_c.col)
                    {
                        candidate.point_c.row = y;      /* 逆时针时，左边异常边越往左说明越接近弧顶。 */
                        candidate.point_c.col = abnormal_edge;
                        candidate.point_c.valid = 1U;
                    }
                }
                else if (candidate.point_c.valid &&
                         abnormal_edge - candidate.point_c.col > (int16_t)ROUNDABOUT_EDGE_JITTER_TOLERANCE)
                {
                    break;  /* 一旦明显回摆，说明已经越过 C 点。 */
                }
            }
        }
    }

    if (stable_rows < ROUNDABOUT_MIN_STABLE_ROWS) return candidate;    /* 稳定边成立行数不够。 */
    if (!candidate.point_a.valid) return candidate;                     /* A 点未找到。 */
    if (!candidate.point_b.valid) return candidate;                     /* B 点未找到。 */
    if (!candidate.point_c.valid) return candidate;                     /* C 点未找到。 */
    if (candidate.point_a.row <= candidate.point_b.row) return candidate; /* A 必须位于 B 下方。 */
    if (candidate.point_b.row <= candidate.point_c.row) return candidate; /* B 必须位于 C 下方。 */
    if (candidate.missing_rows < ROUNDABOUT_MIN_MISSING_ROWS && candidate.point_a.row - candidate.point_b.row < (int16_t)ROUNDABOUT_MIN_MISSING_ROWS) return candidate; /* A-B 纵向距离过小。 */
    if (abs_diff_u16((uint16_t)candidate.point_b.col, (uint16_t)candidate.point_a.col) < ROUNDABOUT_B_POINT_MIN_OFFSET) return candidate; /* A 到 B 偏移不够。 */
    if (abs_diff_u16((uint16_t)candidate.point_c.col, (uint16_t)candidate.point_b.col) < ROUNDABOUT_C_POINT_MIN_OFFSET) return candidate; /* B 到 C 偏移不够。 */

    candidate.arc_peak = candidate.point_c.col;  /* C 点列号同时作为弧顶位置输出。 */

    if (stable_side == ROUNDABOUT_SIDE_LEFT)
    {
        candidate.supplement_width = (uint16_t)(candidate.point_c.col - left_ref);  /* 顺时针时用 C 点到稳定左边界的距离作为补线宽度。 */
    }
    else
    {
        candidate.supplement_width = (uint16_t)(right_ref - candidate.point_c.col); /* 逆时针时用稳定右边界到 C 点的距离作为补线宽度。 */
    }

    if (candidate.supplement_width < ROUNDABOUT_SUPPLEMENT_MIN_WIDTH ||
        candidate.supplement_width > ROUNDABOUT_SUPPLEMENT_MAX_WIDTH) return candidate;  /* 补线宽度不合理。 */

    candidate.detected = 1U;  /* 当前方向下的单帧候选成立。 */
    return candidate;
}

/**
 * @brief 汇总当前帧最佳环岛候选
 *
 * 分别尝试：
 * 1. 左边稳定、右边异常；
 * 2. 右边稳定、左边异常；
 * 最后选出更可信的那个方向。
 */
static Roundabout_Candidate detect_roundabout_candidate(const int16_t left_edges[IMAGE_HEIGHT],
                                                        const int16_t right_edges[IMAGE_HEIGHT],
                                                        const int16_t centers[IMAGE_HEIGHT],
                                                        const uint8_t valid_rows[IMAGE_HEIGHT])
{
    Roundabout_Candidate left_stable_candidate;   /* 左稳定边假设下的候选。 */
    Roundabout_Candidate right_stable_candidate;  /* 右稳定边假设下的候选。 */
    Roundabout_Candidate empty_candidate;         /* 默认空候选。 */
    int16_t left_ref;
    int16_t right_ref;
    int16_t guide_center;
    uint16_t width_ref;

    empty_candidate.detected = 0U;
    empty_candidate.direction = ROUNDABOUT_DIRECTION_NONE;
    empty_candidate.stable_side = ROUNDABOUT_SIDE_NONE;
    empty_candidate.lower_width = 0U;
    empty_candidate.stable_edge = 0U;
    empty_candidate.missing_rows = 0U;
    empty_candidate.arc_peak = -1;
    empty_candidate.supplement_width = 0U;
    empty_candidate.reference_left = 0;
    empty_candidate.reference_right = 0;
    empty_candidate.guide_center = (int16_t)(IMAGE_WIDTH / 2U);
    empty_candidate.point_a = make_invalid_point();
    empty_candidate.point_b = make_invalid_point();
    empty_candidate.point_c = make_invalid_point();

    if (!get_lower_reference(left_edges, right_edges, centers, valid_rows, &left_ref, &right_ref, &width_ref, &guide_center))
    {
        return empty_candidate;  /* 下部不像正常直道时，不判环岛。 */
    }

    left_stable_candidate = detect_candidate_by_stable_side(ROUNDABOUT_SIDE_LEFT,
                                                            left_edges,
                                                            right_edges,
                                                            valid_rows,
                                                            left_ref,
                                                            right_ref,
                                                            width_ref,
                                                            guide_center);

    right_stable_candidate = detect_candidate_by_stable_side(ROUNDABOUT_SIDE_RIGHT,
                                                             left_edges,
                                                             right_edges,
                                                             valid_rows,
                                                             left_ref,
                                                             right_ref,
                                                             width_ref,
                                                             guide_center);

    if (left_stable_candidate.detected && right_stable_candidate.detected)
    {
        if (left_stable_candidate.point_a.row >= right_stable_candidate.point_a.row) return left_stable_candidate;  /* 两边都成立时，优先选入口起点更靠下的一边。 */
        return right_stable_candidate;
    }

    if (left_stable_candidate.detected) return left_stable_candidate;   /* 只有左稳定边候选成立。 */
    if (right_stable_candidate.detected) return right_stable_candidate; /* 只有右稳定边候选成立。 */

    empty_candidate.lower_width = width_ref;       /* 即便未识别成功，也保留调试量。 */
    empty_candidate.reference_left = left_ref;
    empty_candidate.reference_right = right_ref;
    empty_candidate.guide_center = guide_center;
    return empty_candidate;
}

/**
 * @brief 用候选结果刷新状态机上下文
 * @param candidate 当前帧环岛候选
 */
static void copy_candidate_to_context(const Roundabout_Candidate *candidate)
{
    g_roundabout_ctx.direction = candidate->direction;              /* 刷新方向。 */
    g_roundabout_ctx.stable_side = candidate->stable_side;          /* 刷新稳定边侧。 */
    g_roundabout_ctx.lower_width = candidate->lower_width;          /* 刷新下部参考宽度。 */
    g_roundabout_ctx.supplement_width = candidate->supplement_width;/* 刷新补线宽度。 */
    g_roundabout_ctx.reference_left = candidate->reference_left;    /* 刷新参考左边界。 */
    g_roundabout_ctx.reference_right = candidate->reference_right;  /* 刷新参考右边界。 */
    g_roundabout_ctx.guide_center = candidate->guide_center;        /* 刷新引导中心。 */
    g_roundabout_ctx.point_a = candidate->point_a;                  /* 刷新 A 点。 */
    g_roundabout_ctx.point_b = candidate->point_b;                  /* 刷新 B 点。 */
    g_roundabout_ctx.point_c = candidate->point_c;                  /* 刷新 C 点。 */
}

/**
 * @brief 将当前候选结果写入调试结构体
 * @param candidate 当前帧候选
 */
static void copy_candidate_to_debug(const Roundabout_Candidate *candidate)
{
    g_roundabout_debug.candidate_detected = candidate->detected;        /* 记录当前帧是否命中候选。 */
    g_roundabout_debug.direction = candidate->direction;                /* 记录当前帧方向。 */
    g_roundabout_debug.stable_side = candidate->stable_side;            /* 记录当前帧稳定边侧。 */
    g_roundabout_debug.lower_width = candidate->lower_width;            /* 记录下部宽度。 */
    g_roundabout_debug.stable_edge = candidate->stable_edge;            /* 记录稳定边位置。 */
    g_roundabout_debug.missing_rows = candidate->missing_rows;          /* 记录缺失行数。 */
    g_roundabout_debug.arc_peak = candidate->arc_peak;                  /* 记录 C 点弧顶。 */
    g_roundabout_debug.supplement_width = candidate->supplement_width;  /* 记录补线宽度。 */
    g_roundabout_debug.guide_center = candidate->guide_center;          /* 记录引导中心。 */
    g_roundabout_debug.point_a = candidate->point_a;                    /* 记录 A 点。 */
    g_roundabout_debug.point_b = candidate->point_b;                    /* 记录 B 点。 */
    g_roundabout_debug.point_c = candidate->point_c;                    /* 记录 C 点。 */
}

/**
 * @brief 按行对两个关键点做线性插值
 *
 * 该函数用于把 A-B 或 B-C 两个关键点之间拉成一条补线。
 *
 * @param start_point 起点
 * @param end_point 终点
 * @param row 目标行号
 * @return int16_t 该行插值得到的列号
 */
static int16_t interpolate_point_by_row(Roundabout_Key_Point start_point,
                                        Roundabout_Key_Point end_point,
                                        uint16_t row)
{
    int32_t numerator;
    int32_t denominator;

    if (!start_point.valid || !end_point.valid) return -1;
    if (start_point.row == end_point.row) return start_point.col;

    numerator = ((int32_t)end_point.col - (int32_t)start_point.col) * ((int32_t)row - (int32_t)start_point.row);
    denominator = (int32_t)end_point.row - (int32_t)start_point.row;

    return (int16_t)((int32_t)start_point.col + numerator / denominator);
}

/**
 * @brief 构造环岛模式下的覆盖中线
 *
 * 控制逻辑分三段：
 * 1. A-B：入口第一段直线补线；
 * 2. B-C：继续向环岛弧线过渡；
 * 3. C 以上：使用稳定边拾边，并按补线宽度外推另一侧。
 *
 * 在 EXITING 状态下，会把环岛中线与普通中线做平滑混合，
 * 让车辆更顺地切回普通巡线。
 */
static void build_roundabout_override_mid_line(const int16_t left_edges[IMAGE_HEIGHT],
                                               const int16_t right_edges[IMAGE_HEIGHT],
                                               const int16_t centers[IMAGE_HEIGHT],
                                               const uint8_t valid_rows[IMAGE_HEIGHT],
                                               const int16_t mid_line[IMAGE_HEIGHT],
                                               int16_t override_mid_line[IMAGE_HEIGHT])
{
    uint16_t y;

    for (y = 0U; y < IMAGE_HEIGHT; y++)
    {
        int16_t base_center = g_roundabout_ctx.guide_center;  /* 默认普通中心。 */
        int16_t ring_center = g_roundabout_ctx.guide_center;  /* 当前环岛中线中心。 */

        if (mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH) base_center = mid_line[y];  /* 优先使用普通中线作为基准。 */
        else if (centers[y] >= 0 && centers[y] < IMAGE_WIDTH) base_center = centers[y]; /* 普通中线缺失时退回真实中心。 */

        if (!g_roundabout_ctx.point_a.valid || y > (uint16_t)g_roundabout_ctx.point_a.row)
        {
            override_mid_line[y] = base_center;  /* A 点以下仍按普通巡线走。 */
            continue;
        }

        if (g_roundabout_ctx.stable_side == ROUNDABOUT_SIDE_LEFT)
        {
            int16_t stable_left = (left_edges[y] >= 0) ? left_edges[y] : g_roundabout_ctx.reference_left;  /* 当前行真实左边界，缺失时退回参考左边界。 */
            int16_t synthetic_right = stable_left + (int16_t)g_roundabout_ctx.supplement_width;            /* 默认按稳定边 + 补线宽度外推右边界。 */

            if (g_roundabout_ctx.point_b.valid &&
                y >= (uint16_t)g_roundabout_ctx.point_b.row &&
                y <= (uint16_t)g_roundabout_ctx.point_a.row)
            {
                synthetic_right = interpolate_point_by_row(g_roundabout_ctx.point_a, g_roundabout_ctx.point_b, y);  /* 入口第一段补 A-B。 */
            }
            else if (g_roundabout_ctx.point_c.valid &&
                     y >= (uint16_t)g_roundabout_ctx.point_c.row &&
                     y < (uint16_t)g_roundabout_ctx.point_b.row)
            {
                synthetic_right = interpolate_point_by_row(g_roundabout_ctx.point_b, g_roundabout_ctx.point_c, y);  /* 入口第二段补 B-C。 */
            }
            else if (right_edges[y] >= 0 && valid_rows[y] && y < (uint16_t)g_roundabout_ctx.point_c.row)
            {
                synthetic_right = right_edges[y];  /* C 点以上如果异常边重新可靠，就优先用真实右边界。 */
            }

            synthetic_right = clamp_column(synthetic_right);                         /* 裁剪补线边界避免越界。 */
            if (synthetic_right <= stable_left) synthetic_right = clamp_column(stable_left + (int16_t)g_roundabout_ctx.supplement_width);  /* 防止补线反向。 */
            ring_center = (int16_t)((stable_left + synthetic_right) / 2);           /* 用稳定左边和补线右边求中点。 */
        }
        else if (g_roundabout_ctx.stable_side == ROUNDABOUT_SIDE_RIGHT)
        {
            int16_t stable_right = (right_edges[y] >= 0) ? right_edges[y] : g_roundabout_ctx.reference_right; /* 当前行真实右边界，缺失时退回参考右边界。 */
            int16_t synthetic_left = stable_right - (int16_t)g_roundabout_ctx.supplement_width;               /* 默认按稳定边 - 补线宽度外推左边界。 */

            if (g_roundabout_ctx.point_b.valid &&
                y >= (uint16_t)g_roundabout_ctx.point_b.row &&
                y <= (uint16_t)g_roundabout_ctx.point_a.row)
            {
                synthetic_left = interpolate_point_by_row(g_roundabout_ctx.point_a, g_roundabout_ctx.point_b, y);  /* 入口第一段补 A-B。 */
            }
            else if (g_roundabout_ctx.point_c.valid &&
                     y >= (uint16_t)g_roundabout_ctx.point_c.row &&
                     y < (uint16_t)g_roundabout_ctx.point_b.row)
            {
                synthetic_left = interpolate_point_by_row(g_roundabout_ctx.point_b, g_roundabout_ctx.point_c, y);  /* 入口第二段补 B-C。 */
            }
            else if (left_edges[y] >= 0 && valid_rows[y] && y < (uint16_t)g_roundabout_ctx.point_c.row)
            {
                synthetic_left = left_edges[y];  /* C 点以上如果异常边重新可靠，就优先用真实左边界。 */
            }

            synthetic_left = clamp_column(synthetic_left);                            /* 裁剪补线边界避免越界。 */
            if (synthetic_left >= stable_right) synthetic_left = clamp_column(stable_right - (int16_t)g_roundabout_ctx.supplement_width);  /* 防止补线反向。 */
            ring_center = (int16_t)((synthetic_left + stable_right) / 2);            /* 用补线左边和稳定右边求中点。 */
        }

        if (g_roundabout_ctx.state == ROUNDABOUT_STATE_EXITING && mid_line[y] >= 0 && mid_line[y] < IMAGE_WIDTH)
        {
            override_mid_line[y] = (int16_t)((ring_center + mid_line[y]) / 2);  /* 出环时把环岛中线和普通中线做平滑混合。 */
        }
        else
        {
            override_mid_line[y] = ring_center;  /* 其余状态直接用环岛专用中线。 */
        }
    }
}

/**
 * @brief 初始化环岛模块
 */
void roundabout_element_init(void)
{
    g_roundabout_ctx.state = ROUNDABOUT_STATE_NONE;               /* 恢复为空闲状态。 */
    g_roundabout_ctx.direction = ROUNDABOUT_DIRECTION_NONE;       /* 清空方向。 */
    g_roundabout_ctx.stable_side = ROUNDABOUT_SIDE_NONE;          /* 清空稳定边侧。 */
    g_roundabout_ctx.detect_count = 0U;                           /* 清空连续命中计数。 */
    g_roundabout_ctx.lost_count = 0U;                             /* 清空连续丢失计数。 */
    g_roundabout_ctx.normal_count = 0U;                           /* 清空正常直道恢复计数。 */
    g_roundabout_ctx.exit_hold_count = 0U;                        /* 清空出环保持计数。 */
    g_roundabout_ctx.lower_width = 0U;                            /* 清空下部参考宽度。 */
    g_roundabout_ctx.supplement_width = 0U;                       /* 清空补线宽度。 */
    g_roundabout_ctx.reference_left = 0;                          /* 清空参考左边界。 */
    g_roundabout_ctx.reference_right = 0;                         /* 清空参考右边界。 */
    g_roundabout_ctx.guide_center = (int16_t)(IMAGE_WIDTH / 2U);  /* 引导中心恢复图像中心。 */
    g_roundabout_ctx.point_a = make_invalid_point();              /* 清空 A 点。 */
    g_roundabout_ctx.point_b = make_invalid_point();              /* 清空 B 点。 */
    g_roundabout_ctx.point_c = make_invalid_point();              /* 清空 C 点。 */

    g_roundabout_debug.state = ROUNDABOUT_STATE_NONE;             /* 调试状态清零。 */
    g_roundabout_debug.direction = ROUNDABOUT_DIRECTION_NONE;     /* 调试方向清零。 */
    g_roundabout_debug.stable_side = ROUNDABOUT_SIDE_NONE;        /* 调试稳定边侧清零。 */
    g_roundabout_debug.lower_width = 0U;                          /* 调试下部宽度清零。 */
    g_roundabout_debug.stable_edge = 0U;                          /* 调试稳定边位置清零。 */
    g_roundabout_debug.missing_rows = 0U;                         /* 调试缺失行数清零。 */
    g_roundabout_debug.arc_peak = -1;                             /* 调试弧顶清零。 */
    g_roundabout_debug.supplement_width = 0U;                     /* 调试补线宽度清零。 */
    g_roundabout_debug.guide_center = (int16_t)(IMAGE_WIDTH / 2U);/* 调试引导中心恢复默认值。 */
    g_roundabout_debug.candidate_detected = 0U;                   /* 调试候选命中清零。 */
    g_roundabout_debug.point_a = make_invalid_point();            /* 调试 A 点清零。 */
    g_roundabout_debug.point_b = make_invalid_point();            /* 调试 B 点清零。 */
    g_roundabout_debug.point_c = make_invalid_point();            /* 调试 C 点清零。 */
}

/**
 * @brief 环岛识别与补线主流程
 */
uint8_t roundabout_element_process(const uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                   const int16_t mid_line[IMAGE_HEIGHT],
                                   int16_t override_mid_line[IMAGE_HEIGHT],
                                   float *override_speed)
{
    int16_t left_edges[IMAGE_HEIGHT];   /* 当前帧真实左边界。 */
    int16_t right_edges[IMAGE_HEIGHT];  /* 当前帧真实右边界。 */
    int16_t centers[IMAGE_HEIGHT];      /* 当前帧真实中心。 */
    uint8_t valid_rows[IMAGE_HEIGHT];   /* 当前帧有效行标记。 */
    Roundabout_Candidate candidate;     /* 当前帧最佳单帧候选。 */

    build_connected_track_edges(image, mid_line, left_edges, right_edges, centers, valid_rows);  /* 先提取真实连通边线。 */
    candidate = detect_roundabout_candidate(left_edges, right_edges, centers, valid_rows);        /* 再做单帧环岛候选判断。 */
    copy_candidate_to_debug(&candidate);                                                           /* 先把当前帧候选写入调试区。 */

    switch (g_roundabout_ctx.state)
    {
        case ROUNDABOUT_STATE_NONE:
            if (candidate.detected)
            {
                if (g_roundabout_ctx.direction != candidate.direction) g_roundabout_ctx.detect_count = 0U;  /* 方向变化时重新累计确认帧。 */

                copy_candidate_to_context(&candidate);  /* 刷新上下文中的关键点和几何信息。 */
                g_roundabout_ctx.detect_count++;        /* 连续命中计数加一。 */
                g_roundabout_ctx.lost_count = 0U;       /* 清空丢失计数。 */
                g_roundabout_ctx.normal_count = 0U;     /* 清空恢复正常计数。 */
                g_roundabout_ctx.exit_hold_count = 0U;  /* 清空出环保持计数。 */

                if (g_roundabout_ctx.detect_count >= ROUNDABOUT_DETECT_CONFIRM_FRAMES)
                {
                    g_roundabout_ctx.state = ROUNDABOUT_STATE_ENTERING;  /* 连续命中足够帧后进入进环阶段。 */
                    g_roundabout_ctx.detect_count = 0U;                  /* 进入状态后复用 detect_count 统计进环帧。 */
                }
            }
            else
            {
                g_roundabout_ctx.detect_count = 0U;  /* 没命中时持续保持空闲。 */
            }
            break;

        case ROUNDABOUT_STATE_ENTERING:
            if (candidate.detected && candidate.direction == g_roundabout_ctx.direction)
            {
                copy_candidate_to_context(&candidate);  /* 进环阶段持续刷新关键点。 */
                g_roundabout_ctx.lost_count = 0U;       /* 清空丢失计数。 */
                g_roundabout_ctx.detect_count++;        /* 统计已经执行了多少帧入口补线。 */
            }
            else
            {
                g_roundabout_ctx.lost_count++;  /* 入口阶段候选丢失时开始累计。 */
            }

            if (g_roundabout_ctx.detect_count >= ROUNDABOUT_DETECT_CONFIRM_FRAMES || g_roundabout_ctx.lost_count >= ROUNDABOUT_LOST_CONFIRM_FRAMES)
            {
                g_roundabout_ctx.state = ROUNDABOUT_STATE_INSIDE;  /* 入口补线执行一小段后切到环内拾边。 */
                g_roundabout_ctx.normal_count = 0U;                /* 清空恢复正常计数。 */
                g_roundabout_ctx.lost_count = 0U;                  /* 清空丢失计数。 */
            }
            break;

        case ROUNDABOUT_STATE_INSIDE:
            if (candidate.detected && candidate.direction == g_roundabout_ctx.direction)
            {
                copy_candidate_to_context(&candidate);  /* 环内若仍能提到候选，就继续刷新几何量。 */
                g_roundabout_ctx.lost_count = 0U;       /* 清空丢失计数。 */
                g_roundabout_ctx.normal_count = 0U;     /* 仍处于环岛时，不认为已经出环。 */
            }
            else
            {
                g_roundabout_ctx.lost_count++;  /* 记录候选丢失情况，但不直接退出。 */

                if (is_track_back_to_normal(left_edges, right_edges, centers, valid_rows))
                {
                    g_roundabout_ctx.normal_count++;  /* 一旦图像恢复成正常直道，就累计出环确认帧。 */

                    if (g_roundabout_ctx.normal_count >= ROUNDABOUT_EXIT_CONFIRM_FRAMES &&
                        g_roundabout_ctx.lost_count >= ROUNDABOUT_LOST_CONFIRM_FRAMES)
                    {
                        g_roundabout_ctx.state = ROUNDABOUT_STATE_EXITING;  /* 连续恢复正常后进入出环补线。 */
                        g_roundabout_ctx.exit_hold_count = 0U;              /* 清空出环保持计数。 */
                    }
                }
                else
                {
                    g_roundabout_ctx.normal_count = 0U;  /* 还未恢复正常直道时，持续保持环内状态。 */
                }
            }
            break;

        case ROUNDABOUT_STATE_EXITING:
            if (candidate.detected && candidate.direction == g_roundabout_ctx.direction)
            {
                copy_candidate_to_context(&candidate);             /* 如果在出环时又重新出现明显环岛特征，就回到环内状态。 */
                g_roundabout_ctx.state = ROUNDABOUT_STATE_INSIDE;
                g_roundabout_ctx.normal_count = 0U;
                g_roundabout_ctx.exit_hold_count = 0U;
            }
            else
            {
                g_roundabout_ctx.exit_hold_count++;  /* 出环阶段继续保持几帧，平滑切回普通巡线。 */

                if (g_roundabout_ctx.exit_hold_count >= ROUNDABOUT_EXIT_HOLD_FRAMES)
                {
                    roundabout_element_init();  /* 出环完成后整体复位。 */
                }
            }
            break;

        default:
            roundabout_element_init();  /* 理论兜底：状态异常时整体复位。 */
            break;
    }

    g_roundabout_debug.state = g_roundabout_ctx.state;                  /* 输出当前状态机状态。 */
    g_roundabout_debug.direction = g_roundabout_ctx.direction;          /* 输出当前确认方向。 */
    g_roundabout_debug.stable_side = g_roundabout_ctx.stable_side;      /* 输出当前稳定边侧。 */
    g_roundabout_debug.guide_center = g_roundabout_ctx.guide_center;    /* 输出当前引导中心。 */
    if (g_roundabout_ctx.supplement_width > 0U) g_roundabout_debug.supplement_width = g_roundabout_ctx.supplement_width;  /* 优先显示已确认的补线宽度。 */
    if (g_roundabout_ctx.point_a.valid) g_roundabout_debug.point_a = g_roundabout_ctx.point_a;  /* 输出已确认的 A 点。 */
    if (g_roundabout_ctx.point_b.valid) g_roundabout_debug.point_b = g_roundabout_ctx.point_b;  /* 输出已确认的 B 点。 */
    if (g_roundabout_ctx.point_c.valid) g_roundabout_debug.point_c = g_roundabout_ctx.point_c;  /* 输出已确认的 C 点。 */

    if (g_roundabout_ctx.state == ROUNDABOUT_STATE_NONE)
    {
        return 0U;  /* 未处于环岛状态时，不覆盖普通巡线结果。 */
    }

    build_roundabout_override_mid_line(left_edges, right_edges, centers, valid_rows, mid_line, override_mid_line);  /* 生成环岛覆盖中线。 */
    *override_speed = ROUNDABOUT_SPECIAL_TARGET_SPEED;  /* 输出环岛专用速度。 */
    return 1U;  /* 通知主循环启用环岛专用控制。 */
}

/**
 * @brief 获取当前环岛状态
 * @return Roundabout_State 当前状态机状态
 */
Roundabout_State roundabout_element_get_state(void)
{
    return g_roundabout_ctx.state;
}

/**
 * @brief 获取当前环岛方向
 * @return Roundabout_Direction 当前确认方向
 */
Roundabout_Direction roundabout_element_get_direction(void)
{
    return g_roundabout_ctx.direction;
}

/**
 * @brief 获取当前环岛调试信息
 * @return Roundabout_Debug_Info 当前调试信息快照
 */
Roundabout_Debug_Info roundabout_element_get_debug_info(void)
{
    return g_roundabout_debug;
}
