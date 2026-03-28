#ifndef __TUNING_PARAMS_H__
#define __TUNING_PARAMS_H__

/**
 * @file tuning_params.h
 * @brief 集中管理需要频繁上车调试的参数
 * @details
 * 整理原则如下：
 * 1. 引脚、硬件映射仍放在各自模块中；
 * 2. 经常需要按赛道、车速、舵机状态调整的参数统一放这里；
 * 3. 尽量按“车体/控制/图像/保护/调试”分类，方便快速定位。
 */

/* 车体与基础控制参数 */
#define CAR_WIDTH                          120     // 车体宽度参数。
#define MAX_STEERING_ANGLE                 10      // 相对中位允许的最大转向角。
#define MIN_SPEED                          30      // 控制允许的最小速度目标。
#define MAX_SPEED                          100     // 控制允许的最大速度目标。

/* 舵机参数 */
#define TUNE_SERVO_CENTER_ANGLE            90.0f   // 舵机回中时的绝对角度。

/* 控制算法通用参数 */
#define TUNE_PID_INTEGRAL_LIMIT            200.0f  // PID 积分项限幅。
#define TUNE_SPEED_OUTPUT_LIMIT            100.0f  // 速度环输出上限。
#define TUNE_CURVE_SPEED_GAIN              0.35f   // 转向误差引起的减速增益。
#define TUNE_DT_MIN_VALUE                  0.001f  // 控制周期最小保护值。
#define TUNE_ENCODER_SPEED_SCALE           1.0f    // 编码器脉冲到速度反馈的缩放系数。
#define TUNE_STEER_FILTER_ALPHA            0.35f   // 转向误差低通滤波系数。

/* 弯道增强控制参数 */
#define TUNE_TRACK_LOST_TOLERANCE_FRAMES   2U      // 连续丢线多少帧后触发急停。
#define TUNE_STEER_SLOPE_GAIN              10.0f   // 中线斜率对转向误差的增益。
#define TUNE_STEER_CURVATURE_GAIN          2.2f    // 中线曲率对转向误差的增益。
#define TUNE_CURVATURE_SPEED_GAIN          5.5f    // 曲率对减速量的增益。
#define TUNE_SLOPE_SPEED_GAIN              8.0f    // 斜率对减速量的增益。
#define TUNE_LOW_VALID_SPEED_PENALTY       8.0f    // 有效点偏少时的额外减速惩罚。
#define TUNE_STRAIGHT_SLOPE_DEADBAND       0.10f   // 小斜率死区。
#define TUNE_STRAIGHT_CURVATURE_DEADBAND   0.80f   // 小曲率死区。

/* 图像处理中线提取参数 */
#define TUNE_MIDLINE_MIN_WHITE_RUN         2U      // 边界搜索时最短连续白段长度。
#define TUNE_MIDLINE_SEARCH_STEP           1U      // 中线边界搜索步长。
#define TUNE_MIDLINE_SMOOTH_WINDOW         2U      // 中线平滑窗口半宽。

/* 十字交叉路口元素参数 */
#define TUNE_CROSS_SAMPLE_ROW_LOWER_RATIO_NUM      3U     // 十字检测下采样行的分子比例。
#define TUNE_CROSS_SAMPLE_ROW_LOWER_RATIO_DEN      4U     // 十字检测下采样行的分母比例。
#define TUNE_CROSS_SAMPLE_ROW_MIDDLE_RATIO_NUM     1U     // 十字检测中采样行的分子比例。
#define TUNE_CROSS_SAMPLE_ROW_MIDDLE_RATIO_DEN     2U     // 十字检测中采样行的分母比例。
#define TUNE_CROSS_SAMPLE_ROW_UPPER_RATIO_NUM      1U     // 十字检测上采样行的分子比例。
#define TUNE_CROSS_SAMPLE_ROW_UPPER_RATIO_DEN      3U     // 十字检测上采样行的分母比例。
#define TUNE_CROSS_NORMAL_MIN_WIDTH                18U    // 普通直道最小宽度阈值。
#define TUNE_CROSS_NORMAL_MAX_WIDTH                72U    // 普通直道最大宽度阈值。
#define TUNE_CROSS_WIDE_MIN_WIDTH                  92U    // 十字中部最小扩展宽度。
#define TUNE_CROSS_WIDTH_EXPAND_THRESHOLD          28U    // 中部相对下部至少增加的宽度。
#define TUNE_CROSS_CENTER_DIFF_LIMIT               18     // 上下采样行中心允许最大偏差。
#define TUNE_CROSS_DETECT_CONFIRM_FRAMES           2U     // 连续多少帧命中后确认进入十字。
#define TUNE_CROSS_LOST_CONFIRM_FRAMES             2U     // 连续多少帧未命中后确认离开十字主体。
#define TUNE_CROSS_EXIT_HOLD_FRAMES                6U     // 离开十字后继续保持直行控制的帧数。
#define TUNE_CROSS_SPECIAL_TARGET_SPEED            35.0f  // 十字专用保守目标速度。

/* 环岛元素参数 */
#define TUNE_ROUNDABOUT_LOWER_START_RATIO_NUM      2U     // 环岛下部参考区域起始行分子比例。
#define TUNE_ROUNDABOUT_LOWER_START_RATIO_DEN      3U     // 环岛下部参考区域起始行分母比例。
#define TUNE_ROUNDABOUT_REF_MIN_ROWS               8U     // 建立下部直道参考所需最少有效行数。
#define TUNE_ROUNDABOUT_NORMAL_MIN_WIDTH           18U    // 环岛入口下部普通直道最小宽度。
#define TUNE_ROUNDABOUT_NORMAL_MAX_WIDTH           72U    // 环岛入口下部普通直道最大宽度。
#define TUNE_ROUNDABOUT_CENTER_DIFF_LIMIT          16U    // 下部中心允许最大波动量。
#define TUNE_ROUNDABOUT_STABLE_EDGE_TOLERANCE      6U     // 稳定侧边线允许波动量。
#define TUNE_ROUNDABOUT_MIN_MISSING_ROWS           5U     // 至少连续多少行出现单侧丢线。
#define TUNE_ROUNDABOUT_MIN_RISING_ROWS            4U     // 弧线发展阶段至少持续多少行。
#define TUNE_ROUNDABOUT_MIN_FALLING_ROWS           3U     // 弧线回摆阶段至少持续多少行。
#define TUNE_ROUNDABOUT_ARC_PEAK_MIN_OFFSET        12U    // 弧顶相对原边线最小偏移量。
#define TUNE_ROUNDABOUT_SUPPLEMENT_MIN_WIDTH       18U    // 估算补线距离的最小阈值。
#define TUNE_ROUNDABOUT_SUPPLEMENT_MAX_WIDTH       80U    // 估算补线距离的最大阈值。
#define TUNE_ROUNDABOUT_DETECT_CONFIRM_FRAMES      2U     // 连续多少帧命中后确认进入环岛。
#define TUNE_ROUNDABOUT_LOST_CONFIRM_FRAMES        3U     // 连续多少帧未命中后确认退出环岛主体。
#define TUNE_ROUNDABOUT_EXIT_HOLD_FRAMES           6U     // 退出环岛后继续保持状态的帧数。
#define TUNE_ROUNDABOUT_EDGE_JITTER_TOLERANCE      2U     // 判断弧线单调趋势时允许的小抖动。
#define TUNE_ROUNDABOUT_SPECIAL_TARGET_SPEED       32.0f  // 环岛专用保守目标速度。

/* 防跑飞与保护参数 */
#define TUNE_TRACK_VALID_MIN_POINTS        16      // 赛道有效判定最小点数。
#define TUNE_FAILSAFE_RELEASE_FRAMES       5       // 退出保护前需要的连续有效帧数。

/* 调试输出参数 */
#define TUNE_DEBUG_PRINT_INTERVAL_FRAMES   10U     // 每隔多少帧打印一次调试信息。

/* 掉帧超时参数 */
#define TUNE_FRAME_TIMEOUT_MIN_MS          120U    // 帧超时下限。
#define TUNE_FRAME_TIMEOUT_MAX_MS          500U    // 帧超时上限。
#define TUNE_FRAME_TIMEOUT_FRAME_FACTOR    3.0f    // 用控制周期换算超时阈值的倍数。

/* 默认目标速度 */
#define TUNE_DEFAULT_TARGET_SPEED          45.0f   // 默认基础目标速度。

/* 默认 PID 参数 */
#define TUNE_STEERING_KP                   0.70f   // 转向 PID 比例系数。
#define TUNE_STEERING_KI                   0.00f   // 转向 PID 积分系数。
#define TUNE_STEERING_KD                   0.18f   // 转向 PID 微分系数。
#define TUNE_SPEED_KP                      1.20f   // 速度 PID 比例系数。
#define TUNE_SPEED_KI                      0.08f   // 速度 PID 积分系数。
#define TUNE_SPEED_KD                      0.00f   // 速度 PID 微分系数。

#endif
