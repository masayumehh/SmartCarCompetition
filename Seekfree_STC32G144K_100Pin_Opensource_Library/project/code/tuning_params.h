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