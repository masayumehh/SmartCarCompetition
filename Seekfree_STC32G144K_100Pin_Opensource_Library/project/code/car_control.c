/**
 * @file car_control.c
 * @brief 小车底盘控制模块
 * @details
 * 本模块接收图像处理中提取出的赛道中线，然后完成两件事：
 * 1. 根据中线相对图像中心的偏移，计算舵机转向；
 * 2. 根据中线的偏移、斜率、曲率，动态调整电机速度。
 * 当前中线数组的定义是：
 * mid_line[y] = 第 y 行赛道中心所在的列号
 *
 * 因此这里的控制分析是沿行方向进行的：
 * - 先看很多行的中心点整体偏到左边还是右边；
 * - 再看这些中心点沿图像上下方向的变化趋势；
 * - 最后把这些几何特征转换成转向和速度控制量。
 */
#include "car_control.h"

#include "zf_driver_encoder.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

/** 舵机 PWM 输出引脚 */
#define SERVO_PWM_PIN            (PWME_CH1P_PA0)
/** 舵机 PWM 频率，常规模拟舵机一般为 50Hz */
#define SERVO_PWM_FREQ           (50)
/** 舵机机械中位角 */
#define SERVO_CENTER_ANGLE       (TUNE_SERVO_CENTER_ANGLE)
/** 舵机允许的绝对最小角度，等于中位角减最大相对转角 */
#define SERVO_ANGLE_MIN          (SERVO_CENTER_ANGLE - (float)MAX_STEERING_ANGLE)
/** 舵机允许的绝对最大角度，等于中位角加最大相对转角 */
#define SERVO_ANGLE_MAX          (SERVO_CENTER_ANGLE + (float)MAX_STEERING_ANGLE)
/** 舵机可用最小占空比百分比，按实测极限设置 */
#define SERVO_DUTY_MIN_PERCENT   (6.8f)
/** 舵机可用最大占空比百分比，按实测极限设置 */
#define SERVO_DUTY_MAX_PERCENT   (8.0f)

/** 电机 1 使用 P7.5 输出 PWM，P7.4 固定拉低 */
#define MOTOR1_PWM_PIN           (PWMB_CH2_P75)
/** 电机 1 反向端固定拉低，形成单向驱动 */
#define MOTOR1_GND_PIN           (IO_P74)
/** 电机 2 使用 P7.7 输出 PWM，P7.6 固定拉低 */
#define MOTOR2_PWM_PIN           (PWMB_CH4_P77)
/** 电机 2 反向端固定拉低，形成单向驱动 */
#define MOTOR2_GND_PIN           (IO_P76)
/** 电机 PWM 频率 */
#define MOTOR_PWM_FREQ           (17000)

/** 编码器 1 使用的正交解码模块 */
#define SPEED_ENCODER1           (PWMA_ENCODER)
/** 编码器 1 A 相输入引脚 */
#define SPEED_ENCODER1_A_PIN     (PWMA_ENCODER_CH1P_P60)
/** 编码器 1 B 相输入引脚 */
#define SPEED_ENCODER1_B_PIN     (PWMA_ENCODER_CH2P_P62)
/** 编码器 2 使用的正交解码模块 */
#define SPEED_ENCODER2           (PWMC_ENCODER)
/** 编码器 2 A 相输入引脚 */
#define SPEED_ENCODER2_A_PIN     (PWMC_ENCODER_CH1P_P40)
/** 编码器 2 B 相输入引脚 */
#define SPEED_ENCODER2_B_PIN     (PWMC_ENCODER_CH2P_P42)

/** PID 积分项限幅，防止积分饱和 */
#define PID_INTEGRAL_LIMIT       (TUNE_PID_INTEGRAL_LIMIT)
/** 速度环最大输出，占空比百分比上限 */
#define SPEED_OUTPUT_LIMIT       (TUNE_SPEED_OUTPUT_LIMIT)
/** 偏差引起的基础降速系数 */
#define CURVE_SPEED_GAIN         (TUNE_CURVE_SPEED_GAIN)
/** 控制周期最小值，避免 dt 过小导致微分项过大 */
#define DT_MIN_VALUE             (TUNE_DT_MIN_VALUE)
/** 编码器计数到速度反馈的缩放比例 */
#define ENCODER_SPEED_SCALE      (TUNE_ENCODER_SPEED_SCALE)
#define MOTOR_MIN_DUTY_PERCENT   (TUNE_MOTOR_MIN_DUTY_PERCENT)
#define CRAWL_TEST_MODE          (TUNE_CRAWL_TEST_MODE)
#define CRAWL_TEST_DUTY_PERCENT  (TUNE_CRAWL_TEST_DUTY_PERCENT)
#define STEERING_ONLY_MODE       (TUNE_STEERING_ONLY_MODE)
/** 转向误差低通滤波系数 */
#define STEER_FILTER_ALPHA       (TUNE_STEER_FILTER_ALPHA)
#define STEER_OFFSET_DEADBAND    (TUNE_STEER_OFFSET_DEADBAND)
/** 判定赛道有效所需的最少有效中线点数 */
#define TRACK_VALID_MIN_POINTS   (TUNE_TRACK_VALID_MIN_POINTS)
/** 急停后恢复控制前，需要连续满足多少帧有效赛道 */
#define FAILSAFE_RELEASE_FRAMES  (TUNE_FAILSAFE_RELEASE_FRAMES)

/** 允许短时丢线的帧数，避免瞬时误判直接急停 */
#define TRACK_LOST_TOLERANCE_FRAMES   (TUNE_TRACK_LOST_TOLERANCE_FRAMES)
/** 中线斜率对舵机前馈的增益 */
#define STEER_SLOPE_GAIN              (TUNE_STEER_SLOPE_GAIN)
/** 中线曲率对舵机前馈的增益 */
#define STEER_CURVATURE_GAIN          (TUNE_STEER_CURVATURE_GAIN)
/** 曲率越大，目标速度下降越多 */
#define CURVATURE_SPEED_GAIN          (TUNE_CURVATURE_SPEED_GAIN)
/** 斜率越大，目标速度下降越多 */
#define SLOPE_SPEED_GAIN              (TUNE_SLOPE_SPEED_GAIN)
/** 有效中线点减少时，附加的保守降速 */
#define LOW_VALID_SPEED_PENALTY       (TUNE_LOW_VALID_SPEED_PENALTY)
/** 斜率死区，小于该值视为直道噪声 */
#define STRAIGHT_SLOPE_DEADBAND       (TUNE_STRAIGHT_SLOPE_DEADBAND)
/** 曲率死区，小于该值视为直道噪声 */
#define STRAIGHT_CURVATURE_DEADBAND   (TUNE_STRAIGHT_CURVATURE_DEADBAND)
#define MIDLINE_NEAR_ROWS             (TUNE_MIDLINE_NEAR_ROWS)
#define MIDLINE_NEAR_WEIGHT           (TUNE_MIDLINE_NEAR_WEIGHT)
#define STEER_LARGE_ERROR_THRESHOLD   (TUNE_STEER_LARGE_ERROR_THRESHOLD)
#define STEER_LARGE_ERROR_GAIN        (TUNE_STEER_LARGE_ERROR_GAIN)
#define STEER_CURVE_GAIN_MIN          (TUNE_STEER_CURVE_GAIN_MIN)
#define STEER_CURVE_GAIN_MAX          (TUNE_STEER_CURVE_GAIN_MAX)
#define STEER_CURVE_GAIN_REF          (TUNE_STEER_CURVE_GAIN_REF)
#define STEER_CURVE_FOLLOW_REF        (TUNE_STEER_CURVE_FOLLOW_REF)
#define STEER_OFFSET_CURVE_MIN_WEIGHT (TUNE_STEER_OFFSET_CURVE_MIN_WEIGHT)
#define STEER_ENTER_RATE_DPS          (TUNE_STEER_ENTER_RATE_DPS)
#define STEER_RELEASE_RATE_DPS        (TUNE_STEER_RELEASE_RATE_DPS)
#define STEER_REVERSE_RATE_DPS        (TUNE_STEER_REVERSE_RATE_DPS)

/**
 * @brief 赛道几何特征
 * @details
 * 这些特征都是根据 mid_line[y] = x_center 这条中线计算出来的。
 * - offset_mean：中线平均偏到图像中心左边还是右边
 * - slope：中线整体是向左斜还是向右斜
 * - curvature：中线弯曲程度
 * - valid_points：当前用于控制分析的有效中线点数量
 */
typedef struct
{
    float offset_mean;      // 当前中线相对目标中心线的平均横向偏移。
    float slope;            // 中线整体斜率。
    float curvature;        // 中线弯曲程度及弯曲方向。
    uint16_t valid_points;  // 当前用于分析的有效中线点数量。
} Track_Features;

static Car_State car_state = {
    0.0f,
    TUNE_DEFAULT_TARGET_SPEED,
    0.0f,
    {0.0f, 0.0f, 0.0f, TUNE_STEERING_KP, TUNE_STEERING_KI, TUNE_STEERING_KD, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, TUNE_SPEED_KP, TUNE_SPEED_KI, TUNE_SPEED_KD, 0.0f, 0.0f, 0.0f}
};

/** 低通滤波后的转向误差，供舵机控制和速度降速共同使用 */
static float filtered_steer_error = 0.0f;
/** 上一拍已经下发到电机的占空比，用于短时丢线时保持动力连续 */
static float last_motor_duty = 0.0f;
/** 当前是否处于保护状态 */
static uint8_t failsafe_active = 0;
/** 从保护状态恢复时，连续有效赛道帧计数 */
static uint8_t valid_track_streak = 0;
/** 连续丢线帧计数，用于短时容错 */
static uint8_t lost_track_streak = 0;
/** 调试信息快照，主循环会周期性打印 */
static Car_Debug_Info car_debug_info = {0};

/**
 * @brief 通用限幅函数
 * @param value 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的值
 */
static float limit_output(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief 本地浮点绝对值函数
 * @param value 输入值
 * @return 绝对值
 */
static float absf_local(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float clamp01_local(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

/**
 * @brief 把一个控制周期内的正交编码器计数换算成速度反馈
 * @param pulse_count 本周期累计到的正交编码器计数
 * @param dt 控制周期，单位秒
 * @return 换算后的速度反馈值
 * @details
 * 对正交编码器来说，脉冲周期越短，单位时间内计数越多，速度也越快。
 * 因此这里先除以 dt 得到“每秒计数率”，再乘以缩放系数得到速度反馈。
 */
static float encoder_count_to_speed(int16 pulse_count, float dt)
{
    if (dt < DT_MIN_VALUE) dt = DT_MIN_VALUE;
    return absf_local((float)pulse_count) * ENCODER_SPEED_SCALE / dt;
}

/**
 * @brief 对很小的特征量做死区处理
 * @param value 输入特征值
 * @param deadband 死区阈值
 * @return 死区处理后的值
 * @details
 * 很小的斜率和曲率通常更像图像抖动，不希望它们干扰直道控制。
 */
static float apply_deadband(float value, float deadband)
{
    if (value > deadband) return value - deadband;
    if (value < -deadband) return value + deadband;
    return 0.0f;
}

/**
 * @brief 清空一个 PID 控制器的内部状态。
 * @param pid PID 控制器指针。
 */
static void clear_pid_state(PID_Controller *pid)
{
    pid->integral = 0.0f;   // 清零积分累计值。
    pid->prev_error = 0.0f; // 清零上一拍误差。
    pid->output = 0.0f;     // 清零上一次 PID 输出。
}

/**
 * @brief 初始化一个 PID 控制器。
 * @param pid PID 控制器指针。
 * @param Kp 比例系数。
 * @param Ki 积分系数。
 * @param Kd 微分系数。
 */
static void pid_init(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;           // 写入比例系数。
    pid->Ki = Ki;           // 写入积分系数。
    pid->Kd = Kd;           // 写入微分系数。
    pid->integral = 0.0f;   // 初始化积分项。
    pid->prev_error = 0.0f; // 初始化上一拍误差。
    pid->output = 0.0f;     // 初始化 PID 输出值。
}

/**
 * @brief 执行一次 PID 计算。
 * @param pid PID 控制器指针。
 * @param setpoint 目标值。
 * @param actual 当前反馈值。
 * @param dt 控制周期，单位秒。
 * @return PID 输出值。
 */
static float pid_calculate(PID_Controller *pid, float setpoint, float actual, float dt)
{
    float error = setpoint - actual; // 当前误差 = 目标值 - 反馈值。
    float derivative;                // 误差变化率，对应 D 项。

    if (dt < DT_MIN_VALUE) dt = DT_MIN_VALUE; // 防止 dt 过小导致计算不稳定。

    pid->integral += error * dt; // 累加本周期积分项。
    pid->integral = limit_output(pid->integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT); // 对积分项做限幅。

    derivative = (error - pid->prev_error) / dt; // 计算误差变化率。
    pid->prev_error = error; // 保存当前误差给下一拍使用。

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 组合 P、I、D 三部分输出。
    return pid->output; // 返回本次 PID 输出。
}

/**
 * @brief 把舵机绝对角度换算成 PWM 占空比计数值
 * @param angle_deg 舵机绝对角度，范围为中位角两侧的可用机械角度
 * @return 对应的 PWM 占空比计数值
 */
static uint32 servo_angle_to_duty(float angle_deg)
{
    float duty_percent;  // 舵机角度换算后的占空比百分比。
    float duty;  // 最终写给 PWM 的占空比计数值。

    angle_deg = limit_output(angle_deg, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);  // 先把角度限制在中位角两侧的真实可用范围内。
    duty_percent = SERVO_DUTY_MIN_PERCENT
                 + (angle_deg - SERVO_ANGLE_MIN) * (SERVO_DUTY_MAX_PERCENT - SERVO_DUTY_MIN_PERCENT)
                 / (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN);  // 把 80°~100° 线性映射到 6.8%~8.0% 的有效占空比范围。
    duty = (float)PWM_DUTY_MAX * duty_percent / 100.0f;  // 再把占空比百分比换成 PWM 计数值。

    return (uint32)limit_output(duty, 0.0f, (float)PWM_DUTY_MAX);  // 最终输出给 PWM 的整数计数值。
}

/**
 * @brief 输出舵机控制量
 * @param steer_cmd_deg 相对中位角的转向命令，单位度
 * @details
 * 正负方向是否符合你的车体安装，最终要以上车测试为准。
 */
static void control_steering(float steer_cmd_deg)
{
    float servo_angle;  // 舵机最终要去的绝对角度。

    steer_cmd_deg = limit_output(steer_cmd_deg, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);  // 限制相对打角范围。
    servo_angle = SERVO_CENTER_ANGLE + steer_cmd_deg;  // 把相对打角转换成舵机绝对角度。

    pwm_set_duty(SERVO_PWM_PIN, servo_angle_to_duty(servo_angle));  // 把舵机角度转换成 PWM 并真正输出。
    car_state.steering_angle = steer_cmd_deg;  // 记录本次下发的相对转向命令。
}

/**
 * @brief 输出电机占空比
 * @param duty_percent 占空比百分比，范围 0~100
 */
static void control_motor_duty(float duty_percent)
{
    uint32 duty;  // 最终写给电机 PWM 的计数值。

    duty_percent = limit_output(duty_percent, 0.0f, 100.0f);  // 限制占空比百分比范围。
    duty = (uint32)(duty_percent * ((float)PWM_DUTY_MAX / 100.0f));  // 百分比转换成 PWM 计数值。

    gpio_low(MOTOR1_GND_PIN);  // 保持 H 桥这一侧为低电平，方向由接线验证后的 PWM 端决定。
    gpio_low(MOTOR2_GND_PIN);  // 保持 H 桥这一侧为低电平，方向由接线验证后的 PWM 端决定。
    pwm_set_duty(MOTOR1_PWM_PIN, duty);  // 同步输出电机 1 占空比。
    pwm_set_duty(MOTOR2_PWM_PIN, duty);  // 同步输出电机 2 占空比。
}

/**
 * @brief 输出安全状态
 * @details
 * 电机停转，舵机回中，用于保护状态和紧急停车。
 */
static void apply_safe_output(void)
{
    gpio_low(MOTOR1_GND_PIN);  // 安全状态下保持地脚为低，避免桥臂悬空。
    gpio_low(MOTOR2_GND_PIN);  // 安全状态下保持地脚为低，避免桥臂悬空。
    pwm_set_duty(MOTOR1_PWM_PIN, 0);  // 安全状态下立刻停止电机 1。
    pwm_set_duty(MOTOR2_PWM_PIN, 0);  // 安全状态下立刻停止电机 2。
    encoder_clear_count(SPEED_ENCODER1);  // 安全状态下同步清编码器 1，避免恢复后读到陈旧计数。
    encoder_clear_count(SPEED_ENCODER2);  // 安全状态下同步清编码器 2，避免恢复后读到陈旧计数。
    pwm_set_duty(SERVO_PWM_PIN, servo_angle_to_duty(SERVO_CENTER_ANGLE));  // 安全状态下让舵机回中。
    car_state.current_speed = 0.0f;  // 软件状态中的当前速度也同步清零。
    car_state.steering_angle = 0.0f;  // 软件状态中的当前转向角同步回中。
    car_debug_info.speed_output = 0.0f;  // 调试信息中的速度输出同步清零。
    last_motor_duty = 0.0f;  // 安全状态下同时清空最近一次电机占空比。
}

/**
 * @brief 获取当前速度反馈
 * @return 编码器反馈的速度值
 * @param dt 控制周期，单位秒
 * @note 当前实现读取两路正交编码器的计数率，并取平均作为整车速度反馈。
 */
static float get_speed_feedback(float dt)
{
    int16 pulse_count_1 = encoder_get_count(SPEED_ENCODER1);  // 读取电机 1 对应编码器本周期累计计数。
    int16 pulse_count_2 = encoder_get_count(SPEED_ENCODER2);  // 读取电机 2 对应编码器本周期累计计数。
    float speed_feedback_1;
    float speed_feedback_2;

    encoder_clear_count(SPEED_ENCODER1);  // 读完后清零，为下一周期重新计数。
    encoder_clear_count(SPEED_ENCODER2);  // 读完后清零，为下一周期重新计数。

    speed_feedback_1 = encoder_count_to_speed(pulse_count_1, dt);  // 把编码器 1 计数换算成速度。
    speed_feedback_2 = encoder_count_to_speed(pulse_count_2, dt);  // 把编码器 2 计数换算成速度。
    return (speed_feedback_1 + speed_feedback_2) * 0.5f;  // 当前速度环取两侧平均速度。
}

/**
 * @brief 统计当前分析窗口内的有效中线点数
 * @param mid_line 中线数组，满足 mid_line[y] = x_center
 * @param line_width 参与分析的行数，0 表示整幅图高度
 * @return 有效中线点数
 * @details
 * 这里的“有效”指的是该行确实提取到了一个落在图像宽度范围内的中心列号。
 */
static uint16_t count_valid_mid_points(const int16_t mid_line[IMAGE_HEIGHT], uint16_t line_width)
{
    uint16_t start_idx;  // 分析窗口起始行。
    uint16_t end_idx;  // 分析窗口结束行后一位。
    uint16_t i;  // 当前检查的行号。
    uint16_t valid_points = 0;  // 当前统计到的有效中线点数量。

    if (line_width == 0 || line_width > IMAGE_HEIGHT) line_width = (uint16_t)((IMAGE_HEIGHT * 2U) / 5U);  // 默认只统计更靠近车前的中下部区域，减少提前看见远端弯道。

    end_idx = (uint16_t)(IMAGE_HEIGHT - (IMAGE_HEIGHT / 10U));  // 最底部少量区域容易受车体和近场反光干扰，控制时不参与。
    if (end_idx > line_width) start_idx = (uint16_t)(end_idx - line_width);
    else start_idx = 0U;

    for (i = start_idx; i < end_idx; i++)
    {
        if (mid_line[i] >= 0 && mid_line[i] < IMAGE_WIDTH)
        {
            valid_points++;  // 当前行中线落在图像范围内，计为有效点。
        }
    }

    return valid_points;  // 返回窗口内有效中线点总数。
}

/**
 * @brief 判断当前中线是否足够可靠，可以参与控制
 * @param mid_line 中线数组
 * @param line_width 参与判断的行数，0 表示整幅图高度
 * @return 1 表示有效，0 表示无效
 */
static uint8_t is_track_valid(const int16_t mid_line[IMAGE_HEIGHT], uint16_t line_width)
{
    uint16_t valid_threshold;  // 判定赛道有效所需的最少点数。

    if (line_width == 0 || line_width > IMAGE_HEIGHT) line_width = (uint16_t)((IMAGE_HEIGHT * 2U) / 5U);  // 默认只用更靠近车前的中下部视野判断赛道是否可靠。

    valid_threshold = line_width / 3U;  // 默认至少要求三分之一的行有有效中线。
    if (valid_threshold < TRACK_VALID_MIN_POINTS) valid_threshold = TRACK_VALID_MIN_POINTS;  // 阈值不能低于最小配置值。

    return (count_valid_mid_points(mid_line, line_width) >= valid_threshold) ? 1U : 0U;  // 满足阈值返回有效，否则返回无效。
}

/**
 * @brief 计算中线的平均二阶差分，作为曲率近似
 * @param mid_line 中线数组
 * @param start_idx 起始行
 * @param end_idx 结束行
 * @return 曲率均值
 * @details
 * 这里不是严格几何曲率，而是便于控制使用的离散弯曲强度估计。
 */
static float get_signed_curvature(const int16_t mid_line[IMAGE_HEIGHT], uint16_t start_idx, uint16_t end_idx)
{
    uint16_t i;  // 当前作为中点的行号。
    uint16_t valid_segments = 0;  // 成功参与曲率计算的三点段数量。
    float curvature_sum = 0.0f;  // 所有有效三点段的二阶差分累加值。

    if (end_idx <= start_idx + 2U) return 0.0f;  // 少于 3 行时无法计算曲率。

    for (i = (uint16_t)(start_idx + 1U); i + 1U < end_idx; i++)
    {
        if (mid_line[i - 1U] >= 0 && mid_line[i] >= 0 && mid_line[i + 1U] >= 0)
        {
            float second_diff = (float)mid_line[i + 1U] - 2.0f * (float)mid_line[i] + (float)mid_line[i - 1U];  // 当前三点段的二阶差分。
            curvature_sum += second_diff;  // 把当前弯曲量累加到总和里。
            valid_segments++;  // 记录当前这组三点有效。
        }
    }

    if (valid_segments == 0U) return 0.0f;  // 一组有效三点都没有时，返回 0。

    return curvature_sum / (float)valid_segments;  // 返回平均二阶差分作为曲率近似。
}

/**
 * @brief 从中线数组中提取控制需要的几何特征
 * @param mid_line 中线数组
 * @param line_width 参与分析的行数，0 表示整幅图高度
 * @return 赛道几何特征
 * @details
 * 这里的分析窗口是沿图像高度方向取的，也就是从若干行中线点里估计：
 * - 平均偏移
 * - 斜率
 * - 曲率
 * - 有效点数
 */
static Track_Features analyze_track_features(const int16_t mid_line[IMAGE_HEIGHT], uint16_t line_width)
{
    Track_Features features = {0.0f, 0.0f, 0.0f, 0U};
    uint16_t start_idx;
    uint16_t end_idx;
    uint16_t i;
    float sum_x = 0.0f;   // 所有有效点的行号和
    float sum_y = 0.0f;   // 所有有效点的列号和
    float sum_x2 = 0.0f;  // 所有有效点行号平方和
    float sum_xy = 0.0f;  // 所有有效点“行号 * 列号”的乘积和
    float denominator;    // 后面计算斜率时的分母
    float weighted_sum_y = 0.0f;  // 近底部加权后的列号总和。
    float total_weight = 0.0f;  // 偏差计算使用的总权重。
    float near_sum_y = 0.0f;  // 靠近车前区域的列号总和。
    uint16_t near_count = 0U;  // 靠近车前区域的有效点数量。
    uint16_t near_start_idx;

    if (line_width == 0 || line_width > IMAGE_HEIGHT) line_width = (uint16_t)((IMAGE_HEIGHT * 2U) / 5U);  // 默认优先分析更靠近车前的中下部赛道，减小远端噪声和提前入弯。

    end_idx = (uint16_t)(IMAGE_HEIGHT - (IMAGE_HEIGHT / 10U));
    if (end_idx > line_width) start_idx = (uint16_t)(end_idx - line_width);
    else start_idx = 0U;
    if (end_idx > MIDLINE_NEAR_ROWS) near_start_idx = (uint16_t)(end_idx - MIDLINE_NEAR_ROWS);
    else near_start_idx = start_idx;
    if (near_start_idx < start_idx) near_start_idx = start_idx;

    for (i = start_idx; i < end_idx; i++)
    {
        if (mid_line[i] >= 0 && mid_line[i] < IMAGE_WIDTH)
        {
            /* 这里把“行号 i”看成自变量，把“中心列号 mid_line[i]”看成因变量。 */
            float x = (float)i;
            float y = (float)mid_line[i];
            float weight = 1.0f + 1.5f * ((float)(i - start_idx) / (float)((end_idx - start_idx) ? (end_idx - start_idx) : 1U));

            sum_x += x;  // 累加行号。
            sum_y += y;  // 累加列号。
            sum_x2 += x * x;  // 累加行号平方。
            sum_xy += x * y;  // 累加行号与列号乘积。
            weighted_sum_y += y * weight;  // 更靠近车前的行给予更高权重。
            total_weight += weight;  // 累加偏差计算的总权重。
            features.valid_points++;  // 有效中线点数量加一。

            if (i >= near_start_idx)
            {
                near_sum_y += y;  // 记录底部近场区域的中线位置。
                near_count++;
            }
        }
    }

    if (features.valid_points == 0U)
    {
        return features;  // 没有有效点时直接返回全 0 特征。
    }

    /* 偏差更偏向底部近场，减少普通弯道被远端走势提前拉偏。 */
    if (total_weight <= 0.0f) total_weight = 1.0f;
    if (near_count > 0U)
    {
        float near_offset = (near_sum_y / (float)near_count) - (float)TARGET_LINE;
        float weighted_offset = (weighted_sum_y / total_weight) - (float)TARGET_LINE;
        features.offset_mean = MIDLINE_NEAR_WEIGHT * near_offset + (1.0f - MIDLINE_NEAR_WEIGHT) * weighted_offset;
    }
    else
    {
        features.offset_mean = (weighted_sum_y / total_weight) - (float)TARGET_LINE;
    }
    denominator = (float)features.valid_points * sum_x2 - sum_x * sum_x;  // 线性拟合斜率公式的分母。
    if (absf_local(denominator) > 0.001f)
    {
        features.slope = ((float)features.valid_points * sum_xy - sum_x * sum_y) / denominator;  // 用最小二乘拟合中线斜率。
    }
    features.curvature = get_signed_curvature(mid_line, start_idx, end_idx);  // 计算当前窗口的中线曲率。
    features.slope = apply_deadband(features.slope, STRAIGHT_SLOPE_DEADBAND);  // 对很小的斜率做死区处理。
    features.curvature = apply_deadband(features.curvature, STRAIGHT_CURVATURE_DEADBAND);  // 对很小的曲率做死区处理。

    return features;
}

/**
 * @brief 根据中线几何特征计算舵机控制误差
 * @param features 赛道几何特征
 * @return 转向误差
 * @details
 * 核心项是平均偏移，斜率和曲率是附加补偿项。
 */
static float calculate_line_deviation_from_features(const Track_Features *features)
{
    float steer_error;
    float offset_term;
    float curve_ratio;
    float offset_weight;

    offset_term = apply_deadband(features->offset_mean, STEER_OFFSET_DEADBAND);
    curve_ratio = clamp01_local(absf_local(features->curvature) / STEER_CURVE_FOLLOW_REF);
    offset_weight = 1.0f - (1.0f - STEER_OFFSET_CURVE_MIN_WEIGHT) * curve_ratio;

#if STEERING_ONLY_MODE
    steer_error = offset_term;  // 推车调舵机阶段只保留基础横向偏移，并给直道小抖动留死区。
#else
    steer_error = offset_term * offset_weight  // 弯道越明显，偏差项越只负责微调，不再主导整次回舵。
                + features->slope * STEER_SLOPE_GAIN  // 用中线走势提前给转向趋势。
                + features->curvature * STEER_CURVATURE_GAIN;  // 再叠加曲率前馈，让舵机更像顺着赛道弯。
#endif

    if (absf_local(steer_error) > STEER_LARGE_ERROR_THRESHOLD)
    {
        float excess = absf_local(steer_error) - STEER_LARGE_ERROR_THRESHOLD;
        if (steer_error > 0.0f) steer_error += excess * STEER_LARGE_ERROR_GAIN;
        else steer_error -= excess * STEER_LARGE_ERROR_GAIN;
    }

    return steer_error;
}

static float shape_steering_command(float steer_output, const Track_Features *features, float dt)
{
    float curvature_ratio;
    float curve_gain;
    float desired_cmd;
    float current_cmd;
    float max_delta;
    float delta_cmd;
    float normalized_cmd;
    float shaped_normalized_cmd;
    float cmd_sign = 1.0f;

    if (dt < DT_MIN_VALUE) dt = DT_MIN_VALUE;

    curvature_ratio = clamp01_local(absf_local(features->curvature) / STEER_CURVE_GAIN_REF);
    curve_gain = STEER_CURVE_GAIN_MIN + (STEER_CURVE_GAIN_MAX - STEER_CURVE_GAIN_MIN) * curvature_ratio;
    desired_cmd = steer_output * curve_gain;
    desired_cmd = limit_output(desired_cmd, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    if (desired_cmd < 0.0f) cmd_sign = -1.0f;
    normalized_cmd = clamp01_local(absf_local(desired_cmd) / (float)MAX_STEERING_ANGLE);
    shaped_normalized_cmd = 0.22f * normalized_cmd + 0.78f * normalized_cmd * normalized_cmd;  // 小偏差时明显更柔，大偏差再逐步抬升。
    desired_cmd = cmd_sign * shaped_normalized_cmd * (float)MAX_STEERING_ANGLE;

    current_cmd = car_state.steering_angle;
    if (desired_cmd * current_cmd < 0.0f)
    {
        desired_cmd = current_cmd * 0.58f + desired_cmd * 0.42f;  // 反向换向时减少对旧打角的保留，别让错误方向拖得太久。
        max_delta = STEER_REVERSE_RATE_DPS * dt;  // 过零换向时最慢，避免出弯后一下子抽到另一边。
    }
    else if (absf_local(desired_cmd) < absf_local(current_cmd))
    {
        desired_cmd = current_cmd * 0.30f + desired_cmd * 0.70f;  // 回舵阶段更快把控制权交还给新目标，压缩 S 线里的拖尾。
        max_delta = STEER_RELEASE_RATE_DPS * dt;  // 出弯回舵时刻意更慢，减轻反向偏摆。
    }
    else
    {
        max_delta = (STEER_ENTER_RATE_DPS + 40.0f * curvature_ratio) * dt;  // 进弯时允许更快加舵，曲率越大越敢打。
    }

    delta_cmd = desired_cmd - current_cmd;
    if (absf_local(desired_cmd) < 1.8f && absf_local(current_cmd) > 1.8f)
    {
        float near_center_limit = 30.0f * dt;  // 靠近回中时进一步放快收尾，减少回正末段拖太久继续摆向另一侧。
        delta_cmd = limit_output(delta_cmd, -near_center_limit, near_center_limit);
    }
    delta_cmd = limit_output(delta_cmd, -max_delta, max_delta);
    return current_cmd + delta_cmd;
}

/**
 * @brief 根据中线几何特征计算本周期目标速度
 * @param features 赛道几何特征
 * @return 目标速度
 * @details
 * 中线越偏、越斜、越弯，目标速度就越保守。
 */
static float calculate_curve_speed_target(const Track_Features *features)
{
    float reduction = 0.0f;  // 当前赛道状态总共需要减掉的速度量。
    float speed_target;  // 本周期最终目标速度。
    float valid_ratio = (float)features->valid_points / (float)IMAGE_HEIGHT;  // 有效中线点占整幅图高度的比例。

    reduction += CURVE_SPEED_GAIN * absf_local(filtered_steer_error);  // 偏差越大，减速越多。
    reduction += SLOPE_SPEED_GAIN * absf_local(features->slope);  // 斜率越大，减速越多。
    reduction += CURVATURE_SPEED_GAIN * absf_local(features->curvature);  // 曲率越大，减速越多。

    if (valid_ratio < 0.55f)
    {
        reduction += LOW_VALID_SPEED_PENALTY * (0.55f - valid_ratio) / 0.55f;  // 有效点比例偏低时额外保守降速。
    }

    speed_target = car_state.target_speed - reduction;  // 从基础目标速度中扣掉当前减速量。
    return limit_output(speed_target, MIN_SPEED, MAX_SPEED);  // 最终速度限制在允许范围内。
}

/**
 * @brief 初始化底盘控制模块
 * @details
 * 完成 PID、舵机、电机、编码器和内部状态变量初始化。
 */
void car_control_init(void)
{
    pid_init(&car_state.pid_steering, car_state.pid_steering.Kp, car_state.pid_steering.Ki, car_state.pid_steering.Kd);  // 初始化转向 PID。
    pid_init(&car_state.pid_speed, car_state.pid_speed.Kp, car_state.pid_speed.Ki, car_state.pid_speed.Kd);  // 初始化速度 PID。

    filtered_steer_error = 0.0f;
    failsafe_active = 0;
    valid_track_streak = 0;
    lost_track_streak = 0;
    car_debug_info.valid_mid_points = 0;  // 当前有效中线点数
    car_debug_info.track_valid = 0;        // 当前赛道是否有效
    car_debug_info.failsafe_active = 0;    // 当前是否处于 failsafe
    car_debug_info.steer_error = 0.0f;     // 当前转向误差
    car_debug_info.speed_target = 0.0f;    // 当前速度目标
    car_debug_info.speed_output = 0.0f;    // 当前电机输出
    car_state.current_speed = 0.0f;        // 启动时当前速度记为 0
    car_state.steering_angle = 0.0f;       // 当前转向角记为 0

    gpio_init(MOTOR1_GND_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);  // 电机 1 另一端固定拉低。
    gpio_init(MOTOR2_GND_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);  // 电机 2 另一端固定拉低。
    pwm_init(MOTOR1_PWM_PIN, MOTOR_PWM_FREQ, 0);  // 初始化电机 1 PWM，初始占空比为 0。
    pwm_init(MOTOR2_PWM_PIN, MOTOR_PWM_FREQ, 0);  // 初始化电机 2 PWM，初始占空比为 0。
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, servo_angle_to_duty(SERVO_CENTER_ANGLE));  // 初始化舵机 PWM，初始回中。

    encoder_quad_init(SPEED_ENCODER1, SPEED_ENCODER1_A_PIN, SPEED_ENCODER1_B_PIN);  // 初始化编码器 1 的正交解码。
    encoder_quad_init(SPEED_ENCODER2, SPEED_ENCODER2_A_PIN, SPEED_ENCODER2_B_PIN);  // 初始化编码器 2 的正交解码。
    encoder_clear_count(SPEED_ENCODER1);  // 清零编码器 1 计数。
    encoder_clear_count(SPEED_ENCODER2);  // 清零编码器 2 计数。
}

/**
 * @brief 触发紧急停车并进入保护状态
 * @details
 * 会清空 PID 内部状态，并让电机停转、舵机回中。
 */
void car_emergency_stop(void)
{
    failsafe_active = 1;                 // 打开保护状态标志
    valid_track_streak = 0;              // 清连续有效赛道帧计数
    lost_track_streak = 0;               // 清连续丢线帧计数
    filtered_steer_error = 0.0f;         // 清转向误差滤波状态
    car_debug_info.failsafe_active = 1;  // 同步调试状态里的 failsafe 标志
    car_debug_info.speed_target = 0.0f;  // 把调试速度目标清零
    clear_pid_state(&car_state.pid_steering);
    clear_pid_state(&car_state.pid_speed); // 清两个 PID 的内部状态
    last_motor_duty = 0.0f;
    apply_safe_output();
}

/**
 * @brief 小车主控制函数
 * @param mid_line 图像处理中得到的中线数组，满足 mid_line[y] = x_center
 * @param line_width 参与控制分析的行数，0 表示整幅图高度
 * @param dt 控制周期，单位秒
 * @details
 * 控制流程如下：
 * 1. 从中线提取赛道几何特征；
 * 2. 判断中线是否有效；
 * 3. 若短时丢线，则先短暂容错；
 * 4. 若连续丢线，则进入保护状态；
 * 5. 若中线有效，则用中线偏差计算舵机控制量；
 * 6. 再用中线偏差、斜率、曲率计算速度目标；
 * 7. 最后由速度 PID 输出电机占空比。
 */
void car_control(int16_t mid_line[IMAGE_HEIGHT], uint16_t line_width, float dt)
{
    Track_Features track_features;  // 保存当前帧赛道几何特征
    float steer_error;              // 当前综合转向误差
    float steer_output;             // 转向 PID 算出的舵机输出
    float speed_feedback;           // 当前编码器测得的速度反馈
    float speed_target;             // 当前帧动态计算出的目标速度
    float speed_output;             // 速度 PID 算出的电机输出
    uint8_t track_valid;            // 当前赛道是否有效的判断结果
    if (dt < DT_MIN_VALUE) dt = DT_MIN_VALUE;  // 控制周期过小时强制抬到最小值。

    track_features = analyze_track_features(mid_line, line_width);  // 先从中线提取几何特征。
    car_debug_info.valid_mid_points = track_features.valid_points;  // 记录当前有效中线点数。

    track_valid = is_track_valid(mid_line, line_width);  // 判断这帧赛道是否足够可靠。
    car_debug_info.track_valid = track_valid;  // 记录赛道有效标志。
    car_debug_info.failsafe_active = failsafe_active;  // 记录当前是否处于保护状态。

    if (!track_valid)
    {
        if (lost_track_streak < 255U) lost_track_streak++;  // 连续丢线帧数加一。

        if (lost_track_streak >= TRACK_LOST_TOLERANCE_FRAMES)
        {
            car_emergency_stop();  // 连续丢线超过阈值，直接急停并进入保护。
        }
        else
        {
            control_steering(car_state.steering_angle);  // 短时丢线时保持上一拍舵机姿态，避免直道和小弯里突然回中。
            control_motor_duty(last_motor_duty);  // 短时丢线时保持上一拍电机输出，避免动力一拍一拍抽断。
            car_debug_info.speed_output = last_motor_duty;
        }
        return;  // 本拍赛道无效，不再继续后面的正常控制。
    }

    lost_track_streak = 0;  // 只要当前帧有效，就清连续丢线计数。

    if (failsafe_active)
    {
        if (valid_track_streak < 255U) valid_track_streak++;  // 连续有效赛道帧数加一。

        if (valid_track_streak < FAILSAFE_RELEASE_FRAMES)
        {
            apply_safe_output();  // 还没满足解除保护条件时，继续保持安全输出。
            return;  // 本拍不恢复正常控制。
        }

        failsafe_active = 0;  // 满足恢复条件后，退出保护状态。
        valid_track_streak = 0;  // 清恢复计数。
        clear_pid_state(&car_state.pid_steering);  // 清转向 PID 历史状态。
        clear_pid_state(&car_state.pid_speed);  // 清速度 PID 历史状态。
        filtered_steer_error = 0.0f;  // 清转向误差滤波状态。
    }

    /* 舵机控制直接使用中线提取出来的几何误差。 */
    steer_error = calculate_line_deviation_from_features(&track_features);
    filtered_steer_error = STEER_FILTER_ALPHA * steer_error + (1.0f - STEER_FILTER_ALPHA) * filtered_steer_error;  // 对转向误差做一阶低通滤波。
    car_debug_info.steer_error = filtered_steer_error;  // 记录滤波后的转向误差。

    steer_output = pid_calculate(&car_state.pid_steering, 0.0f, filtered_steer_error, dt);  // 计算本拍舵机控制输出。
    steer_output = shape_steering_command(steer_output, &track_features, dt);  // 按曲率和进/出弯状态整形成更平滑的舵机命令。
    control_steering(steer_output);  // 把舵机输出真正下发到硬件。

#if STEERING_ONLY_MODE
    control_motor_duty(0.0f);  // 推车调舵机阶段彻底关闭电机输出，避免速度环干扰判断。
    car_debug_info.speed_target = 0.0f;
    car_debug_info.speed_output = 0.0f;
    last_motor_duty = 0.0f;
    return;
#endif

#if CRAWL_TEST_MODE
    speed_output = limit_output(CRAWL_TEST_DUTY_PERCENT, 0.0f, 100.0f);  // 低速调环岛阶段改为开环恒定小占空比，避免低速闭环一冲一停。
    car_debug_info.speed_target = 0.0f;
    car_debug_info.speed_output = speed_output;
    car_debug_info.failsafe_active = failsafe_active;
    last_motor_duty = speed_output;
    control_motor_duty(speed_output);
    return;
#endif

    /* 电机速度也由这条中线的偏差、斜率、曲率共同决定。 */
    speed_target = calculate_curve_speed_target(&track_features);  // 根据赛道状态计算本拍目标速度。
    car_debug_info.speed_target = speed_target;  // 记录当前目标速度。

    speed_feedback = get_speed_feedback(dt);  // 读取当前速度反馈。
    car_state.current_speed = speed_feedback;  // 把测速结果写入车辆状态。

    speed_output = pid_calculate(&car_state.pid_speed, speed_target, speed_feedback, dt);  // 计算本拍电机输出。
    speed_output = limit_output(speed_output, 0.0f, SPEED_OUTPUT_LIMIT);  // 把电机输出限制在允许范围内。
    if (speed_output > 0.0f && speed_output < MOTOR_MIN_DUTY_PERCENT)
    {
        speed_output = MOTOR_MIN_DUTY_PERCENT;  // 赛道有效且已经准备给动力时，给一个可真正起转的最小占空比。
    }
    car_debug_info.speed_output = speed_output;  // 记录当前电机输出。
    car_debug_info.failsafe_active = failsafe_active;  // 再同步一次保护状态到调试信息。
    last_motor_duty = speed_output;  // 保存本拍有效输出，给短时丢线容错时复用。

    control_motor_duty(speed_output);  // 把电机占空比真正下发到硬件。
}

/**
 * @brief 获取当前车辆控制状态
 * @return 当前状态副本
 */
Car_State get_car_state(void)
{
    return car_state;
}

/**
 * @brief 获取当前控制调试信息
 * @return 调试信息副本
 */
Car_Debug_Info get_car_debug_info(void)
{
    return car_debug_info;
}

/**
 * @brief 设置基础目标速度
 * @param target_speed 目标速度
 */
void set_target_speed(float target_speed)
{
    car_state.target_speed = limit_output(target_speed, MIN_SPEED, MAX_SPEED);  // 保存并限制基础目标速度。
}

/**
 * @brief 设置 PID 参数
 * @param controller_type 控制器类型
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 */
void set_pid_parameters(uint8_t controller_type, float Kp, float Ki, float Kd)
{
    if (controller_type == CONTROLLER_STEERING)
    {
        car_state.pid_steering.Kp = Kp;  // 更新转向 PID 的比例系数。
        car_state.pid_steering.Ki = Ki;  // 更新转向 PID 的积分系数。
        car_state.pid_steering.Kd = Kd;  // 更新转向 PID 的微分系数。
    }
    else if (controller_type == CONTROLLER_SPEED)
    {
        car_state.pid_speed.Kp = Kp;  // 更新速度 PID 的比例系数。
        car_state.pid_speed.Ki = Ki;  // 更新速度 PID 的积分系数。
        car_state.pid_speed.Kd = Kd;  // 更新速度 PID 的微分系数。
    }
}

/**
 * @brief 重置指定 PID 的内部状态
 * @param controller_type 控制器类型
 */
void reset_pid(uint8_t controller_type)
{
    if (controller_type == CONTROLLER_STEERING)
    {
        clear_pid_state(&car_state.pid_steering);  // 重置转向 PID 历史状态。
    }
    else if (controller_type == CONTROLLER_SPEED)
    {
        clear_pid_state(&car_state.pid_speed);  // 重置速度 PID 历史状态。
    }
}
