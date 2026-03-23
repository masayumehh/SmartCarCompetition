#ifndef __CAR_CONTROL_H__  // 防止头文件被重复包含。
#define __CAR_CONTROL_H__  // 打开本头文件的包含保护。

#include "stdint.h"  // 提供 uint8_t、uint16_t 等标准整型。
#include "img_processing.h"  // 提供图像尺寸和中线数组相关定义。
#include "tuning_params.h"  // 提供控制和调参宏。

/* 图像中期望的赛道中心列。 */
#define TARGET_LINE           (IMAGE_WIDTH / 2)

/* 控制器类型选择编号。 */
#define CONTROLLER_STEERING   0
#define CONTROLLER_SPEED      1

typedef struct {
    float current_speed;    // 当前速度状态缓存。
    float target_speed;     // 当前目标速度状态缓存。
    float steering_angle;   // 当前转向角状态缓存。
    float Kp;               // PID 比例系数。
    float Ki;               // PID 积分系数。
    float Kd;               // PID 微分系数。
    float integral;         // PID 积分累计值。
    float prev_error;       // PID 上一拍误差。
    float output;           // PID 上一次输出值。
} PID_Controller;           // 通用 PID 控制器结构体。

typedef struct {
    float current_speed;          // 当前车辆速度反馈。
    float target_speed;           // 当前车辆基础目标速度。
    float steering_angle;         // 当前车辆转向命令角。
    PID_Controller pid_steering;  // 转向 PID 控制器。
    PID_Controller pid_speed;     // 速度 PID 控制器。
} Car_State;                      // 小车整体控制状态。

typedef struct {
    uint16_t valid_mid_points;  // 当前帧有效中线点数量。
    uint8_t track_valid;        // 当前帧赛道是否有效。
    uint8_t failsafe_active;    // 当前是否处于保护状态。
    float steer_error;          // 当前转向误差。
    float speed_target;         // 当前速度目标值。
    float speed_output;         // 当前速度环输出值。
} Car_Debug_Info;               // 调试打印用状态快照。

void car_control_init(void);  // 初始化底盘控制模块。
void car_control(int16_t mid_line[IMAGE_HEIGHT], uint16_t line_width, float dt);  // 执行一拍完整控制。
void car_emergency_stop(void);  // 触发急停并进入保护状态。
Car_State get_car_state(void);  // 获取当前车辆状态副本。
Car_Debug_Info get_car_debug_info(void);  // 获取当前调试信息副本。
void set_target_speed(float target_speed);  // 设置基础目标速度。
void set_pid_parameters(uint8_t controller_type, float Kp, float Ki, float Kd);  // 设置指定 PID 参数。
void reset_pid(uint8_t controller_type);  // 清空指定 PID 内部状态。

#endif  // 结束头文件包含保护。
