/*********************************************************************************************************************
* STC32G144K Opensourec Library
* Copyright (c) 2025 SEEKFREE
*********************************************************************************************************************/
#include "zf_common_headfile.h"  // 逐飞公共驱动总头文件。
#include "img_processing.h"  // 图像处理模块接口。
#include "car_control.h"  // 底盘控制模块接口。
#include "cross_element.h"  // 十字元素状态机接口。
#include "obstacle_element.h"  // 路障元素状态机接口。
#include "roundabout_element.h"  // 环岛元素状态机接口。
#include "slope_element.h"  // 坡道元素状态机接口。
#include "tuning_params.h"  // 调参宏定义。

#define FRAME_TIMEOUT_MIN_MS        TUNE_FRAME_TIMEOUT_MIN_MS  // 帧超时下限。
#define FRAME_TIMEOUT_MAX_MS        TUNE_FRAME_TIMEOUT_MAX_MS  // 帧超时上限。
#define FRAME_TIMEOUT_FRAME_FACTOR  TUNE_FRAME_TIMEOUT_FRAME_FACTOR  // 控制周期到超时阈值的倍数。
#define DEBUG_PRINT_INTERVAL_FRAMES TUNE_DEBUG_PRINT_INTERVAL_FRAMES  // 调试打印分频值。

/*
 * 全局缓冲区说明：
 * g_img_output  : 图像处理输出缓冲（纯二值图）
 * g_mid_line    : 赛道中线数组，满足 g_mid_line[y] = 第 y 行的中心列号
 * g_control_dt  : 控制周期（秒），默认 0.01s，初始化后根据摄像头帧率修正
 */
static uint8_t g_img_output[IMAGE_HEIGHT][IMAGE_WIDTH];  // 图像处理输出缓冲。
static int16_t g_mid_line[IMAGE_HEIGHT];  // 当前帧提取出的中线数组。
static int16_t g_obstacle_override_mid_line[IMAGE_HEIGHT];  // 路障专用控制时使用的覆盖中线。
static int16_t g_cross_override_mid_line[IMAGE_HEIGHT];  // 十字专用控制时使用的覆盖中线。
static int16_t g_roundabout_override_mid_line[IMAGE_HEIGHT];  // 环岛专用控制时使用的覆盖中线。
static int16_t g_slope_override_mid_line[IMAGE_HEIGHT];  // 坡道专用控制时使用的覆盖中线。
static float g_control_dt = 0.01f;  // 控制周期，单位秒。
static volatile uint16_t g_frame_timeout_ms = 0;  // 距离上一帧到来的超时计数。
static uint16_t g_frame_timeout_limit_ms = FRAME_TIMEOUT_MIN_MS;  // 允许的最大帧间隔。
static uint16_t g_debug_print_divider = 0;  // 调试打印计数器。

static void control_watchdog_tick(void)
{
    if (g_frame_timeout_ms < 0xFFFFU)
    {
        g_frame_timeout_ms++;  // 每 1ms 增加一次帧超时计数。
    }
}

/**
 * @brief 输出一行控制调试信息
 * @note 为避免串口带宽被占满，主循环只会按固定间隔调用本函数。
 */
static void print_control_debug_info(void)
{
    Car_State state;  // 当前车辆状态快照。
    Car_Debug_Info debug_info;  // 当前调试信息快照。
    Obstacle_Debug_Info obstacle_debug;  // 当前路障状态机调试信息。
    Cross_Debug_Info cross_debug;  // 当前十字状态机调试信息。
    Roundabout_Debug_Info roundabout_debug;  // 当前环岛状态机调试信息。
    Slope_Debug_Info slope_debug;  // 当前坡道状态机调试信息。

    state = get_car_state();  // 读取当前车辆状态。
    debug_info = get_car_debug_info();  // 读取当前控制调试信息。
    obstacle_debug = obstacle_element_get_debug_info();  // 读取当前路障状态机信息。
    cross_debug = cross_element_get_debug_info();  // 读取当前十字状态机信息。
    roundabout_debug = roundabout_element_get_debug_info();  // 读取当前环岛状态机信息。
    slope_debug = slope_element_get_debug_info();  // 读取当前坡道状态机信息。

    printf("mid=%u track=%u stop=%u obs=%u os=%u ps=%u\r\n",
           debug_info.valid_mid_points,
           debug_info.track_valid,
           debug_info.failsafe_active,
           obstacle_debug.state,
           obstacle_debug.obstacle_side,
           obstacle_debug.pass_side);

    printf("oc=%d ow=%u ob=%u cross=%u wide=%u/%u/%u\r\n",
           obstacle_debug.obstacle_center,
           obstacle_debug.obstacle_width,
           obstacle_debug.obstacle_bottom_row,
           cross_debug.state,
           cross_debug.lower_width,
           cross_debug.middle_width,
           cross_debug.upper_width);

    printf("ring=%u dir=%u side=%u miss=%u supp=%u\r\n",
           roundabout_debug.state,
           roundabout_debug.direction,
           roundabout_debug.stable_side,
           roundabout_debug.missing_rows,
           roundabout_debug.supplement_width);

    printf("A=%d/%d B=%d/%d C=%d/%d\r\n",
           roundabout_debug.point_a.row,
           roundabout_debug.point_a.col,
           roundabout_debug.point_b.row,
           roundabout_debug.point_b.col,
           roundabout_debug.point_c.row,
           roundabout_debug.point_c.col);

    printf("slope=%u trap=%u flat=%u sw=%u/%u/%u\r\n",
           slope_debug.state,
           slope_debug.trapezoid_detected,
           slope_debug.flat_detected,
           slope_debug.lower_width,
           slope_debug.middle_width,
           slope_debug.upper_width);

    printf("sref=%u close=%u timeout=%u/%u\r\n",
           slope_debug.base_lower_width,
           slope_debug.close_rows,
           g_frame_timeout_ms,
           g_frame_timeout_limit_ms);

    printf("steer_err=%f steer=%f\r\n",
           debug_info.steer_error,
           state.steering_angle);

    printf("spd_fb=%f spd_tgt=%f duty=%f\r\n",
           state.current_speed,
           debug_info.speed_target,
           debug_info.speed_output);
}

void main(void)
{
    /* 1) 系统基础初始化 */
    clock_init(SYSTEM_CLOCK_96M);    /* 系统时钟初始化（必须） */
    debug_init();                    /* 调试串口初始化 */

    /* 2) 由摄像头默认帧率估算控制周期 dt */
    if (MT9V03X_FPS_DEF > 0)
    {
        g_control_dt = 1.0f / (float)MT9V03X_FPS_DEF;  // 用默认帧率估算每帧控制周期。
    }

    g_frame_timeout_limit_ms = (uint16_t)(g_control_dt * 1000.0f * FRAME_TIMEOUT_FRAME_FACTOR);
    if (g_frame_timeout_limit_ms < FRAME_TIMEOUT_MIN_MS)
    {
        g_frame_timeout_limit_ms = FRAME_TIMEOUT_MIN_MS;  // 过小时夹到最小超时值。
    }
    if (g_frame_timeout_limit_ms > FRAME_TIMEOUT_MAX_MS)
    {
        g_frame_timeout_limit_ms = FRAME_TIMEOUT_MAX_MS;  // 过大时夹到最大超时值。
    }

    pit_ms_init(TIM0_PIT, 1, control_watchdog_tick);  // 配置 1ms 周期的帧超时看门狗定时器。

    /* 3) 初始化底盘控制模块（舵机、电机、编码器、PID） */
    car_control_init();
    obstacle_element_init();  // 初始化路障元素状态机。
    cross_element_init();  // 初始化十字元素状态机。
    roundabout_element_init();  // 初始化环岛元素状态机。
    slope_element_init();  // 初始化坡道元素状态机。
    set_target_speed(TUNE_DEFAULT_TARGET_SPEED); /* 设置基础目标速度，可按实际需求调参 */

    /* 4) 初始化摄像头，失败则循环重试 */
    while (mt9v03x_init())
    {
        system_delay_ms(200);  // 摄像头初始化失败时延时后重试。
    }
    DMA_LCM_CFG &= (uint8_t)(~0x80U);  // Keil C251 不支持高号中断，这里改为主循环轮询 DMA 完成标志。

    /* 5) 主循环：每收到一帧图像，执行一次图像处理与控制输出 */
    while (1)
    {
        if ((DMA_LCM_STA & 0x03U) != 0U)
        {
            mt9v03x_dma_handler();  // 轮询触发 DMA 收尾逻辑，替代 DMA_LCM 中断入口。
        }

        if (mt9v03x_finish_flag)
        {
            float obstacle_override_speed = TUNE_DEFAULT_TARGET_SPEED;  // 路障专用控制时的目标速度。
            float cross_override_speed = TUNE_DEFAULT_TARGET_SPEED;  // 十字专用控制时的目标速度。
            float roundabout_override_speed = TUNE_DEFAULT_TARGET_SPEED;  // 环岛专用控制时的目标速度。
            float slope_override_speed = TUNE_DEFAULT_TARGET_SPEED;  // 坡道专用控制时的目标速度。
            uint8_t obstacle_active;  // 当前帧是否启用路障专用控制。
            uint8_t cross_active;  // 当前帧是否启用十字专用控制。
            uint8_t roundabout_active;  // 当前帧是否启用环岛专用控制。
            uint8_t slope_active;  // 当前帧是否启用坡道专用控制。

            mt9v03x_finish_flag = 0; /* 清帧完成标志，准备接收下一帧 */
            g_frame_timeout_ms = 0;  // 收到新帧后清空超时计数。

            image_processing(g_img_output, g_mid_line);          /* 图像处理：得到中线数组 */
            roundabout_active = roundabout_element_process(g_img_output,
                                                           g_mid_line,
                                                           g_roundabout_override_mid_line,
                                                           &roundabout_override_speed);  // 先给环岛模块机会建立状态，避免被路障误抢占。
            if (roundabout_active)
            {
                obstacle_active = 0U;
                obstacle_element_init();  // 环岛一旦确认接管，就主动清空路障状态，避免环岛内黑洞被当成砖块继续绕行。
            }
            else
            {
                obstacle_active = obstacle_element_process(g_img_output,
                                                           g_mid_line,
                                                           g_obstacle_override_mid_line,
                                                           &obstacle_override_speed);  // 只有当前没有环岛接管时，才运行路障识别与绕行。
            }
            cross_active = cross_element_process(g_img_output,
                                                 g_mid_line,
                                                 g_cross_override_mid_line,
                                                 &cross_override_speed);  // 再做十字识别与专用控制结果更新。
            if (!obstacle_active && !roundabout_active && !cross_active)
            {
                slope_active = slope_element_process(g_img_output,
                                                     g_mid_line,
                                                     g_slope_override_mid_line,
                                                     &slope_override_speed);  // 只有没有更强元素接管时，才运行坡道模块。
            }
            else
            {
                slope_active = 0U;
                slope_element_init();  // 更强元素触发时，重置坡道阶段状态，避免模块间状态互相污染。
            }

            if (roundabout_active)
            {
                set_target_speed(roundabout_override_speed);  // 环岛专用状态下切到保守目标速度。
                car_control(g_roundabout_override_mid_line, IMAGE_HEIGHT, g_control_dt);  // 环岛状态下使用补线后的覆盖中线控制。
            }
            else if (obstacle_active)
            {
                set_target_speed(obstacle_override_speed);  // 路障状态下切到更保守目标速度。
                car_control(g_obstacle_override_mid_line, IMAGE_HEIGHT, g_control_dt);  // 路障状态下使用绕行覆盖中线。
            }
            else if (cross_active)
            {
                set_target_speed(cross_override_speed);  // 十字专用状态下切到保守目标速度。
                car_control(g_cross_override_mid_line, IMAGE_HEIGHT, g_control_dt);  // 十字状态下使用专用直行中线控制。
            }
            else if (slope_active)
            {
                set_target_speed(slope_override_speed);  // 坡道状态下切到坡道阶段对应的目标速度。
                car_control(g_slope_override_mid_line, IMAGE_HEIGHT, g_control_dt);  // 坡道状态下使用稳像后的覆盖中线控制。
            }
            else
            {
                set_target_speed(TUNE_DEFAULT_TARGET_SPEED);  // 普通巡线时恢复基础目标速度。
                car_control(g_mid_line, IMAGE_HEIGHT, g_control_dt);  // 普通状态下继续使用真实中线控制。
            }

            /* 周期性打印调试信息，便于观察图像识别和控制是否一致。 */
            g_debug_print_divider++;  // 每处理一帧就加一次调试打印计数。
            if (g_debug_print_divider >= DEBUG_PRINT_INTERVAL_FRAMES)
            {
                g_debug_print_divider = 0;
                print_control_debug_info();
            }
        }
        else if (g_frame_timeout_ms >= g_frame_timeout_limit_ms)
        {
            car_emergency_stop();    /* 长时间未收到新帧，立即切断动力 */
        }
    }
}
