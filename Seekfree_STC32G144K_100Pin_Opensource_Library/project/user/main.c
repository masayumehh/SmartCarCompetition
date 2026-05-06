/*********************************************************************************************************************
* STC32G144K Opensourec Library
* Copyright (c) 2025 SEEKFREE
*********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "img_processing.h"
#include "car_control.h"
#include "cross_element.h"
#include "obstacle_element.h"
#include "roundabout_element.h"
#include "slope_element.h"
#include "start_line_element.h"
#include "tuning_params.h"

/**
 * @file main.c
 * @brief 视觉车主流程入口
 * @details
 * 主循环每收到一帧摄像头图像，就按以下顺序运行：
 * 1. 拷贝当前帧，避免 DMA 继续改写原始缓冲；
 * 2. 执行图像二值化与中线提取；
 * 3. 依次尝试发车线、环岛、路障、十字、坡道等元素逻辑；
 * 4. 根据当前激活元素选择普通中线或覆盖中线；
 * 5. 调用底盘控制模块输出舵机和电机命令。
 */

/* 看门狗式掉帧保护参数。 */
#define FRAME_TIMEOUT_MIN_MS         TUNE_FRAME_TIMEOUT_MIN_MS
#define FRAME_TIMEOUT_MAX_MS         TUNE_FRAME_TIMEOUT_MAX_MS
#define FRAME_TIMEOUT_FRAME_FACTOR   TUNE_FRAME_TIMEOUT_FRAME_FACTOR
/* 控制周期估计的低通滤波系数。 */
#define CONTROL_DT_FILTER_ALPHA      (0.25f)
/* 图传和调试相关开关。 */
#define ASSISTANT_SEND_EVERY_N_FRAMES TUNE_ASSISTANT_SEND_EVERY_N_FRAMES
#define CAMERA_RAW_STREAM_ONLY       TUNE_CAMERA_RAW_STREAM_ONLY
#define IMAGE_DEBUG_ONLY             TUNE_IMAGE_DEBUG_ONLY
#define IMAGE_DEBUG_CAMERA_FPS       TUNE_IMAGE_DEBUG_CAMERA_FPS
#define IMAGE_DEBUG_THRESHOLD_OFFSET TUNE_IMAGE_DEBUG_THRESHOLD_OFFSET
#define IMAGE_DEBUG_THRESHOLD_MIN    TUNE_IMAGE_DEBUG_THRESHOLD_MIN
#define IMAGE_DEBUG_THRESHOLD_MAX    TUNE_IMAGE_DEBUG_THRESHOLD_MAX
/* 发车线重新启用检测前的延时。 */
#define START_LINE_ENABLE_DELAY_MS   TUNE_START_LINE_ENABLE_DELAY_MS
#define START_LINE_POLL_INTERVAL     TUNE_START_LINE_POLL_INTERVAL
#define OBSTACLE_POLL_INTERVAL       TUNE_OBSTACLE_POLL_INTERVAL
#define CROSS_POLL_INTERVAL          TUNE_CROSS_POLL_INTERVAL
#define SLOPE_POLL_INTERVAL          TUNE_SLOPE_POLL_INTERVAL

static uint8_t far g_img_output[IMAGE_HEIGHT][IMAGE_WIDTH];            // 当前帧图像处理工作缓冲，灰度图会在此被改写成二值图。
static int16_t g_mid_line[IMAGE_HEIGHT];                              // 普通巡线模块输出的逐行中线。
static int16_t g_obstacle_override_mid_line[IMAGE_HEIGHT];            // 路障模块接管时使用的覆盖中线。
static int16_t g_cross_override_mid_line[IMAGE_HEIGHT];               // 十字模块接管时使用的覆盖中线。
static int16_t g_roundabout_override_mid_line[IMAGE_HEIGHT];          // 环岛模块接管时使用的覆盖中线。
static int16_t g_slope_override_mid_line[IMAGE_HEIGHT];               // 坡道模块接管时使用的覆盖中线。
static float g_control_dt = 0.01f;                                    // 当前估计得到的控制周期，供底盘控制和超时保护共同使用。
static volatile uint16_t g_frame_timeout_ms = 0;                      // 距离上一帧处理完成后已经过去的毫秒数。
static uint16_t g_frame_timeout_limit_ms = FRAME_TIMEOUT_MIN_MS;      // 依据当前帧率动态换算出的掉帧急停阈值。
static uint8_t g_run_completed = 0U;                                  // 单圈完赛后锁定停车标志。
static uint32_t g_start_line_elapsed_ms = 0U;                         // 自启动以来累计的运行时间，用于延后启用完赛检测。
static uint8_t g_start_line_detection_enabled = 0U;                   // 发车线完赛检测当前是否已经允许工作。
static uint8_t g_feature_poll_counter = 0U;                           // 元素降频轮询计数器。
static uint8_t g_obstacle_active_last = 0U;                           // 上一帧路障是否激活。
static uint8_t g_cross_active_last = 0U;                              // 上一帧十字是否激活。
static uint8_t g_slope_active_last = 0U;                              // 上一帧坡道是否激活。
static uint8_t g_assistant_send_divider = 0U;                         // 图传分频计数器。
static uint8_t g_assistant_frame_pending = 0U;                        // 当前帧是否被标记为待发送到助手。

/**
 * @brief 预留的图传初始化入口
 * @details
 * 当前版本暂时关闭图传，只保留函数壳，后续恢复时不用再改主流程结构。
 */
static void assistant_camera_init(void)
{
    /* 图传暂时关闭，保留空函数便于后续快速恢复。 */
}

/**
 * @brief 为当前帧打上“允许图传发送”的标记
 * @details
 * 通过分频减少图传发送频率，避免图传挤占实际控制周期。
 */
static void assistant_camera_prepare_current_frame(void)
{
    if (ASSISTANT_SEND_EVERY_N_FRAMES > 1U)
    {
        g_assistant_send_divider++;
        if (g_assistant_send_divider < ASSISTANT_SEND_EVERY_N_FRAMES)
        {
            g_assistant_frame_pending = 0U;
            return;
        }
        g_assistant_send_divider = 0U;
    }

    g_assistant_frame_pending = 1U;
}

/**
 * @brief 清除当前帧的待发送标记
 */
static void assistant_camera_flush_pending_frame(void)
{
    g_assistant_frame_pending = 0U;
}

/**
 * @brief 丢弃当前帧的待发送标记
 * @details
 * 常用于完赛停车或异常退出时，避免继续处理无意义的预览帧。
 */
static void assistant_camera_discard_pending_frame(void)
{
    g_assistant_frame_pending = 0U;
}

/**
 * @brief 对图像调试模式下的二值阈值做限幅修正
 * @param threshold 原始 OTSU 阈值
 * @return 修正后的调试阈值
 */
static uint8_t tune_image_debug_threshold(uint8_t threshold)
{
    int16_t adjusted = (int16_t)threshold + (int16_t)IMAGE_DEBUG_THRESHOLD_OFFSET;

    if (adjusted < (int16_t)IMAGE_DEBUG_THRESHOLD_MIN)
    {
        adjusted = (int16_t)IMAGE_DEBUG_THRESHOLD_MIN;
    }
    else if (adjusted > (int16_t)IMAGE_DEBUG_THRESHOLD_MAX)
    {
        adjusted = (int16_t)IMAGE_DEBUG_THRESHOLD_MAX;
    }

    return (uint8_t)adjusted;
}

/**
 * @brief 把摄像头 DMA 帧缓冲复制到当前处理缓冲
 * @details
 * 先复制一份静态快照，再做图像处理，避免 DMA 正在更新时读到半帧数据。
 */
static void copy_camera_frame_to_output(void)
{
    memcpy(g_img_output[0], mt9v03x_image[0], (uint16_t)(IMAGE_WIDTH * IMAGE_HEIGHT));
}


/**
 * @brief 1ms 看门狗节拍回调
 * @details
 * 只负责累计“当前已经多久没处理到新帧”，主循环据此决定是否急停。
 */
static void control_watchdog_tick(void)
{
    if (g_frame_timeout_ms < 0xFFFFU)
    {
        g_frame_timeout_ms++;
    }
}

/**
 * @brief 原子读取当前帧超时计数
 * @return 当前超时毫秒数快照
 */
static uint16_t get_frame_timeout_ms_snapshot(void)
{
    uint8_t interrupt_state;
    uint16_t frame_timeout_ms;

    interrupt_state = EA;
    EA = 0;
    frame_timeout_ms = g_frame_timeout_ms;
    EA = interrupt_state;

    return frame_timeout_ms;
}

/**
 * @brief 读取并清零当前帧超时计数
 * @return 自上次清零以来累计的毫秒数
 */
static uint16_t fetch_and_clear_frame_timeout_ms(void)
{
    uint8_t interrupt_state;
    uint16_t frame_timeout_ms;

    interrupt_state = EA;
    EA = 0;
    frame_timeout_ms = g_frame_timeout_ms;
    g_frame_timeout_ms = 0;
    EA = interrupt_state;

    return frame_timeout_ms;
}

/**
 * @brief 清零当前帧超时计数
 */
static void clear_frame_timeout_ms(void)
{
    uint8_t interrupt_state;

    interrupt_state = EA;
    EA = 0;
    g_frame_timeout_ms = 0;
    EA = interrupt_state;
}

/**
 * @brief 根据当前控制周期刷新掉帧急停阈值
 */
static void refresh_frame_timeout_limit(void)
{
    float timeout_ms;

    timeout_ms = g_control_dt * 1000.0f * FRAME_TIMEOUT_FRAME_FACTOR;
    g_frame_timeout_limit_ms = (uint16_t)timeout_ms;
    if (g_frame_timeout_limit_ms < FRAME_TIMEOUT_MIN_MS)
    {
        g_frame_timeout_limit_ms = FRAME_TIMEOUT_MIN_MS;
    }
    if (g_frame_timeout_limit_ms > FRAME_TIMEOUT_MAX_MS)
    {
        g_frame_timeout_limit_ms = FRAME_TIMEOUT_MAX_MS;
    }
}

/**
 * @brief 根据两帧之间的间隔更新控制周期估计值
 * @param frame_interval_ms 当前两帧间隔，单位毫秒
 */
static void update_control_dt_from_frame_interval(uint16_t frame_interval_ms)
{
    float measured_dt;

    if (frame_interval_ms == 0U)
    {
        return;
    }

    if (frame_interval_ms > FRAME_TIMEOUT_MAX_MS)
    {
        frame_interval_ms = FRAME_TIMEOUT_MAX_MS;
    }

    measured_dt = (float)frame_interval_ms / 1000.0f;
    if (measured_dt < TUNE_DT_MIN_VALUE)
    {
        measured_dt = TUNE_DT_MIN_VALUE;
    }

    g_control_dt += (measured_dt - g_control_dt) * CONTROL_DT_FILTER_ALPHA;
    refresh_frame_timeout_limit();
}

/**
 * @brief 程序主入口
 * @details
 * 初始化硬件、等待摄像头工作后，进入“取帧 -> 识别 -> 控制 -> 保护”的循环。
 */
void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);

    if (MT9V03X_FPS_DEF > 0)
    {
        g_control_dt = 1.0f / (float)MT9V03X_FPS_DEF;
    }

    refresh_frame_timeout_limit();
    pit_ms_init(TIM0_PIT, 1, control_watchdog_tick);

#if !CAMERA_RAW_STREAM_ONLY
    car_control_init();
    obstacle_element_init();
    cross_element_init();
    roundabout_element_init();
    slope_element_init();
    start_line_element_init();
    g_start_line_elapsed_ms = 0U;
    g_start_line_detection_enabled = 0U;
    set_target_speed(TUNE_DEFAULT_TARGET_SPEED);
#endif

    while (mt9v03x_init())
    {
        system_delay_ms(200);
    }
    /* assistant_camera_init(); */  // 暂时关闭图传初始化，优先保证实车控制周期。

    while (1)
    {
        if (g_run_completed)
        {
            clear_frame_timeout_ms();
            assistant_camera_discard_pending_frame();
            car_emergency_stop();
            if (mt9v03x_finish_flag)
            {
                mt9v03x_finish_flag = 0;
            }
            continue;
        }

        if (mt9v03x_finish_flag)
        {
#if CAMERA_RAW_STREAM_ONLY
            copy_camera_frame_to_output();
            assistant_camera_prepare_current_frame();
            assistant_camera_flush_pending_frame();
            mt9v03x_finish_flag = 0;
            fetch_and_clear_frame_timeout_ms();
            continue;
#else
            float obstacle_override_speed = TUNE_DEFAULT_TARGET_SPEED;
            float cross_override_speed = TUNE_DEFAULT_TARGET_SPEED;
            float roundabout_override_speed = TUNE_DEFAULT_TARGET_SPEED;
            float slope_override_speed = TUNE_DEFAULT_TARGET_SPEED;
            uint16_t frame_interval_ms;
#if IMAGE_DEBUG_ONLY
            uint8_t threshold;
            uint16_t debug_mask_y;
            uint16_t debug_mask_x;
#endif
            uint8_t obstacle_active;
              uint8_t cross_active;
              uint8_t roundabout_active;
              uint8_t slope_active;

            assistant_camera_prepare_current_frame();
            copy_camera_frame_to_output();
#if IMAGE_DEBUG_ONLY
            threshold = calculate_otsu_threshold(g_img_output, IMAGE_WIDTH, IMAGE_HEIGHT);
            threshold = tune_image_debug_threshold(threshold);
            binarize_with_threshold(g_img_output, g_img_output, IMAGE_WIDTH, IMAGE_HEIGHT, threshold);
            for (debug_mask_y = 0U; debug_mask_y < (IMAGE_HEIGHT / 4U); debug_mask_y++)
            {
                for (debug_mask_x = 0U; debug_mask_x < IMAGE_WIDTH; debug_mask_x++)
                {
                    g_img_output[debug_mask_y][debug_mask_x] = 0U;
                }
            }
            for (debug_mask_y = (uint16_t)(IMAGE_HEIGHT - (IMAGE_HEIGHT / 25U)); debug_mask_y < IMAGE_HEIGHT; debug_mask_y++)
            {
                for (debug_mask_x = 0U; debug_mask_x < IMAGE_WIDTH; debug_mask_x++)
                {
                    g_img_output[debug_mask_y][debug_mask_x] = 0U;
                }
            }
            get_mid_line(g_img_output, IMAGE_WIDTH, IMAGE_HEIGHT, g_mid_line);
            assistant_camera_flush_pending_frame();
            mt9v03x_finish_flag = 0;
            fetch_and_clear_frame_timeout_ms();
            continue;
#else
            mt9v03x_finish_flag = 0;
            frame_interval_ms = fetch_and_clear_frame_timeout_ms();
            g_feature_poll_counter++;
            update_control_dt_from_frame_interval(frame_interval_ms);
            if (g_start_line_elapsed_ms < 0xFFFFFFFFUL - (uint32_t)frame_interval_ms)
            {
                g_start_line_elapsed_ms += (uint32_t)frame_interval_ms;
            }
            else
            {
                g_start_line_elapsed_ms = 0xFFFFFFFFUL;
            }

            if (!g_start_line_detection_enabled && g_start_line_elapsed_ms >= START_LINE_ENABLE_DELAY_MS)
            {
                start_line_element_arm();
                g_start_line_detection_enabled = 1U;
            }

            image_processing(g_img_output, g_mid_line);
#endif
            if (g_start_line_detection_enabled &&
                (((g_feature_poll_counter % START_LINE_POLL_INTERVAL) == 0U) || g_run_completed) &&
                start_line_element_process(g_img_output, g_mid_line))
            {
                g_run_completed = 1U;
                assistant_camera_discard_pending_frame();
                car_emergency_stop();
                continue;
            }

            roundabout_active = roundabout_element_process(g_img_output,
                                                           g_mid_line,
                                                           g_roundabout_override_mid_line,
                                                           &roundabout_override_speed);
            if (roundabout_active)
            {
                obstacle_active = 0U;
                obstacle_element_init();
            }
            else
            {
                if (g_obstacle_active_last || ((g_feature_poll_counter % OBSTACLE_POLL_INTERVAL) == 0U))
                {
                    obstacle_active = obstacle_element_process(g_img_output,
                                                               g_mid_line,
                                                               g_obstacle_override_mid_line,
                                                               &obstacle_override_speed);
                }
                else
                {
                    obstacle_active = 0U;
                }
            }

            if (g_cross_active_last || ((g_feature_poll_counter % CROSS_POLL_INTERVAL) == 0U))
            {
                cross_active = cross_element_process(g_img_output,
                                                     g_mid_line,
                                                     g_cross_override_mid_line,
                                                     &cross_override_speed);
            }
            else
            {
                cross_active = 0U;
            }

            if (!obstacle_active && !roundabout_active && !cross_active)
            {
                if (g_slope_active_last || ((g_feature_poll_counter % SLOPE_POLL_INTERVAL) == 0U))
                {
                    slope_active = slope_element_process(g_img_output,
                                                         g_mid_line,
                                                         g_slope_override_mid_line,
                                                         &slope_override_speed);
                }
                else
                {
                    slope_active = 0U;
                }
            }
            else
            {
                slope_active = 0U;
                slope_element_init();
            }

            if (roundabout_active)
            {
                set_target_speed(roundabout_override_speed);
                car_control(g_roundabout_override_mid_line, 0U, g_control_dt);
            }
            else if (obstacle_active)
            {
                set_target_speed(obstacle_override_speed);
                car_control(g_obstacle_override_mid_line, 0U, g_control_dt);
            }
            else if (cross_active)
            {
                set_target_speed(cross_override_speed);
                car_control(g_cross_override_mid_line, 0U, g_control_dt);
            }
            else if (slope_active)
            {
                set_target_speed(slope_override_speed);
                car_control(g_slope_override_mid_line, 0U, g_control_dt);
            }
            else
            {
                set_target_speed(TUNE_DEFAULT_TARGET_SPEED);
                car_control(g_mid_line, 0U, g_control_dt);
            }

            g_obstacle_active_last = obstacle_active;
            g_cross_active_last = cross_active;
            g_slope_active_last = slope_active;

            assistant_camera_flush_pending_frame();
#endif
        }
        else if (get_frame_timeout_ms_snapshot() >= g_frame_timeout_limit_ms)
        {
#if !CAMERA_RAW_STREAM_ONLY
            car_emergency_stop();
#endif
        }
    }
}
















