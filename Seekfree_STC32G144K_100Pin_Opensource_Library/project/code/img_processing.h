#ifndef __IMG_PROCESSING_H__
#define __IMG_PROCESSING_H__

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#include "tuning_params.h"

/**
 * @file img_processing.h
 * @brief 图像预处理与中线提取模块接口
 * @details
 * 该模块直接处理摄像头灰度图，输出两类结果：
 * 1. 供元素识别继续使用的二值图；
 * 2. 供循线控制使用的逐行中线数组。
 */

/* 图像尺寸直接复用摄像头驱动配置。 */
#define IMAGE_WIDTH            MT9V03X_W
#define IMAGE_HEIGHT           MT9V03X_H

/* 图像处理基础参数。 */
#define OTSU_BLOCK_SIZE        32    // 预留的 OTSU 统计块大小参数，当前实现未单独分块。
#define OTSU_THRESHOLD_INIT    128   // OTSU 阈值的默认初值和异常回退值。
#define FILTER_SIZE            3     // 当前中值滤波窗口大小，代码按 3x3 实现。

/**
 * @brief 计算整帧图像的 OTSU 自适应阈值
 * @param src 输入灰度图
 * @param width 图像宽度
 * @param height 图像高度
 * @return 当前帧建议使用的二值化阈值
 */
uint8_t calculate_otsu_threshold(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height);

/**
 * @brief 对输入图像执行 3x3 中值滤波
 * @param src 输入图像
 * @param dst 输出图像
 * @param width 图像宽度
 * @param height 图像高度
 * @param filter_size 滤波窗口尺寸，当前只支持 3
 */
void simple_median_filter(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH],
                          uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH],
                          uint16_t width,
                          uint16_t height,
                          uint8_t filter_size);

/**
 * @brief 对单个像素按阈值执行二值化
 * @param value 输入灰度值
 * @param threshold 二值化阈值
 * @return 大于等于阈值返回 255，否则返回 0
 */
uint8_t correct_threshold(uint8_t value, uint8_t threshold);

/**
 * @brief 对整幅图像执行阈值二值化
 * @param src 输入灰度图
 * @param dst 输出二值图
 * @param width 图像宽度
 * @param height 图像高度
 * @param threshold 本帧使用的阈值
 */
void binarize_with_threshold(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH],
                             uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH],
                             uint16_t width,
                             uint16_t height,
                             uint8_t threshold);

/**
 * @brief 从二值图中逐行提取赛道中线
 * @param src 输入二值图
 * @param width 图像宽度
 * @param height 图像高度
 * @param mid_line 输出中线数组
 * @details
 * 中线数组采用 `mid_line[y] = x_center` 的定义：
 * 数组下标表示图像行号，数组值表示该行赛道中心所在列号。
 */
void get_mid_line(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH],
                  uint16_t width,
                  uint16_t height,
                  int16_t mid_line[IMAGE_HEIGHT]);

/**
 * @brief 执行完整图像处理主流程
 * @param output_buffer 输入为当前帧灰度图，输出会被原地改写为二值图
 * @param mid_line_buffer 输出中线数组
 * @details
 * 主流程为：阈值估计 -> 二值化 -> 上下遮罩 -> 中线提取。
 */
void image_processing(uint8_t output_buffer[IMAGE_HEIGHT][IMAGE_WIDTH],
                      int16_t mid_line_buffer[IMAGE_HEIGHT]);

/**
 * @brief 读取指定行的中线值
 * @param y 需要读取的行号
 * @param mid_line_buffer 中线数组
 * @param height 当前图像高度
 * @return 若行号合法则返回该行中线列号，否则返回 -1
 */
int16_t get_mid_line_at(uint16_t y, const int16_t mid_line_buffer[IMAGE_HEIGHT], uint16_t height);

#endif
