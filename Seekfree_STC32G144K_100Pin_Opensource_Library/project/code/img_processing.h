#ifndef __IMG_PROCESSING_H__
#define __IMG_PROCESSING_H__

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#include "tuning_params.h"

/**
 * @file img_processing.h
 * @brief 图像处理模块头文件
 */

/* 图像尺寸（来自摄像头驱动） */
#define IMAGE_WIDTH  MT9V03X_W  // 图像宽度。
#define IMAGE_HEIGHT MT9V03X_H  // 图像高度。

/* OTSU 阈值参数 */
#define OTSU_BLOCK_SIZE      32   // 预留的 OTSU 统计块大小参数。
#define OTSU_THRESHOLD_INIT  128  // OTSU 阈值初始值。

/* 中值滤波窗口大小（当前实现仅支持 3） */
#define FILTER_SIZE 3  // 当前使用的中值滤波窗口大小。

/**
 * @brief 计算 OTSU 自适应阈值
 * @param src 输入灰度图
 * @param width 图像宽度
 * @param height 图像高度
 * @return 最佳阈值（0~255）
 */
uint8_t calculate_otsu_threshold(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height);  // 计算整帧 OTSU 阈值。

/**
 * @brief 3x3 中值滤波
 * @param src 输入图像
 * @param dst 输出图像
 * @param width 图像宽度
 * @param height 图像高度
 * @param filter_size 滤波窗口大小（建议 3）
 */
void simple_median_filter(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height, uint8_t filter_size);  // 对图像做中值滤波。

/**
 * @brief 单像素阈值二值化
 * @param value 输入灰度值
 * @param threshold 阈值
 * @return 二值化结果（0 或 255）
 */
uint8_t correct_threshold(uint8_t value, uint8_t threshold);  // 对单个像素做阈值二值化。

/**
 * @brief 全图阈值二值化
 */
void binarize_with_threshold(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint8_t dst[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height, uint8_t threshold);  // 对整幅图执行阈值二值化。

/**
 * @brief 提取赛道中线
 * @details
 * 输出数组的定义是：mid_line[y] = 第 y 行赛道中心所在的列号。
 * 也就是说，数组下标表示“行”，数组值表示“列”。
 */
void get_mid_line(uint8_t src[IMAGE_HEIGHT][IMAGE_WIDTH], uint16_t width, uint16_t height, int16_t mid_line[IMAGE_HEIGHT]);  // 从二值图提取逐行中线。

/**
 * @brief 图像处理主流程：滤波 -> 阈值 -> 二值化 -> 中线
 */
void image_processing(uint8_t output_buffer[IMAGE_HEIGHT][IMAGE_WIDTH], int16_t mid_line_buffer[IMAGE_HEIGHT]);  // 执行完整图像处理主流程。

/**
 * @brief 获取指定行的中线值
 * @details
 * 返回 mid_line_buffer[y]，也就是第 y 行赛道中心所在的列号。
 */
int16_t get_mid_line_at(uint16_t y, const int16_t mid_line_buffer[IMAGE_HEIGHT], uint16_t height);  // 读取指定行中线值。

#endif
