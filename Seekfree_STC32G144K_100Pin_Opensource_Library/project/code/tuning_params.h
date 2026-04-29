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
#define MAX_STEERING_ANGLE                 10      // 保留一点更大的最大打角，给圈型弯和更急的回头弯留出余量。
#define MIN_SPEED                          8       // 控制允许的最小速度目标。
#define MAX_SPEED                          30      // 控制允许的最大速度目标。

/* 舵机参数 */
#define TUNE_SERVO_CENTER_ANGLE            90.0f   // 舵机回中时的绝对角度。

/* 控制算法通用参数 */
#define TUNE_PID_INTEGRAL_LIMIT            200.0f  // PID 积分项限幅。
#define TUNE_SPEED_OUTPUT_LIMIT            45.0f   // 速度环输出上限。
#define TUNE_CURVE_SPEED_GAIN              0.22f   // 转向误差引起的减速增益。
#define TUNE_DT_MIN_VALUE                  0.001f  // 控制周期最小保护值。
#define TUNE_ENCODER_SPEED_SCALE           0.012f  // 编码器“每秒计数率”到速度反馈的缩放系数，减小反馈过大导致的电机抽动。
#define TUNE_STEER_FILTER_ALPHA            0.19f   // 速度提上来后略微放快转向误差响应，减少反打后长时间拖着不回中。
#define TUNE_STEER_CURVE_GAIN_MIN          0.82f   // 把普通弯基础舵量提回来，避免入弯反应太慢。
#define TUNE_STEER_CURVE_GAIN_MAX          1.18f   // 明显弯道时允许更积极地跟随曲率入弯。
#define TUNE_STEER_CURVE_GAIN_REF          6.0f    // 曲率增益从“小弯”过渡到“大弯”的参考曲率。
#define TUNE_STEER_ENTER_RATE_DPS          210.0f  // 明显提高入弯加舵速度，把普通弯转向响应提回来。
#define TUNE_STEER_RELEASE_RATE_DPS        110.0f  // 进一步加快回舵释放，缩短反打后的错误方向持续时间。
#define TUNE_STEER_REVERSE_RATE_DPS        42.0f   // 反向换向更快放手，减轻高速下左右来回摆成 S 线。

/* 弯道增强控制参数 */
#define TUNE_TRACK_LOST_TOLERANCE_FRAMES   4U      // 连续丢线多少帧后触发急停。
#define TUNE_STEER_SLOPE_GAIN              5.0f    // 保留走势前馈，但收一点，避免过度依赖远端趋势。
#define TUNE_STEER_CURVATURE_GAIN          1.0f    // 保留曲率前馈，让弯道能顺着走但不过度保守。
#define TUNE_STEER_OFFSET_DEADBAND         2.6f    // 再略增死区，让轻微跑偏先温和处理，避免刚偏一点就大幅修正。
#define TUNE_CURVATURE_SPEED_GAIN          4.0f    // 曲率对减速量的增益。
#define TUNE_SLOPE_SPEED_GAIN              5.0f    // 斜率对减速量的增益。
#define TUNE_LOW_VALID_SPEED_PENALTY       4.0f    // 有效点偏少时的额外减速惩罚。
#define TUNE_STRAIGHT_SLOPE_DEADBAND       0.10f   // 小斜率死区。
#define TUNE_STRAIGHT_CURVATURE_DEADBAND   0.80f   // 小曲率死区。

/* 图像处理中线提取参数 */
#define TUNE_MIDLINE_MIN_WHITE_RUN         2U      // 边界搜索时最短连续白段长度。
#define TUNE_MIDLINE_SEARCH_STEP           1U      // 中线边界搜索步长。
#define TUNE_MIDLINE_SMOOTH_WINDOW         2U      // 中线平滑窗口半宽。
#define TUNE_MIDLINE_MAX_CENTER_JUMP       14      // 近场相邻行中线允许的最大跳变量，抑制远端误检和反光白块。
#define TUNE_MIDLINE_MAX_WIDTH_JUMP        26      // 近场相邻行赛道宽度允许的最大跳变量，抑制单侧反光造成的宽度突变。
#define TUNE_MIDLINE_UPPER_CENTER_JUMP_BONUS 14    // 上半幅允许额外更大的中线跳变，避免十字/环岛特征被连续性约束压掉。
#define TUNE_MIDLINE_UPPER_WIDTH_JUMP_BONUS  28    // 上半幅允许额外更大的宽度跳变，给环岛入口和十字扩宽留空间。
#define TUNE_MIDLINE_NEAR_ROWS             10U     // 近场优先平均时使用的底部有效行数。
#define TUNE_MIDLINE_NEAR_WEIGHT           0.64f   // 再少看一点近场，减轻转向后因局部偏线而过度拉回。
#define TUNE_STEER_LARGE_ERROR_THRESHOLD   10.0f   // 提前触发大偏差补舵，解决入弯太慢。
#define TUNE_STEER_LARGE_ERROR_GAIN        0.62f   // 弯道和急弯给更强补舵。
#define TUNE_STEER_CURVE_FOLLOW_REF        5.0f    // 稍微放宽“明显弯道”的判定，别太早把偏差项压太低。
#define TUNE_STEER_OFFSET_CURVE_MIN_WEIGHT 0.55f   // 弯道里保留更多中线偏差权重，入弯更积极，不会只跟趋势慢慢磨进去。

/* 十字交叉路口元素参数 */
#define TUNE_CROSS_SAMPLE_ROW_LOWER_RATIO_NUM      3U     // 十字检测下采样行的分子比例。
#define TUNE_CROSS_SAMPLE_ROW_LOWER_RATIO_DEN      4U     // 十字检测下采样行的分母比例。
#define TUNE_CROSS_SAMPLE_ROW_MIDDLE_RATIO_NUM     1U     // 十字检测中采样行的分子比例。
#define TUNE_CROSS_SAMPLE_ROW_MIDDLE_RATIO_DEN     2U     // 十字检测中采样行的分母比例。
#define TUNE_CROSS_SAMPLE_ROW_UPPER_RATIO_NUM      1U     // 十字检测上采样行的分子比例。
#define TUNE_CROSS_SAMPLE_ROW_UPPER_RATIO_DEN      3U     // 十字检测上采样行的分母比例。
#define TUNE_CROSS_NORMAL_MIN_WIDTH                18U    // 普通直道最小宽度阈值。
#define TUNE_CROSS_NORMAL_MAX_WIDTH                82U    // 放宽普通直道最大宽度，避免透视和反光下入口被误排除。
#define TUNE_CROSS_WIDE_MIN_WIDTH                  82U    // 降低十字中部最小扩展宽度，先让十字更容易进入候选。
#define TUNE_CROSS_WIDTH_EXPAND_THRESHOLD          18U    // 放宽中部相对下部的扩展要求，适配当前视角。
#define TUNE_CROSS_CENTER_DIFF_LIMIT               24     // 允许更大的上下中心偏差，减少把轻微偏置十字漏掉。
#define TUNE_CROSS_DETECT_CONFIRM_FRAMES           2U     // 连续多少帧命中后确认进入十字。
#define TUNE_CROSS_LOST_CONFIRM_FRAMES             1U     // 十字图样一消失就更快准备退出，避免持续转弯段放手太晚。
#define TUNE_CROSS_EXIT_HOLD_FRAMES                2U     // 大幅缩短十字退出保持，减少经过十字后继续强制直行导致的丢线。
#define TUNE_CROSS_SPECIAL_TARGET_SPEED            10.0f  // 十字专用保守目标速度。

/* 环岛元素参数 */
#define TUNE_ROUNDABOUT_ENABLE                     0U     // 先暂时关闭环岛识别，集中把普通弯道和十字跑顺。
#define TUNE_ROUNDABOUT_LOWER_START_RATIO_NUM      3U     // 把下部参考区域再往上抬一点，更早感知入口几何变化。
#define TUNE_ROUNDABOUT_LOWER_START_RATIO_DEN      5U     // 参考区域从约 3/5 高度开始，兼顾入口预判和下部稳定性。
#define TUNE_ROUNDABOUT_REF_MIN_ROWS               6U     // 进一步降低建立下部直道参考所需行数，减少一侧方向完全进不了候选。
#define TUNE_ROUNDABOUT_NORMAL_MIN_WIDTH           18U    // 环岛入口下部普通直道最小宽度。
#define TUNE_ROUNDABOUT_NORMAL_MAX_WIDTH           84U    // 放宽环岛入口直道宽度上限，避免当前透视下入口被排除。
#define TUNE_ROUNDABOUT_CENTER_DIFF_LIMIT          24U    // 再放宽下部中心波动容忍，减轻某一侧入口透视变化造成的误排除。
#define TUNE_ROUNDABOUT_STABLE_EDGE_TOLERANCE      10U    // 稳定边允许更大轻微抖动，减少一侧方向因为边线不够“直”而漏检。
#define TUNE_ROUNDABOUT_MIN_STABLE_ROWS            6U     // 再降稳定边最少行数，让候选更早成立。
#define TUNE_ROUNDABOUT_ENTRY_OFFSET_MIN           2U     // 入口另一侧只要开始明显偏离就先进入候选，避免等主道快丢了才触发。
#define TUNE_ROUNDABOUT_RECOVER_MIN_ROWS           1U     // 找 B 点时不再要求太长恢复段，提升早期入口命中率。
#define TUNE_ROUNDABOUT_B_POINT_MIN_OFFSET         2U     // 进一步降低 B 点偏移要求，避免一侧方向始终过不了入口几何门槛。
#define TUNE_ROUNDABOUT_C_POINT_MIN_OFFSET         4U     // 进一步降低 C 点偏移要求，让小尺度入口弧形也能识别。
#define TUNE_ROUNDABOUT_MIN_MISSING_ROWS           2U     // 放宽单侧连续丢线要求，入口阶段只要出现明显缺边就先认为可疑。
#define TUNE_ROUNDABOUT_MIN_RISING_ROWS            2U     // 放宽弧线发展阶段持续要求。
#define TUNE_ROUNDABOUT_MIN_FALLING_ROWS           1U     // 放宽弧线回摆阶段持续要求。
#define TUNE_ROUNDABOUT_ARC_PEAK_MIN_OFFSET        6U     // 降低弧顶偏移要求，减轻晚识别。
#define TUNE_ROUNDABOUT_SUPPLEMENT_MIN_WIDTH       18U    // 估算补线距离的最小阈值。
#define TUNE_ROUNDABOUT_SUPPLEMENT_MAX_WIDTH       80U    // 估算补线距离的最大阈值。
#define TUNE_ROUNDABOUT_DETECT_CONFIRM_FRAMES      1U     // 先把入环确认放快，减少环岛入口识别偏晚。
#define TUNE_ROUNDABOUT_LOST_CONFIRM_FRAMES        2U     // 入口/环内短暂抖动少容错一帧，让状态切换更积极。
#define TUNE_ROUNDABOUT_EXIT_CONFIRM_FRAMES        1U     // 一旦恢复正常直道就更快开始出环，避免右转打得过晚。
#define TUNE_ROUNDABOUT_EXIT_HOLD_FRAMES           1U     // 再缩短出环保持，尽快把控制权还给普通巡线。
#define TUNE_ROUNDABOUT_EXIT_NORMAL_ROWS           8U     // 放宽回到正常直道所需的有效行数。
#define TUNE_ROUNDABOUT_EDGE_JITTER_TOLERANCE      3U     // 允许更大的边线小抖动，减少误退出。
#define TUNE_ROUNDABOUT_SPECIAL_TARGET_SPEED       9.0f   // 环岛专用保守目标速度。
#define TUNE_ROUNDABOUT_ENTRY_PRESTART_ROWS        16U    // 再提前一些行开始预拉，减轻前一弯回舵对环岛入口的干扰。
#define TUNE_ROUNDABOUT_EXIT_RING_PERCENT          10U    // 出环时只保留很小的环岛权重，避免放手太慢再次被拉回去。

/* 路障元素参数 */
#define TUNE_OBSTACLE_SCAN_TOP_RATIO_NUM           1U     // 路障搜索区域起始行分子比例。
#define TUNE_OBSTACLE_SCAN_TOP_RATIO_DEN           5U     // 路障搜索区域起始行分母比例。
#define TUNE_OBSTACLE_SCAN_BOTTOM_RATIO_NUM        4U     // 路障搜索区域结束行分子比例。
#define TUNE_OBSTACLE_SCAN_BOTTOM_RATIO_DEN        5U     // 路障搜索区域结束行分母比例。
#define TUNE_OBSTACLE_BLEND_START_RATIO_NUM        2U     // 绕行中线开始明显偏转的行分子比例。
#define TUNE_OBSTACLE_BLEND_START_RATIO_DEN        3U     // 绕行中线开始明显偏转的行分母比例。
#define TUNE_OBSTACLE_AVOID_TRIGGER_RATIO_NUM      3U     // 障碍物靠近到触发正式绕行的行分子比例。
#define TUNE_OBSTACLE_AVOID_TRIGGER_RATIO_DEN      5U     // 障碍物靠近到触发正式绕行的行分母比例。
#define TUNE_OBSTACLE_TRACK_MIN_WIDTH              18U    // 参与路障检测的最小赛道宽度。
#define TUNE_OBSTACLE_TRACK_MAX_WIDTH              90U    // 参与路障检测的最大赛道宽度。
#define TUNE_OBSTACLE_MIN_RUN_WIDTH                4U     // 单行内部黑块最小宽度。
#define TUNE_OBSTACLE_MAX_RUN_WIDTH                36U    // 单行内部黑块最大宽度。
#define TUNE_OBSTACLE_MIN_ROWS                     5U     // 内部黑块至少连续出现的行数。
#define TUNE_OBSTACLE_MIN_HEIGHT                   5U     // 障碍物包围盒最小高度。
#define TUNE_OBSTACLE_MIN_AREA                     28U    // 障碍物黑块总面积最小阈值。
#define TUNE_OBSTACLE_MIN_SIDE_GAP                 2U     // 障碍物与赛道边界至少保留的最小间隙。
#define TUNE_OBSTACLE_SIDE_OFFSET_MIN              5U     // 障碍物中心相对赛道中心最小偏移量。
#define TUNE_OBSTACLE_CLUSTER_CENTER_LIMIT         12U    // 连续命中行的障碍中心允许最大跳变。
#define TUNE_OBSTACLE_MAX_WIDTH_SPAN               12U    // 同一障碍在多行中的黑块宽度允许最大波动。
#define TUNE_OBSTACLE_MAX_EDGE_DRIFT               10U    // 同一障碍在多行中的左右边缘允许最大漂移量。
#define TUNE_OBSTACLE_DETECT_CONFIRM_FRAMES        2U     // 连续多少帧命中后确认进入路障状态。
#define TUNE_OBSTACLE_LOST_CONFIRM_FRAMES          2U     // 连续多少帧未命中后确认离开路障主体。
#define TUNE_OBSTACLE_EXIT_HOLD_FRAMES             5U     // 路障消失后继续保持绕行的帧数。
#define TUNE_OBSTACLE_CLEARANCE_PIXELS             8U     // 规划绕行通道时，和障碍物至少保留的像素安全余量。
#define TUNE_OBSTACLE_OUTSIDE_ALLOWANCE_PIXELS     6U     // 必要时允许覆盖中线临时偏出赛道边缘的像素余量。
#define TUNE_OBSTACLE_APPROACH_BLEND_PERCENT       55U    // 接近阶段把中线拉向绕行通道的基础比例。
#define TUNE_OBSTACLE_AVOID_BLEND_PERCENT          88U    // 正式绕行阶段把中线拉向绕行通道的基础比例。
#define TUNE_OBSTACLE_EXIT_BLEND_PERCENT           42U    // 出障阶段保留绕行偏置的基础比例。
#define TUNE_OBSTACLE_APPROACH_TARGET_SPEED        10.0f  // 路障接近阶段的保守目标速度。
#define TUNE_OBSTACLE_AVOID_TARGET_SPEED           8.0f   // 路障绕行阶段的更保守目标速度。

/* 坡道元素参数 */
#define TUNE_SLOPE_SAMPLE_BAND_HALF_HEIGHT         2U     // 坡道宽度采样时，每个采样行上下额外平均的行数。
#define TUNE_SLOPE_SAMPLE_ROW_LOWER_RATIO_NUM      3U     // 坡道下采样行分子比例。
#define TUNE_SLOPE_SAMPLE_ROW_LOWER_RATIO_DEN      4U     // 坡道下采样行分母比例。
#define TUNE_SLOPE_SAMPLE_ROW_MIDDLE_RATIO_NUM     11U    // 坡道中采样行分子比例。
#define TUNE_SLOPE_SAMPLE_ROW_MIDDLE_RATIO_DEN     20U    // 坡道中采样行分母比例。
#define TUNE_SLOPE_SAMPLE_ROW_UPPER_RATIO_NUM      7U     // 坡道上采样行分子比例。
#define TUNE_SLOPE_SAMPLE_ROW_UPPER_RATIO_DEN      20U    // 坡道上采样行分母比例。
#define TUNE_SLOPE_BLEND_START_RATIO_NUM           2U     // 坡道覆盖中线开始明显拉向引导中心的行分子比例。
#define TUNE_SLOPE_BLEND_START_RATIO_DEN           3U     // 坡道覆盖中线开始明显拉向引导中心的行分母比例。
#define TUNE_SLOPE_REF_MIN_ROWS                    8U     // 坡道几何分析要求的最少有效赛道行数。
#define TUNE_SLOPE_NORMAL_MIN_WIDTH                18U    // 坡道判定允许的普通直道最小宽度。
#define TUNE_SLOPE_NORMAL_MAX_WIDTH                90U    // 坡道判定允许的普通直道最大宽度。
#define TUNE_SLOPE_CENTER_DIFF_LIMIT               12U    // 三个采样高度中心允许的最大偏差。
#define TUNE_SLOPE_EDGE_SYMMETRY_LIMIT             10U    // 左右边线向内收缩量允许的最大不对称差。
#define TUNE_SLOPE_BASE_WIDTH_TOLERANCE            8U     // 当前宽度与普通直道参考宽度允许的基础偏差。
#define TUNE_SLOPE_FLAT_WIDTH_TOLERANCE            5U     // 平台/平直直道与参考宽度允许的偏差。
#define TUNE_SLOPE_TRAPEZOID_MIDDLE_SHRINK         6U     // 中采样宽度相对普通直道参考至少收窄多少才算梯形。
#define TUNE_SLOPE_TRAPEZOID_UPPER_SHRINK          10U    // 上采样宽度相对普通直道参考至少收窄多少才算梯形。
#define TUNE_SLOPE_CLOSE_ROW_SHRINK                4U     // 判定“已经接近坡脚”时，下部行至少要比参考收窄多少。
#define TUNE_SLOPE_CLOSE_MIN_ROWS                  5U     // 至少多少行明显收窄，才认为已经接近坡脚。
#define TUNE_SLOPE_DETECT_CONFIRM_FRAMES           2U     // 上坡/下坡梯形候选连续命中多少帧后确认切换。
#define TUNE_SLOPE_LOST_CONFIRM_FRAMES             3U     // 上坡接近阶段候选连续丢失多少帧后认为误触发。
#define TUNE_SLOPE_PLATFORM_CONFIRM_FRAMES         2U     // 宽度恢复平台外观后连续多少帧确认进入顶部平台。
#define TUNE_SLOPE_PLATFORM_MIN_HOLD_FRAMES        3U     // 顶部平台至少保持多少帧后才允许寻找下坡。
#define TUNE_SLOPE_EXIT_CONFIRM_FRAMES             2U     // 下坡结束后连续多少帧恢复平直外观才确认出坡。
#define TUNE_SLOPE_EXIT_HOLD_FRAMES                5U     // 出坡后继续保持坡道专用控制的帧数。
#define TUNE_SLOPE_MAX_PHASE_FRAMES                35U    // 某一坡道阶段最长允许保持多少帧，防止状态机卡死。
#define TUNE_SLOPE_APPROACH_BLEND_PERCENT          35U    // 上坡接近阶段覆盖中线向引导中心拉直的基础比例。
#define TUNE_SLOPE_CLIMB_BLEND_PERCENT             58U    // 上坡阶段覆盖中线向引导中心拉直的基础比例。
#define TUNE_SLOPE_PLATFORM_BLEND_PERCENT          25U    // 顶部平台阶段覆盖中线向引导中心拉直的基础比例。
#define TUNE_SLOPE_DOWNHILL_BLEND_PERCENT          62U    // 下坡阶段覆盖中线向引导中心拉直的基础比例。
#define TUNE_SLOPE_EXIT_BLEND_PERCENT              30U    // 出坡阶段覆盖中线向引导中心拉直的基础比例。
#define TUNE_SLOPE_APPROACH_TARGET_SPEED           12.0f  // 上坡前补速阶段的目标速度。
#define TUNE_SLOPE_CLIMB_TARGET_SPEED              12.0f  // 上坡过程中保持动力的目标速度。
#define TUNE_SLOPE_PLATFORM_TARGET_SPEED           10.0f  // 顶部平台阶段的回收目标速度。
#define TUNE_SLOPE_DOWNHILL_TARGET_SPEED           8.0f   // 下坡阶段的保守目标速度。
#define TUNE_SLOPE_EXIT_TARGET_SPEED               10.0f  // 出坡回正阶段的过渡目标速度。

/* 发车线/单圈停车参数 */
#define TUNE_START_LINE_TRACK_MIN_WIDTH            18U    // 发车线识别时，直道行允许的最小赛道宽度。
#define TUNE_START_LINE_TRACK_MAX_WIDTH            90U    // 发车线识别时，直道行允许的最大赛道宽度。
#define TUNE_START_LINE_CENTER_DIFF_LIMIT          10U    // 直道行中心相对引导中心允许的最大偏差。
#define TUNE_START_LINE_SAMPLE_HALF_WIDTH          3U     // 发车线检测时，中心采样窗口半宽。
#define TUNE_START_LINE_DARK_PIXEL_MIN_COUNT       5U     // 一行被视为黑带所需的最少黑像素数。
#define TUNE_START_LINE_BAND_MIN_ROWS              2U     // 单条黑带最小厚度（图像行数）。
#define TUNE_START_LINE_BAND_MAX_ROWS              12U    // 单条黑带最大厚度（图像行数）。
#define TUNE_START_LINE_GAP_MIN_ROWS               2U     // 两条黑带之间最小白间隔厚度。
#define TUNE_START_LINE_GAP_MAX_ROWS               12U    // 两条黑带之间最大白间隔厚度。
#define TUNE_START_LINE_BAND_MAX_DIFF_ROWS         4U     // 两条黑带厚度允许的最大差值。
#define TUNE_START_LINE_APPROACH_MIN_ROWS          10U    // 双胶带前方至少需要的有效直道行数。
#define TUNE_START_LINE_ARM_CLEAR_FRAMES           6U     // 起步后连续多少帧看不到发车线才算驶离发车区。
#define TUNE_START_LINE_DETECT_CONFIRM_FRAMES      2U     // 回到发车线时连续多少帧确认完赛。
#define TUNE_START_LINE_ENABLE_DELAY_MS            40000U // 起步后 40 秒再启用斑马线完赛检测，避免前半圈参与识别。

/* 防跑飞与保护参数 */
#define TUNE_TRACK_VALID_MIN_POINTS        16      // 赛道有效判定最小点数。
#define TUNE_FAILSAFE_RELEASE_FRAMES       5       // 退出保护前需要的连续有效帧数。

/* 调试输出参数 */
#define TUNE_DEBUG_PRINT_INTERVAL_FRAMES   10U     // 每隔多少帧打印一次调试信息。
#define TUNE_ASSISTANT_SEND_EVERY_N_FRAMES 6U      // 实车调舵机阶段降低图传频率，减少对控制周期的干扰。
#define TUNE_CAMERA_RAW_STREAM_ONLY        0U      // 原始图传模式：只发送摄像头原图，先确认摄像头和图传链路正常。
#define TUNE_IMAGE_DEBUG_ONLY              0U      // 实车控制模式：恢复元素识别与整车控制链路。
#define TUNE_IMAGE_DEBUG_CAMERA_FPS        20U     // 图像调试模式下把摄像头帧率降下来，减小 DMA 覆盖当前帧的风险。
#define TUNE_IMAGE_DEBUG_THRESHOLD_OFFSET  (-2)    // 二值调试阶段收紧一点阈值，减少远端墙面和杂线被误保留。
#define TUNE_IMAGE_DEBUG_THRESHOLD_MIN     70U     // 二值调试阈值下限，避免过暗时阈值过低导致整屏发白。
#define TUNE_IMAGE_DEBUG_THRESHOLD_MAX     170U    // 二值调试阈值上限，避免过亮时阈值过高吃掉白赛道。
#define TUNE_IMAGE_MASK_TOP_RATIO_NUM      1U      // 全局二值图顶部遮罩比例分子。
#define TUNE_IMAGE_MASK_TOP_RATIO_DEN      6U      // 顶部只轻遮罩，给十字和环岛保留更多远端信息。
#define TUNE_IMAGE_MASK_BOTTOM_RATIO_NUM   1U      // 全局二值图底部遮罩比例分子。
#define TUNE_IMAGE_MASK_BOTTOM_RATIO_DEN   40U     // 底部只裁掉极近处一小条，保留更多当前帧可用赛道信息。

/* 掉帧超时参数 */
#define TUNE_FRAME_TIMEOUT_MIN_MS          120U    // 帧超时下限。
#define TUNE_FRAME_TIMEOUT_MAX_MS          500U    // 帧超时上限。
#define TUNE_FRAME_TIMEOUT_FRAME_FACTOR    3.0f    // 用控制周期换算超时阈值的倍数。

/* 默认目标速度 */
#define TUNE_DEFAULT_TARGET_SPEED          8.0f    // 打开慢速自跑测试，先给一个很低的基础目标速度。
#define TUNE_MOTOR_MIN_DUTY_PERCENT        12.0f   // 慢速测试时下调最小占空比，尽量让小车低速平顺爬行。
#define TUNE_CRAWL_TEST_MODE               1U      // 低速调元素阶段直接用开环恒定小占空比，避免低速闭环一顿一顿。
#define TUNE_CRAWL_TEST_DUTY_PERCENT       5.2f    // 稍微把慢跑速度提一点，减轻过慢导致的入弯拖沓。
#define TUNE_TARGET_LINE_OFFSET            0       // 期望中线相对图像中心的静态偏置，直道持续跑偏时在这里微调。
#define TUNE_STEERING_ONLY_MODE            0U      // 切到低速自跑测试模式，方便不推车复现环岛进出弯问题。

/* 默认 PID 参数 */
#define TUNE_STEERING_KP                   0.48f   // 收一点比例，减轻偏离中线后一次性回舵过猛。
#define TUNE_STEERING_KI                   0.00f   // 转向 PID 积分系数。
#define TUNE_STEERING_KD                   0.12f   // 微增一点微分，用来给回正过程加阻尼，减少左右摆。
#define TUNE_SPEED_KP                      1.00f   // 速度 PID 比例系数。
#define TUNE_SPEED_KI                      0.04f   // 速度 PID 积分系数。
#define TUNE_SPEED_KD                      0.00f   // 速度 PID 微分系数。

#endif
