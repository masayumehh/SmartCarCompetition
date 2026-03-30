v1.0.0  完成基础图像处理和巡线功能，完成电机和舵机的控制，后续需要调参
v1.1.0  完成了十字路口和环岛的第一版判断，具体能否稳定完成判断并控制期待后续装车实测
v1.2.0  完成了路障和坡道的第一版判断，具体能否稳定完成判断并控制期待后续装车实测
            第一版巡线与元素识别代码已基本完成
v1.2.1
Keil 修复记录：

1. `stdint.h` 缺失
改为使用工程自带的 `zf_common_typedef.h`。

2. `uint8_t / uint16_t / int16_t` 未定义
在 `zf_common_typedef.h` 中补充 `*_t` 类型别名。

3. `uint8_t / uint16_t / uint32_t` 重复定义
在 `zf_common_typedef.h` 和 `stc.h` 中加入保护宏。

4. `zf_common_math.h` 缺失
删除 `img_processing.c` 中的无效引用。

5. `TUNE_MIDLINE_*` 宏未定义
在 `img_processing.h` 中补充 `tuning_params.h`。

6. `roundabout_element.c` 语法错误
按 C251 规则前移局部变量声明。

7. 高号中断不支持
屏蔽未使用的高号中断，相机 DMA 改为主循环轮询。

8. `printf` 参数过多
将长 `printf` 拆成多条短 `printf`。

9. `XDATA OVERFLOW`
删除 `edge_buffer`，去掉独立 `temp_buffer`，改为输出缓冲原地二值化。

结果：
编译通过，`0 Error`。
