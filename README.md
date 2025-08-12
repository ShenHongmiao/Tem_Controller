# STM32温度控制程序
## GPIO设置
引脚	功能描述	配置模式
PA0	ADC1_IN0	模拟输入 (Analog Mode)
PB0	Control_1	推挽输出 (Push-Pull Output)
PA13	SWDIO	调试接口
PA14	SWCLK	调试接口
！[引脚设置]引脚设置.png
PA0 作为ADC的模拟输入，用于采集外部传感器的模拟信号。

PB0 被配置为推挽输出，用于向外部设备提供控制信号。
## ADC设置 (ADC Configuration)
本项目使用了 ADC1 模块，配置如下：

- 通用设置 (Common Settings)
- 模式： 独立模式 (Independent Mode)
- ADC常规转换模式 (ADC_Regular_ConversionMode)
- 转换通道： 仅启用一个通道，即 ADC1_IN0 (PA0)。
- 数据对齐： 右对齐 (Right Alignment)。
- 连续转换模式： 已启用 (Continuous Conversion Mode Enabled)。ADC完成一次转换后将立即启动下一次转换。
- 触发源： 由软件启动 (Regular Conversion launched by software)。只需软件触发一次，ADC即可进入连续转换状态。

## 模拟看门狗 (Analog Watchdog)
- 看门狗模式： 已启用 (Analog WatchDog Mode Enabled)
- 监控范围： 监控 ADC1_IN0 通道
   -阈值设置：
        高阈值： xxxx (请填写你的12位高阈值数值)
        低阈值： yyyy (请填写你的12位低阈值数值)