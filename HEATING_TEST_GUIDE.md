# 加热测试模式使用指南

## 概述
加热测试模式允许您在不使用PID控制的情况下，以固定的输出状态测试加热器，用于硬件调试和功能验证。

## 配置参数
位置：`Core/Src/freertos.c` 文件中的宏定义

```c
// 加热测试模式配置
#define HEATING_TEST_MODE       0       // 加热测试模式 (0=关闭, 1=开启)
#define TEST_PWM_DUTY_CYCLE     50.0f   // 测试模式PWM占空比 (%)
#define TEST_RELAY_STATE        0       // 测试模式继电器状态 (0=关闭, 1=开启)
#define STARTUP_DELAY_MS        2000    // 上电后延时时间 (ms)
```

## 使用步骤

### 1. PWM模式测试
```c
// 设置控制模式为PWM
#define CONTROL_MODE            CONTROL_MODE_PWM

// 启用测试模式
#define HEATING_TEST_MODE       1

// 设置测试占空比（例如30%）
#define TEST_PWM_DUTY_CYCLE     30.0f
```

**编译并烧录后的行为**：
- 上电后PWM输出保持0% 2秒
- 之后PWM输出固定30%占空比
- 串口显示：`TEST - Temp: xx.x C, PWM: 30.0%, Output: ON/OFF`

### 2. 继电器模式测试
```c
// 设置控制模式为继电器
#define CONTROL_MODE            CONTROL_MODE_RELAY

// 启用测试模式
#define HEATING_TEST_MODE       1

// 设置继电器状态（1=开启加热）
#define TEST_RELAY_STATE        1
```

**编译并烧录后的行为**：
- 上电后继电器保持关闭状态2秒
- 之后继电器固定开启状态
- 串口显示：`TEST - Temp: xx.x C, Relay: ON`

### 3. 恢复正常PID控制
```c
// 关闭测试模式
#define HEATING_TEST_MODE       0
```

**编译并烧录后恢复正常PID控制功能**

## 安全注意事项

### ⚠️ 重要安全提醒
1. **温度监控**：测试模式下必须持续监控温度，防止过热
2. **时间限制**：不要长时间运行高功率测试，建议测试时间不超过5分钟
3. **渐进测试**：PWM占空比从低到高逐步测试（如10%→30%→50%）
4. **应急准备**：准备随时断电的应急措施

### 安全保护机制
- **冷启动**：上电后自动延时2秒，输出强制关闭
- **固定输出**：忽略温度反馈，避免失控振荡
- **状态显示**：串口明确提示当前为测试模式

## 典型测试场景

### 1. 硬件验证
```c
#define HEATING_TEST_MODE       1
#define CONTROL_MODE            CONTROL_MODE_RELAY
#define TEST_RELAY_STATE        1
```
用于验证继电器和加热器硬件连接是否正常。

### 2. PWM功能测试
```c
#define HEATING_TEST_MODE       1
#define CONTROL_MODE            CONTROL_MODE_PWM
#define TEST_PWM_DUTY_CYCLE     25.0f
```
用于测试PWM输出和加热器响应特性。

### 3. 功率校准
通过不同占空比测试，确定合适的PID输出范围：
- 10% → 观察加热效果
- 30% → 记录升温速率
- 50% → 测量最大功率

## 故障排除

### 问题：测试模式下没有输出
**可能原因**：
1. `HEATING_TEST_MODE`设置为0
2. `TEST_RELAY_STATE`设置为0（继电器模式）
3. `TEST_PWM_DUTY_CYCLE`设置为0（PWM模式）

### 问题：串口显示正常模式信息
**可能原因**：
1. 没有重新编译烧录
2. 宏定义设置错误

### 问题：上电后立即加热
**说明**：这是异常情况，系统设计上电后必须延时2秒
**解决**：检查代码是否正确烧录，必要时重新烧录

## 测试记录模板

| 测试项目 | 模式 | 设定值 | 实际输出 | 温度变化 | 备注 |
|---------|------|--------|----------|----------|------|
| 继电器开启 | Relay | ON | ✓ | +2℃/min | 正常 |
| PWM 25% | PWM | 25% | ✓ | +1℃/min | 适中 |
| PWM 50% | PWM | 50% | ✓ | +3℃/min | 较快 |
| PWM 75% | PWM | 75% | ✓ | +5℃/min | 过快 |

记录测试数据有助于优化PID参数和确定安全工作范围。
