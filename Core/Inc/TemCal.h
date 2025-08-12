#ifndef __NTC_H
#define __NTC_H

#include <stdint.h>

// 热敏电阻和分压电路参数
#define NTC_R25         10000.0       // 25摄氏度时的标称电阻值 (10kΩ)
#define NTC_BETA        3977.0        // B参数 (3977K)
#define NTC_T25         298.15        // 25摄氏度的开尔文温度
#define PULL_UP_RES     10000.0       // 串联电阻的阻值 (10kΩ)
#define ADC_VREF_VOLT   5.0           // ADC参考电压 (5V)
#define ADC_MAX_VALUE   4095.0        // 12位ADC的最大值

/**
 * @brief 从ADC读数计算NTC热敏电阻的温度
 * @param adc_value ADC采集到的原始读数 (0-4095)
 * @return float 返回计算出的温度，单位为摄氏度
 */
float NTC_CalculateTemperature(uint16_t adc_value);

/**
 * @brief 读取ADC1的值（阻塞方式）
 * @return uint16_t 返回ADC采集到的原始值
 */
uint16_t Read_ADC1_Value(void);

/**
 * @brief 将ADC值转换为电压
 * @param adc_value ADC采集到的原始值 (0-4095)
 * @return float 返回电压值（单位：V）
 */
float ADC_ValueToVoltage(uint16_t adc_value);

#endif /* __NTC_H */