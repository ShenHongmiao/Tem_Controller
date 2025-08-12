#include "TemCal.h"
#include <math.h>

/**
 * @brief 从ADC读数计算NTC热敏电阻的温度
 * @param adc_value ADC采集到的原始读数 (0-4095)
 * @return float 返回计算出的温度，单位为摄氏度
 */
float NTC_CalculateTemperature(uint16_t adc_value)
{
    if (adc_value == 0) {
        return -273.15f; // 绝对零度
    }

    // 1. 根据ADC读数和分压电路计算热敏电阻的电阻值 R_T
    float R_T = ( (float)adc_value * PULL_UP_RES ) / (ADC_MAX_VALUE - (float)adc_value);

    // 2. 根据B参数方程计算温度（开尔文）
    // 1/T = 1/T25 + (1/Beta) * ln(R_T / R25)
    float T_kelvin = 1.0f / ( (1.0f / NTC_T25) + (1.0f / NTC_BETA) * logf(R_T / NTC_R25) );

    // 3. 将开尔文温度转换为摄氏度
    float T_celsius = T_kelvin - 273.15f;

    return T_celsius;
}