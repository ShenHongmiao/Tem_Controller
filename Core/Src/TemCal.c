#include "stm32f1xx_hal.h"
#include "adc.h"
extern ADC_HandleTypeDef hadc1;
#include "TemCal.h"
#include <math.h>

/**
 * @brief 读取ADC1的值（适用于FreeRTOS任务）
 * @return uint16_t 返回ADC采集到的原始值，返回0表示转换失败
 */
uint16_t Read_ADC1_Value(void)
{
    uint16_t adc_value = 0;
    
    // 启动ADC转换
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0; // 启动失败
    }
    
    // 等待转换完成，增加超时时间适应FreeRTOS调度
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        adc_value = HAL_ADC_GetValue(&hadc1);
    }
    
    // 停止ADC
    HAL_ADC_Stop(&hadc1);
    
    return adc_value;
}

/**
 * @brief 从ADC读数计算NTC热敏电阻的温度
 * @param adc_value ADC采集到的原始读数 (0-4095)
 * @return float 返回计算出的温度，单位为摄氏度
 */


float NTC_CalculateTemperature(uint16_t adc_value)//将ADC读数转换为温度
{
    if (adc_value == 0) {
        return -273.15f; // 避免除零错误，返回绝对零度
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

