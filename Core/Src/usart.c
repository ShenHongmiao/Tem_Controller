/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/**
 * @brief 将浮点数分解为整数和小数部分进行格式化
 * @param value: 浮点数值
 * @param decimals: 小数位数(支持1-3位)
 * @param buffer: 输出缓冲区
 * @retval None
 */
void FloatToString(float value, int decimals, char* buffer)
{
    int int_part, frac_part;
    int multiplier = 1;
    
    // 计算小数部分的乘数
    for (int i = 0; i < decimals; i++) {
        multiplier *= 10;
    }
    
    // 处理负数
    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }
    
    // 分离整数和小数部分
    int_part = (int)value;
    frac_part = (int)((value - int_part) * multiplier + 0.5f); // 四舍五入
    
    // 处理进位情况
    if (frac_part >= multiplier) {
        int_part++;
        frac_part = 0;
    }
    
    // 格式化输出
    if (decimals == 1) {
        sprintf(buffer, "%d.%01d", int_part, frac_part);
    } else if (decimals == 2) {
        sprintf(buffer, "%d.%02d", int_part, frac_part);
    } else if (decimals == 3) {
        sprintf(buffer, "%d.%03d", int_part, frac_part);
    } else {
        sprintf(buffer, "%d", int_part); // 默认只显示整数部分
    }
}

/**
 * @brief 简化的浮点数打印函数 - 带温度单位
 * @param label: 标签字符串
 * @param value: 温度值
 * @retval None
 */
void UART_PrintTemp(const char* label, float value)
{
    char temp_str[12];
    FloatToString(value, 1, temp_str);
    UART_Printf("%s%s C\r\n", label, temp_str);
}

/**
 * @brief 简化的浮点数打印函数 - 带百分号
 * @param label: 标签字符串  
 * @param value: 百分比值
 * @retval None
 */
void UART_PrintPercent(const char* label, float value)
{
    char percent_str[12];
    FloatToString(value, 1, percent_str);
    UART_Printf("%s%s%%\r\n", label, percent_str);
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief 通过UART发送字符串
 * @param str: 要发送的字符串
 * @retval None
 */
void UART_SendString(const char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief 通过UART发送格式化字符串
 * @param format: 格式化字符串
 * @param ...: 可变参数
 * @retval None
 */
void UART_Printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    UART_SendString(buffer);
}

/* USER CODE END 1 */
