/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TemCal.h"
#include "gpio.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// PID控制器结构体
typedef struct {
    float setpoint;         // 设定值
    float kp, ki, kd;      // PID参数
    float prev_error;      // 上一次误差
    float integral;        // 积分累积值
    float output;          // PID输出
    uint32_t last_time;    // 上次计算时间(ms)
} PID_Controller;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 控制模式选择
#define CONTROL_MODE_RELAY      0       // 继电器控制模式
#define CONTROL_MODE_PWM        1       // PWM控制模式
#define CONTROL_MODE            CONTROL_MODE_RELAY  // 当前控制模式

// PID温度控制相关宏定义
#define TEMP_SET_POINT_1        25.0f   // 温度设定点1 (摄氏度)
#define TEMP_SET_POINT_2        60.0f   // 温度设定点2 (摄氏度)
#define CONTROL_CYCLE_MS        100     // 控制任务周期 (毫秒)
#define ADC_CYCLE_MS            500     // ADC读取任务周期 (毫秒)
#define UART_SEND_CYCLE_MS      2000    // 串口发送温度周期 (毫秒)

// PID参数
#define PID_KP                  2.0f    // 比例系数
#define PID_KI                  0.01f   // 积分系数
#define PID_KD                  0.0f    // 微分系数
#define PID_MAX_OUTPUT          100.0f  // PID输出最大值
#define PID_MIN_OUTPUT          0.0f    // PID输出最小值
#define PID_MAX_INTEGRAL        50.0f   // 积分限幅

// PWM控制参数
#define PWM_FREQUENCY           1000    // PWM频率 (Hz)
#define PWM_PERIOD              1000    // PWM周期 (ms)
#define PWM_MIN_DUTY            0       // 最小占空比 (%)
#define PWM_MAX_DUTY            100     // 最大占空比 (%)
#define RELAY_THRESHOLD         50.0f   // 继电器开关阈值 (%)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern float g_temperature;
extern uint8_t g_control_flag;
extern float g_ADC_output;

// PID模式切换标志位
volatile uint8_t PID_Mode_Flag = 0;  // 0: 模式1(TEMP_SET_POINT_1), 1: 模式2(TEMP_SET_POINT_2)

// PID控制器实例
PID_Controller temp_pid = {
    .setpoint = TEMP_SET_POINT_1,
    .kp = PID_KP,
    .ki = PID_KI, 
    .kd = PID_KD,
    .prev_error = 0.0f,
    .integral = 0.0f,
    .output = 0.0f,
    .last_time = 0
};

// PWM控制变量
static uint32_t pwm_period_start = 0;  // PWM周期开始时间
static float current_duty_cycle = 0.0f; // 当前占空比 (%)
static uint8_t pwm_output_state = 0;   // PWM输出状态
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId tempTaskHandle;
osThreadId controlTaskHandle;
osThreadId uartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void TemperatureTask(void const * argument);
void ControlTask(void const * argument);
void UartSendTask(void const * argument);
void StartDefaultTask(void const * argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/*非任务函数*/
float PID_Calculate(PID_Controller* pid, float current_value);
void PID_Reset(PID_Controller* pid);
void PWM_UpdateOutput(float duty_cycle);
void Relay_UpdateOutput(float pid_output);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 创建温度读取任务 - 普通优先级 */
  osThreadDef(tempTask, TemperatureTask, osPriorityNormal, 0, 256);
  tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);
  
  /* 创建控制任务 - 高优先级 */
  osThreadDef(controlTask, ControlTask, osPriorityHigh, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);
  
  /* 创建串口发送温度任务 - 低优先级 */
  osThreadDef(uartTask, UartSendTask, osPriorityLow, 0, 256);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
  
  /* 检查任务创建是否成功 */
  if (tempTaskHandle == NULL || controlTaskHandle == NULL || uartTaskHandle == NULL) {
    /* 任务创建失败，进入错误处理 */
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    /* 心跳指示 - 每500ms切换一次PB0状态作为系统运行指示 */
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 温度读取任务 - 专门负责ADC读取和温度计算
 * @param argument: 任务参数
 * @retval None
 */
void TemperatureTask(void const * argument)
{
  uint16_t adc_value;
  
  /* 任务初始化 */
  osDelay(1000); // 等待系统稳定
  
  /* 无限循环 */
  for(;;)
  {
    // 读取ADC值
    adc_value = Read_ADC1_Value();
    g_ADC_output = adc_value;
    if (adc_value > 0)
    {
      // 计算温度并更新全局变量
      g_temperature = NTC_CalculateTemperature(adc_value);
    }
    
    // 任务延时
    osDelay(ADC_CYCLE_MS);
  }
}

/**
 * @brief 控制任务 - 高优先级，使用PID控制温度，支持继电器和PWM两种模式
 * @param argument: 任务参数
 * @retval None
 */
void ControlTask(void const * argument)
{
  float pid_output = 0.0f;
  static uint8_t last_mode = 0xFF;  // 初始化为无效值，强制第一次更新
  
  /* 任务初始化 */
  osDelay(1500); // 等待温度读取任务启动
  
  // 初始化PID控制器
  PID_Reset(&temp_pid);
  
  // 初始化PWM控制
  if (CONTROL_MODE == CONTROL_MODE_PWM)
  {
    pwm_period_start = osKernelSysTick();
    current_duty_cycle = 0.0f;
    pwm_output_state = 0;
  }
  
  /* 无限循环 */
  for(;;)
  {
    // 检查PID模式是否发生变化
    if (PID_Mode_Flag != last_mode)
    {
      // 更新PID设定点
      if (PID_Mode_Flag == 0)
      {
        temp_pid.setpoint = TEMP_SET_POINT_1;
        UART_Printf("Mode switched to 1, Target: %.1f C\r\n", TEMP_SET_POINT_1);
      }
      else
      {
        temp_pid.setpoint = TEMP_SET_POINT_2;
        UART_Printf("Mode switched to 2, Target: %.1f C\r\n", TEMP_SET_POINT_2);
      }
      
      // 重置PID控制器状态
      PID_Reset(&temp_pid);
      
      last_mode = PID_Mode_Flag;
    }
    
    // 计算PID输出
    pid_output = PID_Calculate(&temp_pid, g_temperature);
    
    // 根据控制模式选择输出方式
    if (CONTROL_MODE == CONTROL_MODE_PWM)
    {
      // PWM控制模式
      PWM_UpdateOutput(pid_output);
    }
    else
    {
      // 继电器控制模式
      Relay_UpdateOutput(pid_output);
    }
    
    // 控制任务延时
    osDelay(CONTROL_CYCLE_MS);
  }
}

/**
 * @brief 串口发送温度任务 - 低优先级，定期向上位机发送当前温度
 * @param argument: 任务参数
 * @retval None
 */
void UartSendTask(void const * argument)
{
  /* 任务初始化 */
  osDelay(2000); // 等待系统稳定和其他任务启动
  
  /* 发送启动信息 */
  UART_SendString("Temperature Control System Started\r\n");
  UART_SendString("PA3 button: Switch PID target temperature\r\n");
  UART_Printf("Mode 1 Target: %.1f C, Mode 2 Target: %.1f C\r\n", TEMP_SET_POINT_1, TEMP_SET_POINT_2);
  UART_SendString("==================================\r\n");
  
  /* 无限循环 */
  for(;;)
  {
    // 发送当前温度和PID信息到上位机
    UART_Printf("Temp: %.1f C, Target: %.1f C, Mode: %d\r\n", 
                g_temperature, temp_pid.setpoint, PID_Mode_Flag + 1);
    
    // 任务延时
    osDelay(UART_SEND_CYCLE_MS);
  }
}










/**
 * @brief PID控制器计算函数
 * @param pid: PID控制器指针
 * @param current_value: 当前测量值
 * @return float: PID输出值
 */
float PID_Calculate(PID_Controller* pid, float current_value)
{
  uint32_t current_time = osKernelSysTick(); // 获取当前时间
  float dt = (current_time - pid->last_time) * 0.001f; // 转换为秒
  
  // 第一次调用时跳过
  if (pid->last_time == 0) {
    pid->last_time = current_time;
    pid->prev_error = pid->setpoint - current_value;
    return 0.0f;
  }
  
  // 计算误差
  float error = pid->setpoint - current_value;
  
  // 比例项
  float proportional = pid->kp * error;
  
  // 积分项（带限幅）
  pid->integral += error * dt;
  if (pid->integral > PID_MAX_INTEGRAL) pid->integral = PID_MAX_INTEGRAL;
  if (pid->integral < -PID_MAX_INTEGRAL) pid->integral = -PID_MAX_INTEGRAL;
  float integral = pid->ki * pid->integral;
  
  // 微分项
  float derivative = pid->kd * (error - pid->prev_error) / dt;
  
  // PID输出
  pid->output = proportional + integral + derivative;
  
  // 输出限幅
  if (pid->output > PID_MAX_OUTPUT) pid->output = PID_MAX_OUTPUT;
  if (pid->output < PID_MIN_OUTPUT) pid->output = PID_MIN_OUTPUT;
  
  // 保存当前值用于下次计算
  pid->prev_error = error;
  pid->last_time = current_time;
  
  return pid->output;
}

/**
 * @brief 重置PID控制器
 * @param pid: PID控制器指针
 * @retval None
 */
void PID_Reset(PID_Controller* pid)
{
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
  pid->last_time = 0;
}

/**
 * @brief PWM控制输出函数
 * @param duty_cycle: 占空比 (0-100%)
 * @retval None
 */
void PWM_UpdateOutput(float duty_cycle)
{
  uint32_t current_time = osKernelSysTick();
  uint32_t elapsed_time = current_time - pwm_period_start;
  
  // 限制占空比范围
  if (duty_cycle > PWM_MAX_DUTY) duty_cycle = PWM_MAX_DUTY;
  if (duty_cycle < PWM_MIN_DUTY) duty_cycle = PWM_MIN_DUTY;
  
  current_duty_cycle = duty_cycle;
  
  // 计算PWM周期内的开启时间 (ms)
  uint32_t on_time = (uint32_t)((duty_cycle / 100.0f) * PWM_PERIOD);
  
  // PWM周期控制
  if (elapsed_time >= PWM_PERIOD)
  {
    // 重新开始PWM周期
    pwm_period_start = current_time;
    elapsed_time = 0;
    
    // 如果占空比 > 0，开启输出
    if (duty_cycle > 0.0f)
    {
      pwm_output_state = 1;
      g_control_flag = 1;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else
    {
      pwm_output_state = 0;
      g_control_flag = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
  }
  else if (elapsed_time >= on_time && pwm_output_state == 1)
  {
    // 到达关闭时间，关闭输出
    pwm_output_state = 0;
    g_control_flag = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  }
}

/**
 * @brief 继电器控制输出函数
 * @param pid_output: PID输出值 (0-100%)
 * @retval None
 */
void Relay_UpdateOutput(float pid_output)
{
  // 简单的阈值控制
  if (pid_output > RELAY_THRESHOLD)
  {
    g_control_flag = 1;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // 开启加热
  }
  else
  {
    g_control_flag = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // 关闭加热
  }
}

/* USER CODE END Application */

