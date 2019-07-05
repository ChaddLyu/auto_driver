/**
  ******************************************************************************
  * 文件名程: bsp_general_gpio.c 
  * 作    者: HuYixuan
  * 版    本: V1.0
  * 编写日期: 2018-11-24
  * 功    能: General_GPIO driver
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "General_GPIO/bsp_general_gpio.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载按键IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_key.h
  *           文件相关宏定义就可以方便修改引脚。
  */
void Left_Motor_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
  Left_Motor_Enable_RCC_CLK_ENABLE();
  Left_Motor_Orientation_RCC_CLK_ENABLE();
	Left_Motor_Break_RCC_CLK_ENABLE();
	Left_Motor_Hall_Feedback_RCC_CLK_ENABLE();
	Left_Motor_ALM_RCC_CLK_ENABLE();
	Left_Motor_Speedx1_RCC_CLK_ENABLE();
	Left_Motor_Speedx2_RCC_CLK_ENABLE();
	Left_Motor_Speedx3_RCC_CLK_ENABLE();
  
	HAL_GPIO_WritePin(Left_Motor_Enable_GPIO, Left_Motor_Enable_GPIO_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = Left_Motor_Enable_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Left_Motor_Enable_GPIO, &GPIO_InitStruct);  
  
	HAL_GPIO_WritePin(Left_Motor_Orientation_GPIO, Left_Motor_Orientation_GPIO_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = Left_Motor_Orientation_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Left_Motor_Orientation_GPIO, &GPIO_InitStruct);
  
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = Left_Motor_Break_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Left_Motor_Break_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Left_Motor_Hall_Feedback_GPIO_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(Left_Motor_Hall_Feedback_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Left_Motor_ALM_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(Left_Motor_ALM_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(Left_Motor_Speedx1_GPIO, Left_Motor_Speedx1_GPIO_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = Left_Motor_Speedx1_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Left_Motor_Speedx1_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(Left_Motor_Speedx2_GPIO, Left_Motor_Speedx2_GPIO_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = Left_Motor_Speedx2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Left_Motor_Speedx2_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(Left_Motor_Speedx3_GPIO, Left_Motor_Speedx3_GPIO_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = Left_Motor_Speedx3_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Left_Motor_Speedx3_GPIO, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}


void Right_Motor_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
  Right_Motor_Enable_RCC_CLK_ENABLE();
  Right_Motor_Orientation_RCC_CLK_ENABLE();
	Right_Motor_Break_RCC_CLK_ENABLE();
	Right_Motor_Hall_Feedback_RCC_CLK_ENABLE();
	Right_Motor_ALM_RCC_CLK_ENABLE();
	Right_Motor_Speedx1_RCC_CLK_ENABLE();
	Right_Motor_Speedx2_RCC_CLK_ENABLE();
	Right_Motor_Speedx3_RCC_CLK_ENABLE();
  
	HAL_GPIO_WritePin(Right_Motor_Enable_GPIO, Right_Motor_Enable_GPIO_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = Right_Motor_Enable_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Right_Motor_Enable_GPIO, &GPIO_InitStruct);  
  
	HAL_GPIO_WritePin(Right_Motor_Orientation_GPIO, Right_Motor_Orientation_GPIO_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = Right_Motor_Orientation_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Right_Motor_Orientation_GPIO, &GPIO_InitStruct);
  
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = Right_Motor_Break_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Right_Motor_Break_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Right_Motor_Hall_Feedback_GPIO_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(Right_Motor_Hall_Feedback_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Right_Motor_ALM_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(Right_Motor_ALM_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(Right_Motor_Speedx1_GPIO, Right_Motor_Speedx1_GPIO_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = Right_Motor_Speedx1_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Right_Motor_Speedx1_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(Right_Motor_Speedx2_GPIO, Right_Motor_Speedx2_GPIO_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = Right_Motor_Speedx2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Right_Motor_Speedx2_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(Right_Motor_Speedx3_GPIO, Right_Motor_Speedx3_GPIO_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = Right_Motor_Speedx3_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Right_Motor_Speedx3_GPIO, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void R6093U_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* 使能(开启)GYRO引脚对应IO端口时钟 */  
  R6093U_RCC_CLK_ENABLE();
  /* GYRO引脚输出电压 */
  HAL_GPIO_WritePin(R6093U_GPIO, R6093U_GPIO_PIN, GPIO_PIN_SET);

  /* 设定 GYRO 对应引脚IO编号 */
  GPIO_InitStruct.Pin = R6093U_GPIO_PIN;
  /* 设定GYRO对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  /* 设定GYRO对应引脚IO操作速度 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* 初始化GYRO对应引脚IO */
  HAL_GPIO_Init(R6093U_GPIO, &GPIO_InitStruct);
}

void Bumper_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
  Bumper_RCC_CLK_ENABLE();

  /* 设定 GYRO 对应引脚IO编号 */
  GPIO_InitStruct.Pin = Bumper_GPIO_PIN;
  /* 设定GYRO对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  /* 初始化GYRO对应引脚IO */
  HAL_GPIO_Init(Bumper_GPIO, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void Avoidance_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
  Avoidance_RCC_CLK_ENABLE();

  /* 设定 GYRO 对应引脚IO编号 */
  GPIO_InitStruct.Pin = Avoidance_GPIO_PIN;
  /* 设定GYRO对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  /* 初始化GYRO对应引脚IO */
  HAL_GPIO_Init(Avoidance_GPIO, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}