#ifndef __BSP_GENERAL_GPIO_H__
#define __BSP_GENERAL_GPIO_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
  LEVEL_UP   = 0,
  LEVEL_DOWN = 1,
}LEVELState_TypeDef;

/* Left Motor 宏定义 --------------------------------------------------------------------*/
#define Left_Motor_Enable_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define Left_Motor_Enable_GPIO_PIN                 GPIO_PIN_6
#define Left_Motor_Enable_GPIO                     GPIOE
#define Left_Motor_Enable_DOWN_LEVEL               0  

#define Left_Motor_Orientation_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define Left_Motor_Orientation_GPIO_PIN                 GPIO_PIN_3
#define Left_Motor_Orientation_GPIO                     GPIOA
#define Left_Motor_Orientation_DOWN_LEVEL               0  

#define Left_Motor_Break_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define Left_Motor_Break_GPIO_PIN                 GPIO_PIN_0
#define Left_Motor_Break_GPIO                     GPIOA
#define Left_Motor_Break_DOWN_LEVEL               0  

#define Left_Motor_Hall_Feedback_RCC_CLK_ENABLE           __HAL_RCC_GPIOB_CLK_ENABLE
#define Left_Motor_Hall_Feedback_GPIO_PIN                 GPIO_PIN_0
#define Left_Motor_Hall_Feedback_GPIO                     GPIOB
#define Left_Motor_Hall_Feedback_DOWN_LEVEL               0  

#define Left_Motor_ALM_RCC_CLK_ENABLE           __HAL_RCC_GPIOH_CLK_ENABLE
#define Left_Motor_ALM_GPIO_PIN                 GPIO_PIN_10
#define Left_Motor_ALM_GPIO                     GPIOH
#define Left_Motor_ALM_DOWN_LEVEL               0 

#define Left_Motor_Speedx1_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define Left_Motor_Speedx1_GPIO_PIN                 GPIO_PIN_4
#define Left_Motor_Speedx1_GPIO                     GPIOA
#define Left_Motor_Speedx1_DOWN_LEVEL               0 

#define Left_Motor_Speedx2_RCC_CLK_ENABLE           __HAL_RCC_GPIOC_CLK_ENABLE
#define Left_Motor_Speedx2_GPIO_PIN                 GPIO_PIN_0
#define Left_Motor_Speedx2_GPIO                     GPIOC
#define Left_Motor_Speedx2_DOWN_LEVEL               0 

#define Left_Motor_Speedx3_RCC_CLK_ENABLE           __HAL_RCC_GPIOH_CLK_ENABLE
#define Left_Motor_Speedx3_GPIO_PIN                 GPIO_PIN_12
#define Left_Motor_Speedx3_GPIO                     GPIOH
#define Left_Motor_Speedx3_DOWN_LEVEL               0 

/* Right Motor 宏定义 --------------------------------------------------------------------*/
#define Right_Motor_Enable_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define Right_Motor_Enable_GPIO_PIN                 GPIO_PIN_5
#define Right_Motor_Enable_GPIO                     GPIOE
#define Right_Motor_Enable_DOWN_LEVEL               0  

#define Right_Motor_Orientation_RCC_CLK_ENABLE           __HAL_RCC_GPIOF_CLK_ENABLE
#define Right_Motor_Orientation_GPIO_PIN                 GPIO_PIN_4
#define Right_Motor_Orientation_GPIO                     GPIOF
#define Right_Motor_Orientation_DOWN_LEVEL               0  

#define Right_Motor_Break_RCC_CLK_ENABLE           __HAL_RCC_GPIOF_CLK_ENABLE
#define Right_Motor_Break_GPIO_PIN                 GPIO_PIN_3
#define Right_Motor_Break_GPIO                     GPIOF
#define Right_Motor_Break_DOWN_LEVEL               0  

#define Right_Motor_Hall_Feedback_RCC_CLK_ENABLE           __HAL_RCC_GPIOB_CLK_ENABLE
#define Right_Motor_Hall_Feedback_GPIO_PIN                 GPIO_PIN_1
#define Right_Motor_Hall_Feedback_GPIO                     GPIOB
#define Right_Motor_Hall_Feedback_DOWN_LEVEL               0  

#define Right_Motor_ALM_RCC_CLK_ENABLE           __HAL_RCC_GPIOF_CLK_ENABLE
#define Right_Motor_ALM_GPIO_PIN                 GPIO_PIN_5
#define Right_Motor_ALM_GPIO                     GPIOF
#define Right_Motor_ALM_DOWN_LEVEL               0 

#define Right_Motor_Speedx1_RCC_CLK_ENABLE           __HAL_RCC_GPIOF_CLK_ENABLE
#define Right_Motor_Speedx1_GPIO_PIN                 GPIO_PIN_9
#define Right_Motor_Speedx1_GPIO                     GPIOF
#define Right_Motor_Speedx1_DOWN_LEVEL               0 

#define Right_Motor_Speedx2_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define Right_Motor_Speedx2_GPIO_PIN                 GPIO_PIN_8
#define Right_Motor_Speedx2_GPIO                     GPIOA
#define Right_Motor_Speedx2_DOWN_LEVEL               0 

#define Right_Motor_Speedx3_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define Right_Motor_Speedx3_GPIO_PIN                 GPIO_PIN_9
#define Right_Motor_Speedx3_GPIO                     GPIOA
#define Right_Motor_Speedx3_DOWN_LEVEL               0 

#define R6093U_RCC_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE()
#define R6093U_GPIO_PIN                 GPIO_PIN_12
#define R6093U_GPIO                     GPIOD

#define Bumper_RCC_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define Bumper_GPIO_PIN                 GPIO_PIN_3
#define Bumper_GPIO                     GPIOB

#define Avoidance_RCC_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define Avoidance_GPIO_PIN                 GPIO_PIN_4
#define Avoidance_GPIO                     GPIOA

void R6093U_GPIO_Init(void);
void Bumper_GPIO_Init(void);
void Avoidance_GPIO_Init(void);
void Left_Motor_GPIO_Init(void);
void Right_Motor_GPIO_Init(void);

#endif  // __BSP_GENERAL_GPIO_H__

