#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define GENERAL_TIMx                        TIM2
#define GENERAL_TIM_GPIO_AF                 GPIO_AF1_TIM2
#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM2_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM2_CLK_DISABLE()
#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   {__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();}
#define GENERAL_TIM_CH1_PORT                GPIOA
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_15
#define GENERAL_TIM_CH2_PORT                GPIOB
#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_3
#define GENERAL_TIM_CH3_PORT                GPIOB
#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_10
#define GENERAL_TIM_CH4_PORT                GPIOB
#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_11


//#define GENERAL_TIMx                        TIM3
//#define GENERAL_TIM_GPIO_AF                 GPIO_AF2_TIM3
//#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM3_CLK_ENABLE()
//#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM3_CLK_DISABLE()
//#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()
//#define GENERAL_TIM_CH1_PORT                GPIOC
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_6
//#define GENERAL_TIM_CH2_PORT                GPIOC
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_7
//#define GENERAL_TIM_CH3_PORT                GPIOC
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_8
//#define GENERAL_TIM_CH4_PORT                GPIOC
//#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_9


//#define GENERAL_TIMx                        TIM4
//#define GENERAL_TIM_GPIO_AF                 GPIO_AF2_TIM4
//#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM4_CLK_ENABLE()
//#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM4_CLK_DISABLE()
//#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
//#define GENERAL_TIM_CH1_PORT                GPIOD
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_12
//#define GENERAL_TIM_CH2_PORT                GPIOD
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_13
//#define GENERAL_TIM_CH3_PORT                GPIOD
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_14
//#define GENERAL_TIM_CH4_PORT                GPIOD
//#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_15


//#define GENERAL_TIMx                        TIM5
//#define GENERAL_TIM_GPIO_AF                 GPIO_AF2_TIM5
//#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM5_CLK_ENABLE()
//#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM5_CLK_DISABLE()
//#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   {__HAL_RCC_GPIOH_CLK_ENABLE();__HAL_RCC_GPIOI_CLK_ENABLE();}
//#define GENERAL_TIM_CH1_PORT                GPIOH
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_10
//#define GENERAL_TIM_CH2_PORT                GPIOH
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_11
//#define GENERAL_TIM_CH3_PORT                GPIOH
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_12
//#define GENERAL_TIM_CH4_PORT                GPIOI
//#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_0

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（GENERAL_TIMx_PRESCALER+1）
#define GENERAL_TIM_PRESCALER            24  // 实际时钟频率为：1MHz

// 定义定时器周期，当定时器开始计数到GENERAL_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD               999  // 定时器产生中断频率为： 84/25MHz/1000=3.3KHz，即0.3ms定时周期

#define GENERAL_TIM_CH1_PULSE            0   // Left Motor control PWM
#define GENERAL_TIM_CH2_PULSE            0   // Right Motor control PWM
#define GENERAL_TIM_CH3_PULSE            0   // 
#define GENERAL_TIM_CH4_PULSE            0   // 


/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx;
extern uint32_t Left_Hall_count;
extern uint32_t Right_Hall_count;

/* 函数声明 ------------------------------------------------------------------*/
void GENERAL_TIMx_Init(void);

#endif	/* __GENERAL_TIM_H__ */


