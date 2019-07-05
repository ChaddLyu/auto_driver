/* 包含头文件 ----------------------------------------------------------------*/
//#include "stm32f4xx_hal.h"
//#include "string.h"
//#include "usart/bsp_usartx.h"

#include "control.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // 使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // 设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // 打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // 打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{

  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  
	/* 板载LED初始化 */
  LED_GPIO_Init();
  
  /* 板载蜂鸣器初始化 */
  BEEP_GPIO_Init();
	
  R6093U_GPIO_Init();
	Bumper_GPIO_Init();
	Avoidance_GPIO_Init();
	
	MX_USARTx_Init();
	MX_DEBUG_USART_Init();
	MX_USARTx_5_Init();
	MX_USARTx_2_Init();
  /* 调用格式化输出函数打印输出数据 */
//  printf("Gyroscope data test!!!\n");
  HAL_UART_Receive_IT(&husart_debug,&aRxCmd,1);
	HAL_UART_Receive_IT(&husartx_5,&aRxCmd_5,1);	
	HAL_UART_Receive_IT(&husartx_2,&aRxCmd_2,1);
	
	uint32_t angle_output_tick = HAL_GetTick();
  /* 无限循环 */
  while (1)
  {
//			if(HAL_GetTick() - angle_output_tick >= 1000)
//			{
//			  angle_output_tick = HAL_GetTick();
////				printf("====== Angle : %3.3f ======\n",angle);
////				printf("===>Pose -- N : %3.3f | E : %3.3f <===\n",rtk_pose_n, rtk_pose_e);
////				if(obj_detected == 1)
////				{
////					printf("==> Object Detected : YES! <==\n");
////				}
////				else
////				{
////					printf("==> Object Detected : NO! <==\n");
////				}
//				printf("==> Object Distance Detected : %3.3f! <==\n",jsn_dist);
//			}
			if(bumper_event == 1)
			{
				bumper_event = 0;
				while(HAL_GPIO_ReadPin(Bumper_GPIO,Bumper_GPIO_PIN) == RESET)
				{
					BEEP_ON;
					printf("==> Bumper Detected! <==\n");
				}
				BEEP_OFF;
				printf("*** Bumper Released! ***\n");
			}
			if(avoidance_event == 1)
			{
				avoidance_event = 0;
				while(HAL_GPIO_ReadPin(Avoidance_GPIO,Avoidance_GPIO_PIN) == RESET)
				{
					BEEP_ON;
					printf("==> Ultrasonic Detected! <==\n");
				}
				BEEP_OFF;
				printf("*** Ultrasonic Released! ***\n");
			}
  }
}

