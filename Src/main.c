/* ����ͷ�ļ� ----------------------------------------------------------------*/
//#include "stm32f4xx_hal.h"
//#include "string.h"
//#include "usart/bsp_usartx.h"

#include "control.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // ���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // ��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{

  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  
	/* ����LED��ʼ�� */
  LED_GPIO_Init();
  
  /* ���ط�������ʼ�� */
  BEEP_GPIO_Init();
	
  R6093U_GPIO_Init();
	Bumper_GPIO_Init();
	Avoidance_GPIO_Init();
	
	MX_USARTx_Init();
	MX_DEBUG_USART_Init();
	MX_USARTx_5_Init();
	MX_USARTx_2_Init();
  /* ���ø�ʽ�����������ӡ������� */
//  printf("Gyroscope data test!!!\n");
  HAL_UART_Receive_IT(&husart_debug,&aRxCmd,1);
	HAL_UART_Receive_IT(&husartx_5,&aRxCmd_5,1);	
	HAL_UART_Receive_IT(&husartx_2,&aRxCmd_2,1);
	
	uint32_t angle_output_tick = HAL_GetTick();
  /* ����ѭ�� */
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

