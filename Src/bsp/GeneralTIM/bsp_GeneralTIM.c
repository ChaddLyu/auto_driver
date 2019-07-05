/**
  ******************************************************************************
  * �ļ�����: bsp_general_gpio.c 
  * ��    ��: HuYixuan
  * ��    ��: V1.0
  * ��д����: 2018-11-24
  * ��    ��: General_GPIO driver
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "GeneralTIM/bsp_GeneralTIM.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx;
uint32_t Left_Hall_count = 0;
uint32_t Right_Hall_count = 0;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ��ʱ��Ӳ����ʼ������
  * �������: htim����ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����GENERAL_TIMx_Init��������
  */
void HAL_GeneralTIM_MspPostInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ������ʱ������ʱ��ʹ�� */
  GENERAL_TIM_RCC_CLK_ENABLE();
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
  GENERAL_TIM_GPIO_RCC_CLK_ENABLE();
  
  /* ��ʱ��ͨ��1��������IO��ʼ�� */
  GPIO_InitStruct.Pin = GENERAL_TIM_CH1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GENERAL_TIM_GPIO_AF;
  HAL_GPIO_Init(GENERAL_TIM_CH1_PORT, &GPIO_InitStruct);

  /* ��ʱ��ͨ��2��������IO��ʼ�� */
  GPIO_InitStruct.Pin = GENERAL_TIM_CH2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GENERAL_TIM_GPIO_AF;
  HAL_GPIO_Init(GENERAL_TIM_CH2_PORT, &GPIO_InitStruct);

  /* ��ʱ��ͨ��3��������IO��ʼ�� */
  GPIO_InitStruct.Pin = GENERAL_TIM_CH3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GENERAL_TIM_GPIO_AF;
  HAL_GPIO_Init(GENERAL_TIM_CH3_PORT, &GPIO_InitStruct);
  
  /* ��ʱ��ͨ��4��������IO��ʼ�� */
  GPIO_InitStruct.Pin = GENERAL_TIM_CH4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GENERAL_TIM_GPIO_AF;
  HAL_GPIO_Init(GENERAL_TIM_CH4_PORT, &GPIO_InitStruct);    
}

/**
  * ��������: ͨ�ö�ʱ����ʼ��������ͨ��PWM���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void GENERAL_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  
  HAL_GeneralTIM_MspPostInit();
  
  htimx.Instance = GENERAL_TIMx;
  htimx.Init.Prescaler = GENERAL_TIM_PRESCALER;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = GENERAL_TIM_PERIOD;
  htimx.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htimx);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig);
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = GENERAL_TIM_CH1_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.Pulse = GENERAL_TIM_CH2_PULSE;
  HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_2);

  sConfigOC.Pulse = GENERAL_TIM_CH3_PULSE;
  HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_3);

  sConfigOC.Pulse = GENERAL_TIM_CH4_PULSE;
  HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_4);
  
}

