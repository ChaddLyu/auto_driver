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
#include "General_GPIO/bsp_general_gpio.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ���ذ���IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_key.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void Left_Motor_GPIO_Init(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
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
   /* ����IOӲ����ʼ���ṹ����� */
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
   /* ����IOӲ����ʼ���ṹ����� */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* ʹ��(����)GYRO���Ŷ�ӦIO�˿�ʱ�� */  
  R6093U_RCC_CLK_ENABLE();
  /* GYRO���������ѹ */
  HAL_GPIO_WritePin(R6093U_GPIO, R6093U_GPIO_PIN, GPIO_PIN_SET);

  /* �趨 GYRO ��Ӧ����IO��� */
  GPIO_InitStruct.Pin = R6093U_GPIO_PIN;
  /* �趨GYRO��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  /* �趨GYRO��Ӧ����IO�����ٶ� */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* ��ʼ��GYRO��Ӧ����IO */
  HAL_GPIO_Init(R6093U_GPIO, &GPIO_InitStruct);
}

void Bumper_GPIO_Init(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
  GPIO_InitTypeDef GPIO_InitStruct;
  Bumper_RCC_CLK_ENABLE();

  /* �趨 GYRO ��Ӧ����IO��� */
  GPIO_InitStruct.Pin = Bumper_GPIO_PIN;
  /* �趨GYRO��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  /* ��ʼ��GYRO��Ӧ����IO */
  HAL_GPIO_Init(Bumper_GPIO, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void Avoidance_GPIO_Init(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
  GPIO_InitTypeDef GPIO_InitStruct;
  Avoidance_RCC_CLK_ENABLE();

  /* �趨 GYRO ��Ӧ����IO��� */
  GPIO_InitStruct.Pin = Avoidance_GPIO_PIN;
  /* �趨GYRO��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  /* ��ʼ��GYRO��Ӧ����IO */
  HAL_GPIO_Init(Avoidance_GPIO, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}