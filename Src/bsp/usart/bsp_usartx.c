/**
  ******************************************************************************
  * �ļ�����: bsp_usartx.c 
  * Software Closed Source
  *
  * Copyright (c) 2018, iRobSen, Inc.
  * All rights reserved.
  *
  * ��    ��: ���ش��ڵײ���������
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "usart/bsp_usartx.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef husartx;
UART_HandleTypeDef husart_debug;
UART_HandleTypeDef husartx_5;
UART_HandleTypeDef husartx_2;

__IO uint8_t RxBuf[FRAME_LENTH] ; // ���ջ�����
__IO uint8_t TxBuf[FRAME_LENTH] ; // ���ͻ�����
/* ��չ���� ------------------------------------------------------------------*/
MSG_TypeDef Msg;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  if(huart->Instance==DEBUG_USARTx)
  {

		DEBUG_USARTx_GPIO_ClK_ENABLE();
		
    GPIO_InitStruct.Pin = DEBUG_USARTx_Tx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DEBUG_USARTx_AFx;
    HAL_GPIO_Init(DEBUG_USARTx_Tx_GPIO, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = DEBUG_USARTx_Rx_GPIO_PIN;  
    HAL_GPIO_Init(DEBUG_USARTx_Rx_GPIO, &GPIO_InitStruct); 

  } 
	
  if(huart == &husartx)
  {
    /* ʹ�ܴ��ڹ�������GPIOʱ�� */
    USARTx_GPIO_ClK_ENABLE();
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = USARTx_Tx_GPIO_PIN|USARTx_Rx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_AFx;
    HAL_GPIO_Init(USARTx_Tx_GPIO, &GPIO_InitStruct);
#ifdef _RS485_
    /* 485ʹ������ */
    RS485_REDE_GPIO_ClK_ENABLE();
    
    GPIO_InitStruct.Pin = RS485_REDE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(RS485_REDE_PORT, &GPIO_InitStruct);
#endif
  }
	
	if(huart->Instance==USARTx_5)
  {
    /* ʹ�ܴ��ڹ�������GPIOʱ�� */
    USARTx_5_GPIO_ClK_ENABLE();
  
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = USARTx_5_Tx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_5_AFx;
    HAL_GPIO_Init(USARTx_5_Tx_GPIO, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = USARTx_5_Rx_GPIO_PIN;
    HAL_GPIO_Init(USARTx_5_Rx_GPIO, &GPIO_InitStruct);
  }
	
	if(huart->Instance==USARTx_2)
  {
    /* ʹ�ܴ��ڹ�������GPIOʱ�� */
    USARTx_2_GPIO_ClK_ENABLE();
  
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = USARTx_2_Tx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_2_AFx;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = USARTx_2_Rx_GPIO_PIN;
    HAL_GPIO_Init(USARTx_2_Rx_GPIO, &GPIO_InitStruct);
  }
}

/**
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	
  if(huart->Instance==DEBUG_USARTx)
  {
    DEBUG_USART_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(DEBUG_USARTx_Tx_GPIO, DEBUG_USARTx_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(DEBUG_USARTx_Rx_GPIO, DEBUG_USARTx_Rx_GPIO_PIN);

    HAL_NVIC_DisableIRQ(DEBUG_USARTx_IRQn);
  }
	
  if(huart == &husartx)
  {
    /* ��������ʱ�ӽ��� */
    USART_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(USARTx_Tx_GPIO, USARTx_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(USARTx_Rx_GPIO, USARTx_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(USARTx_IRQn);
  }
	
	if(huart->Instance==USARTx_5)
  {
    /* ��������ʱ�ӽ��� */
    USART5_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(USARTx_5_Tx_GPIO, USARTx_5_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(USARTx_5_Rx_GPIO, USARTx_5_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(USARTx_5_IRQn);
  }
	
	if(huart->Instance==USARTx_2)
  {
    /* ��������ʱ�ӽ��� */
    USART_2_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(USARTx_2_Tx_GPIO, USARTx_2_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(USARTx_2_Rx_GPIO, USARTx_2_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(USARTx_2_IRQn);
  }
}

/**
  * ��������: NVIC����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void MX_NVIC_USARTx_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

static void MX_NVIC_USART4_Init(void)
{
  // USART4_IRQn interrupt configuration 
  HAL_NVIC_SetPriority(DEBUG_USARTx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DEBUG_USARTx_IRQn);
	
//  HAL_NVIC_SetPriority(USARTx_DMAx_CHANNELn_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(USARTx_DMAx_CHANNELn_IRQn);  
}

static void MX_NVIC_USARTx_5_Init(void)
{
  /* USART5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USARTx_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_5_IRQn);
}

static void MX_NVIC_USARTx_2_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USARTx_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_2_IRQn);
}

void MX_DEBUG_USART_Init(void)
{

	/* ????????GPIO?? */
  DEBUG_USART_RCC_CLK_ENABLE(); 
	//DEBUG_USARTx_GPIO_ClK_ENABLE();
  husart_debug.Instance = DEBUG_USARTx;
  husart_debug.Init.BaudRate = DEBUG_USARTx_BAUDRATE;
  husart_debug.Init.WordLength = UART_WORDLENGTH_8B;
  husart_debug.Init.StopBits = UART_STOPBITS_1;
  husart_debug.Init.Parity = UART_PARITY_NONE;
  husart_debug.Init.Mode = UART_MODE_TX_RX;
  husart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husart_debug);

  MX_NVIC_USART4_Init();
}


/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_USARTx_Init(void)
{
  /* ��������ʱ��ʹ�� */
  USART_RCC_CLK_ENABLE();
  
  husartx.Instance = USARTx;
  husartx.Init.BaudRate = USARTx_BAUDRATE;
  husartx.Init.WordLength = UART_WORDLENGTH_8B;
  husartx.Init.StopBits = UART_STOPBITS_1;
  husartx.Init.Parity = UART_PARITY_NONE;
  husartx.Init.Mode = UART_MODE_TX_RX;
  husartx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx);
  
  /* ���ô����жϲ�ʹ�ܣ���Ҫ����HAL_UART_Init������ִ���޸Ĳ���Ч */
//  HAL_UART_Receive_IT(&husartx,(uint8_t *)&RxBuf,FRAME_LENTH); // ����ʹ�ܽ����ж�
  MX_NVIC_USARTx_Init();
}

void MX_USARTx_5_Init(void)
{
  /* ��������ʱ��ʹ�� */
  USART5_RCC_CLK_ENABLE();
  
  husartx_5.Instance = USARTx_5;
  husartx_5.Init.BaudRate = USARTx_5_BAUDRATE;
  husartx_5.Init.WordLength = UART_WORDLENGTH_8B;
  husartx_5.Init.StopBits = UART_STOPBITS_1;
  husartx_5.Init.Parity = UART_PARITY_NONE;
  husartx_5.Init.Mode = UART_MODE_TX_RX;
  husartx_5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx_5.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx_5);
  
  /* ���ô����жϲ�ʹ�ܣ���Ҫ����HAL_UART_Init������ִ���޸Ĳ���Ч */
  MX_NVIC_USARTx_5_Init();
}

void MX_USARTx_2_Init(void)
{
  /* ��������ʱ��ʹ�� */
  USART_2_RCC_CLK_ENABLE();

  husartx_2.Instance = USARTx_2;
  husartx_2.Init.BaudRate = USARTx_2_BAUDRATE;
  husartx_2.Init.WordLength = UART_WORDLENGTH_8B;
  husartx_2.Init.StopBits = UART_STOPBITS_1;
  husartx_2.Init.Parity = UART_PARITY_NONE;
  husartx_2.Init.Mode = UART_MODE_TX_RX;
  husartx_2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx_2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx_2);
  
  /* ���ô����жϲ�ʹ�ܣ���Ҫ����HAL_UART_Init������ִ���޸Ĳ���Ч */
  MX_NVIC_USARTx_2_Init();
}

/**
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
#ifdef _RS485_
  RS485_TX_MODE();
#endif 
  HAL_UART_Transmit(&husartx, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
#ifdef _RS485_
  RS485_RX_MODE();
#endif 
  while(HAL_UART_Receive(&husartx,&ch, 1, 0xffff)!=HAL_OK);
  return ch;
}

/**
  * ��������: ��������
  * �������: Ptr:������У��͵�������ʼ��ַ , Num:��������ֽ���
  * �� �� ֵ: ����õ���У���
  * ˵    ��: ��������
  */
uint8_t CheckSum(uint8_t *Ptr,uint8_t Num )
{
  uint8_t Sum = 0;
  while(Num--)
  {
    Sum += *Ptr;
    Ptr++;
  }
  return Sum;
}

/**
  * ��������: ���ͷ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ������ֵ���͵�����
  */
void Transmit_FB( __IO int32_t *Feedback)
{
  uint8_t i = 0;
  for(i=0;i<FRAME_LENTH;i++)
  {
    TxBuf[i] = FILL_VALUE;  // ������� 0x55
  }
  
  Msg.data[0].Int = *Feedback;//����ֵ �ٶ�
  
  TxBuf[0] = FRAME_START;   // ֡ͷ
  TxBuf[1] = 0x80|CODE_SETTGT; // ָ����
  TxBuf[2] = Msg.data[0].Ch[0];
  TxBuf[3] = Msg.data[0].Ch[1];
  TxBuf[4] = Msg.data[0].Ch[2];
  TxBuf[5] = Msg.data[0].Ch[3];
  
  TxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&TxBuf[FRAME_CHECK_BEGIN],FRAME_CHECK_NUM);  // ����У���
  TxBuf[FRAME_LENTH-1] = FRAME_END;   // ����֡β
#ifdef _RS485_
  RS485_TX_MODE();
#endif 
  HAL_UART_Transmit_IT(&husartx,(uint8_t *)&TxBuf,FRAME_LENTH); // ��������֡
}
