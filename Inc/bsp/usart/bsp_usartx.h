#ifndef __BSP_USARTX_H__
#define __BSP_USARTX_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 数据格式化转换 */
typedef union {
  char Ch[4];
  float Float;
  int32_t Int;
}Format_UnionTypedef;

typedef struct {
  __IO uint8_t  Code ;  	
  __IO Format_UnionTypedef data[3];//数据帧有3个参数
}MSG_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/
//#define _RS485_
//#define _RS232_
#define _USART_

#ifdef _USART_

  #define USARTx                                 USART1
  #define USARTx_BAUDRATE                        115200
  #define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART1_CLK_ENABLE()
  #define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART1_CLK_DISABLE()

  #define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
  #define USARTx_Tx_GPIO_PIN                     GPIO_PIN_6
  #define USARTx_Tx_GPIO                         GPIOB
  #define USARTx_Rx_GPIO_PIN                     GPIO_PIN_7   
  #define USARTx_Rx_GPIO                         GPIOB

  #define USARTx_AFx                             GPIO_AF7_USART1

  #define USARTx_IRQHANDLER                      USART1_IRQHandler
  #define USARTx_IRQn                            USART1_IRQn
#else 
  #define USARTx                                 USART3
  #define USARTx_BAUDRATE                        115200
  #define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
  #define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

  #define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
  #define USARTx_Tx_GPIO_PIN                     GPIO_PIN_10
  #define USARTx_Tx_GPIO                         GPIOB
  #define USARTx_Rx_GPIO_PIN                     GPIO_PIN_11
  #define USARTx_Rx_GPIO                         GPIOB

  #define USARTx_AFx                             GPIO_AF7_USART3
  
  #define USARTx_IRQHANDLER                      USART3_IRQHandler
  #define USARTx_IRQn                            USART3_IRQn
  
  #ifdef _RS485_  
    #define RS485_REDE_GPIO_ClK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
    #define RS485_REDE_PORT                     GPIOC
    #define RS485_REDE_PIN                      GPIO_PIN_12
    #define RS485_RX_MODE()                     HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
    #define RS485_TX_MODE()                     HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)
  #endif
  
#endif

#define DEBUG_USARTx                                 UART4
#define DEBUG_USARTx_BAUDRATE                        38400
#define DEBUG_USART_RCC_CLK_ENABLE()                 __HAL_RCC_UART4_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()                __HAL_RCC_UART4_CLK_DISABLE()

#define DEBUG_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define DEBUG_USARTx_Tx_GPIO_PIN                     GPIO_PIN_10
#define DEBUG_USARTx_Tx_GPIO                         GPIOC
#define DEBUG_USARTx_Rx_GPIO_PIN                     GPIO_PIN_11
#define DEBUG_USARTx_Rx_GPIO                         GPIOC

#define DEBUG_USARTx_AFx                             GPIO_AF8_UART4

#define DEBUG_USARTx_IRQHANDLER                      UART4_IRQHandler
#define DEBUG_USARTx_IRQn                            UART4_IRQn

#define USARTx_5                                 UART5
#define USARTx_5_BAUDRATE                        115200
#define USART5_RCC_CLK_ENABLE()                 __HAL_RCC_UART5_CLK_ENABLE()
#define USART5_RCC_CLK_DISABLE()                __HAL_RCC_UART5_CLK_DISABLE()

#define USARTx_5_GPIO_ClK_ENABLE()               {__HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();}
#define USARTx_5_Tx_GPIO_PIN                     GPIO_PIN_12
#define USARTx_5_Tx_GPIO                         GPIOC
#define USARTx_5_Rx_GPIO_PIN                     GPIO_PIN_2
#define USARTx_5_Rx_GPIO                         GPIOD

#define USARTx_5_AFx                             GPIO_AF8_UART5

#define USARTx_5_IRQHANDLER                      UART5_IRQHandler
#define USARTx_5_IRQn                            UART5_IRQn

#define USARTx_2                                 USART2
#define USARTx_2_BAUDRATE                        9600
#define USART_2_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define USART_2_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define USARTx_2_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_2_Tx_GPIO_PIN                     GPIO_PIN_5
#define USARTx_2_Tx_GPIO                         GPIOD
#define USARTx_2_Rx_GPIO_PIN                     GPIO_PIN_6   
#define USARTx_2_Rx_GPIO                         GPIOD

#define USARTx_2_AFx                             GPIO_AF7_USART2

#define USARTx_2_IRQHANDLER                      USART2_IRQHandler
#define USARTx_2_IRQn                            USART2_IRQn

// 协议相关定义
#define FRAME_LENTH               16    // 指令长度
#define FRAME_START               0xAA  // 协议帧开始
#define FRAME_END                 '/'   // 协议帧结束
#define FRAME_CHECK_BEGIN          1    // 校验码开始的位置 RxBuf[1]
#define FRAME_CHECKSUM            14    // 校验码的位置   RxBuf[14]
#define FRAME_CHECK_NUM           13    // 需要校验的字节数
#define FILL_VALUE                0x55  // 填充值
#define CODE_SETPID               0x07  // 设置PID参数
#define CODE_SETTGT               0x08  // 设置目标值
#define CODE_RESET                0x09   // 复位重启
#define CODE_STARTMOTOR           0x0A   // 启动电机

/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx;
extern UART_HandleTypeDef husart_debug;
extern UART_HandleTypeDef husartx_5;
extern UART_HandleTypeDef husartx_2;

void MX_USARTx_2_Init(void);
void MX_USARTx_Init(void);
void MX_DEBUG_USART_Init(void);
void MX_USARTx_5_Init(void);
/* 移植的时候需要修改以下指针所指向的变量 */ 
extern  MSG_TypeDef Msg;
extern __IO uint8_t RxBuf[FRAME_LENTH] ; // 接收缓存区
extern void (*ptr_Fun_)(void) ;//函数指针
/* 函数声明 ------------------------------------------------------------------*/

void Transmit_FB(__IO int32_t *Feedback);

#endif  /* __BSP_USARTX_H__ */
