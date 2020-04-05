/**
 *  @file       bsp_uart_config.h
 *
 *  @brief      
 *
 *  @details
 *
 *  @author     Stulov Tikhon
 *
 *  @date       2020/03/14
 *
 *  @warning
 *
 *  @todo
 *
 */

#pragma once

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

#include "bsp_gpio_macro.h"

/***************************************************************************************************
 *                                           DEFINITIONS                                           *
 **************************************************************************************************/

#define APB1_FREQ           ( 50UL * 1000 * 1000)
#define APB2_FREQ           (100UL * 1000 * 1000)

#define UART_BAUD           (115200UL)

#define UART_RX_BUF_SIZE    (128U)
#define UART_RX_BUF_NUM     (2U)

#define UART0_UNIT              USART1
#define UART0_IRQ_HNDL          USART1_IRQHandler
#define UART0_PIN               PORTA_15
#define UART0_AF                GPIO_AF7_USART1
#define UART0_DMA_TX            DMA2_Stream7
#define UART0_DMA_TX_IRQ_HNDL   DMA2_Stream7_IRQHandler
#define UART0_DMA_TX_CH         (4U)

#define UART1_UNIT              USART2
#define UART1_IRQ_HNDL          USART2_IRQHandler
#define UART1_PIN               PORTA_02
#define UART1_AF                GPIO_AF7_USART2
#define UART1_DMA_TX            DMA1_Stream6
#define UART1_DMA_TX_IRQ_HNDL   DMA1_Stream6_IRQHandler
#define UART1_DMA_TX_CH         (4U)

#define UART2_UNIT              USART3
#define UART2_IRQ_HNDL          USART3_IRQHandler
#define UART2_PIN               PORTC_10
#define UART2_AF                GPIO_AF7_USART3
#define UART2_DMA_TX            DMA1_Stream3
#define UART2_DMA_TX_IRQ_HNDL   DMA1_Stream3_IRQHandler
#define UART2_DMA_TX_CH         (4U)

#define UART3_UNIT              UART4
#define UART3_IRQ_HNDL          UART4_IRQHandler
#define UART3_PIN               PORTA_00
#define UART3_AF                GPIO_AF8_UART4
#define UART3_DMA_TX            DMA1_Stream4
#define UART3_DMA_TX_IRQ_HNDL   DMA1_Stream4_IRQHandler
#define UART3_DMA_TX_CH         (4U)

#define UART4_UNIT              UART5
#define UART4_IRQ_HNDL          UART5_IRQHandler
#define UART4_PIN               PORTC_12
#define UART4_AF                GPIO_AF8_UART5
#define UART4_DMA_TX            DMA1_Stream7
#define UART4_DMA_TX_IRQ_HNDL   DMA1_Stream7_IRQHandler
#define UART4_DMA_TX_CH         (8U)

#define UART5_UNIT              USART6
#define UART5_IRQ_HNDL          USART6_IRQHandler
#define UART5_PIN               PORTC_06
#define UART5_AF                GPIO_AF8_USART6
#define UART5_DMA_TX            DMA2_Stream6
#define UART5_DMA_TX_IRQ_HNDL   DMA2_Stream6_IRQHandler
#define UART5_DMA_TX_CH         (5U)

#define UART6_UNIT              UART7
#define UART6_IRQ_HNDL          UART7_IRQHandler
#define UART6_PIN               PORTE_08
#define UART6_AF                GPIO_AF8_UART7
#define UART6_DMA_TX            DMA1_Stream1
#define UART6_DMA_TX_IRQ_HNDL   DMA1_Stream1_IRQHandler
#define UART6_DMA_TX_CH         (5U)

#define UART7_UNIT              UART8
#define UART7_IRQ_HNDL          UART8_IRQHandler
#define UART7_PIN               PORTE_01
#define UART7_AF                GPIO_AF8_UART8
#define UART7_DMA_TX            DMA1_Stream0
#define UART7_DMA_TX_IRQ_HNDL   DMA1_Stream0_IRQHandler
#define UART7_DMA_TX_CH         (5U)

#define UART8_UNIT              UART9
#define UART8_IRQ_HNDL          UART9_IRQHandler
#define UART8_PIN               PORTD_15
#define UART8_AF                GPIO_AF11_UART9
#define UART8_DMA_TX            DMA2_Stream0
#define UART8_DMA_TX_IRQ_HNDL   DMA2_Stream0_IRQHandler
#define UART8_DMA_TX_CH         (1U)

#define UART9_UNIT              UART10
#define UART9_IRQ_HNDL          UART10_IRQHandler
#define UART9_PIN               PORTE_03
#define UART9_AF                GPIO_AF11_UART10
#define UART9_DMA_TX            DMA2_Stream5
#define UART9_DMA_TX_IRQ_HNDL   DMA2_Stream5_IRQHandler
#define UART9_DMA_TX_CH         (9U)

/***************************************************************************************************
 *                                          PUBLIC TYPES                                           *
 **************************************************************************************************/

/***************************************************************************************************
 *                                         GLOBAL VARIABLES                                        *
 **************************************************************************************************/

/***************************************************************************************************
 *                                    PUBLIC FUNCTION PROTOTYPES                                   *
 **************************************************************************************************/

/***************************************************************************************************
 *                                        END OF FILE
 **************************************************************************************************/
