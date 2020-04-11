/**
 *  @file       bsp_uart.h
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

#include <stdint.h>
#include <stdbool.h>

/***************************************************************************************************
 *                                           DEFINITIONS                                           *
 **************************************************************************************************/

#define UART_RX_BUF_SIZE    (64U)
#define UART_RX_BUF_NUM     (16U)
#define UART_CNT            (10U)

/***************************************************************************************************
 *                                          PUBLIC TYPES                                           *
 **************************************************************************************************/

/***************************************************************************************************
 *                                         GLOBAL VARIABLES                                        *
 **************************************************************************************************/

/***************************************************************************************************
 *                                    PUBLIC FUNCTION PROTOTYPES                                   *
 **************************************************************************************************/

void bsp_uart_init(void);

bool bsp_uart_tx(const uint8_t _n, const uint8_t *const _data, const uint8_t _size);

void bsp_uart_tx_callback(const uint8_t _n, const bool _ok);

bool bsp_uart_rx_callback(uint8_t *const _data);

/***************************************************************************************************
 *                                        END OF FILE
 **************************************************************************************************/
