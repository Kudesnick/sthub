/***************************************************************************************************
 *   Project:       lib
 *   Author:
 ***************************************************************************************************
 *   Distribution:
 *
 ***************************************************************************************************
 *   MCU Family:
 *   Compiler:
 ***************************************************************************************************
 *   File:          fifo.h
 *   Description:   fifo library
 *
 *   structure of FIFO-buffer must be as:
 *
 *   typedef struct
 *   {
 *       volatile counter_t begin_idx;
 *       volatile counter_t end_idx;
 *       data_t data[SIZE];
 *   } queue_t;
 *
 *   SIZE - must be binary exponent
 *   counter_t - unsigned fixed point type
 *   data_t - user data type
 *
 ***************************************************************************************************
 *   History:       15.03.2011 - [] - file created
 *
 **************************************************************************************************/

#pragma once

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

#include "RTE_Components.h"
#include CMSIS_device_header

/***************************************************************************************************
 *                                           DEFINITIONS                                           *
 **************************************************************************************************/

#define BUF_CNT  32
#define BUF_SIZE 32

/***************************************************************************************************
 *                                          PUBLIC TYPES                                           *
 **************************************************************************************************/

typedef __PACKED_STRUCT
{
    __PACKED_UNION
    {
        __PACKED_STRUCT
        {
            uint8_t state  : 4;
            uint8_t channel: 4;
        };
        uint8_t free_status;
    };
    uint8_t len;
} buf_hdr_t;

typedef __PACKED_STRUCT
{
    buf_hdr_t head;
    uint8_t data[BUF_SIZE - sizeof(buf_hdr_t)];
} buf_t;

typedef enum
{
    BUF_FREE = 0,
    BUF_HOST_RX_WAIT, // 1
    BUF_HOST_TX_WAIT, // 2
    BUF_UART_RX_WAIT, // 3
    BUF_UART_TX_WAIT, // 4
} buf_state_t;

/***************************************************************************************************
 *                                         GLOBAL VARIABLES                                        *
 **************************************************************************************************/

/***************************************************************************************************
 *                                    PUBLIC FUNCTION PROTOTYPES                                   *
 **************************************************************************************************/

buf_t *const buf_catch(const buf_state_t _state);
buf_t *const buf_get(const buf_state_t _state);
buf_t *const buf_get_ch(const buf_state_t _state, const uint8_t _ch);
void buf_free(buf_t *const _buf);
