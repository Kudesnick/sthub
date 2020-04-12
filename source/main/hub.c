/***************************************************************************************************
 *   Project:
 *   Author:        Stulov Tikhon (kudesnick@inbox.ru)
 ***************************************************************************************************
 *   Distribution:
 *
 ***************************************************************************************************
 *   MCU Family:    STM32F
 *   Compiler:      ARMCC
 ***************************************************************************************************
 *   File:          hub.c
 *   Description:
 *
 ***************************************************************************************************
 *   History:       2020/04/11 - file created
 *
 **************************************************************************************************/

/***************************************************************************************************
 *                                      INCLUDED FILES
 **************************************************************************************************/

#include <stdio.h>
#include <stdbool.h>

#include "hub.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "misc_macro.h"
#include "fifo.h"
#include "bsp.h"

/***************************************************************************************************
 *                                       DEFINITIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                                      PRIVATE TYPES
 **************************************************************************************************/

typedef FIFO_TYPEDEF(uint8_t *, uint8_t, FIFO_SIZE_128) hub_fifo_t;

typedef struct
{
    hub_fifo_t fifo;
          bool not_ready;
} hub_ch_t;

/***************************************************************************************************
 *                               PRIVATE FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                       PRIVATE DATA
 **************************************************************************************************/

hub_ch_t ch_uart_tx[UART_CNT];
hub_ch_t ch_uart_rx;
hub_ch_t ch_spi_tx;
hub_ch_t ch_spi_rx;

/***************************************************************************************************
 *                                       PUBLIC DATA
 **************************************************************************************************/

/***************************************************************************************************
 *                                      EXTERNAL DATA
 **************************************************************************************************/

/***************************************************************************************************
 *                              EXTERNAL FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                    PRIVATE FUNCTIONS
 **************************************************************************************************/

static bool _crc_calc(uint8_t *const _data)
{
    // CRC-8/SAE-J1850
    // width=8  poly=0x1d  init=0xff  refin=false  refout=false  xorout=0xff  check=0x4b
    static const uint8_t poly   = 0x1d;
    static const uint8_t xorout = 0xff;
    uint8_t crc = 0xff;

    uint8_t len = _data[LEN_PTR];

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= _data[i]; 
        
        for (uint8_t j = 8; j > 0; --j)
        {
            crc = crc & (1 << 7) ? (crc << 1) ^ poly : crc << 1;
        }
    }

    crc ^= xorout;
    len--;

    bool result = (_data[len] == crc);
    
    _data[len] = crc ^ xorout;
    
    return result;
};

/***************************************************************************************************
 *                                    WEAKLY FUNCTIONS
 **************************************************************************************************/

void bsp_spi_tx_callback(const bool _ok)
{
    ch_spi_tx.not_ready = !_ok;
    
    if (!_ok)
    {
        HUB_PRINTF("<hub> spi tx callback error!");
    }
}

void bsp_uart_tx_callback(const uint8_t _n, const bool _ok)
{
    ch_uart_tx[_n].not_ready = !_ok;
    
    if (!_ok)
    {
        HUB_PRINTF("<hub> uart#%d tx callback error!", _n);
    }
}

bool bsp_spi_rx_callback(uint8_t *const _data)
{
#warning exclude verification into routine
    if (FIFO_IS_FULL(ch_uart_tx[_data[IFACE_NUM_PTR]].fifo))
    {
        HUB_PRINTF("<hub> spi rx callback error! Buffer i overflow.");
        return false;
    }
    
    _data[LEN_PTR]--;
    FIFO_ADD(ch_uart_tx[_data[IFACE_NUM_PTR]].fifo, _data);
    
    return true;
}

bool bsp_uart_rx_callback(uint8_t *const _data)
{
    if (FIFO_IS_FULL(ch_uart_rx.fifo))
    {
        HUB_PRINTF("<hub> uart#%d rx callback error! Buffer is overflow.", _data[IFACE_NUM_PTR]);
        return false;
    }

    FIFO_ADD(ch_uart_rx.fifo, _data);

    return true;
}

/***************************************************************************************************
 *                                    PUBLIC FUNCTIONS
 **************************************************************************************************/

void hub_routine(void)
{
    // uart rx route
    if (true
        && !FIFO_IS_EMPTY(ch_uart_rx.fifo)
        && !FIFO_IS_FULL(ch_spi_tx.fifo)
       )
    {
        uint8_t *head = FIFO_EXTRACT(ch_uart_rx.fifo);
        head[LEN_PTR]++;
        _crc_calc(head);
        FIFO_ADD(ch_spi_tx.fifo, head);
    }
    
    // uart tx route
    for (uint8_t i = 0; i < UART_CNT; i++)
    {
        if (true
            && !ch_uart_tx[i].not_ready
            && !FIFO_IS_EMPTY(ch_uart_tx[i].fifo)
           )
        {
            uint8_t *head = FIFO_EXTRACT(ch_uart_tx[i].fifo);
            if (!bsp_uart_tx(head))
            {
                FIFO_ADD(ch_uart_tx[i].fifo, head);
            }
        }
    }
    
    // spi rx route
    if (!FIFO_IS_EMPTY(ch_spi_rx.fifo))
    {
        uint8_t *head = FIFO_EXTRACT(ch_spi_rx.fifo);
        
        if (!_crc_calc(head))
        {
            HUB_PRINTF("<hub> spi rx CRC error!");
        }
        
        else if (head[IFACE_NUM_PTR] >= UART_CNT)
        {
            HUB_PRINTF("<hub> spi rx UART number error!");
        }
        
        else if (FIFO_IS_FULL(ch_uart_tx[head[IFACE_NUM_PTR]].fifo))
        {
            FIFO_ADD(ch_spi_rx.fifo, head);
            HUB_PRINTF("<hub> uart#%d tx buffer overflow", head[IFACE_NUM_PTR]);
        }
        
        else
        {
            head[LEN_PTR]--;
            FIFO_ADD(ch_uart_tx[head[IFACE_NUM_PTR]].fifo, head);
        }
    }
    
    // spi tx routine
    if (true
        && ch_spi_tx.not_ready
        && !FIFO_IS_EMPTY(ch_spi_tx.fifo)
       )
    {
        uint8_t *head = FIFO_EXTRACT(ch_spi_tx.fifo);
        if (!bsp_spi_tx(head))
        {
            FIFO_ADD(ch_spi_tx.fifo, head);
        }
    }
}

/***************************************************************************************************
 *                                       END OF FILE
 **************************************************************************************************/
