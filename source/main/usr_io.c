/***************************************************************************************************
 *   Project:       stm_commander
 *   Author:        Stulov Tikhon
 ***************************************************************************************************
 *   Distribution:
 *
 ***************************************************************************************************
 *   MCU Family:    STM32F
 *   Compiler:      ARMCC
 ***************************************************************************************************
 *   File:          usr_io.c
 *   Description:
 *
 ***************************************************************************************************
 *   History:       22.03.2020 - file created
 *
 **************************************************************************************************/

/***************************************************************************************************
 *                                      INCLUDED FILES
 **************************************************************************************************/

#include "RTE_Components.h"
#include CMSIS_device_header
#include <stdio.h>

#include "fifo.h"

/***************************************************************************************************
 *                                       DEFINITIONS
 **************************************************************************************************/

/* ITM registers */
#define ITM_PORT0_U8  (*((volatile uint8_t  *)0xE0000000))
#define ITM_PORT0_U32 (*((volatile uint32_t *)0xE0000000))
#define ITM_TER       (*((volatile uint32_t *)0xE0000E00))
#define ITM_TCR       (*((volatile uint32_t *)0xE0000E80))
#ifndef ITM_TCR_ITMENA_Msk
    #define ITM_TCR_ITMENA_Msk    (1UL << 0)
#endif

#define countof(_a) (sizeof(_a)/sizeof((_a)[0]))
#define FIFO_ADD(_q, _value) ((_q).data[(_q).end_idx++ & (countof((_q).data) - 1)] = (_value))
#define FIFO_IS_EMPTY(_q) ((_q).end_idx == (_q).begin_idx)
#define FIFO_EXTRACT(_q) ((_q).data[(_q).begin_idx++ & (countof((_q).data) - 1)])

/***************************************************************************************************
 *                                      PRIVATE TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                      PRIVATE DATA
 **************************************************************************************************/

struct io_queue_t
{
    volatile uint16_t begin_idx;
    volatile uint16_t end_idx;
    uint8_t data[2048];
} io_queue;

/***************************************************************************************************
 *                                     PRIVATE FUNCTIONS
 **************************************************************************************************/

#if    defined(RTE_Compiler_IO_TTY)    \
    || defined(RTE_Compiler_IO_STDOUT) \
    || defined(RTE_Compiler_IO_STDERR)

#if    defined(RTE_Compiler_IO_TTY_User)    \
    || defined(RTE_Compiler_IO_STDOUT_User) \
    || defined(RTE_Compiler_IO_STDERR_User)

#define IO_ROUTINE

int usr_put_char(int ch)
{
    __disable_irq();
    FIFO_ADD(io_queue, ch);
    __enable_irq();
    
    return ch;
}

#ifdef RTE_Compiler_IO_TTY_User
void ttywrch(int ch)
{
    usr_put_char(ch);
}
#endif

#ifdef RTE_Compiler_IO_STDOUT_User
int stdout_putchar(int ch)
{
    return usr_put_char(ch);
}
#endif

#ifdef RTE_Compiler_IO_STDERR_User
int stderr_putchar(int ch)
{
    return usr_put_char(ch);
}
#endif

#endif

#endif



#if defined(RTE_Compiler_IO_TTY) || defined(RTE_Compiler_IO_STDIN)

#if defined(RTE_Compiler_IO_TTY_User) || defined(RTE_Compiler_IO_STDIN_User)

#if (!defined(RTE_Compiler_IO_TTY_ITM)    && \
     !defined(RTE_Compiler_IO_STDIN_ITM)  && \
     !defined(RTE_Compiler_IO_STDOUT_ITM) && \
     !defined(RTE_Compiler_IO_STDERR_ITM))

volatile int32_t ITM_RxBuffer;
volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY;

#endif

int stdin_getchar(void)
{
    int result = -1;

    // Чтобы не читать данные из двух конкурирующих источников
    do
    {
        if (result < 0)
        {
            extern volatile int32_t ITM_RxBuffer;
    
            if (ITM_RxBuffer != ITM_RXBUFFER_EMPTY)
            {
                result = ITM_RxBuffer;
                ITM_RxBuffer = ITM_RXBUFFER_EMPTY;  /* ready for next character */
            }
        }
    }
    while (result == -1);

    // Некоторые терминалы посылают только один символ.
    // Для работы gets требуется именно \n
    if (result == '\r')
    {
        result = '\n';
    }

    return result;
}
#endif

#endif

/***************************************************************************************************
 *                                     PUBLIC FUNCTIONS
 **************************************************************************************************/

#ifndef IO_ROUTINE

void usr_put_routine(void)
{
};

#else

void usr_put_routine(void)
{
    while (!FIFO_IS_EMPTY(io_queue))
    {
        const uint8_t ch = FIFO_EXTRACT(io_queue);
    
        if ((ITM_TCR & ITM_TCR_ITMENA_Msk) && /* ITM enabled */
            (ITM_TER & (1UL << 0)))           /* ITM Port #0 enabled */
        {
            while (ITM_PORT0_U32 == 0);
            __NOP();
            ITM_PORT0_U8 = (uint8_t)ch;
        }
    }
}

#endif

/***************************************************************************************************
 *                                       END OF FILE
 **************************************************************************************************/
