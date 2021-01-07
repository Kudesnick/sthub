/**
 *  @file       bsp_uart.c
 *
 *  @brief      
 *
 *  @details
 *
 *  @author     Stulov Tikhon
 *
 *  @date       2020/03/14
 */

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "bsp_uart_config.h"
#include "misc_macro.h"
#include "bsp.h"
#include "fifo.h"

/***************************************************************************************************
 *                                           DEFINITIONS
 **************************************************************************************************/

#define TX_PRI 2
#define RX_PRI 1

#define DMA_FLAG_FEIF  DMA_FLAG_FEIF0_4  // 0x00000001U
#define DMA_FLAG_DMEIF DMA_FLAG_DMEIF0_4 // 0x00000004U
#define DMA_FLAG_TEIF  DMA_FLAG_TEIF0_4  // 0x00000008U
#define DMA_FLAG_HTIF  DMA_FLAG_HTIF0_4  // 0x00000010U
#define DMA_FLAG_TCIF  DMA_FLAG_TCIF0_4  // 0x00000020U

#define DMA_BASE(_stream) ((bsp_DMA_TypeDef *)((uint32_t)(_stream) & (uint32_t)DMA2))
#define DMA_STREAM_NO(_stream) ((((uint32_t)(_stream) & ~(uint32_t)DMA2) - 0x10) / 0x18)
#define DMA_IFCR(_stream) (DMA_BASE(_stream)->IFCR[DMA_STREAM_NO(_stream) >> 2])
#define DMA_ISR(_stream) (DMA_BASE(_stream)->ISR[DMA_STREAM_NO(_stream) >> 2])
#define DMA_IF_LS(_stream, _flag) ((uint32_t)(_flag) << \
    (((DMA_STREAM_NO(_stream) & 1U) ? 6  : 0) + ((DMA_STREAM_NO(_stream) & 2U) ? 16 : 0)))

#define BAUD_CALC(_uart, _baud) \
    ((((uint32_t)(_uart) < (uint32_t)APB2PERIPH_BASE) ? APB1_FREQ : APB2_FREQ) / (_baud))

/***************************************************************************************************
 *                                          PRIVATE TYPES
 **************************************************************************************************/

typedef struct
{
  __IO uint32_t ISR[2];
  __IO uint32_t IFCR[2];
} bsp_DMA_TypeDef;

typedef struct
{
    USART_TypeDef      *uart;
    uint8_t            pin;
    uint8_t            af;
    DMA_Stream_TypeDef *tx_dma;
    uint8_t            tx_dma_ch;
} bsp_uart_unit_t;

/***************************************************************************************************
 *                                           PRIVATE DATA
 **************************************************************************************************/

const bsp_uart_unit_t uart[UART_CNT] =
{
    {
        .uart           = UART0_UNIT     ,
        .pin            = UART0_PIN      ,
        .af             = UART0_AF       ,
        .tx_dma         = UART0_DMA_TX   ,
        .tx_dma_ch      = UART0_DMA_TX_CH,
    },
    {
        .uart           = UART1_UNIT     ,
        .pin            = UART1_PIN      ,
        .af             = UART1_AF       ,
        .tx_dma         = UART1_DMA_TX   ,
        .tx_dma_ch      = UART1_DMA_TX_CH,
    },      
    {       
        .uart           = UART2_UNIT     ,
        .pin            = UART2_PIN      ,
        .af             = UART2_AF       ,
        .tx_dma         = UART2_DMA_TX   ,
        .tx_dma_ch      = UART2_DMA_TX_CH,
    },      
    {       
        .uart           = UART3_UNIT     ,
        .pin            = UART3_PIN      ,
        .af             = UART3_AF       ,
        .tx_dma         = UART3_DMA_TX   ,
        .tx_dma_ch      = UART3_DMA_TX_CH,
    },      
    {       
        .uart           = UART4_UNIT     ,
        .pin            = UART4_PIN      ,
        .af             = UART4_AF       ,
        .tx_dma         = UART4_DMA_TX   ,
        .tx_dma_ch      = UART4_DMA_TX_CH,
    },      
    {       
        .uart           = UART5_UNIT     ,
        .pin            = UART5_PIN      ,
        .af             = UART5_AF       ,
        .tx_dma         = UART5_DMA_TX   ,
        .tx_dma_ch      = UART5_DMA_TX_CH,
    },      
    {       
        .uart           = UART6_UNIT     ,
        .pin            = UART6_PIN      ,
        .af             = UART6_AF       ,
        .tx_dma         = UART6_DMA_TX   ,
        .tx_dma_ch      = UART6_DMA_TX_CH,
    },      
    {       
        .uart           = UART7_UNIT     ,
        .pin            = UART7_PIN      ,
        .af             = UART7_AF       ,
        .tx_dma         = UART7_DMA_TX   ,
        .tx_dma_ch      = UART7_DMA_TX_CH,
    },      
    {       
        .uart           = UART8_UNIT     ,
        .pin            = UART8_PIN      ,
        .af             = UART8_AF       ,
        .tx_dma         = UART8_DMA_TX   ,
        .tx_dma_ch      = UART8_DMA_TX_CH,
    },      
    {       
        .uart           = UART9_UNIT     ,
        .pin            = UART9_PIN      ,
        .af             = UART9_AF       ,
        .tx_dma         = UART9_DMA_TX   ,
        .tx_dma_ch      = UART9_DMA_TX_CH,
    },
};

buf_t *buf[UART_CNT];

/***************************************************************************************************
 *                                           PUBLIC DATA
 **************************************************************************************************/

/***************************************************************************************************
 *                                          EXTERNAL DATA
 **************************************************************************************************/

/***************************************************************************************************
 *                                      PRIVATE FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                      PRIVATE FUNCTIONS
 **************************************************************************************************/

static __INLINE IRQn_Type _irq_num(const void *const _u)
{
    return
    (_u == USART1) ? USART1_IRQn :
    (_u == USART2) ? USART2_IRQn :
    (_u == USART3) ? USART3_IRQn :
    (_u == UART4 ) ? UART4_IRQn  :
    (_u == UART5 ) ? UART5_IRQn  :
    (_u == USART6) ? USART6_IRQn :
    (_u == UART7 ) ? UART7_IRQn  :
    (_u == UART8 ) ? UART8_IRQn  :
    (_u == UART9 ) ? UART9_IRQn  :
    (_u == UART10) ? UART10_IRQn :
    
    (_u == DMA1_Stream0) ? DMA1_Stream0_IRQn :
    (_u == DMA1_Stream1) ? DMA1_Stream1_IRQn :
    (_u == DMA1_Stream2) ? DMA1_Stream2_IRQn :
    (_u == DMA1_Stream3) ? DMA1_Stream3_IRQn :
    (_u == DMA1_Stream4) ? DMA1_Stream4_IRQn :
    (_u == DMA1_Stream5) ? DMA1_Stream5_IRQn :
    (_u == DMA1_Stream6) ? DMA1_Stream6_IRQn :
    (_u == DMA1_Stream7) ? DMA1_Stream7_IRQn :
    
    (_u == DMA2_Stream0) ? DMA2_Stream0_IRQn :
    (_u == DMA2_Stream1) ? DMA2_Stream1_IRQn :
    (_u == DMA2_Stream2) ? DMA2_Stream2_IRQn :
    (_u == DMA2_Stream3) ? DMA2_Stream3_IRQn :
    (_u == DMA2_Stream4) ? DMA2_Stream4_IRQn :
    (_u == DMA2_Stream5) ? DMA2_Stream5_IRQn :
    (_u == DMA2_Stream6) ? DMA2_Stream6_IRQn :
    (_u == DMA2_Stream7) ? DMA2_Stream7_IRQn :
    
    NonMaskableInt_IRQn;
};

static __INLINE void _rcc_dma_en(const void *const _dma)
{
    switch ((const uint32_t)DMA_BASE(_dma))
    {
        case (const uint32_t)DMA1: RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; break;
        case (const uint32_t)DMA2: RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; break;
    }
};

static __INLINE void _rcc_uart_en(const void *const _u)
{
    switch ((const uint32_t)_u)
    {
        case (const uint32_t)USART1: RCC->APB2ENR |= RCC_APB2ENR_USART1EN; break;
        case (const uint32_t)USART2: RCC->APB1ENR |= RCC_APB1ENR_USART2EN; break;
        case (const uint32_t)USART3: RCC->APB1ENR |= RCC_APB1ENR_USART3EN; break;
        case (const uint32_t)UART4 : RCC->APB1ENR |= RCC_APB1ENR_UART4EN ; break;
        case (const uint32_t)UART5 : RCC->APB1ENR |= RCC_APB1ENR_UART5EN ; break;
        case (const uint32_t)USART6: RCC->APB2ENR |= RCC_APB2ENR_USART6EN; break;
        case (const uint32_t)UART7 : RCC->APB1ENR |= RCC_APB1ENR_UART7EN ; break;
        case (const uint32_t)UART8 : RCC->APB1ENR |= RCC_APB1ENR_UART8EN ; break;
        case (const uint32_t)UART9 : RCC->APB2ENR |= RCC_APB2ENR_UART9EN ; break;
        case (const uint32_t)UART10: RCC->APB2ENR |= RCC_APB2ENR_UART10EN; break;
    }
}

static __INLINE void _rcc_reset(const void *const _u)
{
    __IO uint32_t *reg = NULL;
    uint32_t mask;
    
    switch ((const uint32_t)_u)
    {
        case (const uint32_t)USART1: reg = &(RCC->APB2RSTR); mask = RCC_APB2RSTR_USART1RST; break;
        case (const uint32_t)USART2: reg = &(RCC->APB1RSTR); mask = RCC_APB1RSTR_USART2RST; break;
        case (const uint32_t)USART3: reg = &(RCC->APB1RSTR); mask = RCC_APB1RSTR_USART3RST; break;
        case (const uint32_t)UART4 : reg = &(RCC->APB1RSTR); mask = RCC_APB1RSTR_UART4RST ; break;
        case (const uint32_t)UART5 : reg = &(RCC->APB1RSTR); mask = RCC_APB1RSTR_UART5RST ; break;
        case (const uint32_t)USART6: reg = &(RCC->APB2RSTR); mask = RCC_APB2RSTR_USART6RST; break;
        case (const uint32_t)UART7 : reg = &(RCC->APB1RSTR); mask = RCC_APB1RSTR_UART7RST ; break;
        case (const uint32_t)UART8 : reg = &(RCC->APB1RSTR); mask = RCC_APB1RSTR_UART8RST ; break;
        case (const uint32_t)UART9 : reg = &(RCC->APB2RSTR); mask = RCC_APB2RSTR_UART9RST ; break;
        case (const uint32_t)UART10: reg = &(RCC->APB2RSTR); mask = RCC_APB2RSTR_UART10RST; break;
    }
    
    if (reg)
    {
        *reg |= mask;
        __NOP(); __NOP(); __NOP(); __NOP();
        *reg &= ~mask;
    }
}

static void _rx_callback(const uint8_t _n)
{
    for (;;)
    {
        buf_t *const buf_tmp = buf_catch(BUF_UART_RX_WAIT);
        
        if (buf_tmp == NULL)
        {
            BSP_PRINTF("<U%d>" ERR_STR "Buf OVF\n", _n);
        }
        else
        {
            BSP_PRINTF("<U%d>rx %d\n", _n, buf[_n]->head.len);
        
            buf_tmp->head.channel = _n;
            buf_tmp->head.len = 0;
            buf[_n]->head.state = BUF_HOST_TX_WAIT;
            buf[_n] = buf_tmp;
            
            return;
        }
    }
}

static __INLINE void _uart_irq_hdl(const uint8_t _n)
{
    bool result = false;

    const bsp_uart_unit_t *const u = &uart[_n];
    // If you read the status register, then read the data register,
    // the flags IDLE, ORE, NE, FE and PE will be reset. See RM0430 Rev 8 page 926/1324
    volatile const uint32_t sreg = u->uart->SR;
    volatile const uint32_t data = u->uart->DR;
    
    if (u->uart->CR1 & USART_CR1_RXNEIE)
    {
        if (sreg & USART_SR_ORE)
        {
            BSP_PRINTF("<U%d>irOR\r\n", _n);
        }
        if (sreg & USART_SR_RXNE)
        {
            u->uart->CR1 |= USART_CR1_IDLEIE;
            // not use printf for speed optimization 
            // BSP_PRINTF("<u%d>RXNE\n", _n);

            // received one byte
            buf_t *const buf_tmp = buf[_n];
            buf_tmp->data[buf_tmp->head.len++] = data;
            
            if (buf_tmp->head.len >= (sizeof(buf_tmp->data)))
            {
                _rx_callback(_n);
            }
            
            return;
        }
    }
    
    if (true
        && u->uart->CR1 & USART_CR1_IDLEIE
        && sreg & USART_SR_IDLE
       )
    {
        SET_IRQ_PRI(_irq_num(uart[_n].uart), TX_PRI);
        u->uart->CR1 &= ~USART_CR1_IDLEIE;
        
        BSP_PRINTF("<U%d>irIDL\n", _n);
        
        // received all bytes
        _rx_callback(_n);
        
        SET_IRQ_PRI(_irq_num(uart[_n].uart), RX_PRI);
        
        return;
    }
    
    if (true
        && u->uart->CR1 & USART_CR1_TCIE
        && sreg & USART_SR_TC
       )
    {
        // TX complete
        u->uart->CR1 &= ~(USART_CR1_TE | USART_CR1_TCIE);
        BSP_PRINTF("<U%d>irTX\n", _n);

        buf_t *const buf_tmp = (buf_t *)(u->tx_dma->M0AR - sizeof(buf_hdr_t));
        buf_free(buf_tmp);

        u->uart->CR1 |=  USART_CR1_RE;
        SET_IRQ_PRI(_irq_num(uart[_n].uart), RX_PRI);
        
        return;
    }
    
    if (!result)
    {
        BSP_PRINTF("<U%d>irU\n", _n);
    }
}

// DMA TX
static void _uart_dma_tx_irq_hndl(const uint8_t _n)
{
    const bsp_uart_unit_t *const u = &uart[_n];

    if (DMA_ISR(u->tx_dma) & DMA_IF_LS(u->tx_dma, DMA_FLAG_TCIF))
    {
        DMA_IFCR(u->tx_dma) |= DMA_IF_LS(u->tx_dma, DMA_FLAG_TCIF); 
        u->uart->CR1 |= USART_CR1_TCIE;
        BSP_PRINTF("<U%d>dmaTC\n", _n);
    }
    else if (DMA_ISR(u->tx_dma) & DMA_IF_LS(u->tx_dma, DMA_FLAG_DMEIF))
    {
        DMA_IFCR(u->tx_dma) |= DMA_IF_LS(u->tx_dma, DMA_FLAG_DMEIF);
        BSP_PRINTF("<U%d>" ERR_STR "dmaDME\n", _n);
    }
    else if (DMA_ISR(u->tx_dma) & DMA_IF_LS(u->tx_dma, DMA_FLAG_TEIF))
    {
        DMA_IFCR(u->tx_dma) |= DMA_IF_LS(u->tx_dma, DMA_FLAG_TEIF);
        BSP_PRINTF("<U%d>" ERR_STR "dmaTE\n", _n);
    }
    else
    {
        BSP_PRINTF("<U%d>" ERR_STR "dmaU\n", _n);
    }
}

static void _uart_init(const uint8_t _n)
{
    // UART pins setting
    GPIO_AF_SET(uart[_n].pin, uart[_n].af, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);

    _rcc_dma_en(uart[_n].tx_dma);

    _rcc_uart_en(uart[_n].uart);
    CLR_PENDING(_irq_num(uart[_n].uart));
    ENABLE_IRQ(_irq_num(uart[_n].uart), RX_PRI);
    CLR_PENDING(_irq_num(uart[_n].tx_dma)); // TX
    ENABLE_IRQ(_irq_num(uart[_n].tx_dma), TX_PRI);
    _rcc_reset(uart[_n].uart);
    
    uart[_n].uart->SR   &= ~USART_SR_RXNE;
    uart[_n].uart->CR1   = USART_CR1_RXNEIE | USART_CR1_RE;
    uart[_n].uart->CR2   = 0;
    uart[_n].uart->CR3   = USART_CR3_HDSEL | USART_CR3_EIE;
    uart[_n].uart->BRR   = BAUD_CALC(uart[_n].uart, UART_BAUD);
    uart[_n].uart->CR1  |= USART_CR1_UE;
    
    // Tx DMA init
    uart[_n].tx_dma->CR &= ~DMA_SxCR_EN; // Disable DMA
    while(uart[_n].tx_dma->CR & DMA_SxCR_EN);
    uart[_n].tx_dma->CR  = ((uint32_t)(uart[_n].tx_dma_ch) << DMA_SxCR_CHSEL_Pos) |
                           DMA_SxCR_MINC | 
                           DMA_SxCR_DIR_0;
    uart[_n].tx_dma->FCR = 0;
    uart[_n].tx_dma->PAR = (uint32_t)&(uart[_n].uart->DR);
    DMA_IFCR(uart[_n].tx_dma) = DMA_IF_LS(uart[_n].tx_dma, 0x3FU);
    uart[_n].tx_dma->CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    uart[_n].uart->CR3  |= USART_CR3_DMAT;
}

#ifdef UART0_IRQ_HNDL
void UART0_IRQ_HNDL(void) {_uart_irq_hdl(0);}
#endif

#ifdef UART1_IRQ_HNDL
void UART1_IRQ_HNDL(void) {_uart_irq_hdl(1);}
#endif

#ifdef UART2_IRQ_HNDL
void UART2_IRQ_HNDL(void) {_uart_irq_hdl(2);}
#endif

#ifdef UART3_IRQ_HNDL
void UART3_IRQ_HNDL(void) {_uart_irq_hdl(3);}
#endif

#ifdef UART4_IRQ_HNDL
void UART4_IRQ_HNDL(void) {_uart_irq_hdl(4);}
#endif

#ifdef UART5_IRQ_HNDL
void UART5_IRQ_HNDL(void) {_uart_irq_hdl(5);}
#endif

#ifdef UART6_IRQ_HNDL
void UART6_IRQ_HNDL(void) {_uart_irq_hdl(6);}
#endif

#ifdef UART7_IRQ_HNDL
void UART7_IRQ_HNDL(void) {_uart_irq_hdl(7);}
#endif

#ifdef UART8_IRQ_HNDL
void UART8_IRQ_HNDL(void) {_uart_irq_hdl(8);}
#endif

#ifdef UART9_IRQ_HNDL
void UART9_IRQ_HNDL(void) {_uart_irq_hdl(9);}
#endif

#ifdef UART0_DMA_TX_IRQ_HNDL
void UART0_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(0);}
#endif

#ifdef UART1_DMA_TX_IRQ_HNDL
void UART1_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(1);}
#endif

#ifdef UART2_DMA_TX_IRQ_HNDL
void UART2_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(2);}
#endif

#ifdef UART3_DMA_TX_IRQ_HNDL
void UART3_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(3);}
#endif

#ifdef UART4_DMA_TX_IRQ_HNDL
void UART4_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(4);}
#endif

#ifdef UART5_DMA_TX_IRQ_HNDL
void UART5_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(5);}
#endif

#ifdef UART6_DMA_TX_IRQ_HNDL
void UART6_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(6);}
#endif

#ifdef UART7_DMA_TX_IRQ_HNDL
void UART7_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(7);}
#endif

#ifdef UART8_DMA_TX_IRQ_HNDL
void UART8_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(8);}
#endif

#ifdef UART9_DMA_TX_IRQ_HNDL
void UART9_DMA_TX_IRQ_HNDL(void) {_uart_dma_tx_irq_hndl(9);}
#endif

/***************************************************************************************************
 *                                      PUBLIC FUNCTIONS
 **************************************************************************************************/

void bsp_uart_init(void)
{
    for (uint8_t i = 0; i < UART_CNT; i++)
    {
        _uart_init(i);
        buf[i] = buf_catch(BUF_UART_RX_WAIT);
        buf[i]->head.channel = i;
        buf[i]->head.len = 0;
    }
    
#ifdef TEST_BSP_UART
    for (volatile uint8_t i = 0; i < UART_CNT; i++)
    {
        buf_t *const buf_tmp = buf_catch(BUF_UART_TX_WAIT);
        buf_tmp->head.channel = i;
        buf_tmp->head.len = i + 3;
        
        for (volatile uint8_t j = 0; j < buf_tmp->head.len; j++)
        {
            buf_tmp->data[j] = (i << 4) | j;
        }
    }

    for (volatile uint8_t i = 1; i < UART_CNT; i+=2)
    {
        while (!bsp_uart_tx(i));
    }

    /* This delay is necessary because no byte receive interrupts have yet occurred at the start
    of the cycle. Because of this, transmission begins on a line that is already receiving data. */
    for (volatile uint16_t i = 0xFFF; i > 0; i--){};
    
    for (volatile uint8_t i = 0; i < UART_CNT; i+=2)
    {
        while (!bsp_uart_tx(i));
    }
#endif
}

bool bsp_uart_tx(const uint8_t _n)
{
    buf_t *const buf_tmp = buf_get_ch(BUF_UART_TX_WAIT, _n);

    if (false
        || buf_tmp == NULL
        || uart[_n].uart->CR1 & (USART_CR1_TE | USART_CR1_IDLEIE)
        || uart[_n].tx_dma->CR & DMA_SxCR_EN
        )
    {
        return false;
    }

    uart[_n].tx_dma->NDTR     = buf_tmp->head.len;
    uart[_n].tx_dma->M0AR     = (uint32_t)(buf_tmp->data);
    DMA_IFCR(uart[_n].tx_dma) = DMA_IF_LS(uart[_n].tx_dma, 0x3FU);
    uart[_n].tx_dma->CR      |= DMA_SxCR_EN;

    uart[_n].uart->CR1 &= ~(USART_CR1_RE | USART_CR1_IDLEIE);
    SET_IRQ_PRI(_irq_num(uart[_n].uart), TX_PRI);
    uart[_n].uart->CR1 |=  (USART_CR1_TE);

    BSP_PRINTF("<U%d>txT\n", _n);

    return true;
}

/**************************************************************************************************
 *                                        END OF FILE
 **************************************************************************************************/
