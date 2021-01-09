/**
 *  @file       bsp_spi.c
 *
 *  @brief      hardware independent settings of SPI interface
 *
 *  @details
 *
 *  @author     Stulov Tikhon (kudesnick@inbox.ru)
 *
 *  @date       2020/03/08
 */

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "bsp_spi.h"
#include "bsp_spi_config.h"
#include "misc_macro.h"
#include "bsp.h"
#include "fifo.h"

/***************************************************************************************************
 *                                           DEFINITIONS
 **************************************************************************************************/

#define RX_PRI 0
#define TX_PRI 0

/***************************************************************************************************
 *                                          PRIVATE TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                           PRIVATE DATA
 **************************************************************************************************/

buf_t fake_buf __attribute__((section("DMA_BUFFERS")));

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

static void _DMA_RX_reload(void)
{
    SPI_DMA_RX->CR   &= ~DMA_SxCR_EN;
    while (SPI_DMA_RX->CR & DMA_SxCR_EN);
    SPI_DMA_RX->CR   |= DMA_SxCR_EN;
    BSP_PRINTF("<s>RXr\n");
}

static void _DMA_TX_reload(void)
{
    SPI_DMA_TX->CR   &= ~DMA_SxCR_EN;
    while (SPI_DMA_TX->CR & DMA_SxCR_EN);
    SPI_DMA_TX->CR   |= DMA_SxCR_EN;
    BSP_PRINTF("<s>TXr\n");
}

/***************************************************************************************************
 *                                   INTERRUPTS HANDLERS
 **************************************************************************************************/

void EXTI4_IRQHandler(void)
{
    BSP_PRINTF("<s>exti\n");

    EXTI->PR = GPIO_EXTI_LINE(SPI_PIN_NSS);
    if (SPI_DMA_RX->NDTR != sizeof(buf_t))
    {
        _DMA_RX_reload();
        _DMA_TX_reload();
    }
}

// Rx
void DMA2_Stream2_IRQHandler(void)
{
    if (DMA2->LISR & DMA_FLAG_TCIF2_6)
    {
        // Called when switching RX buffer or when buffer is full
        DMA2->LIFCR |= DMA_FLAG_TCIF2_6;
        BSP_PRINTF("<s>rxTC\n");

        __IO uint32_t *const buf_cmplt = (SPI_DMA_RX->CR & DMA_SxCR_CT) ?
                                         &SPI_DMA_RX->M0AR : &SPI_DMA_RX->M1AR;
        ((buf_t *const)*buf_cmplt)->head.state = BUF_UART_TX_WAIT;
        *buf_cmplt = (uint32_t)buf_catch(BUF_HOST_RX_WAIT);
        
        return;
    }
    else if (DMA2->LISR & DMA_FLAG_DMEIF2_6)
    {
        DMA2->LIFCR |= DMA_FLAG_DMEIF2_6;
        BSP_PRINTF("<s>" ERR_STR "rxDME\n"); // Direct mode error
    }
    else if (DMA2->LISR & DMA_FLAG_FEIF2_6)
    {
        DMA2->LIFCR |= DMA_FLAG_FEIF2_6;
        BSP_PRINTF("<s>" ERR_STR "rxFE\n"); // FIFO overrun/underrun
    }
    else if (DMA2->LISR & DMA_FLAG_TEIF2_6)
    {
        DMA2->LIFCR |= DMA_FLAG_TEIF2_6;
        BSP_PRINTF("<s>" ERR_STR "rxTE\n"); // Transfer error
    }
    else
    {
        BSP_PRINTF("<s>" ERR_STR "rx unknwn\n"); // Unknown event
    }
    
    _DMA_RX_reload();
}

// Tx
void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & DMA_FLAG_TCIF3_7)
    {
        DMA2->LIFCR |= DMA_FLAG_TCIF3_7;  
        BSP_PRINTF("<s>txTC\n");

        __IO uint32_t *const buf_cmplt = (SPI_DMA_RX->CR & DMA_SxCR_CT) ? &SPI_DMA_RX->M0AR : &SPI_DMA_RX->M1AR;
        buf_free((buf_t *const)*buf_cmplt);
        *buf_cmplt = (uint32_t)buf_get(BUF_HOST_TX_WAIT);
        
        if (*buf_cmplt == NULL)
        {
            *buf_cmplt = (uint32_t)&fake_buf;
            fake_buf.data[0]++;
        }
        
        return;
    }
    else if (DMA2->LISR & DMA_FLAG_DMEIF3_7)
    {
        DMA2->LIFCR |= DMA_FLAG_DMEIF3_7;
        BSP_PRINTF("<s>" ERR_STR "txDM\n");
    }
    else if (DMA2->LISR & DMA_FLAG_TEIF3_7)
    {
        DMA2->LIFCR |= DMA_FLAG_TEIF3_7;
        BSP_PRINTF("<s>" ERR_STR "txT\n");
    }
    else
    {
        BSP_PRINTF("<s>" ERR_STR "tx unknwn\n");
    }
    
    _DMA_TX_reload();
}

void SPI1_IRQHandler(void)
{
    if (SPI_UNIT->SR & SPI_SR_CRCERR)
    {
        SPI_UNIT->SR &= ~SPI_SR_CRCERR;
        BSP_PRINTF("<s>CRCERR\n");
    }
    else if (SPI_UNIT->SR & SPI_SR_OVR)
    {
        do
        {
            volatile uint16_t reg = SPI_UNIT->DR;
        }
        while (SPI_UNIT->SR & SPI_SR_OVR);
        BSP_PRINTF("<s>" ERR_STR "OVR\n");
    }
    else
    {
        BSP_PRINTF("<s>" ERR_STR "irq unknwn\n");
    }
    
    _DMA_RX_reload();
}

/***************************************************************************************************
 *                                      PUBLIC FUNCTIONS
 **************************************************************************************************/

void bsp_spi_init(void)
{
    // GPIO
    GPIO_AF_SET(SPI_PIN_MOSI, GPIO_AF5_SPI1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    GPIO_AF_SET(SPI_PIN_MISO, GPIO_AF5_SPI1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    GPIO_AF_SET(SPI_PIN_SCK , GPIO_AF5_SPI1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    GPIO_AF_SET(SPI_PIN_NSS , GPIO_AF5_SPI1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    
    // Interrupt from SPI_PIN_NSS
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[GPIO_PIN_SOURCE(SPI_PIN_NSS) >> 2] |=
        GPIO_PORT_SOURCE(SPI_PIN_NSS) << (GPIO_PIN_SOURCE(SPI_PIN_NSS) & 3);
    EXTI->IMR  |=  (GPIO_EXTI_LINE(SPI_PIN_NSS));
    EXTI->EMR  &= ~(GPIO_EXTI_LINE(SPI_PIN_NSS));
    // Rising trigger enable
    EXTI->RTSR |=  (GPIO_EXTI_LINE(SPI_PIN_NSS));
    // Falling trigger disable
    EXTI->FTSR &= ~(GPIO_EXTI_LINE(SPI_PIN_NSS));
    ENABLE_IRQ(GPIO_IRQ_CHANNEL(SPI_PIN_NSS), RX_PRI);

    // DMA and SPI
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    CLR_PENDING(SPI1_IRQn);
    ENABLE_IRQ(SPI1_IRQn, TX_PRI);
    CLR_PENDING(DMA2_Stream2_IRQn); // RX
    ENABLE_IRQ(DMA2_Stream2_IRQn, RX_PRI);
    CLR_PENDING(DMA2_Stream3_IRQn); // TX
    ENABLE_IRQ(DMA2_Stream3_IRQn, TX_PRI);
    // Reset unit
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    __NOP(); __NOP(); __NOP(); __NOP();
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    // MODE_SLAVE, CPOL0, CPHA0, MSB_LSB, DATA_8_BITS, Max speed
    SPI_UNIT->CR2     = SPI_CR2_ERRIE;
    SPI_UNIT->CR1    |= SPI_CR1_SPE;
    
    // Rx DMA init
    SPI_DMA_RX->CR   &= ~DMA_SxCR_EN; // Disable DMA
    while(SPI_DMA_RX->CR & DMA_SxCR_EN);
    SPI_DMA_RX->CR    = (SPI_DMA_RX_CH << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                        (3 << DMA_SxCR_PL_Pos);
    SPI_DMA_RX->FCR   = 0;
    SPI_DMA_RX->CR   |= DMA_SxCR_DBM;
    SPI_DMA_RX->NDTR  = sizeof(buf_t);
    SPI_DMA_RX->PAR   = (uint32_t)&(SPI_UNIT->DR);
    SPI_DMA_RX->M0AR  = (uint32_t)buf_catch(BUF_HOST_RX_WAIT);
    SPI_DMA_RX->M1AR  = (uint32_t)buf_catch(BUF_HOST_RX_WAIT);
    DMA2->LIFCR = 0x3FU << 0x10; // Clear all interrupt flags
    SPI_DMA_RX->CR   |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    SPI_DMA_RX->CR   |= DMA_SxCR_EN; // Enable DMA
    SPI_UNIT->CR2    |= SPI_CR2_RXDMAEN;
    
    // Tx DMA init
    SPI_DMA_TX->CR   &= ~DMA_SxCR_EN; // Disable DMA
    while(SPI_DMA_TX->CR & DMA_SxCR_EN);
    SPI_DMA_TX->CR    = (SPI_DMA_TX_CH << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
    SPI_DMA_TX->FCR   = 0;
    SPI_DMA_TX->CR   |= DMA_SxCR_DBM;
    SPI_DMA_TX->NDTR  = sizeof(buf_t);
    SPI_DMA_TX->PAR   = (uint32_t)&(SPI_UNIT->DR);
    SPI_DMA_TX->M0AR  = (uint32_t)&fake_buf;
    SPI_DMA_TX->M1AR  = (uint32_t)&fake_buf;
    DMA2->LIFCR = 0x3FU << 0x16; // Clear all interrupt flags
    SPI_DMA_TX->CR   |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    SPI_DMA_TX->CR   |= DMA_SxCR_EN; // Enable DMA
    SPI_UNIT->CR2    |= SPI_CR2_TXDMAEN;

#ifdef TEST_BSP_SPI
    // Test code
    {
        static uint8_t data[] = {0x01, 0x0d, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0xa2};
        
        bsp_spi_tx(data);
    }
#endif
}

/**************************************************************************************************
 *                                        END OF FILE
 **************************************************************************************************/
