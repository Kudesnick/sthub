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

/***************************************************************************************************
 *                                           DEFINITIONS
 **************************************************************************************************/

#define RX_PRI 0
#define TX_PRI 2

/**
 *  @brief      Clear data register after transmit complete.
 *  @details    Data register not atomatic clear after transmit. And master receives last byte
 *              always. This setting has clear data register after ending of transmit. And master
 *              receives zero data untill new transaction.
 */
#ifndef TXCLR
    #define TXCLR 1
#endif

/***************************************************************************************************
 *                                          PRIVATE TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                           PRIVATE DATA
 **************************************************************************************************/

uint8_t buf_rx[BUF_RX_CNT][BUF_RX_LEN];
uint8_t buf_cnt = 1;

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
    BSP_PRINTF("<spi> DMA RX reload\r\n");
}

static void _DMA_TX_reload(void)
{
    SPI_DMA_TX->CR   &= ~DMA_SxCR_EN;
    while (SPI_DMA_TX->CR & DMA_SxCR_EN);
    SPI_DMA_TX->CR   |= DMA_SxCR_EN;
    BSP_PRINTF("<spi> DMA TX reload\r\n");
}

/***************************************************************************************************
 *                                   INTERRUPTS HANDLERS
 **************************************************************************************************/

void EXTI4_IRQHandler(void)
{
    const uint16_t len = sizeof(buf_rx[0]) - SPI_DMA_RX->NDTR;

    // Switch RX buffer
    SPI_DMA_RX->CR &= ~DMA_SxCR_EN;
    SPI_DMA_RX->CR ^= DMA_SxCR_CT;
    SPI_DMA_RX->CR |= DMA_SxCR_EN;
    
    SET_IRQ_PRI(GPIO_IRQ_CHANNEL(SPI_PIN_NSS), TX_PRI);

    if (len)
    {
        const uint8_t buf_num = (SPI_DMA_RX->CR & DMA_SxCR_CT) ? 0 : 1;
        buf_rx[buf_num][LEN_PTR] = len;

        if (bsp_spi_rx_callback(buf_rx[buf_num]))
        {
            if (buf_cnt++ > BUF_RX_CNT)
            {
                buf_cnt = 0;
            }
            if (buf_num)
            {
                SPI_DMA_RX->M0AR = (uint32_t)&buf_rx[buf_cnt];
            }
            else
            {
                SPI_DMA_RX->M1AR = (uint32_t)&buf_rx[buf_cnt];
            }
        }
    }
    
    EXTI->PR = GPIO_EXTI_LINE(SPI_PIN_NSS);
    
    SET_IRQ_PRI(GPIO_IRQ_CHANNEL(SPI_PIN_NSS), RX_PRI);
}

// Rx
void DMA2_Stream2_IRQHandler(void)
{
    if (DMA2->LISR & DMA_FLAG_TCIF2_6)
    {
        // Called from EXTI interrupt, when switching RX buffer or when buffer is full
        DMA2->LIFCR |= DMA_FLAG_TCIF2_6;
        BSP_PRINTF("<spi> DMA RX irq DMA_FLAG_TCIF\r\n");
        
        return;
    }
    else if (DMA2->LISR & DMA_FLAG_DMEIF2_6)
    {
        DMA2->LIFCR |= DMA_FLAG_DMEIF2_6;
        BSP_PRINTF("<spi>" ERR_STR "DMA RX irq DMA_FLAG_DMEIF\r\n"); // Direct mode error
    }
    else if (DMA2->LISR & DMA_FLAG_FEIF2_6)
    {
        DMA2->LIFCR |= DMA_FLAG_FEIF2_6;
        BSP_PRINTF("<spi>" ERR_STR "DMA RX irq DMA_FLAG_FEIF\r\n"); // FIFO overrun/underrun
    }
    else if (DMA2->LISR & DMA_FLAG_TEIF2_6)
    {
        DMA2->LIFCR |= DMA_FLAG_TEIF2_6;
        BSP_PRINTF("<spi>" ERR_STR "DMA RX irq DMA_FLAG_TEIF\r\n"); // Transfer error
    }
    else
    {
        BSP_PRINTF("<spi>" ERR_STR "DMA RX irq unknown\r\n"); // Unknown event
    }
    
    _DMA_RX_reload();
}

// Tx
void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & DMA_FLAG_TCIF3_7)
    {
        DMA2->LIFCR |= DMA_FLAG_TCIF3_7;
#if (TXCLR != 0)
        SPI_UNIT->CR2 |= SPI_CR2_TXEIE;
#endif   
        BSP_PRINTF("<spi> DMA TX irq DMA_FLAG_TCIF\r\n");

        bsp_spi_tx_callback(true);
        return;
    }
    else if (DMA2->LISR & DMA_FLAG_DMEIF3_7)
    {
        DMA2->LIFCR |= DMA_FLAG_DMEIF3_7;
        BSP_PRINTF("<spi>" ERR_STR "DMA TX irq DMA_FLAG_DMEIF\r\n");
    }
    else if (DMA2->LISR & DMA_FLAG_TEIF3_7)
    {
        DMA2->LIFCR |= DMA_FLAG_TEIF3_7;
        BSP_PRINTF("<spi>" ERR_STR "DMA TX irq DMA_FLAG_TEIF\r\n");
    }
    else
    {
        BSP_PRINTF("<spi>" ERR_STR "DMA TX irq unknown\r\n");
    }
    
    _DMA_TX_reload();
    bsp_spi_tx_callback(false);
}

void SPI1_IRQHandler(void)
{
#if (TXCLR != 0)
    if (true
        && SPI_UNIT->CR2 & SPI_CR2_TXEIE
        && SPI_UNIT->SR & SPI_SR_TXE
       )
    {
        SPI_UNIT->DR = 0x55;
        SPI_UNIT->CR2 &= ~SPI_CR2_TXEIE;
        BSP_PRINTF("<spi> SPI irq flag: SPI_SR_TXE (TXCLR)\r\n");
        
        return;
    }
#endif
    if (SPI_UNIT->SR & SPI_SR_CRCERR)
    {
        SPI_UNIT->SR &= ~SPI_SR_CRCERR;
        BSP_PRINTF("<spi> SPI irq flag: SPI_SR_CRCERR\r\n");
    }
    else if (SPI_UNIT->SR & SPI_SR_OVR)
    {
        do
        {
            volatile uint16_t reg = SPI_UNIT->DR;
        }
        while (SPI_UNIT->SR & SPI_SR_OVR);
        BSP_PRINTF("<spi>" ERR_STR "SPI irq flag: SPI_SR_OVR\r\n");
    }
    else
    {
        BSP_PRINTF("<spi>" ERR_STR "DMA RX irq unknown\r\n");
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
    EXTI->RTSR |=  (GPIO_EXTI_LINE(SPI_PIN_NSS));
    EXTI->FTSR &= ~(GPIO_EXTI_LINE(SPI_PIN_NSS));
    ENABLE_IRQ(GPIO_IRQ_CHANNEL(SPI_PIN_NSS), RX_PRI);

    // DMA and SPI
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    CLR_PENDING(SPI1_IRQn);
    ENABLE_IRQ(SPI1_IRQn, TX_PRI);
    CLR_PENDING(DMA2_Stream2_IRQn); // RX
    ENABLE_IRQ(DMA2_Stream2_IRQn, TX_PRI);
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
    SPI_DMA_RX->NDTR  = sizeof(buf_rx[0]);
    SPI_DMA_RX->PAR   = (uint32_t)&(SPI_UNIT->DR);
    SPI_DMA_RX->M0AR  = (uint32_t)buf_rx[0];
    SPI_DMA_RX->M1AR  = (uint32_t)buf_rx[1];
    DMA2->LIFCR = 0x3FU << 0x10; // Clear all interrupt flags
    SPI_DMA_RX->CR   |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    SPI_DMA_RX->FCR  |= DMA_IT_FE;
    SPI_DMA_RX->CR   |= DMA_SxCR_EN; // Enable DMA
    SPI_UNIT->CR2    |= SPI_CR2_RXDMAEN;
    
    // Tx DMA init
    SPI_DMA_TX->CR   &= ~DMA_SxCR_EN; // Disable DMA
    while(SPI_DMA_TX->CR & DMA_SxCR_EN);
    SPI_DMA_TX->CR    = (SPI_DMA_TX_CH << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
    SPI_DMA_TX->FCR   = 0;
    SPI_DMA_TX->CR   &= ~DMA_SxCR_DBM;
    SPI_DMA_TX->PAR   = (uint32_t)&(SPI_UNIT->DR);
    DMA2->LIFCR = 0x3FU << 0x16; // Clear all interrupt flags
    SPI_DMA_TX->CR   |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    SPI_UNIT->CR2    |= SPI_CR2_TXDMAEN;

#ifdef DEBUG_BSP
    // Test code
    {
        static uint8_t data[] = {0x01, 0x0d, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0xa2};
        
        bsp_spi_tx(data);
    }
#endif
}

bool bsp_spi_tx(const uint8_t *const _data)
{
    if (SPI_DMA_TX->CR & DMA_SxCR_EN)
    {
        BSP_PRINTF("<spi> bsp_spi_tx false\r\n");
        return false;
    }
    
    SPI_DMA_TX->NDTR  = _data[LEN_PTR];
    SPI_DMA_TX->M0AR  = (uint32_t)_data;
    DMA2->LIFCR = 0x3FU << 0x16; // Clear all interrupt flags
    SPI_DMA_TX->CR   |= DMA_SxCR_EN; // Enable DMA

    BSP_PRINTF("<spi> bsp_spi_tx true\r\n");
    return true;
}

__WEAK void bsp_spi_tx_callback(const bool _ok)
{
    BSP_PRINTF("<spi> SPI TX complete callback. Result: %s\r\n", (_ok) ? "true" : "false");
}

__WEAK bool bsp_spi_rx_callback(uint8_t *const _data)
{
    BSP_PRINTF("<spi> SPI RX callback addr: %#08X, size: %d bytes\r\n", (uint32_t)_data, _data[LEN_PTR]);
    
    return true;
}

/**************************************************************************************************
 *                                        END OF FILE
 **************************************************************************************************/
