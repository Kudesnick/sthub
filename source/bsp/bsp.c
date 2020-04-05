/**
 *  @file       bsp.c
 *
 *  @brief      
 *
 *  @details
 *
 *  @author     Stulov Tikhon
 *
 *  @date       2020/07/03
 *
 *  @warning
 *
 *  @todo
 *
 */

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

#include <stdio.h>
#include "stm32f4xx_hal.h" // Device header
#include "bsp.h"

/***************************************************************************************************
 *                                           DEFINITIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                                          PRIVATE TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                                           PRIVATE DATA
 **************************************************************************************************/

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

/**
  * @brief   System Clock Configuration
  * @details core clock - 100 MHz
  *          clock source - external
  *          USB clock source - PLLI2S
  */
void SystemClock_Config(void)
{
    // Configure the main internal regulator output voltage
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    PWR->CR |= PWR_CR_VOS;
    
    uint32_t reg = SCB->AIRCR & ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk);
    reg |= (0x5FAU << SCB_AIRCR_VECTKEY_Pos) | (NVIC_PRIORITYGROUP_4 << SCB_AIRCR_PRIGROUP_Pos);
    SCB->AIRCR = reg;
    
    // Select HSE freq source
#if defined(HSE_BYPASS)
    RCC->CR |= RCC_HSE_BYPASS;
#elif !defined(HSE_ON)
#error HSE Source not configure
#endif
    RCC->CR |= RCC_CR_HSEON;
    while((RCC->CR & RCC_CR_HSERDY) == 0);
    
    // configure PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) != 0);
    
    RCC->PLLCFGR =
        RCC_PLLSOURCE_HSE                                      |
        (HSE_VALUE / 1000000 /* PLLM */)                       |
        (400 << RCC_PLLCFGR_PLLN_Pos)                          |
        (((RCC_PLLP_DIV4 >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) |
        (8 << RCC_PLLCFGR_PLLQ_Pos)                            |
        (2 << RCC_PLLCFGR_PLLR_Pos);
    
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Initializes the CPU, AHB and APB busses clocks
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_LATENCY_4;
    
    RCC->CFGR &= ~(RCC_CFGR_PPRE1       | RCC_CFGR_PPRE2       | RCC_CFGR_HPRE);
    RCC->CFGR |=   RCC_CFGR_PPRE1_DIV16 | RCC_CFGR_PPRE2_DIV16 | RCC_CFGR_HPRE_DIV1;
    
    while((RCC->CR & RCC_CR_HSERDY) == 0);
    while((RCC->CR & RCC_CR_PLLRDY) == 0);
    
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_SYSCLKSOURCE_PLLCLK;
    while((RCC->CFGR & RCC_CFGR_SWS) != (RCC_SYSCLKSOURCE_PLLCLK << RCC_CFGR_SWS_Pos));
    
    RCC->CFGR &= ~(RCC_CFGR_PPRE1       | RCC_CFGR_PPRE2);
    RCC->CFGR |=   RCC_CFGR_PPRE1_DIV2  | RCC_CFGR_PPRE2_DIV1;

    // USB clock
    RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_CK48MSEL;
    RCC->DCKCFGR2 |= RCC_PERIPHCLK_CLK48;
    
    RCC->CR &= ~RCC_CR_PLLI2SON;
    while((RCC->CR & RCC_CR_PLLI2SRDY) != 0);

    RCC->PLLI2SCFGR = RCC_PLLI2SCLKSOURCE_PLLSRC;
    RCC->PLLI2SCFGR =
        (HSE_VALUE / 2000000)              /* PLLI2SM */ |
        (96 << RCC_PLLI2SCFGR_PLLI2SN_Pos) /* PLLI2SN */ |
        RCC_PLLI2SCFGR_PLLI2SQ_2           /* PLLI2SQ */ |
        RCC_PLLI2SCFGR_PLLI2SR_1           /* PLLI2SR */ ;

    RCC->CR |= RCC_CR_PLLI2SON;
    while((RCC->CR & RCC_CR_PLLI2SRDY) == 0);
}

void SysTick_Handler(void)
{
    __NOP();
}

/***************************************************************************************************
 *                                      PUBLIC FUNCTIONS
 **************************************************************************************************/

void bsp_init(void)
{
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        ((NVIC_PRIORITYGROUP_4 & 7) << SCB_AIRCR_PRIGROUP_Pos);
    
    SystemClock_Config();
}

/**************************************************************************************************
 *                                        END OF FILE
 **************************************************************************************************/
