/***************************************************************************************************
 *   Project:
 *   Author:       Stulov Tikhon
 ***************************************************************************************************
 *   Distribution:
 *
 ***************************************************************************************************
 *   MCU Family:    STM32Fx
 *   Compiler:      ARMCC
 ***************************************************************************************************
 *   File:          bsp_io_macro.h
 *   Description:
 *
 ***************************************************************************************************
 *   History:       26.03.2018 - [] - file created
 *
 **************************************************************************************************/

#pragma once

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

/***************************************************************************************************
 *                                           DEFINITIONS aliases of pins                           *
 **************************************************************************************************/

#define PORT_NA  0xFF

#define PORTA_00 0x00
#define PORTA_01 0x01
#define PORTA_02 0x02
#define PORTA_03 0x03
#define PORTA_04 0x04
#define PORTA_05 0x05
#define PORTA_06 0x06
#define PORTA_07 0x07
#define PORTA_08 0x08
#define PORTA_09 0x09
#define PORTA_10 0x0A
#define PORTA_11 0x0B
#define PORTA_12 0x0C
#define PORTA_13 0x0D
#define PORTA_14 0x0E
#define PORTA_15 0x0F

#define PORTB_00 0x10
#define PORTB_01 0x11
#define PORTB_02 0x12
#define PORTB_03 0x13
#define PORTB_04 0x14
#define PORTB_05 0x15
#define PORTB_06 0x16
#define PORTB_07 0x17
#define PORTB_08 0x18
#define PORTB_09 0x19
#define PORTB_10 0x1A
#define PORTB_11 0x1B
#define PORTB_12 0x1C
#define PORTB_13 0x1D
#define PORTB_14 0x1E
#define PORTB_15 0x1F

#define PORTC_00 0x20
#define PORTC_01 0x21
#define PORTC_02 0x22
#define PORTC_03 0x23
#define PORTC_04 0x24
#define PORTC_05 0x25
#define PORTC_06 0x26
#define PORTC_07 0x27
#define PORTC_08 0x28
#define PORTC_09 0x29
#define PORTC_10 0x2A
#define PORTC_11 0x2B
#define PORTC_12 0x2C
#define PORTC_13 0x2D
#define PORTC_14 0x2E
#define PORTC_15 0x2F

#define PORTD_00 0x30
#define PORTD_01 0x31
#define PORTD_02 0x32
#define PORTD_03 0x33
#define PORTD_04 0x34
#define PORTD_05 0x35
#define PORTD_06 0x36
#define PORTD_07 0x37
#define PORTD_08 0x38
#define PORTD_09 0x39
#define PORTD_10 0x3A
#define PORTD_11 0x3B
#define PORTD_12 0x3C
#define PORTD_13 0x3D
#define PORTD_14 0x3E
#define PORTD_15 0x3F

#define PORTE_00 0x40
#define PORTE_01 0x41
#define PORTE_02 0x42
#define PORTE_03 0x43
#define PORTE_04 0x44
#define PORTE_05 0x45
#define PORTE_06 0x46
#define PORTE_07 0x47
#define PORTE_08 0x48
#define PORTE_09 0x49
#define PORTE_10 0x4A
#define PORTE_11 0x4B
#define PORTE_12 0x4C
#define PORTE_13 0x4D
#define PORTE_14 0x4E
#define PORTE_15 0x4F

#define PORTF_00 0x50
#define PORTF_01 0x51
#define PORTF_02 0x52
#define PORTF_03 0x53
#define PORTF_04 0x54
#define PORTF_05 0x55
#define PORTF_06 0x56
#define PORTF_07 0x57
#define PORTF_08 0x58
#define PORTF_09 0x59
#define PORTF_10 0x5A
#define PORTF_11 0x5B
#define PORTF_12 0x5C
#define PORTF_13 0x5D
#define PORTF_14 0x5E
#define PORTF_15 0x5F

#define PORTG_00 0x60
#define PORTG_01 0x61
#define PORTG_02 0x62
#define PORTG_03 0x63
#define PORTG_04 0x64
#define PORTG_05 0x65
#define PORTG_06 0x66
#define PORTG_07 0x67
#define PORTG_08 0x68
#define PORTG_09 0x69
#define PORTG_10 0x6A
#define PORTG_11 0x6B
#define PORTG_12 0x6C
#define PORTG_13 0x6D
#define PORTG_14 0x6E
#define PORTG_15 0x6F

/***************************************************************************************************
 *                                           DEFINITIONS addreses, biases and soon calculation     *
 **************************************************************************************************/

#define GPIO_PORT_SOURCE(GPIO) (GPIO >> 4)
#define GPIO_PIN_SOURCE(GPIO) (GPIO & 0xF)

#define GPIO_PORT(GPIO) ((GPIO_TypeDef *)(GPIOA_BASE + 0x400 * GPIO_PORT_SOURCE(GPIO)))

#define GPIO_PIN(GPIO) (1U << GPIO_PIN_SOURCE(GPIO))

#define GPIO_PORT_RCC(GPIO) (1U << GPIO_PORT_SOURCE(GPIO))

#define GPIO_ADC_CHANNEL(GPIO) GPIO_PIN_SOURCE(GPIO)

#define GPIO_EXTI_LINE(GPIO) GPIO_PIN(GPIO)

#define GPIO_IRQ_CHANNEL(GPIO)                                \
    (                                                         \
            (GPIO_PIN_SOURCE(GPIO) == 0x0) ? EXTI0_IRQn     : \
            (GPIO_PIN_SOURCE(GPIO) == 0x1) ? EXTI1_IRQn     : \
            (GPIO_PIN_SOURCE(GPIO) == 0x2) ? EXTI2_IRQn     : \
            (GPIO_PIN_SOURCE(GPIO) == 0x3) ? EXTI3_IRQn     : \
            (GPIO_PIN_SOURCE(GPIO) == 0x4) ? EXTI4_IRQn     : \
            (GPIO_PIN_SOURCE(GPIO) == 0x5) ? EXTI9_5_IRQn   : \
            (GPIO_PIN_SOURCE(GPIO) == 0x6) ? EXTI9_5_IRQn   : \
            (GPIO_PIN_SOURCE(GPIO) == 0x7) ? EXTI9_5_IRQn   : \
            (GPIO_PIN_SOURCE(GPIO) == 0x8) ? EXTI9_5_IRQn   : \
            (GPIO_PIN_SOURCE(GPIO) == 0x9) ? EXTI9_5_IRQn   : \
            (GPIO_PIN_SOURCE(GPIO) == 0xA) ? EXTI15_10_IRQn : \
            (GPIO_PIN_SOURCE(GPIO) == 0xB) ? EXTI15_10_IRQn : \
            (GPIO_PIN_SOURCE(GPIO) == 0xC) ? EXTI15_10_IRQn : \
            (GPIO_PIN_SOURCE(GPIO) == 0xD) ? EXTI15_10_IRQn : \
            (GPIO_PIN_SOURCE(GPIO) == 0xE) ? EXTI15_10_IRQn : \
            EXTI15_10_IRQn                                    \
    )

#define GPIO_AF_SET(GPIO, AF_NUM, AF_MODE, PU, SPEED)                         \
    RCC->AHB1ENR |= GPIO_PORT_RCC(GPIO);                                      \
    GPIO_PORT(GPIO)->AFR[GPIO_PIN_SOURCE(GPIO) >> 3] |=                       \
        (AF_NUM) << ((GPIO_PIN_SOURCE(GPIO) & 7) * 4);                        \
    GPIO_PORT(GPIO)->MODER   &= ~(3U << (GPIO_PIN_SOURCE(GPIO) * 2));         \
    GPIO_PORT(GPIO)->MODER   |= (AF_MODE & 3) << (GPIO_PIN_SOURCE(GPIO) * 2); \
    GPIO_PORT(GPIO)->OTYPER  |= (AF_MODE >> 4) << GPIO_PIN_SOURCE(GPIO);      \
    GPIO_PORT(GPIO)->OSPEEDR |= SPEED << (GPIO_PIN_SOURCE(GPIO) * 2);         \
    GPIO_PORT(GPIO)->PUPDR   &= ~(3U << (GPIO_PIN_SOURCE(GPIO) * 2));         \
    GPIO_PORT(GPIO)->PUPDR   |= PU << (GPIO_PIN_SOURCE(GPIO) * 2);

#define CLR_PENDING(IRQn) NVIC->ICPR[(IRQn) >> 5] = (1U << ((IRQn) & 0x1F))

#define SET_IRQ_PRI(IRQn, PRI) NVIC->IP[IRQn] = PRI << 4U

#define ENABLE_IRQ(IRQn, PRI) NVIC->IP[IRQn] = PRI << 4U ;\
        NVIC->ISER[(IRQn) >> 5] = (1U << ((IRQn) & 0x1F));

/***************************************************************************************************
 *                                          PUBLIC TYPES                                           *
 **************************************************************************************************/

/***************************************************************************************************
 *                                         GLOBAL VARIABLES                                        *
 **************************************************************************************************/

/***************************************************************************************************
 *                                    PUBLIC FUNCTION PROTOTYPES                                   *
 **************************************************************************************************/
