/******************************************************************************
 * @file    robot_port_baremetal.c
 * @brief   Bare-metal implementation of the execution environment port.
 *
 * @details The RAII-free mapping is intentionally trivial; a FreeRTOS port
 *          replaces this file only (see robot_port.h for the contract and
 *          Docs/02 for the full migration example).
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include "robot_port.h"
#include "stm32f7xx_hal.h"

/** @brief Enters a critical section by masking maskable interrupts. */
void ROBOT_PORT_critical_enter(void)
{
    __disable_irq();
}

/** @brief Leaves the critical section by restoring interrupt handling. */
void ROBOT_PORT_critical_exit(void)
{
    __enable_irq();
}

/** @brief Returns the HAL millisecond tick (TIM6 based, 1 kHz). */
uint32_t ROBOT_PORT_tick_ms(void)
{
    return HAL_GetTick();
}

/** @brief Blocking delay built on the HAL tick. */
void ROBOT_PORT_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/** @brief CPU idle: wait for interrupt. */
void ROBOT_PORT_idle(void)
{
    __WFI();
}
