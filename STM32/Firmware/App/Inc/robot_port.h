/******************************************************************************
 * @file    robot_port.h
 * @brief   Execution environment portability layer: critical sections, time
 *          base, blocking delay and idle wait.
 *
 * @details Every application module uses only these primitives, so switching
 *          the firmware between bare metal and an RTOS (FreeRTOS) means
 *          implementing this interface once:
 *
 *          Bare metal (current):  App/Src/robot_port_baremetal.c
 *              critical : __disable_irq / __enable_irq
 *              tick     : HAL_GetTick()
 *              delay    : HAL_Delay()
 *              idle     : __WFI()
 *
 *          FreeRTOS (future):     e.g. App/Src/robot_port_freertos.c
 *              critical : taskENTER_CRITICAL / taskEXIT_CRITICAL
 *              tick     : xTaskGetTickCount()
 *              delay    : vTaskDelay(pdMS_TO_TICKS(...))
 *              idle     : taskYIELD()
 *          (see Docs/02-firmware-architecture.md, section "RTOS porting").
 *
 * @note    Critical sections are only entered from task/main context in this
 *          design - never from an interrupt - which keeps both mappings legal.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_PORT_H
#define ROBOT_PORT_H

#include <stdint.h>

/** @brief Enters a short critical section (task context only). */
void ROBOT_PORT_critical_enter(void);

/** @brief Leaves the critical section entered by ROBOT_PORT_critical_enter. */
void ROBOT_PORT_critical_exit(void);

/** @brief Returns the free running millisecond time base. */
uint32_t ROBOT_PORT_tick_ms(void);

/** @brief Blocking delay in milliseconds (task context only). */
void ROBOT_PORT_delay_ms(uint32_t delay_ms);

/** @brief Idle wait: sleeps the CPU until the next interrupt. */
void ROBOT_PORT_idle(void);

#endif /* ROBOT_PORT_H */
