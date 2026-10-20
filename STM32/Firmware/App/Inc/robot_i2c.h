/******************************************************************************
 * @file    robot_i2c.h
 * @brief   I2C bus manager: slave listen mode, master windows for the OLED,
 *          bus error recovery and the HAL I2C callbacks.
 *
 * @details The board is an I2C slave for the Raspberry Pi (address 0x0A) and
 *          a temporary master for the SSD1306 display on the SAME bus.
 *          Listen mode is the default state; the display task acquires the bus
 *          for a refresh and releases it afterwards. All functions run in
 *          task/main context except the HAL callbacks, which run in the I2C
 *          interrupt and only touch the protocol ISR interface.
 *
 *          Robustness contract: whatever error happens (master or slave,
 *          timeout, arbitration lost, stuck bus), the manager ALWAYS brings
 *          the peripheral back to slave listen mode, so the Raspberry Pi can
 *          never permanently lose control of the board.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.1.0
 * @date    2026-09-30
 ******************************************************************************
 */
#ifndef ROBOT_I2C_H
#define ROBOT_I2C_H

#include <stdint.h>

/** @brief (Re-)arms the I2C slave listen mode; flags a recovery on failure. */
void ROBOT_I2C_arm_listen(void);

/** @brief Services a deferred bus error: DeInit/Init cycle, then listen again.
 *  @note  Called from the communication task, retried until it succeeds. */
void ROBOT_I2C_service_recovery(void);

/** @brief  Acquires the bus for a display master transfer.
 *  @return 1 when the transfer may start (listen disabled, bus quiet and idle),
 *          0 when the RPi won the race or the bus is not usable
 *          (listen mode stays or is restored - the refresh is simply aborted).
 *  @note   Only call when no request is pending and the bus has been quiet for
 *          ROBOT_I2C_BUS_QUIET_MS (the display task already enforces this). */
uint8_t ROBOT_I2C_display_bus_acquire(void);

/** @brief Releases the bus after a successful display transfer (listen mode). */
void ROBOT_I2C_display_bus_release(void);

/** @brief Reports a FAILED display transfer: the peripheral may be stuck BUSY,
 *         so a full DeInit/Init cycle is scheduled before listening again. */
void ROBOT_I2C_display_bus_fault(void);

#endif /* ROBOT_I2C_H */
