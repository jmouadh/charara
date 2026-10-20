/******************************************************************************
 * @file    robot_i2c.c
 * @brief   I2C bus manager implementation and HAL I2C callbacks.
 *
 * @details Bus arbitration strategy (hardware is fixed, one shared bus):
 *          - Default state: slave listen mode enabled (RPi can access anytime).
 *          - Display refresh: listen is disabled only when no transfer has
 *            used the bus for ROBOT_I2C_BUS_QUIET_MS (checked by the display
 *            task); the RPi may still lose a window and must retry - the
 *            protocol sequence numbers make retries safe.
 *          - Errors: never recover inside the ISR; a flag defers a full
 *            DeInit / bus-clear / Init cycle to ROBOT_I2C_service_recovery().
 *            Whatever failed (master OLED window or slave transfer), the
 *            peripheral is ALWAYS brought back to slave listen mode, so the
 *            RPi can never permanently lose control of the board.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.1.0
 * @date    2026-09-30
 ******************************************************************************
 */
#include "robot_i2c.h"
#include "robot_board.h"
#include "robot_protocol.h"
#include "robot_events.h"
#include "robot_port.h"
#include "robot_configs.h"

/** @brief I2C1 pin map (PB7 = SDA, PB8 = SCL; see stm32f7xx_hal_msp.c). */
#define ROBOT_I2C_PORT      GPIOB
#define ROBOT_I2C_SDA_PIN   GPIO_PIN_7
#define ROBOT_I2C_SCL_PIN   GPIO_PIN_8
#define ROBOT_I2C_GPIO_AF   GPIO_AF4_I2C1

/** @brief Deferred recovery request set by any error path (ISR safe). */
static volatile uint8_t s_recover_req = 0U;

/** @brief Earliest tick of the next recovery attempt (throttles retries). */
static uint32_t s_recover_next_ms = 0U;

/* ------------------------------- bus helpers -------------------------------- */

/** @brief @return 1 when both lines are released (bus idle). The pin level can
 *         be read even while the pins are in alternate-function mode. */
static uint8_t bus_lines_idle(void)
{
    return ((HAL_GPIO_ReadPin(ROBOT_I2C_PORT, ROBOT_I2C_SCL_PIN) == GPIO_PIN_SET) &&
            (HAL_GPIO_ReadPin(ROBOT_I2C_PORT, ROBOT_I2C_SDA_PIN) == GPIO_PIN_SET)) ? 1U : 0U;
}

/** @brief Frees a bus stuck low by a slave (9 SCL pulses + STOP) and restores
 *         the I2C alternate function.
 *  @note  Must run with the peripheral disabled.
 *  @return 1 when the recovery pulses were actually issued, 0 when the bus was
 *          already idle. */
static uint8_t bus_clear(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t pulse;
    uint8_t cleared = 0U;

    HAL_GPIO_DeInit(ROBOT_I2C_PORT, ROBOT_I2C_SDA_PIN | ROBOT_I2C_SCL_PIN);
    gpio.Pin = ROBOT_I2C_SDA_PIN | ROBOT_I2C_SCL_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ROBOT_I2C_PORT, &gpio);
    HAL_GPIO_WritePin(ROBOT_I2C_PORT, ROBOT_I2C_SDA_PIN | ROBOT_I2C_SCL_PIN, GPIO_PIN_SET);

    if (bus_lines_idle() == 0U)
    {
        cleared = 1U;
        for (pulse = 0U; pulse < (uint8_t)ROBOT_I2C_BUS_CLEAR_CLOCKS; pulse++)
        {
            HAL_GPIO_WritePin(ROBOT_I2C_PORT, ROBOT_I2C_SCL_PIN, GPIO_PIN_RESET);
            ROBOT_PORT_delay_ms(ROBOT_I2C_BUS_CLEAR_DELAY_MS);
            HAL_GPIO_WritePin(ROBOT_I2C_PORT, ROBOT_I2C_SCL_PIN, GPIO_PIN_SET);
            ROBOT_PORT_delay_ms(ROBOT_I2C_BUS_CLEAR_DELAY_MS);
        }
        /* STOP condition: SDA low while SCL is high, then SDA released. */
        HAL_GPIO_WritePin(ROBOT_I2C_PORT, ROBOT_I2C_SDA_PIN, GPIO_PIN_RESET);
        ROBOT_PORT_delay_ms(ROBOT_I2C_BUS_CLEAR_DELAY_MS);
        HAL_GPIO_WritePin(ROBOT_I2C_PORT, ROBOT_I2C_SCL_PIN, GPIO_PIN_SET);
        ROBOT_PORT_delay_ms(ROBOT_I2C_BUS_CLEAR_DELAY_MS);
        HAL_GPIO_WritePin(ROBOT_I2C_PORT, ROBOT_I2C_SDA_PIN, GPIO_PIN_SET);
        ROBOT_PORT_delay_ms(ROBOT_I2C_BUS_CLEAR_DELAY_MS);
    }

    /* Hand the pins back to the I2C peripheral. */
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Alternate = ROBOT_I2C_GPIO_AF;
    HAL_GPIO_Init(ROBOT_I2C_PORT, &gpio);
    return cleared;
}

/** @brief (Re-)arms the slave listen mode; failure schedules a recovery. */
void ROBOT_I2C_arm_listen(void)
{
    if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
    {
        s_recover_req = 1U;
    }
}

/** @brief DeInit / bus-clear / Init cycle triggered by a bus error, then listen
 *         again. Throttled so a dead bus can never stall the main loop.
 *  @note  Runs in task context (ROBOT_APP_task_comms). */
void ROBOT_I2C_service_recovery(void)
{
    uint32_t now_ms;
    uint8_t cleared;

    if (s_recover_req == 0U)
    {
        return;
    }
    now_ms = ROBOT_PORT_tick_ms();
    if ((int32_t)(now_ms - s_recover_next_ms) < 0)
    {
        return; /* retry pacing: keep the loop (and the IWDG) alive */
    }
    s_recover_next_ms = now_ms + (uint32_t)ROBOT_I2C_RECOVERY_RETRY_MS;

    (void)HAL_I2C_DeInit(&hi2c1); /* MspDeInit: releases pins, clock and IRQs */
    cleared = bus_clear();        /* free a stuck bus, then restore the AF pins */
    ROBOT_EVENTS_record(ROBOT_EVENT_I2C_RECOVER, cleared);

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        return; /* keep the request armed and retry on a later pass */
    }
    (void)HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U);
    s_recover_req = 0U;
    ROBOT_I2C_arm_listen(); /* back to slave mode as fast as possible */
}

/** @brief Takes the bus for a display refresh only when it is truly free. */
uint8_t ROBOT_I2C_display_bus_acquire(void)
{
    /* Serve the RPi first: keep listening while a request is queued. */
    if (PROTO_has_pending_requests() != 0U)
    {
        return 0U;
    }
    /* A transfer is running or the RPi is about to start one. */
    if (PROTO_bus_is_active() != 0U)
    {
        return 0U;
    }
    /* Lines stuck low: free the bus instead of racing a broken transfer. */
    if (bus_lines_idle() == 0U)
    {
        s_recover_req = 1U;
        return 0U;
    }
    if (HAL_I2C_DisableListen_IT(&hi2c1) != HAL_OK)
    {
        s_recover_req = 1U;
        return 0U;
    }
    /* A request may have landed while listen was being disabled: give it back. */
    if (PROTO_has_pending_requests() != 0U)
    {
        ROBOT_I2C_arm_listen();
        return 0U;
    }
    return 1U;
}

/** @brief Gives the bus back to the RPi after a successful display refresh. */
void ROBOT_I2C_display_bus_release(void)
{
    ROBOT_I2C_arm_listen();
}

/** @brief Reports a failed display transfer: the peripheral may be stuck BUSY,
 *         so force a full DeInit/Init cycle before listening again. */
void ROBOT_I2C_display_bus_fault(void)
{
    s_recover_req = 1U;
}

/* ======================= I2C listen-mode callbacks ======================= */

/** @brief Address matched: arm a 16 B receive (write) or 32 B transmit (read). */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    UNUSED(AddrMatchCode);
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    PROTO_note_bus_activity();

    if (TransferDirection == I2C_DIRECTION_TRANSMIT)
    {
        /* Master writes a request frame. */
        if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, PROTO_get_rx_buffer(), PROTO_REQ_SIZE, I2C_LAST_FRAME) != HAL_OK)
        {
            s_recover_req = 1U; /* cannot arm the receive: reset and listen */
        }
    }
    else
    {
        /* Master reads the latest response frame. */
        PROTO_copy_serving_isr();
        if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)PROTO_get_serving_buffer(), PROTO_RSP_SIZE, I2C_LAST_FRAME) != HAL_OK)
        {
            s_recover_req = 1U; /* cannot arm the transmit: reset and listen */
        }
    }
}

/** @brief Request frame received: validation, safety flags and queueing. */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    PROTO_on_rx_complete_isr();
}

/** @brief Response frame fully transmitted; nothing further to do. */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    /* The RPi received its 32-byte frame. */
}

/** @brief Listen mode finished: re-arm it (recovery on failure). */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    PROTO_note_bus_activity();
    ROBOT_I2C_arm_listen();
}

/** @brief Bus error: defer the DeInit/bus-clear/Init recovery to task context
 *         and record the HAL error code in the event log (low byte: the bitmask
 *         flags BERR/ARLO/AF/OVR/DMA/TIMEOUT). */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    ROBOT_EVENTS_record(ROBOT_EVENT_I2C_ERROR, (uint8_t)(hi2c->ErrorCode & 0xFFU));
    s_recover_req = 1U;
}
