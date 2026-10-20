/**
 ******************************************************************************
 * @file    robot_events.h
 * @brief   Firmware error/event log: every failure is recorded with a code
 *          from the list below and can be read back by the Raspberry Pi
 *          (PROTO_CMD_QUERY_EVENTS), so the service always knows WHAT went
 *          wrong instead of just "an error".
 *
 * @details The code list is part of the wire contract: it is mirrored in
 *          Docs/03-i2c-protocol-spec.md and in the RPi service
 *          (backend/app/robot/protocol.py). Keep the three in sync.
 *
 *          Severity convention (used by the RPi service for log levels):
 *            0x01..0x0D = error / warning   (something failed)
 *            0x0E..0x10 = informational     (config life-cycle)
 *            0x11..0x14 = boot cause notice (recorded once at startup)
 *
 * @note    Recording is ISR-safe (I2C and EXTI callbacks use it); the ring
 *          keeps the newest ROBOT_EVENT_RING_SIZE events.
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_EVENTS_H
#define ROBOT_EVENTS_H

#include <stdint.h>

/* ------------------------------- Event codes -------------------------------- */
#define ROBOT_EVENT_NONE            (0x00U) /**< never stored in the ring      */
#define ROBOT_EVENT_ESTOP           (0x01U) /**< aux: 0                        */
#define ROBOT_EVENT_LIMIT           (0x02U) /**< aux: limit switch bit mask    */
#define ROBOT_EVENT_STALL           (0x03U) /**< aux: motion state that stalled */
#define ROBOT_EVENT_TIMEOUT         (0x04U) /**< aux: motion state that timed out */
#define ROBOT_EVENT_DEADMAN         (0x05U) /**< aux: 0                        */
#define ROBOT_EVENT_BAD_CRC         (0x06U) /**< aux: 0                        */
#define ROBOT_EVENT_BAD_FRAME       (0x07U) /**< aux: offending byte or 0      */
#define ROBOT_EVENT_UNKNOWN_CMD     (0x08U) /**< aux: command code             */
#define ROBOT_EVENT_BAD_PARAM       (0x09U) /**< aux: command code             */
#define ROBOT_EVENT_BUSY_REJECT     (0x0AU) /**< aux: command code             */
#define ROBOT_EVENT_I2C_ERROR       (0x0BU) /**< aux: HAL I2C error code       */
#define ROBOT_EVENT_FLASH_ERROR     (0x0CU) /**< aux: 1 erase, 2 program, 3 verify */
#define ROBOT_EVENT_CONFIG_FALLBACK (0x0DU) /**< aux: 1 magic, 2 version, 3 crc, 4 read */
#define ROBOT_EVENT_CONFIG_SAVED    (0x0EU) /**< aux: 0                        */
#define ROBOT_EVENT_CONFIG_APPLIED  (0x0FU) /**< aux: 0                        */
#define ROBOT_EVENT_CONFIG_DEFAULTS (0x10U) /**< aux: 0                        */
#define ROBOT_EVENT_RESET_IWDG      (0x11U) /**< boot: watchdog reset          */
#define ROBOT_EVENT_RESET_POR       (0x12U) /**< boot: power-on/pin reset      */
#define ROBOT_EVENT_RESET_SW        (0x13U) /**< boot: software reset          */
#define ROBOT_EVENT_RESET_OTHER     (0x14U) /**< boot: brown-out/other reset   */
#define ROBOT_EVENT_BAD_LEN         (0x15U) /**< aux: command code             */
#define ROBOT_EVENT_I2C_RECOVER     (0x16U) /**< aux: 1 = stuck bus was cleared */

/** @brief One recorded event: monotonic index, code and context byte. */
typedef struct
{
    uint16_t index; /**< 1-based monotonic index (stable across reads).        */
    uint8_t code;   /**< ROBOT_EVENT_* code.                                   */
    uint8_t aux;    /**< context: command, mask or reason - see the list.      */
} ROBOT_EVENT_RECORD;

/** @brief Clears the log and records the boot cause (call once at startup,
 *         before any subsystem can record anything else). */
void ROBOT_EVENTS_init(void);

/** @brief Records one event (ISR-safe; oldest events are overwritten). */
void ROBOT_EVENTS_record(uint8_t code, uint8_t aux);

/** @brief @return Total number of events recorded since boot. */
uint16_t ROBOT_EVENTS_count(void);

/** @brief @return Index of the newest recorded event (0 when none yet). */
uint16_t ROBOT_EVENTS_latest_index(void);

/** @brief Copies the events newer than @p after_index, oldest first.
 *  @param after_index  Read everything with index > after_index.
 *  @param out          Destination array (must hold @p max entries).
 *  @param max          Maximum number of records to copy.
 *  @return Number of records copied. */
uint8_t ROBOT_EVENTS_read(uint16_t after_index, ROBOT_EVENT_RECORD *out, uint8_t max);

/** @brief @return Code of the newest recorded event (0 = none). */
uint8_t ROBOT_EVENTS_last_code(void);

/** @brief @return Aux byte of the newest recorded event. */
uint8_t ROBOT_EVENTS_last_aux(void);

/** @brief @return Boot cause code (ROBOT_EVENT_RESET_*) for the status frame. */
uint8_t ROBOT_EVENTS_reset_cause(void);

#endif /* ROBOT_EVENTS_H */
