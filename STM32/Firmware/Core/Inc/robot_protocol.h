/******************************************************************************
 * @file    robot_protocol.h
 * @brief   I2C protocol between the Raspberry Pi (master) and this STM32 board
 *          (slave address 0x0A): fixed 16 B request / 32 B response frames,
 *          CRC-8 protected and sequence-numbered for safe retries.
 *
 * @details Version 3: speeds are 16-bit mm/s (up to the 900 mm/s absolute
 *          limit), the board reports a full error/event log and its
 *          calibrated settings can be read, written, saved to flash and
 *          restored over the same frames.
 *
 *          Threading model:
 *          - PROTO_on_rx_complete_isr() and PROTO_copy_serving_isr() run in the
 *            I2C interrupt (highest priority). They validate, enqueue and
 *            snapshot only.
 *          - PROTO_task() runs in the main loop and executes the queued
 *            requests, building the responses.
 *
 * @note    Full wire specification (with examples and a Python client):
 *          Docs/03-i2c-protocol-spec.md.
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.5.0
 * @date    2026-10-19
 ******************************************************************************
 */
#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include <stdint.h>

/* ------------------------------- Frame sizes ------------------------------- */
#define PROTO_REQ_SIZE          (16U)   /**< RPi -> STM32 frame, fixed size.    */
#define PROTO_RSP_SIZE          (32U)   /**< STM32 -> RPi frame, fixed size.    */
#define PROTO_REQ_PAYLOAD_SIZE  (11U)   /**< Bytes 4..14 of the request frame.  */
#define PROTO_RSP_PAYLOAD_SIZE  (27U)   /**< Bytes 4..30 of the response frame. */
#define PROTO_RING_SIZE         (16U)   /**< Request ring, must be a power of 2.*/

/* --------------------------------- SOF bytes -------------------------------- */
#define PROTO_SOF_REQUEST       (0xA5U) /**< First byte of every request.  */
#define PROTO_SOF_RESPONSE      (0x5AU) /**< First byte of every response. */

/* ------------------------------ Command codes ------------------------------- */
#define PROTO_CMD_STOP                  (0x01U) /**< Controlled stop, pose kept.   */
#define PROTO_CMD_ESTOP                 (0x02U) /**< Immediate stop, latched fault.*/
#define PROTO_CMD_CLEAR_FAULTS          (0x03U) /**< Clear latched faults.         */
#define PROTO_CMD_SET_POSE              (0x04U) /**< x,y [0.1mm], theta [0.1deg].  */
#define PROTO_CMD_RESET_ODOMETRY        (0x05U) /**< Pose := (0,0,0).              */
#define PROTO_CMD_RELEASE_MOTORS        (0x06U) /**< Drivers off when IDLE only;
                                                     no fault latched; next move
                                                     re-energizes automatically. */
#define PROTO_CMD_DRIVE                 (0x10U) /**< d [mm i16], v [mm/s u16].     */
#define PROTO_CMD_TURN                  (0x11U) /**< angle [0.1deg i16], v [u16].  */
#define PROTO_CMD_GOTO                  (0x12U) /**< x,y [0.1mm], th, v [u16].     */
#define PROTO_CMD_STRAFE                (0x13U) /**< d [mm i16] +left, v [u16].    */
#define PROTO_CMD_SET_MOTION_LIMITS     (0x14U) /**< vmax, accel, decel (u16 x3).  */
#define PROTO_CMD_SET_DEADMAN           (0x15U) /**< Timeout [ms u16], 0 = off.    */
#define PROTO_CMD_ARC                   (0x16U) /**< r [mm i16], sweep [0.1deg] i16,
                                                     v [u16]; signs must agree.  */
#define PROTO_CMD_QUEUE_ARC             (0x17U) /**< Same payload as ARC; while a
                                                     segment runs it is queued and
                                                     blended into it without
                                                     stopping (arc chaining).   */
#define PROTO_CMD_SET_GYRO              (0x18U) /**< Silent gyro stream: i16 rate
                                                     [0.1 deg/s] + u8 age [10 ms]
                                                     units; executes WITHOUT a
                                                     published response and never
                                                     enters the retry window.   */
#define PROTO_CMD_SET_VELOCITY          (0x19U) /**< Silent velocity stream (fw
                                                     >= 3.4): i16 v [mm/s] +
                                                     i16 omega [0.1 deg/s];
                                                     latest-wins, executes
                                                     WITHOUT a published
                                                     response like SET_GYRO.    */
#define PROTO_CMD_QUERY_ODOMETRY        (0x20U) /**< Pose + speeds + state + faults. */
#define PROTO_CMD_QUERY_STATUS          (0x21U) /**< FW, uptime, faults, last error. */
#define PROTO_CMD_QUERY_TICKS           (0x22U) /**< Raw encoder accumulators.     */
#define PROTO_CMD_QUERY_EVENTS          (0x23U) /**< Error/event log since index.  */
#define PROTO_CMD_DISPLAY_MODE          (0x30U) /**< 0=off, 1=auto, 2=manual.      */
#define PROTO_CMD_DISPLAY_TEXT          (0x31U) /**< line, offset, up to 9 chars.  */
#define PROTO_CMD_GET_CONFIG            (0x40U) /**< offset u8, count u8 -> u16s.  */
#define PROTO_CMD_SET_CONFIG            (0x41U) /**< offset u8, count u8, u16s.    */
#define PROTO_CMD_SAVE_CONFIG           (0x42U) /**< Persist RAM settings to flash.*/
#define PROTO_CMD_LOAD_CONFIG           (0x43U) /**< Reload settings from flash.   */
#define PROTO_CMD_RESET_CONFIG          (0x44U) /**< Restore compile-time defaults.*/
#define PROTO_CMD_PING                  (0x7FU) /**< Echoes 4 payload bytes.       */

/** @brief Largest register run GET_CONFIG can return in one frame. */
#define PROTO_CFG_GET_MAX               (12U)

/* ------------------------------- Status codes ------------------------------- */
#define PROTO_STATUS_OK          (0x00U) /**< Executed / accepted.              */
#define PROTO_STATUS_BAD_CRC     (0x01U) /**< CRC failure, frame not executed.  */
#define PROTO_STATUS_BAD_FRAME   (0x02U) /**< Wrong SOF / framing error.        */
#define PROTO_STATUS_UNKNOWN_CMD (0x03U) /**< Command code not implemented.     */
#define PROTO_STATUS_BUSY        (0x04U) /**< A segment is active.              */
#define PROTO_STATUS_RANGE       (0x05U) /**< Parameter outside allowed range.  */
#define PROTO_STATUS_FAULT       (0x06U) /**< Latched fault, clear it first.    */
#define PROTO_STATUS_BAD_LEN     (0x07U) /**< Payload shorter than required.    */
#define PROTO_STATUS_STORAGE     (0x08U) /**< Flash storage operation failed.   */
#define PROTO_STATUS_NODATA      (0x7FU) /**< No request processed yet.         */

/* ------------------------------ Display modes ------------------------------- */
#define PROTO_DISPLAY_OFF       (0U) /**< Status screen disabled.              */
#define PROTO_DISPLAY_AUTO      (1U) /**< Board renders state / pose / speed.  */
#define PROTO_DISPLAY_MANUAL    (2U) /**< RPi supplies text via DISPLAY_TEXT.  */

/* ---------------------------------- API ------------------------------------- */

/** @brief  One-time initialization: clears the ring, builds the placeholder
 *          response frame and sets the default display mode. */
void PROTO_init(void);

/** @brief  @return Pointer to the 16-byte reception buffer that the I2C driver
 *          fills. Used by the HAL_I2C_AddrCallback (write direction). */
uint8_t *PROTO_get_rx_buffer(void);

/** @brief  @return Pointer to the constant "serving" copy of the latest
 *          response frame, transmitted when the master reads. */
const uint8_t *PROTO_get_serving_buffer(void);

/** @brief  I2C RX-complete entry point (ISR context): validates SOF and CRC-8,
 *          applies immediate safety commands (STOP/ESTOP) and enqueues the
 *          frame for PROTO_task(). Invalid frames are answered with an error
 *          status but never poison the duplicate-sequence memory. */
void PROTO_on_rx_complete_isr(void);

/** @brief  I2C address-match entry point (ISR context): copies the latest
 *          response into the serving buffer before the transmit is armed. */
void PROTO_copy_serving_isr(void);

/** @brief  Records I2C bus activity; called from every I2C callback. */
void PROTO_note_bus_activity(void);

/** @brief  @return 1 when an I2C transfer used the bus within the last few ms. */
uint8_t PROTO_bus_is_active(void);

/** @brief  @return Milliseconds elapsed since the last I2C bus activity.
 *  @param  now_ms Current HAL tick value. */
uint32_t PROTO_ms_since_bus_activity(uint32_t now_ms);

/** @brief  @return 1 when at least one received request is waiting for
 *          execution in the ring buffer. */
uint8_t PROTO_has_pending_requests(void);

/** @brief  Main-loop worker: executes every queued request, builds and
 *          publishes the response frames; also runs the deadman check. */
void PROTO_task(void);

/** @brief  @return Current display mode (PROTO_DISPLAY_*). */
uint8_t PROTO_get_display_mode(void);

/** @brief  @return Text line buffer supplied by the RPi via DISPLAY_TEXT
 *          (never NULL).
 *  @param  line_idx Line index 0..ROBOT_OLED_LINE_COUNT-1. */
const char *PROTO_get_display_line(uint8_t line_idx);

/** @brief  @return Tick of the last accepted DISPLAY_TEXT (0 = none yet).
 *          Used by the display task to run the message hold window. */
uint32_t PROTO_last_message_ms(void);

/** @brief  @return Number of accepted DISPLAY_TEXT frames (0 = none yet).
 *          The display task redraws only when this counter changes. */
uint32_t PROTO_message_seq(void);

#endif /* ROBOT_PROTOCOL_H */
