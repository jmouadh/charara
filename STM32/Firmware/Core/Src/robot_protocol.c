/**
 ******************************************************************************
 * @file    robot_protocol.c
 * @brief   I2C slave protocol engine (Raspberry Pi <-> STM32).
 *
 * Wire format (see Docs/03-i2c-protocol-spec.md for the full spec):
 *
 *   Request  (16 B): | A5 | seq | cmd | len | payload[11] | crc8 |
 *   Response (32 B): | 5A | seq | status | len | payload[27] | crc8 |
 *
 * CRC-8/ATM: polynomial 0x07, init 0x00, no reflection.
 * The I2C ISR only validates frames, handles immediate safety commands
 * (STOP / E-STOP) and enqueues work. Execution and response building happen
 * in PROTO_task() from the main loop.
 *
 * MISRA C:2012 orientation: no dynamic memory, bounded loops, explicit casts,
 * every switch has a default, no magic numbers.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include <string.h>
#include "robot_protocol.h"
#include "main.h"
#include "robot_port.h"
#include "robot_configs.h"
#include "robot_events.h"
#include "robot_settings.h"
#include "motion.h"
#include "ENCODERS_MouadhJeridi.h"

#define PROTO_RING_MASK       (PROTO_RING_SIZE - 1U)
#define PROTO_PAYLOAD_OFFSET  (4U)
#define PROTO_CRC_BYTE        (PROTO_REQ_SIZE - 1U)
#define PROTO_RSP_CRC_BYTE    (PROTO_RSP_SIZE - 1U)
#define PROTO_DISPLAY_CHARS_PER_FRAME (9U)
#define PROTO_BUS_ACTIVE_MS   (5U)
#define PROTO_SEQ_HISTORY     (4U)   /**< Executed-seq retry window depth. */

typedef struct
{
    uint8_t frame[PROTO_REQ_SIZE];
    uint8_t rx_status;
} PROTO_RX_ITEM;

static PROTO_RX_ITEM s_ring[PROTO_RING_SIZE];
static volatile uint8_t s_ring_head;
static volatile uint8_t s_ring_tail;

static uint8_t s_rx_buf[PROTO_REQ_SIZE];
static uint8_t s_serving[PROTO_RSP_SIZE];
static uint8_t s_active[PROTO_RSP_SIZE];
static uint8_t s_stage[PROTO_RSP_SIZE];
static uint8_t s_stage_pos;

static uint8_t s_seq_history[PROTO_SEQ_HISTORY];
static uint8_t s_seq_history_count;
static uint8_t s_last_seq;
static uint8_t s_last_status;

static volatile uint32_t s_bus_activity_ms;
static uint8_t s_display_mode;
static uint32_t s_last_message_ms;  /**< tick of the last accepted DISPLAY_TEXT.  */
static uint32_t s_message_seq;      /**< number of accepted DISPLAY_TEXT frames.  */
static char s_lines[ROBOT_OLED_LINE_COUNT][ROBOT_OLED_LINE_LEN];

/** @brief Returns 1 when the sequence number was already executed recently. */
static uint8_t proto_seq_is_duplicate(uint8_t seq)
{
    uint8_t index;

    for (index = 0U; index < s_seq_history_count; index++)
    {
        if (s_seq_history[index] == seq)
        {
            return 1U;
        }
    }
    return 0U;
}

/** @brief Records an executed sequence number in the retry history window. */
static void proto_seq_remember(uint8_t seq)
{
    uint8_t index;

    if (s_seq_history_count < PROTO_SEQ_HISTORY)
    {
        s_seq_history[s_seq_history_count] = seq;
        s_seq_history_count++;
    }
    else
    {
        for (index = 1U; index < PROTO_SEQ_HISTORY; index++)
        {
            s_seq_history[index - 1U] = s_seq_history[index];
        }
        s_seq_history[PROTO_SEQ_HISTORY - 1U] = seq;
    }
}

/* --------------------------------- CRC-8 ------------------------------------ */
/** @brief CRC-8/ATM (polynomial 0x07, init 0x00) over a byte buffer. */
static uint8_t proto_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0U;
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < len; index++)
    {
        crc = (uint8_t)(crc ^ data[index]);
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((uint8_t)(crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

/* ------------------------------ LE accessors -------------------------------- */
/** @brief Reads one unsigned byte from a little-endian payload. */
static uint8_t proto_get_u8(const uint8_t *p, uint8_t offset)
{
    return p[offset];
}

/** @brief Reads a little-endian signed 16-bit value from a payload. */
static int16_t proto_get_i16(const uint8_t *p, uint8_t offset)
{
    uint16_t raw = (uint16_t)p[offset] | (uint16_t)((uint16_t)p[offset + 1U] << 8);
    return (int16_t)raw;
}

/** @brief Reads a little-endian unsigned 16-bit value from a payload. */
static uint16_t proto_get_u16(const uint8_t *p, uint8_t offset)
{
    return (uint16_t)((uint16_t)p[offset] | (uint16_t)((uint16_t)p[offset + 1U] << 8));
}

/** @brief Reads a little-endian unsigned 32-bit value from a payload. */
static uint32_t proto_get_u32(const uint8_t *p, uint8_t offset)
{
    return (uint32_t)p[offset]
         | ((uint32_t)p[offset + 1U] << 8)
         | ((uint32_t)p[offset + 2U] << 16)
         | ((uint32_t)p[offset + 3U] << 24);
}

/* ------------------------------ stage builders ------------------------------ */
/** @brief Starts a new response frame (SOF, seq, zeroed payload). */
static void proto_rsp_begin(uint8_t seq)
{
    (void)memset(s_stage, 0, sizeof(s_stage));
    s_stage[0] = PROTO_SOF_RESPONSE;
    s_stage[1] = seq;
    s_stage[2] = PROTO_STATUS_NODATA;
    s_stage[3] = 0U;
    s_stage_pos = 0U;
}

/** @brief Stores the status byte of the response under construction. */
static void proto_rsp_set_status(uint8_t status)
{
    s_stage[2] = status;
}

/** @brief Appends one byte to the response payload (bounded). */
static void proto_rsp_put_u8(uint8_t value)
{
    if ((uint32_t)s_stage_pos < (uint32_t)PROTO_RSP_PAYLOAD_SIZE)
    {
        s_stage[PROTO_PAYLOAD_OFFSET + s_stage_pos] = value;
        s_stage_pos++;
    }
}

/** @brief Appends a signed 16-bit value (little-endian) to the payload. */
static void proto_rsp_put_i16(int16_t value)
{
    uint16_t raw = (uint16_t)value;
    proto_rsp_put_u8((uint8_t)(raw & 0xFFU));
    proto_rsp_put_u8((uint8_t)((raw >> 8) & 0xFFU));
}

/** @brief Appends an unsigned 16-bit value (little-endian) to the payload. */
static void proto_rsp_put_u16(uint16_t value)
{
    proto_rsp_put_u8((uint8_t)(value & 0xFFU));
    proto_rsp_put_u8((uint8_t)((value >> 8) & 0xFFU));
}

/** @brief Appends a signed 32-bit value (little-endian) to the payload. */
static void proto_rsp_put_i32(int32_t value)
{
    uint32_t raw = (uint32_t)value;
    proto_rsp_put_u8((uint8_t)(raw & 0xFFU));
    proto_rsp_put_u8((uint8_t)((raw >> 8) & 0xFFU));
    proto_rsp_put_u8((uint8_t)((raw >> 16) & 0xFFU));
    proto_rsp_put_u8((uint8_t)((raw >> 24) & 0xFFU));
}

/** @brief Appends an unsigned 32-bit value (little-endian) to the payload. */
static void proto_rsp_put_u32(uint32_t value)
{
    proto_rsp_put_u8((uint8_t)(value & 0xFFU));
    proto_rsp_put_u8((uint8_t)((value >> 8) & 0xFFU));
    proto_rsp_put_u8((uint8_t)((value >> 16) & 0xFFU));
    proto_rsp_put_u8((uint8_t)((value >> 24) & 0xFFU));
}

/** @brief Completes the response: sets the payload length and the CRC byte. */
static void proto_rsp_finalize(void)
{
    uint8_t crc;

    s_stage[3] = s_stage_pos;
    crc = proto_crc8(s_stage, (uint32_t)PROTO_RSP_CRC_BYTE);
    s_stage[PROTO_RSP_CRC_BYTE] = crc;
}

/** @brief Publishes the staged response as the active frame (critical section). */
static void proto_publish(void)
{
    ROBOT_PORT_critical_enter();
    (void)memcpy(s_active, s_stage, PROTO_RSP_SIZE);
    ROBOT_PORT_critical_exit();
}

/* ------------------------------ command execution ---------------------------- */
/** @brief Payload bytes each command needs before it may execute (0 = any). */
static uint8_t proto_required_len(uint8_t cmd)
{
    uint8_t need;

    switch (cmd)
    {
        case PROTO_CMD_SET_POSE:          need = 6U;  break;
        case PROTO_CMD_DRIVE:             need = 4U;  break;
        case PROTO_CMD_TURN:              need = 4U;  break;
        case PROTO_CMD_GOTO:              need = 8U;  break;
        case PROTO_CMD_STRAFE:            need = 4U;  break;
        case PROTO_CMD_ARC:               need = 6U;  break;
        case PROTO_CMD_QUEUE_ARC:         need = 6U;  break;
        case PROTO_CMD_SET_GYRO:          need = 3U;  break;
        case PROTO_CMD_SET_VELOCITY:      need = 4U;  break;
        case PROTO_CMD_SET_MOTION_LIMITS: need = 6U;  break;
        case PROTO_CMD_SET_DEADMAN:       need = 2U;  break;
        case PROTO_CMD_DISPLAY_MODE:      need = 1U;  break;
        case PROTO_CMD_DISPLAY_TEXT:      need = 2U;  break;
        case PROTO_CMD_GET_CONFIG:        need = 2U;  break;
        case PROTO_CMD_SET_CONFIG:        need = 2U;  break;
        case PROTO_CMD_PING:              need = 4U;  break;
        default:                          need = 0U;  break;
    }
    return need;
}

/** @brief Maps a motion-engine accept result to a protocol status code.
 *  @note  Rejections (busy/range) are also recorded in the error event log
 *         with the offending command, so the RPi sees why a move was not
 *         accepted instead of a bare status. */
static uint8_t proto_map_accept(MOTION_ACCEPT result, uint8_t cmd)
{
    uint8_t status;

    switch (result)
    {
        case MOTION_ACCEPT_OK:
            status = PROTO_STATUS_OK;
            break;
        case MOTION_ACCEPT_BUSY:
            status = PROTO_STATUS_BUSY;
            ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, cmd);
            break;
        case MOTION_ACCEPT_RANGE:
            status = PROTO_STATUS_RANGE;
            ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, cmd);
            break;
        case MOTION_ACCEPT_FAULT:
            status = PROTO_STATUS_FAULT;
            break;
        default:
            status = PROTO_STATUS_FAULT;
            break;
    }
    return status;
}

/** @brief Executes one validated request and returns the response status. */
static uint8_t proto_execute(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t status;

    if (len < proto_required_len(cmd))
    {
        /* Short frame: never execute on stale or zero-padded bytes. */
        ROBOT_EVENTS_record(ROBOT_EVENT_BAD_LEN, cmd);
        return PROTO_STATUS_BAD_LEN;
    }

    switch (cmd)
    {
        case PROTO_CMD_STOP:
            MOTION_request_stop();
            status = PROTO_STATUS_OK;
            break;

        case PROTO_CMD_ESTOP:
            MOTION_request_estop();
            status = PROTO_STATUS_OK;
            break;

        case PROTO_CMD_CLEAR_FAULTS:
            MOTION_clear_faults();
            status = PROTO_STATUS_OK;
            break;

        case PROTO_CMD_RELEASE_MOTORS:
            /* Idle-only by contract: the motion engine enforces it, so a
               request racing the end of a segment is rejected with BUSY
               instead of dropping the drivers while the wheels still turn. */
            status = proto_map_accept(MOTION_release_motors(), PROTO_CMD_RELEASE_MOTORS);
            break;

        case PROTO_CMD_SET_POSE:
        {
            float x_mm = (float)proto_get_i16(payload, 0U) * 0.1f;
            float y_mm = (float)proto_get_i16(payload, 2U) * 0.1f;
            float theta_deg = (float)proto_get_i16(payload, 4U) * 0.1f;
            if (MOTION_is_busy() != 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, PROTO_CMD_SET_POSE);
                status = PROTO_STATUS_BUSY;
            }
            else
            {
                MOTION_set_pose(x_mm, y_mm, theta_deg * (ROBOT_PI / 180.0f));
                status = PROTO_STATUS_OK;
            }
            break;
        }

        case PROTO_CMD_RESET_ODOMETRY:
            if (MOTION_is_busy() != 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, PROTO_CMD_RESET_ODOMETRY);
                status = PROTO_STATUS_BUSY;
            }
            else
            {
                MOTION_reset_odometry();
                status = PROTO_STATUS_OK;
            }
            break;

        case PROTO_CMD_DRIVE:
        {
            float distance_mm = (float)proto_get_i16(payload, 0U);
            float speed_mm_s = (float)proto_get_u16(payload, 2U);
            status = proto_map_accept(MOTION_accept_drive(distance_mm, speed_mm_s), PROTO_CMD_DRIVE);
            break;
        }

        case PROTO_CMD_TURN:
        {
            float angle_rad = (float)proto_get_i16(payload, 0U) * 0.1f * (ROBOT_PI / 180.0f);
            float speed_mm_s = (float)proto_get_u16(payload, 2U);
            status = proto_map_accept(MOTION_accept_turn(angle_rad, speed_mm_s), PROTO_CMD_TURN);
            break;
        }

        case PROTO_CMD_GOTO:
        {
            float x_mm = (float)proto_get_i16(payload, 0U) * 0.1f;
            float y_mm = (float)proto_get_i16(payload, 2U) * 0.1f;
            float theta_rad = (float)proto_get_i16(payload, 4U) * 0.1f * (ROBOT_PI / 180.0f);
            float speed_mm_s = (float)proto_get_u16(payload, 6U);
            status = proto_map_accept(MOTION_accept_goto(x_mm, y_mm, theta_rad, speed_mm_s), PROTO_CMD_GOTO);
            break;
        }

        case PROTO_CMD_STRAFE:
        {
            float distance_mm = (float)proto_get_i16(payload, 0U);
            float speed_mm_s = (float)proto_get_u16(payload, 2U);
            status = proto_map_accept(MOTION_accept_strafe(distance_mm, speed_mm_s), PROTO_CMD_STRAFE);
            break;
        }

        case PROTO_CMD_ARC:
        {
            float radius_mm = (float)proto_get_i16(payload, 0U);
            float sweep_rad = (float)proto_get_i16(payload, 2U) * 0.1f * (ROBOT_PI / 180.0f);
            float speed_mm_s = (float)proto_get_u16(payload, 4U);
            status = proto_map_accept(MOTION_accept_arc(radius_mm, sweep_rad, speed_mm_s), PROTO_CMD_ARC);
            break;
        }

        case PROTO_CMD_QUEUE_ARC:
        {
            float radius_mm = (float)proto_get_i16(payload, 0U);
            float sweep_rad = (float)proto_get_i16(payload, 2U) * 0.1f * (ROBOT_PI / 180.0f);
            float speed_mm_s = (float)proto_get_u16(payload, 4U);
            status = proto_map_accept(MOTION_queue_arc(radius_mm, sweep_rad, speed_mm_s),
                                      PROTO_CMD_QUEUE_ARC);
            break;
        }

        case PROTO_CMD_SET_MOTION_LIMITS:
        {
            uint16_t max_speed = proto_get_u16(payload, 0U);
            uint16_t accel = proto_get_u16(payload, 2U);
            uint16_t decel = proto_get_u16(payload, 4U);
            /* Through the settings store: same ranges as the config map, and
               a save (SAVE_CONFIG) persists the new limits to flash. */
            if (ROBOT_SETTINGS_write_limits(max_speed, accel, decel) == 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, PROTO_CMD_SET_MOTION_LIMITS);
                status = PROTO_STATUS_RANGE;
            }
            else
            {
                status = PROTO_STATUS_OK;
            }
            break;
        }

        case PROTO_CMD_SET_DEADMAN:
            MOTION_set_deadman_ms(proto_get_u16(payload, 0U));
            status = PROTO_STATUS_OK;
            break;

        case PROTO_CMD_SET_GYRO:
            /* Normally reached only through the silent path in PROTO_task();
               kept here as well so the command stays functional (and never
               an UNKNOWN_CMD) whatever the entry path. */
            MOTION_set_gyro_rate(proto_get_i16(payload, 0U), proto_get_u8(payload, 2U));
            status = PROTO_STATUS_OK;
            break;

        case PROTO_CMD_SET_VELOCITY:
        {
            /* Normally reached only through the silent path in PROTO_task();
               kept here as well so a direct transact (test tool) gets a real
               status instead of UNKNOWN_CMD. NO event is recorded and no
               status is ever published for the stream traffic itself: a busy
               or faulted frame is silently ignored (see PROTO_task). */
            float v_mm_s = (float)proto_get_i16(payload, 0U);
            float omega_rad_s = (float)proto_get_i16(payload, 2U) * 0.1f * (ROBOT_PI / 180.0f);
            MOTION_ACCEPT accepted = MOTION_accept_stream(v_mm_s, omega_rad_s);
            if (accepted == MOTION_ACCEPT_OK)
            {
                status = PROTO_STATUS_OK;
            }
            else if (accepted == MOTION_ACCEPT_BUSY)
            {
                status = PROTO_STATUS_BUSY;
            }
            else
            {
                status = PROTO_STATUS_FAULT;
            }
            break;
        }

        case PROTO_CMD_QUERY_ODOMETRY:
        {
            MOTION_SNAPSHOT snapshot;
            MOTION_get_snapshot(&snapshot);
            proto_rsp_put_i32(snapshot.x_um);
            proto_rsp_put_i32(snapshot.y_um);
            proto_rsp_put_i32(snapshot.theta_mdeg);
            proto_rsp_put_i16(snapshot.v_left_mm_s);
            proto_rsp_put_i16(snapshot.v_right_mm_s);
            proto_rsp_put_u8(snapshot.state);
            proto_rsp_put_u16(snapshot.faults);
            proto_rsp_put_u16(snapshot.segments_done);
            status = PROTO_STATUS_OK;
            break;
        }

        case PROTO_CMD_QUERY_STATUS:
        {
            uint8_t queue_len = (uint8_t)((uint8_t)(s_ring_head - s_ring_tail) & PROTO_RING_MASK);
            proto_rsp_put_u16((uint16_t)((uint16_t)ROBOT_FW_VERSION_MAJOR << 8) | (uint16_t)ROBOT_FW_VERSION_MINOR);
            proto_rsp_put_u32(ROBOT_PORT_tick_ms());
            proto_rsp_put_u8(MOTION_get_state());
            proto_rsp_put_u16(MOTION_get_faults());
            proto_rsp_put_u8(MOTION_get_limit_status());
            proto_rsp_put_u8(queue_len);
            proto_rsp_put_u8(s_last_seq);
            proto_rsp_put_u8(s_last_status);
            proto_rsp_put_u8(ROBOT_EVENTS_last_code());
            proto_rsp_put_u8(ROBOT_EVENTS_last_aux());
            proto_rsp_put_u8(ROBOT_EVENTS_reset_cause());
            proto_rsp_put_u8((uint8_t)ROBOT_CFG_MAP_VERSION);
            proto_rsp_put_u8(ROBOT_SETTINGS_is_dirty());
            proto_rsp_put_u8(MOTION_gyro_fresh());
            status = PROTO_STATUS_OK;
            break;
        }

        case PROTO_CMD_QUERY_EVENTS:
        {
            ROBOT_EVENT_RECORD records[ROBOT_EVENT_READ_MAX];
            uint16_t after = (len >= 2U) ? proto_get_u16(payload, 0U) : 0U;
            uint8_t count = ROBOT_EVENTS_read(after, records, (uint8_t)ROBOT_EVENT_READ_MAX);
            uint8_t index;
            proto_rsp_put_u16(ROBOT_EVENTS_latest_index());
            proto_rsp_put_u16(MOTION_get_faults());
            proto_rsp_put_u8(count);
            for (index = 0U; index < count; index++)
            {
                proto_rsp_put_u16(records[index].index);
                proto_rsp_put_u8(records[index].code);
                proto_rsp_put_u8(records[index].aux);
            }
            status = PROTO_STATUS_OK;
            break;
        }

        case PROTO_CMD_QUERY_TICKS:
            proto_rsp_put_i32(ENCODERS_MJ_get_absolut_ticks(ROBOT_ENCODER_ID_LEFT));
            proto_rsp_put_i32(ENCODERS_MJ_get_absolut_ticks(ROBOT_ENCODER_ID_RIGHT));
            status = PROTO_STATUS_OK;
            break;

        case PROTO_CMD_DISPLAY_MODE:
        {
            uint8_t mode = proto_get_u8(payload, 0U);
            if (mode <= PROTO_DISPLAY_MANUAL)
            {
                s_display_mode = mode;
                status = PROTO_STATUS_OK;
            }
            else
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, PROTO_CMD_DISPLAY_MODE);
                status = PROTO_STATUS_RANGE;
            }
            break;
        }

        case PROTO_CMD_DISPLAY_TEXT:
        {
            uint8_t line = proto_get_u8(payload, 0U);
            uint8_t offset = proto_get_u8(payload, 1U);
            /* Only the characters actually present in the frame - never a
               fixed 9-byte copy of stale buffer bytes. */
            uint8_t count = (uint8_t)(len - 2U);
            if (count > (uint8_t)PROTO_DISPLAY_CHARS_PER_FRAME)
            {
                count = (uint8_t)PROTO_DISPLAY_CHARS_PER_FRAME;
            }
            if ((line >= ROBOT_OLED_LINE_COUNT) || (offset >= (ROBOT_OLED_LINE_LEN - 1U)))
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, PROTO_CMD_DISPLAY_TEXT);
                status = PROTO_STATUS_RANGE;
            }
            else
            {
                if ((uint8_t)(offset + count) > (ROBOT_OLED_LINE_LEN - 1U))
                {
                    count = (uint8_t)((ROBOT_OLED_LINE_LEN - 1U) - offset);
                }
                (void)memcpy(&s_lines[line][offset], &payload[2], (size_t)count);
                s_lines[line][(uint8_t)(offset + count)] = '\0';
                status = PROTO_STATUS_OK;
                /* Start/refresh the display hold window (robot_display.c). */
                s_last_message_ms = ROBOT_PORT_tick_ms();
                s_message_seq++;
            }
            break;
        }

        case PROTO_CMD_GET_CONFIG:
        {
            uint8_t offset = proto_get_u8(payload, 0U);
            uint8_t count = proto_get_u8(payload, 1U);
            uint16_t values[PROTO_CFG_GET_MAX];
            if ((count == 0U) || (count > (uint8_t)PROTO_CFG_GET_MAX) ||
                (ROBOT_SETTINGS_read_registers(offset, values, count) == 0U))
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, PROTO_CMD_GET_CONFIG);
                status = PROTO_STATUS_RANGE;
            }
            else
            {
                uint8_t index;
                proto_rsp_put_u8(offset);
                proto_rsp_put_u8(count);
                for (index = 0U; index < count; index++)
                {
                    proto_rsp_put_u16(values[index]);
                }
                status = PROTO_STATUS_OK;
            }
            break;
        }

        case PROTO_CMD_SET_CONFIG:
        {
            uint8_t offset = proto_get_u8(payload, 0U);
            uint8_t count = proto_get_u8(payload, 1U);
            uint16_t values[ROBOT_CFG_SET_CHUNK_MAX];
            uint8_t index;
            if (MOTION_is_busy() != 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, PROTO_CMD_SET_CONFIG);
                status = PROTO_STATUS_BUSY;
            }
            else if ((count == 0U) || (count > (uint8_t)ROBOT_CFG_SET_CHUNK_MAX))
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, PROTO_CMD_SET_CONFIG);
                status = PROTO_STATUS_RANGE;
            }
            else if ((uint16_t)len < (uint16_t)(2U + (2U * (uint16_t)count)))
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BAD_LEN, PROTO_CMD_SET_CONFIG);
                status = PROTO_STATUS_BAD_LEN;
            }
            else
            {
                for (index = 0U; index < count; index++)
                {
                    values[index] = proto_get_u16(payload, (uint8_t)(2U + (2U * index)));
                }
                if (ROBOT_SETTINGS_write_registers(offset, values, count) == 0U)
                {
                    ROBOT_EVENTS_record(ROBOT_EVENT_BAD_PARAM, PROTO_CMD_SET_CONFIG);
                    status = PROTO_STATUS_RANGE;
                }
                else
                {
                    status = PROTO_STATUS_OK;
                }
            }
            break;
        }

        case PROTO_CMD_SAVE_CONFIG:
            if (MOTION_is_busy() != 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, PROTO_CMD_SAVE_CONFIG);
                status = PROTO_STATUS_BUSY;
            }
            else if (ROBOT_SETTINGS_save() == 0U)
            {
                status = PROTO_STATUS_STORAGE;
            }
            else
            {
                status = PROTO_STATUS_OK;
            }
            break;

        case PROTO_CMD_LOAD_CONFIG:
            if (MOTION_is_busy() != 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, PROTO_CMD_LOAD_CONFIG);
                status = PROTO_STATUS_BUSY;
            }
            else
            {
                (void)ROBOT_SETTINGS_load();
                ROBOT_SETTINGS_apply();
                status = PROTO_STATUS_OK;
            }
            break;

        case PROTO_CMD_RESET_CONFIG:
            if (MOTION_is_busy() != 0U)
            {
                ROBOT_EVENTS_record(ROBOT_EVENT_BUSY_REJECT, PROTO_CMD_RESET_CONFIG);
                status = PROTO_STATUS_BUSY;
            }
            else
            {
                ROBOT_SETTINGS_restore_defaults();
                status = PROTO_STATUS_OK;
            }
            break;

        case PROTO_CMD_PING:
            proto_rsp_put_u32(proto_get_u32(payload, 0U));
            status = PROTO_STATUS_OK;
            break;

        default:
            ROBOT_EVENTS_record(ROBOT_EVENT_UNKNOWN_CMD, cmd);
            status = PROTO_STATUS_UNKNOWN_CMD;
            break;
    }
    return status;
}

/* ---------------------------------- public API ------------------------------- */
/** @brief Initializes the protocol: ring, retry window, placeholder response. */
void PROTO_init(void)
{
    uint8_t index;

    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_last_seq = 0U;
    s_seq_history_count = 0U;
    s_last_status = PROTO_STATUS_NODATA;
    s_bus_activity_ms = 0U;
    s_display_mode = PROTO_DISPLAY_AUTO;
    s_last_message_ms = 0U;
    s_message_seq = 0U;

    for (index = 0U; index < ROBOT_OLED_LINE_COUNT; index++)
    {
        (void)memset(s_lines[index], 0, ROBOT_OLED_LINE_LEN);
    }

    /* Placeholder response until the first request arrives. */
    proto_rsp_begin(0U);
    proto_rsp_set_status(PROTO_STATUS_NODATA);
    proto_rsp_finalize();
    (void)memcpy(s_active, s_stage, PROTO_RSP_SIZE);
    (void)memcpy(s_serving, s_stage, PROTO_RSP_SIZE);
}

/** @brief Returns the 16-byte I2C reception buffer. */
uint8_t *PROTO_get_rx_buffer(void)
{
    return s_rx_buf;
}

/** @brief Returns the 32-byte response frame currently served to the master. */
const uint8_t *PROTO_get_serving_buffer(void)
{
    return s_serving;
}

/** @brief Copies the active response into the serving buffer (ISR context). */
void PROTO_copy_serving_isr(void)
{
    (void)memcpy(s_serving, s_active, PROTO_RSP_SIZE);
}

/** @brief Validates a received frame, applies STOP/ESTOP and enqueues it. */
void PROTO_on_rx_complete_isr(void)
{
    uint8_t status = PROTO_STATUS_OK;
    uint8_t next_head;

    if (s_rx_buf[0] != PROTO_SOF_REQUEST)
    {
        status = PROTO_STATUS_BAD_FRAME;
        ROBOT_EVENTS_record(ROBOT_EVENT_BAD_FRAME, s_rx_buf[0]);
    }
    else if (proto_crc8(s_rx_buf, (uint32_t)PROTO_CRC_BYTE) != s_rx_buf[PROTO_CRC_BYTE])
    {
        status = PROTO_STATUS_BAD_CRC;
        ROBOT_EVENTS_record(ROBOT_EVENT_BAD_CRC, 0U);
    }
    else
    {
        /* Immediate safety commands, executed before the frame is queued. */
        if (s_rx_buf[2] == PROTO_CMD_STOP)
        {
            MOTION_request_stop();
        }
        else if (s_rx_buf[2] == PROTO_CMD_ESTOP)
        {
            MOTION_request_estop();
        }
        else
        {
            /* normal command */
        }
    }

    next_head = (uint8_t)((s_ring_head + 1U) & PROTO_RING_MASK);
    if (next_head != s_ring_tail)
    {
        (void)memcpy(s_ring[s_ring_head].frame, s_rx_buf, PROTO_REQ_SIZE);
        s_ring[s_ring_head].rx_status = status;
        s_ring_head = next_head;
    }
    else
    {
        /* Ring full - frame dropped. The RPi retries because it timestamps
           its requests (see protocol spec section "Retries"). */
    }
}

/** @brief Records the current time as last I2C bus activity. */
void PROTO_note_bus_activity(void)
{
    s_bus_activity_ms = ROBOT_PORT_tick_ms();
}

/** @brief Returns 1 when a transfer used the bus during the last few ms. */
uint8_t PROTO_bus_is_active(void)
{
    return ((uint32_t)(ROBOT_PORT_tick_ms() - s_bus_activity_ms) < (uint32_t)PROTO_BUS_ACTIVE_MS) ? 1U : 0U;
}

/** @brief Returns the milliseconds elapsed since the last bus activity. */
uint32_t PROTO_ms_since_bus_activity(uint32_t now_ms)
{
    return (uint32_t)(now_ms - s_bus_activity_ms);
}

/** @brief Returns 1 when requests are waiting in the ring buffer. */
uint8_t PROTO_has_pending_requests(void)
{
    return (s_ring_head != s_ring_tail) ? 1U : 0U;
}

/** @brief Returns the current display mode. */
uint8_t PROTO_get_display_mode(void)
{
    return s_display_mode;
}

/** @brief Returns one manual display line buffer (never NULL). */
const char *PROTO_get_display_line(uint8_t line_idx)
{
    return (line_idx < ROBOT_OLED_LINE_COUNT) ? (const char *)s_lines[line_idx] : (const char *)"";
}

/** @brief Returns the tick of the last accepted DISPLAY_TEXT (0 = none yet). */
uint32_t PROTO_last_message_ms(void)
{
    return s_last_message_ms;
}

/** @brief Returns the number of accepted DISPLAY_TEXT frames (0 = none yet). */
uint32_t PROTO_message_seq(void)
{
    return s_message_seq;
}

/** @brief Main-loop worker: executes queued requests with sequence-number
 *         idempotency (4-deep retry window) and runs the deadman check. */
void PROTO_task(void)
{
    while (s_ring_head != s_ring_tail)
    {
        PROTO_RX_ITEM item;
        uint32_t now_ms = ROBOT_PORT_tick_ms();
        uint8_t seq;
        uint8_t cmd;
        uint8_t status;

        (void)memcpy(&item, (const void *)&s_ring[s_ring_tail], sizeof(item));
        s_ring_tail = (uint8_t)((s_ring_tail + 1U) & PROTO_RING_MASK);

        seq = item.frame[1];
        cmd = item.frame[2];
        MOTION_notify_request(now_ms);

        if ((item.rx_status == PROTO_STATUS_OK) &&
            ((cmd == PROTO_CMD_SET_GYRO) || (cmd == PROTO_CMD_SET_VELOCITY)))
        {
            /* Silent stream frames: apply and forget. No response is
               published, s_last_seq/s_last_status stay command-only and the
               frames never enter the 4-deep retry window - a 20-100 Hz
               stream must never evict command sequence numbers, because an
               idempotent retry (DRIVE etc.) depends on them. A malformed
               stream frame is ignored SILENTLY too (never an event), and a
               BUSY/FAULT rejection of a SET_VELOCITY frame is swallowed the
               same way (the stream is best-effort by design). */
            uint8_t stream_need = (cmd == PROTO_CMD_SET_GYRO) ? 3U : 4U;
            if (item.frame[3] >= stream_need)
            {
                (void)proto_execute(cmd, (const uint8_t *)&item.frame[PROTO_PAYLOAD_OFFSET], item.frame[3]);
            }
            else
            {
                /* short stream frame: nothing to apply */
            }
        }
        else if (item.rx_status != PROTO_STATUS_OK)
        {
            /* Invalid frame: answer with the error status but do NOT update the
               duplicate-seq memory - the RPi's corrected retry keeps the same
               seq and must be executed. */
            proto_rsp_begin(seq);
            proto_rsp_set_status(item.rx_status);
            proto_rsp_finalize();
            proto_publish();
        }
        else if (proto_seq_is_duplicate(seq) != 0U)
        {
            /* Duplicate (RPi retried a write whose response was lost, or the
               read was NACKed by an OLED window). Do NOT execute again; the
               active response frame already carries the correct answer. */
        }
        else
        {
            proto_rsp_begin(seq);
            status = proto_execute(cmd, (const uint8_t *)&item.frame[PROTO_PAYLOAD_OFFSET], item.frame[3]);
            proto_rsp_set_status(status);
            proto_rsp_finalize();
            proto_publish();

            /* Only executed requests are remembered for idempotent retries. */
            proto_seq_remember(seq);
            s_last_seq = seq;
            s_last_status = status;
        }
    }

    MOTION_check_deadman(ROBOT_PORT_tick_ms());
}
