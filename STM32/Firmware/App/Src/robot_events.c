/**
 ******************************************************************************
 * @file    robot_events.c
 * @brief   Error/event log implementation (RAM ring + boot-cause capture).
 *
 * @details Single storage head that never decreases; readers walk the ring
 *          from the oldest entry and filter by index. The whole record/read
 *          pair runs inside a short critical section so ISR callers (I2C
 *          error callback, frame validator, limit switches) cannot overlap a
 *          main-loop reader.
 *
 * MISRA C:2012 orientation: no dynamic memory, bounded loops, explicit casts.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include "robot_events.h"
#include "robot_configs.h"
#include "robot_port.h"
#include "main.h"

/** @brief RAM ring entry (index + code + aux, packed). */
typedef struct
{
    uint16_t index;
    uint8_t code;
    uint8_t aux;
} ROBOT_EVENT_SLOT;

static ROBOT_EVENT_SLOT s_ring[ROBOT_EVENT_RING_SIZE];
static uint16_t s_count;      /**< events recorded since boot               */
static uint8_t s_reset_cause; /**< ROBOT_EVENT_RESET_* value for the status */

/** @brief Reads and clears the RCC reset flags; returns the event code. */
static uint8_t robot_events_boot_cause(void)
{
    uint8_t cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
    {
        cause = ROBOT_EVENT_RESET_IWDG;
    }
    else if ((__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U) ||
             (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U))
    {
        cause = ROBOT_EVENT_RESET_POR;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U)
    {
        cause = ROBOT_EVENT_RESET_SW;
    }
    else if ((__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U) ||
             (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0U) ||
             (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U))
    {
        cause = ROBOT_EVENT_RESET_OTHER;
    }
    else
    {
        cause = ROBOT_EVENT_RESET_OTHER;
    }

    /* Owned by this module: clear the flags so the next boot starts clean. */
    RCC->CSR |= RCC_CSR_RMVF;
    return cause;
}

/** @brief Initializes the log and records why the board just restarted. */
void ROBOT_EVENTS_init(void)
{
    uint8_t slot;

    s_count = 0U;
    s_reset_cause = robot_events_boot_cause();
    for (slot = 0U; slot < ROBOT_EVENT_RING_SIZE; slot++)
    {
        s_ring[slot].index = 0U;
        s_ring[slot].code = ROBOT_EVENT_NONE;
        s_ring[slot].aux = 0U;
    }
    ROBOT_EVENTS_record(s_reset_cause, 0U);
}

/** @brief Records one event (ISR-safe, oldest entry overwritten). */
void ROBOT_EVENTS_record(uint8_t code, uint8_t aux)
{
    uint16_t index;
    uint8_t slot;

    if (code == ROBOT_EVENT_NONE)
    {
        return;
    }

    ROBOT_PORT_critical_enter();
    s_count++;
    index = s_count;
    slot = (uint8_t)((s_count - 1U) % (uint16_t)ROBOT_EVENT_RING_SIZE);
    s_ring[slot].index = index;
    s_ring[slot].code = code;
    s_ring[slot].aux = aux;
    ROBOT_PORT_critical_exit();
}

/** @brief @return Total number of recorded events since boot. */
uint16_t ROBOT_EVENTS_count(void)
{
    uint16_t count;

    ROBOT_PORT_critical_enter();
    count = s_count;
    ROBOT_PORT_critical_exit();
    return count;
}

/** @brief @return Index of the newest recorded event (0 when none). */
uint16_t ROBOT_EVENTS_latest_index(void)
{
    return ROBOT_EVENTS_count();
}

/** @brief Copies the events newer than @p after_index, oldest first. */
uint8_t ROBOT_EVENTS_read(uint16_t after_index, ROBOT_EVENT_RECORD *out, uint8_t max)
{
    uint8_t copied = 0U;
    uint8_t slot;
    uint16_t count;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }

    ROBOT_PORT_critical_enter();
    count = s_count;
    for (slot = 0U; (slot < ROBOT_EVENT_RING_SIZE) && (copied < max); slot++)
    {
        ROBOT_EVENT_SLOT entry = s_ring[slot];
        if ((entry.index > after_index) && (entry.index != 0U) && (entry.index <= count))
        {
            /* the ring is written in order; skipping stale slots is safe */
            if ((uint16_t)(count - entry.index) >= (uint16_t)ROBOT_EVENT_RING_SIZE)
            {
                continue; /* already overwritten by newer events */
            }
            out[copied].index = entry.index;
            out[copied].code = entry.code;
            out[copied].aux = entry.aux;
            copied++;
        }
    }
    ROBOT_PORT_critical_exit();

    /* the ring stores oldest-first only until it wraps; re-order here so the
       caller always receives ascending indices */
    for (slot = 0U; slot < copied; slot++)
    {
        uint8_t other;
        for (other = (uint8_t)(slot + 1U); other < copied; other++)
        {
            if (out[other].index < out[slot].index)
            {
                ROBOT_EVENT_RECORD swap = out[slot];
                out[slot] = out[other];
                out[other] = swap;
            }
        }
    }
    return copied;
}

/** @brief @return Code of the newest recorded event (0 = none). */
uint8_t ROBOT_EVENTS_last_code(void)
{
    uint16_t count = ROBOT_EVENTS_count();
    uint8_t slot;
    uint8_t code = ROBOT_EVENT_NONE;

    if (count == 0U)
    {
        return code;
    }
    slot = (uint8_t)((count - 1U) % (uint16_t)ROBOT_EVENT_RING_SIZE);
    ROBOT_PORT_critical_enter();
    if (s_ring[slot].index == count)
    {
        code = s_ring[slot].code;
    }
    ROBOT_PORT_critical_exit();
    return code;
}

/** @brief @return Aux byte of the newest recorded event. */
uint8_t ROBOT_EVENTS_last_aux(void)
{
    uint16_t count = ROBOT_EVENTS_count();
    uint8_t slot;
    uint8_t aux = 0U;

    if (count == 0U)
    {
        return aux;
    }
    slot = (uint8_t)((count - 1U) % (uint16_t)ROBOT_EVENT_RING_SIZE);
    ROBOT_PORT_critical_enter();
    if (s_ring[slot].index == count)
    {
        aux = s_ring[slot].aux;
    }
    ROBOT_PORT_critical_exit();
    return aux;
}

/** @brief @return Boot cause recorded at startup. */
uint8_t ROBOT_EVENTS_reset_cause(void)
{
    return s_reset_cause;
}
