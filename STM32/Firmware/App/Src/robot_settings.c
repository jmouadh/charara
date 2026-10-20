/**
 ******************************************************************************
 * @file    robot_settings.c
 * @brief   Settings store: register map, validation, apply and the
 *          append-only flash persistence of the config sector.
 *
 * @details Canonical state is the packed u16 register image; the float view
 *          (ROBOT_SETTINGS) is recomputed after every accepted change. The
 *          flash sector keeps 64-byte records chained from its base; a load
 *          walks them until the first blank one and keeps the newest valid
 *          record. Every failure path records a ROBOT_EVENT_* entry so the
 *          RPi sees WHY the store fell back to defaults.
 *
 * MISRA C:2012 orientation: no dynamic memory, bounded loops, explicit casts,
 * every switch has a default, no magic numbers.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include "robot_settings.h"
#include "robot_configs.h"
#include "robot_events.h"
#include "robot_port.h"
#include "robot_board.h"
#include "motion.h"
#include "STEPPERS_MouadhJeridi.h"
#include "ENCODERS_MouadhJeridi.h"
#include "main.h"

/** @brief Words per stored record (magic, info, hash + 17 registers + pad). */
#define ROBOT_SETTINGS_RECORD_WORDS  (ROBOT_FLASH_CONFIG_WORDS)
#define ROBOT_SETTINGS_INFO_WORD     (1U)  /**< word index of version/size   */
#define ROBOT_SETTINGS_HASH_WORD     (2U)  /**< word index of the hash       */
#define ROBOT_SETTINGS_REGS_WORD     (3U)  /**< first word holding registers */
#define ROBOT_SETTINGS_SLOT_COUNT    ((128U * 1024U) / (ROBOT_SETTINGS_RECORD_WORDS * 4U))
#define ROBOT_SETTINGS_SLOT_ADDR(slot) \
    (ROBOT_FLASH_CONFIG_ADDR + ((uint32_t)(slot) * ROBOT_SETTINGS_RECORD_WORDS * 4U))

/** @brief Fallback reasons carried in the CONFIG_FALLBACK event aux byte. */
#define ROBOT_SETTINGS_REASON_MAGIC   (1U)
#define ROBOT_SETTINGS_REASON_VERSION (2U)
#define ROBOT_SETTINGS_REASON_HASH    (3U)
#define ROBOT_SETTINGS_REASON_RANGE   (4U)

/** @brief Flash failure kinds carried in the FLASH_ERROR event aux byte. */
#define ROBOT_SETTINGS_FLASH_ERASE    (1U)
#define ROBOT_SETTINGS_FLASH_PROGRAM  (2U)
#define ROBOT_SETTINGS_FLASH_VERIFY   (3U)

/** @brief Per-register accepted range. */
typedef struct
{
    uint16_t min_value;
    uint16_t max_value;
} ROBOT_SETTINGS_RANGE;

static uint16_t s_regs[ROBOT_CFG_REG_COUNT];
static ROBOT_SETTINGS s_settings;
static uint8_t s_dirty;

/** @brief Accepted ranges, indexed like the register map (index 0 read-only). */
static const ROBOT_SETTINGS_RANGE s_ranges[ROBOT_CFG_REG_COUNT] =
{
    { (uint16_t)ROBOT_CFG_MAP_VERSION, (uint16_t)ROBOT_CFG_MAP_VERSION }, /* 0  version   */
    { 100U,   3000U  },  /* 1  wheel diameter 0.1 mm      */
    { 1U,     64U    },  /* 2  microsteps                 */
    { 100U,   4000U  },  /* 3  robot track 0.1 mm         */
    { 100U,   4000U  },  /* 4  wheelbase 0.1 mm           */
    { 200U,   10000U },  /* 5  odom circumference 0.1 mm  */
    { 100U,   20000U },  /* 6  odom ticks per rev         */
    { 100U,   4000U  },  /* 7  odom track 0.1 mm          */
    { 50U,    900U   },  /* 8  max speed mm/s             */
    { 20U,    150U   },  /* 9  min speed mm/s             */
    { 50U,    2000U  },  /* 10 accel mm/s2                */
    { 50U,    2000U  },  /* 11 decel mm/s2                */
    { 0U,     500U   },  /* 12 heading gain x100          */
    { 0U,     500U   },  /* 13 turn gain x100             */
    { 0U,     15U    },  /* 14 strafe pattern             */
    { 0U,     15U    },  /* 15 steer invert mask          */
    { 0U,     3U     },  /* 16 encoder invert mask        */
};

/** @brief Wired DIR polarity per stepper ID (1 = inverted): the base the
 *         runtime steer_invert_mask is XORed on top of (robot_configs.h). */
static const uint8_t s_stepper_base_invert[STEPPERS_MJ_MAX_STEPPERS_NUMBER] =
{
    (ROBOT_STEPPER_INVERT_DIR_0 != GPIO_PIN_RESET) ? 1U : 0U,
    (ROBOT_STEPPER_INVERT_DIR_1 != GPIO_PIN_RESET) ? 1U : 0U,
    (ROBOT_STEPPER_INVERT_DIR_2 != GPIO_PIN_RESET) ? 1U : 0U,
    (ROBOT_STEPPER_INVERT_DIR_3 != GPIO_PIN_RESET) ? 1U : 0U,
};

/* ------------------------------ helpers ------------------------------------- */
/** @brief @return 1 when the value passes the range table for @p index. */
static uint8_t settings_value_ok(uint8_t index, uint16_t value)
{
    if (index >= (uint8_t)ROBOT_CFG_REG_COUNT)
    {
        return 0U;
    }
    return ((value >= s_ranges[index].min_value) && (value <= s_ranges[index].max_value)) ? 1U : 0U;
}

/** @brief FNV-1a hash over the register image (flash record integrity). */
static uint32_t settings_hash(void)
{
    uint32_t hash = 2166136261UL;
    uint8_t index;

    for (index = 0U; index < (uint8_t)ROBOT_CFG_REG_COUNT; index++)
    {
        hash ^= (uint32_t)s_regs[index];
        hash *= 16777619UL;
    }
    return hash;
}

/** @brief Recomputes the float view from the register image. */
static void settings_recompute(void)
{
    s_settings.wheel_diameter_mm = (float)s_regs[ROBOT_CFG_IDX_WHEEL_DIAM] * 0.1f;
    s_settings.stepper_ticks_per_rev = (float)ROBOT_STEPPER_FULL_STEPS * (float)s_regs[ROBOT_CFG_IDX_MICROSTEPS];
    s_settings.stepper_circumference_mm = ROBOT_PI * s_settings.wheel_diameter_mm;
    s_settings.robot_track_mm = (float)s_regs[ROBOT_CFG_IDX_ROBOT_TRACK] * 0.1f;
    s_settings.wheelbase_mm = (float)s_regs[ROBOT_CFG_IDX_WHEELBASE] * 0.1f;
    s_settings.odom_wheel_circumference_mm = (float)s_regs[ROBOT_CFG_IDX_ODOM_CIRC] * 0.1f;
    s_settings.odom_ticks_per_rev = (float)s_regs[ROBOT_CFG_IDX_ODOM_TICKS];
    s_settings.odom_mm_per_tick = s_settings.odom_wheel_circumference_mm / s_settings.odom_ticks_per_rev;
    s_settings.odom_track_mm = (float)s_regs[ROBOT_CFG_IDX_ODOM_TRACK] * 0.1f;
    s_settings.max_speed_mm_s = (float)s_regs[ROBOT_CFG_IDX_MAX_SPEED];
    s_settings.min_speed_mm_s = (float)s_regs[ROBOT_CFG_IDX_MIN_SPEED];
    s_settings.accel_mm_s2 = (float)s_regs[ROBOT_CFG_IDX_ACCEL];
    s_settings.decel_mm_s2 = (float)s_regs[ROBOT_CFG_IDX_DECEL];
    s_settings.heading_gain = (float)s_regs[ROBOT_CFG_IDX_HEADING_GAIN] * 0.01f;
    s_settings.turn_gain = (float)s_regs[ROBOT_CFG_IDX_TURN_GAIN] * 0.01f;
    s_settings.strafe_pattern = (uint8_t)s_regs[ROBOT_CFG_IDX_STRAFE_PATTERN];
    s_settings.steer_invert_mask = (uint8_t)s_regs[ROBOT_CFG_IDX_STEER_INVERT];
    s_settings.encoder_invert_mask = (uint8_t)s_regs[ROBOT_CFG_IDX_ENCODER_INVERT];
}

/** @brief Fills the register image with the compile-time defaults. */
static void settings_defaults(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)ROBOT_CFG_REG_COUNT; index++)
    {
        s_regs[index] = 0U;
    }

    s_regs[ROBOT_CFG_IDX_VERSION] = (uint16_t)ROBOT_CFG_MAP_VERSION;
    s_regs[ROBOT_CFG_IDX_WHEEL_DIAM] = (uint16_t)(ROBOT_WHEEL_DIAMETER_MM * 10.0f);
    s_regs[ROBOT_CFG_IDX_MICROSTEPS] = (uint16_t)ROBOT_STEPPER_MICROSTEPS;
    s_regs[ROBOT_CFG_IDX_ROBOT_TRACK] = (uint16_t)(ROBOT_TRACK_MM * 10.0f);
    s_regs[ROBOT_CFG_IDX_WHEELBASE] = (uint16_t)(ROBOT_WHEELBASE_MM * 10.0f);
    s_regs[ROBOT_CFG_IDX_ODOM_CIRC] = (uint16_t)(ROBOT_ODOM_WHEEL_CIRCUMFERENCE_MM * 10.0f);
    s_regs[ROBOT_CFG_IDX_ODOM_TICKS] = (uint16_t)ROBOT_ODOM_TICKS_PER_REV;
    s_regs[ROBOT_CFG_IDX_ODOM_TRACK] = (uint16_t)(ROBOT_ODOM_TRACK_MM * 10.0f);
    s_regs[ROBOT_CFG_IDX_MAX_SPEED] = (uint16_t)ROBOT_MAX_SPEED_MM_S;
    s_regs[ROBOT_CFG_IDX_MIN_SPEED] = (uint16_t)ROBOT_MIN_SPEED_MM_S;
    s_regs[ROBOT_CFG_IDX_ACCEL] = (uint16_t)ROBOT_DEFAULT_ACCEL_MM_S2;
    s_regs[ROBOT_CFG_IDX_DECEL] = (uint16_t)ROBOT_DEFAULT_DECEL_MM_S2;
    s_regs[ROBOT_CFG_IDX_HEADING_GAIN] = (uint16_t)(ROBOT_HEADING_GAIN_PER_S * 100.0f);
    s_regs[ROBOT_CFG_IDX_TURN_GAIN] = (uint16_t)(ROBOT_TURN_GAIN_PER_S * 100.0f);
    s_regs[ROBOT_CFG_IDX_STRAFE_PATTERN] = (uint16_t)((ROBOT_STRAFE_PLUS_Y_FL & 0x1U)
        | ((ROBOT_STRAFE_PLUS_Y_FR & 0x1U) << 1)
        | ((ROBOT_STRAFE_PLUS_Y_RL & 0x1U) << 2)
        | ((ROBOT_STRAFE_PLUS_Y_RR & 0x1U) << 3));
    s_regs[ROBOT_CFG_IDX_STEER_INVERT] = 0U;
    s_regs[ROBOT_CFG_IDX_ENCODER_INVERT] = 0U;
    settings_recompute();
}

/* ------------------------------- flash IO ----------------------------------- */
/** @brief @return 1 when the record at @p slot passes magic/version/hash. */
static uint8_t settings_slot_valid(uint32_t slot)
{
    const uint32_t *record = (const uint32_t *)ROBOT_SETTINGS_SLOT_ADDR(slot);
    uint32_t info;
    uint32_t expected;
    uint32_t hash = 2166136261UL;
    uint8_t index;

    if (record[0] != ROBOT_FLASH_CONFIG_MAGIC)
    {
        return 0U;
    }
    info = record[ROBOT_SETTINGS_INFO_WORD];
    if ((info >> 16) != (uint32_t)ROBOT_CFG_MAP_VERSION)
    {
        return 0U;
    }
    if ((info & 0xFFFFU) != (uint32_t)ROBOT_CFG_REG_COUNT)
    {
        return 0U;
    }
    for (index = 0U; index < (uint8_t)ROBOT_CFG_REG_COUNT; index++)
    {
        uint32_t word = record[ROBOT_SETTINGS_REGS_WORD + ((uint32_t)index / 2U)];
        uint16_t value = (uint16_t)((index % 2U) == 0U ? (word & 0xFFFFU) : ((word >> 16) & 0xFFFFU));
        hash ^= (uint32_t)value;
        hash *= 16777619UL;
    }
    expected = record[ROBOT_SETTINGS_HASH_WORD];
    return (hash == expected) ? 1U : 0U;
}

/** @brief Copies the register image of the record at @p slot into RAM. */
static void settings_slot_read(uint32_t slot)
{
    const uint32_t *record = (const uint32_t *)ROBOT_SETTINGS_SLOT_ADDR(slot);
    uint8_t index;

    for (index = 0U; index < (uint8_t)ROBOT_CFG_REG_COUNT; index++)
    {
        uint32_t word = record[ROBOT_SETTINGS_REGS_WORD + ((uint32_t)index / 2U)];
        s_regs[index] = (uint16_t)((index % 2U) == 0U ? (word & 0xFFFFU) : ((word >> 16) & 0xFFFFU));
    }
}

/** @brief Programs the 16-word record at @p slot from the current image. */
static uint8_t settings_slot_write(uint32_t slot)
{
    uint32_t record[ROBOT_SETTINGS_RECORD_WORDS];
    uint32_t address = ROBOT_SETTINGS_SLOT_ADDR(slot);
    uint8_t index;
    uint8_t word;

    for (word = 0U; word < (uint8_t)ROBOT_SETTINGS_RECORD_WORDS; word++)
    {
        record[word] = 0U;
    }
    record[0] = ROBOT_FLASH_CONFIG_MAGIC;
    record[ROBOT_SETTINGS_INFO_WORD] =
        (((uint32_t)ROBOT_CFG_MAP_VERSION << 16) | (uint32_t)ROBOT_CFG_REG_COUNT);
    record[ROBOT_SETTINGS_HASH_WORD] = settings_hash();
    for (index = 0U; index < (uint8_t)ROBOT_CFG_REG_COUNT; index++)
    {
        uint32_t word_index = ROBOT_SETTINGS_REGS_WORD + ((uint32_t)index / 2U);
        if ((index % 2U) == 0U)
        {
            record[word_index] = (uint32_t)s_regs[index];
        }
        else
        {
            record[word_index] |= ((uint32_t)s_regs[index] << 16);
        }
    }

    for (word = 0U; word < (uint8_t)ROBOT_SETTINGS_RECORD_WORDS; word++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + ((uint32_t)word * 4U), record[word]) != HAL_OK)
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief @return 1 when @p slot still reads back as blank (all ones). */
static uint8_t settings_slot_blank(uint32_t slot)
{
    const uint32_t *record = (const uint32_t *)ROBOT_SETTINGS_SLOT_ADDR(slot);

    return (record[0] == 0xFFFFFFFFUL) ? 1U : 0U;
}

/* ------------------------------- public API --------------------------------- */
/** @brief Loads defaults, then the newest valid flash record when present. */
void ROBOT_SETTINGS_init(void)
{
    s_dirty = 0U;
    settings_defaults();

    if (!ROBOT_SETTINGS_load())
    {
        s_dirty = 0U; /* defaults and flash agree by definition of "empty" */
    }
}

/** @brief @return Pointer to the live settings struct. */
const ROBOT_SETTINGS *ROBOT_SETTINGS_get(void)
{
    return &s_settings;
}

/** @brief Copies raw register values; 0 when out of bounds. */
uint8_t ROBOT_SETTINGS_read_registers(uint8_t offset, uint16_t *values, uint8_t count)
{
    uint8_t index;

    if ((values == NULL) || (count == 0U) ||
        (((uint16_t)offset + (uint16_t)count) > (uint16_t)ROBOT_CFG_REG_COUNT))
    {
        return 0U;
    }
    for (index = 0U; index < count; index++)
    {
        values[index] = s_regs[(uint8_t)(offset + index)];
    }
    return 1U;
}

/** @brief Validates and applies a run of register values (all or nothing). */
uint8_t ROBOT_SETTINGS_write_registers(uint8_t offset, const uint16_t *values, uint8_t count)
{
    uint8_t index;

    if ((values == NULL) || (count == 0U) ||
        (((uint16_t)offset + (uint16_t)count) > (uint16_t)ROBOT_CFG_REG_COUNT))
    {
        return 0U;
    }
    for (index = 0U; index < count; index++)
    {
        uint8_t reg = (uint8_t)(offset + index);
        if ((reg == (uint8_t)ROBOT_CFG_IDX_VERSION) || (settings_value_ok(reg, values[index]) == 0U))
        {
            return 0U;
        }
    }

    for (index = 0U; index < count; index++)
    {
        s_regs[(uint8_t)(offset + index)] = values[index];
    }
    settings_recompute();
    s_dirty = 1U;
    ROBOT_SETTINGS_apply();
    ROBOT_EVENTS_record(ROBOT_EVENT_CONFIG_APPLIED, (uint8_t)offset);
    return 1U;
}

/** @brief Validates and applies the three motion limits atomically. */
uint8_t ROBOT_SETTINGS_write_limits(uint16_t max_speed_mm_s, uint16_t accel_mm_s2, uint16_t decel_mm_s2)
{
    if ((settings_value_ok((uint8_t)ROBOT_CFG_IDX_MAX_SPEED, max_speed_mm_s) == 0U) ||
        (settings_value_ok((uint8_t)ROBOT_CFG_IDX_ACCEL, accel_mm_s2) == 0U) ||
        (settings_value_ok((uint8_t)ROBOT_CFG_IDX_DECEL, decel_mm_s2) == 0U))
    {
        return 0U;
    }

    s_regs[ROBOT_CFG_IDX_MAX_SPEED] = max_speed_mm_s;
    s_regs[ROBOT_CFG_IDX_ACCEL] = accel_mm_s2;
    s_regs[ROBOT_CFG_IDX_DECEL] = decel_mm_s2;
    settings_recompute();
    s_dirty = 1U;
    /* Only the limits are pushed: they are safe to change while a segment
       runs (the protocol allows SET_MOTION_LIMITS mid-motion; reconfiguring
       the drivers themselves is reserved to the idle-guarded write path). */
    MOTION_set_limits(s_settings.max_speed_mm_s, s_settings.accel_mm_s2, s_settings.decel_mm_s2);
    ROBOT_EVENTS_record(ROBOT_EVENT_CONFIG_APPLIED, (uint8_t)ROBOT_CFG_IDX_MAX_SPEED);
    return 1U;
}

/** @brief Appends the current image to the config sector (newest wins). */
uint8_t ROBOT_SETTINGS_save(void)
{
    uint32_t slot = 0U;
    uint8_t ok;

    while ((slot < (uint32_t)ROBOT_SETTINGS_SLOT_COUNT) && (settings_slot_blank(slot) == 0U))
    {
        slot++;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        ROBOT_EVENTS_record(ROBOT_EVENT_FLASH_ERROR, (uint8_t)ROBOT_SETTINGS_FLASH_PROGRAM);
        return 0U;
    }

    if (slot >= (uint32_t)ROBOT_SETTINGS_SLOT_COUNT)
    {
        FLASH_EraseInitTypeDef erase;
        uint32_t sector_error = 0U;

        BSP_watchdog_refresh(); /* the 128K sector erase is the long operation */
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.Sector = (uint32_t)ROBOT_FLASH_CONFIG_SECTOR;
        erase.NbSectors = 1U;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            ROBOT_EVENTS_record(ROBOT_EVENT_FLASH_ERROR, (uint8_t)ROBOT_SETTINGS_FLASH_ERASE);
            return 0U;
        }
        slot = 0U;
    }

    ok = settings_slot_write(slot);
    (void)HAL_FLASH_Lock();

    if ((ok == 0U) || (settings_slot_valid(slot) == 0U))
    {
        ROBOT_EVENTS_record(ROBOT_EVENT_FLASH_ERROR, (uint8_t)ROBOT_SETTINGS_FLASH_VERIFY);
        return 0U;
    }

    s_dirty = 0U;
    ROBOT_EVENTS_record(ROBOT_EVENT_CONFIG_SAVED, 0U);
    return 1U;
}

/** @brief Reloads the newest valid flash record; falls back to defaults. */
uint8_t ROBOT_SETTINGS_load(void)
{
    uint32_t slot = 0U;
    uint32_t newest = 0U;
    uint8_t found = 0U;
    uint8_t index;
    uint8_t reason = (uint8_t)ROBOT_SETTINGS_REASON_HASH;

    if (settings_slot_blank(0U) != 0U)
    {
        /* empty sector: factory-fresh board, nothing to report */
        return 0U;
    }

    while ((slot < (uint32_t)ROBOT_SETTINGS_SLOT_COUNT) && (settings_slot_blank(slot) == 0U))
    {
        if (settings_slot_valid(slot) != 0U)
        {
            newest = slot;
            found = 1U;
        }
        else if (found == 0U)
        {
            /* first chain entry invalid: remember why (magic/version/hash) */
            const uint32_t *record = (const uint32_t *)ROBOT_SETTINGS_SLOT_ADDR(slot);
            if (record[0] != ROBOT_FLASH_CONFIG_MAGIC)
            {
                reason = (uint8_t)ROBOT_SETTINGS_REASON_MAGIC;
            }
            else if ((record[ROBOT_SETTINGS_INFO_WORD] >> 16) != (uint32_t)ROBOT_CFG_MAP_VERSION)
            {
                reason = (uint8_t)ROBOT_SETTINGS_REASON_VERSION;
            }
            else
            {
                reason = (uint8_t)ROBOT_SETTINGS_REASON_HASH;
            }
        }
        else
        {
            /* a gap inside the chain: keep the newest valid record seen */
        }
        slot++;
    }

    if (found == 0U)
    {
        settings_defaults();
        s_dirty = 1U;
        ROBOT_EVENTS_record(ROBOT_EVENT_CONFIG_FALLBACK, reason);
        return 0U;
    }

    settings_slot_read(newest);
    for (index = 0U; index < (uint8_t)ROBOT_CFG_REG_COUNT; index++)
    {
        if (settings_value_ok(index, s_regs[index]) == 0U)
        {
            settings_defaults();
            s_dirty = 1U;
            ROBOT_EVENTS_record(ROBOT_EVENT_CONFIG_FALLBACK, (uint8_t)ROBOT_SETTINGS_REASON_RANGE);
            return 0U;
        }
    }
    settings_recompute();
    s_dirty = 0U;
    return 1U;
}

/** @brief Restores defaults in RAM (event recorded, not saved). */
void ROBOT_SETTINGS_restore_defaults(void)
{
    settings_defaults();
    s_dirty = 1U;
    ROBOT_SETTINGS_apply();
    ROBOT_EVENTS_record(ROBOT_EVENT_CONFIG_DEFAULTS, 0U);
}

/** @brief @return 1 when RAM differs from the saved record. */
uint8_t ROBOT_SETTINGS_is_dirty(void)
{
    return s_dirty;
}

/** @brief Pushes the settings into the drivers and the motion limits. */
void ROBOT_SETTINGS_apply(void)
{
    uint8_t motor;

    /* Stepper wheels: ticks per revolution and circumference. */
    STEPPERS_MJ_configure(s_settings.stepper_ticks_per_rev, s_settings.stepper_circumference_mm);
    /* Odometry wheels: encoder counts and circumference. */
    ENCODERS_MJ_configure(s_settings.odom_ticks_per_rev, s_settings.odom_wheel_circumference_mm);

    /* Direction inversions (mechanical orientation calibration): the wired
       polarity from robot_configs.h is the base, the runtime masks are XORed
       on top of it, so a mask change toggles relative to the wiring instead of
       replacing it. */
    for (motor = 0U; motor < (uint8_t)STEPPERS_MJ_MAX_STEPPERS_NUMBER; motor++)
    {
        uint8_t inverted = (uint8_t)(s_stepper_base_invert[motor]
                                     ^ ((s_settings.steer_invert_mask >> motor) & 0x1U));
        STEPPERS_MJ_set_invert_direction(motor,
            (inverted != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    ENCODERS_MJ_set_invert_counting(ROBOT_ENCODER_ID_LEFT,
        (uint8_t)(((s_settings.encoder_invert_mask >> 0) & 0x1U) ^ ROBOT_ENCODER_INVERT_COUNT_LEFT));
    ENCODERS_MJ_set_invert_counting(ROBOT_ENCODER_ID_RIGHT,
        (uint8_t)(((s_settings.encoder_invert_mask >> 1) & 0x1U) ^ ROBOT_ENCODER_INVERT_COUNT_RIGHT));

    /* Motion limits (speed floor/ceiling and ramps). */
    MOTION_set_limits(s_settings.max_speed_mm_s, s_settings.accel_mm_s2, s_settings.decel_mm_s2);
}
