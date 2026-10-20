/**
 ******************************************************************************
 * @file    robot_settings.h
 * @brief   Runtime robot settings (geometry, wheels, encoders, limits,
 *          gains): calibrated from the Raspberry Pi over I2C, kept in RAM as
 *          a packed register image and persisted in the last flash sector.
 *
 * @details Register map (u16 values; the RPi service mirrors this list in
 *          backend/app/robot/robot_config.py):
 *
 *            idx  value unit              default  range
 *            0    map version (read-only)  1         -
 *            1    stepper wheel diameter   0.1 mm   100..3000   (10..300 mm)
 *            2    stepper microsteps       1        1..64
 *            3    robot track (drive)      0.1 mm   100..4000   (10..400 mm)
 *            4    wheelbase               0.1 mm   100..4000
 *            5    odom wheel circumference 0.1 mm   200..10000  (20..1000 mm)
 *            6    odom ticks per rev       1        100..20000
 *            7    odom track              0.1 mm   100..4000
 *            8    max speed               mm/s     50..900
 *            9    min speed               mm/s     20..150
 *            10   accel                   mm/s2    50..2000
 *            11   decel                   mm/s2    50..2000
 *            12   heading gain            x100     0..500
 *            13   turn gain               x100     0..500
 *            14   strafe pattern          bits     0..15  (FL,FR,RL,RR)
 *            15   steer invert mask       bits     0..15  (per motor ID)
 *            16   encoder invert mask     bits     0..3   (bit0 L, bit1 R)
 *
 *          Derived at every change: stepper ticks/rev = 200 * microsteps,
 *          stepper circumference = PI * diameter, odom mm/tick = circ/ticks.
 *
 *          Flash layout: 64-byte records (16 words) appended one after the
 *          other in sector ROBOT_FLASH_CONFIG_SECTOR; the newest valid record
 *          wins, the sector is erased only when it is full. A save therefore
 *          costs one word-program cycle - short enough to never disturb the
 *          watchdog.
 *
 * @note    Config changes are only accepted while the robot is idle (the
 *          protocol layer enforces it); applying reconfigures the stepper
 *          and encoder drivers and refreshes the motion limits.
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_SETTINGS_H
#define ROBOT_SETTINGS_H

#include <stdint.h>

/* ------------------------------- Register map -------------------------------- */
#define ROBOT_CFG_MAP_VERSION        (1U)

#define ROBOT_CFG_IDX_VERSION        (0U)
#define ROBOT_CFG_IDX_WHEEL_DIAM     (1U)
#define ROBOT_CFG_IDX_MICROSTEPS     (2U)
#define ROBOT_CFG_IDX_ROBOT_TRACK    (3U)
#define ROBOT_CFG_IDX_WHEELBASE      (4U)
#define ROBOT_CFG_IDX_ODOM_CIRC      (5U)
#define ROBOT_CFG_IDX_ODOM_TICKS     (6U)
#define ROBOT_CFG_IDX_ODOM_TRACK     (7U)
#define ROBOT_CFG_IDX_MAX_SPEED      (8U)
#define ROBOT_CFG_IDX_MIN_SPEED      (9U)
#define ROBOT_CFG_IDX_ACCEL          (10U)
#define ROBOT_CFG_IDX_DECEL          (11U)
#define ROBOT_CFG_IDX_HEADING_GAIN   (12U)
#define ROBOT_CFG_IDX_TURN_GAIN      (13U)
#define ROBOT_CFG_IDX_STRAFE_PATTERN (14U)
#define ROBOT_CFG_IDX_STEER_INVERT   (15U)
#define ROBOT_CFG_IDX_ENCODER_INVERT (16U)
#define ROBOT_CFG_REG_COUNT          (17U)

/** @brief Largest register run one SET_CONFIG frame can carry (protocol). */
#define ROBOT_CFG_SET_CHUNK_MAX      (4U)

/** @brief Live settings in engineering units, derived from the register map. */
typedef struct
{
    float wheel_diameter_mm;
    float stepper_ticks_per_rev;
    float stepper_circumference_mm;
    float robot_track_mm;
    float wheelbase_mm;
    float odom_wheel_circumference_mm;
    float odom_ticks_per_rev;
    float odom_mm_per_tick;
    float odom_track_mm;
    float max_speed_mm_s;
    float min_speed_mm_s;
    float accel_mm_s2;
    float decel_mm_s2;
    float heading_gain;
    float turn_gain;
    uint8_t strafe_pattern;    /**< bits: 0 FL, 1 FR, 2 RL, 3 RR.          */
    uint8_t steer_invert_mask; /**< bit per motor ID 0..3.                 */
    uint8_t encoder_invert_mask; /**< bit0 = left encoder, bit1 = right.   */
} ROBOT_SETTINGS;

/** @brief Loads the defaults, then the flash record when it is valid.
 *  @note  Call once at startup BEFORE the stepper/encoder drivers are
 *         configured; does not touch any driver. */
void ROBOT_SETTINGS_init(void);

/** @brief @return Pointer to the live settings (never NULL, read-only use). */
const ROBOT_SETTINGS *ROBOT_SETTINGS_get(void);

/** @brief Copies @p count raw register values starting at @p offset.
 *  @return 1 on success, 0 when the range is out of bounds. */
uint8_t ROBOT_SETTINGS_read_registers(uint8_t offset, uint16_t *values, uint8_t count);

/** @brief Validates and applies @p count raw register values (all or nothing).
 *  @details Read-only registers, unknown indices and out-of-range values are
 *           rejected. On success the driver-relevant settings are applied
 *           immediately and the store is marked dirty (not yet saved).
 *  @return 1 on success, 0 when rejected (nothing changed). */
uint8_t ROBOT_SETTINGS_write_registers(uint8_t offset, const uint16_t *values, uint8_t count);

/** @brief Validates and applies the three motion limits atomically (used by
 *         the SET_MOTION_LIMITS command path).
 *  @return 1 on success, 0 when rejected. */
uint8_t ROBOT_SETTINGS_write_limits(uint16_t max_speed_mm_s, uint16_t accel_mm_s2, uint16_t decel_mm_s2);

/** @brief Appends the current register image to the config sector.
 *  @return 1 on success (verified by read-back), 0 on flash failure. */
uint8_t ROBOT_SETTINGS_save(void);

/** @brief Reloads the store from flash.
 *  @return 1 when a valid record was loaded; 0 when the stored record was
 *          missing/invalid (the defaults are restored and an event is
 *          recorded with the reason). */
uint8_t ROBOT_SETTINGS_load(void);

/** @brief Restores the compile-time defaults (RAM only, not saved). */
void ROBOT_SETTINGS_restore_defaults(void);

/** @brief @return 1 when RAM differs from the saved record. */
uint8_t ROBOT_SETTINGS_is_dirty(void);

/** @brief Pushes the current settings into the drivers and the motion limits.
 *  @note  Call only while the robot is idle and the drivers are initialized
 *         (the protocol layer guarantees both). */
void ROBOT_SETTINGS_apply(void);

#endif /* ROBOT_SETTINGS_H */
