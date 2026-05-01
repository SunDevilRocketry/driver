/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : LSM6DSV320X IMU Driver Interface
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Hardware Assumptions:
  *        Target: STM32H733 (A0010, Rev 3 PCB)
  *        IMU:    LSM6DSV320X (ST 6-axis IMU, ±320g high-g accelerometer)
  *        Bus:    SPI via hspi1 (Mode 3, CS active-low on PA4)
  *        INT1:   PC4  (FIFO watermark interrupt)
  *        INT2:   PC5  (reserved / ODR-triggered mode reference)
  *
  *        Submodule dependency:
  *          lib/lsm6dsv320x-pid (github.com/STMicroelectronics/lsm6dsv320x-pid)
  *
  *        The LSM6DSV320X embeds a sensor fusion low-power (SFLP) algorithm
  *        outputting a game rotation vector (quaternion), gravity vector, and
  *        gyroscope bias estimate via the FIFO. This driver exposes:
  *          - Blocking init (always)
  *          - Blocking accel + gyro direct register read
  *          - Blocking SFLP embedded-register read
  *          - DMA FIFO burst: accel + gyro + SFLP (non-blocking flight path)
  *          - Runtime full-scale and power-mode setters (no reset required)
  *        
  *        DMA notes (STM32H7):
  *          HAL_SPI_TransmitReceive_DMA() offloads the bus entirely.
  *          After transfer complete, imu_process_async_cb() MUST invalidate
  *          the D-Cache region covering the rx buffer before parsing
  *          (SCB_InvalidateDCache_by_Addr).
  * 
  * 
  *        Performance target: accel + gyro read < 1 ms @ 1.92 kHz ODR
  *        (DS14623 Rev3 Table 54/57: ODR_XL/G = 1010b → 1.92 kHz)
  *
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 Sun Devil Rocketry. All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause (https://opensource.org/license/bsd-3-clause).
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_T_H
#define IMU_T_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "lsm6dsv320x_reg.h"

/*------------------------------------------------------------------------------
 Macros
------------------------------------------------------------------------------*/

/** @brief WHO_AM_I fixed value. DS14623 Rev3 §9.13 (0Fh): reset value 0x73  */
#define IMU_WHO_AM_I_VAL            ( 0x73U )

/** @brief SPI read bit — bit[7] of address byte = 1. DS14623 Rev3 §5.1.3     */
#define IMU_SPI_READ_BIT            ( 0x80U )

/** @brief Blocking SPI timeout (ms) used during init and blocking reads       */
#define IMU_SPI_TIMEOUT_MS          ( 10U )

/**
 * @brief SFLP game rotation vector ODR: 480 Hz (maximum supported).
 *        DS14623 Rev3 Table 364: SFLP_GAME_ODR[2:0] = 101b
 */
#define IMU_SFLP_ODR                LSM6DSV320X_SFLP_480Hz

/**
 * @brief FIFO watermark threshold (FIFO words = slots).
 *        1 LSB = 1 FIFO slot (7 bytes: TAG + 6 data). DS14623 Rev3 Table 36.
 *        NOTE: This macro is NOT used by imu_init(). The watermark is
 *        calculated dynamically at init time based on the enabled features:
 *          Base (Gyro + Accel):          2 slots
 *          + SFLP (Quat, Grav, Gbias):  +3 slots
 *        This ensures INT1 fires exactly once per complete sample group
 *        regardless of which features are enabled. The macro is retained
 *        here as a reference value for bypass/polling configurations only.
 */
#define IMU_FIFO_WATERMARK          ( 5U )

/** @brief Bytes per FIFO slot: 1 tag byte + 6 data bytes. DS14623 §9.90      */
#define IMU_FIFO_SLOT_BYTES         ( 7U )

/**
 * @brief Maximum FIFO slots read per DMA transaction.
 *        Buffer size = IMU_DMA_BUF_SLOTS * IMU_FIFO_SLOT_BYTES + 1 (cmd byte).
 */
#define IMU_DMA_BUF_SLOTS           ( 8U )

/** @brief Total DMA rx buffer size in bytes (cmd + slots).                    */
#define IMU_DMA_BUF_BYTES           ( 1U + ( IMU_DMA_BUF_SLOTS * IMU_FIFO_SLOT_BYTES ) )

/** @brief Padded DMA size ensuring alignment with 32-byte Cortex-M7 cache line */
#define IMU_DMA_BUF_BYTES_ALIGNED   ( ( IMU_DMA_BUF_BYTES + 31U ) & ~31U )

/** @brief FIFO tag: gyroscope NC.              DS14623 Rev3 Table 232: 0x01  */
#define IMU_FIFO_TAG_GYRO           ( 0x01U )

/** @brief FIFO tag: accelerometer NC.          DS14623 Rev3 Table 232: 0x02  */
#define IMU_FIFO_TAG_ACCEL          ( 0x02U )

/** @brief FIFO tag: SFLP game rotation vector. DS14623 Rev3 Table 232: 0x13  */
#define IMU_FIFO_TAG_SFLP_QUAT      ( 0x13U )

/** @brief FIFO tag: SFLP gyroscope bias.       DS14623 Rev3 Table 232: 0x16  */
#define IMU_FIFO_TAG_SFLP_GBIAS     ( 0x16U )

/** @brief FIFO tag: SFLP gravity vector.       DS14623 Rev3 Table 232: 0x17  */
#define IMU_FIFO_TAG_SFLP_GRAV      ( 0x17U )

/** @brief FIFO tag: high-g accelerometer.      DS14623 Rev3 Table 232: 0x1D  */
#define IMU_FIFO_TAG_HG_ACCEL       ( 0x1DU )

/**
 * @brief Data-ready poll timeout (ms) for imu_read_sync().
 *        Must exceed the longest possible sample period. At the slowest
 *        supported ODR (480 Hz), one period = 1000/480 = 2.083 ms, so 2 ms
 *        would time out before the first sample arrives. 5 ms gives ~2.4x
 *        headroom at 480 Hz and ~9.6x at the default 1920 Hz.
 *        DS14623 Rev3 Table 54.
 */
#define IMU_DRDY_TIMEOUT_MS         ( 5U )


/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/

/**
 * @brief Raw 16-bit two's complement output from accel and gyro registers.
 *        DS14623 Rev3 §9.30-9.35 (OUTX_L_G 22h -> OUTZ_H_A 2Dh).
 */
typedef struct _IMU_RAW
    {
    int16_t accel_x;   /* Accelerometer X raw counts */
    int16_t accel_y;   /* Accelerometer Y raw counts */
    int16_t accel_z;   /* Accelerometer Z raw counts */
    int16_t gyro_x;    /* Gyroscope X raw counts     */
    int16_t gyro_y;    /* Gyroscope Y raw counts     */
    int16_t gyro_z;    /* Gyroscope Z raw counts     */
    } IMU_RAW;

/**
 * @brief SFLP sensor fusion output (float32).
 *        Quaternion is the game rotation vector (6-axis, no magnetometer).
 *        Unit quaternion: w^2 + x^2 + y^2 + z^2 = 1.
 *        DS14623 Rev3 §2.8, §13.23-13.30.
 */
typedef struct _IMU_SFLP_DATA
    {
    float quat_w;      /* Quaternion scalar W                                */
    float quat_x;      /* Quaternion vector X                                */
    float quat_y;      /* Quaternion vector Y                                */
    float quat_z;      /* Quaternion vector Z                                */
    float grav_x;      /* Gravity unit vector X                              */
    float grav_y;      /* Gravity unit vector Y                              */
    float grav_z;      /* Gravity unit vector Z                              */
    float gbias_x;     /* Gyroscope bias X [same FS as gyro output]          */
    float gbias_y;     /* Gyroscope bias Y                                   */
    float gbias_z;     /* Gyroscope bias Z                                   */
    } IMU_SFLP_DATA;

/**
 * @brief Physical-unit converted IMU output.
 *        Sensitivity values from DS14623 Rev3 Table 3.
 */
typedef struct _IMU_DATA
    {
    float accel_x;     /* Accelerometer X [g]   */
    float accel_y;     /* Accelerometer Y [g]   */
    float accel_z;     /* Accelerometer Z [g]   */
    float gyro_x;      /* Gyroscope X [dps]     */
    float gyro_y;      /* Gyroscope Y [dps]     */
    float gyro_z;      /* Gyroscope Z [dps]     */
    } IMU_DATA;

/**
 * @brief Output Data Rate (ODR) selection for loop coupling.
 *        Driver-internal enum mapped to ST PID ODR values inside imu_init().
 *        Accel ODR (CTRL1 10h), Gyro ODR (CTRL2 11h), and FIFO BDR (FIFO_CTRL3
 *        09h) are all derived from the same value to keep them perfectly synced.
 *        DS14623 Rev3 Tables 54, 57, 232.
 */
typedef enum _IMU_ODR
    {
    IMU_ODR_480HZ,
    IMU_ODR_960HZ,
    IMU_ODR_1920HZ,
    IMU_ODR_3840HZ,
    IMU_ODR_7680HZ
    } IMU_ODR;

/**
 * @brief Accelerometer full-scale selection.
 *        The LSM6DSV320X has two physically separate accelerometer sensing chains:
 *          Low-G  chain: ±2 / 4 / 8 / 16 g - configured via CTRL8 (17h) FS_XL[1:0].
 *          High-G chain: ±32 / 64 / 128 / 256 / 320 g - configured via CTRL1_XL_HG (4Eh) FS_XL_HG[2:0].
 *        DS14623 Rev3 Table 70 (low-G), Tables 148-150 (high-G).
 *        Sensitivity values from DS14623 Rev3 Table 3.
 *
 *        <IMPORTANT> values are sequential driver-internal integers, NOT
 *        assigned to ST PID macros. The low-G PID enum
 *        (lsm6dsv320x_xl_full_scale_t: 2g=0, 4g=1, 8g=2, 16g=3) and the
 *        high-G PID enum (lsm6dsv320x_xl_hg_full_scale_t: 32g=0, 64g=1 ...)
 *        both start at 0. Aliasing our enum to those macros would cause
 *        IMU_ACC_FS_32G == IMU_ACC_FS_2G == 0, breaking the high-G branch
 *        check and producing duplicate case values in imu_scale_raw().
 *        Mapping to the PID register codes is done internally in imu_init()
 *        and imu_set_accel_fs() via dedicated switch statements to avoid this.
 */
typedef enum _IMU_ACC_FS
    {
    IMU_ACC_FS_2G   = 0, /* ±2 g,    0.061  mg/LSB  [low-G chain]  */
    IMU_ACC_FS_4G,       /* ±4 g,    0.122  mg/LSB  [low-G chain]  */
    IMU_ACC_FS_8G,       /* ±8 g,    0.244  mg/LSB  [low-G chain]  */
    IMU_ACC_FS_16G,      /* ±16 g,   0.488  mg/LSB  [low-G chain]  */
    IMU_ACC_FS_32G,      /* ±32 g,   0.976  mg/LSB  [high-G chain] */
    IMU_ACC_FS_64G,      /* ±64 g,   1.952  mg/LSB  [high-G chain] */
    IMU_ACC_FS_128G,     /* ±128 g,  3.904  mg/LSB  [high-G chain] */
    IMU_ACC_FS_256G,     /* ±256 g,  7.808  mg/LSB  [high-G chain] */
    IMU_ACC_FS_320G,     /* ±320 g,  10.417 mg/LSB  [high-G chain] */
    } IMU_ACC_FS;

/**
 * @brief Gyroscope full-scale selection.
 *        DS14623 Rev3 Table 65: FS_G[2:0] in CTRL6 (15h).
 */
typedef enum _IMU_GYRO_FS
    {
    IMU_GYRO_FS_250DPS  = LSM6DSV320X_250dps,   /* ±250  dps,  8.75  mdps/LSB */
    IMU_GYRO_FS_500DPS  = LSM6DSV320X_500dps,   /* ±500  dps, 17.50  mdps/LSB */
    IMU_GYRO_FS_1000DPS = LSM6DSV320X_1000dps,  /* ±1000 dps, 35.0   mdps/LSB */
    IMU_GYRO_FS_2000DPS = LSM6DSV320X_2000dps,  /* ±2000 dps, 70.0   mdps/LSB */
    IMU_GYRO_FS_4000DPS = LSM6DSV320X_4000dps,  /* ±4000 dps, 140.0  mdps/LSB */
    } IMU_GYRO_FS;

/**
 * @brief Accelerometer operating mode.
 *        DS14623 Rev3 Table 53: OP_MODE_XL[2:0] in CTRL1 (10h).
 *        When the high-G chain is active, the low-G chain MUST remain in
 *        high-performance mode (IMU_ACC_MODE_HP). DS14623 Rev3 §6.1.2.
 *        imu_set_power_mode() enforces this constraint automatically.
 */
typedef enum _IMU_ACC_MODE
    {
    IMU_ACC_MODE_HP     = LSM6DSV320X_XL_HIGH_PERFORMANCE_MD, /* High-performance (required for high-G) */
    IMU_ACC_MODE_NORMAL = LSM6DSV320X_XL_NORMAL_MD,           /* Normal mode                           */
    IMU_ACC_MODE_LP1    = LSM6DSV320X_XL_LOW_POWER_2_AVG_MD,  /* Low-power, 2-sample avg               */
    IMU_ACC_MODE_LP2    = LSM6DSV320X_XL_LOW_POWER_4_AVG_MD,  /* Low-power, 4-sample avg               */
    IMU_ACC_MODE_LP3    = LSM6DSV320X_XL_LOW_POWER_8_AVG_MD,  /* Low-power, 8-sample avg               */
    } IMU_ACC_MODE;

/**
 * @brief Gyroscope operating mode.
 *        DS14623 Rev3 Table 56: OP_MODE_G[2:0] in CTRL2 (11h).
 */
typedef enum _IMU_GYRO_MODE
    {
    IMU_GYRO_MODE_HP    = LSM6DSV320X_GY_HIGH_PERFORMANCE_MD, /* High-performance */
    IMU_GYRO_MODE_SLEEP = LSM6DSV320X_GY_SLEEP_MD,            /* Sleep mode       */
    IMU_GYRO_MODE_LP    = LSM6DSV320X_GY_LOW_POWER_MD,        /* Low-power mode   */
    } IMU_GYRO_MODE;

/**
 * @brief User configuration passed to imu_init().
 */
typedef struct _IMU_CONFIG
    {
    IMU_ODR       odr;           /* Loop-coupled ODR for Accel/Gyro/FIFO */
    IMU_ACC_FS    acc_fs;        /* Accelerometer full-scale range       */
    IMU_GYRO_FS   gyro_fs;       /* Gyroscope full-scale range           */
    IMU_ACC_MODE  acc_mode;      /* Accelerometer operating mode         */
    IMU_GYRO_MODE gyro_mode;     /* Gyroscope operating mode             */
    bool          sflp_enable;   /* Enable SFLP game rotation vector     */
    bool          fifo_enable;   /* Route outputs through FIFO (DMA path)*/
    } IMU_CONFIG;

/**
 * @brief Standard status return codes for all IMU driver operations.
 */
typedef enum _IMU_STATUS
    {
    IMU_OK               = 0,   /* Operation completed successfully         */
    IMU_FAIL,                   /* General failure                          */
    IMU_ERROR,                  /* SPI HAL or hardware error                */
    IMU_TIMEOUT,                /* SPI or DRDY poll timed out               */
    IMU_UNRECOGNIZED_ID,        /* WHO_AM_I did not return 0x73             */
    IMU_INIT_FAIL,              /* Initialization sequence failed           */
    IMU_CONFIG_FAIL,            /* Register configuration step failed       */
    IMU_BUSY,                   /* DMA transfer in progress, data not ready */
    IMU_NO_DATA,                /* FIFO empty or DRDY not set after timeout */
    } IMU_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes
------------------------------------------------------------------------------*/

/* --- System Control & Initialization --- */

/** 
 * @brief  Initializes the LSM6DSV320X IMU (Blocking).
 * @note   Performs SW reset, applies loop-coupled ODR / FS settings.
 *         Configures the PID facade and sets up Register Bank routing for SFLP.
 * @param  config: Pointer to user configuration struct.
 * @retval IMU_STATUS
 */
IMU_STATUS imu_init
    (
    const IMU_CONFIG* config
    );

/** 
 * @brief  Mid-flight Accelerometer Full-Scale transition (Runtime).
 * @note   Handles both same-chain FS changes and low-G ↔ high-G boundary
 *         crossings. On boundary crossing: reroutes FIFO batching, updates
 *         CTRL1_XL_HG / CTRL8, and enforces HP mode constraint (§6.1.2).
 *         No sensor reset required.
 * @param  new_fs: Target full-scale range.
 * @retval IMU_STATUS
 */
IMU_STATUS imu_set_accel_fs
    (
    IMU_ACC_FS new_fs
    );

/** 
 * @brief  Mid-flight Gyroscope Full-Scale transition (Runtime).
 * @param  new_fs: Target full-scale range.
 * @retval IMU_STATUS
 */
IMU_STATUS imu_set_gyro_fs
    (
    IMU_GYRO_FS new_fs
    );

/**
 * @brief  Changes accel and gyro operating mode at runtime (no reset).
 * @param  acc_mode:  Target accel power mode.
 * @param  gyro_mode: Target gyro power mode.
 * @retval IMU_STATUS
 */
IMU_STATUS imu_set_power_mode
    (
    IMU_ACC_MODE  acc_mode,
    IMU_GYRO_MODE gyro_mode
    );


/* --- Synchronous Data Interface (Blocking/Polled) --- */

/** 
 * @brief  Polled read of raw Accel + Gyro.
 * @note   Intended for pre-flight BIT and terminal sensor dump.
 * @param  raw: Pointer to raw counts buffer.
 * @retval IMU_STATUS
 */
IMU_STATUS imu_read_sync
    (
    IMU_RAW* raw
    );

/** 
 * @brief  Blocking SFLP/Quaternion retrieval.
 * @note   Handles internal ST bank-switching logic. Used for BIT/Debug.
 * @param  sflp: Pointer to SFLP output struct.
 * @retval IMU_STATUS
 */
IMU_STATUS imu_read_sflp_sync
    (
    IMU_SFLP_DATA* sflp
    );


/* --- Asynchronous Data Interface (Non-Blocking DMA) --- */

/** 
 * @brief  Triggers an SPI DMA read of the IMU FIFO.
 * @note   Offloads the SPI transfer to meet flight loop timing.
 *         Must be called after INT1 assertions (FIFO watermark).
 * @retval IMU_STATUS: IMU_OK, IMU_BUSY, or IMU_NO_DATA.
 */
IMU_STATUS imu_request_async
    (
    void
    );

/** 
 * @brief  DMA ISR Callback | call from HAL_SPI_TxRxCpltCallback().
  * @note   Runs in the DMA interrupt context. Handles hardware only (CS, 
  *         cache invalidation, flags) to keep ISR latency minimal.
 */
void imu_process_async_cb
    (
    void
    );

/** 
 * @brief  Polls the internal driver state to see if the DMA burst is complete. 
 * @retval bool: true if ready to be consumed.
 */
bool imu_has_new_data
    (
    void
    );

/** 
 * @brief  Retrieves and parses the latest DMA data. 
 * @note   Clears the internal ready flag. Copies data to caller [i.e. move FPU
 *         math out of interrupt]
 * @param  raw:  Pointer to raw counts buffer (may be NULL).
 * @param  sflp: Pointer to quaternion/fusion buffer (may be NULL).
 * @retval IMU_STATUS: IMU_OK or IMU_BUSY.
 */
IMU_STATUS imu_get_latest
    (
    IMU_RAW*       raw,
    IMU_SFLP_DATA* sflp
    );


/* --- Utilities --- */

/**
 * @brief  Scales raw INT16 counts to physical floating-point data.
 * @note   Uses internally cached sensitivity values updated by fs setters.
 * @param  raw:  Source raw counts.
 * @param  data: Destination scaled physical data.
 */
void imu_scale_raw
    (
    const IMU_RAW* raw,
    IMU_DATA*      data
    );


#ifdef __cplusplus
}
#endif

#endif /* IMU_T_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
