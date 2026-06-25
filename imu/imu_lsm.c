/**
  ******************************************************************************
  * @file           : imu_lsm.c
  * @brief          : LSM6DSV320X IMU Driver Implementation
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Facade layer over the ST lsm6dsv320x-pid platform-independent
  *        driver (lib/lsm6dsv320x-pid). The PID driver handles register
  *        encoding; this file provides the SPI callbacks, init sequence,
  *        blocking + DMA data retrieval, and runtime configuration setters.
  *
  *        SPI protocol (DS14623 Rev3 §5.1.3, Mode 3):
  *          - bit[7] of address byte: 1 = read, 0 = write
  *          - IF_INC (CTRL3 12h bit[2]) = 1 for auto-increment multi-byte
  *          - CS active-low, held for the entire transaction
  *          - __NOP() guards on CS toggle to satisfy setup/hold timing
  *
  *        DMA Execution Flow:
  *          1. INT1 (watermark) asserts -> Flight loop calls imu_request_async().
  *          2. HAL_SPI_TransmitReceive_DMA() starts non-blocking burst read.
  *          3. HAL_SPI_TxRxCpltCallback() calls imu_process_async_cb() inside ISR.
  *          4. imu_process_async_cb() de-asserts CS, invalidates D-Cache, sets imu_dma_ready.
  *          5. Flight loop calls imu_get_latest() to copy buffer in critical section.
  *          6. FIFO parsing is deferred to thread context to avoid FPU execution in ISR.
  *          7. On transaction failure, HAL_SPI_ErrorCallback() routes to imu_process_async_error_cb().
  *        
  *        D-Cache coherency (STM32H7):
  *          imu_dma_rx_buf is __ALIGNED(32) and sized to a multiple of 32 bytes
  *          (IMU_DMA_BUF_BYTES_ALIGNED). SCB_InvalidateDCache_by_Addr() in
  *          imu_process_async_cb() touches only those lines. Place these buffers
  *          in a non-cacheable MPU region OR rely on the explicit invalidation
  *          here (Don't do both).

  *          NOTE: these buffers must also live in DMA-accessible RAM (e.g. an
  *          AXI/D2 SRAM section) since DTCM not reachable by DMA1/DMA2.
  *
  *        FIFO quaternion packing (DS14623 Rev3 §13.23, sflp2q convention):
  *          - TAG=0x13 carries X, Y, Z as 16-bit half-floats (IEEE 754-2008).
  *          - w = sqrt(max(0, 1 − x² − y² − z²)).
  *          - One TAG=0x13 slot is self-contained (no state machine needed).
  *
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 Sun Devil Rocketry. All rights reserved.
  * This software is licensed under terms found in the LICENSE file in the root
  * directory of this component. If absent, BSD-3-Clause applies:
  * https://opensource.org/license/bsd-3-clause
  ******************************************************************************
  */

#if !defined( A0010 )
#error "imu_lsm.h is only supported on the A0010 hardware"
#endif

/*------------------------------------------------------------------------------
 Standard Includes
------------------------------------------------------------------------------*/
#include <string.h>
#include <math.h>


/*------------------------------------------------------------------------------
 MCU Pins
------------------------------------------------------------------------------*/
#include "sdr_pin_defines_A0010.h"


/*------------------------------------------------------------------------------
 Project Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu_lsm.h"
#include "error_sdr.h"   /* ERROR_CODE enums */
#include "debug_sdr.h"   /* debug_assert() - mod/debug_sdr/debug_sdr.h */

/*------------------------------------------------------------------------------
 Local Macros
------------------------------------------------------------------------------*/

/**
 * @brief Context-oriented assertion wrapper. Maps to debug_assert() with 
 *        the appropriate IMU initialization error code.
 */
#define IMU_ASSERT( expr )  debug_assert( (expr), ERROR_IMU_INIT_ERROR )

/**
 * @brief Capacity of the local SPI write transmit buffer.
 *        Sized to 64 bytes to safely accommodate address-byte overhead
 *        and maximum expected single payload writes (6-byte SFLP bias init).
 */
#define SPI_WRITE_TX_ARRAY_SIZE  ( 64U )

/** @brief Raw byte-array capacity for platform_spi_read()'s tx/rx buffers.   */
#define SPI_READ_RX_ARRAY_SIZE   ( 64U )


/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/

/** @brief ST PID driver context (holds SPI read/write function pointers)     */
static stmdev_ctx_t imu_ctx;

/** @brief Cached FS settings [required for imu_scale_raw() sensitivity lookup] */
static IMU_ACC_FS  imu_acc_fs;
static IMU_GYRO_FS imu_gyro_fs;

/** @brief Set true by imu_init() after SFLP is successfully enabled           */
static bool imu_sflp_enabled;

/**
 * @brief Cached init-time config needed by imu_set_accel_fs() to reroute the
 *        FIFO correctly when crossing the low-G / high-G boundary at runtime.
 */
static IMU_ODR imu_cached_odr;
static bool    imu_cached_fifo_en;

/**
 * @brief Cached power mode most recently requested via imu_init() /
 *        imu_set_power_mode(). DS14623 Rev3 §6.1.2 forces the low-G chain
 *        into high-performance mode whenever the high-G chain is active;
 *        imu_set_accel_fs() uses this cache to restore the user's actual
 *        requested mode after a High-G -> Low-G transition removes that
 *        constraint, instead of leaving the sensor stuck in HP mode.
 */
static IMU_ACC_MODE imu_cached_acc_mode;

/**
 * @brief SPI DMA buffers placed in D2 SRAM.
 *        - __ALIGNED(32): Prevents cache corruption during cache invalidation.
 *        - Section (.dma_buffer): Forces placement in RAM_D2 (0x30000000).
 *          DTCM (.bss) is not reachable by the DMA1/DMA2 controllers.
 *
 *        Linker requirements:
 *          .dma_buffer (NOLOAD) : { . = ALIGN(32); *(.dma_buffer); } >RAM_D2
 */
__ALIGNED(32) __attribute__((section(".dma_buffer")))
static uint8_t imu_dma_tx_buf[ IMU_DMA_BUF_BYTES_ALIGNED ];
__ALIGNED(32) __attribute__((section(".dma_buffer")))
static uint8_t imu_dma_rx_buf[ IMU_DMA_BUF_BYTES_ALIGNED ];

/** @brief Number of FIFO slots captured in the most recent DMA transaction    */
static volatile uint8_t imu_dma_slots_requested;

/**
 * @brief Synchronization flags.
 *        volatile: both are written in interrupt context (imu_process_async_cb)
 *        and read from task/loop context.
 */
static volatile bool imu_dma_busy  = false;
static volatile bool imu_dma_ready = false;


/*------------------------------------------------------------------------------
 Internal Function Prototypes
------------------------------------------------------------------------------*/

static int32_t platform_spi_write( void* handle, uint8_t reg,
                                   const uint8_t* buf, uint16_t len );
static int32_t platform_spi_read ( void* handle, uint8_t reg,
                                   uint8_t* buf,       uint16_t len );
static void    platform_delay_ms ( uint32_t ms );

static void    sflp2q            ( float quat[4], const uint16_t sflp[3] );
static void    parse_fifo_slot   ( const uint8_t* slot,
                                   IMU_RAW* raw_ptr, IMU_SFLP_DATA* sflp_ptr );
static lsm6dsv320x_data_rate_t map_odr( IMU_ODR odr );

/*------------------------------------------------------------------------------
 Internal Helpers
------------------------------------------------------------------------------*/

/**
  * @brief  Maps a high-level IMU_ODR enum to the PID library data-rate enum.
  * @note   Extracted from imu_init()'s ODR switch so that imu_set_accel_fs()
  *         and imu_set_power_mode() can re-use the same mapping when calling
  *         lsm6dsv320x_xl_setup() / lsm6dsv320x_gy_setup() with the cached ODR.
  */
static lsm6dsv320x_data_rate_t map_odr
    (
    IMU_ODR odr
    )
{
switch ( odr ) {
    case IMU_ODR_480HZ:  return LSM6DSV320X_ODR_AT_480Hz;
    case IMU_ODR_960HZ:  return LSM6DSV320X_ODR_AT_960Hz;
    case IMU_ODR_3840HZ: return LSM6DSV320X_ODR_AT_3840Hz;
    case IMU_ODR_7680HZ: return LSM6DSV320X_ODR_AT_7680Hz;
    case IMU_ODR_1920HZ:
    default:             return LSM6DSV320X_ODR_AT_1920Hz;
}
} /* map_odr */


/*------------------------------------------------------------------------------
 Platform Callbacks (ST PID driver interface)
------------------------------------------------------------------------------*/

/**
  * @brief  SPI write callback registered with the ST PID driver.
  * @note   DS14623 Rev3 §5.1.3.2: RW=0 in bit[7] of address byte.
  *         Fixed-size local array (no VLA) avoids MISRA C issues and stack
  *         uncertainty on Cortex-M7. 64-byte ceiling is conservative; the
  *         largest expected single write is 6 bytes (SFLP gbias init).
  *         __NOP() guards satisfy CS setup/hold time at high SPI clock rates.
  * @param  handle: Unused (imu_ctx.handle is NULL).
  * @param  reg:    7-bit register address.
  * @param  buf:    Data bytes to write.
  * @param  len:    Number of data bytes (address byte excluded).
  * @retval 0 on success, 1 on HAL error or oversized request.
  */
static int32_t platform_spi_write
    (
    void*          handle,
    uint8_t        reg,
    const uint8_t* buf,
    uint16_t       len
    )
{
uint8_t           tx[ SPI_WRITE_TX_ARRAY_SIZE ];
HAL_StatusTypeDef hal_status;

(void)handle;

/*
 * Precondition: caller must never request a write larger than the fixed
 * transmit array. In practice the largest PID write is 6 bytes (SFLP gbias)
 * so this would only fire if new PID functionality were added without
 * increasing SPI_WRITE_TX_ARRAY_SIZE.
 */
IMU_ASSERT( ( (uint32_t)len + 1U ) <= SPI_WRITE_TX_ARRAY_SIZE );

/* Graceful fallback in release builds where the assert compiles out */
if ( ( (uint32_t)len + 1U ) > SPI_WRITE_TX_ARRAY_SIZE ) {
    return 1;
}

/* RW = 0 for write: bit[7] clear (DS14623 §5.1.3) */
tx[0] = reg & (uint8_t)( ~IMU_SPI_READ_BIT );
memcpy( &tx[1], buf, len );

HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_RESET );
__NOP(); __NOP();
hal_status = HAL_SPI_Transmit( &IMU_SPI, tx, (uint16_t)( len + 1U ),
                                IMU_SPI_TIMEOUT_MS );
__NOP(); __NOP();
HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_SET );

if ( hal_status == HAL_OK ) {
    return 0;
}
return 1;
} /* platform_spi_write */


/**
  * @brief  SPI read callback registered with the ST PID driver.
  * @note   DS14623 Rev3 §5.1.3.1: RW=1 in bit[7] of address byte.
  *         HAL_SPI_TransmitReceive clocks the address byte out while rx[0]
  *         captures a dummy byte; useful data starts at rx[1].
  * @param  handle: Unused.
  * @param  reg:    7-bit register address.
  * @param  buf:    Buffer to receive read data.
  * @param  len:    Number of bytes to read.
  * @retval 0 on success, 1 on HAL error or oversized request.
  */
static int32_t platform_spi_read
    (
    void*    handle,
    uint8_t  reg,
    uint8_t* buf,
    uint16_t len
    )
{
uint8_t           tx[ SPI_READ_RX_ARRAY_SIZE ] = { 0 };
uint8_t           rx[ SPI_READ_RX_ARRAY_SIZE ] = { 0 };
HAL_StatusTypeDef hal_status;

(void)handle;

/* Same overflow guard as platform_spi_write */
IMU_ASSERT( ( (uint32_t)len + 1U ) <= SPI_READ_RX_ARRAY_SIZE );

if ( ( (uint32_t)len + 1U ) > SPI_READ_RX_ARRAY_SIZE ) {
    return 1;
}

/* RW = 1 for read: bit[7] set (DS14623 §5.1.3).
   Zero-initialized at declaration to supply dummy bytes. */
tx[0] = reg | IMU_SPI_READ_BIT;

HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_RESET );
__NOP(); __NOP();
hal_status = HAL_SPI_TransmitReceive( &IMU_SPI, tx, rx,
                                      (uint16_t)( len + 1U ),
                                      IMU_SPI_TIMEOUT_MS );
__NOP(); __NOP();
HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_SET );

if ( hal_status == HAL_OK ) {
    memcpy( buf, &rx[1], len ); /* Skip dummy first byte */
    return 0;
}
return 1;
} /* platform_spi_read */


/**
  * @brief  Millisecond delay registered with the ST PID driver.
  */
static void platform_delay_ms
    (
    uint32_t ms
    )
{
HAL_Delay( ms );
} /* platform_delay_ms */


/**
  * @brief  Converts three half-float SFLP components to a unit quaternion.
  * @note   ST packs only X, Y, Z into a TAG=0x13 FIFO slot; W is defined:
  *         w = sqrt(1 − x² − y² − z²) [unit quaterinion]
  *         If numerical noise causes |xyz|² > 1, XYZ is normalised first so
  *         the output remains a valid unit quaternion.
  *         Uses lsm6dsv320x_from_quaternion_lsb_to_float() 
  * @param  quat:  Output [x, y, z, w] indexed [0..3].
  * @param  sflp:  Three raw half-float values from the FIFO slot.
  */
static void sflp2q
    (
    float          quat[4],
    const uint16_t sflp[3]
    )
{
float   sumsq;
uint8_t i;

quat[0] = lsm6dsv320x_from_quaternion_lsb_to_float( sflp[0] ); /* x */
quat[1] = lsm6dsv320x_from_quaternion_lsb_to_float( sflp[1] ); /* y */
quat[2] = lsm6dsv320x_from_quaternion_lsb_to_float( sflp[2] ); /* z */

sumsq = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2];

if ( sumsq > 1.0f ) {
    /* Normalise xyz so W stays real */
    float n = sqrtf( sumsq );
    for ( i = 0U; i < 3U; i++ ) {
        quat[i] /= n;
    }
    sumsq = 1.0f;
}

quat[3] = sqrtf( 1.0f - sumsq ); /* w */
} /* sflp2q */


/**
  * @brief  Parses a single 7-byte FIFO slot into the caller's output structs.
  * @note   FIFO slot layout (DS14623 Rev3 Table 232):
  *           byte[0]  = TAG[7:3] | parity/counter[2:0]
  *           byte[1]  = X_L,  byte[2]  = X_H
  *           byte[3]  = Y_L,  byte[4]  = Y_H
  *           byte[5]  = Z_L,  byte[6]  = Z_H
  *         TAG=0x01 Gyro NC, TAG=0x02 Accel NC (low-G), TAG=0x1D High-G Accel,
  *         TAG=0x13 SFLP game rotation vector (half-float XYZ, W derived),
  *         TAG=0x16 SFLP gyro bias,  TAG=0x17 SFLP gravity vector.
  * @param  slot:     Pointer to first byte of the 7-byte FIFO slot.
  * @param  raw_ptr:  Destination for accel/gyro raw counts (may be NULL).
  * @param  sflp_ptr: Destination for SFLP fusion data   (may be NULL).
  */
static void parse_fifo_slot
    (
    const uint8_t* slot,
    IMU_RAW*       raw_ptr,
    IMU_SFLP_DATA* sflp_ptr
    )
{
uint8_t  tag;
uint16_t x_raw;
uint16_t y_raw;
uint16_t z_raw;

IMU_ASSERT( slot != NULL );

/* TAG_SENSOR[4:0] lives in bits[7:3] of the tag byte */
tag = ( slot[0] >> 3U ) & 0x1FU;

/* Raw 16-bit words from the 6 data bytes (LSB first) */
x_raw = ( (uint16_t)slot[2] << 8U ) | slot[1];
y_raw = ( (uint16_t)slot[4] << 8U ) | slot[3];
z_raw = ( (uint16_t)slot[6] << 8U ) | slot[5];

switch ( tag ) {
    case IMU_FIFO_TAG_GYRO:
        if ( raw_ptr != NULL ) {
            raw_ptr->gyro_x = (int16_t)x_raw;
            raw_ptr->gyro_y = (int16_t)y_raw;
            raw_ptr->gyro_z = (int16_t)z_raw;
        }
        break;

    case IMU_FIFO_TAG_ACCEL:
        if ( raw_ptr != NULL ) {
            raw_ptr->accel_x = (int16_t)x_raw;
            raw_ptr->accel_y = (int16_t)y_raw;
            raw_ptr->accel_z = (int16_t)z_raw;
        }
        break;

    case IMU_FIFO_TAG_HG_ACCEL:
        /*
         * High-g accelerometer data (TAG=0x1D). DS14623 Rev3 Table 232.
         * Same 16-bit two's complement format as the low-G accel slot.
         * Batched in FIFO when XL_HG_BATCH_EN (COUNTER_BDR_REG1 0Bh) = 1.
         * Overwrites accel fields so imu_scale_raw() uses the correct
         * high-G sensitivity for whichever sample arrived last.
         */
        if ( raw_ptr != NULL ) {
            raw_ptr->accel_x = (int16_t)x_raw;
            raw_ptr->accel_y = (int16_t)y_raw;
            raw_ptr->accel_z = (int16_t)z_raw;
        }
        break;

    case IMU_FIFO_TAG_SFLP_QUAT:
        /*
         * ST packs quaternion X, Y, Z as three consecutive half-floats.
         * W is derived from the unit-quaternion constraint in sflp2q().
         * This slot is self-contained - no two-slot state machine needed.
         */
        if ( sflp_ptr != NULL ) {
            uint16_t sflp_raw[3] = { x_raw, y_raw, z_raw };
            float quat[4];
            sflp2q( quat, sflp_raw );
            sflp_ptr->quat_x = quat[0];
            sflp_ptr->quat_y = quat[1];
            sflp_ptr->quat_z = quat[2];
            sflp_ptr->quat_w = quat[3];
        }
        break;

    case IMU_FIFO_TAG_SFLP_GRAV:
        if ( sflp_ptr != NULL ) {
            sflp_ptr->grav_x = lsm6dsv320x_from_quaternion_lsb_to_float( x_raw );
            sflp_ptr->grav_y = lsm6dsv320x_from_quaternion_lsb_to_float( y_raw );
            sflp_ptr->grav_z = lsm6dsv320x_from_quaternion_lsb_to_float( z_raw );
        }
        break;

    case IMU_FIFO_TAG_SFLP_GBIAS:
        if ( sflp_ptr != NULL ) {
            sflp_ptr->gbias_x = lsm6dsv320x_from_quaternion_lsb_to_float( x_raw );
            sflp_ptr->gbias_y = lsm6dsv320x_from_quaternion_lsb_to_float( y_raw );
            sflp_ptr->gbias_z = lsm6dsv320x_from_quaternion_lsb_to_float( z_raw );
        }
        break;

    default:
        /* Unknown or unhandled tag - silently skip */
        break;
    }
} /* parse_fifo_slot */


/*------------------------------------------------------------------------------
 Public API
------------------------------------------------------------------------------*/

/**
  * @brief  Initializes the LSM6DSV320X IMU (Blocking).
  * @note   Sequence:
  *           1. Wire PID callbacks.
  *           2. Disable INT1 EXTI (prevents spurious ISR during config).
  *           3. SW reset via lsm6dsv320x_sw_reset() (DS14623 Rev3 §9.16).
  *           4. WHO_AM_I verification.
  *           5. Set ODR, full-scale, power mode, BDU, IF_INC.
  *           6. Configure SFLP (optional).
  *           7. Configure FIFO (optional, continuous mode).
  *           8. Re-enable INT1 EXTI.
  *
  *        ODR is set once here and not changed at runtime. The FIFO batch rate
  *        (BDR) is kept sync'd to the sensor ODR to prevent timing jitter.
  *
  *        High-G modes (±32 g through ±320 g) use a separate sensing chain
  *        configured via CTRL1_XL_HG (4Eh). When a High-G enum is selected, 
  *        imu_init() handles this alternate routing.
  *
  *        IMPORTANT: imu_request_async() must NOT be called from an ISR.
  *        It issues a blocking SPI transaction to read the FIFO level. Call
  *        it from the flight loop.
  *
  * @param  config: Pointer to user configuration struct (non-NULL).
  * @retval IMU_STATUS
  */
IMU_STATUS imu_init
    (
    const IMU_CONFIG* config
    )
{
/*--------------------------------------------------------------------------
 Local variables
--------------------------------------------------------------------------*/
uint8_t                     who_am_i   = 0U;
int32_t                     pid_status = 0;
lsm6dsv320x_pin_int_route_t int1_route = { 0 };

lsm6dsv320x_data_rate_t     xl_odr;
lsm6dsv320x_data_rate_t     g_odr;
lsm6dsv320x_xl_mode_t       xl_mode;
lsm6dsv320x_gy_mode_t       gy_mode;
lsm6dsv320x_fifo_xl_batch_t xl_bdr;
lsm6dsv320x_fifo_gy_batch_t g_bdr;
IMU_ACC_MODE                effective_acc_mode;


/*--------------------------------------------------------------------------
 Precondition checks
--------------------------------------------------------------------------*/
IMU_ASSERT( config != NULL );

/*--------------------------------------------------------------------------
 Initializations
--------------------------------------------------------------------------*/
imu_sflp_enabled    = false;
imu_dma_ready       = false;
imu_dma_busy        = false;
imu_cached_odr      = config->odr;
imu_cached_fifo_en  = config->fifo_enable;
imu_cached_acc_mode = config->acc_mode;

memset( imu_dma_tx_buf, 0, sizeof( imu_dma_tx_buf ) );
memset( imu_dma_rx_buf, 0, sizeof( imu_dma_rx_buf ) );

/* Wire PID callbacks into stmdev_ctx_t */
imu_ctx.write_reg = platform_spi_write;
imu_ctx.read_reg  = platform_spi_read;
imu_ctx.mdelay    = platform_delay_ms;
imu_ctx.handle    = NULL;


/*--------------------------------------------------------------------------
 Implementation
--------------------------------------------------------------------------*/

/* Disable INT1 EXTI to prevent accidental ISR triggers during init.
   INT1 is routed to EXTI4 (PC4 = IMU_INT1_PIN). */
HAL_NVIC_DisableIRQ( EXTI4_IRQn );

/* SW reset: all control registers restored to datasheet defaults.
   lsm6dsv320x_sw_reset() polls until SW_RESET self-clears (up to 3 ms).
   DS14623 Rev3 §9.16. */
pid_status = lsm6dsv320x_sw_reset( &imu_ctx );
if ( pid_status != 0 ) { return IMU_INIT_FAIL; }
platform_delay_ms( 10U );

/* Verify device identity - WHO_AM_I (0Fh) must read 0x73. DS14623 §9.13. */
pid_status = lsm6dsv320x_device_id_get( &imu_ctx, &who_am_i );
if ( pid_status != 0 )               { return IMU_ERROR;           }
if ( who_am_i != IMU_WHO_AM_I_VAL )  { return IMU_UNRECOGNIZED_ID; }

/*
 * Resolve ODR and FIFO batch rate.
 * Accel ODR (CTRL1 10h), Gyro ODR (CTRL2 11h), and FIFO Batch rates
 * (FIFO_CTRL3 09h) are kept synchronised to the same target rate.
 * DS14623 Rev3 Tables 54, 57, 232.
 *
 * NOTE: the High-G ODR field (CTRL1_XL_HG 4Eh bits[5:3]) uses a separate
 * 3-bit encoding (Table 149) unrelated to the standard 4-bit XL ODR field.
 * That is handled in the acc_fs >= 32G branch below.
 */
switch ( config->odr ) {
    case IMU_ODR_480HZ:
        xl_odr = LSM6DSV320X_ODR_AT_480Hz;
        g_odr  = LSM6DSV320X_ODR_AT_480Hz;
        xl_bdr = LSM6DSV320X_XL_BATCHED_AT_480Hz;
        g_bdr  = LSM6DSV320X_GY_BATCHED_AT_480Hz;
        break;
    case IMU_ODR_960HZ:
        xl_odr = LSM6DSV320X_ODR_AT_960Hz;
        g_odr  = LSM6DSV320X_ODR_AT_960Hz;
        xl_bdr = LSM6DSV320X_XL_BATCHED_AT_960Hz;
        g_bdr  = LSM6DSV320X_GY_BATCHED_AT_960Hz;
        break;
    case IMU_ODR_3840HZ:
        xl_odr = LSM6DSV320X_ODR_AT_3840Hz;
        g_odr  = LSM6DSV320X_ODR_AT_3840Hz;
        xl_bdr = LSM6DSV320X_XL_BATCHED_AT_3840Hz;
        g_bdr  = LSM6DSV320X_GY_BATCHED_AT_3840Hz;
        break;
    case IMU_ODR_7680HZ:
        xl_odr = LSM6DSV320X_ODR_AT_7680Hz;
        g_odr  = LSM6DSV320X_ODR_AT_7680Hz;
        xl_bdr = LSM6DSV320X_XL_BATCHED_AT_7680Hz;
        g_bdr  = LSM6DSV320X_GY_BATCHED_AT_7680Hz;
        break;
    case IMU_ODR_1920HZ: /* fall-through to default */
    default:
        /*
         * Any value reaching this branch should be exactly IMU_ODR_1920HZ.
         * If it is not, an out-of-range enum was passed in config->odr.
         * Caught in debug builds; silently defaults to 1920 Hz in release.
         */
        IMU_ASSERT( config->odr == IMU_ODR_1920HZ );
        xl_odr = LSM6DSV320X_ODR_AT_1920Hz;
        g_odr  = LSM6DSV320X_ODR_AT_1920Hz;
        xl_bdr = LSM6DSV320X_XL_BATCHED_AT_1920Hz;
        g_bdr  = LSM6DSV320X_GY_BATCHED_AT_1920Hz;
        break;
    }

/*
 * Resolve accel operating mode.
 * DS14623 Rev3 §6.1.2: the low-G chain must be in high-performance mode
 * whenever the high-G chain is active. Silently override here so the caller
 * doesn't need to know about this constraint.
 */
effective_acc_mode = ( config->acc_fs >= IMU_ACC_FS_32G )
                   ? IMU_ACC_MODE_HP
                   : config->acc_mode;

switch ( effective_acc_mode ) {
    case IMU_ACC_MODE_NORMAL: xl_mode = LSM6DSV320X_XL_NORMAL_MD;           break;
    case IMU_ACC_MODE_LP1:    xl_mode = LSM6DSV320X_XL_LOW_POWER_2_AVG_MD;  break;
    case IMU_ACC_MODE_LP2:    xl_mode = LSM6DSV320X_XL_LOW_POWER_4_AVG_MD;  break;
    case IMU_ACC_MODE_LP3:    xl_mode = LSM6DSV320X_XL_LOW_POWER_8_AVG_MD;  break;
    case IMU_ACC_MODE_HP:
    default:                  xl_mode = LSM6DSV320X_XL_HIGH_PERFORMANCE_MD; break;
}

gy_mode = (lsm6dsv320x_gy_mode_t)config->gyro_mode;

/*
 * Apply ODR + mode using lsm6dsv320x_xl_setup() / lsm6dsv320x_gy_setup().
 * These are the non-deprecated successors to the separate _data_rate_set()
 * + _mode_set() pairs. Combining ODR and mode in one call also lets the
 * library validate their compatibility per AN6119 Tables 10/13 (e.g.
 * 1.875 Hz is low-power only).
 * DS14623 Rev3 CTRL1 (10h), CTRL2 (11h).
 */
pid_status |= lsm6dsv320x_xl_setup( &imu_ctx, xl_odr, xl_mode );
pid_status |= lsm6dsv320x_gy_setup( &imu_ctx, g_odr,  gy_mode  );

/*
 * Full-scale configuration - two separate accelerometer sensing chains.
 *
 * Low-G chain (±2/4/8/16 g):
 *   Standard control registers via lsm6dsv320x_xl_full_scale_set().
 *
 * High-G chain (±32..320 g):
 *   CTRL1_XL_HG (4Eh) only. The low-G chain must remain ON for SFLP
 *   (§2.8), forced to ±16g HP mode (§6.1.2).
 */
if ( config->acc_fs >= IMU_ACC_FS_32G ) {
    /*
     * High-G chain: build and write CTRL1_XL_HG (4Eh) directly.
     * Register layout: [XL_HG_REGOUT_EN][HG_USR_OFF_ON_OUT][ODR_HG_2][ODR_HG_1]
     *                  [ODR_HG_0][FS_HG_2][FS_HG_1][FS_HG_0]
     *   bit[7]    = XL_HG_REGOUT_EN = 1 (expose to output registers)
     *   bits[5:3] = ODR_XL_HG[2:0]  : Table 149 3-bit code
     *   bits[2:0] = FS_XL_HG[2:0]   : Table 150 3-bit code
     */
    uint8_t hg_odr_code;
    uint8_t hg_fs_code;
    uint8_t ctrl1_xl_hg;

    /* Table 149: ODR_XL_HG[2:0] codes */
    switch ( config->odr ) {
        case IMU_ODR_480HZ:  hg_odr_code = 0x03U; break; /* 011 = 480 Hz  */
        case IMU_ODR_960HZ:  hg_odr_code = 0x04U; break; /* 100 = 960 Hz  */
        case IMU_ODR_3840HZ: hg_odr_code = 0x06U; break; /* 110 = 3.84kHz */
        case IMU_ODR_7680HZ: hg_odr_code = 0x07U; break; /* 111 = 7.68kHz */
        case IMU_ODR_1920HZ:                             /* fall-through  */
        default:             hg_odr_code = 0x05U; break; /* 101 = 1.92kHz */
    }

    /* Table 150: FS_XL_HG[2:0] codes */
    switch ( config->acc_fs ) {
        case IMU_ACC_FS_64G:  hg_fs_code = 0x01U; break;
        case IMU_ACC_FS_128G: hg_fs_code = 0x02U; break;
        case IMU_ACC_FS_256G: hg_fs_code = 0x03U; break;
        case IMU_ACC_FS_320G: hg_fs_code = 0x04U; break;
        case IMU_ACC_FS_32G:  /* fall-through */
        default:              hg_fs_code = 0x00U; break;
    }

    ctrl1_xl_hg = (uint8_t)( 0x80U                         /* XL_HG_REGOUT_EN */
                            | ( hg_odr_code << 3U )        /* ODR_XL_HG[2:0]  */
                            | ( hg_fs_code  & 0x07U ) );   /* FS_XL_HG[2:0]   */

    pid_status |= lsm6dsv320x_write_reg( &imu_ctx,
                                         LSM6DSV320X_CTRL1_XL_HG,
                                         &ctrl1_xl_hg, 1 );

    /* Force low-G chain to ±16g. DS14623 Rev3 §6.1.2. */
    pid_status |= lsm6dsv320x_xl_full_scale_set( &imu_ctx, LSM6DSV320X_16g );

    /* Enable XL_HG_BATCH_EN in COUNTER_BDR_REG1 (0Bh) bit[5]
       so high-G data is routed to the FIFO. DS14623 Rev3 Table 44.
       Read-modify-write to preserve the other bits (TRIG_COUNTER_BDR, CNT_BDR). */
    if ( config->fifo_enable ) {
        uint8_t bdr_reg = 0U;
        pid_status |= lsm6dsv320x_read_reg( &imu_ctx,
                                            LSM6DSV320X_COUNTER_BDR_REG1,
                                            &bdr_reg, 1 );
        bdr_reg |= (uint8_t)( 1U << 5U ); /* XL_HG_BATCH_EN = bit[5] */
        pid_status |= lsm6dsv320x_write_reg( &imu_ctx,
                                             LSM6DSV320X_COUNTER_BDR_REG1,
                                             &bdr_reg, 1 );
    }
} else {
    /*
     * Low-G chain: map IMU_ACC_FS enum to lsm6dsv320x_xl_full_scale_t
     * register code. 
     * DS14623 Rev3 Table 70.
     */
    lsm6dsv320x_xl_full_scale_t xl_fs;
    switch ( config->acc_fs ) {
        case IMU_ACC_FS_4G:  xl_fs = LSM6DSV320X_4g;  break;
        case IMU_ACC_FS_8G:  xl_fs = LSM6DSV320X_8g;  break;
        case IMU_ACC_FS_16G: xl_fs = LSM6DSV320X_16g; break;
        case IMU_ACC_FS_2G:  /* fall-through */
        default:             xl_fs = LSM6DSV320X_2g;  break;
    }
    pid_status |= lsm6dsv320x_xl_full_scale_set( &imu_ctx, xl_fs );
}

/* Gyro FS: map IMU_GYRO_FS to lsm6dsv320x_gy_full_scale_t. DS14623 Table 65. */
{
    lsm6dsv320x_gy_full_scale_t gy_fs;
    switch ( config->gyro_fs ) {
        case IMU_GYRO_FS_500DPS:  gy_fs = LSM6DSV320X_500dps;  break;
        case IMU_GYRO_FS_1000DPS: gy_fs = LSM6DSV320X_1000dps; break;
        case IMU_GYRO_FS_2000DPS: gy_fs = LSM6DSV320X_2000dps; break;
        case IMU_GYRO_FS_4000DPS: gy_fs = LSM6DSV320X_4000dps; break;
        case IMU_GYRO_FS_250DPS:  /* fall-through */
        default:                  gy_fs = LSM6DSV320X_250dps;  break;
    }
    pid_status |= lsm6dsv320x_gy_full_scale_set( &imu_ctx, gy_fs );
}

/* Cache FS for imu_scale_raw() sensitivity lookup */
imu_acc_fs  = config->acc_fs;
imu_gyro_fs = config->gyro_fs;

/*
 * BDU and IF_INC via dedicated PID wrappers (read-modify-write internally).
 * DS14623 Rev3 CTRL3 (12h), Table 59.
 */
pid_status |= lsm6dsv320x_block_data_update_set( &imu_ctx, PROPERTY_ENABLE );
pid_status |= lsm6dsv320x_auto_increment_set( &imu_ctx, PROPERTY_ENABLE );

if ( pid_status != 0 ) { return IMU_CONFIG_FAIL; }


/* -----------------------------------------------------------------------
   SFLP (Sensor Fusion Low-Power) [optional]
   lsm6dsv320x_sflp_game_rotation_set() internally opens FUNC_CFG_ACCESS
   (01h), sets SFLP_GAME_EN in EMB_FUNC_EN_A (04h), then closes the bank.
   DS14623 Rev3 §2.8, §13.1.
   ----------------------------------------------------------------------- */
if ( config->sflp_enable ) {
    lsm6dsv320x_sflp_gbias_t gbias = { 0 };

    pid_status  = lsm6dsv320x_sflp_game_rotation_set( &imu_ctx, PROPERTY_ENABLE );
    pid_status |= lsm6dsv320x_sflp_data_rate_set( &imu_ctx, IMU_SFLP_ODR );

    /* Seed gyro bias to zero; SFLP estimates the true bias at runtime */
    gbias.gbias_x = 0.0f;
    gbias.gbias_y = 0.0f;
    gbias.gbias_z = 0.0f;
    pid_status |= lsm6dsv320x_sflp_game_gbias_set( &imu_ctx, &gbias );

    if ( pid_status != 0 ) { return IMU_CONFIG_FAIL; }

    imu_sflp_enabled = true;
}


/* -----------------------------------------------------------------------
 * FIFO Configuration (Optional; required only for the DMA async path):
 * - BDR (Batching Data Rate) is configured to match the selected ODR.
 * - FIFO_MODE is set to continuous/stream mode (LSM6DSV320X_STREAM_MODE, 110b).
 * - Watermark is dynamically calculated from enabled slot types. (INT1 fires per
 *   finished sample group when SFLP = raw ODR @ same rate (480Hz))
 * - At higher raw ODRs (e.g., 1.92 kHz), SFLP is capped at 480 Hz and arrives
 *   less frequently. See IMU_DMA_BUF_SLOTS in imu_lsm.h for how the DMA buffer
 *   is sized to handle and drain the resulting backlog.
 *
 * References (Datasheet DS14623 Rev 3):
 * - Table 36: "1 LSB = TAG (1 byte) + 1 sensor (6 bytes) written in FIFO."
 * - Sections 9.7–9.8: FIFO_CTRL3 (09h), FIFO_CTRL4 (0Ah).
   ----------------------------------------------------------------------- */
if ( config->fifo_enable ) {
    lsm6dsv320x_fifo_sflp_raw_t sflp_batch  = { 0 };
    uint8_t                      watermark   = 0U;

    /*
     * Slot count per sample group:
     *   1 Gyro slot always.
     *   1 Accel slot - either low-G (TAG=0x02) or high-G (TAG=0x1D).
     *   When high-G is active, explicitly disable low-G FIFO batching so
     *   only one accel slot appears per group. Both low-G and high-G
     *   batching enabled simultaneously would push two accel slots per
     *   frame, waste DMA bandwidth, and cause the second to silently
     *   overwrite the first in parse_fifo_slot(). The low-G sensor itself
     *   must remain ON for SFLP (§2.8), it just must not batch to FIFO.
     */
    if ( config->acc_fs >= IMU_ACC_FS_32G ) {
        /* High-G path: suppress low-G FIFO batching */
        pid_status  = lsm6dsv320x_fifo_xl_batch_set( &imu_ctx,
                          LSM6DSV320X_XL_NOT_BATCHED );
    } else {
        /* Low-G path: batch as normal */
        pid_status  = lsm6dsv320x_fifo_xl_batch_set( &imu_ctx, xl_bdr );
    }

    pid_status |= lsm6dsv320x_fifo_gy_batch_set( &imu_ctx, g_bdr );

    watermark = 2U; /* 1 gyro + 1 accel */

    /* SFLP adds 3 slots per group: Quat (0x13), Grav (0x17), Gbias (0x16) */
    if ( imu_sflp_enabled ) {
        sflp_batch.game_rotation = 1U;
        sflp_batch.gravity       = 1U;
        sflp_batch.gbias         = 1U;
        pid_status |= lsm6dsv320x_fifo_sflp_batch_set( &imu_ctx, sflp_batch );
        watermark += 3U; /* + Quat, Grav, Gbias */
    }

    pid_status |= lsm6dsv320x_fifo_watermark_set( &imu_ctx, watermark );
    pid_status |= lsm6dsv320x_fifo_mode_set( &imu_ctx, LSM6DSV320X_STREAM_MODE );

    /* Route FIFO watermark to INT1. DS14623 Rev3 INT1_CTRL (0Dh) §9.11. */
    int1_route.fifo_th = PROPERTY_ENABLE;
    pid_status |= lsm6dsv320x_pin_int1_route_set( &imu_ctx, &int1_route );

    if ( pid_status != 0 ) { return IMU_CONFIG_FAIL; }
}

HAL_NVIC_EnableIRQ( EXTI4_IRQn );

return IMU_OK;
} /* imu_init */


/**
  * @brief  Mid-flight Accelerometer Full-Scale transition (Runtime).
  * @note   Handles both same-chain FS changes and low-G ↔ high-G boundary
  *         crossings. On crossing from low-G to high-G: writes CTRL1_XL_HG
  *         (4Eh), forces low-G to ±16g HP mode (§6.1.2), and reroutes FIFO
  *         batching (XL_HG_BATCH_EN on, low-G batch off). On crossing from
  *         high-G to low-G: clears XL_HG_REGOUT_EN, restores low-G batching,
  *         and restores whichever accel power mode the user last requested
  *         via imu_init() / imu_set_power_mode() (the §6.1.2 forced-HP
  *         constraint no longer applies once the high-G chain is off).
  *         DS14623 Rev3 §6.1.2, Tables 70, 149, 150; COUNTER_BDR_REG1 (0Bh).
  */
IMU_STATUS imu_set_accel_fs
    (
    IMU_ACC_FS new_fs
    )
{
int32_t    pid_status    = 0;
bool       current_is_hg = ( imu_acc_fs >= IMU_ACC_FS_32G );
bool       new_is_hg     = ( new_fs     >= IMU_ACC_FS_32G );

if ( new_is_hg ) {
    uint8_t hg_odr_code;
    uint8_t hg_fs_code;
    uint8_t ctrl1_xl_hg;

    /* Table 149: ODR_XL_HG[2:0] codes */
    switch ( imu_cached_odr ) {
        case IMU_ODR_480HZ:  hg_odr_code = 0x03U; break;
        case IMU_ODR_960HZ:  hg_odr_code = 0x04U; break;
        case IMU_ODR_3840HZ: hg_odr_code = 0x06U; break;
        case IMU_ODR_7680HZ: hg_odr_code = 0x07U; break;
        case IMU_ODR_1920HZ: /* fall-through */
        default:
            IMU_ASSERT( imu_cached_odr == IMU_ODR_1920HZ );
            hg_odr_code = 0x05U;
            break;
    }

    /* Table 150: FS_XL_HG[2:0] codes */
    switch ( new_fs ) {
        case IMU_ACC_FS_64G:  hg_fs_code = 0x01U; break;
        case IMU_ACC_FS_128G: hg_fs_code = 0x02U; break;
        case IMU_ACC_FS_256G: hg_fs_code = 0x03U; break;
        case IMU_ACC_FS_320G: hg_fs_code = 0x04U; break;
        case IMU_ACC_FS_32G:  /* fall-through */
        default:              hg_fs_code = 0x00U; break;
    }

    ctrl1_xl_hg = (uint8_t)( 0x80U                        /* XL_HG_REGOUT_EN */
                            | ( hg_odr_code << 3U )       /* ODR_XL_HG[2:0]  */
                            | ( hg_fs_code  & 0x07U ) );  /* FS_XL_HG[2:0]   */

    pid_status |= lsm6dsv320x_write_reg( &imu_ctx,
                                         LSM6DSV320X_CTRL1_XL_HG,
                                         &ctrl1_xl_hg, 1 );

    /* Boundary crossing: low-G -> high-G */
    if ( !current_is_hg ) {
        /* §6.1.2: force low-G to ±16g HP mode.
           Use xl_setup() to update both FS and mode atomically. */
        pid_status |= lsm6dsv320x_xl_full_scale_set( &imu_ctx, LSM6DSV320X_16g );
        pid_status |= lsm6dsv320x_xl_setup( &imu_ctx,
                          map_odr( imu_cached_odr ),
                          LSM6DSV320X_XL_HIGH_PERFORMANCE_MD );

        if ( imu_cached_fifo_en ) {
            uint8_t bdr_reg = 0U;
            pid_status |= lsm6dsv320x_read_reg( &imu_ctx,
                                                LSM6DSV320X_COUNTER_BDR_REG1,
                                                &bdr_reg, 1 );
            bdr_reg |= (uint8_t)( 1U << 5U ); /* XL_HG_BATCH_EN = 1 */
            pid_status |= lsm6dsv320x_write_reg( &imu_ctx,
                                                 LSM6DSV320X_COUNTER_BDR_REG1,
                                                 &bdr_reg, 1 );
            pid_status |= lsm6dsv320x_fifo_xl_batch_set( &imu_ctx,
                              LSM6DSV320X_XL_NOT_BATCHED );
        }
    }
} else {
    /* Low-G: explicit mapping to lsm6dsv320x_xl_full_scale_t. DS14623 Table 70. */
    lsm6dsv320x_xl_full_scale_t xl_fs;
    switch ( new_fs ) {
        case IMU_ACC_FS_4G:  xl_fs = LSM6DSV320X_4g;  break;
        case IMU_ACC_FS_8G:  xl_fs = LSM6DSV320X_8g;  break;
        case IMU_ACC_FS_16G: xl_fs = LSM6DSV320X_16g; break;
        case IMU_ACC_FS_2G:  /* fall-through */
        default:             xl_fs = LSM6DSV320X_2g;  break;
    }
    pid_status |= lsm6dsv320x_xl_full_scale_set( &imu_ctx, xl_fs );

    /* Boundary crossing: high-G -> low-G */
    if ( current_is_hg ) {
        uint8_t ctrl1_xl_hg = 0x00U; /* Clear XL_HG_REGOUT_EN and ODR */
        pid_status |= lsm6dsv320x_write_reg( &imu_ctx,
                                             LSM6DSV320X_CTRL1_XL_HG,
                                             &ctrl1_xl_hg, 1 );

        if ( imu_cached_fifo_en ) {
            lsm6dsv320x_fifo_xl_batch_t xl_bdr;
            uint8_t bdr_reg = 0U;

            pid_status |= lsm6dsv320x_read_reg( &imu_ctx,
                                                LSM6DSV320X_COUNTER_BDR_REG1,
                                                &bdr_reg, 1 );
            bdr_reg &= (uint8_t)( ~( 1U << 5U ) ); /* XL_HG_BATCH_EN = 0 */
            pid_status |= lsm6dsv320x_write_reg( &imu_ctx,
                                                 LSM6DSV320X_COUNTER_BDR_REG1,
                                                 &bdr_reg, 1 );

            /* Restore low-G batch rate to match original ODR */
            switch ( imu_cached_odr ) {
                case IMU_ODR_480HZ:  xl_bdr = LSM6DSV320X_XL_BATCHED_AT_480Hz;  break;
                case IMU_ODR_960HZ:  xl_bdr = LSM6DSV320X_XL_BATCHED_AT_960Hz;  break;
                case IMU_ODR_3840HZ: xl_bdr = LSM6DSV320X_XL_BATCHED_AT_3840Hz; break;
                case IMU_ODR_7680HZ: xl_bdr = LSM6DSV320X_XL_BATCHED_AT_7680Hz; break;
                case IMU_ODR_1920HZ: /* fall-through */
                default:             xl_bdr = LSM6DSV320X_XL_BATCHED_AT_1920Hz; break;
            }
            pid_status |= lsm6dsv320x_fifo_xl_batch_set( &imu_ctx, xl_bdr );
        }

        /*
         * §6.1.2 only applies while the high-G chain is active. Restore
         * the user-requested mode via lsm6dsv320x_xl_setup() with the
         * cached ODR. ODR_UNCHANGED is intentionally not used here since we
         * always want to reconfirm the cached ODR on the boundary crossing.
         */
        {
            lsm6dsv320x_xl_mode_t restore_mode;
            switch ( imu_cached_acc_mode ) {
                case IMU_ACC_MODE_NORMAL: restore_mode = LSM6DSV320X_XL_NORMAL_MD;          break;
                case IMU_ACC_MODE_LP1:    restore_mode = LSM6DSV320X_XL_LOW_POWER_2_AVG_MD; break;
                case IMU_ACC_MODE_LP2:    restore_mode = LSM6DSV320X_XL_LOW_POWER_4_AVG_MD; break;
                case IMU_ACC_MODE_LP3:    restore_mode = LSM6DSV320X_XL_LOW_POWER_8_AVG_MD; break;
                case IMU_ACC_MODE_HP:     /* fall-through */
                default:                  restore_mode = LSM6DSV320X_XL_HIGH_PERFORMANCE_MD; break;
            }
            pid_status |= lsm6dsv320x_xl_setup( &imu_ctx,
                              map_odr( imu_cached_odr ), restore_mode );
        }
    }
}

if ( pid_status != 0 ) { return IMU_CONFIG_FAIL; }

imu_acc_fs = new_fs;
return IMU_OK;
} /* imu_set_accel_fs */


/**
  * @brief  Mid-flight Gyroscope Full-Scale transition (Runtime).
  * @note   DS14623 Rev3 CTRL6 (15h) Table 65.
  */
IMU_STATUS imu_set_gyro_fs
    (
    IMU_GYRO_FS new_fs
    )
{
int32_t pid_status;

pid_status = lsm6dsv320x_gy_full_scale_set( &imu_ctx,
                 (lsm6dsv320x_gy_full_scale_t)new_fs );
if ( pid_status != 0 ) { return IMU_CONFIG_FAIL; }

imu_gyro_fs = new_fs;
return IMU_OK;
} /* imu_set_gyro_fs */


/**
  * @brief  Changes accel and gyro operating mode at runtime (no reset).
  * @note   DS14623 Rev3 §6.1.2: when the high-G chain is active the low-G
  *         chain must remain in high-performance mode.
  *         Uses lsm6dsv320x_xl_setup() / lsm6dsv320x_gy_setup() with the cached ODR.
  */
IMU_STATUS imu_set_power_mode
    (
    IMU_ACC_MODE  acc_mode,
    IMU_GYRO_MODE gyro_mode
    )
{
int32_t               pid_status = 0;
lsm6dsv320x_xl_mode_t xl_mode;
IMU_ACC_MODE          effective_acc_mode;

/* Cache caller's request; used by imu_set_accel_fs() on HG->LG transition */
imu_cached_acc_mode = acc_mode;

/* Enforce §6.1.2: high-G chain requires low-G in high-performance mode */
effective_acc_mode = ( imu_acc_fs >= IMU_ACC_FS_32G ) ? IMU_ACC_MODE_HP : acc_mode;

switch ( effective_acc_mode ) {
    case IMU_ACC_MODE_NORMAL: xl_mode = LSM6DSV320X_XL_NORMAL_MD;           break;
    case IMU_ACC_MODE_LP1:    xl_mode = LSM6DSV320X_XL_LOW_POWER_2_AVG_MD;  break;
    case IMU_ACC_MODE_LP2:    xl_mode = LSM6DSV320X_XL_LOW_POWER_4_AVG_MD;  break;
    case IMU_ACC_MODE_LP3:    xl_mode = LSM6DSV320X_XL_LOW_POWER_8_AVG_MD;  break;
    case IMU_ACC_MODE_HP:     /* fall-through */
    default:                  xl_mode = LSM6DSV320X_XL_HIGH_PERFORMANCE_MD; break;
}

pid_status |= lsm6dsv320x_xl_setup( &imu_ctx,
                  map_odr( imu_cached_odr ), xl_mode );
pid_status |= lsm6dsv320x_gy_setup( &imu_ctx,
                  map_odr( imu_cached_odr ),
                  (lsm6dsv320x_gy_mode_t)gyro_mode );

return ( pid_status == 0 ) ? IMU_OK : IMU_CONFIG_FAIL;
} /* imu_set_power_mode */


/**
  * @brief  Polled read of raw Accel + Gyro (Blocking).
  * @note   Reads STATUS_REG (1Eh) once per loop iteration w/ a single SPI
  *         transaction. DS14623 Rev3 Table 84/85:
  *           Bit 1 (GDA):    Gyro new data ready.
  *           Bit 0 (XLDA):   Low-G accel new data ready.
  *           Bit 3 (XLHGDA): High-G accel new data ready.
  *         Timeout = IMU_DRDY_TIMEOUT_MS (5 ms), giving ≥2.4x headroom at
  *         the slowest ODR (480 Hz, period = 2.083 ms).
  *         High-G output data at UI_OUTX_L_A_OIS_HG (34h–39h). DS14623 §9.39.
  *         Intended for pre-flight BIT and terminal sensor dump.
  */
IMU_STATUS imu_read_sync
    (
    IMU_RAW* raw
    )
{
int16_t  data_raw[3] = { 0 };
int32_t  pid_status;
uint32_t tick_start;
bool     accel_ready = false;
bool     gyro_ready  = false;

/* Poll STATUS_REG (1Eh) : one SPI transaction per loop iteration */
tick_start = HAL_GetTick();
do {
    uint8_t status_reg = 0U;
    pid_status = lsm6dsv320x_read_reg( &imu_ctx, LSM6DSV320X_STATUS_REG,
                                       &status_reg, 1 );
    if ( pid_status != 0 ) { return IMU_ERROR; }

    gyro_ready = ( ( status_reg & 0x02U ) != 0U );      /* Bit 1: GDA     */

    if ( imu_acc_fs >= IMU_ACC_FS_32G ) {
        accel_ready = ( ( status_reg & 0x08U ) != 0U ); /* Bit 3: XLHGDA */
    } else {
        accel_ready = ( ( status_reg & 0x01U ) != 0U ); /* Bit 0: XLDA   */
    }

    if ( accel_ready && gyro_ready ) { break; }
} while ( ( HAL_GetTick() - tick_start ) < IMU_DRDY_TIMEOUT_MS );

if ( !accel_ready || !gyro_ready ) {
    return IMU_TIMEOUT;
}

/* Gyro: lsm6dsv320x_angular_rate_raw_get() takes int16_t 
 * OUTX_L_G (22h) -> OUTZ_H_G (27h)
 */
pid_status = lsm6dsv320x_angular_rate_raw_get( &imu_ctx, data_raw );
if ( pid_status != 0 ) { return IMU_ERROR; }
raw->gyro_x = data_raw[0];
raw->gyro_y = data_raw[1];
raw->gyro_z = data_raw[2];

/*
 *   Accelerometer read: branch on active sensing chain.
 *   Low-G : OUTX_L_A (28h) -> OUTZ_H_A (2Dh) via ST PID wrapper.
 *   High-G: UI_OUTX_L_A_OIS_HG (34h) -> UI_OUTZ_H_A_OIS_HG (39h).
 *           6 bytes, LSB-first int16 pairs. DS14623 Rev3 §9.39-9.41.
 */
if ( imu_acc_fs >= IMU_ACC_FS_32G ) {
    uint8_t hg_bytes[6] = { 0U };
    pid_status = lsm6dsv320x_read_reg( &imu_ctx,
                                       LSM6DSV320X_UI_OUTX_L_A_OIS_HG,
                                       hg_bytes, 6 );
    if ( pid_status != 0 ) { return IMU_ERROR; }
    raw->accel_x = (int16_t)( ( (uint16_t)hg_bytes[1] << 8U ) | hg_bytes[0] );
    raw->accel_y = (int16_t)( ( (uint16_t)hg_bytes[3] << 8U ) | hg_bytes[2] );
    raw->accel_z = (int16_t)( ( (uint16_t)hg_bytes[5] << 8U ) | hg_bytes[4] );
} else {
    /* lsm6dsv320x_acceleration_raw_get() takes int16_t* */
    pid_status = lsm6dsv320x_acceleration_raw_get( &imu_ctx, data_raw );
    if ( pid_status != 0 ) { return IMU_ERROR; }
    raw->accel_x = data_raw[0];
    raw->accel_y = data_raw[1];
    raw->accel_z = data_raw[2];
}

return IMU_OK;
} /* imu_read_sync */


/**
  * @brief  Blocking SFLP/Quaternion retrieval.
  * @note   Convert with lsm6dsv320x_from_gravity_lsb_to_mg() and
  *         lsm6dsv320x_from_gbias_lsb_to_mdps() respectively.
  *           lsm6dsv320x_sflp_gravity_raw_get() -> int16_t[3] in mg LSBs
  *           lsm6dsv320x_sflp_gbias_raw_get()   -> int16_t[3] in mdps LSBs
  */
IMU_STATUS imu_read_sflp_sync
    (
    IMU_SFLP_DATA* sflp
    )
{
int32_t                  pid_status;
lsm6dsv320x_quaternion_t quat        = { 0 };
int16_t                  grav_raw[3] = { 0 };
int16_t                  gbias_raw[3]= { 0 };

if ( !imu_sflp_enabled ) { return IMU_CONFIG_FAIL; }

/* DS14623 Rev3 §13.23-13.27. (2Ah-31h)*/
/* Quaternion */
pid_status = lsm6dsv320x_sflp_quaternion_get( &imu_ctx, &quat );
if ( pid_status != 0 ) { return IMU_ERROR; }
sflp->quat_w = quat.quat_w;
sflp->quat_x = quat.quat_x;
sflp->quat_y = quat.quat_y;
sflp->quat_z = quat.quat_z;

/* Gravity vector, raw_get() -> float */
pid_status = lsm6dsv320x_sflp_gravity_raw_get( &imu_ctx, grav_raw );
if ( pid_status != 0 ) { return IMU_ERROR; }
sflp->grav_x = lsm6dsv320x_from_gravity_lsb_to_mg( grav_raw[0] );
sflp->grav_y = lsm6dsv320x_from_gravity_lsb_to_mg( grav_raw[1] );
sflp->grav_z = lsm6dsv320x_from_gravity_lsb_to_mg( grav_raw[2] );

/* Gyroscope bias, raw_get() -> float */
pid_status = lsm6dsv320x_sflp_gbias_raw_get( &imu_ctx, gbias_raw );
if ( pid_status != 0 ) { return IMU_ERROR; }
sflp->gbias_x = lsm6dsv320x_from_gbias_lsb_to_mdps( gbias_raw[0] );
sflp->gbias_y = lsm6dsv320x_from_gbias_lsb_to_mdps( gbias_raw[1] );
sflp->gbias_z = lsm6dsv320x_from_gbias_lsb_to_mdps( gbias_raw[2] );

return IMU_OK;
} /* imu_read_sflp_sync */


/**
  * @brief  Manually triggers an SPI DMA burst from the IMU FIFO.
  * @note   Must NOT be called from an ISR. Returns IMU_BUSY when a DMA
  *         transfer is already in flight - this is a valid non-error
  *         condition (flight loop called faster than the watermark rate).
  */
IMU_STATUS imu_request_async
    (
    void
    )
{
HAL_StatusTypeDef         hal_status;
lsm6dsv320x_fifo_status_t fifo_status = { 0 };
int32_t                   pid_status;
uint8_t                   slots_to_read;
uint16_t                  tx_len;

/* IMU_BUSY is a valid runtime return - do NOT assert here */
if ( imu_dma_busy ) { return IMU_BUSY; }

/* Read FIFO_STATUS1/2: DIFF_FIFO[8:0] gives unread word count.
   DS14623 Rev3 §9.25-9.26. */
pid_status = lsm6dsv320x_fifo_status_get( &imu_ctx, &fifo_status );
if ( pid_status != 0 )              { return IMU_ERROR;   }
if ( fifo_status.fifo_level == 0U ) { return IMU_NO_DATA; }

/* Clamp to the fixed DMA buffer capacity */
slots_to_read = ( (uint16_t)fifo_status.fifo_level < (uint16_t)IMU_DMA_BUF_SLOTS )
              ? (uint8_t)fifo_status.fifo_level
              : (uint8_t)IMU_DMA_BUF_SLOTS;

/*
 * Invariant: clamped slot count must fit within the fixed DMA buffer.
 * Fires if IMU_DMA_BUF_SLOTS was reduced below fifo_status.fifo_level
 * without a corresponding buffer resize - a build-time mistake.
 */
IMU_ASSERT( slots_to_read <= IMU_DMA_BUF_SLOTS );

imu_dma_slots_requested = slots_to_read;

/* Build TX buffer: address byte (RW=1, FIFO_DATA_OUT_TAG) + dummy bytes
   for the data clock-in cycles. FIFO_DATA_OUT_TAG = 0x78 -> read = 0xF8. */
tx_len = (uint16_t)( 1U + ( (uint16_t)slots_to_read * IMU_FIFO_SLOT_BYTES ) );
memset( imu_dma_tx_buf, 0x00U, tx_len );
imu_dma_tx_buf[0] = (uint8_t)( LSM6DSV320X_FIFO_DATA_OUT_TAG | IMU_SPI_READ_BIT );

imu_dma_ready = false;
imu_dma_busy  = true;

/*
 * D-Cache coherency for DMA buffers (Cortex-M7 write-back cache):
 *
 * TX: The CPU wrote the command byte to imu_dma_tx_buf. In write-back mode,
 *     those writes sit in cache; the DMA peripheral fetches from SRAM directly
 *     and would see stale data. SCB_CleanDCache_by_Addr() flushes the cache
 *     lines to SRAM so the DMA reads the correct command.
 *
 * RX: Invalidate imu_dma_rx_buf so the CPU cannot read speculative prefetch
 *     data captured before the DMA writes arrive. The post-transfer
 *     invalidation in imu_process_async_cb() handles any lines the CPU
 *     fetches speculatively during the transfer.
 */
SCB_CleanDCache_by_Addr( (uint32_t*)imu_dma_tx_buf,
                          (int32_t)IMU_DMA_BUF_BYTES_ALIGNED );
SCB_InvalidateDCache_by_Addr( (uint32_t*)imu_dma_rx_buf,
                               (int32_t)IMU_DMA_BUF_BYTES_ALIGNED );

/* Assert CS and launch DMA - CS de-asserted in imu_process_async_cb() */
HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_RESET );
__NOP(); __NOP();
hal_status = HAL_SPI_TransmitReceive_DMA( &IMU_SPI,
                                          imu_dma_tx_buf,
                                          imu_dma_rx_buf,
                                          tx_len );

if ( hal_status != HAL_OK ) {
    /* DMA failed to start - release CS and clear busy flag */
    __NOP(); __NOP();
    HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_SET );
    imu_dma_busy = false;
    return IMU_ERROR;
}

return IMU_OK;
} /* imu_request_async */


/**
  * @brief  DMA ISR Callback - call from HAL_SPI_TxRxCpltCallback().
  * @note   Execution context: DMA interrupt (ISR). Kept strictly to hardware
  *         management only - no FPU math, no parsing.
  *
  *         Steps performed here:
  *           1. De-assert CS (held since imu_request_async()).
  *           2. SCB_InvalidateDCache_by_Addr() - evicts stale H7 D-Cache lines
  *              covering imu_dma_rx_buf so the CPU reads DMA-written data.
  *           3. Snapshot imu_dma_slots_requested into imu_dma_slots_captured.
  *           4. Set imu_dma_ready = true for the flight loop to consume.
  *
  *         Parsing (lsm6dsv320x_from_quaternion_lsb_to_float / sflp2q / sqrtf) is intentionally deferred
  *         to imu_get_latest() which runs in thread context. This keeps ISR
  *         latency minimal and avoids FPU register pressure in interrupt context.
  */
void imu_process_async_cb
    (
    void
    )
{
/* De-assert CS - must come before any data access */
__NOP(); __NOP();
HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_SET );

if ( !imu_dma_busy ) { return; }

/* Invalidate D-Cache lines covering the DMA receive buffer */
SCB_InvalidateDCache_by_Addr( (uint32_t*)imu_dma_rx_buf,
                               (int32_t)IMU_DMA_BUF_BYTES_ALIGNED );

imu_dma_busy  = false;
imu_dma_ready = true;
} /* imu_process_async_cb */


/**
  * @brief  DMA Error ISR Callback - call from HAL_SPI_ErrorCallback().
  * @note   Execution context: DMA/SPI interrupt (ISR). Hardware-only - no
  *         parsing, mirrors imu_process_async_cb()'s structure.
  *
  *         Without this hook, an SPI/DMA error during the burst started by
  *         imu_request_async() leaves CS asserted low forever (locking out
  *         every other device sharing this SPI bus) and leaves imu_dma_busy
  *         permanently true (deadlocking the async data path - neither
  *         imu_has_new_data() nor a fresh imu_request_async() can ever
  *         succeed again).
  *
  *         No valid data is assumed to be present after an error, so
  *         imu_dma_ready is left/cleared false rather than set true.
  */
void imu_process_async_error_cb
    (
    void
    )
{
/* De-assert CS immediately to free the shared SPI bus */
__NOP(); __NOP();
HAL_GPIO_WritePin( IMU_NSS_GPIO_PORT, IMU_NSS_PIN, GPIO_PIN_SET );

imu_dma_busy  = false;
imu_dma_ready = false;
} /* imu_process_async_error_cb */


/**
  * @brief  Returns true if a completed DMA burst is waiting to be consumed.
  * @retval true  - new data available (call imu_get_latest()).
  * @retval false - DMA in flight or no burst has completed yet.
  */
bool imu_has_new_data
    (
    void
    )
{
return imu_dma_ready;
} /* imu_has_new_data */


/**
  * @brief  Thread-safe consumer - retrieves the latest DMA-parsed samples.
  * @note   Parsing (lsm6dsv320x_from_quaternion_lsb_to_float / sflp2q / sqrtf) is performed here
  *         in thread context rather than in the DMA ISR.
  *         This section covers only the memcpy of the raw DMA
  *         buffer and the slot count - the FPU-heavy parse runs outside it.
  *         Either pointer may be NULL if the caller only needs one dataset.
  * @param  raw:  Pointer to destination raw counts buffer  (may be NULL).
  * @param  sflp: Pointer to destination SFLP fusion buffer (may be NULL).
  * @retval IMU_OK on success, IMU_BUSY if no completed burst is available.
  */
IMU_STATUS imu_get_latest
    (
    IMU_RAW*       raw,
    IMU_SFLP_DATA* sflp
    )
{
uint32_t primask;
uint8_t  dma_snapshot[ IMU_DMA_BUF_BYTES_ALIGNED ];
uint8_t  slots;
uint8_t  i;
IMU_RAW       raw_out;
IMU_SFLP_DATA sflp_out;

/* At least one output must be non-NULL, otherwise the call is a no-op */
IMU_ASSERT( raw != NULL || sflp != NULL );

if ( !imu_dma_ready ) { return IMU_BUSY; }

/*
 * Critical section: snapshot the slot count, derive exactly how many bytes
 * this DMA transaction actually populated (1 cmd byte + slots*7), copy only
 * that range, then clear the ready flag. The slot count MUST be captured
 * before it's used to size the copy (to avoid garbage/wrapping)
 * memcpy protection req. + the parse runs outside the critical
 * section so FPU operations do not block other interrupts.
 */
primask = __get_PRIMASK();
__disable_irq();

slots = imu_dma_slots_requested;

/* Invariant: captured slot count must be within the allocated buffer */
IMU_ASSERT( slots <= IMU_DMA_BUF_SLOTS );

{
    uint16_t valid_len = (uint16_t)( 1U + ( (uint16_t)slots * IMU_FIFO_SLOT_BYTES ) );
    memcpy( dma_snapshot, imu_dma_rx_buf, valid_len );
}
imu_dma_ready = false;

__set_PRIMASK( primask );

/* Parse in thread context - safe for FPU, sqrtf, and variable latency */
memset( &raw_out,  0, sizeof( raw_out  ) );
memset( &sflp_out, 0, sizeof( sflp_out ) );

/* dma_snapshot[0] is the dummy address-clock byte; slots start at index 1 */
for ( i = 0U; i < slots; i++ ) {
    const uint8_t* slot = &dma_snapshot[ 1U + ( (uint16_t)i * IMU_FIFO_SLOT_BYTES ) ];
    parse_fifo_slot( slot, &raw_out, &sflp_out );
}

if ( raw  != NULL ) { memcpy( raw,  &raw_out,  sizeof( IMU_RAW       ) ); }
if ( sflp != NULL ) { memcpy( sflp, &sflp_out, sizeof( IMU_SFLP_DATA ) ); }

return IMU_OK;
} /* imu_get_latest */


/**
  * @brief  Scales raw INT16 counts to physical floating-point data.
  * @note   Sensitivity values from DS14623 Rev3 Table 3.
  *         Low-G chain  (±2..16 g):  LA_So in mg/LSB.
  *         High-G chain (±32..320 g): LA_So in mg/LSB (separate sensing chain).
  *         A switch-based index lookup is used because the high-G FS enum
  *         values are not contiguous with the low-G ones in the PID enum space.
  *         Output: accel in [g], gyro in [dps].
  * @param  raw:  Source raw counts (non-NULL).
  * @param  data: Destination scaled physical data (non-NULL).
  */
void imu_scale_raw
    (
    const IMU_RAW* raw,
    IMU_DATA*      data
    )
{
float   acc_sens;
float   gyro_sens;
uint8_t acc_idx;
uint8_t gy_idx;

IMU_ASSERT( raw  != NULL );
IMU_ASSERT( data != NULL );

/*
 * Accelerometer sensitivity look-up table [mg/LSB].
 * All 9 ranges from DS14623 Rev3 Table 3, in ascending FS order.
 * Index 0-3: low-G chain  (±2/4/8/16 g).
 * Index 4-8: high-G chain (±32/64/128/256/320 g).
 */
static const float acc_sens_mg_lsb[9] = {
    0.061f,    /* ±2g    [low-G]  */
    0.122f,    /* ±4g    [low-G]  */
    0.244f,    /* ±8g    [low-G]  */
    0.488f,    /* ±16g   [low-G]  */
    0.976f,    /* ±32g   [high-G] */
    1.952f,    /* ±64g   [high-G] */
    3.904f,    /* ±128g  [high-G] */
    7.808f,    /* ±256g  [high-G] */
    10.417f,   /* ±320g  [high-G] */
};

static const float gyro_sens_mdps_lsb[5] = {
    8.75f,    /* ±250  dps */
    17.50f,   /* ±500  dps */
    35.0f,    /* ±1000 dps */
    70.0f,    /* ±2000 dps */
    140.0f,   /* ±4000 dps */
};

/* Map acc FS enum to table index */
switch ( imu_acc_fs ) {
    case IMU_ACC_FS_4G:   acc_idx = 1U; break;
    case IMU_ACC_FS_8G:   acc_idx = 2U; break;
    case IMU_ACC_FS_16G:  acc_idx = 3U; break;
    case IMU_ACC_FS_32G:  acc_idx = 4U; break;
    case IMU_ACC_FS_64G:  acc_idx = 5U; break;
    case IMU_ACC_FS_128G: acc_idx = 6U; break;
    case IMU_ACC_FS_256G: acc_idx = 7U; break;
    case IMU_ACC_FS_320G: acc_idx = 8U; break;
    case IMU_ACC_FS_2G:   /* fall-through */
    default:              acc_idx = 0U; break;
}

/* Map gyro FS enum to table index */
switch ( imu_gyro_fs ) {
    case IMU_GYRO_FS_500DPS:  gy_idx = 1U; break;
    case IMU_GYRO_FS_1000DPS: gy_idx = 2U; break;
    case IMU_GYRO_FS_2000DPS: gy_idx = 3U; break;
    case IMU_GYRO_FS_4000DPS: gy_idx = 4U; break;
    case IMU_GYRO_FS_250DPS:  /* fall-through */
    default:                  gy_idx = 0U; break;
}

/* Invariant: indices must be within table bounds */
IMU_ASSERT( acc_idx < 9U );
IMU_ASSERT( gy_idx  < 5U );

acc_sens  = acc_sens_mg_lsb[acc_idx]   / 1000.0f; /* mg/LSB  -> g/LSB   */
gyro_sens = gyro_sens_mdps_lsb[gy_idx] / 1000.0f; /* mdps/LSB -> dps/LSB */

data->accel_x = (float)raw->accel_x * acc_sens;
data->accel_y = (float)raw->accel_y * acc_sens;
data->accel_z = (float)raw->accel_z * acc_sens;
data->gyro_x  = (float)raw->gyro_x  * gyro_sens;
data->gyro_y  = (float)raw->gyro_y  * gyro_sens;
data->gyro_z  = (float)raw->gyro_z  * gyro_sens;
} /* imu_scale_raw */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
