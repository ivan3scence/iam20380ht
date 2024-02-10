/**
 * @file iam20380ht_defs.h
 * @author ivan.gorich@gmail.com
 * @brief Header with sensor registers and structers.
 * @version 0.1
 * @date 2024-01-27
 */


#ifndef IAM20380HT_DEFS_H_
#define IAM20380HT_DEFS_H_

/********************************************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stdio.h>
#include <linux/types.h>
#endif
#include <linux/i2c.h>

/*!
 * @defgroup group1 Sensor macro API.
 * @brief Some useful macros for whole project.
 * @{
 */
#define UINT8_C(x)    ((uint8_t)x)
#define INT8_C(x)     ((int8_t)x)

#define UINT16_C(x)   ((uint16_t)x)
#define INT16_C(x)    ((int16_t)x)

#define UINT32_C(x)   ((uint32_t)x)
#define INT32_C(x)    ((int32_t)x)

#ifdef IAM_DEBUG
#define IAM_INFO(fmt, ...)  \
    pr_info(IAM20380HT_DEV_NAME ":%s:%d:\n----> " fmt, \
            __func__, __LINE__, ##__VA_ARGS__)
#else
  #define IAM_INFO(fmt, ...)
#endif
#define IAM_ERR(fmt, ...)  \
    pr_err(IAM20380HT_DEV_NAME "[ERROR]:%s:%d:\n----> " fmt, \
            __func__, __LINE__, ##__VA_ARGS__)

#ifdef NEW_KERNEL
  typedef struct 
  {
    long  tv_sec;
    long  tv_usec;
  } timeval;
#else
  typedef struct timeval timeval;
#endif

/*!
 * @brief C standard macros
 * @{
 */
#ifndef NULL
#ifdef __cplusplus
#define NULL                                0
#else
#define NULL                                ((void *) 0)
#endif
#endif
#define TRUE                                UINT8_C(1)
#define FALSE                               UINT8_C(0)
/**@}*/

/*!
 * @brief API return codes
 * @{
 */
#define IAM20380HT_OK                       INT8_C(0)
#define IAM20380HT_FAIL                     INT8_C(-8)
#define IAM20380HT_E_NULL_PTR               INT8_C(-1)
#define IAM20380HT_E_COM_FAIL               INT8_C(-2)
#define IAM20380HT_E_DEV_NOT_FOUND          INT8_C(-3)
#define IAM20380HT_E_OUT_OF_RANGE           INT8_C(-4)
#define IAM20380HT_E_INVALID_INPUT          INT8_C(-5)
#define IAM20380HT_E_ODR_BW_INVALID         INT8_C(-6)
#define IAM20380HT_E_INVALID_TEMPERATURE    INT8_C(-7)
#define IAM20380HT_SELF_TEST_FAIL           INT8_C(-228)
/**@}*/

#define IAM20380HT_kHz(n)                   (1000 * n)
#define SENSOR_NAME                         "iam20380ht"
#define IAM20380HT_DEV_NAME                 "iam20380htdrv"
#define IAM_DELAY_MIN                       (1)
#define IAM_DELAY_DEFAULT                   (200)
#define IAM_VALUE_MAX                       (32767)
#define IAM_VALUE_MIN                       (-32768)
#define IAM20380HT_DUMP_LEN                 0x80
#define MILLION_I                           1000000
#define DEV_FROM_CLIENT_DATA(client_data)   (&((client_data)->device.i2c_client->dev))
#define IAM20380HT_CHIP_ID                  UINT8_C(0xFD) ///< Chip ID
#define IAM20380HT_US_PER_FRAME(smplrt)     (1000 * (1 + smplrt))
/**
 * @brief Start up time after exiting sleep mode. */
#define IAM20380HT_AFTER_SLEEP_DELAY        UINT8_C(35)
/**
 * @brief Checks register value if the sensor is in sleep mode. */
#define SLEEPING(regvar)                    (IAM20380HT_GET_BITS(regvar, SLEEP) == 1)
/**
 * @{ \name I2C addresses.
 */
#define IAM20380HT_I2C_ADDR                 UINT8_C(0x68) ///< I2C address when SDO pulled to GND 
#define IAM20380HT_I2C_ADDR2                UINT8_C(0x69) ///< I2C address when SDO pulled to VDDIO
/**@}*/
/**@}*/

/*!
 * @defgroup group2 Register map.
 * @brief All iam20380ht registers as macros.
 * @{
 */
#define IAM20380HT_ADDR_SELF_TEST_X         UINT8_C(0x00) ///< Self-test reg.
#define IAM20380HT_ADDR_SELF_TEST_Y         UINT8_C(0x01) ///< Self-test reg.
#define IAM20380HT_ADDR_SELF_TEST_Z         UINT8_C(0x02) ///< Self-test reg.

#define IAM20380HT_ADDR_OFFSET_X_H          UINT8_C(0x13) ///< Gyro offsets.
#define IAM20380HT_ADDR_OFFSET_X_L          UINT8_C(0x14) ///< Gyro offsets.
#define IAM20380HT_ADDR_OFFSET_Y_H          UINT8_C(0x15) ///< Gyro offsets.
#define IAM20380HT_ADDR_OFFSET_Y_L          UINT8_C(0x16) ///< Gyro offsets.
#define IAM20380HT_ADDR_OFFSET_Z_H          UINT8_C(0x17) ///< Gyro offsets.
#define IAM20380HT_ADDR_OFFSET_Z_L          UINT8_C(0x18) ///< Gyro offsets.

#define IAM20380HT_ADDR_SMPLRT_DIV          UINT8_C(0x19) ///< This register is only effective when FCHOICE_B register bits are 2’b00, and (0 < DLPF_CFG < 7)

#define IAM20380HT_ADDR_CONFIG              UINT8_C(0x1A)
#define IAM20380HT_ADDR_GYRO_CONFIG         UINT8_C(0x1B)
#define IAM20380HT_ADDR_LP_MODE_CFG         UINT8_C(0x1E) ///< Low-power mode configuration.
#define IAM20380HT_ADDR_FIFO_EN             UINT8_C(0x23)
#define IAM20380HT_ADDR_FSYNC_INT           UINT8_C(0x36) ///< Fsync interrupt status.
#define IAM20380HT_ADDR_INT_PIN_CFG         UINT8_C(0x37)
#define IAM20380HT_ADDR_INT_ENABLE          UINT8_C(0x38)
#define IAM20380HT_ADDR_INT_STATUS          UINT8_C(0x3A) ///< Interrupt status.

#define IAM20380HT_ADDR_TEMP_OUT_H          UINT8_C(0x41) ///< Sensor temperature regs.
#define IAM20380HT_ADDR_TEMP_OUT_L          UINT8_C(0x42) ///< Sensor temperature regs.

#define IAM20380HT_ADDR_GYRO_XOUT_H         UINT8_C(0x43) ///< Gyro values.
#define IAM20380HT_ADDR_GYRO_XOUT_L         UINT8_C(0x44) ///< Gyro values.
#define IAM20380HT_ADDR_GYRO_YOUT_H         UINT8_C(0x45) ///< Gyro values.
#define IAM20380HT_ADDR_GYRO_YOUT_L         UINT8_C(0x46) ///< Gyro values.
#define IAM20380HT_ADDR_GYRO_ZOUT_H         UINT8_C(0x47) ///< Gyro values.
#define IAM20380HT_ADDR_GYRO_ZOUT_L         UINT8_C(0x48) ///< Gyro values.

#define IAM20380HT_ADDR_SIGNAL_PATH_RESET   UINT8_C(0x68)
#define IAM20380HT_ADDR_USER_CTRL           UINT8_C(0x6A)
#define IAM20380HT_ADDR_PWR_MGMT_1          UINT8_C(0x6B)
#define IAM20380HT_ADDR_PWR_MGMT_2          UINT8_C(0x6C)
#define IAM20380HT_ADDR_FIFO_COUNTH         UINT8_C(0x72) ///< Number of bytes in FIFO.
#define IAM20380HT_ADDR_FIFO_COUNTL         UINT8_C(0x73) ///< Number of bytes in FIFO.
#define IAM20380HT_ADDR_FIFO_R_W            UINT8_C(0x74) ///< Register to read sensor FIFO.

#define IAM20380HT_ADDR_WHO_AM_I            UINT8_C(0x75) ///< Chip ID. Always 0xFD.
/**@}*/

/*!
 * @defgroup group3 Register fields.
 * @brief These macros define register fields and mask for them.
 * @{
 */
/**
 * \brief Self test trigger.
 *
 * Self-test allows for the testing of the mechanical and electrical portions of the sensors.
 * The self-test for each measurement axis can
 * be activated by means of the gyroscope self-test register (register 27).
 * When the self-test is activated, the electronics cause the sensors to be actuated
 * and produce an output signal. The output signal is
 * used to observe the self-test response.
 */
/**
 * @brief X Gyro self-test
 * TODO
 */
#define IAM20380HT_XG_ST_MSK                UINT8_C(0x80)
#define IAM20380HT_XG_ST_POS                UINT8_C(0x07)
/**
 * @brief Y Gyro self-test
 * TODO
 */
#define IAM20380HT_YG_ST_MSK                UINT8_C(0x40)
#define IAM20380HT_YG_ST_POS                UINT8_C(0x06)
/**
 * @brief Z Gyro self-test
 * TODO
 */
#define IAM20380HT_ZG_ST_MSK                UINT8_C(0x20)
#define IAM20380HT_ZG_ST_POS                UINT8_C(0x05)

#define IAM20380HT_SELF_TEST_TRIGGER      ( IAM20380HT_XG_ST_MSK \
                                            | IAM20380HT_YG_ST_MSK \
                                            | IAM20380HT_ZG_ST_MSK )

/**
 * @brief Gyro Full Scale Select.
 */
#define IAM20380HT_FS_SEL_MSK               UINT8_C(0x18)
#define IAM20380HT_FS_SEL_POS               UINT8_C(0x03)
/**
 * @brief Used to bypass DLPF.
 */
#define IAM20380HT_FCHOICE_B_MSK            UINT8_C(0x03)
#define IAM20380HT_FCHOICE_B_POS            UINT8_C(0x00)
/**@}*/

#define IAM20380HT_ADDR_USER_CTRL           UINT8_C(0x6A)
/**
 * @{ \name USER_CTRL.
 * @brief Register - 0x6A.
 */
/**
 * @brief 1 – Enable FIFO operation mode.
 * 0 – Disable FIFO access from serial interface.
 * To disable FIFO writes by DMA, use FIFO_EN register.
 */
#define IAM20380HT_FIFO_EN_MSK              UINT8_C(0x40)
#define IAM20380HT_FIFO_EN_POS              UINT8_C(0x06)
/**
 * @brief 1 – Disable I2C Slave module and put the serial interface in SPI mode only.
 */
#define IAM20380HT_I2C_IF_DIS_MSK           UINT8_C(0x10)
#define IAM20380HT_I2C_IF_DIS_POS           UINT8_C(0x04)
/**
 * @brief 1 – Reset FIFO module. Reset is asynchronous.
 * This bit auto clears after one clock cycle of the internal 20 MHz clock.
 */
#define IAM20380HT_FIFO_RST_MSK             UINT8_C(0x04)
#define IAM20380HT_FIFO_RST_POS             UINT8_C(0x02)
/**
 * @brief 1 – Reset all gyro digital signal path and temp digital signal path.
 * This bit also clears all the sensor registers.
 */
#define IAM20380HT_SIG_COND_RST_MSK         UINT8_C(0x01)
#define IAM20380HT_SIG_COND_RST_POS         UINT8_C(0x00)

/*!
 * @brief FIFO modes. Values are set according to USER_CTRL register.
 */
typedef enum {
  IAM20380HT_FIFO_DIS     = 0x00,
  IAM20380HT_SIG_COND_RST = IAM20380HT_SIG_COND_RST_MSK,
  IAM20380HT_FIFO_RST     = IAM20380HT_FIFO_RST_MSK,
  IAM20380HT_FIFO_EN      = IAM20380HT_FIFO_EN_MSK
} fifo_mode_t;

#define IAM20380HT_FIFO_MODE(mode)          (mode == IAM20380HT_FIFO_EN) ? \
                                            IAM20380HT_ENABLE : IAM20380HT_DISABLE
/**@}*/

/**
 * @{ \name CONFIG.
 * @brief Register - 0x1A.
 */
/**
 * @brief When set to ‘1’, when the FIFO is full, additional
 * writes will not be written to FIFO.
 * When set to ‘0’, when the FIFO is full, additional writes
 * will be written to the FIFO, replacing the oldest data.
 */
#define IAM20380HT_FIFO_MODE_MSK            UINT8_C(0x40)
#define IAM20380HT_FIFO_MODE_POS            UINT8_C(0x06)

/**
 * @brief FIFO modes enum.
 * IAM20380HT_FIFO_REWRITE_OLD: when the FIFO is full,
 * additional writes will be written to the FIFO,
 * replacing the oldest data.
 * IAM20380HT_FIFO_SAVE_OLD: when the FIFO is full,
 * additional writes will not be written to FIFO.
 */
typedef enum {
  IAM20380HT_FIFO_REWRITE_OLD               =  0x00,
  IAM20380HT_FIFO_SAVE_OLD                  =  IAM20380HT_FIFO_MODE_MSK,
} fifo_ovmode_t;
/**
 * @brief Enables the FSYNC pin data to be sampled.
 */
#define IAM20380HT_EXT_SYNC_SET_MSK         UINT8_C(0x38)
#define IAM20380HT_EXT_SYNC_SET_POS         UINT8_C(0x03)
/**
 * @brief Enables the FSYNC pin data to be sampled.
 * For the DLPF to be used, FCHOICE_B[1:0] is 2’b00.
 */
#define IAM20380HT_DLPF_CFG_MSK             UINT8_C(0x07)
#define IAM20380HT_DLPF_CFG_POS             UINT8_C(0x00)
/**@}*/

/**
 * @{ \name LP_MODE_CFG.
 * @brief Register - 0x1E.
 */
/**
 * @brief When set to ‘1’ low-power gyroscope mode
 * is enabled. Default setting is ‘0’.
 */
#define IAM20380HT_MODE_LP_EN_MSK           UINT8_C(0x80)
#define IAM20380HT_MODE_LP_EN_POS           UINT8_C(0x07)
/**
 * @brief Averaging filter configuration for low-power
 * gyroscope mode. Default setting is ‘000’.
 */
#define IAM20380HT_MODE_G_AVGCFG_MSK        UINT8_C(0x70)
#define IAM20380HT_MODE_G_AVGCFG_POS        UINT8_C(0x04)
/*TODO: low-noise mode*/
/**@}*/

/**
 * @{ \name PWR_MGMT_1.
 * @brief Register - 0x6B.
 */
/**
 * @brief 1 – Reset the internal registers
 * and restores the default settings. The bit automatically
 * clears to 0 once the reset is done.
 */
#define IAM20380HT_DEVICE_RESET_MSK         UINT8_C(0x80)
#define IAM20380HT_DEVICE_RESET_POS         UINT8_C(0x07)
/**
 * @brief When set to 1, the chip is set to sleep mode.
 * Note: The default value is 1, the chip comes up in Sleep mode.
 */
#define IAM20380HT_SLEEP_MSK                UINT8_C(0x40)
#define IAM20380HT_SLEEP_POS                UINT8_C(0x06)
/**
 * @brief When set, the gyro drive and pll circuitry are enabled,
 * but the sense paths are disabled. This is a low power
 * mode that allows quick enabling of the gyros.
 */
#define IAM20380HT_GYRO_STANDBY_MSK         UINT8_C(0x10)
#define IAM20380HT_GYRO_STANDBY_POS         UINT8_C(0x04)
/**
 * @brief When set to 1, this bit disables the temperature sensor.
 */
#define IAM20380HT_TEMP_DIS_MSK             UINT8_C(0x08)
#define IAM20380HT_TEMP_DIS_POS             UINT8_C(0x03)
/**
 * @brief Clock Source.
 * The default value of CLKSEL[2:0] is 000.
 * It is required that CLKSEL[2:0] be set to 001 to achieve
 * full gyroscope performance.
 */
#define IAM20380HT_CLK_SEL_MSK              UINT8_C(0x07)
#define IAM20380HT_CLK_SEL_POS              UINT8_C(0x00)
/**
 * @brief Macro to set right register value for power mode.
 */
#define IAM20380HT_CLEAR_PM_BITS_MSK        ~(IAM20380HT_SLEEP_MSK | IAM20380HT_GYRO_STANDBY_MSK | IAM20380HT_TEMP_DIS_MSK)
#define IAM20380HT_SET_PM(regvar, pm)       ((regvar & IAM20380HT_CLEAR_PM_BITS_MSK) | pm)

typedef enum
{
  /**
   * @brief Normal operation mode. */
  IAM20380HT_PM_NORMAL_MODE   = 0,
  /**
   * @brief 1 - disable temperature sensor. */
  IAM20380HT_PM_TEMP_OFF      = IAM20380HT_TEMP_DIS_MSK,
  /**
   * @brief Drive and pll circuitry are enabled,
   * but the sense paths are disabled. This is a low
   * power mode that allows quick enabling of the gyros. */
  IAM20380HT_PM_STANDBY       = IAM20380HT_GYRO_STANDBY_MSK,
  /**
   * @brief Sleeping mode. */
  IAM20380HT_PM_SLEEP         = IAM20380HT_SLEEP_MSK,
  /**
   * @brief TODO */
  IAM20380HT_PM_LOW_NOISE     = 0xdead
} iam20380ht_pm_mode_t;
/**
 * @brief Checks if sensor is in sleep mode.
 */
#define IAM20380HT_IS_SLEEPING(regvar)      (regvar & IAM20380HT_PM_SLEEP)
/**@}*/

/**
 * @{ \name PWR_MGMT_2.
 * @brief Register - 0x6C.
 */
/**
 * @brief 1 – X gyro is disabled. 0 – X gyro is on.
 */
#define IAM20380HT_STBY_XG_MSK              UINT8_C(0x04)
#define IAM20380HT_STBY_XG_POS              UINT8_C(0x02)
/**
 * @brief 1 – Y gyro is disabled. 0 – Y gyro is on.
 */
#define IAM20380HT_STBY_YG_MSK              UINT8_C(0x02)
#define IAM20380HT_STBY_YG_POS              UINT8_C(0x01)
/**
 * @brief 1 – Z gyro is disabled. 0 – Z gyro is on.
 */
#define IAM20380HT_STBY_ZG_MSK              UINT8_C(0x01)
#define IAM20380HT_STBY_ZG_POS              UINT8_C(0x00)
/**@}*/
/**
 * @{ \name FIFO_EN.
 * @brief Register - 0x23.
 */
/**
 * @brief 1 – Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at
 * the sample rate; If enabled, buffering of data occurs even
 * if data path is in standby. 0 – Function is disabled.
 */
#define IAM20380HT_TEMP_FIFO_EN_MSK         UINT8_C(0x80)
#define IAM20380HT_TEMP_FIFO_EN_POS         UINT8_C(0x07)
/**
 * @brief 1 – Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at
 * the sample rate; If enabled, buffering of data occurs even
 * if data path is in standby. 0 – Function is disabled.
 */
#define IAM20380HT_XG_FIFO_EN_MSK           UINT8_C(0x40)
#define IAM20380HT_XG_FIFO_EN_POS           UINT8_C(0x06)
/**
 * @brief 1 – Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at
 * the sample rate; If enabled, buffering of data occurs even
 * if data path is in standby. 0 – Function is disabled.
 */
#define IAM20380HT_YG_FIFO_EN_MSK           UINT8_C(0x20)
#define IAM20380HT_YG_FIFO_EN_POS           UINT8_C(0x05)
/**
 * @brief 1 – Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at
 * the sample rate; If enabled, buffering of data occurs even
 * if data path is in standby. 0 – Function is disabled.
 */
#define IAM20380HT_ZG_FIFO_EN_MSK           UINT8_C(0x10)
#define IAM20380HT_ZG_FIFO_EN_POS           UINT8_C(0x04)
/**
 * @brief Length of temperature sensor data.
 */
#define IAM20380HT_FIFO_T_LENGTH            UINT8_C(2) /**
 * @brief Length of gyro sensor data.
 */
#define IAM20380HT_FIFO_G_LENGTH            UINT8_C(6) /**
 * @brief Length of gyro sensor data.
 */
#define IAM20380HT_FIFO_TG_LENGTH           (IAM20380HT_FIFO_T_LENGTH + IAM20380HT_FIFO_G_LENGTH)
#define FIFO_EN_XYZ                         (IAM20380HT_XG_FIFO_EN_MSK | IAM20380HT_YG_FIFO_EN_MSK | IAM20380HT_ZG_FIFO_EN_MSK)
/**
 * @brief 1 – Z gyro is disabled. 0 – Z gyro is on.
 */
#define IAM20380HT_WO_TEMPERATURE           FALSE
#define IAM20380HT_WITH_TEMPERATURE         TRUE

/**@}*/

/**
 * @{ \name FIFO_COUNTH.
 * @brief Register - 0x72.
 */
/**
 * @brief High Bits; count indicates the number of written bytes
 * in the FIFO. Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUNTL.
 */
#define IAM20380HT_FIFO_COUNTH_MSK          UINT8_C(0x1F)
#define IAM20380HT_FIFO_COUNTH_POS          UINT8_C(0x00)
#define IAM20380HT_FIFO_COUNTH_SHIFT        UINT8_C(8)
/**@}*/

/**
 * @{ \name FIFO_COUNTL.
 * @brief Register - 0x73.
 */
/**
 * @brief Low Bits; count indicates the number of written bytes
 * in the FIFO. NOTE: Must read FIFO_COUNTH to latch new data for
 * both FIFO_COUNTH and FIFO_COUNTL.
 */
#define IAM20380HT_FIFO_COUNTL_MSK          UINT8_C(0xFF)
#define IAM20380HT_FIFO_COUNTL_POS          UINT8_C(0x00)
/**@}*/

/**
 * @{ \name FIFO_R_W.
 * @brief Register - 0x74.
 */
/**
 * @brief Read/Write command provides Read or Write operation for the FIFO.
 */
#define IAM20380HT_FIFO_DATA_MSK            UINT8_C(0xFF)
#define IAM20380HT_FIFO_DATA_POS            UINT8_C(0x00)
/**@}*/

#define IAM20380HT_DISABLE                  UINT8_C(0)
#define IAM20380HT_ENABLE                   UINT8_C(1)

/**
 * @{ \name FSYNC_INT.
 * @brief Register - 0x36.
 */
/**
 * @brief This bit automatically sets to 1 when a FSYNC interrupt
 * has been generated. The bit clears to 0 after the register has been read.
 */
#define IAM20380HT_FSYNC_INT_MSK            UINT8_C(0xFF)
#define IAM20380HT_FSYNC_INT_POS            UINT8_C(0x00)
/**@}*/

/**
 * @{ \name FIFO_R_W.
 * @brief Register - 0x74.
 */
/**
 * @brief Read/Write command provides Read or Write operation for the FIFO.
 */
#define IAM20380HT_FIFO_DATA_MSK            UINT8_C(0xFF)
#define IAM20380HT_FIFO_DATA_POS            UINT8_C(0x00)
/**@}*/

/**
 * @{ \name INT_PIN_CFG.
 * @brief Register - 0x37.
 * Interrupt functionality is configured via the Interrupt Configuration register. Items that are configurable include the INT pin
 * configuration, the interrupt latching and clearing method, and triggers for the interrupt. Items that can trigger an interrupt are
 * (1) Clock generator locked to new reference oscillator (used when switching clock sources);
 * (2) new data are available to be read (from the FIFO and Data registers);
 * (3) FIFO overflow. The interrupt status can be read from the Interrupt Status register.
 */
/**
 * @brief 1 – The logic level for INT/DRDY pin is active low.
 * 0 – The logic level for INT/DRDY pin is active high.
 */
#define IAM20380HT_INT_LEVEL_MSK            UINT8_C(0x80)
#define IAM20380HT_INT_LEVEL_POS            UINT8_C(0x07)
/**
 * @brief 1 – INT/DRDY pin is configured as open drain.
 * 0 – INT/DRDY pin is configured as push-pull.
 */
#define IAM20380HT_INT_OPEN_MSK             UINT8_C(0x40)
#define IAM20380HT_INT_OPEN_POS             UINT8_C(0x06)
/**
 * @brief 1 – INT/DRDY pin level held until interrupt
 * status is cleared. 0 – INT/DRDY pin indicates
 * interrupt pulse’s width is 50 µs.
 */
#define IAM20380HT_LATCH_INT_EN_MSK         UINT8_C(0x20)
#define IAM20380HT_LATCH_INT_EN_POS         UINT8_C(0x05)
/**
 * @brief 1 – Interrupt status is cleared if any read
 * operation is performed. 0 – Interrupt status is
 * cleared only by reading INT_STATUS register
 */
#define IAM20380HT_INT_RD_CLEAR_MSK         UINT8_C(0x10)
#define IAM20380HT_INT_RD_CLEAR_POS         UINT8_C(0x04)
/**
 * @brief 1 – The logic level for the FSYNC pin as an
 * interrupt is active low. 0 – The logic level for
 * the FSYNC pin as an interrupt is active high.
 */
#define IAM20380HT_FSYNC_INT_LEVEL_MSK      UINT8_C(0x08)
#define IAM20380HT_FSYNC_INT_LEVEL_POS      UINT8_C(0x03)
/**
 * @brief When this bit is equal to 1, the FSYNC pin
 * will trigger an interrupt when it transitions to
 * the level specified by FSYNC_INT_LEVEL. When this
 * bit is equal to 0, the FSYNC pin is disabled from causing an interrupt.
 */
#define IAM20380HT_FSYNC_INT_MODE_EN_MSK    UINT8_C(0x04)
#define IAM20380HT_FSYNC_INT_MODE_EN_POS    UINT8_C(0x02)
/**@}*/

/**
 * @{ \name INT_ENABLE.
 * @brief Register - 0x38.
 * Sets interrupt triggers.
 */
/**
 * @brief 1 – Enables a FIFO buffer overflow to generate
 * an interrupt. 0 – Function is disabled.
 */
#define IAM20380HT_FIFO_OFLOW_EN_MSK        UINT8_C(0x10)
#define IAM20380HT_FIFO_OFLOW_EN_POS        UINT8_C(0x04)
/**
 * @brief Motion Detection interrupt enable.
 */
#define IAM20380HT_GDRIVE_INT_EN_MSK        UINT8_C(0x04)
#define IAM20380HT_GDRIVE_INT_EN_POS        UINT8_C(0x02)
/**
 * @brief Data ready interrupt enable.
 */
#define IAM20380HT_DATA_RDY_INT_EN_MSK      UINT8_C(0x01)
#define IAM20380HT_DATA_RDY_INT_EN_POS      UINT8_C(0x00)
/**@}*/

/**
 * @{ \name INT_STATUS.
 * @brief Register - 0x3A.
 * Reads interrupt status.
 */
/**
 * @brief This bit automatically sets to 1 when a FIFO
 * buffer overflow has been generated. The bit clears
 * to 0 after the register has been read.
 */
#define IAM20380HT_FIFO_OFLOW_INT_MSK       UINT8_C(0x10)
#define IAM20380HT_FIFO_OFLOW_INT_POS       UINT8_C(0x04)
/**
 * @brief Motion Detection interrupt
 */
#define IAM20380HT_GDRIVE_INT_MSK           UINT8_C(0x04)
#define IAM20380HT_GDRIVE_INT_POS           UINT8_C(0x02)
/**
 * @brief This bit automatically sets to 1 when a Data
 * Ready interrupt is generated. The bit clears to 0
 * after the register has been read.
 */
#define IAM20380HT_DATA_RDY_INT_MSK         UINT8_C(0x01)
#define IAM20380HT_DATA_RDY_INT_POS         UINT8_C(0x00)
/**@}*/

#define IAM20380HT_ERR_REG_MASK           UINT8_C(0x0F)

/**
 * @{ \name SELF_TEST_GYRO,
 * @brief Registers: [0x00-0x02].
 * GYROSCOPE SELF-TEST REGISTERS.
 */
/**
 * @brief The value in this register indicates the
 * self-test output generated during manufacturing tests.
 * This value is to be used to check against subsequent
 * self-test outputs performed by the end user.
 */
#define IAM20380HT_SELF_TEST_GYRO_MSK       UINT8_C(0xFF)
#define IAM20380HT_SELF_TEST_GYRO_POS       UINT8_C(0x00)
/**@}*/

/**
 * @{ \name G_OFFS_USRH.
 * @brief Register - 0x13,0x15,0x17.
 * GYRO OFFSET ADJUSTMENT REGISTER
 */
/**
 * @brief Bits 15 to 8 of the 16-bit offset of X/Y/Z gyroscope
 * (2’s complement). This register is used to remove DC bias
 * from the sensor output. The value in this register is
 * added to the gyroscope sensor value before going into the sensor register.
 */
#define IAM20380HT_OFFS_USR_15_8_MSK        UINT8_C(0xFF)
#define IAM20380HT_OFFS_USR_15_8_POS        UINT8_C(0x00)
/**@}*/

/**
 * @{ \name G_OFFS_USRL.
 * @brief Register - 0x14,0x16,0x18.
 * GYRO OFFSET ADJUSTMENT REGISTER
 */
/**
 * @brief Bits 7 to 0 of the 16-bit offset of X/Y/Z
 * gyroscope (2’s complement). This register is used to
 * remove DC bias from the sensor output. The value in this
 * register is added to the gyroscope sensor value before
 * going into the sensor register.
 */
#define IAM20380HT_OFFS_USR_7_0_MSK         UINT8_C(0xFF)
#define IAM20380HT_OFFS_USR_7_0_POS         UINT8_C(0x00)
/**@}*/

/**
 * @{ \name G_OFFS_USRL.
 * @brief Register - 0x14,0x16,0x18.
 * GYRO OFFSET ADJUSTMENT REGISTER
 */
/**
 * @brief Divides the internal sample rate (see register CONFIG)
 * to generate the sample rate that controls sensor data
 * output rate, FIFO sample rate. Note: This register is
 * only effective when FCHOICE_B register bits are 2’b00,
 * and (0 < DLPF_CFG < 7).
 */
#define IAM20380HT_SMPLRT_DIV_MSK           UINT8_C(0xFF)
#define IAM20380HT_SMPLRT_DIV_POS           UINT8_C(0x00)
/**
 * @brief Formula to calculate sample rate.
 */
#define IAM20380HT_INTERNAL_SAMPLE_RATE     IAM20380HT_kHz(1)
#define IAM20380HT_ODR(SMPLRT_DIV)          (INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV))/**@}*/

/**
 * @{ \name SIGNAL_PATH_RESET.
 * @brief Register - 0x68.
 * Reset temp digital signal path.
 */
#define IAM20380HT_TEMP_RST_MSK             UINT8_C(0x01)
#define IAM20380HT_TEMP_RST_POS             UINT8_C(0x00)

#define IAM20380HT_TEMP_RST                 IAM20380HT_TEMP_RST_MSK
/**@}*/
/**@}*/

/**
 * @{ \name BIT OPERATIONS.
 * @brief Bit slice GET and SET functions.
 */
#define IAM20380HT_GET_BITS(regvar, bitname) \
    ((regvar & IAM20380HT_ ## bitname ## _MSK) >> IAM20380HT_ ## bitname ## _POS)

#define IAM20380HT_SET_BITS(regvar, bitname, val) \
    ((regvar & (~ IAM20380HT_ ## bitname ## _MSK)) | \
      ((val << IAM20380HT_ ## bitname ## _POS) & IAM20380HT_ ## bitname ## _MSK))

#define IAM20380HT_GET_BITS_POS_0(reg_data, bitname) (reg_data & (IAM20380HT_ ## bitname ## _MSK))

#define IAM20380HT_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & (~(IAM20380HT_ ## bitname ## _MSK))) | \
      (data & IAM20380HT_ ## bitname ## _MSK))
/**@}*/

/********************************************************/

/*!
 * @defgroup group4 Type definitions.
 * @brief Project enums and structers.
 * @{
 */
/**
 * @brief Read-function prototype
 */
typedef int8_t (*iam20380ht_read_fptr_t)(struct i2c_client *client,
    uint8_t reg_addr, uint8_t *data, uint16_t len);/**
 * @brief Write-function prototype
 */
typedef int8_t (*iam20380ht_write_fptr_t)(struct i2c_client *client,
    uint8_t reg_addr, uint8_t const *data, uint16_t len);/**
 * @brief Delat-function prototype
 */
typedef void (*iam20380ht_delay_fptr_t)(uint32_t period);
/*!
 * @brief Interface selection Enums
 */
typedef enum
{
  /**
   * @brief SPI interface */
  IAM20380HT_SPI_INTF,
  /**
   * @brief I2C interface */
  IAM20380HT_I2C_INTF
} iam20380ht_intf_t;
/*!
 * @brief Select read values.
 */
typedef enum {
  IAM20380HT_DATA_TEMP_SEL,
  IAM20380HT_DATA_ONLY_GYRO_SEL,
} data_sel_t;
/*!
 * @brief Clock source enum
 */
typedef enum
{
  /**
   * @brief Internal 20 MHz oscillator. */
  IAM20380HT_CLKSEL_INTERNAL  = 0,  /**
   * @brief Auto selects the best available
   * clock source – PLL if ready, else use
   * the Internal oscillator. */
  IAM20380HT_CLKSEL_AUTO      = 1,  /**
   * @brief Stops the clock and keeps timing generator in reset. */
  IAM20380HT_CLKSEL_STOP      = 7,} iam20380ht_clk_t;
/*!
 * @brief iam20380ht interrupt type selection enum
 */
typedef enum
{
  IAM20380HT_FIFO_OFLOW,  ///< Fifo overflow

  IAM20380HT_GDRIVE_INT,  ///< Motion Detection

  IAM20380HT_RDY_INT      ///< Data ready interrupt
} iam20380ht_int_t;
/**
 * @{ \name Output Data Rate.
 *  Values of SMPLRT_DIV (0x19) register.
 */
typedef enum {
  IAM20380HT_ODR_1000HZ = 0,
  IAM20380HT_ODR_500HZ  = 1,
  IAM20380HT_ODR_333HZ  = 2,
  IAM20380HT_ODR_250HZ  = 3,
  IAM20380HT_ODR_200HZ  = 4,
  IAM20380HT_ODR_125HZ  = 7,
  IAM20380HT_ODR_100HZ  = 9,
  IAM20380HT_ODR_50HZ   = 19,
  IAM20380HT_ODR_30HZ   = 32,
} smplrt_div_t;
/**@}*/

/**
 * @{ \name Angular rate measurement range.
 * @brief FS_SEL bit fild of reg GYRO_CONFIG (0x1B)
 */
typedef enum {
  IAM20380HT_RANGE_250_DPS  = 0,
  IAM20380HT_RANGE_500_DPS,
  IAM20380HT_RANGE_1000_DPS,
  IAM20380HT_RANGE_2000_DPS,
} arm_range_t;
/**@}*/

/**
 * @{ \name BW modes.
 * @brief For bandwidth = IAM20380HT_BW_NORMAL_MODE, gyro data is
 * sampled at equidistant points in the time defined by ODR.
 * For bandwidth = IAM20380HT_BW_OSR2_MODE, both stages of digital
 * filter are used & data is oversampled
 * with an oversampling rate of 2. The ODR has to be 2 times higher
 * than that of the normal mode.
 * For bandwidth = IAM20380HT_BW_OSR4_MODE, both stages of digital
 * filter are used & data is oversampled
 * with an oversampling rate of 4. The ODR has to be 4 times
 * higher than that of the normal mode.
 *
 *
 */
typedef enum {
  IAM20380HT_BW_NORMAL_MODE   = 0,
  IAM20380HT_BW_OSR2_MODE     = 1,
  IAM20380HT_BW_OSR4_MODE     = 2
} fchoiceb_t;
/**@}*/

/**
 * @{ \name DLPF_CFG values.
 * @brief Values for bandwidth.
 */
typedef enum
{
  IAM20380HT_BANDWIDTH_250HZ  = 0,
  IAM20380HT_BANDWIDTH_176HZ  = 1,
  IAM20380HT_BANDWIDTH_92HZ   = 2,
  IAM20380HT_BANDWIDTH_41HZ   = 3,
  IAM20380HT_BANDWIDTH_20HZ   = 4,
  IAM20380HT_BANDWIDTH_10HZ   = 5,
  IAM20380HT_BANDWIDTH_5HZ    = 6,
  IAM20380HT_BANDWIDTH_3281HZ = 7
} dlpf_t;

typedef struct
{
  /**
   * @brief FCHOICE_B bit field of 0x1B register. See 'fchoiceb_t'. */
  uint8_t   mode;
  /**
   * @brief DLPF_CFG bit field of 0x1A register. See 'dlpf_t'. */
  uint8_t   dlpf;
} bw_t;
/**@}*/

/*!
 * @brief iam20380ht sensor configuration structure
 */
typedef struct
{
  /**
   * @brief Angular rate measurement rate. FS_SEL. See 'arm_range_t'. */
  uint8_t   range;
  /**
   * @brief Bandwidth. FCHOICE_B & DLPF_CFG. See 'bw_t'. */
  bw_t      bw;
  /**
   * @brief Sample rate divider, corresponding to selected ODR (output data rate).
   * See 'smplrt_div_t'. */
  uint8_t   smplrt_div;
} iam20380ht_cfg;

/*!
 * @brief iam20380ht sensor data structure which has gyro data,
 * temperature, timestamp.
 */
typedef struct
{
  union
  {
    struct
    {
      int16_t x;  ///< X-axis sensor data
      int16_t y;  ///< Y-axis sensor data
      int16_t z;  ///< Z-axis sensor data
    };
    int16_t   data[3];
  };
  uint16_t    temp;
  timeval     timestamp;
} iam20380ht_sensor_data;

/*!
 *  @brief This structure holds the information for usage of
 *  FIFO by the user.
 */
typedef struct
{
  /**
   * @brief Data buffer of user defined length is to be mapped here */
  uint8_t     *data;
  /**
   * @brief While calling the API  "iam20380ht_get_fifo_data" , length stores
   *  number of bytes in FIFO to be read (specified by user as input)
   *  and after execution of the API ,number of FIFO data bytes
   *  available is provided as an output to user
   */
  uint16_t    length;
  /**
   * @brief Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate */
  uint8_t     fifo_temp_enable;
  /**
   * @brief  Can be one of these: FIFO_DIS, FIFO_RST, FIFO_EN */
  fifo_mode_t fifo_mode;
  /**
   * @brief Write GYRO_X/Y/ZOUT_H and GYRO_X/Y/ZOUT_L to the FIFO at the sample rate */
  uint8_t     fifo_gyro_enable;
  /**
   * @brief Streaming of the data in FIFO */
  uint8_t     fifo_enable;
  /**
   * @brief Will be equal to length when no more frames are there to parse */
  uint16_t    gyro_byte_start_idx;
  /**
   * @brief Value of Skipped frame counts */
  uint8_t     skipped_frame_count;
  /**
   * @brief Value to select FIFO downsampling and
   *  filtered/unfiltered data in FIFO
   */
  uint8_t     fifo_down;
} iam20380ht_fifo_frame;

/*!
 * @brief iam20380ht device structure.
 */
typedef struct
{
  /**
   * @brief Chip Id. */
  uint8_t                       chip_id;
  /**
   * @brief i2c client. */
  struct i2c_client             *i2c_client;
  /**
   * @brief SPI/I2C interface. */
  iam20380ht_intf_t          intf;
  /**
   * @brief Read function pointer. */
  iam20380ht_read_fptr_t        read;
  /**
   * @brief Write function pointer. */
  iam20380ht_write_fptr_t       write;
  /**
   * @brief Delay function pointer. */
  iam20380ht_delay_fptr_t       delay_ms;
  /**
   * @brief FIFO related configurations. */
  iam20380ht_fifo_frame         *fifo;
  /**
   * @brief Power mode of sensor. */
  iam20380ht_pm_mode_t          power_mode;
  /**
   * @brief Current output data rate. */
  smplrt_div_t                  odr;
} iam20380ht_dev;
/**@}*/
/**@}*/
/**@}*/
/**@}*/
#endif /* IAM20380HT_DEFS_H_ */