/**
 * @file iam20380ht.h
 * @author ivan.gorich@gmail.com
 * @brief Driver internal API.
 * @version 0.1
 * @date 2024-01-27
 */


#ifndef IAM20380HT_H_
#define IAM20380HT_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "iam20380ht_defs.h"
#include "iam20380ht-ioctl.h"

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of iam20380ht sensor.
 *
 * @param[in] dev       : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_init(iam20380ht_dev *dev);

/*!
 * @brief This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] data     : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in] dev       : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const iam20380ht_dev *dev);

/*!
 * @brief This API writes the given data to the register address of sensor.
 *
 * @param[in] reg_addr  : Register address where the data is to be written.
 * @param[in] data      : Pointer to data buffer, whose data is to be written
 *                        in the sensor.
 * @param[in] len       : No of bytes of data to be writen.
 * @param[in] dev       : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_regs(uint8_t reg_addr, uint8_t const *data, uint16_t len,
    const iam20380ht_dev *dev);

/*! @brief This API turn fifo on/off/reset.
 *
 *  @param[in] mode : one of these:
      IAM20380HT_FIFO_DIS
      IAM20380HT_FIFO_RST
      IAM20380HT_FIFO_EN
 *
 *  @param[in] dev : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_fifo_en(const uint8_t mode, const iam20380ht_dev *dev);

/**
 * @brief Define fifo overwrite on overflow.
 * 
 * @param mode 1 - additional writes will not be written to FIFO.
 *  0 - additional writes will be written to FIFO, replacing the oldest data.
 * @param[in] dev : Structure instance of iam20380ht_dev.
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_fifo_onovflw(const uint8_t mode,
    const iam20380ht_dev *dev);

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 * After reset verification of WHO_AM_I is needed to confirm
 * that registers are ready to be read.
 *
 * @param[in] dev       : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_soft_reset(const iam20380ht_dev *dev);

/*!
 * @brief Reads WHO_AM_I register and compares it with manual value.
 * Saves chip id in dev struct.
 *
 * @param[in] dev       : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_check_chip_id(iam20380ht_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev     : Structure instance of iam20380ht_dev.
 *
 * @note prerequisite : Set the required macro to dev->power_mode
 * set the corresponding power mode from the below table
 *
 *       dev->power_mode            |  Power mode set
 * ---------------------------------|----------------------
 * IAM20380HT_GYRO_SUSPEND_MODE     |   Suspend mode
 * IAM20380HT_PM_NORMAL_MODE        |   Normal mode
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_power_mode(const iam20380ht_dev *dev);
/*!
 * @brief This API sets the sensor configurations like ODR, Range, BW.
 *
 * @param[in] gyro_cfg          : Structure instance to configure gyro.
 * @param[in] dev               : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_sensor_settings(const iam20380ht_cfg *gyro_cfg, const iam20380ht_dev *dev);

/*!
 * @brief This API gets the sensor configurations like
 * ODR, Range, BW from the sensor.
 *
 * @param[in] gyro_cfg          : Structure instance to configure gyro
 * @param[in] dev               : Structure instance of iam20380ht_dev.
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_get_sensor_settings(iam20380ht_cfg *gyro_cfg, const iam20380ht_dev *dev);

/*! @brief This API sets what data to save in fifo.

 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_fifo_config(const uint8_t config, const uint8_t enable, const iam20380ht_dev *dev);


/*! @brief This API allow to set bandwidth length.
 *
 *  @param[in] rate : value from enum, setting actual size of bandwidth.
 *
 *  @note : User can set bw size only when bw->mode=0 is selected
 * in FCHOICE_B field of GYRO_CONFIG register.
 * 
 *  @param[in] dev : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_set_bw(const dlpf_t rate, const iam20380ht_dev *dev);

/*! @brief Set up of FIFO buffer.
 * 1st: enable fifo.
 * 2nd: allow buffering of xyz-gyro values.
 * 3rd: set rewriting buffer on overflow.
 * 
 *  @param[in] dev : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t  iam20380_setup_fifo(const iam20380ht_dev *dev);

/*! @brief Reads 0x19 register and store value in dst.
 * 
 *  @param[in] dst : Pointer where to store read value.
 *  @param[in] dev : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t  iam20380_get_odr(int32_t * const dst, const iam20380ht_dev * const dev);

/*!
 * @brief API for value sysfs.
 *
 * @param[in] data_sel    : Selection macro to select data/data & sensor-temperature
 * @param[in] gyro        : Structure pointer to store gyro data
 * @param[in] dev         : Structure instance of iam20380ht_dev.
 *
 *  Value of "data_sel"   |  Data available in "gyro"
 * -----------------------|----------------------------
 * IAM20380HT_DATA_SEL        |   Gyro data
 * IAM20380HT_DATA_TEMP_SEL   |   Gyro data + temperature
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_get_sensor_data(data_sel_t data_sel, iam20380ht_sensor_data *gyro, const iam20380ht_dev *dev);

/*!
 *  @brief This API reads the FIFO data from the sensor in dev struct.
 *
 *  @note User has to allocate the FIFO buffer along with
 *  corresponding fifo length from his side before calling this API
 *  as mentioned in the readme.md
 *
 *  @note User must specify the number of bytes to read from the FIFO in
 *  dev->fifo->length , It will be updated by the number of bytes actually
 *  read from FIFO after calling this API
 *
 *  @param[in] dev     : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_get_fifo_data(const iam20380ht_dev *dev);

/*!
 *  @brief This API parses and extracts the gyro frames from
 *  FIFO data read by the "iam20380ht_get_fifo_data" API and stores it in
 *  the "gyro_data" structure instance.
 *
 *  @note The iam20380ht_extract_gyro API should be called only after
 *  reading the FIFO data by calling the iam20380ht_get_fifo_data() API.
 *
 *  @param[out] gyro_data       : Structure instance of iam20380ht_sensor_data
 *                                where the gyro data in FIFO is stored.
 *  @param[in,out] data_length  : Number of gyro frames (x,y,z axes data)
 *                                user needs
 *  @param[in] current_time     : Structure instance of time before function
 * call, this time is used to make timestamps for gyro frames.
 *  @param[in] dev              : Structure instance of iam20380ht_dev.
 *
 *  @note data_length is updated with the number of valid gyro
 *  frames extracted from fifo (1 gyro frame = 6 bytes) at the end of
 *  execution of this API.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t iam20380ht_extract_gyro(iam20380ht_sensor_data *gyro_data,
    uint8_t *data_length, timeval *current_time, const iam20380ht_dev *dev);
/**@}*/
/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* IAM20380HT_H_ */
