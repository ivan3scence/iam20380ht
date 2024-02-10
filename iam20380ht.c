/**
 * @file iam20380ht.c
 * @author ivan.gorich@gmail.com
 * @brief Driver internal API.
 * @version 0.1
 * @date 2024-01-27
 */

#include "iam20380ht.h"


/*********************************************************************/

/**
 * @{ \name Static function declarations.
 */

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev    : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t null_ptr_check(const iam20380ht_dev *dev);

/*!
 *  @brief This internal API is used to reset the FIFO related configurations
 *  in the iam20380ht_fifo_frame structure.
 *
 * @param[in] dev             : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static void reset_fifo_data_structure(const iam20380ht_dev *dev);

/*!
 *  @brief This internal API is used to read fifo_byte_counter value (i.e)
 *  current fill-level in FIFO buffer.
 *
 * @param[out] bytes_to_read  : Number of bytes available in FIFO at the
 *                              instant obtained from FIFO counter.
 * @param[in] dev             : Structure instance of iam20380ht_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, const iam20380ht_dev *dev);

/*!
 *  @brief This internal API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 *
 *  @param[out] data_index       : The start index for parsing data
 *  @param[out] data_read_length : No of bytes to be parsed from FIFO buffer
 *  @param[in] gyro_frame_count  : Number of Gyro data frames to be read
 *  @param[in] dev               : Structure instance of iam20380ht_dev.
 */
static void get_gyro_len_to_parse(uint16_t *data_index,
                  uint16_t *data_read_length,
                  const uint8_t *gyro_frame_count,
                  const iam20380ht_dev *dev);

/*!
 *  @brief This internal API checks the presence of non-valid frames in the read fifo data.
 *
 *  @param[in,out] data_index    : The index of the current data to
 *                                 be parsed from fifo data
 *  @param[in] dev               : Structure instance of iam20380ht_dev.
 */
static void check_frame_validity(uint16_t *data_index, const iam20380ht_dev *dev);

/*!
 *  @brief This internal API is used to parse the gyroscope's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 *  @param[in,out] gyro     : structure instance of sensor data
 *  @param[in,out] idx      : Index value of number of bytes parsed
 *  @param[in,out] gyro_idx : Index value of gyro data
 *                                (x,y,z axes) frames parsed
 *  @param[in] frame_info       : It consists of either fifo_data_enable
 *                                parameter in header-less mode or
 *                                frame header data in header mode
 *  @param[in] dev      : structure instance of iam20380ht_dev.
 */
static void unpack_gyro_frame(iam20380ht_sensor_data *gyro,
    uint16_t *idx, uint8_t *gyro_idx, uint8_t frame_info,
    timeval *current_time, const iam20380ht_dev *dev);

/*!
 * @brief Reads bytes with data and fill with them iam20380ht_sensor_data struct.
 *
 * @param[in] gyro            : Struct to fill.
 * @param[in] start_index     : Start index of data_array from which to read.
 * @param[in] data_array      : Array with data for reading.
 * @param[in] read_temp       : If 1 - reads temperature too.
 */
static void iam20380ht_get_gyro_data(iam20380ht_sensor_data *gyro,
    uint16_t start_index, uint8_t *data_array, uint8_t read_temp);

/**
 * @brief Sets timestamp for gyro frame.
 * Adds IAM20380HT_US_PER_FRAME microseconds
 * for next frame timestamp.
 * 
 * @param current_time roughly time of current frame.
 * @param new_time pointer to timestamp of current frame.
 * @param odr this needs to calculate time delay between two consecutive frames.
 */
static void iam20380ht_make_timestamp(timeval *current_time,
    timeval *new_time, const smplrt_div_t odr);

/**
 * @}
 */
/*********************************************************************/

/**
 * @{ \name User function definitions.
 */

int8_t iam20380ht_init(iam20380ht_dev *dev)
{
  int8_t  rslt;
  uint8_t data;

  rslt = null_ptr_check(dev);
  if (rslt == IAM20380HT_OK)
  {
    rslt = iam20380ht_soft_reset(dev);
    if (rslt != IAM20380HT_OK)
    {
      IAM_INFO("Could not soft reset sensor!\n");
      return rslt;
    }
    /* Setting the power mode as normal mode without temperature sensor */
    dev->power_mode = IAM20380HT_PM_NORMAL_MODE;
    rslt = iam20380ht_set_power_mode(dev);
    IAM_INFO("power_mode set to 0x%02X\n", dev->power_mode);
    if (rslt != IAM20380HT_OK)
      IAM_ERR("iam20380ht_set_power_mode status %d\n", rslt);

    rslt = iam20380ht_set_fifo_en(IAM20380HT_FIFO_EN, dev);
    if (rslt != IAM20380HT_OK)
      IAM_ERR("iam20380ht_set_fifo_en status %d\n", rslt);

  }
  return rslt;
}

int8_t iam20380ht_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const iam20380ht_dev *dev)
{
  int8_t rslt;

  rslt = null_ptr_check(dev);
  if ((rslt == IAM20380HT_OK) && (data != NULL))
  {
    rslt = dev->read(dev->i2c_client, reg_addr, data, len);

    /* Delay for proper data read */
    dev->delay_ms(1);
    if (rslt != IAM20380HT_OK)
      rslt = IAM20380HT_E_COM_FAIL;
  }
  else
    rslt = IAM20380HT_E_NULL_PTR;

  return rslt;
}

int8_t iam20380ht_set_regs(uint8_t reg_addr, uint8_t const *data, uint16_t len,
    const iam20380ht_dev *dev)
{
  int8_t    rslt;
  uint16_t  count = 0;

  rslt = null_ptr_check(dev);
  if ((rslt == IAM20380HT_OK) && (data != NULL))
  {
    /* Burst write is allowed only in normal mode */
    if (dev->power_mode == IAM20380HT_PM_NORMAL_MODE)
    {
      rslt = dev->write(dev->i2c_client, reg_addr, data, len);
      dev->delay_ms(1);
    }
    else
    {	
      for (; count < len; count++)
      {
        rslt = dev->write(dev->i2c_client, reg_addr, &data[count], 1);
        reg_addr++;
        dev->delay_ms(1);
      }
    }
    if (rslt != IAM20380HT_OK)
      rslt = IAM20380HT_E_COM_FAIL;
  }
  else
    rslt = IAM20380HT_E_NULL_PTR;

  return rslt;
}

int8_t iam20380ht_soft_reset(const iam20380ht_dev *dev)
{
  int8_t  rslt;
  uint8_t data = IAM20380HT_TEMP_RST;

  rslt = iam20380ht_set_regs(IAM20380HT_ADDR_SIGNAL_PATH_RESET, &data, 1, dev);
  rslt = iam20380ht_set_fifo_en(IAM20380HT_FIFO_RST | IAM20380HT_SIG_COND_RST, dev);
  data = IAM20380HT_DEVICE_RESET_MSK | IAM20380HT_CLKSEL_AUTO;
  rslt = iam20380ht_set_regs(IAM20380HT_ADDR_PWR_MGMT_1, &data, 1, dev);
  if (rslt == IAM20380HT_OK)
    dev->delay_ms(IAM20380HT_AFTER_SLEEP_DELAY);
  
  rslt = iam20380ht_check_chip_id(dev);
  if (rslt != IAM20380HT_OK)
    IAM_ERR("failed to check CHIP ID: %d\n", rslt);

  return rslt;
}

int8_t iam20380ht_set_power_mode(const iam20380ht_dev *dev)
{
  int8_t  rslt;
  uint8_t gyro_pmu_status = 0;
  uint8_t was_sleeping    = FALSE;

  switch (dev->power_mode)
  {
    case IAM20380HT_PM_NORMAL_MODE:
    case IAM20380HT_PM_TEMP_OFF:
    case IAM20380HT_PM_STANDBY:
    case IAM20380HT_PM_SLEEP:
/*TODO low noise mode*/
    case IAM20380HT_PM_LOW_NOISE:
      rslt = iam20380ht_get_regs(IAM20380HT_ADDR_PWR_MGMT_1, &gyro_pmu_status, 1, dev);
      if (IAM20380HT_IS_SLEEPING(gyro_pmu_status))
        was_sleeping = TRUE;
      gyro_pmu_status = IAM20380HT_SET_PM(gyro_pmu_status, dev->power_mode);
      rslt = iam20380ht_set_regs(IAM20380HT_ADDR_PWR_MGMT_1, &gyro_pmu_status, 1, dev);
      if (rslt == IAM20380HT_OK && was_sleeping)
        dev->delay_ms(IAM20380HT_AFTER_SLEEP_DELAY);
      break ;
    default:
      rslt = IAM20380HT_E_INVALID_INPUT;
      break ;
  }
  return rslt;
}

int8_t iam20380ht_set_sensor_settings(const iam20380ht_cfg *gyro_cfg,
    const iam20380ht_dev *dev)
{
  int8_t	rslt;
  uint8_t	data;

  iam20380ht_get_regs(IAM20380HT_ADDR_GYRO_CONFIG, &data, 1, dev);
  data = IAM20380HT_SET_BITS(data, FS_SEL, gyro_cfg->range);
  data = IAM20380HT_SET_BITS(data, FCHOICE_B, gyro_cfg->bw.mode);
  IAM_INFO("setting fs_sel to %d, "
          "fchoice_b to %d, register value: 0x%02X\n",
          gyro_cfg->range, gyro_cfg->bw.mode, data);
  rslt = iam20380ht_set_regs(IAM20380HT_ADDR_GYRO_CONFIG, &data, 1, dev);

  if (gyro_cfg->bw.mode == IAM20380HT_BW_NORMAL_MODE)
  {
    IAM_INFO("setting dlfp\n");
    rslt = iam20380ht_set_bw(gyro_cfg->bw.dlpf, dev);
    if (rslt != IAM20380HT_OK)
      IAM_ERR("ERROR iam20380ht_set_bw status %d", rslt);
    if (gyro_cfg->bw.dlpf != IAM20380HT_BANDWIDTH_250HZ
            && gyro_cfg->bw.dlpf != IAM20380HT_BANDWIDTH_3281HZ)
    {
      iam20380ht_get_regs(IAM20380HT_ADDR_SMPLRT_DIV, &data, 1, dev);
      data = IAM20380HT_SET_BITS(data, SMPLRT_DIV, gyro_cfg->smplrt_div);
      IAM_INFO("setting sample rate divider to %hd, "
              "register value: 0x%02X\n", gyro_cfg->smplrt_div, data);
      rslt = iam20380ht_set_regs(IAM20380HT_ADDR_SMPLRT_DIV, &data, 1, dev);
    }
  }

  return rslt;
}

int8_t iam20380ht_set_bw(const dlpf_t dlpf, const iam20380ht_dev *dev)
{
  int8_t	rslt;
  uint8_t	data = 0;

  if (iam20380ht_get_regs(IAM20380HT_ADDR_CONFIG, &data, 1, dev) != 0)
    return IAM20380HT_FAIL;
  data = IAM20380HT_SET_BITS(data, DLPF_CFG, UINT8_C(dlpf));
  rslt = iam20380ht_set_regs(IAM20380HT_ADDR_GYRO_CONFIG, &data, 1, dev);
  IAM_INFO("setting dlpf to %hd, data: 0x%02X\n",
      dlpf, data);

  return rslt;
}

int8_t iam20380ht_get_sensor_settings(iam20380ht_cfg *gyro_cfg,
    const iam20380ht_dev *dev)
{
  uint8_t	data = 0;

  if (iam20380ht_get_regs(IAM20380HT_ADDR_GYRO_CONFIG, &data, 1, dev)
          != IAM20380HT_OK)
    return IAM20380HT_FAIL;
  gyro_cfg->range = IAM20380HT_GET_BITS(data, FS_SEL);
  gyro_cfg->bw.mode = IAM20380HT_GET_BITS(data, FCHOICE_B);

  if (iam20380ht_get_regs(IAM20380HT_ADDR_CONFIG, &data, 1, dev)
          != IAM20380HT_OK)
    return IAM20380HT_FAIL;
  gyro_cfg->bw.dlpf = IAM20380HT_GET_BITS(data, DLPF_CFG);

  if (iam20380ht_get_regs(IAM20380HT_ADDR_SMPLRT_DIV, &data, 1, dev)
          != IAM20380HT_OK)
    return IAM20380HT_FAIL;
  gyro_cfg->smplrt_div = IAM20380HT_GET_BITS(data, SMPLRT_DIV);

  return IAM20380HT_OK;
}

int8_t iam20380ht_set_fifo_config(const uint8_t config, const uint8_t enable,
    const iam20380ht_dev *dev)
{
  int8_t	rslt;
  uint8_t	reg_data;

  rslt = iam20380ht_get_regs(IAM20380HT_ADDR_FIFO_EN, &reg_data, 1, dev);
  if (rslt == IAM20380HT_OK)
  {
    if (config > 0)
    {
      if (enable == IAM20380HT_ENABLE)
        reg_data = reg_data | config;
      else
        reg_data = reg_data & (~config);
      rslt = iam20380ht_set_regs(IAM20380HT_ADDR_FIFO_EN, &reg_data, 1, dev);
      if (rslt == IAM20380HT_OK)
      {
        dev->fifo->fifo_gyro_enable =
                           IAM20380HT_GET_BITS(reg_data, XG_FIFO_EN)
                        && IAM20380HT_GET_BITS(reg_data, YG_FIFO_EN)
                        && IAM20380HT_GET_BITS(reg_data, ZG_FIFO_EN);
        dev->fifo->fifo_temp_enable = IAM20380HT_GET_BITS(reg_data,
            TEMP_FIFO_EN);
      }
    }
  }

  return rslt;
}

int8_t iam20380ht_set_fifo_onovflw(const uint8_t mode,
    const iam20380ht_dev *dev)
{
  int8_t	rslt;

  rslt = iam20380ht_set_regs(IAM20380HT_ADDR_CONFIG, &mode, 1, dev);
  if (rslt == IAM20380HT_OK)
    dev->fifo->fifo_mode = mode;

  return rslt;
}

int8_t iam20380ht_set_fifo_en(const uint8_t mode,
    const iam20380ht_dev *dev)
{
  int8_t	rslt;

  rslt = iam20380ht_set_regs(IAM20380HT_ADDR_USER_CTRL, &mode, 1, dev);
  if (rslt == IAM20380HT_OK)
  {
    dev->fifo->fifo_enable = IAM20380HT_FIFO_MODE(mode);
    IAM_INFO("fifo_enable = %d\n", dev->fifo->fifo_enable);
  }
    
  return rslt;
}

int8_t  iam20380_setup_fifo(const iam20380ht_dev *dev)
{
  int8_t  rslt;

  rslt = iam20380ht_set_fifo_en(IAM20380HT_FIFO_EN, dev);
  if (rslt != IAM20380HT_OK)
    IAM_ERR("iam20380ht_set_fifo_en status %d\n", rslt);

  rslt = iam20380ht_set_fifo_config(FIFO_EN_XYZ, IAM20380HT_ENABLE, dev);
  if (rslt != IAM20380HT_OK)
    IAM_ERR(" Could not set fifo-XYZ enable: %d", rslt);

  rslt = iam20380ht_set_fifo_onovflw(IAM20380HT_FIFO_REWRITE_OLD, dev);
  if (rslt != IAM20380HT_OK)
    IAM_ERR("Could not set fifo on overflow mode:%d\n", rslt);

  return rslt;
}

int8_t iam20380ht_extract_gyro(iam20380ht_sensor_data *gyro_data,
    uint8_t *data_length, timeval *current_time, const iam20380ht_dev *dev)
{
  int8_t    rslt;
  uint8_t   gyro_index        = 0;
  uint16_t  data_index        = 0;
  uint16_t  data_read_length  = 0;

  /* Null-pointer check */
  rslt = null_ptr_check(dev);
  if (rslt == IAM20380HT_OK)
  {
    /* Number of bytes to be parsed from FIFO */
    get_gyro_len_to_parse(&data_index, &data_read_length, data_length, dev);
    for (; data_index < data_read_length;)
    {
      /*Check for the availability of next two bytes of FIFO data */
      check_frame_validity(&data_index, dev);
      unpack_gyro_frame(gyro_data, &data_index, &gyro_index,
                          dev->fifo->fifo_enable, current_time, dev);
    }
    /* update number of gyro data read */
    *data_length = gyro_index;
    /* update the gyro byte index */
    dev->fifo->gyro_byte_start_idx = data_index;
  }

  return rslt;
}

int8_t iam20380ht_get_sensor_data(data_sel_t data_sel, iam20380ht_sensor_data *gyro,
    const iam20380ht_dev *dev)
{
  int8_t    rslt;
  uint8_t   data_array[8] = { 0 };
  uint32_t  time_0 = 0;
  uint32_t  time_1 = 0;
  uint32_t  time_2 = 0;

  rslt = null_ptr_check(dev);
  if ((rslt == IAM20380HT_OK) && (gyro != NULL))
  {
    if (data_sel == IAM20380HT_DATA_ONLY_GYRO_SEL)
    {
      /* read gyro sensor data */
      rslt = iam20380ht_get_regs(IAM20380HT_ADDR_GYRO_XOUT_H, data_array,
                IAM20380HT_FIFO_G_LENGTH, dev);
      if (rslt == IAM20380HT_OK)
        iam20380ht_get_gyro_data(gyro, 0, data_array, FALSE);
    }
    else if (data_sel == IAM20380HT_DATA_TEMP_SEL)
    {
      /* read gyro sensor data along with sensor temperature */
      rslt = iam20380ht_get_regs(IAM20380HT_ADDR_TEMP_OUT_H, data_array,
                IAM20380HT_FIFO_TG_LENGTH, dev);
      if (rslt == IAM20380HT_OK)
        iam20380ht_get_gyro_data(gyro, 0, data_array, TRUE);
    }
    else
      rslt = IAM20380HT_E_INVALID_INPUT;
  }
  else
  {
    rslt = IAM20380HT_E_NULL_PTR;
  }

  return rslt;
}

int8_t iam20380ht_check_chip_id(iam20380ht_dev *dev)
{
  uint8_t rslt;
  uint8_t chip_id;

  rslt = iam20380ht_get_regs(IAM20380HT_ADDR_WHO_AM_I, &chip_id, 1, dev);
  if ((rslt == IAM20380HT_OK) && (chip_id == IAM20380HT_CHIP_ID))
  {
    dev->chip_id = chip_id;
    IAM_INFO("CHIP ID = 0x%02X!\n", chip_id);
  }
  else
    rslt = IAM20380HT_E_DEV_NOT_FOUND;
  
  return rslt;
}

int8_t iam20380ht_get_fifo_data(const iam20380ht_dev *dev)
{
  int8_t    rslt;
  uint16_t  bytes_to_read = 0;
  uint16_t  user_fifo_len = 0;

  rslt = null_ptr_check(dev);
  if (rslt == IAM20380HT_OK)
  {
    reset_fifo_data_structure(dev);
    rslt = get_fifo_byte_counter(&bytes_to_read, dev);
    if (rslt == IAM20380HT_OK)
    {
      user_fifo_len = dev->fifo->length;
      if (dev->fifo->length > bytes_to_read)
        dev->fifo->length = bytes_to_read;
      rslt = dev->read(dev->i2c_client, IAM20380HT_ADDR_FIFO_R_W,
          dev->fifo->data, dev->fifo->length);
    }
  }

  return rslt;
}

int8_t  iam20380_get_odr(int32_t * const dst, const iam20380ht_dev * const dev)
{
  uint8_t data;

  if (iam20380ht_get_regs(IAM20380HT_ADDR_SMPLRT_DIV, &data, 1, dev) != IAM20380HT_OK)
    return IAM20380HT_FAIL;
  *dst = (int32_t) IAM20380HT_GET_BITS(data, SMPLRT_DIV);

  return IAM20380HT_OK;
}

#ifdef IAM_DEBUG
void show_gyro_data(uint8_t* data, int len)
{
  int i = 0;
  if (len % 6 != 0)
  {
    printk("%s %d Unulignded size %d\n",__func__, __LINE__, len);
  }
  for( i =0; i<len; i+=6)
  {
    printk("%s %d [ %d] %02X %02X %02X %02X %02X %02X\n",
        __func__, __LINE__, i,data[i], data[i+1], data[i+2],
        data[i+3], data[i+4], data[i+5]);
  }
}
#endif
/**
 * @}
 */
/*********************************************************************/
/**
 * @{ \name Static function definitions.
 */

/*!
 * @brief This internal API is used to validate the device structure pointer
 * for null conditions.
 */
static int8_t null_ptr_check(const iam20380ht_dev *dev)
{
  int8_t rslt;

  if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    rslt = IAM20380HT_E_NULL_PTR;

  else
    /* Device structure is fine */
    rslt = IAM20380HT_OK;

  return rslt;
}

static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, const iam20380ht_dev *dev)
{
  int8_t    rslt;
  uint8_t   data;
  uint16_t  count = 0;

  rslt = iam20380ht_get_regs(IAM20380HT_ADDR_FIFO_COUNTL, &data, 1, dev);
  count = (count | IAM20380HT_GET_BITS(data, FIFO_COUNTL));
  rslt = iam20380ht_get_regs(IAM20380HT_ADDR_FIFO_COUNTH, &data, 1, dev);
  count = count | (IAM20380HT_GET_BITS(data, FIFO_COUNTH) << IAM20380HT_FIFO_COUNTH_SHIFT);
  IAM_INFO("count = %hu\n", __func__, __LINE__, count);
  *bytes_to_read = count;
  return rslt;
}

/*!
 *  @brief This API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_gyro_len_to_parse(uint16_t *data_index,
                  uint16_t                 *data_read_length,
                  const uint8_t            *gyro_frame_count,
                  const iam20380ht_dev     *dev)
{
  *data_index = dev->fifo->gyro_byte_start_idx;
  if (dev->fifo->fifo_enable == IAM20380HT_ENABLE)
    *data_read_length = (*gyro_frame_count) * IAM20380HT_FIFO_G_LENGTH;
  else
  {
    /* When gyro data is not enabled in FIFO,
     * there will be no gyro data,
     * so we update the data index as complete
     */
    IAM_INFO("fifo was not enabled\n");
    *data_index = dev->fifo->length;
  }
  if (*data_read_length > dev->fifo->length)
  {
    /* Handling the case where more data is requested
     * than that is available
     */
    *data_read_length = dev->fifo->length;
  }
}

/*!
 *  @brief This API checks the presence of non-valid frames in the read fifo data.
 */
static void check_frame_validity(uint16_t *data_index, const iam20380ht_dev *dev)
{
  /* Check if at least 1 frame exists */
  if ((*data_index + 2) < dev->fifo->length)
  {
    /* Check if FIFO is empty */
    if ((dev->fifo->data[*data_index + 1] == 0)
        && (dev->fifo->data[*data_index] == 0))
    {
#ifdef IAM_DEBUG
      printk(" *data_index=%d\n",*data_index);
      show_gyro_data(dev->fifo->data, dev->fifo->length);
#endif
      /* Update the data index as complete */
      *data_index = dev->fifo->length;
    }
  }
}

static void iam20380ht_get_gyro_data(iam20380ht_sensor_data *gyro,
    uint16_t start_index, uint8_t *data_array, uint8_t read_temp)
{
  uint8_t i = 0;

  if (read_temp == IAM20380HT_WITH_TEMPERATURE)
  {
    gyro->temp = (int16_t)((UINT16_C(data_array[start_index++]) << 8) | data_array[start_index++]);
    /* Formula to get actual temperature in Degree Celsius. */
    gyro->temp = gyro->temp / 326 + 25;
  }
  for(; i < 3; ++i)
    gyro->data[i] = (int16_t)((UINT16_C(data_array[start_index++]) << 8) | data_array[start_index++]);
}

/*! TODO
 *  @brief This API is used to reset the FIFO related configurations
 *  in the iam20380ht_fifo_frame structure.
 *  Prepare for next FIFO read by resetting FIFO's
 *  internal data structures
 */
static void reset_fifo_data_structure(const iam20380ht_dev *dev)
{
  dev->fifo->gyro_byte_start_idx = 0;
  dev->fifo->skipped_frame_count = 0;
}

static void iam20380ht_make_timestamp(timeval *current_time,
    timeval *new_time, const smplrt_div_t odr)
{
  new_time->tv_sec = current_time->tv_sec;
  new_time->tv_usec = current_time->tv_usec;

  current_time->tv_usec += IAM20380HT_US_PER_FRAME(odr);
  if (current_time->tv_usec >= MILLION_I)
  {
    current_time->tv_sec++;
    current_time->tv_usec -= MILLION_I;
  }
  if (current_time->tv_usec < 0)
  {
    current_time->tv_sec--;
    current_time->tv_usec += MILLION_I;
  }
}

static void unpack_gyro_frame(iam20380ht_sensor_data  *gyro,
                                            uint16_t  *idx,
                                            uint8_t   *gyro_idx,
                                            uint8_t   frame_info,
                                      timeval  *current_time,
                                const iam20380ht_dev  *dev)
{
  if (frame_info == IAM20380HT_ENABLE)
  {
    /* Partial read, then skip the data */
    if ((*idx + IAM20380HT_FIFO_G_LENGTH) > dev->fifo->length)
    {
      /* Update the data index as complete */
      *idx = dev->fifo->length;
    }
    else
    {
      /*Unpack the data array into structure instance "gyro"*/
      iam20380ht_get_gyro_data(&gyro[*gyro_idx], *idx, dev->fifo->data, IAM20380HT_WO_TEMPERATURE);
      iam20380ht_make_timestamp(current_time, &gyro[*gyro_idx].timestamp, dev->odr);
      /*Move the data index*/
      (*idx) = (*idx) + IAM20380HT_FIFO_G_LENGTH;
      ++(*gyro_idx);
    }
  }
}
/**
 * @}
 */