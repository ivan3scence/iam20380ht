/**
 * @file iam20380ht_driver.c
 * @author ivan.gorich@gmail.com
 * @brief IAM20380HT Linux Driver
 * @version 0.1
 * @date 2024-01-27 
 */

#include "iam20380ht_driver.h"


/*!
 *  @brief <250 current frames with gyro data.
 * Filled by kthread, user can get it with QUEUEDATAFRAME-ioctl.
 */
iam20380ht_queue_frames_t  *iam_local_queue = NULL;

#ifdef IAM_DEBUG
/*!
 *  @brief Sensetivity koefficients for gyro data.
 */
static const uint8_t gyro_sens[] = {
  [IAM20380HT_RANGE_250_DPS]  = 131,
  [IAM20380HT_RANGE_500_DPS]  = 65,
  [IAM20380HT_RANGE_1000_DPS] = 32,
  [IAM20380HT_RANGE_2000_DPS] = 16,
};

static void print_gyro_frames(const iam20380ht_queue_frames_t *gframes)
{
  int i;

  for (i = 0; i < gframes->frame_count; ++i)
    pr_info("frame[%d]: sec[%10d], usec[%06d], %04d, %04d, %04d\n", i,
        gframes->data[i].timestamp.tv_sec, gframes->data[i].timestamp.tv_usec,
        gframes->data[i].x / gyro_sens[IAM20380HT_RANGE_250_DPS],
        gframes->data[i].y / gyro_sens[IAM20380HT_RANGE_250_DPS],
        gframes->data[i].z / gyro_sens[IAM20380HT_RANGE_250_DPS]);
}
#endif

/*!
 *  @brief Kernel thread that reads FIFO buffer, pasting it in
 * 'gyro_fifo_frames', than copying it in 'iam_local_queue'.
 *
 *  @note This worker is called by 'STOREENABLE'-IOCTL,
 *  sysfs 'enable' and after suspend on driver resume.
 *
 * @retval zero
 */
static int iam_work_func(void *data)
{
  int err;
  iam_client_data                   *client_data = data;
  static iam20380ht_queue_frames_t  gyro_fifo_frames;
  iam20380ht_dev                    *dev = NULL;
  struct task_struct                *tsk = current;
  timeval                    current_time;
#ifndef NEW_KERNEL
  struct sched_param                param = { .sched_priority = 1 };
#endif

  IAM_INFO("%s %d started 0x%p\n", __func__, __LINE__, current);
#ifndef NEW_KERNEL
  sched_setscheduler(tsk, SCHED_FIFO, &param); //TODO!!!!!!!!
#endif
  /* clean queue packages */
  memset(iam_local_queue, 0, sizeof(iam20380ht_queue_frames_t));
  dev = &client_data->device;
  while(!kthread_should_stop())
  {
    int overflow = 0;
    dev->fifo->length = IAM20380HT_FIFO_RAW_DATA_BUFFER_SIZE;
    memset(&gyro_fifo_frames, 0, sizeof(gyro_fifo_frames));
#ifndef NEW_KERNEL
    do_gettimeofday(&current_time);
#else
    current_time.tv_sec = 0;
    current_time.tv_usec = 0;
#endif
    err = iam20380ht_get_fifo_data(dev);
    if (err != IAM20380HT_OK)
      IAM_ERR("iam20380ht_get_fifo_data status %d", err);
    gyro_fifo_frames.frame_count = IAM20380HT_QUEUE_MAX_FRAME_COUNT;
    err = iam20380ht_extract_gyro(&gyro_fifo_frames.data[0],
        &gyro_fifo_frames.frame_count, &current_time, dev);
    if (err != IAM20380HT_OK)
      IAM_ERR("iam20380ht_extract_gyro status %d", err);

    if (gyro_fifo_frames.frame_count == 0)
    {
      if (printk_ratelimit())
        IAM_INFO("frame_count=%d\n",gyro_fifo_frames.frame_count);
    }
    else
    {
      mutex_lock(&client_data->mutex_work);
      overflow = (iam_local_queue->frame_count + gyro_fifo_frames.frame_count) - IAM20380HT_QUEUE_MAX_FRAME_COUNT;
      if (overflow > 0)
      {
        IAM_ERR("Overflow remove: %db.", overflow);
        iam_local_queue->frame_count = iam_local_queue->frame_count - overflow;
        memmove(&iam_local_queue->data[0], &iam_local_queue->data[overflow],
            iam_local_queue->frame_count * sizeof(iam20380ht_sensor_data));
      }
      memmove(&iam_local_queue->data[iam_local_queue->frame_count],
          &gyro_fifo_frames.data[0], gyro_fifo_frames.frame_count * sizeof(iam20380ht_sensor_data));
      iam_local_queue->frame_count += gyro_fifo_frames.frame_count;
      mutex_unlock(&client_data->mutex_work);
#ifdef IAM_DEBUG
      print_gyro_frames(&gyro_fifo_frames);
#endif
#ifdef USE_WAIT_NEW_DATA
      wake_up_interruptible(&client_data->wait_lock);
#endif
    }
    usleep_range(5000, 15000);
  }
  mutex_lock(&client_data->mutex_work);
  memset(iam_local_queue, 0, sizeof(iam20380ht_queue_frames_t));
  mutex_unlock(&client_data->mutex_work);
  IAM_INFO("%s %d stopped %p\n", __func__, __LINE__, current);
  return IAM20380HT_OK;
}

/**
 * @brief Turn kworker(iam_work_func) on/off.
 * 
 * @param client_data 
 * @param enable 1 - turn of, 0 - turn off.
 * @return ssize_t 
 */
static ssize_t iam_store_queue_enable(iam_client_data *client_data,
    int enable)
{
  mutex_lock(&client_data->mutex_enable);
  if (enable != client_data->enable)
  {
    if (enable == IAM20380HT_ENABLE)
    {
      iam20380_setup_fifo(&client_data->device);
      client_data->drv_thread = kthread_create(iam_work_func,
                                client_data, IAM20380HT_DEV_NAME);
      if (!IS_ERR(client_data->drv_thread))
        wake_up_process(client_data->drv_thread);
      else
        IAM_ERR("Error on create worker function");
    }
    else
    {
      mutex_unlock(&client_data->mutex_enable);
      kthread_stop(client_data->drv_thread);
      client_data->drv_thread = NULL;
    }
    client_data->enable = enable;
  }
  mutex_unlock(&client_data->mutex_enable);

  return IAM20380HT_OK;
}

/**
 * @{ \name SYSFS functionality.
 * @brief Functions below provide some functionality,
 * that can be accessed from console.
 */

/**
 * @brief Prints chip id from register.
 * 
 * @param dev 
 * @param attr 
 * @param buf output for user.
 * @return ssize_t 
 */
static ssize_t iam_show_chip_id(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  struct input_dev const  *input = to_input_dev(dev);
  iam_client_data const   *client_data = input_get_drvdata(input);
  uint8_t                 chip_id;

  iam20380ht_get_regs(IAM20380HT_ADDR_WHO_AM_I,
      &chip_id, 1, &client_data->device);
  
  // mutex_lock(&client_data->mutex_chip_id);
  // const uint8_t chip_id = client_data->device.chip_id;
  // mutex_unlock(&client_data->mutex_chip_id);
  if (chip_id == IAM20380HT_CHIP_ID)
  {
    sprintf(buf, "Correct Chip ID(0x%02X)!\n", chip_id);
    return smile(buf);
  }
  return sprintf(buf, "Wrong Chip ID(0x%02X)!\n", chip_id);
}

/**
 * @brief Prints current iam20380ht_pm_mode_t.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_show_op_mode(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  struct input_dev const  *input        = to_input_dev(dev);
  iam_client_data const   *client_data  = input_get_drvdata(input);

  mutex_lock(&client_data->mutex_op_mode);
  const uint8_t op_mode = client_data->device.power_mode;
  mutex_unlock(&client_data->mutex_op_mode);

  return sprintf(buf, "%d\n", op_mode);
}

/**
 * @brief Sets iam20380ht_pm_mode_t.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @param count 
 * @return ssize_t 
 */
static ssize_t iam_store_op_mode(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
  int               err;
  unsigned long     op_mode;
  struct input_dev  *input = to_input_dev(dev);
  iam_client_data   *client_data = input_get_drvdata(input);


  err = kstrtoul(buf, 10, &op_mode);
  if (err)
    return err;

  mutex_lock(&client_data->mutex_op_mode);
  client_data->device.power_mode = op_mode;
  err = iam20380ht_set_power_mode(&client_data->device);
  mutex_unlock(&client_data->mutex_op_mode);
  if (err != IAM20380HT_OK)
    IAM_ERR("iam20380ht_set_power_mode %d\n", err);

  return err ? err : count;
}

/**
 * @brief Reads gyro data and current temperature.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_show_value(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  int                     err;
  int                     count;
  struct input_dev const  *input        = to_input_dev(dev);
  iam_client_data const   *client_data  = input_get_drvdata(input);
  iam20380ht_sensor_data  res;

  err = iam20380ht_get_sensor_data(IAM20380HT_DATA_TEMP_SEL,
      &res, &client_data->device);
  if (err != IAM20380HT_OK)
   IAM_ERR("iam20380ht_get_sensor_settings status %d", err);

  count = sprintf(buf, "x: %hd\ny: %hd\nz: %hd\ntemperature: %hd\n",
      res.x,
      res.y,
      res.z,
      res.temp);

  return count;
}

/**
 * @brief Reads sensor angular measurement range.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_show_range(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  int                     err;
  iam20380ht_cfg          gyro_cfg;
  struct input_dev const  *input        = to_input_dev(dev);
  iam_client_data const   *client_data  = input_get_drvdata(input);
  
  err = iam20380ht_get_sensor_settings(&gyro_cfg, &client_data->device);
  if (err != IAM20380HT_OK)
  {
    IAM_ERR("iam20380ht_get_sensor_settings status %d\n", err);
  }

  err = sprintf(buf, "%d\n", gyro_cfg.range);
  return err;
}

/**
 * @brief Sets sensor angular measurement range.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_store_range(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
  int               err;
  unsigned long     range;
  struct input_dev  *input = to_input_dev(dev);
  iam_client_data   *client_data = input_get_drvdata(input);

  err = kstrtoul(buf, 10, &range);
  dev_info(DEV_FROM_CLIENT_DATA(client_data),
      "range = %hd\n", (int)range);
  if (err)
    return err;
  err = iam_set_odr_range_bw(client_data, range, NULL, -1);
  if (err != IAM20380HT_OK)
  {
    IAM_ERR("iam_set_odr_range_bw status %d", err);
  }

  return count;
}

/**
 * @brief Reads sensor bandwidth.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_show_bandwidth(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  int                     err;
  iam20380ht_cfg          gyro_cfg;
  struct input_dev const  *input = to_input_dev(dev);
  iam_client_data const   *client_data = input_get_drvdata(input);
  
  err = iam20380ht_get_sensor_settings(&gyro_cfg, &client_data->device);
  if (err != IAM20380HT_OK)
  {
    IAM_ERR("iam20380ht_get_sensor_settings status %d", err);
  }

  err = sprintf(buf, "mode: %d\ndlpf: %d\n", gyro_cfg.bw.mode, gyro_cfg.bw.dlpf);
  return err;
}

/**
 * @brief Sets DLPF_CFG bit field.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @param count 
 * @return ssize_t 
 */
static ssize_t iam_store_bandwidth(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
  int               err;
  unsigned long     toul;
  struct input_dev  *input = to_input_dev(dev);
  iam_client_data   *client_data = input_get_drvdata(input);
  bw_t              bw = {
    .mode = IAM20380HT_BW_NORMAL_MODE,
    .dlpf   = 0
  };
  
  err = kstrtoul(buf, 10, &toul);
  bw.dlpf = INT8_C(toul);
  dev_info(DEV_FROM_CLIENT_DATA(client_data),
      "bw.dlpf = %hd\n", bw.dlpf);
  if (err)
    return err;
  err = iam_set_odr_range_bw(client_data, -1, &bw, -1);
  if (err != IAM20380HT_OK)
  {
    IAM_ERR("iam_set_odr_range_bw status %d", err);
  }
  return count;
}

/**
 * @brief Checks if kworker (iam_work_func) is working.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_show_enable(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  int                     err;
  struct input_dev const  *input = to_input_dev(dev);
  iam_client_data const   *client_data = input_get_drvdata(input);

  mutex_lock(&client_data->mutex_enable);
  err = sprintf(buf, "%d\n", client_data->enable);
  mutex_unlock(&client_data->mutex_enable);
  return err;
}

/**
 * @brief Turn on/off kworker (iam_work_func).
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @param count 
 * @return ssize_t 
 */
static ssize_t iam_store_enable(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
  int               err;
  unsigned long     data;
  struct input_dev  *input        = to_input_dev(dev);
  iam_client_data   *client_data  = input_get_drvdata(input);

  err = kstrtoul(buf, 10, &data);
  if (err)
    return err;

  data = data ? IAM20380HT_ENABLE : IAM20380HT_DISABLE;
  iam_store_queue_enable(client_data, data);
  return count;
}

/**
 * @brief Soft reset of sensor.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @param count 
 * @return ssize_t 
 */
static ssize_t iam_store_softreset(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
  int                     err;
  struct input_dev const  *input       = to_input_dev(dev);
  iam_client_data const   *client_data = input_get_drvdata(input);

  err = iam20380ht_soft_reset(&client_data->device);
  // err = iam_reconfigure(&client_data);
  return err ? err : count;
}

/**
 * @brief Prints values of registers.
 * 
 * @param dev 
 * @param attr 
 * @param buf 
 * @return ssize_t 
 */
static ssize_t iam_show_dumpreg(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  int                     i;
  size_t                  count = 0;
  uint8_t                 reg[0x40];
  struct input_dev const  *input        = to_input_dev(dev);
  iam_client_data const   *client_data  = input_get_drvdata(input);

  iam_dump_reg(client_data->device.i2c_client);
  return count;
}

static DEVICE_ATTR(chip_id, S_IRUGO,
    iam_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
    iam_show_op_mode, iam_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
    iam_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR,
    iam_show_range, iam_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR,
    iam_show_bandwidth, iam_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
    iam_show_enable, iam_store_enable);
#ifdef DELETEOLD
static DEVICE_ATTR(selftest, S_IRUGO,
    iam_show_selftest, NULL);
#endif
#ifdef IAM_DEBUG
static DEVICE_ATTR(softreset, S_IRUGO|S_IWUSR,
    NULL, iam_store_softreset);
static DEVICE_ATTR(regdump, S_IRUGO,
    iam_show_dumpreg, NULL);
#endif

static struct attribute *iam_attributes[] = {
  &dev_attr_chip_id.attr,
  &dev_attr_op_mode.attr,
  &dev_attr_value.attr,
  &dev_attr_range.attr,
  &dev_attr_bandwidth.attr,
  &dev_attr_enable.attr,
  &dev_attr_softreset.attr,
  &dev_attr_regdump.attr,
  NULL
};

static struct attribute_group iam_attribute_group = {
  .attrs = iam_attributes
};

/*!
 * @brief This internal API is used to register kernel input device.
 * Register device that is able to describe absolute axis value changes (EV_ABS).
 * Sets max/min values.
 * 
 * @retval zero -> Success / -ve value -> Error
 */
static int iam_input_init(iam_client_data *client_data)
{
  struct input_dev *dev;
  int err = 0;

  dev = input_allocate_device();
  if (NULL == dev)
    return -ENOMEM;

  dev->name = SENSOR_NAME;
  dev->id.bustype = BUS_I2C;

  input_set_capability(dev, EV_ABS, ABS_MISC);
  input_set_abs_params(dev, ABS_X, IAM_VALUE_MIN, IAM_VALUE_MAX, 0, 0);		//< noise
  input_set_abs_params(dev, ABS_Y, IAM_VALUE_MIN, IAM_VALUE_MAX, 0, 0);
  input_set_abs_params(dev, ABS_Z, IAM_VALUE_MIN, IAM_VALUE_MAX, 0, 0);
  input_set_drvdata(dev, client_data);

  err = input_register_device(dev);
  if (err < 0)
  {
    input_free_device(dev);
    return err;
  }
  client_data->input = dev;

  return IAM20380HT_OK;
}

static void iam_input_destroy(iam_client_data *client_data)
{
  struct input_dev *dev = client_data->input;
  if (dev)
  {
    input_unregister_device(dev);
    input_free_device(dev);
  }
}

/*!
 *  @brief Sets default sensor settings.
 *
 *  @param[in] client_data : Structure instance of client data.
 *
 * @return Result of API execution status
 * @retval zero -> Success / nonzero value -> Error
 */
static int iam_reconfigure(iam_client_data *client_data)
{
  int   err;
  bw_t  bw = {
    .mode = IAM20380HT_BW_OSR2_MODE,
    .dlpf   = 0
  };

  err = iam20380_setup_fifo(&client_data->device);
  if (err != IAM20380HT_OK)
    IAM_ERR("failed to set up fifo buffer!\n");

  err = iam_set_odr_range_bw(client_data, IAM20380HT_RANGE_250_DPS,
      &bw, IAM20380HT_ODR_200HZ);
  if (err != IAM20380HT_OK)
    IAM_ERR("iam_set_odr_range_bw %d", err);

  return err;
}

/**
 * @{ \name CDEV functionality.
 */

/**
 * @brief Handler of opening /dev/IAM20380HT_DEV_NAME
 * 
 * @param inode 
 * @param filep 
 * @return int 
 */
static int cdev_open(struct inode *inode, struct file *filep)
{
  iam_client_data *client_data;

  client_data = container_of((struct cdev*)inode->i_cdev,
      iam_client_data, kernel_cdev);

  filep->private_data = client_data;
  if (client_data == NULL)
    IAM_ERR("client data = NULL\n");

  client_data->client_count++;

  IAM_INFO("Add client #%d\n", client_data->client_count);

  return IAM20380HT_OK;
}

/**
 * @brief Handler of closing /dev/IAM20380HT_DEV_NAME
 * 
 * @param inode 
 * @param filp 
 * @return int 
 */
static int cdev_release(struct inode *inode, struct file *filp)
{
  iam_client_data *client_data;

  client_data = (iam_client_data *)filp->private_data;
  if (client_data == NULL)
  {
    IAM_ERR("client_data = NULL\n");
    return -EFAULT;
  }
  client_data->client_count--;
  IAM_INFO("Release client #%d\n", client_data->client_count);

  return IAM20380HT_OK;
}

/**
 * @brief Handler of ioctls /dev/IAM20380HT_DEV_NAME
 * 
 * @param filp 
 * @param cmd 
 * @param arg 
 * @return long 
 */
static long cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int                     value = IAM20380HT_FAIL;
  int                     err;
  bw_t                    bw;
  struct device           *dev;
  iam20380ht_sensor_data  value_data;
  iam_client_data         *client_data;

  client_data = (iam_client_data *)filp->private_data;
  dev = DEV_FROM_CLIENT_DATA(client_data);
  if (client_data == NULL)
  {
    IAM_ERR("iam20380ht client data is NULL\n");
    return -EFAULT;
  }

  if (_IOC_TYPE(cmd) != IAM20380HT_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > IAM20380HT_IOC_MAXNR) return -ENOTTY;

#ifdef NEW_KERNEL
  if (_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
#else
  if (_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
#endif

  if (err) return -EFAULT;

  switch(cmd)
  {
  case IAM20380HT_IO_GDATA:
    iam20380ht_get_sensor_data(IAM20380HT_DATA_TEMP_SEL,
        &value_data, &client_data->device);
    if (copy_to_user((void *)arg, &value_data, sizeof(iam20380ht_sensor_data)))
    {
      IAM_ERR( "Copying gyro data failed\n");
      return -EFAULT;
    }
    break;
  case IAM20380HT_IO_SBANDTWITH:
    if (copy_from_user(&bw, (bw_t *)arg, sizeof(bw_t)))
    {
      IAM_ERR( "Copying bandwidth data failed\n");
      return -EACCES;
    }
    iam_set_odr_range_bw(client_data, -1, &bw, -1);
    break;
  case IAM20380HT_IO_SODR:
    if (copy_from_user(&value, (int *)arg, sizeof(int)))
    {
      IAM_ERR( "Copying rate data filter ODR failed\n");
      return -EACCES;
    }
    iam_set_odr_range_bw(client_data, -1, NULL, value);
    break;
  case IAM20380HT_IO_GODR:
    iam20380_get_odr(&value, &client_data->device);
    if (copy_to_user((int *)arg, &value, sizeof(int)))
    {
      IAM_ERR( "Copying ODR failed\n");
      return -EFAULT;
    }
    break;
  case IAM20380HT_IO_SRANGE:
    if (copy_from_user(&value, (int *)arg, sizeof(int)))
    {
      IAM_ERR( "Copying range data failed\n");
      return -EACCES;
    }
    iam_set_odr_range_bw(client_data, value, NULL, -1);
    break;
  case IAM20380HT_IO_SPOWERMODE:
    if (copy_from_user(&value, (int *)arg, sizeof(int)))
    {
      IAM_ERR( "Copying power mode value failed\n");
      return -EACCES;
    }
    mutex_lock(&client_data->mutex_op_mode);
    client_data->device.power_mode = value;
    err = iam20380ht_set_power_mode(&client_data->device);
    if (err != IAM20380HT_OK)
      IAM_ERR( "Error on set mode\n");
    mutex_unlock(&client_data->mutex_op_mode);
    break;
  case IAM20380HT_IO_SSOFT_RESET:
    if (iam20380ht_soft_reset(&client_data->device) != IAM20380HT_OK)
    {
      IAM_ERR("Soft reset failed\n");
      return -EFAULT;
    }
    iam_reconfigure(client_data);
    break;
  case IAM20380HT_IO_QUEUEDATAFRAME:
#ifdef USE_WAIT_NEW_DATA
    if (!wait_event_interruptible_timeout(client_data->wait_lock,
        iam_local_queue->frame_count != 0,  HZ))
        { // ожидаем появления новых данных
      IAM_ERR("timeout while waiting new data from sensor!\n", __func__, __LINE__);
      return -ETIME;
    }
    else if (signal_pending(current))
    {
      IAM_ERR("IAM20380HT_IO_QUEUEDATAFRAME interrupt received\n");
      return -ERESTARTSYS;
    }
#endif
    mutex_lock(&client_data->mutex_work);
    if (copy_to_user((void *)arg, iam_local_queue, sizeof(iam20380ht_queue_frames_t)))
    {
      IAM_ERR("Copying data queue failed\n");
      mutex_unlock(&client_data->mutex_work);
      return -EFAULT;
    }
    iam_local_queue->frame_count = 0;
    mutex_unlock(&client_data->mutex_work);
    break;
  case IAM20380HT_IO_STOREENABLE:
    if (copy_from_user(&value, (int *)arg, sizeof(int)))
    {
      IAM_ERR("Copying store enable failed\n");
      return -EACCES;
    }
    iam_store_queue_enable(client_data, value);
    break;
  default:
    return -ENOTTY;
  }
  return IAM20380HT_OK;
}

static const struct file_operations cdev_fops = {
  open:            cdev_open,
  unlocked_ioctl:  cdev_ioctl,
  release:         cdev_release,
};

static int init_char_device(iam_client_data *client_data)
{
  int		ret;
  int		err;
  dev_t	dev;

  if (client_data == NULL)
    return IAM20380HT_E_NULL_PTR;

  ret = alloc_chrdev_region(&client_data->dev_no, 0, 1, IAM20380HT_DEV_NAME);
  if (ret < 0)
  {
    IAM_ERR("Major number allocation is failed\n");
    return ret;
  }

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
  client_data->iam20380ht_class = class_create(THIS_MODULE, IAM20380HT_DEV_NAME);
#else
  client_data->iam20380ht_class = class_create(IAM20380HT_DEV_NAME);
#endif
  if (IS_ERR(client_data->iam20380ht_class))
  {
    err = PTR_ERR(client_data->iam20380ht_class);
    unregister_chrdev_region(client_data->dev_no, 1);
    return IAM20380HT_FAIL;
  }

  if (device_create(client_data->iam20380ht_class, NULL, client_data->dev_no, client_data,
      IAM20380HT_DEV_NAME) == NULL)
  {
    class_destroy(client_data->iam20380ht_class);
    unregister_chrdev_region(client_data->dev_no, 1);
    return IAM20380HT_FAIL;
  }

  client_data->cdev_major = MAJOR(client_data->dev_no);
  dev = MKDEV(client_data->cdev_major, 0);
  cdev_init(&client_data->kernel_cdev, &cdev_fops);
  client_data->kernel_cdev.owner = THIS_MODULE;
  IAM_INFO("The major number for your device is %d\n", client_data->cdev_major);
  ret = cdev_add(&client_data->kernel_cdev, dev,1);
  if (ret < 0)
  {
    IAM_INFO("Unable to allocate cdev\n");
    device_destroy(client_data->iam20380ht_class, client_data->dev_no);
    class_destroy(client_data->iam20380ht_class);
    unregister_chrdev_region(client_data->dev_no, 1);
    return ret;
  }
  return IAM20380HT_OK;
}
/**
 * @}
 */
static int iam_set_odr_range_bw(iam_client_data *client_data,
    arm_range_t range, bw_t *bw, smplrt_div_t smplrt_div)
{
  iam20380ht_cfg  gyro_cfg;
  int             err;

  err = iam20380ht_get_sensor_settings(&gyro_cfg, &client_data->device);
  if (err != IAM20380HT_OK)
    IAM_ERR("iam20380ht_get_sensor_settings status %d", err);

  if (range >= 0)
    gyro_cfg.range = range;

  if (bw != NULL)
  {
    gyro_cfg.bw.mode = bw->mode;
    gyro_cfg.bw.dlpf = bw->dlpf;
  }

  if (smplrt_div >= 0)
  {
    gyro_cfg.smplrt_div = smplrt_div;
    client_data->device.odr = smplrt_div;
  }

  err = iam20380ht_set_sensor_settings(&gyro_cfg, &client_data->device);
  if (err != IAM20380HT_OK)
    IAM_ERR("iam20380ht_set_sensor_settings status %d", err);
  
  return err;
}

static int iam_detect(struct i2c_client *client,
              struct i2c_board_info *info)
{

  int             err;
  iam20380ht_dev  device;

  dev_info(&client->dev, "Detect 0x%02X class=%d",
      client->addr, client->adapter->class);

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
  {
    IAM_ERR("i2c_check_functionality error!");
    return -ENODEV;
  }

  strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);

  return IAM20380HT_OK;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
static int iam_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int iam_probe(struct i2c_client *client)
#endif
{
  int err = 0;
  iam_client_data *client_data = NULL;

  dev_info(&client->dev, "PROBE 0x%02X class=%d",
      client->addr, client->adapter->class );

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
  {
    IAM_ERR("i2c_check_functionality error!");
    err = -EIO;
    goto exit_err_clean;
  }
  else
  {
    IAM_ERR("i2c_check_functionality is ok!");
  }

  client_data = kzalloc(sizeof(iam_client_data), GFP_KERNEL);
  if (client_data == NULL)
  {
    IAM_ERR("no memory available for client_data");
    err = -ENOMEM;
    goto exit_err_clean;
  }

  client_data->device.fifo = kzalloc(sizeof(iam20380ht_fifo_frame), GFP_KERNEL);
  if (client_data->device.fifo == NULL)
  {
    IAM_ERR("no memory available for client_data->device.fifo");
    err = -ENOMEM;
    goto exit_err_clean;
  }

  client_data->device.fifo->data = kzalloc(IAM20380HT_FIFO_RAW_DATA_BUFFER_SIZE,
      GFP_KERNEL);
  if (client_data->device.fifo == NULL)
  {
    IAM_ERR("no memory available for client_data->device.fifo->data");
    err = -ENOMEM;
    goto exit_err_clean;
  }

  client_data->device.fifo->length = IAM20380HT_FIFO_RAW_DATA_BUFFER_SIZE;

  /* Map the delay function pointer with the function responsible for implementing the delay */
  client_data->device.delay_ms = iam_i2c_delay;

  /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x68) & VDD for SECONDARY(0x69)) */
  client_data->device.i2c_client = client;

  /* Select the interface mode as I2C */
  client_data->device.intf = IAM20380HT_I2C_INTF;

  /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
  client_data->device.read = iam_i2c_read;
  client_data->device.write = iam_i2c_write;

  /* check chip id */
  err = iam20380ht_init(&client_data->device);
  if (err == IAM20380HT_OK)
    dev_notice(&client->dev, "IAM20380HT Sensor Device detected\n", SENSOR_NAME);
  else
  {
    IAM_ERR("IAM20380HT Sensor Device not found, chip id mismatch\n");
    err = -ENODEV;
    goto exit_err_clean;
  }

  iam_local_queue = kzalloc(sizeof(iam20380ht_queue_frames_t), GFP_KERNEL);
  if (iam_local_queue == NULL)
  {
    IAM_ERR("no memory available");
    err = -ENOMEM;
    goto exit_err_clean;
  }
  memset(iam_local_queue, 0, sizeof(iam20380ht_queue_frames_t));

  mutex_init(&client_data->mutex_op_mode);
  mutex_init(&client_data->mutex_enable);
  mutex_init(&client_data->mutex_work);
  // mutex_init(&client_data->mutex_chip_id);
#ifdef USE_WAIT_NEW_DATA
  init_waitqueue_head(&client_data->wait_lock);
#endif
  err = iam_input_init(client_data);
  if (err < 0)
  {
    IAM_ERR(" iam_input_init status %d", err);
    goto exit_err_clean;
  }

  /* sysfs node creation */
  err = sysfs_create_group(&client_data->input->dev.kobj, &iam_attribute_group);
  if (err < 0)
  {
    IAM_ERR(" sysfs_create_group status %d", err);
    goto exit_err_sysfs;
  }

  err = init_char_device(client_data);
  if (err < 0)
  {
    IAM_ERR(" init_char_device failed %d", err);
    printk("init char dev failed");
    goto exit_err_sysfs;
  }

  client_data->enable = 0;
  client_data->drv_thread = NULL;

  err = iam_reconfigure(client_data);
  if (err != IAM20380HT_OK)
  {
    IAM_ERR("ERROR reconfigure  IAM20380HT %d", err);
    goto exit_err_sysfs;
  }

  iam_dump_reg(client);
  i2c_set_clientdata(client, client_data);
  dev_notice(&client->dev, "sensor %s probed successfully", SENSOR_NAME);
  dev_dbg(&client->dev,
    "i2c_client: %p client_data: %p i2c_device: %p input: %p",
    client, client_data, &client->dev, client_data->input);

  return IAM20380HT_OK;

exit_err_sysfs:
  sysfs_remove_group(&client_data->input->dev.kobj, &iam_attribute_group);
  iam_input_destroy(client_data);
  release_char_device(client_data);

exit_err_clean:
  if (client_data != NULL)
  {
    if (client_data->device.fifo)
    {
      if (client_data->device.fifo->data)
      {
        kfree(client_data->device.fifo->data);
        client_data->device.fifo->data = NULL;
      }
      kfree(client_data->device.fifo);
      client_data->device.fifo = NULL;
    }
    kfree(client_data);
    client_data = NULL;
  }
  if (iam_local_queue != NULL)
  {
    kfree(iam_local_queue);
    iam_local_queue = NULL;
  }
  IAM_ERR("sensor %s fail to probe", SENSOR_NAME);

  return err;
}

static ssize_t  smile(char *buf)
{
  static const char *code[] = {
    "\34 \4o\14$\4ox\30 \2o\30$\1ox\25 \2o\36$\1o\11 \1o\1$\3 \2$\1 \1o\1$x\5 \1o\1 \1$\1 \2o\10 \1o\44$\1o\7 \2$\1 \2$\1 \2$\1o\1$x\2 \2o\1 \1$\1 \1$\1 \1\"\1$\6 \1o\11$\4 \15$\4 \11$\1o\7 \3$\1o\2$\1o\1$x\2 \1\"\6$\1o\1$\5 \1o\11$\6 \13$\6 \12$\1o\4 \10$x\4 \7$\4 \13$\6 \13$\6 \27$x\4 \27$\4 \15$\4 \16$\2 \3\"\3$x\5 \1\"\3$\4\"\61$\5 \1\"\3$x\6 \3$\3 \1o\62$\5 \1\"\3$\1ox\5 \1o\2$\1\"\3 \63$\7 \3$\1ox\5 \3$\4 \55$\1\"\1 \1\"\6$",
    "\5o\4$\1ox\4 \1o\3$\4o\5$\2 \45$\3 \1o\21$x\4 \10$\1\"\4$\3 \42$\5 \4$\10\"x\3 \4\"\7 \4$\4 \1\"\34$\1\"\6 \1o\3$x\16 \1\"\3$\1o\5 \3\"\22$\1\"\2$\1\"\11 \3$x\20 \3$\1o\12 \1\"\2$\2\"\6$\4\"\13 \1o\3$x\21 \4$\1o\40 \1o\3$\1\"x\22 \1\"\4$\1o\6 \1o\6$\1o\1\"\4$\1o\10 \1o\4$x\24 \1\"\5$\2o\5 \2\"\4$\1o\5$\1o\3 \1o\4$\2\"x\27 \2\"\5$\4o\2 \1\"\3$\1o\11$\3\"x\32 \2\"\7$\2o\1 \12$x\42 \4\"\13$x\46 \14$x\47 \12$\1\"x\50 \1\"\3$\4\"x"
  };
  char  *p;
  int   n;
  int   i;

  sprintf(buf, "%s\n", buf);
  for (i = 0; i < 2; ++i)
  {
    for (p = code[i]; *p != '\0'; ++p)
    {
      if (*p == 'x')
        sprintf(buf, "%s\n", buf);
      else
      {
        for (n = *p++; n > 0; --n)
          sprintf(buf, "%s%c", buf, *p);
      }
    }
  }
  return sprintf(buf, "%s\n", buf);
}

void iam_shutdown(struct i2c_client *client)
{
  int err = IAM20380HT_OK;
  iam_client_data *client_data =
    (iam_client_data *)i2c_get_clientdata(client);

  mutex_lock(&client_data->mutex_op_mode);
  client_data->device.power_mode = IAM20380HT_PM_SLEEP;
  err = iam20380ht_set_power_mode(&client_data->device);
  if (err != IAM20380HT_OK)
    IAM_ERR("Error on set mode");
  mutex_unlock(&client_data->mutex_op_mode);
}

static int release_char_device(iam_client_data *client_data)
{
  cdev_del(&client_data->kernel_cdev);
  if (client_data->iam20380ht_class)
  {
    device_destroy(client_data->iam20380ht_class, client_data->dev_no);
    class_destroy(client_data->iam20380ht_class);
    unregister_chrdev_region(client_data->dev_no, 1);
    client_data->iam20380ht_class = NULL;
  }
  return IAM20380HT_OK;
}

static int iam_remove(struct i2c_client *client)
{
  int err = 0;
  iam_client_data *client_data =
    (iam_client_data *)i2c_get_clientdata(client);

  if (NULL != client_data)
  {
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&client_data->early_suspend_handler);
#endif
    if (client_data->enable && client_data->drv_thread)
      kthread_stop(client_data->drv_thread);
    mutex_lock(&client_data->mutex_enable);
    client_data->drv_thread = NULL;
    client_data->enable = 0;
    mutex_unlock(&client_data->mutex_enable);


    mutex_lock(&client_data->mutex_op_mode);
    client_data->device.power_mode = IAM20380HT_PM_SLEEP;
    err = iam20380ht_set_power_mode(&client_data->device);
    mutex_unlock(&client_data->mutex_op_mode);

    sysfs_remove_group(&client_data->input->dev.kobj, &iam_attribute_group);
    iam_input_destroy(client_data);
    release_char_device(client_data);

    if (client_data != NULL)
    {
      if (client_data->device.fifo)
      {
        if (client_data->device.fifo->data)
        {
          kfree(client_data->device.fifo->data);
          client_data->device.fifo->data = NULL;
        }
        kfree(client_data->device.fifo);
        client_data->device.fifo = NULL;
      }
      mutex_destroy(&client_data->mutex_op_mode);
      mutex_destroy(&client_data->mutex_enable);
      mutex_destroy(&client_data->mutex_work);
      // mutex_destroy(&client_data->mutex_chip_id);
      kfree(client_data);
      client_data = NULL;
    }
    if (iam_local_queue != NULL)
    {
      kfree(iam_local_queue);
      iam_local_queue = NULL;
    }
  }

  return err;
}

static const struct i2c_device_id iam_id[] = {
//	{ SENSOR_NAME, IAM20380HT_SEC_I2C_ADDR },
  { SENSOR_NAME, 0 },
  { }
};

MODULE_DEVICE_TABLE(i2c, iam_id);

static const unsigned short normal_i2c[] = {IAM20380HT_I2C_ADDR2, I2C_CLIENT_END};

static struct i2c_driver iam_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = SENSOR_NAME,
  },
  .class = I2C_CLASS_HWMON,
  .id_table = iam_id,
  .probe = iam_probe,
  .remove = iam_remove,
  .shutdown = iam_shutdown,
  .detect = iam_detect,
  .address_list = &normal_i2c[0],

#ifndef CONFIG_HAS_EARLYSUSPEND
#ifndef NEW_KERNEL
  .suspend = iam_suspend,
  .resume = iam_resume,
#endif
#endif
};

static int __init IAM_init(void)
{
  IAM_INFO("%s: gyroscope input driver init\n", SENSOR_NAME);
  return i2c_add_driver(&iam_driver);
}

static void __exit IAM_exit(void)
{
  IAM_INFO("%s exit\n", SENSOR_NAME);
  i2c_del_driver(&iam_driver);
}

MODULE_AUTHOR("ivan.gorich@gmail.com");
MODULE_DESCRIPTION("IAM20380HT GYROSCOPE SENSOR DRIVER");
MODULE_LICENSE("GPL");

module_init(IAM_init);
module_exit(IAM_exit);
