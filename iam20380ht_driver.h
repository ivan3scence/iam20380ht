/**
 * @file iam20380ht_driver.h
 * @author ivan.gorich@gmail.com
 * @brief Header file for main driver structure.
 * @version 0.1
 * @date 2024-02-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef IAM20380HT_DRIVER_H
#define IAM20380HT_DRIVER_H
#define DEBUG
#define IAM_DEBUG

#ifndef CONFIG_PRINTK
#define CONFIG_PRINTK
#endif

#ifndef IAM_USE_FIFO
#define IAM_USE_FIFO
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kthread.h>
// #ifdef NEW_KERNEL
// #include <uapi/linux/sched/types.h>
// #endif
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/completion.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "iam20380ht.h"
#include "iam20380ht_i2c.h"
#include "iam20380ht-ioctl.h"


/* Buffer size allocated to store raw FIFO data */
#define IAM20380HT_FIFO_RAW_DATA_BUFFER_SIZE    4096

/*!
 * @defgroup group4 Type definitions.
 * @brief Project enums and structers.
 * @{
 */
typedef struct
{
  iam20380ht_dev        device;
  struct input_dev      *input;
  struct task_struct    *drv_thread;
  /**
   * @brief Is enabled kworker (iam_work_func). */
  uint8_t               enable:1;
  uint32_t              client_count;
  /**
   * @brief Mutex read/write client_data->device.power_mode */
  struct mutex          mutex_op_mode;
  /**
   * @brief Mutex read/write client_data->enable */
  struct mutex          mutex_enable;
  /**
   * @brief Mutex read/write iam_local_queue */
  struct mutex          mutex_work;
  /**
   * @brief Mutex read/write client_data->device.chip_id */
  // struct mutex          mutex_chip_id;
#ifdef USE_WAIT_NEW_DATA
  wait_queue_head_t		wait_lock;
#endif
  /*cdev declaration */
  int                   cdev_major;
  struct cdev           kernel_cdev;
  struct class          *iam20380ht_class;
  dev_t                 dev_no;
} iam_client_data;
/**@}*/
/*!
 * @defgroup group5 Driver static functions.
 * @brief Driver internal funcs.
 * @{
 */
static int  release_char_device(iam_client_data *client_data);
static int  iam_pre_suspend(struct i2c_client *client);
static int  iam_post_resume(struct i2c_client *client);
/*!
 *  @brief Set params for sampling data.
 *	If the params odr, range is negative they are ignored
 *  If bw == NULL, it is ignored.
 *
 * @param[in] smplrt_div        : Sample rate divider for corresponding ODR (Output data rate).
 * @param[in] range            	: Angular rate measurement range in degree per second.
 * @param[in] bw            		: Bandwidth mode and dlpf.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int  iam_set_odr_range_bw(iam_client_data *client_data,
    arm_range_t range, bw_t *bw, smplrt_div_t smplrt_div);
static ssize_t  smile(char *buf);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void iam_early_suspend(struct early_suspend *handler);
static void iam_late_resume(struct early_suspend *handler);
#endif
/**@}*/
#endif // IAM20380HT_DRIVER_H
