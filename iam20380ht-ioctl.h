/**
 * @file iam20380ht-ioctl.h
 * @author ivan.gorich@gmail.com
 * @brief IAM20380HT driver ioctl API
 * @version 0.1
 * @date 2024-01-27
 */


#ifndef IAM20380HT_IOCTL_H_
#define IAM20380HT_IOCTL_H_

#include "iam20380ht_defs.h"


/*!
 * @defgroup group7 Driver ioctl functionality.
 * @brief Internal ioctl macros.
 * @{
 */
/** @{ \name IOCTL-options.*/
#define IAM20380HT_IOC_MAGIC 'g'
#define IAM20380HT_IO_ENABLE        _IOW(IAM20380HT_IOC_MAGIC, 0, int)	///< Power on sensor.
#define IAM20380HT_IO_GDATA         _IOR(IAM20380HT_IOC_MAGIC, 1, iam20380ht_sensor_data)	///< Get sensor data.
#define IAM20380HT_IO_SBANDTWITH    _IOW(IAM20380HT_IOC_MAGIC, 2, int)	///< Set params for sampling data.
#define IAM20380HT_IO_SPOWERMODE    _IOW(IAM20380HT_IOC_MAGIC, 3, int)	///< Select power-mode.
#define IAM20380HT_IO_SODR          _IOW(IAM20380HT_IOC_MAGIC, 4, int)	///< Set output data rate.
#define IAM20380HT_IO_SRANGE        _IOW(IAM20380HT_IOC_MAGIC, 5, int)	///< Set range of the gyro in degrees per second.
#define IAM20380HT_IO_SFAST_OFFSET  _IOW(IAM20380HT_IOC_MAGIC, 6, int)	///< Set fast offset compensation.
#define IAM20380HT_IO_SSOFT_RESET   _IOW(IAM20380HT_IOC_MAGIC, 7, int)	///< Set soft reset.
#define IAM20380HT_IO_GODR          _IOW(IAM20380HT_IOC_MAGIC, 8, int)	///< Get output data rate.
#ifdef TODO
#define IAM20380HT_IO_GCHIP_ID      _IOW(IAM20380HT_IOC_MAGIC, 11, int)	///< Get chip id.
#define IAM20380HT_IO_SELF_TEST     _IOW(IAM20380HT_IOC_MAGIC, 12, int)	///< Start gyro self test.
#define IAM20380HT_IO_SINT          _IOW(IAM20380HT_IOC_MAGIC, 13, int)	///< Set interrupt mode.
#define IAM20380HT_IO_FIFO_RST      _IOW(IAM20380HT_IOC_MAGIC, 14, int)	///< FIFO reset.
#endif
/**@}*/

#ifdef IAM_USE_FIFO
#define IAM20380HT_QUEUE_MAX_FRAME_COUNT	(250)
typedef struct
{
  iam20380ht_sensor_data  data[IAM20380HT_QUEUE_MAX_FRAME_COUNT];
  uint8_t                 frame_count;
} iam20380ht_queue_frames_t;

#define IAM20380HT_IO_QUEUEDATAFRAME  _IOR(IAM20380HT_IOC_MAGIC, 9, \
    iam20380ht_queue_frames_t)
#define IAM20380HT_IO_STOREENABLE     _IOW(IAM20380HT_IOC_MAGIC, 10, int)
#define IAM20380HT_IOC_MAXNR          10
#else
#define IAM20380HT_IOC_MAXNR          8
#endif

#if IAM20380HT_QUEUE_MAX_FRAME_COUNT > 255
#   error IAM20380HT_QUEUE_MAX_FRAME_COUNT  can not be more that 255 because it is used in 8bit fields.
#endif
/**@}*/
#endif /* IAM20380HT_IOCTL_H_ */
