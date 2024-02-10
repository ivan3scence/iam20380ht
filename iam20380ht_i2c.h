/**
 * @file iam20380ht_i2c.h
 * @author ivan.gorich@gmail.com
 * @brief Driver I2C interface
 * @version 0.1
 * @date 2024-01-27
 */


#ifndef IAM20380HT_I2C_H_
#define IAM20380HT_I2C_H_

#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "iam20380ht_defs.h"

/*!
 * @defgroup group6 Driver I2C functionality.
 * @brief Internal I2C funsc and macros.
 * @{
 */
/**
 * @brief Write delay in miliseconds.
 */
#define IAM_I2C_WRITE_DELAY_TIME  1
/**
 * @brief Number of maximum retries on I2C transfering.
 */
#define IAM_MAX_RETRY_I2C_XFER    (100)
/**
 * @brief Number of bytes written per line on dump-function.
 */
#define BYTES_PER_LINE            (16)

/**
 * @brief Delay func in miliseconds.
 * 
 * @param msec 
 */
void  iam_i2c_delay(uint32_t msec);
/**
 * @brief Print all values of registers.
 * 
 * @param client 
 */
void  iam_dump_reg(struct i2c_client const *client);
/**
 * @brief Function to read iam register.
 * 
 * @param client 
 * @param reg_addr 
 * @param data 
 * @param len 
 * @retval zero -> Success / nonzero value -> Error
 */
int8_t  iam_i2c_read(struct i2c_client *client, uint8_t reg_addr, uint8_t *data, uint16_t len);
/**
 * @brief Function to write iam register.
 * 
 * @param client 
 * @param reg_addr 
 * @param data 
 * @param len 
 * @retval zero -> Success / nonzero value -> Error
 */
int8_t  iam_i2c_write(struct i2c_client *client, uint8_t reg_addr,
    uint8_t const *data, uint16_t len);
/**@}*/
#endif /* IAM20380HT_I2C_H_ */
