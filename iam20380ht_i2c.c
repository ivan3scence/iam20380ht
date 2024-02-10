/**
 * @file iam20380ht_i2c.c
 * @author ivan.gorich@gmail.com
 * @brief Driver I2C interface
 * @version 0.1
 * @date 2024-01-27 
 */


#include "iam20380ht_i2c.h"


void iam_i2c_delay(uint32_t msec)
{
  msleep(msec);
}

void  iam_dump_reg(struct i2c_client const *client)
{
  int	i,j,n=-1,buf_idx=0;
  u8	dbg_buf[IAM20380HT_DUMP_LEN];
  u8	dbg_buf_str[(BYTES_PER_LINE * 3 + 4) * (IAM20380HT_DUMP_LEN / BYTES_PER_LINE + 2) + 1] = {0};

  iam_i2c_read(client, 0, dbg_buf, IAM20380HT_DUMP_LEN);
  for (i = -1; i < IAM20380HT_DUMP_LEN / BYTES_PER_LINE; i++)
  {
    for (j = 0; j <= BYTES_PER_LINE; ++j)
    {
      if ((i==-1) && !j)
      {
        sprintf(&dbg_buf_str[buf_idx], "    ");
        buf_idx += 4;
      }
      else if (!j)
      {
        sprintf(&dbg_buf_str[buf_idx], "%02X: ", i);
        buf_idx += 4;
      }
      else if (i==-1)
      {
        sprintf(&dbg_buf_str[buf_idx], "%02X%c", j-1, j != BYTES_PER_LINE ? ' ' : '\n');
        buf_idx += 3;
      }
      else
      {
        sprintf(&dbg_buf_str[buf_idx], "%02X%c", dbg_buf[++n], j != BYTES_PER_LINE ? ' ' : '\n');
        buf_idx += 3;
      }
    }
  }
  dev_info(&client->dev, "\n%s\n", dbg_buf_str);
}

int8_t	iam_i2c_read(struct i2c_client *client, uint8_t reg_addr,
    uint8_t *data, uint16_t len)
{
#ifndef IAM_USE_BASIC_I2C_FUNC
  s32	dummy;
  if (client == NULL)
    return IAM20380HT_E_INVALID_INPUT;

  while (len-- != 0)
  {
#ifdef IAM_SMBUS
    dummy = i2c_smbus_read_byte_data(client, reg_addr);
    if (dummy < 0)
    {
      IAM_ERR("i2c bus read error");
      return IAM20380HT_E_COM_FAIL;
    }
    *data = (u8)(dummy & 0xff);
#else
    dummy = i2c_master_send(client, (char *)&reg_addr, 1);
    if (dummy < 0)
    {
      IAM_ERR("i2c bus send error");
      return IAM20380HT_E_COM_FAIL;
    }
    dummy = i2c_master_recv(client, (char *)data, 1);
    if (dummy < 0)
    {
      IAM_ERR("i2c bus receive error");
      return IAM20380HT_E_COM_FAIL;
    }
#endif
    ++reg_addr;
    ++data;
  }
  return IAM20380HT_OK;
#else
  int	retry;

  struct i2c_msg msg[] = {
    {
      .addr =		client->addr,
      .flags =	0,
      .len =		1,
      .buf =		&reg_addr,
    },
    {
      .addr =		client->addr,
      .flags =	I2C_M_RD,
      .len =		len,
      .buf =		data,
    },
  };
  for (retry = 0; retry < IAM_MAX_RETRY_I2C_XFER; retry++)
  {
    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
      break;
    else
      mdelay(IAM_I2C_WRITE_DELAY_TIME);
  }
  if (retry >= IAM_MAX_RETRY_I2C_XFER)
  {
    IAM_ERR("I2C xfer error");
    return -EIO;
  }
  return IAM20380HT_OK;
#endif
}

int8_t	iam_i2c_write(struct i2c_client *client, uint8_t reg_addr,
    uint8_t const *data, uint16_t len)
{
#ifndef IAM_USE_BASIC_I2C_FUNC
  s32	dummy;
#ifndef IAM_SMBUS
  u8	buffer[2];
#endif
  if (client == NULL)
    return -EPERM;

  while (len-- != 0)
  {
#ifdef IAM_SMBUS
    dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
    buffer[0] = reg_addr;
    buffer[1] = *data;
    dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
    ++reg_addr;
    ++data;
    if (dummy < 0)
    {
      IAM_ERR("error writing i2c bus");
      return -EPERM;
    }
  }
  return IAM20380HT_OK;
#else
  u8				buffer[2];
  int				retry;
  struct i2c_msg	msg[] = {
    {
      .addr = client->addr,
      .flags = 0,
      .len = 2,
      .buf = buffer,
    },
  };

  while (0 != len--)
  {
    buffer[0] = reg_addr;
    buffer[1] = *data;
    for (retry = 0; retry < IAM_MAX_RETRY_I2C_XFER; retry++)
    {
      if (i2c_transfer(client->adapter, msg,
            ARRAY_SIZE(msg)) > 0)
      {
        break;
      }
      else
      {
        mdelay(IAM_I2C_WRITE_DELAY_TIME);
      }
    }
    if (retry >= IAM_MAX_RETRY_I2C_XFER)
    {
      IAM_ERR("I2C xfer error");
      return -EIO;
    }
    ++reg_addr;
    ++data;
  }
  return IAM20380HT_OK;
#endif
}
