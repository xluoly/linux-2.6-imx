/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

//#define DEBUG 1

#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/tavarua.h>
#include <asm/unaligned.h>

#include "si4730-i2c.h"


#define DRIVER_VERSION	"v0.01"
#define RADIO_VERSION	KERNEL_VERSION(0, 0, 1)

#define DRIVER_NAME "si4730_i2c"
#define DRIVER_DESC "I2C driver for Si4730/31 FM Radio Receiver"

/* module parameters */
static int debug = 1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug level (0 - 2)");


/**************************************************************************
 * Module Parameters
 **************************************************************************/
/* Radio Nr */
static int radio_nr = -1;

/* Spacing (kHz) */
/* 0: 200 kHz (USA, Australia) */
/* 1: 100 kHz (Europe, Japan) */
/* 2:  50 kHz */
static unsigned short space = 2;
module_param(space, ushort, 0444);
MODULE_PARM_DESC(space, "Spacing: 0=200kHz 1=100kHz *2=50kHz*");

/* Bottom of Band (MHz) */
/* 0: 87.5 - 108 MHz (USA, Europe)*/
/* 1: 76   - 108 MHz (Japan wide band) */
/* 2: 76   -  90 MHz (Japan) */
static unsigned short band = 1;
module_param(band, ushort, 0444);
MODULE_PARM_DESC(band, "Band: 0=87.5..108MHz *1=76..108MHz* 2=76..90MHz");

/* De-emphasis */
/* 0: 75 us (USA) */
/* 1: 50 us (Europe, Australia, Japan) */
static unsigned short de = 1;
module_param(de, ushort, 0444);
MODULE_PARM_DESC(de, "De-emphasis: 0=75us *1=50us*");

/* Tune timeout */
static unsigned int tune_timeout = 3000;
module_param(tune_timeout, uint, 0644);
MODULE_PARM_DESC(tune_timeout, "Tune timeout: *3000*");

/* Seek timeout */
static unsigned int seek_timeout = 5000;
module_param(seek_timeout, uint, 0644);
MODULE_PARM_DESC(seek_timeout, "Seek timeout: *5000*");

/* RDS buffer blocks */
static unsigned int rds_buf = 100;
module_param(rds_buf, uint, 0444);
MODULE_PARM_DESC(rds_buf, "RDS buffer entries: *100*");

/* RDS maximum block errors */
static unsigned short max_rds_errors = 1;
/* 0 means   0  errors requiring correction */
/* 1 means 1-2  errors requiring correction (used by original USBRadio.exe) */
/* 2 means 3-5  errors requiring correction */
/* 3 means   6+ errors or errors in checkword, correction not possible */
module_param(max_rds_errors, ushort, 0644);
MODULE_PARM_DESC(max_rds_errors, "RDS maximum block errors: *1*");

/* RDS poll frequency */
static unsigned int rds_poll_time = 40;
/* 40 is used by the original USBRadio.exe */
/* 50 is used by radio-cadet */
/* 75 should be okay */
/* 80 is the usual RDS receive interval */
module_param(rds_poll_time, uint, 0644);
MODULE_PARM_DESC(rds_poll_time, "RDS poll time (ms): *40*");


#define DEFAULT_RDS_PI			0x00
#define DEFAULT_RDS_PTY			0x00
#define DEFAULT_RDS_PS_NAME		""
#define DEFAULT_RDS_RADIO_TEXT		DEFAULT_RDS_PS_NAME
#define DEFAULT_RDS_DEVIATION		0x00C8
#define DEFAULT_RDS_PS_REPEAT_COUNT	0x0003
#define DEFAULT_LIMITER_RTIME		0x1392
#define DEFAULT_LIMITER_DEV		0x102CA
#define DEFAULT_PILOT_FREQUENCY	0x4A38
#define DEFAULT_PILOT_DEVIATION		0x1A5E
#define DEFAULT_ACOMP_ATIME		0x0000
#define DEFAULT_ACOMP_RTIME		0xF4240L
#define DEFAULT_ACOMP_GAIN		0x0F
#define DEFAULT_ACOMP_THRESHOLD	(-0x28)
#define DEFAULT_MUTE			0x01
#define DEFAULT_POWER_LEVEL		88
#define DEFAULT_FREQUENCY		8800
#define DEFAULT_PREEMPHASIS		FMPE_EU
#define DEFAULT_TUNE_RNL		0xFF

#define to_si4730_device(sd)	container_of(sd, struct si4730_device, sd)

/* frequency domain transformation (using times 10 to avoid floats) */
#define FREQ_RANGE_LOW			8750
#define FREQ_RANGE_HIGH			10800

#define MAX_ARGS 7

#define RDS_BLOCK			8
#define RDS_BLOCK_CLEAR			0x03
#define RDS_BLOCK_LOAD			0x04
#define RDS_RADIOTEXT_2A		0x20
#define RDS_RADIOTEXT_BLK_SIZE		4
#define RDS_RADIOTEXT_INDEX_MAX		0x0F
#define RDS_CARRIAGE_RETURN		0x0D

#define rds_ps_nblocks(len)	((len / RDS_BLOCK) + (len % RDS_BLOCK ? 1 : 0))

#define get_status_bit(p, b, m)	(((p) & (m)) >> (b))
#define set_bits(p, v, b, m)	(((p) & ~(m)) | ((v) << (b)))

#define ATTACK_TIME_UNIT	500

#define POWER_OFF			0x00
#define POWER_ON			0x01

#define msb(x)                  ((u8)((u16) x >> 8))
#define lsb(x)                  ((u8)((u16) x &  0x00FF))
#define compose_u16(msb, lsb)	(((u16)msb << 8) | lsb)
#define compose_u32(msb, lsb)   (((u32)msb << 16) | lsb)
#define check_command_failed(status)  \
    (!(status & SI4730_CTS) || (status & SI4730_ERR))

/* mute definition */
#define set_mute(p)	((p & 1) | ((p & 1) << 1));
#define get_mute(p)	(p & 0x01)

#if 0 //def DEBUG
#define DBG_BUFFER(device, message, buffer, size)			\
	{								\
		int i;							\
		char str[(size)*5];					\
		for (i = 0; i < size; i++)				\
			sprintf(str + i * 5, " 0x%02x", buffer[i]);	\
		dev_dbg(device, "%s:%s\n", message, str);	\
	}
#else
#define DBG_BUFFER(device, message, buffer, size)
#endif

struct tune_status {
    int band_limit;
    int afc;
    int valid_channel;
    int frequency;
    int rssi;
    int snr;
    int ant_cap;
};

struct rds_block {
    u8 error;
    u16 rds;
};

/*
 * si4730_send_command - sends a command to si4713 and waits its response
 * @sdev: si4713_device structure for the device we are communicating
 * @command: command id
 * @args: command arguments we are sending (up to 7)
 * @argn: actual size of @args
 * @response: buffer to place the expected response from the device (up to 15)
 * @respn: actual size of @response
 * @usecs: amount of time to wait before reading the response (in usecs)
 */
static int si4730_send_command(struct si4730_device *sdev, const u8 command,
				const u8 args[], const int argn,
				u8 response[], const int respn, const int usecs)
{
	struct i2c_client *client = sdev->client;
	u8 data1[MAX_ARGS + 1];
	int err;
	int i;
	if (!client->adapter)
		return -ENODEV;

	/* First send the command and its arguments */
	data1[0] = command;
	memcpy(data1 + 1, args, argn);

    if (command != SI4730_CMD_GET_INT_STATUS) {
        DBG_BUFFER(&client->dev, "Parameters", data1, argn + 1);
    }

	err = i2c_master_send(client, data1, argn + 1);
	if (err != argn + 1) {
		dev_err(&client->dev, "<1>Error while sending command 0x%02x\n",
			command);
		return (err > 0) ? -EIO : err;
	}

	/* Wait response from interrupt */
    /*
	if (!wait_for_completion_timeout(&sdev->work,
				usecs_to_jiffies(usecs) + 1))
		v4l2_warn(&sdev->sd,
				"(%s) Device took too much time to answer.\n",
				__func__);

    */

	/* Then get the response */
	i = 0;
	err = i2c_master_recv(client, response, respn);
	while (i++ < usecs && ((response[0] & 0x80) != 0x80)) {
		err = i2c_master_recv(client, response, respn);
		udelay(1);
	}


	if (err != respn) {
		dev_err(&client->dev,
			"<2>Error while reading response for command 0x%02x\n",
			command);
		return (err > 0) ? -EIO : err;
	}

    if (command != SI4730_CMD_GET_INT_STATUS) {
        DBG_BUFFER(&client->dev, "Response", response, respn);
    }

	if (check_command_failed(response[0]))
    {
		dev_err(&client->dev, "<3>Error while sending command 0x%02x\n",
			command);
		return -EBUSY;
    }

	return 0;
}


/*
 * si4730_write_property - modifies a si4730 property
 * @sdev: si4730_device structure for the device we are communicating
 * @prop: property identification number
 * @val: new value for that property
 */
static int si4730_write_property(struct si4730_device *sdev, u16 prop, u16 val)
{
	int rval;
	u8 resp[SI4730_SET_PROP_NRESP];
	/*
	*	.First byte = 0
	*	.Second byte = property's MSB
	*	.Third byte = property's LSB
	*	.Fourth byte = value's MSB
	*	.Fifth byte = value's LSB
	*/
	const u8 args[SI4730_SET_PROP_NARGS] = {
		0x00,
		msb(prop),
		lsb(prop),
		msb(val),
		lsb(val),
	};

	rval = si4730_send_command(sdev, SI4730_CMD_SET_PROPERTY,
					args, ARRAY_SIZE(args),
					resp, ARRAY_SIZE(resp),
					DEFAULT_TIMEOUT);

	if (rval < 0)
		return rval;

	/*
	 * As there is no command response for SET_PROPERTY,
	 * wait Tcomp time to finish before proceed, in order
	 * to have property properly set.
	 */
	msleep(TIMEOUT_SET_PROPERTY);

	return rval;
}

static int si4730_read_property(struct si4730_device *sdev, u16 prop)
{
	int rval;
	u8 resp[SI4730_GET_PROP_NRESP];
	/*
	*	.First byte = 0
	*	.Second byte = value's MSB
	*	.Third byte = value's LSB
	*/
	const u8 args[SI4730_GET_PROP_NARGS] = {
		0x00,
		msb(prop),
		lsb(prop),
	};

	rval = si4730_send_command(sdev, SI4730_CMD_GET_PROPERTY,
					args, ARRAY_SIZE(args),
					resp, ARRAY_SIZE(resp),
					DEFAULT_TIMEOUT);

	if (rval < 0)
		return rval;

	return (int)compose_u16(resp[2], resp[3]);
}
/*
 * si4730_powerup - Powers the device up
 * @sdev: si4713_device structure for the device we are communicating
 */
static int si4730_powerup(struct si4730_device *sdev)
{
	int err;
	u8 resp[SI4730_PWUP_NRESP];
	const u8 args[SI4730_PWUP_NARGS] = {
		SI4730_PWUP_ARG1,
		SI4730_PWUP_ARG2,
	};


	if (sdev->power_state)
		return 0;
	err = si4730_send_command(sdev, SI4730_CMD_POWER_UP,
					args, ARRAY_SIZE(args),
					resp, ARRAY_SIZE(resp),
					TIMEOUT_POWER_UP);

	if (!err) {
		dev_dbg(&sdev->client->dev, "Powerup response: 0x%02x\n",
				resp[0]);
		dev_dbg(&sdev->client->dev, "Device in power up mode\n");
		sdev->power_state = POWER_ON;

	} else {
	/*	if (gpio_is_valid(sdev->gpio_reset))
			gpio_set_value(sdev->gpio_reset, 0);
		err = regulator_bulk_disable(ARRAY_SIZE(sdev->supplies),
						     sdev->supplies);
	*/	if (err)
			dev_err(&sdev->client->dev,
				 "Failed to disable supplies: %d\n", err);
	}

	return err;
}

/*
 * si4730_powerdown - Powers the device down
 * @sdev: si4713_device structure for the device we are communicating
 */
static int si4730_powerdown(struct si4730_device *sdev)
{
	int err;
	u8 resp[SI4730_PWDN_NRESP];


	err = si4730_send_command(sdev, SI4730_CMD_POWER_DOWN,
					NULL, 0,
					resp, ARRAY_SIZE(resp),
					DEFAULT_TIMEOUT);

	if (!err) {
		dev_dbg(&sdev->client->dev, "Power down response: 0x%02x\n",
				resp[0]);
		dev_dbg(&sdev->client->dev, "Device in reset mode\n");
		if (err)
			dev_err(&sdev->client->dev,
				 "Failed to disable supplies: %d\n", err);
		sdev->power_state = POWER_OFF;
	}

	return err;
}

/*
 * si4730_checkrev - Checks if we are treating a device with the correct rev.
 * @sdev: si4713_device structure for the device we are communicating
 */
static int si4730_checkrev(struct si4730_device *sdev)
{
	struct i2c_client *client = sdev->client;
	int rval;
	u8 resp[SI4730_GETREV_NRESP];

	mutex_lock(&sdev->mutex);

	rval = si4730_send_command(sdev, SI4730_CMD_GET_REV,
					NULL, 0,
					resp, ARRAY_SIZE(resp),
					DEFAULT_TIMEOUT);

	if (rval < 0)
		goto unlock;

    dev_info(&client->dev, "Part Number: %02x\n", resp[1]);
    dev_info(&client->dev, "Firmware Major: %c, Minor: %c\n", resp[2], resp[3]);
    dev_info(&client->dev, "Patch ID: %04x\n", compose_u16(resp[4], resp[5]));
    dev_info(&client->dev, "Component Major: %c, Minor: %c\n", resp[6], resp[7]);
    dev_info(&client->dev, "Chip Revision: %c\n", resp[8]);

	if (resp[1] == SI4730_PRODUCT_NUMBER) {
		dev_info(&client->dev, "chip found @ 0x%02x (%s)\n",
				client->addr << 1, client->adapter->name);
	} else {
		dev_err(&client->dev, "Invalid product number: 0x%x\n", resp[1]);
		rval = -EINVAL;
	}

unlock:
	mutex_unlock(&sdev->mutex);
	return rval;
}

/*
 * si4730_wait_stc - Waits STC interrupt and clears status bits. Usefull
 *		     for TX_TUNE_POWER, TX_TUNE_FREQ and TX_TUNE_MEAS
 * @sdev: si4713_device structure for the device we are communicating
 * @usecs: timeout to wait for STC interrupt signal
 */
static int si4730_wait_stc(struct si4730_device *sdev, const int usecs)
{
	int err;
	u8 resp[SI4730_GET_STATUS_NRESP];

    unsigned long timeout;
    const unsigned long sleep_time = 200;
    int cnt;

	/* Wait response from STC interrupt */
    /*
	if (!wait_for_completion_timeout(&sdev->work,
			usecs_to_jiffies(usecs) + 1))
		dev_warn(&sdev->client->dev,
			"%s: device took too much time to answer (%d usec).\n",
				__func__, usecs);
	*/
    cnt = seek_timeout / sleep_time;
	do {
        timeout = jiffies + msecs_to_jiffies(sleep_time);
    	while (time_before(jiffies, timeout)) {
            schedule();
        }
		/* Clear status bits */
	    err = si4730_send_command(sdev, SI4730_CMD_GET_INT_STATUS,
					NULL, 0,
					resp, ARRAY_SIZE(resp),
					DEFAULT_TIMEOUT);

    	if (err < 0)
    		goto exit;

        --cnt;
	} while (((resp[0] & SI4730_STC_INT) == 0) && (cnt > 0));
    
    if ((err == 0) && (cnt == 0))
	    err = -EAGAIN;

exit:
	return err;
}

static int si4730_get_rds_int(struct si4730_device *sdev)
{
	int err;
	u8 resp[SI4730_GET_STATUS_NRESP];

    err = si4730_send_command(sdev, SI4730_CMD_GET_INT_STATUS,
                NULL, 0,
                resp, ARRAY_SIZE(resp),
                DEFAULT_TIMEOUT);

    if (err < 0)
        goto exit;

    return resp[0] & (0x01 << 2);

exit:
    return err;
}

/*
 * si4730_rx_tune_freq - Sets the state of the RF carrier and sets the tuning
 *			frequency between 76 and 108 MHz in 10 kHz units and
 *			steps of 50 kHz.
 * @sdev: si4713_device structure for the device we are communicating
 * @frequency: desired frequency (76 - 108 MHz, unit 10 KHz, step 50 kHz)
 */
static int si4730_rx_tune_freq(struct si4730_device *sdev, u16 frequency)
{
	int err;
	u8 val[SI4730_TXFREQ_NRESP];
	/*
	*	.First byte = 0
	*	.Second byte = frequency's MSB
	*	.Third byte = frequency's LSB
	 */
	const u8 args[SI4730_TXFREQ_NARGS] = {
		0x00,
		msb(frequency),
		lsb(frequency),
		0x00,
	};

	err = si4730_send_command(sdev, SI4730_CMD_TUNE_FREQ,
				  args, ARRAY_SIZE(args), val,
				  ARRAY_SIZE(val), DEFAULT_TIMEOUT);

	if (err < 0)
		return err;

	return si4730_wait_stc(sdev, TIMEOUT_RX_TUNE);
}

/*
 * si4730_set_seek - set seek
 */
static int si4730_set_seek(struct si4730_device *radio,
		unsigned int wrap_around, unsigned int seek_upward)
{
	int err;
	u8 val[SI4730_FM_SEEK_NRESP];
	/*
	*	.First byte = 0
	*	.Second byte = frequency's MSB
	*	.Third byte = frequency's LSB
	 */
	u8 args[SI4730_FM_SEEK_NARGS] = {
		SI4730_FM_SEEK_WRAP,
	};

    if (seek_upward) {
        args[0] |= SI4730_FM_SEEK_UP;
    }

    dev_dbg(&radio->client->dev, "%s direction: %d\n", __func__, seek_upward);
	err = si4730_send_command(radio, SI4730_CMD_FM_SEEK_START,
				  args, ARRAY_SIZE(args), val,
				  ARRAY_SIZE(val), DEFAULT_TIMEOUT);

	if (err < 0)
		return err;

	return si4730_wait_stc(radio, TIMEOUT_RX_TUNE);
}

static int si4730_cancel_seek(struct si4730_device *sdev)
{
	int err;
	u8 val[SI4730_FM_TUNE_STATUS_NRESP];
	/*
	*	.First byte = intack bit
	 */
	const u8 args[SI4730_FM_TUNE_STATUS_NARGS] = {
		0x03,
	};

	err = si4730_send_command(sdev, SI4730_CMD_FM_TUNE_STATUS,
				  args, ARRAY_SIZE(args), val,
				  ARRAY_SIZE(val), DEFAULT_TIMEOUT);

	return err;
}

/*
 * si4713_tx_tune_measure - Enters receive mode and measures the received noise
 *			level in units of dBuV on the selected frequency.
 *			The Frequency must be between 76 and 108 MHz in 10 kHz
 *			units and steps of 50 kHz. The command also sets the
 *			antenna	tuning capacitance. A value of 0 means
 *			autotuning, and a value of 1 to 191 indicates manual
 *			override.
 * @sdev: si4713_device structure for the device we are communicating
 * @frequency: desired frequency (76 - 108 MHz, unit 10 KHz, step 50 kHz)
 * @antcap: value of antenna tuning capacitor (0 - 191)
 */
static int si4713_tx_tune_measure(struct si4730_device *sdev, u16 frequency,
					u8 antcap)
{
	int err;
	u8 val[SI4713_TXMEA_NRESP];
	/*
	*	.First byte = 0
	*	.Second byte = frequency's MSB
	*	.Third byte = frequency's LSB
	*	.Fourth byte = antcap
	 */
	const u8 args[SI4713_TXMEA_NARGS] = {
		0x00,
		msb(frequency),
		lsb(frequency),
		antcap,
	};

	sdev->tune_rnl = DEFAULT_TUNE_RNL;

	if (antcap > SI4713_MAX_ANTCAP)
		return -EDOM;

	err = si4730_send_command(sdev, SI4713_CMD_TX_TUNE_MEASURE,
				  args, ARRAY_SIZE(args), val,
				  ARRAY_SIZE(val), DEFAULT_TIMEOUT);

	if (err < 0)
		return err;

	dev_dbg(&sdev->client->dev,
			"%s: frequency=0x%02x antcap=0x%02x status=0x%02x\n",
			__func__, frequency, antcap, val[0]);

	return si4730_wait_stc(sdev, TIMEOUT_RX_TUNE);
}

static int si4730_tune_status(struct si4730_device *sdev,
        struct tune_status *status)
{
	int err;
	u8 val[SI4730_FM_TUNE_STATUS_NRESP];
	/*
	*	.First byte = intack bit
	 */
	const u8 args[SI4730_FM_TUNE_STATUS_NARGS] = {
		0x01,
	};

	err = si4730_send_command(sdev, SI4730_CMD_FM_TUNE_STATUS,
				  args, ARRAY_SIZE(args), val,
				  ARRAY_SIZE(val), DEFAULT_TIMEOUT);

    status->band_limit = val[1] >> 7;
    status->afc= (val[1] >> 1) & 0x01;
    status->valid_channel = val[1] & 0x01;
	status->frequency = compose_u16(val[2], val[3]);
    status->rssi = val[4];
    status->snr = val[5];
    status->ant_cap = val[7];
	return err;
}

static int si4730_fm_rsq_status(struct si4730_device *sdev,
					u16 *frequency)
{
	int err;
	u8 val[SI4730_FM_RSQ_STATUS_NRESP];
	/*
	*	.First byte = intack bit
	 */
	const u8 args[SI4730_FM_RSQ_STATUS_NARGS] = {
		0x01,
	};

	err = si4730_send_command(sdev, SI4730_CMD_FM_RSQ_STATUS,
				  args, ARRAY_SIZE(args), val,
				  ARRAY_SIZE(val), DEFAULT_TIMEOUT);

	*frequency = compose_u16(val[1], val[2]);
	return err;
}


static int si4730_set_power_state(struct si4730_device *sdev, u8 value)
{
	int rval;

	mutex_lock(&sdev->mutex);

	if (value)
		rval = si4730_powerup(sdev);
	else
		rval = si4730_powerdown(sdev);

	mutex_unlock(&sdev->mutex);
	return rval;
}


/*
 * si4730_initialize - Sets the device up with default configuration.
 * @sdev: si4730_device structure for the device we are communicating
 */
static int si4730_initialize(struct si4730_device *sdev)
{
	int rval;

	rval = si4730_set_power_state(sdev, POWER_OFF);
	if (rval < 0)
		goto exit;

	rval = si4730_set_power_state(sdev, POWER_ON);
	if (rval < 0)
		goto exit;

	rval = si4730_checkrev(sdev);
	if (rval < 0)
		goto exit;
	rval = si4730_set_power_state(sdev, POWER_OFF);
	if (rval < 0)
		goto exit;

exit:
	return rval;
}

static int si4730_setup(struct si4730_device *sdev)
{
	int rval;
	rval = si4730_rx_tune_freq(sdev, 9420);
	if (rval < 0)
		goto exit;

    /*
	rval = si4713_enable_digitalout(sdev);
	if (rval < 0)
		goto exit;

	rval = si4713_get_digitalout(sdev, &temp);
	if (rval < 0)
		goto exit;
        */

exit:
	return rval;
}

/*
 * si4730_rds_on - switch on rds reception
 */
static int si4730_rds_on(struct si4730_device *radio)
{
    int retval = -1;

    /* sysconfig 1 */
    mutex_lock(&radio->mutex);
    /* RDS_INT_SOURCE */
    retval = si4730_write_property(radio, 0x1500, 0x0001);
    if (retval < 0)
        goto exit;

    /* RDS_INT_FIFO_COUNT */
    retval = si4730_write_property(radio, 0x1501, 0x0001);
    if (retval < 0)
        goto exit;

    /* RDS_CONFIG */
    retval = si4730_write_property(radio, 0x1502, 0xaa01);
    if (retval >= 0)
        radio->rds_on = 1;

exit:
    mutex_unlock(&radio->mutex);
    return retval;
}

static int si4730_rds_status(struct si4730_device *sdev, struct rds_block rds[])
{
	int err;
	u8 resp[SI4730_FM_RDS_STATUS_NRESP];
	const u8 args[SI4730_FM_RDS_STATUS_NARGS] = {
		0x01,
	};

	err = si4730_send_command(sdev, SI4730_CMD_FM_RDS_STATUS,
				  args, ARRAY_SIZE(args), resp,
				  ARRAY_SIZE(resp), DEFAULT_TIMEOUT);
    if ((err < 0) || (resp[1] & 0x01) == 0)
        return -1;

    rds[0].error = (resp[12] >> 6) & 0x03;
	rds[0].rds = compose_u16(resp[4], resp[5]);
    rds[1].error = (resp[12] >> 4) & 0x03;
	rds[1].rds = compose_u16(resp[6], resp[7]);
    rds[2].error = (resp[12] >> 2) & 0x03;
	rds[2].rds = compose_u16(resp[8], resp[9]);
    rds[3].error = resp[12] & 0x03;
	rds[3].rds = compose_u16(resp[10], resp[11]);

	return err;
}

static int check_rds_enable(struct si4730_device *radio)
{
    int retval = si4730_read_property(radio, 0x1502);
    if (retval >= 0)
        return retval & 0x01;

    return retval;
}

/**************************************************************************
 * RDS Driver Functions
 **************************************************************************/

/*
 * si4730_rds - rds processing function
 */
static void si4730_rds(struct si4730_device *radio)
{
    unsigned char blocknum;
    unsigned char tmpbuf[3];
    struct rds_block rds[4];

#if 0
    /* get rds blocks */
    if (si4730_get_rds_registers(radio) < 0)
        return;
    if ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSR) == 0) {
        /* No RDS group ready */
        return;
    }
    if ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSS) == 0) {
        /* RDS decoder not synchronized */
        return;
    }
#endif
    mutex_lock(&radio->mutex);
    if (si4730_get_rds_int(radio) <= 0)
        goto exit;

    if (si4730_rds_status(radio, rds) < 0)
        goto exit;

    /* copy all four RDS blocks to internal buffer */
    for (blocknum = 0; blocknum < 4; blocknum++) {
        /* Fill the V4L2 RDS buffer */
        put_unaligned_le16(rds[blocknum].rds, &tmpbuf);
        //tmpbuf[0] = (u8)rds[blocknum].rds;
        //tmpbuf[1] = (u8)(rds[blocknum].rds >> 8);
        tmpbuf[2] = blocknum;           /* offset name */
        tmpbuf[2] |= blocknum << 3;     /* received offset */
        if (rds[blocknum].error > max_rds_errors)
            tmpbuf[2] |= 0x80; /* uncorrectable errors */
        else if (rds[blocknum].error > 0)
            tmpbuf[2] |= 0x40; /* corrected error(s) */

        //printk("%s %d %02x %02x %02x\n", __func__, blocknum, tmpbuf[0], tmpbuf[1], tmpbuf[2]);
        /* copy RDS block to internal buffer */
        memcpy(&radio->buffer[radio->wr_index], &tmpbuf, 3);
        radio->wr_index += 3;

        /* wrap write pointer */
        if (radio->wr_index >= radio->buf_size)
            radio->wr_index = 0;

        /* check for overflow */
        if (radio->wr_index == radio->rd_index) {
            /* increment and wrap read pointer */
            radio->rd_index += 3;
            if (radio->rd_index >= radio->buf_size)
                radio->rd_index = 0;
        }
    }
    //printk("%s %d: %s [%d %d]\n", __FILE__, __LINE__, __func__, radio->wr_index, radio->rd_index);

    mutex_unlock(&radio->mutex);

    /* wake up read queue */
    if (radio->wr_index != radio->rd_index)
        wake_up_interruptible(&radio->read_queue);

    return;

exit:
    mutex_unlock(&radio->mutex);
    return;
}


/*
 * si4730_work - rds work function
 */
static void si4730_work(struct work_struct *work)
{
    struct si4730_device *radio = container_of(work, struct si4730_device,
            work.work);

    if (radio->rds_on == 0)
        return;

    si4730_rds(radio);
    schedule_delayed_work(&radio->work, msecs_to_jiffies(rds_poll_time));
}


/*
 * Video4Linux Subdev Interface
 */

/* si4713_ioctl - deal with private ioctls (only rnl for now) */
long si4713_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct si4730_device *sdev = to_si4730_device(sd);
	struct si4713_rnl *rnl = arg;
	int frequency = 0;
	int rval = 0;

	if (!arg)
		return -EINVAL;

	mutex_lock(&sdev->mutex);
	switch (cmd) {
	case SI4713_IOC_MEASURE_RNL:

		if (sdev->power_state) {
			/* Set desired measurement frequency */
			rval = si4713_tx_tune_measure(sdev, frequency, 0);
			if (rval < 0)
				goto unlock;
			/* get results from tune status */
		}
		rnl->rnl = sdev->tune_rnl;
		break;

	default:
		/* nothing */
		rval = -ENOIOCTLCMD;
	}

unlock:
	mutex_unlock(&sdev->mutex);
	return rval;
}

/* V4L2 vidioc */
static int si4730_querycap(struct file *file, void  *priv,
					struct v4l2_capability *v)
{
	struct si4730_device *radio = video_drvdata(file);
	struct video_device *videodev = radio->videodev;

	strlcpy(v->driver, radio->client->dev.driver->name, sizeof(v->driver));
	snprintf(v->bus_info, sizeof(v->bus_info), "I2C:%s", dev_name(&videodev->dev));
	v->version = RADIO_VERSION;
	v->capabilities = V4L2_CAP_HW_FREQ_SEEK
        | V4L2_CAP_TUNER
        | V4L2_CAP_RADIO
        | V4L2_CAP_RDS_CAPTURE;

	return 0;
}
/*
 * si4730_v4l2_queryctrl - query control
 */
static struct v4l2_queryctrl si4730_v4l2_queryctrl[] = {
    {
        .id             = V4L2_CID_AUDIO_VOLUME,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Volume",
        .minimum        = 0,
        .maximum        = 63,
        .step           = 1,
        .default_value  = 63,
    },
    {
        .id             = V4L2_CID_AUDIO_MUTE,
        .type           = V4L2_CTRL_TYPE_BOOLEAN,
        .name           = "Mute",
        .minimum        = 0,
        .maximum        = 1,
        .step           = 1,
        .default_value  = 1,
    },
};

/*
 * si4730_vidioc_queryctrl - enumerate control items
 */
static int si4730_queryctrl(struct file *file, void *priv,
        struct v4l2_queryctrl *qc)
{
	struct si4730_device *radio = video_drvdata(file);
    unsigned char i = 0;
    int retval = -EINVAL;

    /* abort if qc->id is below V4L2_CID_BASE */
    if (qc->id < V4L2_CID_BASE)
        goto done;

    /* search video control */
    for (i = 0; i < ARRAY_SIZE(si4730_v4l2_queryctrl); i++) {
        if (qc->id == si4730_v4l2_queryctrl[i].id) {
            memcpy(qc, &(si4730_v4l2_queryctrl[i]), sizeof(*qc));
            retval = 0; /* found */
            break;
        }
    }

    /* disable unsupported base controls */
    /* to satisfy kradio and such apps */
    if ((retval == -EINVAL) && (qc->id < V4L2_CID_LASTP1)) {
        qc->flags = V4L2_CTRL_FLAG_DISABLED;
        retval = 0;
    }

done:
    if (retval < 0)
        dev_err(&radio->client->dev,
                "Error: query controls failed with %d\n", retval);
    return retval;
}


static int si4730_g_tuner(struct file *file, void *priv,
		struct v4l2_tuner *tuner)
{
	struct si4730_device *sdev = video_drvdata(file);
    struct tune_status status;
	int rval = -1;

    strcpy(tuner->name, "FM/AM");
	tuner->type = V4L2_TUNER_RADIO;
    tuner->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
                        V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO;
    tuner->rangelow  = 8750;
    tuner->rangehigh = 10800;
    tuner->audmode = V4L2_TUNER_MODE_STEREO;

    /* If there is a reliable method of detecting an RDS channel,
     * then this code should check for that before setting this
     * RDS subchannel. */
    tuner->rxsubchans |= V4L2_TUNER_SUB_RDS;

	mutex_lock(&sdev->mutex);

	if (sdev->power_state) {
		rval = si4730_tune_status(sdev, &status);
		if (rval < 0)
			goto unlock;

        tuner->signal = status.rssi;
        tuner->afc = status.afc;
	}

unlock:
	mutex_unlock(&sdev->mutex);
    if (rval < 0)
        dev_warn(&sdev->videodev->dev, "get tuner failed with %d\n", rval);

	return rval;
}

/* si4730_g_frequency - get tuner or modulator radio frequency */
static int si4730_g_frequency(struct file *file, void *priv,
		struct v4l2_frequency *freq)
{
	struct si4730_device *sdev = video_drvdata(file);
    struct tune_status status;
	int rval = -1;

	freq->type = V4L2_TUNER_RADIO;

	mutex_lock(&sdev->mutex);

	if (sdev->power_state) {
		rval = si4730_tune_status(sdev, &status);
		if (rval < 0)
			goto unlock;

		sdev->frequency = status.frequency;
	}

	freq->frequency = sdev->frequency;

unlock:
	mutex_unlock(&sdev->mutex);
    if (rval < 0)
        dev_warn(&sdev->videodev->dev, "get frequency failed with %d\n", rval);

	return rval;
}

/* si4730_s_frequency - set tuner or modulator radio frequency */
static int si4730_s_frequency(struct file *file, void *priv,
	struct v4l2_frequency *freq)

{
	struct si4730_device *sdev = video_drvdata(file);
	int rval = 0;
	u16 frequency = freq->frequency;

	/* Check frequency range */
	if (frequency < FREQ_RANGE_LOW || frequency > FREQ_RANGE_HIGH)
    {
        dev_err(&sdev->client->dev,
            "Error: %s: frequency %d is out of range[%d - %d]\n",
            __func__, frequency, FREQ_RANGE_LOW, FREQ_RANGE_HIGH);
		return -EDOM;
    }

	mutex_lock(&sdev->mutex);

	if (sdev->power_state) {
		rval = si4730_rx_tune_freq(sdev, frequency);
		if (rval < 0)
        {
			goto unlock;
        }
		frequency = rval;
		rval = 0;
	}
	sdev->frequency = frequency;
	freq->frequency = frequency;

unlock:
	mutex_unlock(&sdev->mutex);
	return rval;
}

static int si4730_g_ctrl(struct file *file, void *priv,
			    struct v4l2_control *ctrl)
{
	struct si4730_device *radio = video_drvdata(file);

    dev_dbg(&radio->client->dev, "%s[%x]\n", __func__, ctrl->id);
	switch (ctrl->id) {
        case V4L2_CID_AUDIO_MUTE: {
            int val;
            val = si4730_read_property(radio, 0x4001);
            if (val >= 0)
                ctrl->value = val & 0x03;
            return 0;
        }
        case V4L2_CID_AUDIO_VOLUME: {
            int val;
            val = si4730_read_property(radio, 0x4000);
            if (val >= 0)
                ctrl->value = val & 0x3f;
            return 0;
        }
        default:
            break;
    }
	return -EINVAL;
}

static int si4730_s_ctrl(struct file *file, void *priv,
			    struct v4l2_control *ctrl)
{
	struct si4730_device *radio = video_drvdata(file);

    dev_dbg(&radio->client->dev,
            "%s[%x] %x\n", __func__, ctrl->id, ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
        if (ctrl->value != 0)
            si4730_write_property(radio, 0x4001, 0x0003);
        else
            si4730_write_property(radio, 0x4001, 0x0000);
		return 0;

    case V4L2_CID_AUDIO_VOLUME:
        si4730_write_property(radio, 0x4000, ctrl->value & 0x3f);
        return 0;

    case V4L2_CID_PRIVATE_TAVARUA_SRCHON:
        return si4730_cancel_seek(radio);
	}
	return -EINVAL;
}

/*
 * si4730_s_hw_freq_seek - set hardware frequency seek
 */
static int si4730_s_hw_freq_seek(struct file *file, void *priv,
		struct v4l2_hw_freq_seek *seek)
{
	struct si4730_device *radio = video_drvdata(file);
	int retval = 0;

	mutex_lock(&radio->mutex);
	if (seek->tuner != 0) {
        dev_err(&radio->client->dev, "Error in %s\n", __func__);
		retval = -EINVAL;
		goto done;
	}

	retval = si4730_set_seek(radio, seek->wrap_around, seek->seek_upward);

done:
	if (retval < 0)
		dev_warn(&radio->videodev->dev,
			"set hardware frequency seek failed with %d\n", retval);
	mutex_unlock(&radio->mutex);
	return retval;
}

/**************************************************************************
 * File Operations Interface
 **************************************************************************/

/*
 * si4730_fops_open - file open
 */
int si4730_fops_open(struct file *file)
{
	struct si4730_device *radio = video_drvdata(file);
	int retval = 0;

	if (radio->users != 0)
		return -ENODEV;

    dev_dbg(&radio->client->dev, "%s\n", __func__);
	/* start radio */
	si4730_set_power_state(radio, POWER_ON);
	si4730_setup(radio);
	radio->users = 1;

	return retval;
}


/*
 * si4730_fops_release - file release
 */
int si4730_fops_release(struct file *file)
{
	struct si4730_device *radio = video_drvdata(file);
	int retval;

	/* safety check */
	if (!radio)
		return -ENODEV;

    dev_dbg(&radio->client->dev, "%s\n", __func__);
    /* stop rds reception */
    cancel_delayed_work_sync(&radio->work);
    /* cancel read processes */
    wake_up_interruptible(&radio->read_queue);

	/* stop radio */
	retval = si4730_set_power_state(radio, POWER_OFF);
	radio->users = 0;
    radio->rds_on = 0;

	return retval;
}

/*
 * si4730_fops_read - read RDS data
 */
static ssize_t si4730_fops_read(struct file *file, char __user *buf,
        size_t count, loff_t *ppos)
{
    struct si4730_device *radio = video_drvdata(file);
    int retval = 0;
    unsigned int block_count = 0;

    /* switch on rds reception */
    if (check_rds_enable(radio) != 1) {
        si4730_rds_on(radio);
        schedule_delayed_work(&radio->work,
                msecs_to_jiffies(rds_poll_time));
    }

    /* block if no new data available */
    while (radio->wr_index == radio->rd_index) {
        if (file->f_flags & O_NONBLOCK) {
            retval = -EWOULDBLOCK;
            goto done;
        }
        if (wait_event_interruptible(radio->read_queue,
                    radio->wr_index != radio->rd_index) < 0) {
            retval = -EINTR;
            goto done;
        }
    }

    /* calculate block count from byte count */
    count /= 3;

    /* copy RDS block out of internal buffer and to user buffer */
    mutex_lock(&radio->mutex);
    while (block_count < count) {
        if (radio->rd_index == radio->wr_index)
            break;

        /* always transfer rds complete blocks */
        if (copy_to_user(buf, &radio->buffer[radio->rd_index], 3))
            /* retval = -EFAULT; */
            break;

        /* increment and wrap read pointer */
        radio->rd_index += 3;
        if (radio->rd_index >= radio->buf_size)
            radio->rd_index = 0;

        /* increment counters */
        block_count++;
        buf += 3;
        retval += 3;
    }
    //printk("%s %d: %s [%d %d]\n", __FILE__, __LINE__, __func__, radio->wr_index, radio->rd_index);
    mutex_unlock(&radio->mutex);

done:
    return retval;
}


/*
 * si4730_fops_poll - poll RDS data
 */
static unsigned int si4730_fops_poll(struct file *file,
        struct poll_table_struct *pts)
{
    struct si4730_device *radio = video_drvdata(file);
    int retval = 0;

    /* switch on rds reception */
    if (check_rds_enable(radio) != 1) {
        si4730_rds_on(radio);
        schedule_delayed_work(&radio->work,
                msecs_to_jiffies(rds_poll_time));
    }

    poll_wait(file, &radio->read_queue, pts);

    if (radio->rd_index != radio->wr_index)
        retval = POLLIN | POLLRDNORM;

    return retval;
}


/*
 * si4730_fops - file operations interface
 * video_ioctl2 is part of the v4l2 implementations. Change this pointer to the
 * ioctl function you want to implement, in case you don't want to be part of
 * v4l2.
 */
static const struct v4l2_file_operations si4730_fops = {
	.owner                  = THIS_MODULE,
	.ioctl                  = video_ioctl2,
	.open                   = si4730_fops_open,
	.release                = si4730_fops_release,
    .read                   = si4730_fops_read,
    .poll                   = si4730_fops_poll,
};


/*
 * si4730_ioctl_ops - video device ioctl operations
 */
static const struct v4l2_ioctl_ops si4730_ioctl_ops = {
	.vidioc_querycap        = si4730_querycap,
    .vidioc_queryctrl       = si4730_queryctrl,
    .vidioc_g_tuner         = si4730_g_tuner,
	.vidioc_g_frequency     = si4730_g_frequency,
	.vidioc_s_frequency     = si4730_s_frequency,
	.vidioc_g_ctrl          = si4730_g_ctrl,
	.vidioc_s_ctrl          = si4730_s_ctrl,
	.vidioc_s_hw_freq_seek	= si4730_s_hw_freq_seek,
};


/*
 * si4730_viddev_template - video device interface
 */
struct video_device si4730_viddev_template = {
	.fops                   = &si4730_fops,
	.name                   = DRIVER_NAME,
	.release                = video_device_release,
	.ioctl_ops              = &si4730_ioctl_ops,
//	.debug                  = V4L2_DEBUG_IOCTL | V4L2_DEBUG_IOCTL_ARG,
};


/*
 * I2C driver interface
 */
/* si4730_probe - probe for the device */
static int si4730_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct si4730_device *sdev;
	int rval;

    printk("%s:%d %s\n", __FILE__, __LINE__, __func__);
	sdev = kzalloc(sizeof *sdev, GFP_KERNEL);
	if (!sdev) {
		dev_err(&client->dev, "Failed to alloc video device.\n");
		rval = -ENOMEM;
		goto exit;
	}
	sdev->client = client;
	sdev->users = 0;
	/* video device allocation and initialization */
	sdev->videodev = video_device_alloc();
	if (!sdev->videodev) {
		rval = -ENOMEM;
		goto free_video;
	}

	mutex_init(&sdev->mutex);
	memcpy(sdev->videodev, &si4730_viddev_template,
			sizeof(si4730_viddev_template));
	video_set_drvdata(sdev->videodev, sdev);

	rval = si4730_initialize(sdev);
	if (rval < 0) {
		dev_err(&sdev->client->dev, "Failed to probe device information.\n");
		goto free_sdev;
	}

    /* rds buffer allocation */
    sdev->buf_size = rds_buf * 3;
    sdev->buffer = kmalloc(sdev->buf_size, GFP_KERNEL);
    if (!sdev->buffer) {
        rval = -EIO;
        goto free_video;
    }

    /* rds buffer configuration */
    sdev->wr_index = 0;
    sdev->rd_index = 0;
    init_waitqueue_head(&sdev->read_queue);
    
    /* prepare rds work function */
    INIT_DELAYED_WORK(&sdev->work, si4730_work);

	/* register video device */
	rval = video_register_device(sdev->videodev, VFL_TYPE_RADIO,
			radio_nr);
	if (rval) {
		dev_warn(&client->dev, "Could not register video device\n");
		goto free_all;
	}
	i2c_set_clientdata(client, sdev);
    dev_dbg(&client->dev, "driver probed.\n");
    
	return 0;

free_all:
    kfree(sdev->buffer);

free_video:
	video_device_release(sdev->videodev);

free_sdev:
	kfree(sdev);
exit:
	return rval;
}

/* si4730_remove - remove the device */
static int si4730_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct si4730_device *sdev = to_si4730_device(sd);

	if (sdev->power_state)
		si4730_set_power_state(sdev, POWER_DOWN);

	if (client->irq > 0)
		free_irq(client->irq, sdev);

    cancel_delayed_work_sync(&sdev->work);
	v4l2_device_unregister_subdev(sd);
    video_unregister_device(sdev->videodev);
    kfree(sdev->buffer);
	kfree(sdev);

	return 0;
}

/* si4730_i2c_driver - i2c driver interface */
static const struct i2c_device_id si4730_id[] = {
	{ DRIVER_NAME , 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, si4730_id);

static struct i2c_driver si4730_i2c_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
	},
	.probe		= si4730_probe,
	.remove     = si4730_remove,
	.id_table   = si4730_id,
};

/* Module Interface */
static int __init si4730_module_init(void)
{
    printk(DRIVER_DESC"\n");
	return i2c_add_driver(&si4730_i2c_driver);
}

static void __exit si4730_module_exit(void)
{
	i2c_del_driver(&si4730_i2c_driver);
}

module_init(si4730_module_init);
module_exit(si4730_module_exit);

/* Module information */
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

