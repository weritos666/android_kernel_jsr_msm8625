/*
* drivers/input/touchscreen/ft5306.c
*
* Truly CT0402 ft5306 TouchScreen driver.
*
* Copyright (c) 2012, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>

#include <linux/ft5306_ts.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/mfd/pmic8058.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define TS_IRQ_GPIO				48
#define TS_WAKE_GPIO				83		
#define TS_RESET_PMIC_GPIO			6
//#define TS_I2C_ADDR					0x38
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)	(NR_GPIO_IRQS + pm_gpio)

//#define FW_UPGRADE_ENABLE 1		/*Enable upgrade TP Firmware through i2c bus*/

#define RESET_ENABLE 0		/*Enable RESET */
//#define WAKE_ENABLE 1		/*Enable WAKE*/

/*registers*/
#define RAW_DATA_BEGIN_ADDR			0x00
#define TD_STATUS_ADDR				0x02
#define CHIP_ID_ADDR				0x5A
#define POWER_MODE_ADDR				0xA5
/* pressed event */
#define TOUCH_DOWN			0x00
#define TOUCH_UP			0x01
#define	HOLD				0x02
#define HIBERNATE			0x03

#define EVENT_DOWN				1
#define EVENT_UP				0

#define MIN_X				0
#define MAX_X				540
#define MIN_Y				0
#define MAX_Y				960
#define KEY_Y_TOP		981
//#define KEY_Y_BOTTOM		839
#define KEY_MENU_XL		0
#define KEY_MENU_XR		119    //250
#define KEY_HOME_XL		120       //250
#define KEY_HOME_XR		299      //350
#define KEY_BACK_XL		300        //350
#define KEY_BACK_XR		479         //540
//#define KEY_SEARCH_XL		378
//#define KEY_SEARCH_XR		458

#define ft5306_PRINT_ERROR (1U << 0)
#define ft5306_PRINT_WARNING   (1U << 1)
#define ft5306_PRINT_INFO  (1U << 2)
#define ft5306_PRINT_DEBUG (1U << 3)


static int debug_mask = ft5306_PRINT_ERROR | \
			ft5306_PRINT_INFO  | \
			ft5306_PRINT_WARNING;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define pr_ft5306(debug_level_mask, args...) \
	do { \
		if (debug_mask & ft5306_PRINT_##debug_level_mask) { \
			printk(KERN_##debug_level_mask "[ft5306] "args); \
		} \
	} while (0)


#ifdef SUPPORT_FIVE_POINT_TOUCH
#define MAX_TOUCH_POINTS	5	/*support five points pressed*/
#define MAX_RAW_DATA_BYTE   31
#else
#define MAX_TOUCH_POINTS	10//2	/*support two points pressed*/
#define MAX_RAW_DATA_BYTE   61//15
#endif
#define POINT_DATA_LENGTH		6


#define REPORT_TP(a, b) { \
	input_report_abs(a->input, ABS_MT_TRACKING_ID, b.touch_id); \
	input_report_abs(a->input, ABS_MT_TOUCH_MAJOR, b.pressed); \
	input_report_abs(a->input, ABS_MT_POSITION_X, b.x); \
	input_report_abs(a->input, ABS_MT_POSITION_Y, b.y); \
	input_mt_sync(a->input); \
}

struct point_data {
	u16 x;
	u16 y;
	unsigned int touch_event;
	unsigned int touch_id;
	unsigned int pressed;
};

struct ft5306_data {
	struct input_dev *input;
	struct i2c_client *client; 
	struct work_struct work;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	u32 irq;
	int minx;
	int miny;
	int maxx;
	int maxy;
	int maxTouch;
	u8 raw_data[MAX_RAW_DATA_BYTE];
	struct point_data points[MAX_TOUCH_POINTS];
	unsigned int total_tp;
	unsigned int last_total_tp;
};


//static struct i2c_adapter *this_adapter;
//static struct i2c_client *this_client;

static int old_key_type = 0; 


void ft5306_release_old_key(struct input_dev *dev)
{
	if(old_key_type)
	{
		del_timer(&dev->timer);
		memset(dev->key,0,sizeof(long)*8);
		input_report_key(dev,KEY_UNKNOWN,EVENT_DOWN);
		input_report_key(dev,KEY_UNKNOWN,EVENT_UP);		
		old_key_type = 0;
	}
}




/*Multi-byte read*/
static int ft5306_i2c_read_value(struct i2c_client *client,
		u8 saddr, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	if(client == NULL) {
		pr_err("%s:i2c_client is NULL\n", __func__);
		return -EINVAL;
	}
	if(buf == NULL) {
		pr_err("%s:buf is NULL\n", __func__);
		return -EINVAL;
	}

	msgs[0].addr = client->addr;		/*i2c adress*/
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &saddr;			/*register address*/

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		pr_err("%s:i2c_transfer failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

/*Transform raw data to the specify data*/
static void inline filt_data(struct point_data *data, const u8 *rd) {
	data->touch_event = (rd[0] & 0xC0) >> 6;
	data->touch_id = (rd[2] & 0xF0) >> 4;
	data->x = ((rd[0] & 0x0F) << 8) | rd[1];
	data->y = ((rd[2] & 0x0F) << 8) | rd[3];


}

#if 1
/*Get the virtual key type and report*/
static void report_key(struct ft5306_data *data, u16 x, int flag)
{
	int key_type = 0; 
	if (x > KEY_MENU_XL && x < KEY_MENU_XR) {
		key_type = KEY_MENU;
	} else if (x > KEY_HOME_XL && x < KEY_HOME_XR) {
		key_type = KEY_HOME;
	} else if (x > KEY_BACK_XL && x < KEY_BACK_XR) {
		key_type = KEY_BACK;
	//} else if (x > KEY_SEARCH_XL && x < KEY_SEARCH_XR) {
		//key_type = KEY_SEARCH;			
	}

	if(flag == EVENT_DOWN)
	{
		old_key_type = key_type; 	
	}
	else
	{
		if(old_key_type != key_type)	
		{
			ft5306_release_old_key(data->input);
		}
	}
	if (key_type)
	{		
		input_report_key(data->input, key_type, flag);
	}
}
#endif
/*Irq handler*/
static irqreturn_t ts_irq(int irq, void *dev_id)
{
	struct ft5306_data *data = (struct ft5306_data *)dev_id;
	if (data == NULL) {
		pr_err("%s:data is null!\n", __func__);
		return -EINVAL;
	}
	disable_irq_nosync(data->irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

static void work_func(struct work_struct *work)
{
	int i,j = 3;
	int ret;
	static int normal_key=0;
	struct ft5306_data *data = container_of(work,
			struct ft5306_data, work);
	if (data == NULL) {
		pr_err("%s:data is NULL\n", __func__);
		return ;
	}

	ret = ft5306_i2c_read_value(data->client, RAW_DATA_BEGIN_ADDR, data->raw_data, MAX_RAW_DATA_BYTE);
	if(ret) {
		pr_ft5306(INFO, "%s read failed.\n", __func__);
		goto exit_work;
	}
	data->total_tp = data->raw_data[2] & 0x0F;      /*Get current total pressed points*/
	if(data->total_tp > MAX_TOUCH_POINTS)           /*Ignore points which exceed the max pressed points we support */
		data->total_tp = MAX_TOUCH_POINTS;

	if (data->last_total_tp < data->total_tp) {	/*Had a new points pressed down*/
		data->last_total_tp = data->total_tp;
	}

	/*Tansform data*/
	for(i = 0; i < data->last_total_tp; i++) {
		filt_data(&data->points[i], &data->raw_data[j]);
		j += POINT_DATA_LENGTH;
	}
	input_report_key(data->input,BTN_TOUCH, !!(data->total_tp));
	for(i = 0; i < data->last_total_tp; i++) {

		if (data->points[i].y > KEY_Y_TOP) {	/*pressed on the key area*/
			if (data->points[i].touch_event == TOUCH_DOWN)
			{
				report_key(data, data->points[i].x, EVENT_DOWN);
				normal_key=1;
			}
			else if (data->points[i].touch_event == TOUCH_UP)
			{
				report_key(data, data->points[i].x, EVENT_UP);
				normal_key=0;
			}

		}
		if (data->points[i].touch_event == TOUCH_DOWN || data->points[i].touch_event == HOLD)
			data->points[i].pressed = EVENT_DOWN;
		else if(data->points[i].touch_event == TOUCH_UP)
		{
			data->points[i].pressed = EVENT_UP;

			if(normal_key)
			{	
				//KEY_DOWN  AND   TOUCH_UP
				ft5306_release_old_key(data->input);
				normal_key=0;
			}
		}
		
		REPORT_TP(data, data->points[i]);

		pr_ft5306(DEBUG, "\n");
		pr_ft5306(DEBUG, "ABS_MT_TRACKING_ID: %d\n", data->points[i].touch_id); \
			pr_ft5306(DEBUG, "ABS_MT_TRACKING_MAJOR: %d\n", data->points[i].pressed); \
			pr_ft5306(INFO, "ABS_MT_POSITION_X: %d\n", data->points[i].x); \
			pr_ft5306(INFO, "ABS_MT_POSITION_Y: %d\n", data->points[i].y); \
	}

	input_sync(data->input);

	pr_ft5306(DEBUG, "input_sync\n");
	data->last_total_tp = data->total_tp;	/*Record previous total pressed points*/
exit_work:
	enable_irq(data->irq);
}

#ifdef CONFIG_PM
static int ft5306_suspend(struct i2c_client *client, pm_message_t state)
{
	struct ft5306_data *data = i2c_get_clientdata(client);
#ifdef WAKE_ENABLE
	int ret;
#endif
	pr_info("[ft5306] %s \n", __func__);
	disable_irq(data->irq);
	flush_work(&data->work);
	if (cancel_work_sync(&data->work)) {
		pr_err("[ft5306]  %s cancel work failed!\n", __func__);
		enable_irq(data->irq);
	}

#ifdef WAKE_ENABLE
	/*Set IC switch to hibernate mode*/
	ret = i2c_smbus_write_byte_data(client, POWER_MODE_ADDR, HIBERNATE); 
	if (ret)
		pr_err("i2c write hibernate command error: %d\n", ret);
	/*config WAKE from output to input*/

	gpio_tlmm_config(GPIO_CFG(TS_WAKE_GPIO, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
#endif

	return 0;
}

static int ft5306_resume(struct i2c_client *client)
{
	struct ft5306_data *data = i2c_get_clientdata(client);
	pr_info("[ft5306] %s \n", __func__);

	/*config WAKE from input to output*/
#ifdef WAKE_ENABLE
	gpio_tlmm_config(GPIO_CFG(TS_WAKE_GPIO, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	msleep(1);
	/*Wake IC,switch to nomal mode*/
	gpio_set_value(TS_WAKE_GPIO, 0);
	msleep(1);
	gpio_set_value(TS_WAKE_GPIO, 1);
	msleep(200);
#endif
	enable_irq(data->irq);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5306_ts_early_suspend(struct early_suspend *h)
{
	struct ft5306_data *data = container_of(h, struct ft5306_data,
			early_suspend);
	ft5306_suspend(data->client, PMSG_SUSPEND);
}

static void ft5306_ts_late_resume(struct early_suspend *h)
{
	struct ft5306_data *data = container_of(h, struct ft5306_data,
			early_suspend);
	ft5306_resume(data->client);
}
#endif

#else
#define ft5306_suspend	NULL
#define ft5306_resume	NULL

#endif

#ifdef FW_UPGRADE_ENABLE  
static u8 fts_ctpm_get_upg_ver(void);
static int fts_ctpm_fw_upgrade_with_i_file(void);
static u8 fts_register_write(u8 e_reg_name, u8 b_data);
#endif
static int __devinit ft5306_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct ft5306_platform_data *pdata;
	struct ft5306_data *data;
	struct input_dev         *input;

	int err = 0;
	int error;
	int i=0;
#ifdef FW_UPGRADE_ENABLE        
	u8 firmware_ver;
	u8 chip_fw_ver;
	int ret;
#endif

	//static struct vreg *vreg_ts,*vreg_ts2;
	u8 chip_id;

	//struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	data = kzalloc(sizeof(struct ft5306_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_data_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	/* Initialize Platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}

	if(pdata->init_platform_hw)
		pdata->init_platform_hw();
	//Check chip ID
	do{
		i++;
		msleep(35);
		if(ft5306_i2c_read_value(client, CHIP_ID_ADDR, &chip_id, 1))
		{
			pr_err("Read chip id failed!Maybe have not connected device!\n");
			goto device_error_exit;	
		}
		else break;
	}while(i<4);


#ifdef FW_UPGRADE_ENABLE
	msleep(10);
	firmware_ver = fts_ctpm_get_upg_ver();	/*Get Firmware file version*/
	ret = ft5306_i2c_read_value(client, 0x40, &chip_fw_ver, 1); /*Get chip current Firmware version*/
	if(ret) {
		pr_ft5306(INFO, "get chip current Firmware version failed.\n");
		goto exit_init;
	}
	pr_ft5306(INFO, " chip_fw_ver 0x%2x  firmware_ver 0x%2x\n", chip_fw_ver, firmware_ver);

	/*If chip current Firmware version less than Firmware file version,we upgrade Firmware*/
	if (chip_fw_ver < firmware_ver || chip_fw_ver == 0x40) {
		pr_ft5306(INFO, "need upgrade TP firmware!\n");

		ret = fts_ctpm_fw_upgrade_with_i_file();
		if (ret) {
			pr_ft5306(INFO, "upgrade TP firmware failed!\n");
		} else {
			msleep(10);
		}
		/*after updata need calibrate*/
		fts_register_write(0xfc, 4);
		msleep(1000);
		ft5306_i2c_read_value(client, 0x40, &chip_fw_ver, 1);
		pr_ft5306(INFO, "the new firmware version is %x\n", chip_fw_ver);
	} else { 
		pr_ft5306(INFO, "no need upgrade TP firmware!\n");
		pr_ft5306(INFO, "the chip FW version is %x\n", chip_fw_ver);
	}
#endif

	if(pdata->display_minx)
		data->minx = pdata->display_minx;
	if(pdata->display_maxx)
		data->maxx = pdata->display_maxx;
	if(pdata->display_miny)
		data->miny = pdata->display_miny;
	if(pdata->display_maxy)
		data->maxy = pdata->display_maxy;
	if(pdata->numtouch)
		data->maxTouch = pdata->numtouch;

	data->client = client;
	data->input  = input;
	i2c_set_clientdata(data->client, data);


	data->input->name = "ft5306_ts";
	data->input->id.bustype = BUS_I2C;


	data->input->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	data->input->absbit[0] = BIT_MASK(ABS_MT_POSITION_X) | \
				 BIT_MASK(ABS_MT_POSITION_Y) | \
				 BIT_MASK(ABS_MT_TOUCH_MAJOR) | \
				 BIT_MASK(ABS_MT_TRACKING_ID);
	set_bit(KEY_MENU, data->input->keybit);
	set_bit(KEY_BACK, data->input->keybit);
	set_bit(KEY_HOME, data->input->keybit);
	set_bit(KEY_SEARCH, data->input->keybit);
	set_bit(KEY_UNKNOWN,data->input->keybit);
	set_bit(BTN_TOUCH, data->input->keybit);
	input_set_abs_params(data->input, ABS_MT_POSITION_X, data->minx,
			data->maxx, 0, 0);
	input_set_abs_params(data->input, ABS_MT_POSITION_Y, data->miny, data->maxy, 0, 0);
	input_set_abs_params(data->input, ABS_MT_TOUCH_MAJOR, 0,
			1, 0, 0);
	input_set_abs_params(data->input, ABS_MT_TRACKING_ID, 0, 
			2, 0, 0);


	input_set_drvdata(data->input, data);


	err = input_register_device(data->input);
	if (err<0) {
		pr_err("%s: Cannot register input (%d)\n", __func__, err);
		goto err_register_device;
	}

	INIT_WORK(&data->work, work_func);



	if(pdata->init_gpio)
		pdata->init_gpio();
	data->irq = gpio_to_irq(pdata->irq_gpio);

	err = request_irq(data->irq, ts_irq, IRQF_TRIGGER_RISING,
			client->dev.driver->name, data);
	if (err < 0) {
		dev_err(&client->dev,
				"failed to allocate irq %d\n", data->irq);
		goto err_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = ft5306_ts_early_suspend;
	data->early_suspend.resume = ft5306_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	return 0;

err_irq:
err_register_device:
#ifdef FW_UPGRADE_ENABLE
exit_init:
#endif
device_error_exit:
err_pdata:
	input_free_device(input);
err_input_dev_alloc:
	kfree(data);
err_data_alloc:
	return 0;

}

static int __devexit ft5306_remove(struct i2c_client *client)
{
	struct ft5306_data *data = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(data->irq, data);
	gpio_free(TS_IRQ_GPIO);
	gpio_free(TS_WAKE_GPIO);

	cancel_work_sync(&data->work);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static const struct i2c_device_id ft5306_id[] = {
	{"ft5306_ts", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ft5306_id);

static struct i2c_driver ft5306_driver = {
	.driver = {
		.name = "ft5306_ts",
		.owner = THIS_MODULE,
	},
	.probe = ft5306_probe,
	.remove = ft5306_remove,
	.id_table = ft5306_id,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ft5306_suspend,
	.resume = ft5306_resume,
#endif
};

/*For upgrade TP Firmware through i2c bus*/
#ifdef FW_UPGRADE_ENABLE
typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0
#define FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
#include "FTS0094P430_V1a_20110921_app.h"
};

static u8 fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2) {    
		return CTPM_FW[ui_sz - 2];
	} else {    
		/*TBD, error handling?*/
		return 0xff; /*default value*/
	}
	return 0;    
}

static u8 i2c_write_interface(u8* pbt_buf, u16 dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret<=0) {
		printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

static u8 i2c_read_interface(u8* pbt_buf, u32 dw_lenth)
{
	int ret;

	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret<=0) {
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

static u8 fts_register_write(u8 e_reg_name, u8 b_data)
{
	//	u8 ecc = 0;
	u8 write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = b_data;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}

static int fts_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr    = this_client->addr,
			.flags    = 0,
			.len    = length,
			.buf    = txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("[TSP ]%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static u8 byte_write(u8* pbt_buf, u32 dw_len)
{	    
	return i2c_write_interface(pbt_buf, dw_len);
}

static u8 cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	u8 write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;

	return i2c_write_interface(write_cmd, num);
}

static u8 byte_read(u8* pbt_buf, u8 bt_len)
{
	return i2c_read_interface(pbt_buf, bt_len);
}

static void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
}

static E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, u16 dw_lenth)
{
	u8 reg_val[2] = {0};
	u16 i = 0;

	u16  packet_number;
	u16  j;
	u16  temp;
	u16  lenght;
	u8  packet_buf[FTS_PACKET_LENGTH + 6];
	u8  auc_i2c_write_buf[10];
	u8 bt_ecc;
	int		i_ret;

	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	fts_register_write(0xfc,0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	fts_register_write(0xfc,0x55);
	printk("[TSP] Step 1: Reset CTPM test\n");

	delay_qt_ms(30);   

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i ++;
		i_ret = fts_i2c_txdata(auc_i2c_write_buf, 2);
		delay_qt_ms(5);
	}while(i_ret <= 0 && i < 10 );

	/*********Step 3:check READ-ID***********************/        
	cmd_write(0x90,0x00,0x00,0x00,4);
	byte_read(reg_val,2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3) {
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	} else {
		return ERR_READID;
	}

	/*********Step 4:erase app*******************************/
	cmd_write(0x61,0x00,0x00,0x00,1);

	delay_qt_ms(1500);
	printk("[TSP] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(lenght>>8);
		packet_buf[5] = (u8)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++) {
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		delay_qt_ms(FTS_PACKET_LENGTH/3 + 10);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0) {
			printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;

		for (i=0;i<temp;i++) {
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);    
		delay_qt_ms(20);
	}

	/*send the last six byte*/
	for (i = 0; i<6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		temp = 1;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);  
		delay_qt_ms(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc) {
		return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);
	msleep(200);

	return ERR_OK;
}


static int fts_ctpm_fw_upgrade_with_i_file(void)
{
	u8* pbt_buf = FTS_NULL;
	int i_ret;

	/*=========FW upgrade========================*/
	pbt_buf = CTPM_FW;
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
	if (i_ret != 0) {
		/*error handling ...*/
	}

	return i_ret;
}
#endif
#define FT5306_GPIO_RST 26
static int __init ft5306_ts_init(void)
{
#if 0	
	int ret;
	ret = gpio_request(FT5306_GPIO_RST, "ft5306 RST GPIO");
	if (ret) {
		printk(KERN_ERR "%s: Failed to request GPIO %d\n",
				__func__, FT5306_GPIO_RST);
		return ret;
	}

	gpio_tlmm_config(GPIO_CFG(FT5306_GPIO_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),GPIO_CFG_ENABLE);
	gpio_set_value(FT5306_GPIO_RST, 1);	
#endif
	return i2c_add_driver(&ft5306_driver);
}


static void __exit ft5306_exit(void)
{
	i2c_del_driver(&ft5306_driver);
}

module_init(ft5306_ts_init);
module_exit(ft5306_exit);

MODULE_AUTHOR("FocalTech");
MODULE_DESCRIPTION("ft5306 pressed pannel driver");
MODULE_LICENSE("GPL v2");
