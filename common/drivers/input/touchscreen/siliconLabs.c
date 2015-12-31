/* drivers/input/touchscreen/silabs_f760.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_LTN_DTV
#include <plat/syscfg.h>
#else
#include <linux/d1982/pmic.h>
#include <linux/i2c/tsp_gpio.h>
#endif
#include <linux/firmware.h>
#include "SiLabsFW/siLabs_download.h"
//#include "../../i2c/chips/taos.h"

#define __TOUCH_DEBUG__ 0

#define MAX_X	240 
#define MAX_Y	320 
#define F760_MAX_TOUCH		2
#define ESCAPE_ADDR 	    0xAA
#define TS_READ_START_ADDR       0x10
#define TS_READ_VERSION_ADDR	0x1F
#define TS_READ_ESD_ADDR	        0x1E
#define TS_READ_REGS_LEN 		13
#define SILABS_MAX_TOUCH		F760_MAX_TOUCH
#define I2C_RETRY_CNT			10
#define TOUCH_ON     1
#define TOUCH_OFF   0
#define PRESS_KEY				1
#define RELEASE_KEY				0
#define SILABS_TS_NAME "siliconLabs-ts"
#define JIG_MODE_COMMAND 0xA0
#define BASELINE_ADDRESS         0x013E
#define RAWCOUNT_ADDRESS         0x3A
#ifdef CONFIG_LTN_DTV
#define RAWDATA_ADDRESS          RAWCOUNT_ADDRESS
#endif
#define I2CMAP_BUTTON_ADDRESS    0x029C
#define DEBUG_ADDRESS    0x02A0
#define NUM_TX_CHANNEL 13
#define NUM_RX_CHANNEL 10
#define QUICKSENSE_OVERHEAD     6 
#define NUM_MTRBUTTONS             2

#if defined(CONFIG_LTN_DTV)
#define FW_VER          20
#else
#define FW_VER          14
#endif

#ifdef CONFIG_LTN_DTV
#define TSP_SCL 3
#define TSP_SDA 6
#define OUTPUT_2_8V                         (2800000)   // Totoro TD : 2.8V
#define OUTPUT_2_9V                         (2900000)   // Totoro     : 2.9V
 #endif

static int prev_wdog_val = -1;
//static int check_ic_counter = 3;

static struct workqueue_struct *silabs_wq;
static struct workqueue_struct *check_ic_wq;
static struct regulator *touch_regulator = NULL;

static int touchkey_status[F760_MAX_TOUCH];
int touch_id[2], posX[2], posY[2], strength[2];
static int firmware_ret_val = -1;
int tsp_irq;
int TSP_MODULE_ID;
int value_for_reference = 1;
int value_for_difference = 1;
int IsDuringCall = 0;
int touch_check = 0;
static struct mutex		reset_lock;
#if defined(CONFIG_LTN_DTV)
extern int tsp_charger_type_status;
#else
int charger_status_forTSP = 0;
#endif
static int testmode = 0;
static int pre_ta_stat = 0;
int IsdebugMatirx = 0;
extern char *saved_command_line;

#ifdef CONFIG_LTN_DTV
int FW_VERSION;
EXPORT_SYMBOL(FW_VERSION);
#endif
EXPORT_SYMBOL(TSP_MODULE_ID);
static DEFINE_SPINLOCK(silabs_spin_lock);
#ifdef CONFIG_LTN_DTV
uint8_t buf_firmware[3];
int PHONE_VER;
unsigned long check_node;

int Tx_Channel = NUM_TX_CHANNEL;
int Rx_Channel = NUM_RX_CHANNEL;
uint16_t baseline_node[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
#endif
int firm_update( void );

enum
{
	TOUCH_SCREEN=0,
	TOUCH_KEY
};

struct muti_touch_info
{
	int strength;
#ifdef CONFIG_LTN_DTV
	int status;
#endif
	int width;	
	int posX;
	int posY;
};

struct ts_data {
	uint16_t addr;
	struct i2c_client *client; 
	struct input_dev *input_dev;
    	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
    	struct work_struct  work_timer;
	struct early_suspend early_suspend;
};

struct ts_data *ts_global;

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

void TSP_forced_release_forkey(struct ts_data *ts);
static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_sw_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t tkey_rawcounter_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tkey_rawcounter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t show_node(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_module(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_debugMatrix_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_difference_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_reference_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_difference_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t tsp_reference_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t tsp_xy(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_StartCall_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_EndCall_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t read_debug_address_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t read_debug_address_silabs1(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_reset_outJIGMode(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef CONFIG_LTN_DTV
static ssize_t rawdata_show_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_enable_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_disable_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t baseline_show_silabs(struct device *dev, struct device_attribute *attr, char *buf1);
static ssize_t diff_show_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fimware_show_versname(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t rawdata_pass_fail_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t read_node(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(reference, S_IRUGO, baseline_show_silabs, NULL) ;
static DEVICE_ATTR(raw, S_IRUGO, rawdata_show_silabs, NULL) ;
static DEVICE_ATTR(raw_enable, S_IRUGO, raw_enable_silabs, NULL) ;
static DEVICE_ATTR(raw_disable, S_IRUGO, raw_disable_silabs, NULL) ;
static DEVICE_ATTR(diff, S_IRUGO, diff_show_silabs, NULL) ;
static DEVICE_ATTR(versname, S_IRUGO, fimware_show_versname, NULL) ;
static DEVICE_ATTR(raw_value, S_IRUGO, rawdata_pass_fail_silabs, NULL) ;
static DEVICE_ATTR(firmware	, S_IRUGO, firmware_show, NULL);
#endif
static DEVICE_ATTR(firmware_ver	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, firmware_show, firmware_store);
static DEVICE_ATTR(tsp_StartCall	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_StartCall_show, NULL);
static DEVICE_ATTR(tsp_EndCall	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_EndCall_show, NULL);
static DEVICE_ATTR(debug_matrix	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_debugMatrix_show, NULL);
static DEVICE_ATTR(sw_ver	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, firmware_sw_show, NULL);
static DEVICE_ATTR(tsp_module, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_module, NULL);
static DEVICE_ATTR(tsp_difference,  S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_difference_show, tsp_difference_store);
static DEVICE_ATTR(tsp_reference,  S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_reference_show, tsp_reference_store);
static DEVICE_ATTR(tsp_xy	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_xy, NULL);
static DEVICE_ATTR(firmware_ret	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, firmware_ret_show, firmware_ret_store);
static DEVICE_ATTR(tkey_rawcounter, S_IRUGO | S_IWUSR | S_IWGRP, tkey_rawcounter_show, tkey_rawcounter_store);
#ifdef CONFIG_LTN_DTV
static DEVICE_ATTR(node_read, S_IRUGO, read_node, NULL) ;
#else
static DEVICE_ATTR(node_read, S_IRUGO, show_node, NULL) ;
#endif
static DEVICE_ATTR(debug_address	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, read_debug_address_silabs, NULL);
static DEVICE_ATTR(debug_address1	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, read_debug_address_silabs1, NULL);
static DEVICE_ATTR(reset	, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, tsp_reset_outJIGMode, NULL);


#ifdef CONFIG_HAS_EARLYSUSPEND
static void silabs_ts_early_suspend(struct early_suspend *h);
static void silabs_ts_late_resume(struct early_suspend *h);
#endif
 
static struct muti_touch_info g_Mtouch_info[SILABS_MAX_TOUCH];

void touch_ctrl_regulator(int on_off)
{
    int ret=0;

    if(touch_regulator == NULL)
    {
		touch_regulator = regulator_get(NULL,"touch_vcc");
    }
	
    if(on_off==TOUCH_ON)
    {
   		printk("[TSP] %s ON!\n", __func__);    
        regulator_set_voltage(touch_regulator,OUTPUT_2_8V,OUTPUT_2_8V);
        ret = regulator_enable(touch_regulator);
        if (ret) {
            printk(KERN_ERR "[TSP] regulator_touch enable failed\n");
        }
    }
    else
    {
   		printk("[TSP] %s OFF!\n", __func__);       
        ret = regulator_disable(touch_regulator);
        if (ret) {
            printk(KERN_ERR "[TSP] regulator_touch disable failed\n");
        }        
    }
}
EXPORT_SYMBOL(touch_ctrl_regulator);

int tsp_reset( void )
{
    int ret=1, key = 0;
	printk("[TSP] %s\n", __func__ );

    if(touch_check == 0)
    {
		printk("[TSP] %s start\n", __func__ );
      	mutex_lock(&reset_lock);

		   disable_irq(tsp_irq);

        touch_ctrl_regulator(TOUCH_OFF);
        gpio_direction_output(TSP_SCL,0);
        gpio_direction_output(TSP_SDA,0);
        gpio_direction_output(irq_to_gpio(tsp_irq),0);
        
        msleep(200);    

        TSP_forced_release_forkey(ts_global);
        
        gpio_direction_output(TSP_SCL,1);
        gpio_direction_output(TSP_SDA,1);
        gpio_direction_output(irq_to_gpio(tsp_irq),1);    
        gpio_direction_input(TSP_SCL);
        gpio_direction_input(TSP_SDA);
        gpio_direction_input(irq_to_gpio(tsp_irq));    
        touch_ctrl_regulator(TOUCH_ON);		
        msleep(200);

        enable_irq(tsp_irq);
      	mutex_unlock(&reset_lock);     
		printk("[TSP] %s end\n", __func__ );		
    }

    return ret;
}


int tsp_i2c_write (unsigned char *rbuf, int num)
{
    int ret;
    ret = i2c_master_send(ts_global->client, rbuf, num);

    if(ret<0) {
        printk("[TSP] %s Error\n", __func__ );
    }

    return ret;
}
EXPORT_SYMBOL(tsp_i2c_write);

int tsp_i2c_read(unsigned char *rbuf, int len)
{
    int ret;

    ret = i2c_master_recv(ts_global->client, rbuf, len);

    if(ret<0) {
        printk("[TSP] %s Error\n", __func__ );
    }

    return ret;
}
EXPORT_SYMBOL(tsp_i2c_read);

void set_tsp_for_ta_detect(int state)
{	
    int ret=0;
    uint8_t wdog_val[7] = {0x80, 0x05, 0x00, 0x02, 0x6a, 0x20, 0x00};
    
    if(testmode==0)
    {
        if(state)
        {
            printk("[TSP] set_tsp_for_ta_detect!!! attached\n");

            wdog_val[5] = 0x20;
            ret = i2c_master_send(ts_global->client, &wdog_val, 7);

            if(ret<0) {
				printk("[TSP] i2c_master_send failed\n");
            }

            pre_ta_stat = 1;
        }
        else
        {
            printk("[TSP] set_tsp_for_ta_detect!!! detached\n");

            wdog_val[5] = 0x00;
            ret = i2c_master_send(ts_global->client, &wdog_val, 7);

            if(ret<0) {
				printk("[TSP] i2c_master_send failed\n");
            }
            pre_ta_stat = 0;
        }
    }
} 

void TSP_forced_release_forkey(struct ts_data *ts)
{
    int i, key;
    int temp_value=0;

    for(i=0; i<SILABS_MAX_TOUCH; i++)
    {
#if defined(CONFIG_LTN_DTV)
        if(g_Mtouch_info[i].strength== -1)
            continue;

        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_id[i]);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0 );
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);       				
        input_mt_sync(ts->input_dev);   
         printk("[TSP] force release\n");
 
        if(g_Mtouch_info[i].strength == 0)
		g_Mtouch_info[i].strength = -1;   
        temp_value++;
#else
        if(touch_id[i] == 0)
            continue;

        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_id[i]);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, posX[i]);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, posY[i]);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0 );
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);      				
        input_mt_sync(ts->input_dev);   
        temp_value++;
        printk("[TSP] touch_id[%d] x(%d), y(%d) force release\n", touch_id[i], posX[i], posY[i]);
#endif
    }

    if(temp_value>0)
        input_sync(ts->input_dev);
#if 0
    for(key = 0; key < MAX_KEYS ; key++)
    {
    	touchkey_status[key].key_press = RELEASE_KEY;
    	input_report_key(ts_global->input_dev, touchkey_status[key].key_value, touchkey_status[key].key_press);	
    }  
#endif   
}
EXPORT_SYMBOL(TSP_forced_release_forkey);

static irqreturn_t  silabs_ts_work_func(int irq, void *dev_id)
{
    int ret = 0, i;
    uint8_t buf_temp[TS_READ_REGS_LEN];
    uint8_t buf[TS_READ_REGS_LEN];
    int touch_num=0, button_num =0, touchID=0, posX_value=0, posY_value=0, width = 0, reportID = 0, button_status=0, button_check=0;
    int keyID = 0;
    static int temp_count = 0;
    unsigned long flags;
    struct ts_data *ts = dev_id;

#if __TOUCH_DEBUG__
    printk("[TSP] %s\n", __func__);
#endif

    if(ts == NULL)
    {
        printk("[TSP] silabs_ts_work_func : TS NULL\n");
        touch_check = 0;
        
        return IRQ_HANDLED;
    }

    buf[0] = ESCAPE_ADDR;
    buf[1] = 0x02;

    for(i=0; i<I2C_RETRY_CNT; i++)
    {
        ret = i2c_master_send(ts->client, buf, 2);

        if(ret >=0)
        {
            ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);

            if(ret >=0)
            {
                break; // i2c success
            }
        }
    }

//    spin_lock_irqsave(&silabs_spin_lock, flags);
        
    if (ret < 0)
    {
        printk("[TSP] silabs_ts_work_func: i2c failed\n" );
//        enable_irq(ts->client->irq);
//        spin_unlock_irqrestore(&silabs_spin_lock, flags);
        touch_check = 0;

        return IRQ_HANDLED;
    }
    else 
    {
        touch_num  = buf[0]&0x0F;
        button_num = ((buf[0]&0xC0)>>6);
        button_status=((buf[1]&0x10)>>4);
        button_check=buf[1]&0x0F;

#if __TOUCH_DEBUG__
        printk("[TSP] button_num : %d, touch_num : %d, button_check:%d, buf[1] : %d\n", button_num, touch_num, button_check, buf[1]);
#endif

        if(button_check == 0)
        {
            if(touch_num >0) 
            {
                touch_id[0] = (buf[2]&0xf0)>>4;
                //posX[0] = (240-(( buf[3]<< (8) ) +  buf[4]));
                posX[0] = (( buf[3]<< (8) ) +  buf[4]);
                posY[0] = ( buf[5]<< (8) ) +  buf[6];
                strength[0] = buf[7]; 

                touch_id[1] = (buf[2]&0x0f);
                //posX[1] =  240-(( buf[8]<< (8) ) +  buf[9]);
                posX[1] =  (( buf[8]<< (8) ) +  buf[9]);
                posY[1] = ( buf[10]<< (8) ) +  buf[11];
                strength[1] = buf[12]; 
            }

            if(touch_num==0)
            {
                touch_id[0]=0;
                touch_id[1]=0;
                strength[0]=0;
                strength[1]=0;
            }
#if 0         
            g_Mtouch_info[touchID].posX= posX;
            g_Mtouch_info[touchID].posY= posY;
            g_Mtouch_info[touchID].width= width;			

            if(touchState)
                g_Mtouch_info[touchID].strength= strength;
            else
                g_Mtouch_info[touchID].strength = 0;
#endif
            for(i=0; i<2; i++)
            {
#if defined(CONFIG_LTN_DTV)
                if(touch_id[i] >=1)
                {
                     g_Mtouch_info[i].status = 1;
                }
                else if(touch_id[i] ==0 && g_Mtouch_info[i].status == 1)
                {
                     g_Mtouch_info[i].status = 0;
                }
                else if(touch_id[i] ==0 && g_Mtouch_info[i].status == 0)
                {
                     g_Mtouch_info[i].status = -1;
                }
				
                if(g_Mtouch_info[i].status == -1) continue;
#endif

                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_id[i]);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_X, posX[i]);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  posY[i]);
                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, touch_id[i] );
                input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, strength[i]);      				
                input_mt_sync(ts->input_dev);   

#if defined(CONFIG_LTN_DTV)
                g_Mtouch_info[i].posX = posX[i];
                g_Mtouch_info[i].posY = posY[i];
                g_Mtouch_info[i].width = strength[i];
                g_Mtouch_info[i].strength = strength[i];
#endif

#if __TOUCH_DEBUG__
                printk("[TSP] i : %d, x: %d, y: %d  3:%d, 4:%d\n", touch_id[i], posX[i], posY[i], strength[i], strength[i]);
	          
                if(IsdebugMatirx)
                     printk("[TSP] id(%d), x(%d), y(%d), z(%d)\n", touch_id[i], posX[i], posY[i], strength[i]);
#endif
            }
        }
        else
        {
            if (buf[1] & 0x1)
                input_report_key(ts->input_dev, KEY_MENU, button_status ? PRESS_KEY : RELEASE_KEY);		
            if (buf[1] & 0x2)
                input_report_key(ts->input_dev, KEY_BACK, button_status ? PRESS_KEY : RELEASE_KEY);
            if (buf[1] & 0x4)
                input_report_key(ts->input_dev, KEY_BACK, button_status ? PRESS_KEY : RELEASE_KEY);
            if (buf[1] & 0x8)
                input_report_key(ts->input_dev, KEY_SEARCH, button_status ? PRESS_KEY : RELEASE_KEY);			
#if __TOUCH_DEBUG__
            printk(KERN_ERR "melfas_ts_work_func: buf[1] : %d, button_status: %d\n", buf[1], button_status ? PRESS_KEY : RELEASE_KEY);
#endif		
        }
        input_sync(ts->input_dev);              
    }

//    enable_irq(ts->client->irq);
//    spin_unlock_irqrestore(&silabs_spin_lock, flags);
    touch_check = 0;

    return IRQ_HANDLED;
}

static irqreturn_t silabs_ts_irq_handler(int irq, void *dev_id)
{
    struct ts_data *ts = dev_id;

#if __TOUCH_DEBUG__
    printk("[TSP] %s\n", __func__);
#endif
    touch_check = 1;

//    disable_irq_nosync(ts->client->irq);
//    queue_work(silabs_wq, &ts->work);

    return IRQ_WAKE_THREAD;
}

static void check_ic_work_func(struct work_struct *work_timer)
{
    int ret=0;
    uint8_t buf_esd[2];
    uint8_t i2c_addr = 0x1F;
    uint8_t wdog_val[1];

    struct ts_data *ts = container_of(work_timer, struct ts_data, work_timer);

//    touch_check = 0;
    
    buf_esd[0] = ESCAPE_ADDR;
    buf_esd[1] = TS_READ_ESD_ADDR;

    wdog_val[0] = 1;

    if(touch_check == 0)
    {
#if defined(CONFIG_LTN_DTV)
	if( pre_ta_stat != tsp_charger_type_status )
	{
		set_tsp_for_ta_detect(tsp_charger_type_status);
	}
#else
	if( pre_ta_stat != charger_status_forTSP )
	{
		set_tsp_for_ta_detect(charger_status_forTSP);
	}
#endif    
        
        ret = i2c_master_send(ts->client, &buf_esd, 2);
        
            if(touch_check == 0)
            {    
                if(ret >=0)
                {
                    ret = i2c_master_recv(ts->client, wdog_val, 1);
                }
                        
                if(ret < 0)
                {
                     printk(KERN_ERR "silabs_ts_work_func : i2c_master_send failed\n");			
                    tsp_reset();                    
                }
                else if((touch_check == 0) && (wdog_val[0] == (uint8_t)prev_wdog_val))
                {
                   // printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
                    tsp_reset();
                    prev_wdog_val = -1;
                }
                else
                {
                   //printk("[TSP] %s counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
                    prev_wdog_val = wdog_val[0];
                }
          // check_ic_counter = 3;	
        }
    }
/*   
    else
    {
        check_ic_counter--;
    }
 */
}

static enum hrtimer_restart silabs_watchdog_timer_func(struct hrtimer *timer)
{
    queue_work(check_ic_wq, &ts_global->work_timer);
    
    hrtimer_start(&ts_global->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);
//    hrtimer_start(&ts_global->timer, ktime_set(2, 0), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static unsigned int touch_present = 0;

static int silabs_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ts_data *ts;
    int ret = 0; 
    uint8_t buf[TS_READ_REGS_LEN];   
#if !defined(CONFIG_LTN_DTV)
	uint8_t buf_firmware[3];
#endif
    int i=0;
    char* productionMode = "androidboot.bsp=2";
    char * checkMode = NULL;
#if defined(CONFIG_LTN_DTV)
	int first_check_version, second_check_version;
#endif	
    printk("[TSP] %s\n", __func__ );

#ifdef CONFIG_LTN_DTV
    touch_ctrl_regulator(TOUCH_ON);
    mdelay(200);  
    touch_ctrl_regulator(TOUCH_OFF);
    mdelay(200);      
    touch_ctrl_regulator(TOUCH_ON);
    mdelay(200);
#else
    tsp_reset();
#endif
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL) {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }
    ts->client = client;
    i2c_set_clientdata(client, ts);

    ts_global = ts;

    tsp_irq=client->irq;

//    INIT_WORK(&ts->work, silabs_ts_work_func);
    INIT_WORK(&ts->work_timer, check_ic_work_func );

    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = silabs_watchdog_timer_func;

#ifdef CONFIG_LTN_DTV
/* sys fs */
touch_class = class_create(THIS_MODULE, "touch");
if (IS_ERR(touch_class))
	pr_err("Failed to create class(touch)!\n");
firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
if (IS_ERR(firmware_dev))
	pr_err("Failed to create device(firmware)!\n");

if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ver.attr.name);
if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_versname) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_versname.attr.name);
  if (device_create_file(firmware_dev, &dev_attr_raw) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_reference) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_reference.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_diff) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_diff.attr.name); 	
 if (device_create_file(firmware_dev, &dev_attr_raw_value) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_raw_value.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_node_read) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_node_read.attr.name);	
 if (device_create_file(firmware_dev, &dev_attr_raw_enable) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_raw_enable.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_raw_disable) < 0)
	pr_err("Failed to create device file(%s)!\n", dev_attr_raw_disable.attr.name);


#else
touch_class = class_create(THIS_MODULE, "touch");
 if (IS_ERR(touch_class))
	 pr_err("Failed to create class(touch)!\n");
 firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
 if (IS_ERR(firmware_dev))
	 pr_err("Failed to create device(firmware)!\n");
 if (device_create_file(firmware_dev, &dev_attr_firmware_ver) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ver.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_sw_ver) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_sw_ver.attr.name);	  
 if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_tkey_rawcounter) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_rawcounter.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_node_read) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_node_read.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_tsp_module) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_module.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_tsp_reference) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_reference.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_tsp_difference) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_difference.attr.name);	  
 if (device_create_file(firmware_dev, &dev_attr_tsp_xy) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_xy.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_debug_address) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_debug_address.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_debug_address1) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_debug_address1.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_reset) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_reset.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_tsp_StartCall) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_StartCall.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_tsp_EndCall) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_EndCall.attr.name);
 if (device_create_file(firmware_dev, &dev_attr_debug_matrix) < 0)
	 pr_err("Failed to create device file(%s)!\n", dev_attr_debug_matrix.attr.name);

#endif    
    buf_firmware[0] = ESCAPE_ADDR;
    buf_firmware[1] = TS_READ_VERSION_ADDR;
    ret = i2c_master_send(ts->client, &buf_firmware, 2);
    if(ret < 0)
    {
        printk(KERN_ERR "silabs_ts_probe : i2c_master_send failed\n");			
    }

    ret = i2c_master_recv(ts->client, &buf_firmware, 3);
    if(ret < 0)
    {
        printk(KERN_ERR "silabs_ts_probe : i2c_master_recv failed\n");			
    }
#ifdef CONFIG_LTN_DTV
    printk("[TSP] silabs_ts_probe SW=%d, ver tsp=%d, HW=%d\n", buf_firmware[0], buf_firmware[1], buf_firmware[2]);
#else
    printk("[TSP] silabs_ts_probe %d, %d, %d\n", buf_firmware[0], buf_firmware[1], buf_firmware[2]);
#endif    

#ifdef CONFIG_LTN_DTV
	checkMode=1;   // 1 : Disable TSP auto-update /  0: Enable TSP auto-update
#else
    checkMode = strstr(saved_command_line, productionMode);
#endif    

#if 0 // Block the TSP-auto update
#ifdef CONFIG_LTN_DTV  // HW request for TSP firmware update
	first_check_version = buf_firmware[0] ;
    if(first_check_version == 18 ||first_check_version == 19) // Only previous version 18 or 19 TSP auto-update
    {
		printk(KERN_ERR "[TSP]silabs_ts_probe : first_check_version : %d\n", first_check_version);			
		msleep(500); // Timing delay for rechecking the TSP version.
	
		buf_firmware[0] = ESCAPE_ADDR;
		buf_firmware[1] = TS_READ_VERSION_ADDR;
		ret = i2c_master_send(ts->client, &buf_firmware, 2);
		if(ret < 0)
		{
			printk(KERN_ERR "silabs_ts_probe : i2c_master_send failed\n");			
		}

		ret = i2c_master_recv(ts->client, &buf_firmware, 3);
		if(ret < 0)
		{
			printk(KERN_ERR "silabs_ts_probe : i2c_master_recv failed\n");			
		}

		second_check_version = buf_firmware[0] ;
		printk(KERN_ERR "[TSP]silabs_ts_probe : second_check_version : %d\n", second_check_version);			

		if(first_check_version == second_check_version)
		{
			local_irq_disable();
			printk(KERN_ERR "[TSP]silabs_ts_probe : TSP auto-update\n");			
			ret = Firmware_Download();  
			local_irq_enable();
		}
	}
#else
    if(buf_firmware[0] != FW_VER && checkMode == NULL)
    {
        local_irq_disable();
        ret = Firmware_Download();	
  //      printk("[TSP] enable_irq\n");
        local_irq_enable();
    }		
#endif

    if(ret == 0)
    {
        printk(KERN_ERR "SET Download Failed\n");			
    }
#endif

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        touch_present = 0;
        printk(KERN_ERR "silabs_ts_probe: Failed to allocate input device\n");
        goto err_input_dev_alloc_failed;
    }
    ts->input_dev->name = "sec_touchscreen ";
    ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
    ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
    ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
    ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);		
    ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);	
#if defined(CONFIG_LTN_DTV)
	ts->input_dev->keybit[BIT_WORD(KEY_POWER)] |= BIT_MASK(KEY_POWER);
#endif	
    set_bit(BTN_TOUCH, ts->input_dev->keybit);
    set_bit(EV_ABS,  ts->input_dev->evbit);
    ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);	

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

    set_bit(EV_SYN, ts->input_dev->evbit); 
    set_bit(EV_KEY, ts->input_dev->evbit);	

    /* ts->input_dev->name = ts->keypad_info->name; */
    ret = input_register_device(ts->input_dev);
    if (ret) {
        touch_present = 0;
        printk(KERN_ERR "silabs_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
        goto err_input_register_device_failed;
    }

   // printk("[TSP] %s, irq=%d\n", __func__, client->irq );

    ret = gpio_request(irq_to_gpio(tsp_irq), "ts_irq");
	if(ret < 0)
	{
		printk("[TSP] failed to gpio request for ts_irq\n");
	}
    gpio_direction_input(irq_to_gpio(tsp_irq));
    if (client->irq) 
    {
    ret = request_threaded_irq(client->irq, silabs_ts_irq_handler,  silabs_ts_work_func, IRQF_TRIGGER_FALLING|IRQF_ONESHOT, client->name, ts);

    if (ret < 0)
        dev_err(&client->dev, "request_irq failed\n");
    else 
        ts->use_irq = 1;

    }

#if 0

    if (client->irq) {
        ret = request_irq(client->irq, silabs_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);

        if (ret == 0)
            ts->use_irq = 1;
        else
            dev_err(&client->dev, "request_irq failed\n");
    }
#endif    

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = silabs_ts_early_suspend;
    ts->early_suspend.resume = silabs_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif


    touch_present = 1;
#if defined(CONFIG_LTN_DTV)
    hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);
#else
    hrtimer_start(&ts->timer, ktime_set(5, 0), HRTIMER_MODE_REL);
#endif

    return 0;

err_input_register_device_failed:
    printk(KERN_ERR "silabs-ts: err_input_register_device failed\n");
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    printk(KERN_ERR "silabs-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
    printk(KERN_ERR "silabs-ts: err_alloc_data failed_\n");	
err_check_functionality_failed:    
    return ret;
}

static int silabs_ts_remove(struct i2c_client *client)
{
    struct ts_data *ts = i2c_get_clientdata(client);
    unregister_early_suspend(&ts->early_suspend);
    if (ts->use_irq)
        free_irq(client->irq, ts);
    //else
    //	hrtimer_cancel(&ts->timer);
    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

static int silabs_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct ts_data *ts = i2c_get_clientdata(client);

    printk("[TSP] %s+\n", __func__);

    if( touch_present )
    {
        if (ts->use_irq)
        {
            disable_irq(client->irq);
        }
        ret = cancel_work_sync(&ts->work_timer);

        ret = cancel_work_sync(&ts->work);
        if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
        {
            enable_irq(client->irq);
        }

        hrtimer_cancel(&ts->timer);
        if(IsDuringCall == 0)
        {
        touch_ctrl_regulator(TOUCH_OFF);
        msleep(400);    
        gpio_direction_output(TSP_SCL,1);
        gpio_direction_output(TSP_SDA,1);
        gpio_direction_output(irq_to_gpio(tsp_irq),0);        
    }
    }
    else
        printk("[TSP] TSP isn't present.\n");

    TSP_forced_release_forkey(ts);
    printk("[TSP] %s-\n", __func__ );
    return 0;
}

static int silabs_ts_resume(struct i2c_client *client)
{
    int ret, key, retry_count;
    struct ts_data *ts = i2c_get_clientdata(client);

    printk("[TSP] %s+\n", __func__ );
    if( touch_present )
    {
       if(regulator_is_enabled(touch_regulator) == 0)
        {
        gpio_direction_output(TSP_SCL,1);
        gpio_direction_output(TSP_SDA,1);
        gpio_direction_output(irq_to_gpio(tsp_irq),1);    
        gpio_direction_input(TSP_SCL);
        gpio_direction_input(TSP_SDA);
        gpio_direction_input(irq_to_gpio(tsp_irq));            
        touch_ctrl_regulator(TOUCH_ON);
        msleep(40);
        }
#if 1 
        else if(IsDuringCall == 1)
        {
            touch_ctrl_regulator(TOUCH_OFF);
            msleep(400);    
            gpio_direction_output(TSP_SCL,1);
            gpio_direction_output(TSP_SDA,1);
            gpio_direction_output(irq_to_gpio(tsp_irq),0);    

            gpio_direction_output(TSP_SCL,1);
            gpio_direction_output(TSP_SDA,1);
            gpio_direction_output(irq_to_gpio(tsp_irq),1);    
            gpio_direction_input(TSP_SCL);
            gpio_direction_input(TSP_SDA);
            gpio_direction_input(irq_to_gpio(tsp_irq));            
            touch_ctrl_regulator(TOUCH_ON);
            msleep(40);
        }
#endif
        prev_wdog_val = -1;
        hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);
//        hrtimer_start(&ts->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);
#if defined(CONFIG_LTN_DTV)
	if(tsp_charger_type_status == 1)
	{
		set_tsp_for_ta_detect(tsp_charger_type_status);
	}
#else
	if(charger_status_forTSP == 1)
	{
		set_tsp_for_ta_detect(charger_status_forTSP);
	}
#endif
        enable_irq(client->irq);
    }
    else
        printk("[TSP] TSP isn't present.\n");

    printk("[TSP] %s-\n", __func__ );

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void silabs_ts_early_suspend(struct early_suspend *h)
{
    struct ts_data *ts;
    ts = container_of(h, struct ts_data, early_suspend);
    silabs_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void silabs_ts_late_resume(struct early_suspend *h)
{
    struct ts_data *ts;
    ts = container_of(h, struct ts_data, early_suspend);
    silabs_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id silabs_ts_id[] = {
	{ SILABS_TS_NAME, 0 },
	{ }
};

static struct i2c_driver silabs_ts_driver = {
	.probe		= silabs_ts_probe,
	.remove		= silabs_ts_remove,
#if 1
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	       = silabs_ts_suspend,
	.resume		= silabs_ts_resume,
#endif
#endif
	.id_table	= silabs_ts_id,
	.driver = {
		.name	= SILABS_TS_NAME,
	},
};

static int __devinit silabs_ts_init(void)
{
    printk("[TSP] %s\n",__func__);

 //   silabs_wq = create_workqueue("silabs_wq");
 //   if (!silabs_wq)
 //       return -ENOMEM;
 	mutex_init(&reset_lock);

    	check_ic_wq = create_singlethread_workqueue("check_ic_wq");	
	if (!check_ic_wq)
		return -ENOMEM;

#ifdef CONFIG_LTN_DTV
	touch_regulator = regulator_get(NULL,"touch_vcc");
	board_sysconfig(SYSCFG_TOUCH, SYSCFG_INIT);

#endif
    
    return i2c_add_driver(&silabs_ts_driver);
}

static void __exit silabs_ts_exit(void)
{
#ifdef CONFIG_LTN_DTV
        if (touch_regulator) 
        {
        regulator_put(touch_regulator);
        touch_regulator = NULL;
        }
#endif
    i2c_del_driver(&silabs_ts_driver);
 //   if (silabs_wq)
 //       destroy_workqueue(silabs_wq);

    if (check_ic_wq)
        destroy_workqueue(check_ic_wq);
}

static ssize_t firmware_sw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;

    printk("[TSP] %s ver firmware=[%d]\n",__func__, FW_VER);
    return sprintf(buf, "%d\n", FW_VER);
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_LTN_DTV
    uint8_t buf_firmware[3];
#else
	uint8_t buf_firmware[2];
#endif
   int ret;

    printk("[TSP] %s\n",__func__);

    buf_firmware[0] = ESCAPE_ADDR;
    buf_firmware[1] = TS_READ_VERSION_ADDR;
    ret = i2c_master_send(ts_global->client, &buf_firmware, 2);
    if(ret < 0)
    {
        printk(KERN_ERR "firmware_show : i2c_master_send failed\n");			
    }

#ifdef CONFIG_LTN_DTV
    ret = i2c_master_recv(ts_global->client, &buf_firmware, 3);
#else
    ret = i2c_master_recv(ts_global->client, &buf_firmware, 1);
#endif
    if(ret < 0)
    {
        printk(KERN_ERR "firmware_show : i2c_master_recv failed\n");			
    }

#ifdef CONFIG_LTN_DTV
    PHONE_VER = FW_VER;
    	sprintf(buf, "10%x%02d%02d\n", buf_firmware[2], buf_firmware[0], PHONE_VER);
	return sprintf(buf, "%s", buf );
#else
    printk("[TSP] ver firmware=[%d]\n", buf_firmware[0]);
    return sprintf(buf, "%d\n", buf_firmware[0]);
#endif
}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
    char *after;

    unsigned long value = simple_strtoul(buf, &after, 10);	
    printk(KERN_INFO "[TSP] %s\n", __func__);
    firmware_ret_val = -1;
   // printk("[TSP] firmware_store  valuie : %d\n",value);
    if ( value == 0 )
    {
        printk("[TSP] Firmware update start!!\n" );

        firm_update( );
        return size;
    }

    return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
    printk("[TSP] %s!\n", __func__);
#ifdef CONFIG_LTN_DTV
	firm_update( );
#endif
	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
    printk("[TSP] %s, operate nothing!\n", __func__);

    return size;
}

int silabs_quicksense ( int address, int size, char* buff )
{
    uint8_t buf1[7] = {0x78, 0x05, 0x00, 0, 0, 0, 0};
    int ret;

    buf1[3] = (address >> 8) & 0xFF; // Address High Byte
    buf1[4] = address & 0xFF; // Address Low Byte
    buf1[5] = size;

    ret = i2c_master_send(ts_global->client, buf1, 7);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }

    ret = i2c_master_recv(ts_global->client, buff, size + QUICKSENSE_OVERHEAD);

    if (ret < 0)
    {
        printk("[TSP] i2c_master_recv fail! %s\n", __func__);
        return -1;
    }

    return 0;
}
#ifdef CONFIG_LTN_DTV
static ssize_t rawdata_show_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
      int  written_bytes = 0 ;  /* & error check */
#if defined(CONFIG_LTN_DTV)	  
	uint8_t buffer_temp[NUM_TX_CHANNEL  *NUM_RX_CHANNEL + QUICKSENSE_OVERHEAD] = {0,};
#endif
	uint8_t buffer1[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
    	uint8_t buffer2[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_rawdata[NUM_MTRBUTTONS]={0,};
	uint16_t baseline[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
      int i, j, ret;

      printk("[TSP] %s entered. \n", __func__);

	if(testmode==1) return 0;   

	mdelay(300); 

	buffer1[0] = ESCAPE_ADDR;
	buffer1[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer1, 2);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_send fail! %s\n", __func__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer2, 1);

	if (ret < 0)
	{
		printk("[TSP] i2c_master_recv fail! %s\n", __func__);
		return -1;
	}
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

	//
	//	quicksense format for reading baseline
	//
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(BASELINE_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);
		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer1[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}
#else	
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer1);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return -1;
	}	
#endif	
	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
#if defined(CONFIG_LTN_DTV)	
			baseline[i][j] = (buffer1[(i*Rx_Channel+j)*2] <<8) + buffer1[(i*Rx_Channel+j)*2+1];
#else
			baseline[i][j] = (buffer1[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer1[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
		}
	}

	//
	//	quicksense format for reading rawdata
	//
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(RAWDATA_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);
		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer2[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}
#else		
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer2);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return -1;
	}
#endif	
	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
        {
#if defined(CONFIG_LTN_DTV)	
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2] <<8) + buffer2[(i*Rx_Channel+j)*2+1];
#else
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer2[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
			written_bytes += sprintf(buf+written_bytes, "%d %d\n", rawdata[i][j], baseline[i][j]-rawdata[i][j]) ;
        }
    }

    if (written_bytes > 0)
        return written_bytes ;

    return sprintf(buf, "-1") ;
 }


/* Touch Reference ************************************************************/
static ssize_t raw_enable_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{

		printk("[TSP] %s stop. \n", __func__);

            testmode=0;

    return sprintf(buf, "1") ;
}

static ssize_t raw_disable_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{

		printk("[TSP] %s stop\n", __func__);

            testmode=1;
        	touch_ctrl_regulator(0);  
        	mdelay(2);
        	touch_ctrl_regulator(1);  
        	mdelay(300);


    return sprintf(buf, "1") ;
}

static ssize_t baseline_show_silabs(struct device *dev, struct device_attribute *attr, char *buf1)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
	int written_bytes = 0 ;  /* & error check */
#if defined(CONFIG_LTN_DTV)	
	uint8_t buffer_temp[NUM_TX_CHANNEL*NUM_RX_CHANNEL+QUICKSENSE_OVERHEAD]={0,};
#endif
	uint8_t buffer[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t baseline[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
    int i, j, ret;


	if(testmode==1) return 0;
    
    printk("[TSP] %s entered. \n", __func__);

   mdelay(300); 

	//
	//	Entering JIG_MODE
	//
	buffer[0] = ESCAPE_ADDR;
	buffer[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer, 2);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_send fail! %s\n", __func__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer, 1);

	if (ret < 0)
	{
		printk("[TSP] i2c_master_recv fail! %s\n", __func__);
		return -1;
	}
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

	//
	//	quicksense format for reading baseline
	//
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(BASELINE_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);
		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}
#else		
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return -1;
	}	
#endif

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
#if defined(CONFIG_LTN_DTV)	
			baseline[i][j] = (buffer[(i*Rx_Channel+j)*2] <<8) + buffer[(i*Rx_Channel+j)*2+1];
#else
			baseline[i][j] = (buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
		//	printk(" %5d", baseline[i][j]);
			
			written_bytes += sprintf(buf1+written_bytes, "%d\n", baseline[i][j]) ;
		}
	//	printk("\n");
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf1, "-1") ;
}


static ssize_t diff_show_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
#if defined(CONFIG_LTN_DTV)	
	uint8_t buffer_temp[NUM_TX_CHANNEL*NUM_RX_CHANNEL+QUICKSENSE_OVERHEAD]={0,};
#endif
	uint8_t buffer1[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
    uint8_t buffer2[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_rawdata[NUM_MTRBUTTONS]={0,};
	uint16_t baseline[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
      int i, j, ret;

      printk("[TSP] %s entered\n", __func__);

	if(testmode==1) return 0;   

	mdelay(300); 

	buffer1[0] = ESCAPE_ADDR;
	buffer1[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer1, 2);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_send fail! %s\n", __func__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client, buffer1, 1);

	if (ret < 0)
	{
		printk("[TSP] i2c_master_recv fail! %s\n", __func__);
		return -1;
	}
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

	//
	//	quicksense format for reading baseline
	//
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(BASELINE_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);
		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer1[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}
#else		
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer1);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return -1;
	}	
#endif

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
#if defined(CONFIG_LTN_DTV)	
			baseline[i][j] = (buffer1[(i*Rx_Channel+j)*2] <<8) + buffer1[(i*Rx_Channel+j)*2+1];
#else		
			baseline[i][j] = (buffer1[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer1[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
			//printk(" %5d", baseline[i][j]);
		}
	}

	//
	//	quicksense format for reading rawdata
	//
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(RAWDATA_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);
		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer2[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}
#else	
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer2);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return -1;
	}
#endif

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
        {
#if defined(CONFIG_LTN_DTV)	
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2] <<8) + buffer2[(i*Rx_Channel+j)*2+1];
#else        
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer2[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
	//		printk(" %5d", baseline[i][j]-rawdata[i][j]);
        }
//	printk("\n");
    }

    return 0;
 }

static ssize_t fimware_show_versname(struct device *dev, struct device_attribute *attr, char *buf)
{
       uint8_t buf_firmware_ver[3];
       int ret;
       
	printk("[TSP] %s\n",__func__);

	buf_firmware_ver[0] = ESCAPE_ADDR;
	buf_firmware_ver[1] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts_global->client, &buf_firmware_ver, 2);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send failed\n");			
	}

	ret = i2c_master_recv(ts_global->client, &buf_firmware_ver, 3);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv failed\n");			
	}
	printk("[TSP] ver tsp=%x, HW=%x, SW=%x\n", buf_firmware_ver[1], buf_firmware_ver[2], buf_firmware_ver[0]);

    	sprintf(buf, "%x\n", buf_firmware_ver[0]);
   //    printk("[TSP] %s\n", buf);
       
	return sprintf(buf, "%s", buf );
}


static ssize_t rawdata_pass_fail_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
#if defined(CONFIG_LTN_DTV)	
	uint8_t buffer_temp[NUM_TX_CHANNEL*NUM_RX_CHANNEL+QUICKSENSE_OVERHEAD]={0,};
#endif
	uint8_t buffer1[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
    uint8_t buffer2[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
#if defined(CONFIG_LTN_DTV)   
	uint16_t RAWDATA_MAX[130] = {14253,14231,14305,14367,14400,14380,14389,14413,14384,14176,13608,13861,13958,14067,14089,14073,14062,14061,14013,13748,13527,13818,13929,14038,14067,14051,14037,14040,13985,13705,13477,13776,13897,14005,14044,14029,14017,14025,13966,13685,13456,13748,13874,13985,14017,14007,13999,14006,13951,13672,13504,13730,13844,13954,13993,13978,13967,13975,13931,13651,13273,13669,13801,13920,13963,13962,13967,14007,14043,14450,13250,13645,13790,13901,13941,13933,13938,13975,13982,13964,13241,13625,13778,13893,13933,13920,13930,13966,13956,13866,13233,13617,13761,13872,13914,13906,13909,13940,13931,13812,13215,13587,13728,13845,13884,13870,13870,13898,13881,13754,13559,13594,13708,13808,13849,13837,13832,13862,13851,13717,13755,13883,13991,14091,14127,14113,14111,14150,14139,14028};
	uint16_t RAWDATA_MIN[130] = {10535,10519,10573,10619,10644,10628,10635,10653,10632,10478,10058,10245,10316,10397,10413,10401,10394,10393,10357,10162,9999,10214,10295,10376,10397,10385,10375,10378,10337,10129,9961,10182,10271,10351,10380,10369,10361,10367,10322,10115,9946,10162,10254,10337,10361,10353,10347,10352,10311,10106,9982,10148,10232,10314,10343,10332,10323,10329,10297,10090,9811,10103,10201,10288,10321,10320,10323,10353,10379,10680,9794,10085,10192,10275,10305,10299,10302,10329,10334,10322,9787,10071,10184,10269,10299,10288,10296,10322,10316,10248,9781,10065,10171,10254,10284,10278,10281,10304,10297,10209,9767,10043,10146,10233,10262,10252,10252,10272,10260,10166,10022,10048,10132,10206,10237,10227,10224,10246,10237,10139,10167,10261,10341,10415,10441,10431,10430,10458,10451,10368};
#elif defined(CONFIG_TARGET_LOCALE_AUS_TEL)   
	uint16_t RAWDATA_MAX[108] = {15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000};
//	uint16_t RAWDATA_MAX[108] = {14650,14610,14660,14610,14710,14680,14780,14780,14640,14640,14640,14640,14150,14190,14190,14260,14250,13910,13650,13940,14040,14110,14150,14150,14210,14200,13870,13610,13890,14000,14080,14120,14120,14200,14170,13840,13570,13870,13960,14040,14080,14080,14160,14150,13820,13560,13830,13920,13990,14020,14020,14110,14120,13790,13540,13800,13900,13960,14000,14000,14090,14100,13770,13400,13760,13900,13980,14030,14040,14150,14200,14270,13380,13750,13890,13980,14020,14040,14140,14160,14050,13370,13730,13870,13960,14010,14010,14100,14130,13980,13440,13720,13860,13960,14000,14000,14100,14130,14180,14150,14560,14730,14830,14870,14870,14960,15000,15010};
//	uint16_t RAWDATA_MIN[108] = {10610,10720,10770,10800,10870,10850,10930,10920,10680,10140,10340,10410,10460,10490,10480,10540,10530,10280,10090,10300,10370,10430,10460,10460,10500,10500,10250,10060,10270,10340,10410,10440,10440,10490,10470,10230,10030,10250,10320,10380,10400,10410,10460,10460,10210,10020,10220,10290,10340,10360,10360,10430,10440,10190,10010,10200,10270,10320,10350,10350,10410,10420,10180,9900,10170,10270,10330,10370,10380,10460,10500,10540,9890,10160,10270,10330,10360,10370,10450,10470,10390,9880,10150,10250,10320,10350,10360,10420,10440,10330,9930,10140,10250,10310,10350,10350,10420,10450,10480,10460,10760,10880,10960,10990,10990,11060,11090,11100};
	uint16_t RAWDATA_MIN[108] = {9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000,9000};
#else
	uint16_t RAWDATA_MAX[108] = {14350,14510,14560,14610,14710,14680,14780,14780,14440,13720,13990,14080,14150,14190,14190,14260,14250,13910,13650,13940,14040,14110,14150,14150,14210,14200,13870,13610,13890,14000,14080,14120,14120,14200,14170,13840,13570,13870,13960,14040,14080,14080,14160,14150,13820,13560,13830,13920,13990,14020,14020,14110,14120,13790,13540,13800,13900,13960,14000,14000,14090,14100,13770,13400,13760,13900,13980,14030,14040,14150,14200,14270,13380,13750,13890,13980,14020,14040,14140,14160,14050,13370,13730,13870,13960,14010,14010,14100,14130,13980,13440,13720,13860,13960,14000,14000,14100,14130,14180,14150,14560,14730,14830,14870,14870,14960,15000,15010};
	uint16_t RAWDATA_MIN[108] = {10610,10720,10770,10800,10870,10850,10930,10920,10680,10140,10340,10410,10460,10490,10480,10540,10530,10280,10090,10300,10370,10430,10460,10460,10500,10500,10250,10060,10270,10340,10410,10440,10440,10490,10470,10230,10030,10250,10320,10380,10400,10410,10460,10460,10210,10020,10220,10290,10340,10360,10360,10430,10440,10190,10010,10200,10270,10320,10350,10350,10410,10420,10180,9900,10170,10270,10330,10370,10380,10460,10500,10540,9890,10160,10270,10330,10360,10370,10450,10470,10390,9880,10150,10250,10320,10350,10360,10420,10440,10330,9930,10140,10250,10310,10350,10350,10420,10450,10480,10460,10760,10880,10960,10990,10990,11060,11090,11100};
#endif  

	uint8_t buf_firmware_show[3];

    int i, j, ret;

    printk("[TSP] %s entered\n", __func__);

	if(testmode==1) return sprintf(buf, "-1");   

	mdelay(300); 

	buffer1[0] = ESCAPE_ADDR;
	buffer1[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer1, 2);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_send fail! %s\n", __func__);
		return sprintf(buf, "-1");
	}

	ret = i2c_master_recv(ts_global->client,buffer2, 1);

	if (ret < 0)
	{
		printk("[TSP] i2c_master_recv fail! %s\n", __func__);
		return sprintf(buf, "-1");
	}
#if defined(CONFIG_LTN_DTV)
	mdelay(200); 
#endif

	//
	//	quicksense format for reading rawdata
	//
#if 0//raw data check delete
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(RAWDATA_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);

		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer2[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}	
#else	
#if defined(CONFIG_TARGET_LOCALE_AUS_TEL)   
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer2);
#else
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_RX_CHANNEL*2, buffer2);
#endif
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return sprintf(buf, "-1");
	}
#endif
	
#if defined(CONFIG_TARGET_LOCALE_AUS_TEL) || defined(CONFIG_LTN_DTV)   
	for (i = 0; i < Tx_Channel; i++)
#else
	for (i = 0; i < 1; i++)
#endif
    {
		for(j = 0 ; j < Rx_Channel; j++)
      {
#if defined(CONFIG_LTN_DTV)	
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2] <<8) + buffer2[(i*Rx_Channel+j)*2+1];
#else      
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer2[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
//			printk("[TSP] silabs_quicksense rawdata : %5d / %5d / %5d  \n", RAWDATA_MIN[i*Rx_Channel+j], rawdata[i][j],RAWDATA_MAX[i*Rx_Channel+j]);
			if( RAWDATA_MAX[i*Rx_Channel+j] < rawdata[i][j])
         {
#if defined(CONFIG_TARGET_LOCALE_AUS_TEL)           
//   		   printk("[TSP] rawdata_pass_fail_silabs MAX rawdata[%d][%d] = %d \n", i,j,rawdata[i][j]);            
#endif
            return sprintf(buf, "0"); // fail
			}
			if( RAWDATA_MIN[i*Rx_Channel+j] > rawdata[i][j])
         {
#if defined(CONFIG_TARGET_LOCALE_AUS_TEL)           
 //  		   printk("[TSP] rawdata_pass_fail_silabs MIN rawdata[%d][%d] = %d \n", i,j,rawdata[i][j]);            
#endif
            return sprintf(buf, "0"); // fail
			}
		}
    }
#endif //raw data check delete


	buf_firmware_show[0] = ESCAPE_ADDR;
	buf_firmware_show[1] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts_global->client, &buf_firmware_show, 2);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send failed\n");			
	}

	ret = i2c_master_recv(ts_global->client, &buf_firmware_show, 3);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv failed\n");			
	}
	printk("[TSP] ver tsp=%d, HW=%d, SW=%d\n", buf_firmware_show[1], buf_firmware_show[2], buf_firmware_show[0]);
#if defined(CONFIG_TARGET_LOCALE_AUS_TEL)
     if (buf_firmware[2] == T_YTE_MODULE_VER_NEW)
            PHONE_VER = T_FW_VER_NEW_YTE;
     else
#endif

#ifdef CONFIG_LTN_DTV
	PHONE_VER = FW_VER;
#else
     if (( buf_firmware_show[2] == YTE_MODULE_VER)||( buf_firmware_show[2] == SMAC_MODULE_VER))
             PHONE_VER = FW_VER;

    else if ( buf_firmware_show[2] == YTE_MODULE_VER_OLD)
             PHONE_VER = FW_VER_OLD;

    else if ( buf_firmware_show[2] == SMAC_MODULE_VER_OLD)
			PHONE_VER = FW_VER_OLD_SMAC;

    else if ( buf_firmware_show[2] == YTE_MODULE_VER_NEW)
            PHONE_VER = FW_VER_NEW_YTE;

    else if (buf_firmware_show[2] == SMAC_MODULE_VER_NEW)
			PHONE_VER = FW_VER_NEW_SMAC;
#endif

	if(buf_firmware_show[0]!=PHONE_VER)
		return sprintf(buf, "0");

    return sprintf(buf, "1"); // success
 }



static ssize_t read_node(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *after;


	int written_bytes = 0 ;  /* & error check */
#if defined(CONFIG_LTN_DTV)	
	uint8_t buffer_temp[NUM_TX_CHANNEL*NUM_RX_CHANNEL+QUICKSENSE_OVERHEAD]={0,};
#endif
	uint8_t buffer[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
       int i, j, ret;
       
	check_node = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __func__);

       mdelay(300); 
  
	//
	//	Entering JIG_MODE
	//
	buffer[0] = ESCAPE_ADDR;
	buffer[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer, 2);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_send fail! %s\n", __func__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer, 1);

	if (ret < 0)
	{
		printk("[TSP] i2c_master_recv fail! %s\n", __func__);
		return -1;
	}
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

	//
	//	quicksense format for reading baseline
	//
#if defined(CONFIG_LTN_DTV)	
	for (i = 0; i < 2; i++)
	{
		memset(buffer_temp, 0x00, sizeof(buffer_temp));
		ret = silabs_quicksense(BASELINE_ADDRESS + i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL), NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer_temp);
		for(j = 0; j < NUM_TX_CHANNEL * NUM_RX_CHANNEL; j++)
		{
			buffer[j+i*(NUM_TX_CHANNEL * NUM_RX_CHANNEL)] = buffer_temp[j + QUICKSENSE_OVERHEAD - 1];
		}
		if (ret != 0)
		{
			printk("[TSP] silabs_quicksense fail! %s\n", __func__);
			return -1;
		}	
	}
#else	
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s\n", __func__);
		return -1;
	}	
#endif

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
#if defined(CONFIG_LTN_DTV)	
			baseline_node[i][j] = (buffer[(i*Rx_Channel+j)*2] <<8) + buffer[(i*Rx_Channel+j)*2+1];
#else	
			baseline_node[i][j] = (buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
#endif
		//	printk(" %5d", baseline_node[i][j]);
		}
	//	printk("\n");
	}


    for (j = 0; j < Rx_Channel; j++)
    {
    	for(i = 0 ; i < Tx_Channel; i++)
		{
			written_bytes += sprintf(buf+written_bytes, ",%5d", baseline_node[i][j]) ;
    	}
	}

	// printk("[TSP] %s\n", buf);
	
	touch_ctrl_regulator(0);  
	mdelay(2);
	touch_ctrl_regulator(1);  
	mdelay(300);
	
	return written_bytes;

}


#endif


static ssize_t tsp_reset_outJIGMode(struct device *dev, struct device_attribute *attr, char *buf)
{
    tsp_reset();
    testmode = 0;
    return 0;
}

static ssize_t tkey_rawcounter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t buffer[NUM_TX_CHANNEL  *NUM_RX_CHANNEL  *2 + QUICKSENSE_OVERHEAD] = {0,};
    uint16_t button_rawdata[NUM_MTRBUTTONS] = {0,};
    int i, j, ret;

    testmode = 1;
    //            Entering JIG_MODE
    buffer[0] = ESCAPE_ADDR;
    buffer[1] = JIG_MODE_COMMAND;
    ret = i2c_master_send(ts_global->client, buffer, 2);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }
    ret = i2c_master_recv(ts_global->client,buffer, 1);

    //            quicksense format for reading baseline & rawdata of buttons
    ret = silabs_quicksense(I2CMAP_BUTTON_ADDRESS,NUM_MTRBUTTONS * 2, buffer);
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }              
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

    //            reading rawdata of buttons
    for (i = 0; i < NUM_MTRBUTTONS; i++)
    {
        button_rawdata[i] = (buffer[i * 2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[i * 2+QUICKSENSE_OVERHEAD];
   //     printk(" button[%d] = %5d\n", i, button_rawdata[i]);
    }
//    tsp_reset();
    testmode = 0;
    return sprintf(buf, "%d %d", button_rawdata[0], button_rawdata[1]) ;
}

static ssize_t tkey_rawcounter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *after;

    unsigned long value = simple_strtoul(buf, &after, 10);
    printk(KERN_INFO "[TSP] %s, %d\n", __func__);

    if(value == 0)
    {
        hrtimer_cancel(&ts_global->timer);
        tsp_reset();
        prev_wdog_val = -1;
        hrtimer_start(&ts_global->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);        
    }
    return size;
}

static ssize_t tsp_module(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;

#if defined(CONFIG_LTN_DTV)
	count = sprintf(buf, "SILICONLABS,F766");
#else
    count = sprintf(buf, "SILICONLABS,F761");
#endif
  //  printk(KERN_INFO "[TSP] %s, %s\n", __func__, buf);
    
    return count;
}

static ssize_t tsp_xy(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    
    count = sprintf(buf, "10,13");
  //  printk(KERN_INFO "[TSP] %s, %s\n", __func__, buf);
    
    return count;
}

static ssize_t tsp_reference_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
    uint8_t buffer[NUM_TX_CHANNEL * NUM_RX_CHANNEL + QUICKSENSE_OVERHEAD] = {0,};
    uint8_t buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2] = {0,}; 
    int i, j, ret, k = 0;
    int written_bytes = 0;

    printk(KERN_INFO "[TSP] %s\n", __func__);
    testmode = 1;
    mdelay(300); 

    //	Entering JIG_MODE
    buffer[0] = ESCAPE_ADDR;
    buffer[1] = JIG_MODE_COMMAND;
    ret = i2c_master_send(ts_global->client, buffer, 2);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }

    ret = i2c_master_recv(ts_global->client,buffer, 1);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_recv fail! %s\n", __func__);
        return -1;
    }
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

    //	quicksense format for reading baseline
    ret = silabs_quicksense(RAWCOUNT_ADDRESS, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer3[i] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	
    
    memset(buffer, 0x00, sizeof(buffer));
    ret = silabs_quicksense(RAWCOUNT_ADDRESS + 0x82, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);    
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer3[i+NUM_TX_CHANNEL * NUM_RX_CHANNEL] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	

#if 0
    printk("raw : \n");
    for(i = 1; i <= 130; i++)
    {
        printk(" %d", ((buffer3[i*2-2]<<8) + buffer3[i*2-1]));
        if(i % 10 == 0)
            printk("\n");
    }   
#endif

    if(value_for_reference <= 0 || value_for_reference > 13)
    {
        value_for_reference = 1;
    }
    
    for(j = 1 ; j <= NUM_RX_CHANNEL; j++)
    {
        written_bytes += sprintf(buf+written_bytes, "%d,", (buffer3[((value_for_reference-1)*NUM_RX_CHANNEL+j)*2 -2] <<8)\
            + buffer3[((value_for_reference-1)*NUM_RX_CHANNEL+j)*2 - 1]) ;
    }
    
    tsp_reset();

    if(written_bytes > 0)
        return written_bytes;
    testmode = 0;
    return 260;
}

static ssize_t tsp_reference_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value;
    sscanf(buf, "%d", &value_for_reference);

    printk(KERN_INFO "[TSP] %s\n", __func__);

    return size;
}

static ssize_t tsp_difference_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
    uint8_t buffer[NUM_TX_CHANNEL * NUM_RX_CHANNEL + QUICKSENSE_OVERHEAD] = {0,};
    uint8_t buffer_raw[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2] = {0,}; 
    uint8_t buffer_baseline[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2] = {0,};     
    int i, j, ret, k = 0;
    int written_bytes = 0;

    printk(KERN_INFO "[TSP] %s\n", __func__);
    testmode = 1;
    mdelay(300); 

    //	Entering JIG_MODE
    buffer[0] = ESCAPE_ADDR;
    buffer[1] = JIG_MODE_COMMAND;
    ret = i2c_master_send(ts_global->client, buffer, 2);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }

    ret = i2c_master_recv(ts_global->client,buffer, 1);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_recv fail! %s\n", __func__);
        return -1;
    }
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

    //	quicksense format for reading baseline
    ret = silabs_quicksense(RAWCOUNT_ADDRESS, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer_raw[i] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	
    
    memset(buffer, 0x00, sizeof(buffer));
    ret = silabs_quicksense(RAWCOUNT_ADDRESS + 0x82, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);    
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer_raw[i+NUM_TX_CHANNEL * NUM_RX_CHANNEL ] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	

    ret = silabs_quicksense(BASELINE_ADDRESS, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer_baseline[i] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	
    
    memset(buffer, 0x00, sizeof(buffer));
    ret = silabs_quicksense(BASELINE_ADDRESS + 0x82, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);    
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer_baseline[i+NUM_TX_CHANNEL * NUM_RX_CHANNEL] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	

#if 0
    printk("raw : \n");
    for(i = 1; i <= 130; i++)
    {
        printk(" %d", ((buffer_raw[i*2-2]<<8) + buffer_raw[i*2-1]));
        if(i % 10 == 0)
            printk("\n");
    }   
    
    printk("baseline : \n");
        for(i = 1; i <= 130; i++)
    {
        printk(" %d", ((buffer_baseline[i*2-2]<<8) + buffer_baseline[i*2-1]));
        if(i % 10 == 0)
            printk("\n");
    }   
#endif

    if(value_for_difference <= 0 || value_for_difference > 13)
    {
        value_for_difference = 1;
    }

    for(j = 1 ; j <= NUM_RX_CHANNEL; j++)
    {
        written_bytes += sprintf(buf+written_bytes, "%d,", \
            ((buffer_baseline[((value_for_difference-1)*NUM_RX_CHANNEL+j)*2 -2] <<8) + buffer_baseline[((value_for_difference-1)*NUM_RX_CHANNEL+j)*2 - 1]) \
            - ((buffer_raw[((value_for_difference-1)*NUM_RX_CHANNEL+j)*2 -2] <<8) + buffer_raw[((value_for_difference-1)*NUM_RX_CHANNEL+j)*2 - 1])) ;
    }
    tsp_reset();

    if(written_bytes > 0)
        return written_bytes;
    testmode = 0;
    return 260;
}

static ssize_t tsp_difference_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    sscanf(buf, "%d", &value_for_difference);

    printk(KERN_INFO "[TSP] %s\n", __func__);

    return size;
}

static ssize_t tsp_StartCall_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("%s\n", __func__);

    IsDuringCall = 1;
    
    return sprintf(buf, "%d", IsDuringCall);
}

static ssize_t tsp_EndCall_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("%s\n", __func__);

    IsDuringCall = 0;
        
    return sprintf(buf, "%d", IsDuringCall);
}

static ssize_t tsp_debugMatrix_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("%s\n", __func__);

    if(IsdebugMatirx)
        IsdebugMatirx = 0;
    else
        IsdebugMatirx = 1;
        
    return sprintf(buf, "%d", IsDuringCall);
}

static ssize_t show_node(struct device *dev, struct device_attribute *attr, char *buf)
{	
    uint8_t buffer[NUM_TX_CHANNEL * NUM_RX_CHANNEL + QUICKSENSE_OVERHEAD] = {0,};
    uint8_t buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2] = {0,}; 
    char report_buf[260];
    int i, j, ret, k = 0;

    printk(KERN_INFO "[TSP] %s\n", __func__);
    testmode = 1;
    mdelay(300); 

    //	Entering JIG_MODE
    buffer[0] = ESCAPE_ADDR;
    buffer[1] = JIG_MODE_COMMAND;
    ret = i2c_master_send(ts_global->client, buffer, 2);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }

    ret = i2c_master_recv(ts_global->client,buffer, 1);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_recv fail! %s\n", __func__);
        return -1;
    }
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

    //	quicksense format for reading baseline
    ret = silabs_quicksense(BASELINE_ADDRESS, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer3[i] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	
    
    memset(buffer, 0x00, sizeof(buffer));
    ret = silabs_quicksense(BASELINE_ADDRESS + 0x82, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);    
    for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
    {
        buffer3[i + NUM_TX_CHANNEL * NUM_RX_CHANNEL] = buffer[i + QUICKSENSE_OVERHEAD - 1];
    }
    if (ret != 0)
    {
        printk("[TSP] silabs_quicksense fail! %s\n", __func__);
        return -1;
    }	

    memcpy(buf, buffer3, 260);

#if 0
    for(i = 1; i <= 130; i++)
    {
        printk(" %d", ((buffer3[i*2-2]<<8) + buffer3[i*2-1]));
        if(i % 10 == 0)
            printk("\n");
        
    }   
#endif    
    tsp_reset();
    testmode = 0;
    return 260;
}

static ssize_t read_debug_address_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t buffer[NUM_TX_CHANNEL * NUM_RX_CHANNEL + NUM_MTRBUTTONS * 2 + QUICKSENSE_OVERHEAD] = {0,};
    uint8_t buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2 + NUM_MTRBUTTONS * 2] = {0,}; 
    int i, j, ret, k = 0;

    printk("[TSP] %s entered.\n", __func__);
    testmode = 1;
    mdelay(300); 

    buffer[0] = ESCAPE_ADDR;
    buffer[1] = JIG_MODE_COMMAND;
    ret = i2c_master_send(ts_global->client, buffer, 2);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }

    ret = i2c_master_recv(ts_global->client, buffer, 1);

    if (ret < 0)
    {
        printk("[TSP] i2c_master_recv fail! %s\n", __func__);
        return -1;
    }
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

    while(1)
    {
     //   printk(",RAWDATA,");

        ret = silabs_quicksense(DEBUG_ADDRESS, NUM_TX_CHANNEL * NUM_RX_CHANNEL, buffer);
        if (ret != 0)
        {
            printk("[TSP] silabs_quicksense fail! %s\n", __func__);
            return -1;
        }
        for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL; i++)
        {
            buffer3[i] = buffer[i + QUICKSENSE_OVERHEAD - 1];
        }
        
        memset(buffer, 0x00, sizeof(buffer));
        ret = silabs_quicksense(DEBUG_ADDRESS + 0x82, NUM_TX_CHANNEL * NUM_RX_CHANNEL + NUM_MTRBUTTONS * 2, buffer);
        if (ret != 0)
        {
            printk("[TSP] silabs_quicksense fail! %s\n", __func__);
            return -1;
        }
        for(i = 0; i < NUM_TX_CHANNEL * NUM_RX_CHANNEL + NUM_MTRBUTTONS * 2; i++)
        {
            buffer3[i+NUM_TX_CHANNEL * NUM_RX_CHANNEL] = buffer[i + QUICKSENSE_OVERHEAD - 1];
        }
#if 0
        for (i = 0; i < NUM_TX_CHANNEL; i++)
        {
            for(j = 1 ; j <= NUM_RX_CHANNEL; j++)
            {			
                printk("%d,", (buffer3[(i * NUM_RX_CHANNEL + j ) * 2 - 2] <<8) + buffer3[(i * NUM_RX_CHANNEL + j) * 2 - 1]);			
            }
        }

        printk(",");
        printk("%d,", (buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2] <<8) + buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2 + 1]);
        printk("%d,", (buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2  + 2] <<8) + buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2 + 3]);
        printk("\n");
#endif		
    }
    testmode = 0;
    return 1; // success
}

static ssize_t read_debug_address_silabs1(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t buffer[NUM_RX_CHANNEL *2 + QUICKSENSE_OVERHEAD + 4] = {0,};
//    uint8_t buffer3[NUM_TX_CHANNEL * NUM_RX_CHANNEL * 2 + NUM_MTRBUTTONS * 2] = {0,}; 
    int i, j, ret, k = 0;

    printk("[TSP] %s entered\n", __func__);
    testmode = 1;
    mdelay(300); 

    buffer[0] = ESCAPE_ADDR;
    buffer[1] = JIG_MODE_COMMAND;
    ret = i2c_master_send(ts_global->client, buffer, 2);
    if (ret < 0)
    {
        printk("[TSP] i2c_master_send fail! %s\n", __func__);
        return -1;
    }

    ret = i2c_master_recv(ts_global->client, buffer, 1);

    if (ret < 0)
    {
        printk("[TSP] i2c_master_recv fail! %s\n", __func__);
        return -1;
    }
#if defined(CONFIG_LTN_DTV)
	mdelay(50); 
#endif

    while(1)
    {
  //      printk(",RAWDATA,");

        ret = silabs_quicksense(DEBUG_ADDRESS, NUM_RX_CHANNEL *2 , buffer);
        if (ret != 0)
        {
            printk("[TSP] silabs_quicksense fail! %s\n", __func__);
            return -1;
        }
#if 0		
        for (i = 0; i < NUM_RX_CHANNEL; i++)
        {	
            printk("%d,", (buffer[i*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[i*2+QUICKSENSE_OVERHEAD]);			
        }

        printk("\n");     
#endif		
    }
    testmode = 0;
    return 1; // success
}


int firm_update( void )
{
#ifdef CONFIG_LTN_DTV
      int ret;
#endif
    printk(KERN_INFO "[TSP] %s\n", __func__);

	disable_irq(tsp_irq);

#ifdef CONFIG_LTN_DTV
       ret = cancel_work_sync(&ts_global->work_timer);

       ret = cancel_work_sync(&ts_global->work);
       
	hrtimer_cancel(&ts_global->timer);
       
	touch_ctrl_regulator(TOUCH_OFF);
       mdelay(200);      
	touch_ctrl_regulator(TOUCH_ON);
       mdelay(200);

       TSP_MODULE_ID =  buf_firmware[2];
#endif

    local_irq_disable();
    firmware_ret_val = Firmware_Download();	

    msleep(1000);
    if( firmware_ret_val )
        printk(KERN_INFO "[TSP] %s success\n", __func__);
    else	
        printk(KERN_INFO "[TSP] %s fail\n", __func__);

    local_irq_enable();

    enable_irq(tsp_irq);

#ifdef CONFIG_LTN_DTV
	hrtimer_start(&ts_global->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif

    return 0;
} 

module_init(silabs_ts_init);
module_exit(silabs_ts_exit);

MODULE_DESCRIPTION("silabs Touchscreen Driver");
MODULE_LICENSE("GPL");
