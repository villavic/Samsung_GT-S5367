#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <asm/io.h>

#include <mach/gpio.h>

#include "fc8101.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8101_regs.h"
#include "fc8101_isr.h"
#include "fci_hal.h"

#define FEATURE_GLOBAL_MEM

ISDBT_INIT_INFO_T *hInit;

#ifdef FEATURE_GLOBAL_MEM
ISDBT_OPEN_INFO_T hOpen_Val;
u8 ringbuffer[128*1024];
#endif

int isdbt_open (struct inode *inode, struct file *filp);
int isdbt_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
int isdbt_release (struct inode *inode, struct file *filp);
ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);

#define RING_BUFFER_SIZE	(128 * 1024)  // kmalloc max 128k

//GPIO(RESET & INTRRUPT) Setting
#ifdef CONFIG_LTN_DTV
#define FC8101_NAME		"dmb"
#else
#define FC8101_NAME		"isdbt"
#endif

#ifdef CONFIG_LTN_DTV
#define GPIO_ISDBT_IRQ 12 
#define GPIO_ISDBT_PWR_EN 13
#define GPIO_ISDBT_RST 11
#else
#define GPIO_ISDBT_IRQ 0
#define GPIO_ISDBT_PWR_EN 1
#define GPIO_ISDBT_RST 2
#endif
 
static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);

static u8 isdbt_isr_sig=0;
static struct task_struct *isdbt_kthread = NULL;

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	isdbt_isr_sig=1;
	wake_up_interruptible(&isdbt_isr_wait);

	return IRQ_HANDLED;
}

int isdbt_hw_setting(void)
{
	int err;
	PRINTF(0, "isdbt_hw_setting \n");

	err = 	gpio_request(GPIO_ISDBT_PWR_EN, "isdbt_en");
	if (err) {
		PRINTF(0,"isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 1);

	err = 	gpio_request(GPIO_ISDBT_RST, "isdbt_rst");
	if (err) {
		PRINTF(0,"isdbt_hw_setting: Couldn't request isdbt_rst\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_output(GPIO_ISDBT_RST, 1);

	err = 	gpio_request(GPIO_ISDBT_IRQ, "isdbt_irq");
	if (err) {
		PRINTF(0,"isdbt_hw_setting: Couldn't request isdbt_irq\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_input(GPIO_ISDBT_IRQ);

	err = request_irq(GPIO_TO_IRQ(GPIO_ISDBT_IRQ), isdbt_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8101_NAME, NULL);
	
	if (err < 0) {
		PRINTF(0,"isdbt_hw_setting: couldn't request gpio interrupt %d reason(%d)\n",
				gpio_to_irq(GPIO_ISDBT_IRQ),err);
		goto request_isdbt_irq;
	}
	return 0;
request_isdbt_irq:
	gpio_free(GPIO_ISDBT_IRQ);
gpio_isdbt_irq:
	gpio_free(GPIO_ISDBT_RST);
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN);
gpio_isdbt_en:
	return err;
}

//POWER_ON & HW_RESET & INTERRUPT_CLEAR
void isdbt_hw_init(void)
{
	PRINTF(0, "isdbt_hw_init \n");
	gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
	mdelay(1);
	gpio_set_value(GPIO_ISDBT_RST, 1);
	mdelay(1);
	gpio_set_value(GPIO_ISDBT_RST, 0);
	mdelay(1);
	gpio_set_value(GPIO_ISDBT_RST, 1);
}

//POWER_OFF
void isdbt_hw_deinit(void)
{
	PRINTF(0, "isdbt_hw_deinit \n");
	gpio_set_value(GPIO_ISDBT_RST, 0);
	mdelay(1);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
}

int data_callback(u32 hDevice, u8 *data, int len)
{
	ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;
	hInit = (ISDBT_INIT_INFO_T *)hDevice;

	list_for_each(temp, &(hInit->hHead))
	{
		ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, ISDBT_OPEN_INFO_T, hList);

		if(hOpen->isdbttype == TS_TYPE)
		{
			if(fci_ringbuffer_free(&hOpen->RingBuffer) < (len+2) ) 
			{
				//PRINTF(hDevice, "f");
				return 0;
			}

			FCI_RINGBUFFER_WRITE_BYTE(&hOpen->RingBuffer, len >> 8);
			FCI_RINGBUFFER_WRITE_BYTE(&hOpen->RingBuffer, len & 0xff);

			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);

			wake_up_interruptible(&(hOpen->RingBuffer.queue));
		}
	}

	return 0;
}

static int isdbt_thread(void *hDevice)
{
	static DEFINE_MUTEX(thread_lock);

	ISDBT_INIT_INFO_T *hInit = (ISDBT_INIT_INFO_T *)hDevice;
	
	set_user_nice(current, -20);
	
	PRINTF(hInit, "isdbt_kthread enter\n");

	BBM_CALLBACK_REGISTER((u32)hInit, data_callback);

	while(1)
	{
		wait_event_interruptible(isdbt_isr_wait, isdbt_isr_sig || kthread_should_stop());
		
		isdbt_isr_sig=0;
		
		BBM_ISR(hInit);
	
		if (kthread_should_stop())
			break;
	}

	BBM_CALLBACK_DEREGISTER(hInit);
	
	PRINTF(hInit, "isdbt_kthread exit\n");

	return 0;
}

static struct file_operations isdbt_fops = 
{
	.owner		= THIS_MODULE,
	.ioctl		= isdbt_ioctl,
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

static struct miscdevice fc8101_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = FC8101_NAME,
    .fops = &isdbt_fops,
};

int isdbt_open (struct inode *inode, struct file *filp)
{
	ISDBT_OPEN_INFO_T *hOpen;

	PRINTF(hInit, "isdbt open\n");

#ifdef FEATURE_GLOBAL_MEM
	hOpen = &hOpen_Val;
	hOpen->buf = &ringbuffer[0];
#else
	hOpen = (ISDBT_OPEN_INFO_T *)kmalloc(sizeof(ISDBT_OPEN_INFO_T), GFP_KERNEL);
	hOpen->buf = (u8 *)kmalloc(RING_BUFFER_SIZE, GFP_KERNEL);
#endif

	hOpen->isdbttype = 0;

	list_add(&(hOpen->hList), &(hInit->hHead));

	hOpen->hInit = (HANDLE *)hInit;

	if(hOpen->buf == NULL)
	{
		PRINTF(hInit, "ring buffer malloc error\n");
		return -ENOMEM;
	}

	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

	filp->private_data = hOpen;

	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	s32 non_blocking = filp->f_flags & O_NONBLOCK;
	ISDBT_OPEN_INFO_T *hOpen = (ISDBT_OPEN_INFO_T*)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len;

	if (!cibuf->data || !count)
	{
		//PRINTF(hInit, " return 0\n");
		return 0;
	}
	
	if (non_blocking && (fci_ringbuffer_empty(cibuf)))
	{
		//PRINTF(hInit, "return EWOULDBLOCK\n");
		return -EWOULDBLOCK;
	}
	#if 0
	if (wait_event_interruptible(cibuf->queue, !fci_ringbuffer_empty(cibuf)))
	{
		PRINTF(hInit, "return ERESTARTSYS\n");
	 	return -ERESTARTSYS;
	}
	#endif
	avail = fci_ringbuffer_avail(cibuf);
	
	if (avail < 4)
	{
		PRINTF(hInit, "return 00\n");
		return 0;
	}
	
	len = FCI_RINGBUFFER_PEEK(cibuf, 0) << 8;
	len |= FCI_RINGBUFFER_PEEK(cibuf, 1);
	
	if (avail < len + 2 || count < len)
	{
		PRINTF(hInit, "return EINVAL\n");		
		return -EINVAL;
	}
	
	FCI_RINGBUFFER_SKIP(cibuf, 2);

	return fci_ringbuffer_read_user(cibuf, buf, len);
}

int isdbt_release (struct inode *inode, struct file *filp)
{
	ISDBT_OPEN_INFO_T *hOpen;

	PRINTF(hInit, "isdbt_release\n");

	hOpen = filp->private_data;

	hOpen->isdbttype = 0;

	list_del(&(hOpen->hList));
	
#ifndef FEATURE_GLOBAL_MEM
	kfree(hOpen->buf);
	kfree(hOpen);
#endif

	return 0;
}

int fc8101_if_test(void)
{
	int res=0;
	int i;
	u16 wdata=0;
	u32 ldata=0;
	u8 data=0;

	BBM_PROBE(NULL);
	for(i=0;i<3000;i++){
		bbm_ext_byte_write(0, 0xa0, i&0xff);
		bbm_ext_byte_read(0, 0xa0, &data);
		if((i&0xff)!=data){
			PRINTF(0, "fc8101_if_btest!   i=0x%x, data=0x%x\n", i&0xff, data);
		res=1;
			}
		}

	
	for(i=0;i<3000;i++){
		bbm_ext_word_write(0, 0xa0, i&0xffff);
		bbm_ext_word_read(0, 0xa0, &wdata);
		if((i&0xffff)!=wdata){
			PRINTF(0, "fc8101_if_wtest!   i=0x%x, data=0x%x\n", i&0xffff, wdata);
		res=1;
			}
		}

	for(i=0;i<10000;i++){
		bbm_ext_long_write(0, 0xa0, i&0xffffffff);
		data=0;
		bbm_ext_long_read(0, 0xa0, &ldata);
		if((i&0xffffffff)!=ldata){
			PRINTF(0, "fc8101_if_ltest!   i=0x%x, data=0x%x\n", i&0xffffffff, ldata);
		res=1;
			}
		}

	return res;
}

int isdbt_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	s32 size = 0;
	ISDBT_OPEN_INFO_T *hOpen;

	ioctl_info info;

	if(_IOC_TYPE(cmd) != IOCTL_MAGIC) 
		return -EINVAL;
	if(_IOC_NR(cmd) >= IOCTL_MAXNR) 
		return -EINVAL;

	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);

	switch(cmd) 
	{
		case IOCTL_ISDBT_RESET:
			res = BBM_RESET(hInit);
			break;
		case IOCTL_ISDBT_INIT:
			res = BBM_I2C_INIT(hInit, FCI_I2C_TYPE);
			res |= BBM_INIT(hInit);
			break;
		case IOCTL_ISDBT_BYTE_READ:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_BYTE_READ(hInit, (u16)info.buff[0], (u8 *)(&info.buff[1]));
			err |= copy_to_user((void *)arg, (void *)&info, size);
			break;
		case IOCTL_ISDBT_WORD_READ:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_WORD_READ(hInit, (u16)info.buff[0], (u16 *)(&info.buff[1]));
			err |= copy_to_user((void *)arg, (void *)&info, size);
			break;
		case IOCTL_ISDBT_LONG_READ:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_LONG_READ(hInit, (u16)info.buff[0], (u32 *)(&info.buff[1]));
			err |= copy_to_user((void *)arg, (void *)&info, size);
			break;
		case IOCTL_ISDBT_BULK_READ:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_BULK_READ(hInit, (u16)info.buff[0], (u8 *)(&info.buff[2]), info.buff[1]);
			err |= copy_to_user((void *)arg, (void *)&info, size);
			break;
		case IOCTL_ISDBT_BYTE_WRITE:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_BYTE_WRITE(hInit, (u16)info.buff[0], (u8)info.buff[1]);
			break;
		case IOCTL_ISDBT_WORD_WRITE:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_WORD_WRITE(hInit, (u16)info.buff[0], (u16)info.buff[1]);
			break;
		case IOCTL_ISDBT_LONG_WRITE:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_LONG_WRITE(hInit, (u16)info.buff[0], (u32)info.buff[1]);
			break;
		case IOCTL_ISDBT_BULK_WRITE:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_BULK_WRITE(hInit, (u16)info.buff[0], (u8 *)(&info.buff[2]), info.buff[1]);
			break;
		case IOCTL_ISDBT_TUNER_READ:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_TUNER_READ(hInit, (u8)info.buff[0], (u8)info.buff[1],  (u8 *)(&info.buff[3]), (u8)info.buff[2]);
			err |= copy_to_user((void *)arg, (void *)&info, size);
			break;
		case IOCTL_ISDBT_TUNER_WRITE:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_TUNER_WRITE(hInit, (u8)info.buff[0], (u8)info.buff[1], (u8 *)(&info.buff[3]), (u8)info.buff[2]);
			break;
		case IOCTL_ISDBT_TUNER_SET_FREQ:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_TUNER_SET_FREQ(hInit, (u32)info.buff[0]);
			break;
		case IOCTL_ISDBT_TUNER_SELECT:
			err = copy_from_user((void *)&info, (void *)arg, size);
			res = BBM_TUNER_SELECT(hInit, (u32)info.buff[0], 0);
			break;
		case IOCTL_ISDBT_TS_START:
			hOpen->isdbttype = TS_TYPE;
			break;
		case IOCTL_ISDBT_TS_STOP:
			hOpen->isdbttype = 0;
			break;
		case IOCTL_ISDBT_POWER_ON:
			isdbt_hw_init();
			break;
		case IOCTL_ISDBT_POWER_OFF:
			isdbt_hw_deinit();
			break;
		default:
			PRINTF(hInit, "isdbt ioctl error!\n");
			res = BBM_NOK;
			break;
	}
	
	if(err < 0)
	{
		PRINTF(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res; 
}

int isdbt_init(void)
{
	s32 res;

	PRINTF(hInit, "isdbt_init\n");

	res = misc_register(&fc8101_misc_device);

	if(res < 0)
	{
		PRINTF(hInit, "isdbt init fail : %d\n", res);
		return res;
	}

	isdbt_hw_setting();

	isdbt_hw_init();

	hInit = (ISDBT_INIT_INFO_T *)kmalloc(sizeof(ISDBT_INIT_INFO_T), GFP_KERNEL);

	res = BBM_HOSTIF_SELECT(hInit, BBM_SPI);
	
	if(res)
		PRINTF(hInit, "isdbt host interface select fail!\n");

	isdbt_hw_deinit();

	if (!isdbt_kthread)
	{
		PRINTF(hInit, "kthread run\n");
		isdbt_kthread = kthread_run(isdbt_thread, (void*)hInit, "isdbt_thread");
	}

	INIT_LIST_HEAD(&(hInit->hHead));

	return 0;
}

void isdbt_exit(void)
{
	PRINTF(hInit, "isdbt isdbt_exit \n");

	free_irq(GPIO_TO_IRQ(GPIO_ISDBT_IRQ), NULL);
	gpio_free(GPIO_ISDBT_IRQ);
	gpio_free(GPIO_ISDBT_RST);
	gpio_free(GPIO_ISDBT_PWR_EN);
	
	kthread_stop(isdbt_kthread);
	isdbt_kthread = NULL;

	BBM_HOSTIF_DESELECT(hInit);

	isdbt_hw_deinit();
	
	misc_deregister(&fc8101_misc_device);
	
	kfree(hInit);
}

module_init(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");

