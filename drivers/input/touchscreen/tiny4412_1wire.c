/*
 * tiny4412_1wire_host.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * LCD-CPU one wire communication for Mini6410 from
 *         FriendlyARM Guangzhou CO., LTD.
 *
 * Copyright (c) 2010 FriendlyARM Guangzhou CO., LTD.  <http://www.arm9.net>
 *
 * ChangeLog
 *
 *
 * 2010-10-14: Russell Guo <russell.grey@gmail.com>
 *      - Initial version
 *      -- request touch-screen data
 *      -- request LCD type, Firmware version
 *      -  Backlight control
 *
 * 2017-8-19：transplant by SY <1530454315@qq.com>
 * 		- Using DTS Style
 *
 * the CRC-8 functions is based on web page from http://lfh1986.blogspot.com
 */

// #define DEBUG 

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/mach/time.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>

#define SAMPLE_BPS 9600

#define SLOW_LOOP_FEQ 25
#define FAST_LOOP_FEQ 60

#define REQ_KEY  0x30U
#define REQ_TS   0x40U
#define REQ_INFO 0x60U


struct pwm_base
{
    unsigned int TCFG0;
    unsigned int TCFG1;
    unsigned int TCON;
    unsigned int TCNTB0;
    unsigned int TCMPB0;
    unsigned int TCNTO0;
    unsigned int TCNTB1;
    unsigned int TCMPB1;
    unsigned int TCNTO1;
    unsigned int TCNTB2;
    unsigned int TCMPB2;
    unsigned int TCNTO2;
    unsigned int TCNTB3;
    unsigned int TCMPB3;
    unsigned int TCNTO3;
    unsigned int TCNTB4;
    unsigned int TCBTO4;
    unsigned int TINT_CSTAT;
};

enum rw_status
{
    IDLE = 0,
    START,
    REQUEST,
    WAITING,
    RESPONSE,
    STOPING,
};

static const unsigned char crc8_tab[] =
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

#define crc8_init(crc) ((crc) = 0XACU)
#define crc8(crc, v)   ((crc) = crc8_tab[(crc) ^(v)])

struct lcd_key_data {
	struct input_dev *input;
};


struct tiny4412_1wire_data {
	struct pinctrl *pctrl;
	struct pinctrl_state *pstate_in;
	struct pinctrl_state *pstate_out;
	int gpio_write;
	struct resource *res;
	struct resource *irq;
	volatile struct pwm_base *pwm;
	volatile unsigned int io_bit_count;
	volatile enum rw_status one_wire_status;
	volatile unsigned int io_data;
	int major;
	struct cdev one_wire_cdev;
	struct class *one_wire_class;
	struct clk *timer_clk;
	uint8_t req;
	
	/* Key sub system */
	struct lcd_key_data key;

	/* kernel timer */
	struct timer_list timer;
	unsigned char brightness;
	uint8_t bl_wr_done;
	uint8_t lcd_model;
	int last_key;
};

static struct tiny4412_1wire_data *this = NULL;


const unsigned int KEY_TABLE[] = {
	KEY_HOME,
	KEY_MENU,
	KEY_BACK,
};

static void start_one_wire_session(unsigned char req)
{
    unsigned int tcon;
	unsigned long prescale;

	if (!this) {
		return;
	}

	this->req = req;
    this->one_wire_status = START;
    gpio_set_value(this->gpio_write, 1);
    pinctrl_select_state(this->pctrl, this->pstate_out);
 
 	// IDLE to START
    {
        unsigned char crc;
        crc8_init(crc);
        crc8(crc, req);
        this->io_data = (req << 8) + crc;
        this->io_data <<= 16;
    }
    
	this->io_bit_count = 1;
    pinctrl_select_state(this->pctrl, this->pstate_out);
    
	//Each timer has its 32-bit down-counter
	prescale = (this->pwm->TCFG0 >> 8) & 0xFF;
	this->pwm->TCNTB3 = clk_get_rate(this->timer_clk) / (prescale + 1) / SAMPLE_BPS - 1;
    
	//init tranfer and start timer
    tcon = this->pwm->TCON;
    tcon &= ~(0xF << 16);
    tcon |= (1 << 17);
    this->pwm->TCON = tcon;
    tcon |= (1 << 16);
    tcon |= (1 << 19);
    tcon &= ~(1 << 17);
    this->pwm->TCON = tcon;
    this->pwm->TINT_CSTAT |= 0x08;
    gpio_set_value(this->gpio_write, 0);
}


/*********************************************************
	Brief：backlight
*********************************************************/
static int backlight_open(struct inode *inode, struct file *file)
{
    printk("backlight_open\n");
    return 0;
}

static int backlight_release(struct inode *inode, struct file *file)
{
    printk("backlight_exit\n");
    return 0;
}

DECLARE_WAIT_QUEUE_HEAD(bl_wait);

static ssize_t backlight_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
	unsigned char reg;
	int ret;
 
   	ret = copy_from_user(&reg, buf, 1);
    if (ret < 0)
    {
        printk("%s copy_from_user error\n", __func__);
    }

    if (reg > 127) {
		reg = 127; 
	}
	pr_notice("<0-127> brightness = %d\n", reg);

	this->bl_wr_done = 0;
	this->brightness = 0x80 + reg;
	ret = wait_event_interruptible_timeout(bl_wait, this->bl_wr_done, HZ/10);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		return -ETIMEDOUT;
	}

    return 1;
}

static struct file_operations backlight_fops =
{
    .owner              = THIS_MODULE,
    .open               = backlight_open,
    .release            = backlight_release,
    .write              = backlight_write,
};

static inline void stop_timer_for_1wire(void)
{
	this->pwm->TCON &= ~(1 << 16);
}

static inline void notify_bl_data(unsigned char a, unsigned char b, unsigned char c)
{
	this->bl_wr_done = 1;
	this->brightness = 0;
	wake_up_interruptible(&bl_wait);
	printk(KERN_DEBUG "Write backlight done.\n");
}

static inline void notify_info_data(unsigned char _lcd_type,
		unsigned char ver_year, unsigned char week)
{
	if (_lcd_type != 0xFF) {
		unsigned int firmware_ver = ver_year * 100 + week;

		/* Currently only S702 has hard key */
		pr_notice("[LCD] model = %s, ver = %d\n", (_lcd_type == 24) ? "S702" : "N/A", firmware_ver);
		this->lcd_model = _lcd_type;
	}
}

static void ts_if_report_key(int key) 
{
	int changed = this->last_key ^ key;
	int down;
	int i;

	if (!changed)
		return;

	for (i = 0; i < ARRAY_SIZE(KEY_TABLE); i++) {
		if (changed & (1 << i)) {
			down = !!((1<<i) & key);
			printk(KERN_DEBUG "report key = %d press = %d\n", KEY_TABLE[i], down);
			input_report_key(this->key.input, KEY_TABLE[i], down);
		}
	}

	this->last_key = key;
	input_sync(this->key.input);
	return;
}

static void one_wire_session_complete(unsigned char req, unsigned int res)
{
	unsigned char crc;
	const unsigned char *p = (const unsigned char*)&res;

	crc8_init(crc);
	crc8(crc, p[3]);
	crc8(crc, p[2]);
	crc8(crc, p[1]);

	// CRC dismatch
	if (crc != p[0]) {
		return;
	}
	
	switch (req) {
		case REQ_KEY:
			ts_if_report_key(p[1]);
			break;

		case REQ_TS:
			pr_notice("It does not support touchscreen for onewire!\n");
			break;

		case REQ_INFO:
			notify_info_data(p[3], p[2], p[1]);
			break;

		default:
			notify_bl_data(p[3], p[2], p[1]);
			break;
	}
}

static irqreturn_t timer_for_1wire_interrupt(int irq, void *dev_id)
{
	if (!this) {
		return IRQ_HANDLED;
	}

	/* Clear interrupt status bit */
	this->pwm->TINT_CSTAT |= (1 << 8);

	this->io_bit_count--;
	switch(this->one_wire_status) {
	case IDLE:
		break;
	case START:
		if (this->io_bit_count == 0) {
			this->io_bit_count = 16;
			this->one_wire_status = REQUEST;
		}
		break;

	case REQUEST:
		// Send a bit
		gpio_set_value(this->gpio_write, this->io_data & (1U << 31));
		this->io_data <<= 1;
		if (this->io_bit_count == 0) {
			this->io_bit_count = 2;
			this->one_wire_status = WAITING;
		}
		break;
		
	case WAITING:
		if (this->io_bit_count == 0) {
			this->io_bit_count = 32;
			this->io_data = 0;
			this->one_wire_status = RESPONSE;
		}
		if (this->io_bit_count == 1) {
			pinctrl_select_state(this->pctrl, this->pstate_in);
			gpio_set_value(this->gpio_write, 1);
		}
		break;
		
	case RESPONSE:
		// Get a bit
		this->io_data = (this->io_data << 1) | gpio_get_value(this->gpio_write);
		if (this->io_bit_count == 0) {
			this->io_bit_count = 2;
			this->one_wire_status = STOPING;
			gpio_set_value(this->gpio_write, 1);
			pinctrl_select_state(this->pctrl, this->pstate_out);
			//rx data
			one_wire_session_complete(this->req, this->io_data);
		}
		break;

	case STOPING:
		if (this->io_bit_count == 0) {
			this->one_wire_status = IDLE;
			stop_timer_for_1wire();
		}
		break;
		
	default:
		stop_timer_for_1wire();
	}

	return IRQ_HANDLED;
}

struct samsung_pwm_variant {
	u8 bits;
	u8 div_base;
	u8 tclk_mask;
	u8 output_mask;
	bool has_tint_cstat;
};

struct samsung_pwm_chip {
	struct pwm_chip chip;
	struct samsung_pwm_variant variant;
	u8 inverter_mask;

	void __iomem *base;
	struct clk *base_clk;
	struct clk *tclk0;
	struct clk *tclk1;
};

static int ts_1wire_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tiny4412_1wire_data *priv;	
	int ret;
	dev_t devid;
	
	dev_dbg(dev, "probe.");	

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "calloc mem error!\n");
		return -ENOMEM;
	}
	this = priv;
	
	/* pinctrl */
	priv->pctrl = devm_pinctrl_get(dev);	
	if (IS_ERR(priv->pctrl)) {
		dev_err(dev, "get pinctrl error!\n");
		return -EINVAL;
	}

	priv->pstate_in = pinctrl_lookup_state(priv->pctrl, "backlight_in");
	if (IS_ERR(priv->pstate_in)) {
		dev_err(dev, "backlight_in not found!\n");
		return -EINVAL;
	}
	
	priv->pstate_out = pinctrl_lookup_state(priv->pctrl, "backlight_out");
	if (IS_ERR(priv->pstate_out)) {
		dev_err(dev, "backlight_out not found!\n");
		return -EINVAL;
	}

	/* Get clk */
	priv->timer_clk = devm_clk_get(dev, "timers");
	if (IS_ERR(priv->timer_clk)) {
		dev_err(dev, "devm_clk_get error!\n");
		return -EINVAL;
	}
	dev_dbg(dev, "clk = %ld Hz\n", clk_get_rate(priv->timer_clk));
	
	/* Enable clk */
	ret = clk_prepare_enable(priv->timer_clk);
	if (ret) {
		dev_err(dev, "clk_prepare_enable error!\n");
		return -EINVAL;
	}

	/* Get gpios */
	priv->gpio_write = of_get_named_gpio(dev->of_node, "gpios", 0);	
	if (!gpio_is_valid(priv->gpio_write)) {
		dev_err(dev, "gpios not found!\n");
		return -EINVAL;
	}
	
	/* Request gpios */
	ret = devm_gpio_request_one(dev, priv->gpio_write, GPIOF_OUT_INIT_HIGH, "one_wire");
	if (ret) {
		dev_err(dev, "request gpio error!\n");
		return -EINVAL;
	}

{
	/* io资源在samsung pwm中已申请 */
#if 0
	/* Get res */
	priv->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->res) {
		dev_err(dev, "get res error!\n");
		return -EINVAL;
	}
	dev_err(dev, "start=%x, end=%x name=%s\n", priv->res->start, priv->res->end, priv->res->name);
	
	/* Get ioremap */
	priv->pwm = (volatile struct pwm_base *)devm_ioremap_resource(dev, priv->res);
	if (IS_ERR((struct pwm_base *)priv->pwm)) {
		dev_err(dev, "devm_ioremap_resource error!\n");
		return -EINVAL;
	}
#endif

	struct samsung_pwm_chip *chip;
	struct pwm_device *pwm;
	
	pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(pwm)) {
		return -EINVAL;

	}
	devm_pwm_put(dev, pwm);
	
	chip = (struct samsung_pwm_chip *)pwm->chip;
	priv->pwm = (volatile struct pwm_base *)chip->base;
	if (IS_ERR((struct pwm_base *)priv->pwm)) {
		return -EINVAL;
	}
}
	/* Timer 3 */
	priv->pwm->TCFG0 |=  (0xf << 8);
	priv->pwm->TCFG1 &= ~(0xf << 12);
	dev_dbg(dev, "TCFG0: %08x", priv->pwm->TCFG0);
	dev_dbg(dev, "TCFG1: %08x", priv->pwm->TCFG1);

	/* irq */
	priv->irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!priv->irq) {
		dev_err(dev, "get irq error!\n");
		return -EINVAL;
	}
	
	ret = devm_request_irq(dev, priv->irq->start, timer_for_1wire_interrupt , IRQF_TIMER, "backlight", NULL);
	if (ret) {
		dev_err(dev, "devm_request_irq error!\n");
		return -EINVAL;
	}

	start_one_wire_session(REQ_INFO);
	
	/* char device */
	ret = alloc_chrdev_region(&devid, 0, 1, "backlight");
	if (ret) {
		dev_err(dev, "alloc_chrdev_region error!\n");
		return -EINVAL;
	}
	priv->major = MAJOR(devid);		
	cdev_init(&priv->one_wire_cdev, &backlight_fops);
	cdev_add(&priv->one_wire_cdev, devid, 1);

	/* class */	
	priv->one_wire_class = class_create(THIS_MODULE, "onewire_backlight");
    device_create(priv->one_wire_class, NULL, MKDEV(priv->major, 0), NULL, "backlight");

	platform_set_drvdata(pdev, priv);		

	return 0;
}

static int ts_1wire_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tiny4412_1wire_data *priv = dev_get_drvdata(dev);

	device_destroy(priv->one_wire_class, MKDEV(priv->major, 0));
    class_destroy(priv->one_wire_class);
	cdev_del(&priv->one_wire_cdev);
	
	devm_pinctrl_put(priv->pctrl);
	devm_gpio_free(dev, priv->gpio_write);

	this = NULL;

	return 0;
}

static const struct of_device_id ts_1wire_of_match[] = {
	{ .compatible = "tiny4412, onewire_touchscreen" },
	{ .compatible = "tiny4412, onewire_backlight" },
	{ },
};

static struct platform_driver ts_1wire_device_driver = {
	.probe				= ts_1wire_probe,
	.remove				= ts_1wire_remove,
	.driver				= {
		.name			= "tiny4412_onewire",
		.of_match_table = of_match_ptr(ts_1wire_of_match),
	},
};


/*********************************************************
	Brief：Kernel Timer
*********************************************************/
static void timer_handler(unsigned long timer)
{
	if (this->one_wire_status != IDLE) {
		return;
	}

	if (!this->lcd_model) {
		start_one_wire_session(REQ_INFO);
	} else if (this->brightness) {
		start_one_wire_session(this->brightness);
	} else {
		start_one_wire_session(REQ_KEY);
	}
}

static void	timer_callback(unsigned long timer)
{
	this->timer.expires = jiffies + msecs_to_jiffies(20);
	add_timer(&this->timer);
	
	timer_handler(timer);
}

static void timer_setup(void)
{
	init_timer(&this->timer);
	this->timer.function = timer_callback;
	/* Start up timer */
	timer_callback(0);
}


/*********************************************************
	Brief：LCD key
*********************************************************/

static int lcd_key_init(void)
{
	struct input_dev *input;
	int ret,i;

	input = input_allocate_device();
	if (!input) {
		pr_notice("input alloc error!\n");
		return -ENOMEM;
	}

	input->name = "tiny4412_lcd_key";
	input->id.bustype   = BUS_RS232;
	input->id.vendor	= 0xDEAD;
	input->id.product	= 0xBEEF;
	input->id.version 	= 0x0100;
	
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT(EV_SYN);
	
	for (i=0; i<ARRAY_SIZE(KEY_TABLE); ++i) {
		input_set_capability(input, EV_KEY, KEY_TABLE[i]);
	}
	
	ret = input_register_device(input);
	this->key.input = input;

	return ret;
}



static int __init dev_init(void)
{
	int ret = platform_driver_register(&ts_1wire_device_driver);	
	if (!ret) {
		ret = lcd_key_init();
		timer_setup();
	}

	return ret;
}

static void __exit dev_exit(void)
{
	del_timer_sync(&this->timer);
	input_unregister_device(this->key.input);
	platform_driver_unregister(&ts_1wire_device_driver);
}

module_init(dev_init);
module_exit(dev_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("SY <1530454315@qq.com>");
MODULE_DESCRIPTION("Tiny4412 one-wire host and Touch Screen Driver");


