/* drivers/rtc/rtc-ht1382.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for the ht1382 RTC
 *
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/types.h>
#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

// Command Bytes
#define COMMAND_WR_MULTI      	0xBE
#define COMMAND_RD_MULTI      	0xBF
#define COMMAND_WP_ACCESS_WR  	0x8E
#define COMMAND_WP_ACCESS_RD  	0x8F     
#define COMMAND_OSC_WR_ACCESS 	0x80
#define COMMAND_OSC_RD_ACCESS 	0x81
#define COMMAND_ALARM_ST2_WR 	0x90
#define COMMAND_ALARM_ST2_RD 	0x91
#define COMMAND_ALARM_ACCESS_WR 0x92
#define COMMAND_ALARM_ACCESS_RD 0x93
#define COMMAND_ALARM_SEC_WR 	0x94
#define COMMAND_ALARM_SEC_RD 	0x95
#define COMMAND_ALARM_MIN_WR 	0x96
#define COMMAND_ALARM_MIN_RD 	0x97
#define COMMAND_ALARM_HR_WR 	0x98
#define COMMAND_ALARM_HR_RD 	0x99
#define COMMAND_ALARM_DAY_WR 	0x9A
#define COMMAND_ALARM_DAY_RD 	0x9B
#define COMMAND_ALARM_MON_WR 	0x9C
#define COMMAND_ALARM_MON_RD 	0x9D
#define COMMAND_ALARM_WEEK_WR 	0x9E
#define COMMAND_ALARM_WEEK_RD 	0x9F

#define INX_SEC            		0x0
#define INX_MIN		       		0x1
#define INX_HOUR	       		0x2
#define INX_DAY		       		0x3
#define INX_MONTH	       		0x4
#define INX_WEEK	       		0x5
#define INX_YEAR	       		0x6
#define INX_RESERVE	       		0x7

#define HT1382_CE				0
#define HT1382_CLK				1
#define HT1382_DIO				2

static int tm2bcd(struct rtc_time *tm)
{
	if (rtc_valid_tm(tm) != 0)
		return -EINVAL;

	tm->tm_sec = bin2bcd(tm->tm_sec);
	tm->tm_min = bin2bcd(tm->tm_min);
	tm->tm_hour = bin2bcd(tm->tm_hour);
	tm->tm_mday = bin2bcd(tm->tm_mday);

	tm->tm_mon = bin2bcd(tm->tm_mon + 1);

	/* epoch == 1900 */
	if (tm->tm_year < 100 || tm->tm_year > 199)
		return -EINVAL;
	tm->tm_year = bin2bcd(tm->tm_year - 100);

	return 0;
}

static void bcd2tm(struct rtc_time *tm)
{
	tm->tm_sec = bcd2bin(tm->tm_sec);
	tm->tm_min = bcd2bin(tm->tm_min);
	tm->tm_hour = bcd2bin(tm->tm_hour);
	tm->tm_mday = bcd2bin(tm->tm_mday);
	tm->tm_mon = bcd2bin(tm->tm_mon) - 1;
	/* epoch == 1900 */
	tm->tm_year = bcd2bin(tm->tm_year) + 100;
}

//------------------------------------------------------------------------------
struct ht1382 {
    /* GPIO access */
    struct rtc_device *rtc;
    int irq_alarm;
    spinlock_t lock;
};

//------------------------------------------------------------------------------
#if defined(CONFIG_MACH_ECU1155)
/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank - 1) + (gpio))

#define GPIO_HT1382_CE          GPIO_TO_PIN(4, 5)
#define GPIO_HT1382_DIO         GPIO_TO_PIN(5, 18)
#define GPIO_HT1382_CLK         GPIO_TO_PIN(5, 19)
#endif

#if defined(CONFIG_MACH_ECU150) || defined(CONFIG_MACH_ECU150FL) || defined(CONFIG_MACH_ECU150A1) || defined(CONFIG_MACH_ECU150F) || defined(CONFIG_MACH_ECU1370)
#define GPIO_TO_PIN(bank, gpio) (32 * (bank - 1) + (gpio))

#define GPIO_HT1382_CE          GPIO_TO_PIN(4, 27)
#define GPIO_HT1382_DIO         GPIO_TO_PIN(1, 5)
#define GPIO_HT1382_CLK         GPIO_TO_PIN(1, 4)
#endif

/* ISL1208/1219 Register map */
/* rtc section */
#define ISL1208_REG_SC  0x00
#define ISL1208_REG_MN  0x01
#define ISL1208_REG_HR  0x02
#define ISL1208_REG_HR_MIL     (1<<7)	/* 24h/12h mode */
#define ISL1208_REG_HR_PM      (1<<5)	/* PM/AM bit in 12h mode */
#define ISL1208_REG_DT  0x03
#define ISL1208_REG_MO  0x04
#define ISL1208_REG_YR  0x05
#define ISL1208_REG_DW  0x06
#define ISL1208_RTC_SECTION_LEN 7

/* control/status section */
#define ISL1208_REG_SR  0x07
#define ISL1208_REG_SR_ARST    (1<<7)	/* auto reset */
#define ISL1208_REG_SR_XTOSCB  (1<<6)	/* crystal oscillator */
#define ISL1208_REG_SR_WRTC    (1<<4)	/* write rtc */
#define ISL1208_REG_SR_EVT     (1<<3)	/* event */
#define ISL1208_REG_SR_ALM     (1<<2)	/* alarm */
#define ISL1208_REG_SR_BAT     (1<<1)	/* battery */
#define ISL1208_REG_SR_RTCF    (1<<0)	/* rtc fail */
#define ISL1208_REG_INT 0x08
#define ISL1208_REG_INT_ALME   (1<<6)   /* alarm enable */
#define ISL1208_REG_INT_IM     (1<<7)   /* interrupt/alarm mode */
#define ISL1219_REG_EV  0x09
#define ISL1219_REG_EV_EVEN    (1<<4)   /* event detection enable */
#define ISL1219_REG_EV_EVIENB  (1<<7)   /* event in pull-up disable */
#define ISL1208_REG_ATR 0x0a
#define ISL1208_REG_DTR 0x0b

/* alarm section */
#define ISL1208_REG_SCA 0x0c
#define ISL1208_REG_MNA 0x0d
#define ISL1208_REG_HRA 0x0e
#define ISL1208_REG_DTA 0x0f
#define ISL1208_REG_MOA 0x10
#define ISL1208_REG_DWA 0x11
#define ISL1208_ALARM_SECTION_LEN 6

#define ACK 					0
#define NACK 					1

#define HT1382_RTC				0
#define ISL1219_RTC				1

#define HT1382_ADDR				0xD0
#define ISL1219_ADDR			0xDE

#define GPIO_I2C_UDELAY			4

//------------------------------------------------------------------------------
struct gpio_i2c_platform_data {
    unsigned int	scl_pin;
    unsigned int    sda_pin;
    int     		udelay;
};

struct gpio_i2c_dev {
    struct gpio_i2c_platform_data data;
};

static int rtc_ic = HT1382_RTC;
static int ht1382_i2c_interface = 0;
static struct gpio_i2c_dev rtc_dev;

//------------------------------------------------------------------------------
static void gpio_i2c_delay(struct gpio_i2c_dev *dev)
{
    udelay(dev->data.udelay);
}

static void IIC_Start(struct gpio_i2c_dev *dev)
{
    gpio_i2c_delay(dev);
    gpio_direction_output(dev->data.sda_pin, 1);
    gpio_direction_output (dev->data.scl_pin, 1);
    gpio_set_value(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.sda_pin, 0);
    gpio_i2c_delay(dev);
}	

static void IIC_Stop(struct gpio_i2c_dev *dev)
{
    gpio_direction_output(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.sda_pin, 0);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.sda_pin, 1); // stop
    gpio_i2c_delay(dev);  						   	
}

static unsigned char IIC_Wait_Ack(struct gpio_i2c_dev *dev)
{
    unsigned char ucErrTime = 0;
	
    gpio_direction_input(dev->data.sda_pin); // input
    gpio_i2c_delay(dev);	   
    gpio_set_value(dev->data.scl_pin, 1); 
    while (gpio_get_value(dev->data.sda_pin))
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
        	IIC_Stop(dev);
        	return 1;
        }
    }
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 0); 
    gpio_i2c_delay(dev);
    gpio_direction_output(dev->data.sda_pin, 1); 
    gpio_i2c_delay(dev);
	
    return 0;  
}

static void IIC_Ack(struct gpio_i2c_dev *dev)
{
    gpio_set_value(dev->data.scl_pin, 0);
    gpio_direction_output(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.sda_pin, 0);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 0);
}

static void IIC_NAck(struct gpio_i2c_dev *dev)
{
    gpio_set_value(dev->data.scl_pin, 0);
    gpio_direction_output(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.sda_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 0);
}					 				     

static void IIC_Send_Byte(struct gpio_i2c_dev *dev, unsigned char txd)
{                        
    unsigned char i;       
    gpio_set_value(dev->data.scl_pin, 0); // start
    gpio_i2c_delay(dev);
    gpio_direction_output(dev->data.sda_pin, 1); 	
    for (i = 0; i < 8; i++)
    {              
        if ((txd & 0x80) >> 7)
            gpio_set_value(dev->data.sda_pin, 1);
        else
            gpio_set_value(dev->data.sda_pin, 0);
        txd <<= 1; 	  
        gpio_i2c_delay(dev);  
        gpio_set_value(dev->data.scl_pin, 1);
        gpio_i2c_delay(dev); 
        gpio_set_value(dev->data.scl_pin, 0);	
        if (i == 7)
		{
            ndelay(500); 
            gpio_direction_input(dev->data.sda_pin);
        }
		else
		{
            gpio_i2c_delay(dev);
        }
    } 
}   

static unsigned char IIC_Read_Byte(struct gpio_i2c_dev *dev, unsigned char ack)
{
    unsigned char i, receive = 0;
    gpio_direction_input(dev->data.sda_pin); // SDA input
    for (i = 0; i < 8; i++)
    {
        gpio_set_value(dev->data.scl_pin, 0); 
        gpio_i2c_delay(dev);
        gpio_set_value(dev->data.scl_pin, 1);
        receive <<= 1;
        if (gpio_get_value(dev->data.sda_pin))
            receive++;   
        gpio_i2c_delay(dev); 
    }					 
    if (!ack)
        IIC_NAck(dev);
    else
        IIC_Ack(dev);

    return receive;
}

static int i2c_read_register(struct gpio_i2c_dev *dev, unsigned char device_addr, unsigned char offset, unsigned char *buf, int len)
{
    int i = 0;
    int ack = 1;
    
    IIC_Start(dev);  
    IIC_Send_Byte(dev, (device_addr & 0xfe)); // send address
    ack = IIC_Wait_Ack(dev); 
    if (ack)
    	return -1;

    IIC_Send_Byte(dev, offset);
    ack = IIC_Wait_Ack(dev);
    if (ack)
        return -1;

    IIC_Start(dev);
    IIC_Send_Byte(dev, (device_addr & 0xfe) | 0x01);
    ack = IIC_Wait_Ack(dev);
    if (ack)
        return -1;
    for (i = 0; i < len -1; i++)
	{
        buf[i] = IIC_Read_Byte(dev, 1);
    }
    buf[i] = IIC_Read_Byte(dev, 0);
    IIC_Stop(dev);
	
    return 0;
}

static int i2c_write_register(struct gpio_i2c_dev *dev, unsigned char device_addr, unsigned char offset, unsigned char *buf, int len)
{
    int i = 0;
    int ack = 1;

    IIC_Start(dev);  
    IIC_Send_Byte(dev, device_addr);
    ack = IIC_Wait_Ack(dev);
    if (ack)
        return -1;

    IIC_Send_Byte(dev, offset);
    ack = IIC_Wait_Ack(dev);
    if (ack)
        return -1;

    for (i = 0; i < len; i++)
	{
        IIC_Send_Byte(dev, buf[i]);
        ack = IIC_Wait_Ack(dev);
        if (ack)
            return -1;
    }
    IIC_Stop(dev);
	
    return 0;
}

static int
isl1208_rtc_get_sr(struct gpio_i2c_dev *dev)
{
	int ret;
	unsigned char data;

	ret = i2c_read_register(dev, ISL1219_ADDR, ISL1208_REG_SR, &data, sizeof(data));
	if (ret < 0)
		return ret;
	
	return 0;
}

static int
isl1208_rtc_validate(struct gpio_i2c_dev *dev)
{
	int i;
	int ret;
	u8 regs[ISL1208_RTC_SECTION_LEN] = { 0, };
	u8 zero_mask[ISL1208_RTC_SECTION_LEN] = {
		0x80, 0x80, 0x40, 0xc0, 0xe0, 0x00, 0xf8
	};

	ret = i2c_read_register(dev, ISL1219_ADDR, 0, regs, ISL1208_RTC_SECTION_LEN);
	if (ret < 0)
		return ret;

	for (i = 0; i < ISL1208_RTC_SECTION_LEN; ++i) {
		if (regs[i] & zero_mask[i])	/* check if bits are cleared */
			return -ENODEV;
	}

	return 0;
}

static int
isl1208_rtc_read_time(struct gpio_i2c_dev *dev, struct rtc_time *tm)
{
	int sr;
	u8 regs[ISL1208_RTC_SECTION_LEN] = { 0, };

	sr = isl1208_rtc_get_sr(dev);
	if (sr < 0) {
		pr_err("%s: reading SR failed\n", __func__);
		return -EIO;
	}

	sr = i2c_read_register(dev, ISL1219_ADDR, 0, regs, ISL1208_RTC_SECTION_LEN);
	if (sr < 0) {
		pr_err("%s: reading RTC section failed\n", __func__);
		return sr;
	}

	tm->tm_sec = bcd2bin(regs[ISL1208_REG_SC]);
	tm->tm_min = bcd2bin(regs[ISL1208_REG_MN]);

	/* HR field has a more complex interpretation */
	{
		const u8 _hr = regs[ISL1208_REG_HR];
		if (_hr & ISL1208_REG_HR_MIL)	/* 24h format */
			tm->tm_hour = bcd2bin(_hr & 0x3f);
		else {
			/* 12h format */
			tm->tm_hour = bcd2bin(_hr & 0x1f);
			if (_hr & ISL1208_REG_HR_PM)	/* PM flag set */
				tm->tm_hour += 12;
		}
	}

	tm->tm_mday = bcd2bin(regs[ISL1208_REG_DT]);
	tm->tm_mon = bcd2bin(regs[ISL1208_REG_MO]) - 1;	/* rtc starts at 1 */
	tm->tm_year = bcd2bin(regs[ISL1208_REG_YR]) + 100;
	tm->tm_wday = bcd2bin(regs[ISL1208_REG_DW]);

	return 0;
}

static int
isl1208_rtc_set_time(struct gpio_i2c_dev *dev, struct rtc_time const *tm)
{
	int sr;
	unsigned char data;	
	u8 regs[ISL1208_RTC_SECTION_LEN] = { 0, };

	/* The clock has an 8 bit wide bcd-coded register (they never learn)
	 * for the year. tm_year is an offset from 1900 and we are interested
	 * in the 2000-2099 range, so any value less than 100 is invalid.
	 */
	if (tm->tm_year < 100)
		return -EINVAL;

	regs[ISL1208_REG_SC] = bin2bcd(tm->tm_sec);
	regs[ISL1208_REG_MN] = bin2bcd(tm->tm_min);
	regs[ISL1208_REG_HR] = bin2bcd(tm->tm_hour) | ISL1208_REG_HR_MIL;

	regs[ISL1208_REG_DT] = bin2bcd(tm->tm_mday);
	regs[ISL1208_REG_MO] = bin2bcd(tm->tm_mon + 1);
	regs[ISL1208_REG_YR] = bin2bcd(tm->tm_year - 100);

	regs[ISL1208_REG_DW] = bin2bcd(tm->tm_wday & 7);

	// set WRTC
	sr = isl1208_rtc_get_sr(dev);
	if (sr < 0) {
		pr_err("%s: reading SR failed\n", __func__);
		return sr;
	}

	data = sr | ISL1208_REG_SR_WRTC;
	sr = i2c_write_register(dev, ISL1219_ADDR, ISL1208_REG_SR, &data, sizeof(data));
	if (sr < 0) {
		pr_err("%s: writing SR failed\n", __func__);
		return sr;
	}

	// write RTC registers 
	sr = i2c_write_register(dev, ISL1219_ADDR, 0, regs, ISL1208_RTC_SECTION_LEN);
	if (sr < 0) {
		pr_err("%s: writing RTC section failed\n", __func__);
		return sr;
	}

	// clear WRTC again
	sr = isl1208_rtc_get_sr(dev);
	if (sr < 0) {
		pr_err("%s: reading SR failed\n", __func__);
		return sr;
	}

	data = sr & ~ISL1208_REG_SR_WRTC;
	sr = i2c_write_register(dev, ISL1219_ADDR, ISL1208_REG_SR, &data, sizeof(data));
	if (sr < 0) {
		pr_err("%s: writing SR failed\n", __func__);
		return sr;
	}

	return 0;
}

static int i2c_gpio_request(struct gpio_i2c_dev *dev)
{
    int err;

	if (!ht1382_i2c_interface)
	{
	    err = gpio_request(GPIO_HT1382_CE, "ht1382_ce");
	    if (err)
	        goto err_request1;
	    gpio_direction_input(GPIO_HT1382_CE);
	}

    err = gpio_request(dev->data.scl_pin, "ht1382_clk");
    if (err)
        goto err_request2;
    gpio_direction_output(dev->data.scl_pin, 0);

    err = gpio_request(dev->data.sda_pin, "ht1382_dio");
    if (err)
        goto err_request3;
    gpio_direction_output(dev->data.sda_pin, 1);

    return 0;

err_request3:
    gpio_free(dev->data.scl_pin);
err_request2:
	if (!ht1382_i2c_interface)
    	gpio_free(GPIO_HT1382_CE);
err_request1:

    return err;
}

static int of_i2c_gpio_get_pins(struct device_node *np,
				unsigned int *sda_pin, unsigned int *scl_pin)
{
	if (of_gpio_count(np) < 2)
		return -ENODEV;

	*sda_pin = of_get_gpio(np, 0);
	*scl_pin = of_get_gpio(np, 1);

	if (*sda_pin == -EPROBE_DEFER || *scl_pin == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (!gpio_is_valid(*sda_pin) || !gpio_is_valid(*scl_pin)) {
		pr_err("%s: invalid GPIO pins, sda=%d/scl=%d\n",
		       np->full_name, *sda_pin, *scl_pin);
		return -ENODEV;
	}

	return 0;
}

static int of_i2c_gpio_get_props(struct device_node *np,
				  struct gpio_i2c_dev *pdata)
{
	return of_property_read_u32(np, "i2c-gpio,delay-us", &(pdata->data.udelay));
}

static int ht1382_ce_gpio_request(void)
{
    gpio_direction_output(GPIO_HT1382_CE, 0);

    return 0;
}

static void HT1382_SendByte(unsigned short nCommand, unsigned char* nData, int nByteCount)
{   
    int i, j;
    
    gpio_direction_output(GPIO_HT1382_DIO, 1);
    // Set CE/CLK to low
    gpio_set_value(GPIO_HT1382_CE, 0);
    gpio_set_value(GPIO_HT1382_CLK, 0);
    udelay(1);
    // Set CE to high
    gpio_set_value(GPIO_HT1382_CE, 1);
    udelay(4);
    
    // Send command
    for (i = 0; i < 8; i++)
    {
        if (nCommand & (1 << i))      
            gpio_set_value(GPIO_HT1382_DIO, 1); // set data to 1
        else
            gpio_set_value(GPIO_HT1382_DIO, 0); // set data to 0

        udelay(4);  
        gpio_set_value(GPIO_HT1382_CLK, 1); // clock to high (SET SCLK) 
        udelay(4);          
        gpio_set_value(GPIO_HT1382_CLK, 0); // clock to low (CLR SCLK)
        udelay(4);                  
    }
    
    // Send data
    for (i = 0; i < nByteCount; i++)
    {
        for (j = 0; j < 8; j++)
        {
            if ((*nData) & (1 << j))
                gpio_set_value(GPIO_HT1382_DIO, 1); // set data to 1
            else
                gpio_set_value(GPIO_HT1382_DIO, 0); // set data to 0
                
            udelay(4);          
            gpio_set_value(GPIO_HT1382_CLK, 1); // clock to high (SET SCLK)        
            udelay(4);  
            
            // Check is last cycle
            if ((i == (nByteCount - 1)) && (j == 7))
            {
                gpio_set_value(GPIO_HT1382_CE, 0); // clear CE pin (CLR CE)
                udelay(1);
                gpio_set_value(GPIO_HT1382_CLK, 0); // clock to low (CLR SCLK)
                udelay(6);          
                gpio_set_value(GPIO_HT1382_CE, 1); // set to high (SET CE)
            }
            else
            {
                gpio_set_value(GPIO_HT1382_CLK, 0); // clock to low (CLR SCLK)  
                udelay(4);                  
            }
        }
        nData++;
    }   
    udelay(1);
}

static void HT1382_RecvByte(unsigned short nCommand, unsigned char* nData, int nByteCount)
{
    int i, j;
    
    gpio_direction_output(GPIO_HT1382_DIO, 1);
    // Set CE/CLK to low
    gpio_set_value(GPIO_HT1382_CE, 0);
    gpio_set_value(GPIO_HT1382_CLK, 0);
    udelay(1);
    // Set CE to high
    gpio_set_value(GPIO_HT1382_CE, 1);

    udelay(4);
    
    // Send command
    for (i = 0; i < 8; i++)
    {
        if (nCommand & (1 << i))      
            gpio_set_value(GPIO_HT1382_DIO, 1); // set data to 1
        else
            gpio_set_value(GPIO_HT1382_DIO, 0); // set data to 0

        udelay(4);  
        gpio_set_value(GPIO_HT1382_CLK, 1); // clock to high (SET SCLK)
        udelay(4);          
        gpio_set_value(GPIO_HT1382_CLK, 0); // clock to low (CLR SCLK)
        udelay(4);                  
    } 
    
    // Set DIO as Input
    gpio_direction_input(GPIO_HT1382_DIO);
    udelay(1);
    
    // Receive data
    for (i = 0; i < nByteCount; i++)
    {
        *nData = 0;
        for (j = 0; j < 8; j++)
        {   
            gpio_set_value(GPIO_HT1382_CLK, 1); // clock to high (SET SCLK)
            udelay(4);
        
            if (gpio_get_value(GPIO_HT1382_DIO))
                *nData |= (1 << j);         
                
            // Check is last cycle
            if ((i == (nByteCount - 1)) && (j == 7))
            {
                gpio_set_value(GPIO_HT1382_CE, 0); // clear reset pin (CLR RST)
                udelay(1);
                gpio_set_value(GPIO_HT1382_CLK, 0); // clock to low (CLR SCLK)
                udelay(1);          
            }
            else
            {
                gpio_set_value(GPIO_HT1382_CLK, 0); // clock to low (CLR SCLK)
                udelay(4);
            }   
        }
        nData++;
    }
}

static bool HT1382_WP_Enable(void)
{
	if (ht1382_i2c_interface)
	{
		int rc;
	    unsigned char byTime = 0x80;
	
		rc = i2c_write_register(&rtc_dev, HT1382_ADDR, INX_RESERVE, &byTime, sizeof(byTime));
		if (rc < 0) {
			pr_err("%s: write wp enable failed\n", __func__);
		}
		return rc;
	}
	else
	{
	    unsigned char byTime[8];
	    byTime[0] = 0x80; // enable write protect

	    HT1382_SendByte(COMMAND_WP_ACCESS_WR, byTime, 1);
	    return true;
	}
}

static bool HT1382_WP_Disable(void)
{
	if (ht1382_i2c_interface)
	{
		int rc;
	    unsigned char byTime = 0x00;
	
		rc = i2c_write_register(&rtc_dev, HT1382_ADDR, INX_RESERVE, &byTime, sizeof(byTime));
		if (rc < 0) {
			pr_err("%s: write wp disable failed\n", __func__);
		}
		return rc;
	}
	else
	{
	    unsigned char byTime[8];
	    byTime[0] = 0x00; // disable write protect

	    HT1382_SendByte(COMMAND_WP_ACCESS_WR, byTime, 1);
	    return true;
	}
}

static bool HT1382_OSC_Enable(void)
{
	if (ht1382_i2c_interface)
	{
		int rc;
	    unsigned char byTime = 0x00;

		rc = i2c_read_register(&rtc_dev, HT1382_ADDR, 0x00, &byTime, sizeof(byTime));
		if (rc < 0) {
			pr_err("%s: read sec failed\n", __func__);
			return rc;
		}
	
	    byTime = byTime & 0x7f; // enable OSC	
		rc = i2c_write_register(&rtc_dev, HT1382_ADDR, 0x00, &byTime, sizeof(byTime));
		if (rc < 0) {
			pr_err("%s: write sec failed\n", __func__);
		}
		return rc;
	}
	else
	{
	    unsigned char byTime[8];
    
	    HT1382_RecvByte(COMMAND_OSC_RD_ACCESS, byTime, 1);
	    byTime[0] = byTime[0] & 0x7f;  // enable OSC
	    HT1382_SendByte(COMMAND_OSC_WR_ACCESS, byTime, 1);
	    return true;
	}
}

static bool HT1382_OSC_IsEnabled(void)  
{
	if (ht1382_i2c_interface)
	{
		int rc;
	    unsigned char byTime = 0x80; // clear OSC halt's status.

		rc = i2c_read_register(&rtc_dev, HT1382_ADDR, 0x00, &byTime, sizeof(byTime));
		if (rc < 0) {
			pr_err("%s: read sec failed\n", __func__);
			return false;
		}
	
	    if (byTime & 0x80)
	        return false;
	    else
	        return true;
	}
	else
	{
	    unsigned char byTime[8];
    
	    byTime[0] = 0x80; // clear OSC halt's status.

	    HT1382_RecvByte(COMMAND_OSC_RD_ACCESS, byTime, 1);
    
	    if (byTime[0] & 0x80)
	        return false;
	    else
	        return true;
	}
}

static void HT1382_HW_Init(void)
{
    if (!HT1382_OSC_IsEnabled()) {
        HT1382_WP_Disable();
        HT1382_OSC_Enable();
        mdelay(500);
    }   
    
    HT1382_WP_Disable();
}

//------------------------------------------------------------------------------
static int ht1382_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags;
	struct ht1382 *ht1382_rtc = dev_get_drvdata(dev);
	
	if (HT1382_RTC == rtc_ic)
	{
	    unsigned char byTime[8] = {0};

		spin_lock_irqsave(&ht1382_rtc->lock, flags);
		if (ht1382_i2c_interface)
		{
			int rc;
	
			rc = i2c_read_register(&rtc_dev, HT1382_ADDR, 0x00, byTime, sizeof(byTime));
			if (rc < 0) {
				pr_err("%s: read time failed\n", __func__);
				spin_unlock_irqrestore(&ht1382_rtc->lock, flags);
				return rc;
			}
		}
		else
		{
	    	HT1382_RecvByte(COMMAND_RD_MULTI, byTime, 8);
		}
		spin_unlock_irqrestore(&ht1382_rtc->lock, flags);

	    tm->tm_year      = (unsigned short)(byTime[INX_YEAR]);
	    tm->tm_wday = (unsigned short)(byTime[INX_WEEK]);
	    tm->tm_mon     = (unsigned short)(byTime[INX_MONTH]);
	    tm->tm_mday       = (unsigned short)(byTime[INX_DAY]);
	    byTime[INX_HOUR] &= 0x7F; // use 24Hour mode
	    tm->tm_hour      = (unsigned short)(byTime[INX_HOUR]);
	    tm->tm_min    = (unsigned short)(byTime[INX_MIN]);
	    tm->tm_sec    = (unsigned short)(byTime[INX_SEC]);
		bcd2tm(tm);
/*
		dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
			"mday=%d, mon=%d, year=%d, wday=%d\n",
			__func__,
			tm->tm_sec, tm->tm_min, tm->tm_hour,
			tm->tm_mday, tm->tm_mon + 1, tm->tm_year, tm->tm_wday);
*/
		if (rtc_valid_tm(tm) < 0)
			dev_err(dev, "invalid date\n");
	}
	else
	{
		spin_lock_irqsave(&ht1382_rtc->lock, flags);
		isl1208_rtc_read_time(&rtc_dev, tm);	
		spin_unlock_irqrestore(&ht1382_rtc->lock, flags);
	}

	return 0;
}

static int ht1382_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags;
	struct ht1382 *ht1382_rtc = dev_get_drvdata(dev);
	
	if (HT1382_RTC == rtc_ic)
	{
	   	unsigned char byTime[8] = {0};

		if (tm2bcd(tm) < 0)
			return -EINVAL;
	   	byTime[INX_SEC]     = (unsigned char)(tm->tm_sec);
	   	byTime[INX_MIN]     = (unsigned char)(tm->tm_min);
	   	byTime[INX_HOUR]    = (unsigned char)(tm->tm_hour);
	    byTime[INX_HOUR]    |= 0x80; // use 24Hour mode
	   	byTime[INX_DAY]     = (unsigned char)(tm->tm_mday);
	   	byTime[INX_MONTH]   = (unsigned char)(tm->tm_mon);
	   	byTime[INX_WEEK]    = (unsigned char)(tm->tm_wday);
	   	byTime[INX_YEAR]    = (unsigned char)(tm->tm_year);
	   	byTime[INX_RESERVE] = 0;

		spin_lock_irqsave(&ht1382_rtc->lock, flags);
		if (ht1382_i2c_interface)
		{
			int rc;
	
			rc = i2c_write_register(&rtc_dev, HT1382_ADDR, 0x00, byTime, sizeof(byTime));
			if (rc < 0) {
				pr_err("%s: write time failed\n", __func__);
				spin_unlock_irqrestore(&ht1382_rtc->lock, flags);
				return rc;
			}
		}
		else
		{
	   		HT1382_SendByte(COMMAND_WR_MULTI, byTime, 8);
		}
		spin_unlock_irqrestore(&ht1382_rtc->lock, flags);
	}
	else
	{
		spin_lock_irqsave(&ht1382_rtc->lock, flags);
		isl1208_rtc_set_time(&rtc_dev, tm);	
		spin_unlock_irqrestore(&ht1382_rtc->lock, flags);
	}

	return 0;
}

static const struct rtc_class_ops ht1382_rtc_ops = {
	.read_time	= ht1382_read_time,
	.set_time	= ht1382_set_time,
};

static int ht1382_rtc_probe(struct platform_device *pdev)
{
	struct ht1382 *chip;
	int retval = -EBUSY;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (pdev->dev.of_node) {
		retval = of_i2c_gpio_get_pins(pdev->dev.of_node, &(rtc_dev.data.sda_pin), &(rtc_dev.data.scl_pin));
		if (retval)
		{
			rtc_dev.data.sda_pin = GPIO_HT1382_DIO;
			rtc_dev.data.scl_pin = GPIO_HT1382_CLK;
		}
		
		retval = of_i2c_gpio_get_props(pdev->dev.of_node, &rtc_dev);
		if (retval)
		{
			rtc_dev.data.udelay = GPIO_I2C_UDELAY; // default 4us
		}
		
		retval = of_property_read_bool(pdev->dev.of_node, "ht1382-i2c-interface");
		if (retval)
		{
			ht1382_i2c_interface = 1;
		}
		else
		{
			ht1382_i2c_interface = 0;
		}	
	} 

	spin_lock_init(&chip->lock);
	
	retval = i2c_gpio_request(&rtc_dev);
	if (retval)
		goto err_gpio;

	if (isl1208_rtc_validate(&rtc_dev) < 0)
	{
		rtc_ic = HT1382_RTC;
		//printk("RTC is HT1382.\n");
	}
	else
	{
		rtc_ic = ISL1219_RTC;
		//printk("RTC is ISL1219.\n");
	}
#if defined(CONFIG_MACH_ECU150) || defined(CONFIG_MACH_ECU150FL) || defined(CONFIG_MACH_ECU150A1) || defined(CONFIG_MACH_ECU150F) || defined(CONFIG_MACH_ECU1370)
	rtc_ic = HT1382_RTC;
#endif
		
	if (HT1382_RTC == rtc_ic)
	{
		if (ht1382_i2c_interface)
	    	HT1382_HW_Init();
		else
		{
			retval = ht1382_ce_gpio_request();
			if (retval)
				goto err_gpio1;

		    HT1382_HW_Init();
		}
	}
	
	platform_set_drvdata(pdev, chip);

	chip->rtc = devm_rtc_device_register(&pdev->dev, "ht1382-rtc",
				&ht1382_rtc_ops, THIS_MODULE);
	if (IS_ERR(chip->rtc)) {
		retval = PTR_ERR(chip->rtc);
		goto err_gpio2;
	}

	return 0;

err_gpio2:
	if (!ht1382_i2c_interface)
		gpio_free(GPIO_HT1382_CE);
err_gpio1:
	gpio_free(rtc_dev.data.sda_pin);
	gpio_free(rtc_dev.data.scl_pin);
err_gpio:
    devm_kfree(&pdev->dev, chip);

	return retval;
}

static int ht1382_rtc_remove(struct platform_device *pdev)
{
    struct ht1382 *chip;
    chip = platform_get_drvdata(pdev);

	if (!ht1382_i2c_interface)
		gpio_free(GPIO_HT1382_CE);
	gpio_free(rtc_dev.data.sda_pin);
	gpio_free(rtc_dev.data.scl_pin);	
    devm_kfree(&pdev->dev, chip);
	
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ht1382_rtc_dt_ids[] = {
	{ .compatible = "holtek,ht1382-rtc" },
	{}
};
MODULE_DEVICE_TABLE(of, ht1382_rtc_dt_ids);
#endif

static struct platform_driver ht1382_rtc_device_driver = {
	.probe	= ht1382_rtc_probe,
	.remove = ht1382_rtc_remove,
	.driver = {
		.name	= "ht1382-rtc",
		.of_match_table = of_match_ptr(ht1382_rtc_dt_ids),
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(ht1382_rtc_device_driver);

MODULE_DESCRIPTION("HT1382 RTC driver");
MODULE_AUTHOR("Advantech");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ht1382-rtc");
