/*
 * fm25.c -- support SPI FRAMs, such as Cypress FM25 models
 *
 * Copyright (C) 2014 Jiri Prchal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/version.h>

#include <linux/cdev.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/nvmem-provider.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/of.h>
#include <asm/memory.h>


struct fm25_data {
	struct spi_device	*spi;
	struct mutex		lock;
	struct spi_eeprom	chip;
	
	struct bin_attribute	bin;
	unsigned		addrlen;
	int			has_sernum;

	struct nvmem_config nvmem_config;
	struct nvmem_device *nvmem;

	struct cdev cdev;

};

static DEFINE_MUTEX(fm25_mutex);


#define	FM25_WREN	0x06		/* latch the write enable */
#define	FM25_WRDI	0x04		/* reset the write enable */
#define	FM25_RDSR	0x05		/* read status register */
#define	FM25_WRSR	0x01		/* write status register */
#define	FM25_READ	0x03		/* read byte(s) */
#define	FM25_WRITE	0x02		/* write byte(s)/sector */
#define	FM25_SLEEP	0xb9		/* enter sleep mode */
#define	FM25_RDID	0x9f		/* read device ID */
#define	FM25_RDSN	0xc3		/* read S/N */

#define	FM25_SR_WEN	0x02		/* write enable (latched) */
#define	FM25_SR_BP0	0x04		/* BP for software writeprotect */
#define	FM25_SR_BP1	0x08
#define	FM25_SR_WPEN	0x80		/* writeprotect enable */

#define	FM25_ID_LEN	9		/* ID lenght */
#define	FM25_SN_LEN	8		/* serial number lenght */

#define	FM25_MAXADDRLEN	3		/* 24 bit addresses */

#define	io_limit	PAGE_SIZE	/* bytes */





int fm25_data_read( void *priv, unsigned int offset, void *val, size_t count)
{
	u8			command[FM25_MAXADDRLEN + 1];
	u8			*cp;
	ssize_t			status;
	struct spi_transfer	t[2];
	struct spi_message	m;
	u8			instr;

	struct fm25_data *fm25 = priv;
	char *buf = val;

	if (unlikely(offset >= fm25->bin.size))
		return 0;
	if ((offset + count) > fm25->bin.size)
		count = fm25->bin.size - offset;
	if (unlikely(!count))
		return count;

	cp = command;

	instr = FM25_READ;
	*cp++ = instr;

	/* 8/16/24-bit address is written MSB first */
	switch (fm25->addrlen) {
	default:	/* case 3 */
		*cp++ = offset >> 16;
	case 2:
		*cp++ = offset >> 8;
	case 1:
	case 0:	/* can't happen: for better codegen */
		*cp++ = offset >> 0;
	}

	spi_message_init(&m);
	memset(t, 0, sizeof t);

	t[0].tx_buf = command;
	t[0].len = fm25->addrlen + 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&fm25->lock);

	/* Read it all at once.
	 *
	 * REVISIT that's potentially a problem with large chips, if
	 * other devices on the bus need to be accessed regularly or
	 * this chip is clocked very slowly
	 */
	status = spi_sync(fm25->spi, &m);
	dev_dbg(&fm25->spi->dev,
		"read %zu bytes at %d --> %d\n",
		count, offset, (int) status);

	mutex_unlock(&fm25->lock);
	return status ? status : count;
}

static ssize_t 
fm25_id_read(struct fm25_data *fm25, char *buf)
{
	u8			command = FM25_RDID;
	ssize_t			status;
	struct spi_transfer	t[2];
	struct spi_message	m;

	spi_message_init(&m);
	memset(t, 0, sizeof t);

	t[0].tx_buf = &command;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = FM25_ID_LEN;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&fm25->lock);

	status = spi_sync(fm25->spi, &m);
	dev_dbg(&fm25->spi->dev,
		"read %d bytes of ID --> %d\n",
	 FM25_ID_LEN, (int) status);

	mutex_unlock(&fm25->lock);
	return status ? status : FM25_ID_LEN;
}

static ssize_t 
fm25_sernum_read(struct fm25_data *fm25, char *buf)
{
	u8			command = FM25_RDSN;
	ssize_t			status;
	struct spi_transfer	t[2];
	struct spi_message	m;

	spi_message_init(&m);
	memset(t, 0, sizeof t);

	t[0].tx_buf = &command;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = FM25_SN_LEN;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&fm25->lock);

	status = spi_sync(fm25->spi, &m);
	dev_info(&fm25->spi->dev,
		"read %d bytes of serial number --> %d\n",
		FM25_SN_LEN, (int) status);

	mutex_unlock(&fm25->lock);
	return status ? status : FM25_SN_LEN;
}

static ssize_t
sernum_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char			binbuf[FM25_SN_LEN];
	struct fm25_data	*fm25;
	int			i;
	char			*pbuf = buf;

	fm25 = dev_get_drvdata(dev);
	fm25_sernum_read(fm25, binbuf);
	for (i = 0; i < FM25_SN_LEN; i++)
		pbuf += sprintf(pbuf, "%02x ", binbuf[i]);
	sprintf(--pbuf, "\n");
	return (3 * i);
}
static const DEVICE_ATTR_RO(sernum);

static ssize_t
fm25_bin_read(struct file *filp, struct kobject *kobj,
	      struct bin_attribute *bin_attr,
	      char *buf, loff_t off, size_t count)
{
	struct device		*dev;
	struct fm25_data	*fm25;

	dev = container_of(kobj, struct device, kobj);
	fm25 = dev_get_drvdata(dev);

	return fm25_data_read(fm25, off, buf, count);
}

int fm25_data_write( void *priv, unsigned int off, void *val, size_t count)
{
	ssize_t			status = 0;
	unsigned		written = 0;
	unsigned		buf_size;
	u8			*bounce;

	struct fm25_data *fm25 = priv;
	char *buf = val;

	if (unlikely(off >= fm25->bin.size))
		return -EFBIG;
	if ((off + count) > fm25->bin.size)
		count = fm25->bin.size - off;
	if (unlikely(!count))
		return count;

	/* Temp buffer starts with command and address */
	buf_size = io_limit;
	bounce = kmalloc(buf_size + fm25->addrlen + 1, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;


	/* For write, rollover is within the page ... so we write at
	 * most one page, then manually roll over to the next page.
	 */
	mutex_lock(&fm25->lock);
	do {
		unsigned	segment;
		unsigned	offset = (unsigned) off;
		u8		*cp = bounce;
		u8		instr;

		*cp = FM25_WREN;
		status = spi_write(fm25->spi, cp, 1);
		if (status < 0) {
			dev_dbg(&fm25->spi->dev, "WREN --> %d\n",
					(int) status);
			break;
		}

		instr = FM25_WRITE;
		*cp++ = instr;

		/* 8/16/24-bit address is written MSB first */
		switch (fm25->addrlen) {
		default:	/* case 3 */
			*cp++ = offset >> 16;
		case 2:
			*cp++ = offset >> 8;
		case 1:
		case 0:	/* can't happen: for better codegen */
			*cp++ = offset >> 0;
		}

		/* Write as much of a page as we can */
		segment = buf_size - (offset % buf_size);
		if (segment > count)
			segment = count;

		memcpy(cp, buf, segment);
		
		status = spi_write(fm25->spi, bounce,
				segment + fm25->addrlen + 1);
		
		dev_dbg(&fm25->spi->dev,
				"write %u bytes at %u --> %d\n",
				segment, offset, (int) status);
		if (status < 0)
			break;

		/* REVISIT this should detect (or prevent) failed writes
		 * to readonly sections of the EEPROM...
		 */

		off += segment;
		buf += segment;
		count -= segment;
		written += segment;

	} while (count > 0);

	mutex_unlock(&fm25->lock);

	kfree(bounce);
	return written ? written : status;
}

static ssize_t
fm25_bin_write(struct file *filp, struct kobject *kobj,
	       struct bin_attribute *bin_attr,
	       char *buf, loff_t off, size_t count)
{
	struct device		*dev;
	struct fm25_data	*fm25;

	dev = container_of(kobj, struct device, kobj);
	fm25 = dev_get_drvdata(dev);

	return fm25_data_write(fm25, off, buf, count);
}


/*-------------------------------------------------------------------------*/

static int fm25_np_to_chip(struct device *dev,
			   struct device_node *np,
			   struct spi_eeprom *chip)
{
	memset(chip, 0, sizeof(*chip));
	strncpy(chip->name, np->name, sizeof(chip->name));

	if (of_find_property(np, "read-only", NULL))
		chip->flags |= EE_READONLY;
	return 0;
}

			   
#if 1
#define V_FRAM_SIZE 128*1024  //32 * 4096

static char*buffer=NULL;
static char*buffer_area=NULL;

static struct class *cls;
static int major;

static int fm25_device_open(struct inode *inode, struct file *filp)
{
   struct fm25_data *fm25 = container_of(inode->i_cdev, struct fm25_data, cdev);

   filp->private_data = fm25;

   fm25_data_read( fm25, 0, buffer, V_FRAM_SIZE);

   return 0;
}

static int fm25_device_release(struct inode *inode, struct file *filp)
{
	struct fm25_data *fm25 = filp->private_data;
	fm25_data_write( fm25, 0, buffer, V_FRAM_SIZE);
	return 0;
}

static int fm25_device_mmap(struct file *filp, struct vm_area_struct *vma)
{
   int ret;
   ret = remap_pfn_range(vma,
		   vma->vm_start,
		   virt_to_phys((void*)((unsigned long)buffer_area)) >> PAGE_SHIFT,
		   vma->vm_end-vma->vm_start,
		   PAGE_SHARED);
   if(ret != 0) {
	   return -EAGAIN;
   }
   return 0;
}

static loff_t fm25_device_llseek(struct file *file, loff_t offset, int whence)
{
	loff_t newPos = 0;
	
    switch (whence) {
    case SEEK_SET:
		newPos = offset;
        break;
    case SEEK_CUR:
		newPos = file->f_pos + offset;
        break;
    case SEEK_END:
		newPos = V_FRAM_SIZE + offset;	
        break;
    default:
        offset = -1;
    }
    if (offset < 0 || newPos < 0)
        return -EINVAL;

    file->f_pos = newPos;

    return file->f_pos;

}


static ssize_t 
fm25_device_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{

   int ret = 0;
   loff_t offset = *f_pos;   // *f_ops , 就是文件指针的位置，lseek设置的值，驱动里一定要实现llseek
	struct fm25_data *fm25 = filp->private_data;


   if( offset > V_FRAM_SIZE)  //判断文件是否已经读完
	   goto out;
   
   if( offset + count > V_FRAM_SIZE)  //读的数据过大
	   count = V_FRAM_SIZE - offset;

#if 0
   return fm25_data_read( fm25, offset, buff, count);

#else
   fm25_data_read( fm25, offset, buffer + offset, count);
   
   mutex_lock(&fm25_mutex);
   if(copy_to_user(buff, buffer + offset, count))	  
	  {
		  mutex_unlock(&fm25_mutex);
		  ret = -EFAULT;
		  goto out;
	  }
   
   *f_pos += count;    
   ret = count;
   
   mutex_unlock(&fm25_mutex);
#endif
	
out:
   return ret;
}

static ssize_t 
fm25_device_write(struct file *filp, const char __user *buff, size_t count, loff_t * f_pos)
{

	int ret = 0;
	loff_t offset = *f_pos;
	
	struct fm25_data *fm25 = filp->private_data;

#if 0
	if( offset > V_FRAM_SIZE)
		goto out;

	if( offset + count > V_FRAM_SIZE)
		count = V_FRAM_SIZE - offset;
	
	return fm25_data_write( fm25, offset, (void *)buff, count); // const char *buff will error
	
#else

	if( offset > V_FRAM_SIZE)
		goto out;
	
	if( offset + count > V_FRAM_SIZE)
		count = V_FRAM_SIZE - offset;

	mutex_lock(&fm25_mutex);
	if(copy_from_user(buffer + offset, buff, count))    
	   {
	   	   mutex_unlock(&fm25_mutex);
		   ret = -EFAULT;
		   goto out;
	   }

	*f_pos += count;	
	ret = count;
	
	fm25_data_write( fm25, offset, buffer + offset, count);
	
	mutex_unlock(&fm25_mutex);
	
#endif
	
out:
   return ret;
}


static int 
fm25_device_fsync (struct file *filp, loff_t start, loff_t end, int datasync)
{

	struct fm25_data *fm25 = filp->private_data;

	fm25_data_write( fm25, 0, buffer, V_FRAM_SIZE);
	return 0;
}


struct file_operations simple_fm25_fops = {
   .owner	=  THIS_MODULE,
   .open	=  fm25_device_open,
   .mmap	=  fm25_device_mmap,
   .write	=  fm25_device_write,
   .read	=  fm25_device_read,
   .llseek  =  fm25_device_llseek,
   .release =  fm25_device_release,
   .fsync    = fm25_device_fsync,
};


void fm25_device_exit(void)
{
   unsigned long virt_addr;
   dev_t devno = MKDEV(major, 0);
   device_destroy(cls, devno);
   class_destroy(cls);

   unregister_chrdev(major, "fram");

   
   for(virt_addr=(unsigned long)buffer_area; virt_addr<(unsigned long)buffer_area + V_FRAM_SIZE;
		   virt_addr+=PAGE_SIZE)
   {
	   SetPageReserved(virt_to_page(virt_addr));
   }
   if (buffer)
	   kfree(buffer);
}

int fm25_device_init(struct fm25_data *fm25)
{
   int result;
   unsigned long virt_addr;

   dev_t devid;
   alloc_chrdev_region(&devid, 0, 1, "fram");
   printk(KERN_INFO "MAJOR Number is %d\n",MAJOR(devid));
   printk(KERN_INFO "MINOR Number is %d\n",MINOR(devid));

   if (MAJOR(devid) < 0)
   {
	   printk(KERN_WARNING "DEMO: can't get major\n");
	   goto out_free;
   }

   cdev_init(&fm25->cdev, &simple_fm25_fops);
   fm25->cdev.owner = THIS_MODULE;
   fm25->cdev.ops = &simple_fm25_fops;
   cdev_add(&fm25->cdev, devid, 1);

   buffer = kmalloc(V_FRAM_SIZE,GFP_KERNEL);			
   //printk("mmap buffer = %p\n",buffer); 		  

   buffer_area=(char *)(((unsigned long)buffer + 4096 -1) & PAGE_MASK);
   for (virt_addr=(unsigned long)buffer_area; virt_addr<(unsigned long)buffer_area + V_FRAM_SIZE;
		   virt_addr+=PAGE_SIZE)
   {
	   /* reserve all pages to make them remapable */
   	   //将页配置为保留，防止映射到用户空间的页面被swap out出去；

	   SetPageReserved(virt_to_page(virt_addr));
  }


   major = MAJOR(devid);
   cls = class_create(THIS_MODULE, "fram0");
   if(!device_create(cls, NULL, MKDEV(major, 0), NULL, "fram0"))
   {
	   printk("--device_create failed\n");
   }


	/* copy fram data to kernel_mmap_region */
   fm25_data_read( fm25, 0, buffer, V_FRAM_SIZE);

   return 0;

out_free:
   fm25_device_exit();
   return result;
}
			   
#endif 





static int fm25_probe(struct spi_device *spi)
{
	struct fm25_data	*fm25 = NULL;
	struct spi_eeprom	chip;
	struct device_node	*np = spi->dev.of_node;
	int			err;
	char			id[FM25_ID_LEN];

	/* Chip description */
	if (!spi->dev.platform_data) {
		if (np) {
			err = fm25_np_to_chip(&spi->dev, np, &chip);
			if (err)
				return err;
		} else {
			dev_err(&spi->dev, "Error: no chip description\n");
			return -ENODEV;
		}
	} else
		chip = *(struct spi_eeprom *)spi->dev.platform_data;

	fm25 = devm_kzalloc(&spi->dev, sizeof(*fm25), GFP_KERNEL);
	if (!fm25)
		return -ENOMEM;

	mutex_init(&fm25->lock);
	fm25->chip = chip;
	fm25->spi = spi_dev_get(spi);
	spi_set_drvdata(spi, fm25);

	/* Get ID of chip */
	fm25_id_read(fm25, id);
	if (id[6] != 0xc2) {
		dev_err(&spi->dev, "Error: no Cypress FRAM (id %02x)\n", id[6]);
		return -ENODEV;
	}
	/* set size found in ID */
	switch (id[7]) {
	case 0x21:
		fm25->chip.byte_len = 16 * 1024;
		break;
	case 0x22:
		fm25->chip.byte_len = 32 * 1024;
		break;
	case 0x23:
		fm25->chip.byte_len = 64 * 1024;
		break;
	case 0x24:
		fm25->chip.byte_len = 128 * 1024;
		break;
	case 0x25:
		fm25->chip.byte_len = 256 * 1024;
		break;
	default:
		dev_err(&spi->dev, "Error: unsupported size (id %02x)\n", id[7]);
		return -ENODEV;
		break;
	}

	if (fm25->chip.byte_len > 64 * 1024) {
		fm25->addrlen = 3;
		fm25->chip.flags |= EE_ADDR3;
	}
	else {
		fm25->addrlen = 2;
		fm25->chip.flags |= EE_ADDR2;
	}

	if (id[8])
		fm25->has_sernum = 1;
	else
		fm25->has_sernum = 0;

	fm25->chip.page_size = PAGE_SIZE;

	/* Export the EEPROM bytes through sysfs, since that's convenient.
	 * And maybe to other kernel code; it might hold a board's Ethernet
	 * address, or board-specific calibration data generated on the
	 * manufacturing floor.
	 *
	 * Default to root-only access to the data; EEPROMs often hold data
	 * that's sensitive for read and/or write, like ethernet addresses,
	 * security codes, board-specific manufacturing calibrations, etc.
	 */

#if 1
	sysfs_bin_attr_init(&fm25->bin);
	fm25->bin.attr.name = "fram";
	fm25->bin.attr.mode = S_IRUGO;
	fm25->bin.read = fm25_bin_read;

	fm25->bin.size = fm25->chip.byte_len;
	if (!(chip.flags & EE_READONLY)) {
		fm25->bin.write = fm25_bin_write;
		fm25->bin.attr.mode |= S_IWUSR | S_IWGRP | S_IWOTH;
	}

	err = sysfs_create_bin_file(&spi->dev.kobj, &fm25->bin);
	if (err)
		return err;

	/* Export the FM25 serial number */
	if (fm25->has_sernum) {
		err = device_create_file(&spi->dev, &dev_attr_sernum);
		if (err)
			return err;
	}
#endif

	fm25->nvmem_config.name = dev_name(&spi->dev);
	fm25->nvmem_config.dev = &spi->dev;
	fm25->nvmem_config.read_only = chip.flags & EE_READONLY;
	fm25->nvmem_config.root_only = true;  //?
	fm25->nvmem_config.owner = THIS_MODULE;
	fm25->nvmem_config.compat = true;

	fm25->nvmem_config.base_dev = &spi->dev;
	fm25->nvmem_config.reg_read = fm25_data_read;
	fm25->nvmem_config.reg_write = fm25_data_write;
	fm25->nvmem_config.priv = fm25;
	fm25->nvmem_config.stride = 4;
	fm25->nvmem_config.word_size = 1;
	fm25->nvmem_config.size = chip.byte_len;

	fm25->nvmem = nvmem_register(&fm25->nvmem_config);
	if (IS_ERR(fm25->nvmem))
		return PTR_ERR(fm25->nvmem);

	fm25_device_init(fm25);
	
	dev_info(&spi->dev, "%zu %s %s fram%s\n",
		(fm25->bin.size < 1024)
			? fm25->bin.size
			: (fm25->bin.size / 1024),
		(fm25->bin.size < 1024) ? "Byte" : "KByte",
		fm25->chip.name,
		(chip.flags & EE_READONLY) ? " (readonly)" : "");


	return 0;
}

static int fm25_remove(struct spi_device *spi)
{
	struct fm25_data *fm25 = spi_get_drvdata(spi);

	sysfs_remove_bin_file(&spi->dev.kobj, &fm25->bin);
	if (fm25->has_sernum)
		device_remove_file(&spi->dev, &dev_attr_sernum);
	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct of_device_id fm25_of_match[] = {
	{ .compatible = "cypress,fm25", },
	{ }
};
MODULE_DEVICE_TABLE(of, fm25_of_match);

static struct spi_driver fm25_driver = {
	.driver = {
		.name		= "fm25",
		.of_match_table = fm25_of_match,
	},
	.probe		= fm25_probe,
	.remove		= fm25_remove,
};

module_spi_driver(fm25_driver);

MODULE_DESCRIPTION("Driver for Cypress SPI FRAMs");
MODULE_AUTHOR("Jiri Prchal");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fram");
