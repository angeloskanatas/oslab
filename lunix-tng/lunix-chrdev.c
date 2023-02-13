/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 * Ioannis Asprogerakas <el18942@mail.ntua.gr>
 * Angelos-Nikolaos Kanatas <el19169@mail.ntua.gr>
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/page-flags.h>
#include <asm/page.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
static struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *chrdev_state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON (!(sensor = chrdev_state->sensor));
	
	return (sensor->msr_data[chrdev_state->type]->last_update > chrdev_state->buf_timestamp);
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *chrdev_state)
{
	int ret;
	struct lunix_sensor_struct *sensor;
	uint32_t raw_data;
	uint32_t current_timestamp;
	long measurement;
	unsigned char sign;

	WARN_ON(!(sensor = chrdev_state->sensor));

	/*
	 * Any new data available?
	 */
	if (lunix_chrdev_state_needs_refresh(chrdev_state)) {
		/*
	 	* Grab the raw data quickly, hold the
	 	* spinlock for as little as possible.
	 	* We must disable the interrupts in running CPU to prevent deadlocks!
	 	* We use spinlocks because we compete code that runs on interrupt context.
	 	*/
		spin_lock_irq(&sensor->lock);
		raw_data = sensor->msr_data[chrdev_state->type]->values[0];
		current_timestamp = sensor->msr_data[chrdev_state->type]->last_update;
		spin_unlock_irq(&sensor->lock);

		/*
	 	* Now we can take our time to format them,
	 	* holding only the private state semaphore
	 	*/
	 	if(!atomic_read(&chrdev_state->raw_mode)) {
	 		/* COOKED MODE */
	 		switch (chrdev_state->type) {
				case BATT:
					measurement = lookup_voltage[raw_data];
					break;
				case TEMP:
					measurement = lookup_temperature[raw_data];
					break;
				case LIGHT:
					measurement = lookup_light[raw_data];
					break;
				default: //we have checked this in lunix_chrdev_open()
					goto out;
			}

			sign = (measurement >= 0) ? ' ' : '-';
			measurement = (measurement >= 0) ? measurement : -measurement;

			chrdev_state->buf_lim = snprintf(chrdev_state->buf_data, LUNIX_CHRDEV_BUFSZ, "  %c%ld.%03ld", sign, measurement / 1000, measurement % 1000);

	 	} else 
	 		/* RAW MODE */
	 		chrdev_state->buf_lim = snprintf(chrdev_state->buf_data, LUNIX_CHRDEV_BUFSZ, "%u\n", raw_data);

		/* Update the timestamp */
		chrdev_state->buf_timestamp = current_timestamp;

		ret = 0;
	}
	else
		ret = -EAGAIN;
out:
	return ret;
}

/*************************************
 * Implementation of file operations *
 * for the Lunix character device    *
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	struct lunix_chrdev_state_struct *chrdev_state;
	unsigned int sensor, type;
	int ret;

	debug("entering\n");
	ret = -ENODEV;

	/* Write operation is not permitted */
	if ((filp->f_flags & O_WRONLY) || (filp->f_flags & O_RDWR))
		return -EPERM;

	/* Device does not support seeking */
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;
	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	sensor = iminor(inode) / 8;
	type = iminor(inode) % 8;
	
	/* Allocate a new Lunix character device private state structure */
	if (!(chrdev_state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL))) {
		printk(KERN_ERR "Failed to allocate memory for character device private state\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Initialization of the device private state structure */
	chrdev_state->type = type;
	chrdev_state->sensor = &lunix_sensors[sensor];
	chrdev_state->buf_lim = 0;
	chrdev_state->buf_timestamp = 0;
	sema_init(&chrdev_state->lock, 1);
	chrdev_state->nonblock_mode = (filp->f_flags & O_NONBLOCK) ? 1 : 0;
	atomic_set(&chrdev_state->raw_mode, 0);

	/* Save the device state */
	filp->private_data = chrdev_state;
	debug("character device state initialized successfully\n");

	switch (chrdev_state->type) {
		case BATT: 
			debug("/dev/sensor%d-batt opened\n", sensor);
			break;
		case TEMP: 
			debug("/dev/sensor%d-temp opened\n", sensor);
			break;
		case LIGHT: 
			debug("/dev/sensor%d-light opened\n", sensor);
			break;
		default:
			/* Only battery, temperature and light measurements are supported */
			printk(KERN_ERR "This measurement isn't supported\n");
			goto out;
	}

	ret = 0;	
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	WARN_ON(!filp->private_data);
	kfree(filp->private_data);
	debug("character device state memory released\n");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	struct lunix_chrdev_state_struct *chrdev_state;

	chrdev_state = filp->private_data;
	WARN_ON(!chrdev_state);

	debug("entering\n");
	ret = -ENOTTY;

	if (_IOC_TYPE(cmd) != LUNIX_IOC_MAGIC)
		goto out;
	if (_IOC_NR(cmd) >= LUNIX_IOC_MAXNR)
		goto out;

	switch(cmd) {
		/* We use atomic variables instead of semaphores */
		case LUNIX_IOC_RAWDATA: 
			atomic_set(&chrdev_state->raw_mode, 1);
			break;
		case LUNIX_IOC_COOKEDDATA:
			atomic_set(&chrdev_state->raw_mode, 0);
			break;
		default:
			goto out;
	}

	ret = 0; 
out:
	debug("leaving\n");
	return ret;	
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret, remaining_bytes;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *chrdev_state;

	chrdev_state = filp->private_data;
	WARN_ON(!chrdev_state);

	sensor = chrdev_state->sensor;
	WARN_ON(!sensor);

	debug("entering\n");
	/* Acquire the lock */
	if (down_interruptible(&chrdev_state->lock))
		return -ERESTARTSYS;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(chrdev_state) == -EAGAIN) {
			
			/* The process needs to sleep */
			up(&chrdev_state->lock); //release the lock

			if (chrdev_state->nonblock_mode) // check for non-blocking i/o
				return -EAGAIN;

			/* Sleep */
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(chrdev_state)))
				return -ERESTARTSYS;

			if (down_interruptible(&chrdev_state->lock)) //acquire the lock
				return -ERESTARTSYS;	
		}
	}
	
	/* Determine the number of cached bytes to copy to userspace */
	remaining_bytes = chrdev_state->buf_lim - *f_pos;

	cnt = (cnt < remaining_bytes) ? cnt : remaining_bytes;

	if (copy_to_user(usrbuf, chrdev_state->buf_data + *f_pos, cnt)) {
		ret = -EFAULT;
		goto out;
	}

	/* Update the file offset in the open file description */
	*f_pos += cnt;
	
	/* Return the read number of bytes */
	ret = cnt;

	/* Auto-rewind on EOF mode */
	if (*f_pos == chrdev_state->buf_lim)
		*f_pos = 0;
out:
	/* Unlock */
	up(&chrdev_state->lock);
	debug("leaving\n");
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long addr;
	//struct page *page;

	struct lunix_chrdev_state_struct *chrdev_state;
	struct lunix_sensor_struct *sensor;

	chrdev_state = filp->private_data;
	WARN_ON(!chrdev_state);

	sensor = chrdev_state->sensor;
	WARN_ON(!sensor);

	debug("entering\n");
	/* Associate kernel logical address to struct page */
	/* We map the measurement of the sensor that is associated with this open file */
	//page = virt_to_page(sensor->msr_data[chrdev_state->type]->values);
	
	/* Virtual address of the page */
	//addr = (unsigned long) page_address(page);	

	addr = (unsigned long) sensor->msr_data[chrdev_state->type];

	/* Build the new page tables */
	if (remap_pfn_range(vma, vma->vm_start, __pa(addr) >> PAGE_SHIFT,
		PAGE_SIZE, vma->vm_page_prot))
		return -EAGAIN;

	/* FIXME: try using vm_insert_page() instead of remap_pfn_range() */

	debug("leaving\n");
	return 0;
}

static struct file_operations lunix_chrdev_fops = 
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.llseek		= no_llseek,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
