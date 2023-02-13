/*
 * lunix-chrdev.h
 *
 * Definition file for the
 * Lunix:TNG character device
 *
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 * Ioannis Asprogerakas <el18942@mail.ntua.gr>
 * Angelos-Nikolaos Kanatas <el19169@mail.ntua.gr>
 */

#ifndef _LUNIX_CHRDEV_H
#define _LUNIX_CHRDEV_H

/*
 * Lunix:TNG character device
 */
#define LUNIX_CHRDEV_MAJOR	    60	    /* Reserved for local / experimental use */
#define LUNIX_CHRDEV_BUFSZ      20      /* Buffer size used to hold textual info */

/* Compile-time parameters */

#ifdef __KERNEL__ 

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#include "lunix.h"

/*
 * Private state for an open character device node
 */
struct lunix_chrdev_state_struct {
    /* 
     * Opened devices are associated with a sensor
     * and a type of measurement
     */
	enum lunix_msr_enum type;
	struct lunix_sensor_struct *sensor;

	/* A buffer used to hold cached textual info */
	int buf_lim;
	unsigned char buf_data[LUNIX_CHRDEV_BUFSZ];
	uint32_t buf_timestamp;

    /* For mutual exclusion between threads with the same open file description*/
	struct semaphore lock;

    /* Blocking and nonblocking operation */
	int nonblock_mode;

    /* Raw / cooked mode (default is cooked) */
    atomic_t raw_mode; 
};

/*
 * Function prototypes
 */
int lunix_chrdev_init(void);
void lunix_chrdev_destroy(void);

#endif	/* __KERNEL__ */

#include <linux/ioctl.h>

/*
 * Definition of ioctl commands
 */
#define LUNIX_IOC_MAGIC			    LUNIX_CHRDEV_MAJOR

#define LUNIX_IOC_RAWDATA		    _IO(LUNIX_IOC_MAGIC, 0)
#define LUNIX_IOC_COOKEDDATA        _IO(LUNIX_IOC_MAGIC, 1)

#define LUNIX_IOC_MAXNR			    2	

#endif	/* _LUNIX_CHRDEV_H */

