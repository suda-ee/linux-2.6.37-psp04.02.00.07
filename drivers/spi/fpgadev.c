/*
 * fpgadev.c -- simple userspace interface to FPGA suda SPI device
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 * Copyright (C) 2013 Wenfeng CAI <caiwenfeng@suda.edu.cn> (from spidev.c)
 *
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>

#include <asm/uaccess.h>

/* We'll use our own macros for printk */
#define dbg(format, arg...) do { if (debug) pr_info("fpgadev: %s: " format , __FUNCTION__ , ## arg); } while (0)
#define err(format, arg...) pr_err("fpgadev: " format, ## arg)
#define info(format, arg...) pr_info("fpgadev: " format, ## arg)
#define warn(format, arg...) pr_warn("fpgadev: " format, ## arg)

/*
 * This supports acccess to SPI devices using normal userspace I/O calls.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/fpgadevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define FPGADEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

struct fpgadev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/* Use a Kernel FIFO for read operations */
static DECLARE_KFIFO(fpga_msg_fifo, char, 1024);

/* Module parameters that can be provided on insmod */
static bool debug = false;	/* print extra debug info */
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "enable debug info (default: false)");
static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void fpgadev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
fpgadev_sync(struct fpgadev_data *fpgadev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = fpgadev_complete;
	message->context = &done;

	spin_lock_irq(&fpgadev->spi_lock);
	if (fpgadev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(fpgadev->spi, message);
	spin_unlock_irq(&fpgadev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
fpgadev_sync_read(struct fpgadev_data *fpgadev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= fpgadev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return fpgadev_sync(fpgadev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
fpgadev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t			status = 0;
	int			cpstatus;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	/* mutex_lock(&fpgadev->buf_lock); */
	cpstatus = kfifo_to_user(&fpga_msg_fifo, buf, count, &status);
	if (cpstatus != 0) {
		err("can't copy fifo to user.\n");
	}
	/* mutex_unlock(&fpgadev->buf_lock); */

	return status;
}

static long
fpgadev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct fpgadev_data	*fpgadev;
	struct spi_device	*spi;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fpgadev = filp->private_data;
	spin_lock_irq(&fpgadev->spi_lock);
	spi = spi_dev_get(fpgadev->spi);
	spin_unlock_irq(&fpgadev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&fpgadev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	default:
		retval = -ENOTTY;
		break;
	}

	mutex_unlock(&fpgadev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

static int fpgadev_open(struct inode *inode, struct file *filp)
{
	struct fpgadev_data	*fpgadev;
	int			status = -ENXIO;

	/* This device does not allow write access */
	if ( ((filp->f_flags & O_ACCMODE) == O_WRONLY)
	  || ((filp->f_flags & O_ACCMODE) == O_RDWR) ) {
		warn("write access is prohibited\n");
		return -EACCES;
	}

	mutex_lock(&device_list_lock);

	list_for_each_entry(fpgadev, &device_list, device_entry) {
		if (fpgadev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		fpgadev->users++;
		filp->private_data = fpgadev;
		nonseekable_open(inode, filp);
	} else
		pr_debug("fpgadev: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int fpgadev_release(struct inode *inode, struct file *filp)
{
	struct fpgadev_data	*fpgadev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	fpgadev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	fpgadev->users--;
	if (!fpgadev->users) {
		int		dofree;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&fpgadev->spi_lock);
		dofree = (fpgadev->spi == NULL);
		spin_unlock_irq(&fpgadev->spi_lock);

		if (dofree)
			kfree(fpgadev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations fpgadev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.read =		fpgadev_read,
	.unlocked_ioctl = fpgadev_ioctl,
	.open =		fpgadev_open,
	.release =	fpgadev_release,
	.llseek =	no_llseek,
};

static irqreturn_t receive_data(int irq, void *dev_id)
{
	struct fpgadev_data	*fpgadev;
	ssize_t			status = 0;
	/* test only */
	static char bbb = 'A';

	fpgadev = (struct fpgadev_data*) dev_id;

	/* mutex_lock(&fpgadev->buf_lock); */
	/* status = fpgadev_sync_read(fpgadev, 160); */
	if (status > 0) {
		unsigned long	missing;

		/* missing = kfifo_in(&fpga_msg_fifo, fpgadev->buffer, status);*/
		/* test only */
		missing = kfifo_in(&fpga_msg_fifo, &bbb, status);
		bbb++;
		/* if (missing != status)
			status = -EFAULT; */
	}
	/* mutex_unlock(&fpgadev->buf_lock); */

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/fpgadevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *fpgadev_class;

/*-------------------------------------------------------------------------*/

static int __devinit fpgadev_probe(struct spi_device *spi)
{
	struct fpgadev_data	*fpgadev;
	int			status;
	unsigned long		minor;
	int 			irqctxt;

	/* Allocate driver data */
	fpgadev = kzalloc(sizeof(*fpgadev), GFP_KERNEL);
	if (!fpgadev)
		return -ENOMEM;

	/* struct CHIP_platform_data	*pdata; */

	/* assuming the driver requires board-specific data: */
	/* pdata = &spi->dev.platform_data;
	if (!pdata)
		return -ENODEV; */

	/* Initialize the driver data */
	fpgadev->spi = spi;
	spin_lock_init(&fpgadev->spi_lock);
	mutex_init(&fpgadev->buf_lock);

	INIT_LIST_HEAD(&fpgadev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		fpgadev->devt = MKDEV(FPGADEV_MAJOR, minor);
		dev = device_create(fpgadev_class, &spi->dev, fpgadev->devt,
				    fpgadev, "fpgadev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&fpgadev->device_entry, &device_list);
	}
	if (!fpgadev->buffer) {
		fpgadev->buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!fpgadev->buffer) {
			dev_dbg(&fpgadev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
		}
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		spi_set_drvdata(spi, fpgadev);
		dbg("irq number: %d\n", spi->irq);
		irqctxt = request_any_context_irq(spi->irq, receive_data,
				IRQF_TRIGGER_RISING, "fpgadev", fpgadev);
	} else
		kfree(fpgadev);

	return status;
}

static int __devexit fpgadev_remove(struct spi_device *spi)
{
	struct fpgadev_data	*fpgadev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&fpgadev->spi_lock);
	disable_irq(spi->irq);
	free_irq(spi->irq, fpgadev);
	fpgadev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&fpgadev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);

	kfree(fpgadev->buffer);
	fpgadev->buffer = NULL;

	list_del(&fpgadev->device_entry);
	device_destroy(fpgadev_class, fpgadev->devt);
	clear_bit(MINOR(fpgadev->devt), minors);
	if (fpgadev->users == 0)
		kfree(fpgadev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver fpgadev_spi_driver = {
	.driver = {
		.name =		"fpgadev",
		.owner =	THIS_MODULE,
	},
	.probe =	fpgadev_probe,
	.remove =	__devexit_p(fpgadev_remove),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init fpgadev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(FPGADEV_MAJOR, "spi", &fpgadev_fops);
	if (status < 0)
		return status;

	fpgadev_class = class_create(THIS_MODULE, "fpgadev");
	if (IS_ERR(fpgadev_class)) {
		unregister_chrdev(FPGADEV_MAJOR, fpgadev_spi_driver.driver.name);
		return PTR_ERR(fpgadev_class);
	}

	status = spi_register_driver(&fpgadev_spi_driver);
	if (status < 0) {
		class_destroy(fpgadev_class);
		unregister_chrdev(FPGADEV_MAJOR, fpgadev_spi_driver.driver.name);
	}
	/* This device uses a Kernel FIFO for its read operation */
	INIT_KFIFO(fpga_msg_fifo);

	return status;
}
module_init(fpgadev_init);

static void __exit fpgadev_exit(void)
{
	spi_unregister_driver(&fpgadev_spi_driver);
	class_destroy(fpgadev_class);
	unregister_chrdev(FPGADEV_MAJOR, fpgadev_spi_driver.driver.name);
	kfifo_free(&fpga_msg_fifo);
}
module_exit(fpgadev_exit);

MODULE_AUTHOR("Wenfeng CAI, <caiwenfeng@suda.edu.cn>");
MODULE_DESCRIPTION("FPGA suda SPI device");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fpgadev");
