/*
 * ATUSB driver 0.1
 *
 * Copyright (c) 2011 Richard Sharpe (realrichardsharpe@gmail.com)
 * Copyright (c) 2011 Stefan Schmidt <stefan@datenfreihafen.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>

#include <linux/spi/spi.h>
#include <linux/spi/at86rf230.h>

#define DRIVER_AUTHOR "Richard Sharpe, realrichardsharpe@gmail.com"
#define DRIVER_DESC "ATUSB ben-wpan Driver"

#define VENDOR_ID     0x20b7
#define PRODUCT_ID    0x1540

/*
 * The devices we work with ...
 */
static const struct usb_device_id atusb_device_table[] = {
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
	{ },
};

MODULE_DEVICE_TABLE(usb, atusb_device_table);

/*
 * Our device ...
 */
#define ATUSB_BUILD_SIZE 256
struct atusb_local {
	struct usb_device * udev;
	/* The RF part info below */
	uint8_t rf_part_num;
	uint8_t rf_version_num;
	/* The interface to the RF part info, if applicable */
	uint8_t ep0_atusb_major;
	uint8_t ep0_atusb_minor;
	uint8_t atusb_hw_type;
	char atusb_build[ATUSB_BUILD_SIZE + 1];
	struct spi_master *master;
	struct spi_device *spi;
	int slave_irq;
};

/*
 * RF Registers
 */
enum rf_registers {
	RF_PART_NUM_REG			= 0x1c,
	RF_VERSION_NUM_REG		= 0x1d
};

/*
 * Commands to our device. Make sure this is synced with the firmware
 */
enum atspi_requests {
	ATUSB_ID			= 0x00,	/* system status/control grp */
	ATUSB_BUILD,
	ATUSB_RESET,
	ATUSB_RF_RESET			= 0x10,	/* debug/test group */
	ATUSB_POLL_INT,
	ATUSB_TEST,			/* atusb-sil only */
	ATUSB_TIMER,
	ATUSB_GPIO,
	ATUSB_SLP_TR,
	ATUSB_GPIO_CLEANUP,
	ATUSB_REG_WRITE			= 0x20,	/* transceiver group */
	ATUSB_REG_READ,
	ATUSB_BUF_WRITE,
	ATUSB_BUF_READ,
	ATUSB_SRAM_WRITE,
	ATUSB_SRAM_READ,
	ATUSB_SPI_WRITE			= 0x30,	/* SPI group */
	ATUSB_SPI_READ1,
	ATUSB_SPI_READ2,
};

/*
 * Direction	bRequest		wValue		wIndex	wLength
 *
 * ->host	ATUSB_ID		-		-	3
 * ->host	ATUSB_BUILD		-		-	#bytes
 * host->	ATUSB_RESET		-		-	0
 *
 * host->	ATUSB_RF_RESET		-		-	0
 * ->host	ATUSB_POLL_INT		-		-	1
 * host->	ATUSB_TEST		-		-	0
 * ->host	ATUSB_TIMER		-		-	#bytes (6)
 * ->host	ATUSB_GPIO		dir+data	mask+p#	3
 * host->	ATUSB_SLP_TR		-		-	0
 * host->	ATUSB_GPIO_CLEANUP	-		-	0
 *
 * host->	ATUSB_REG_WRITE		value		addr	0
 * ->host	ATUSB_REG_READ		-		addr	1
 * host->	ATUSB_BUF_WRITE		-		-	#bytes
 * ->host	ATUSB_BUF_READ		-		-	#bytes
 * host->	ATUSB_SRAM_WRITE	-		addr	#bytes
 * ->host	ATUSB_SRAM_READ		-		addr	#bytes
 *
 * host->	ATUSB_SPI_WRITE		byte0		byte1	#bytes
 * ->host	ATUSB_SPI_READ1		byte0		-	#bytes
 * ->host	ATUSB_SPI_READ2		byte0		byte1	#bytes
 */

#define ATUSB_FROM_DEV (USB_TYPE_VENDOR | USB_DIR_IN)
#define ATUSB_TO_DEV (USB_TYPE_VENDOR | USB_DIR_OUT)

static ssize_t rf_show_part(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct atusb_local *atusb = usb_get_intfdata(intf);
	char *chip;

	switch(atusb->rf_part_num) {
	case 2:
		chip = "AT86RF230";
		break;
	case 3:
		chip = "AT86RF231";
		break;
	default:
		chip = "Unknown";
	}

	return snprintf(buf, PAGE_SIZE, "%s (%u)\n", chip, atusb->rf_part_num);
}

static DEVICE_ATTR(rf_part_num, S_IRUGO, rf_show_part, NULL);

static ssize_t rf_show_version(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct atusb_local *atusb = usb_get_intfdata(intf);

	return sprintf(buf, "%s\n",
			(atusb->rf_version_num == 1) ?
				"Rev A" : (atusb->rf_version_num == 2) ?
					"Rev B" : "Unknown");
}

static DEVICE_ATTR(rf_version_num, S_IRUGO, rf_show_version, NULL);

static int atusb_setup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct atusb_local *atusb = spi_master_get_devdata(master);

	spi->irq = atusb->slave_irq;
	return 0;
}

static int atusb_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct atusb_local *atusb = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	struct spi_transfer *x[2];
	int n;

//printk(KERN_INFO "xfer, prv %p\n", prv);
	 if (unlikely(list_empty(&msg->transfers))) {
		dev_err(&atusb->udev->dev, "transfer is empty\n");
		return -EINVAL;
	}

	/*
	 * Classify the request. This is just a proof of concept - we don't
	 * need it in this driver.
	 */
	n = 0;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (n == ARRAY_SIZE(x)) {
			dev_err(&atusb->udev->dev, "too many transfers\n");
			return -EINVAL;
		}
		x[n] = xfer;
		n++;
	}

	if (!x[0]->tx_buf || x[0]->len != 2)
		goto bad_req;
	if (n == 1) {
		if (x[0]->rx_buf) {
			dev_dbg(&atusb->udev->dev, "read 1\n");
		} else {
			dev_dbg(&atusb->udev->dev, "write 2\n");
		}
	} else {
		if (x[0]->rx_buf) {
			if (x[1]->tx_buf || !x[1]->rx_buf)
				goto bad_req;
			dev_dbg(&atusb->udev->dev, "read 1+\n");
		} else {
			if (!x[1]->tx_buf ||x[1]->rx_buf)
				goto bad_req;
			dev_dbg(&atusb->udev->dev, "write 2+n\n");
		}
	}

	msg->status = 0;
	msg->actual_length = x[0]->len+(n == 2 ? x[1]->len : 0);
	msg->complete(msg->context);

	return 0;

bad_req:
	dev_err(&atusb->udev->dev, "unrecognized request:\n");
	list_for_each_entry(xfer, &msg->transfers, transfer_list)
		dev_err(&atusb->udev->dev, "%stx %srx len %u\n",
		    xfer->tx_buf ? "" : "!", xfer->rx_buf ? " " : "!",
		    xfer->len);
	return -EINVAL;
}

static void atusb_cleanup(struct spi_device *spi)
{
}

static void atben_reset(void *reset_data)
{
	int retval;
	struct atusb_local *atusb = reset_data;

	retval = usb_control_msg(atusb->udev,
				usb_rcvctrlpipe(atusb->udev, 0),
				ATUSB_RF_RESET,
				ATUSB_TO_DEV,
				0,
				0,
				NULL,
				1, /* 0? */
				1000);
	if (retval < 0) {
		dev_dbg(&atusb->udev->dev,
			"%s: error doing reset retval = %d\n",
			__func__, retval);
	}
}

static int atusb_get_static_info(struct atusb_local *atusb)
{
	int retval;
	unsigned char *buffer;

	buffer = kmalloc(3, GFP_KERNEL);
	if (!buffer) {
		dev_err(&atusb->udev->dev, "out of memory\n");
		retval = -ENOMEM;
		goto out;
	}

	/*
	 * Get the FP part id and version num
	 */
	retval = usb_control_msg(atusb->udev,
				usb_rcvctrlpipe(atusb->udev, 0),
				ATUSB_REG_READ,
				ATUSB_FROM_DEV,
				0x00,
				RF_PART_NUM_REG,
				&atusb->rf_part_num,
				1,
				1000);
	if (retval < 0) {
		dev_dbg(&atusb->udev->dev,
			"%s: error getting BUILD: retval = %d\n",
			__func__,
			retval);
		goto out_free;
	}

	retval = usb_control_msg(atusb->udev,
				usb_rcvctrlpipe(atusb->udev, 0),
				ATUSB_REG_READ,
				ATUSB_FROM_DEV,
				0x00,
				RF_VERSION_NUM_REG,
				&atusb->rf_version_num,
				1,
				1000);
	if (retval < 0) {
		dev_dbg(&atusb->udev->dev,
			"%s: error getting BUILD: retval = %d\n",
			__func__,
			retval);
		goto out_free;
	}

	/*
	 * Get a couple of the ATMega Firmware values as well
	 */
	retval = usb_control_msg(atusb->udev,
				usb_rcvctrlpipe(atusb->udev, 0),
				ATUSB_BUILD,
				ATUSB_FROM_DEV,
				0x00,
				0x00,
				atusb->atusb_build,
				ATUSB_BUILD_SIZE+1,
				1000);
	if (retval < 0) {
		dev_dbg(&atusb->udev->dev,
			"%s: error getting BUILD: retval = %d\n",
			__func__,
			retval);
		goto out_free;
	}

	retval = usb_control_msg(atusb->udev,
				usb_rcvctrlpipe(atusb->udev, 0),
				ATUSB_ID,
				ATUSB_FROM_DEV,
				0x00,
				0x00,
				buffer,
				3,
				1000);

	if (retval == 3) {
		atusb->ep0_atusb_major = buffer[0];
		atusb->ep0_atusb_minor = buffer[1];
		atusb->atusb_hw_type   = buffer[2];
		retval = 0;
	} else
		dev_dbg(&atusb->udev->dev, "%s: retval = %d\n",
			__func__,
			retval);

out_free:
	kfree(buffer);
out:
	return retval;
}

struct at86rf230_platform_data at86rf230_platform_data = {
	.rstn	= -1,
//	.slp_tr	= JZ_GPIO_PORTD(9),
	.dig2	= -1,
//	.reset	= atben_reset,
};

struct spi_board_info atusb_spi_board_info[] = {
	{
		.modalias = "at86rf230",
		.platform_data = &at86rf230_platform_data,
//		.controller_data = (void *)JZ_GPIO_PORTD(13),
		/* set .irq later */
		.chip_select = 0,
		.bus_num = 2,
		.max_speed_hz = 8 * 1000 * 1000,
	},
	};

static int atusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct atusb_local *atusb = NULL;
	struct spi_master *master;
	struct spi_device *spi;
	int retval;

#if 1
	atusb = kzalloc(sizeof(struct atusb_local), GFP_KERNEL);
	if (!atusb)
		return -ENOMEM;
#endif
	atusb->udev = usb_get_dev(udev);
	usb_set_intfdata(interface, atusb);

	master = spi_alloc_master(&atusb->udev->dev, sizeof(*atusb));
	if (!master)
		return -ENOMEM;

	//atusb = spi_master_get_devdata(master);

	atusb->master = master;

	master->mode_bits	= SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bus_num		= 23;
	master->num_chipselect	= 1;
	master->setup		= atusb_setup;
	master->transfer	= atusb_transfer;
	master->cleanup		= atusb_cleanup;

	spi = spi_new_device(master, atusb_spi_board_info);
	atusb->spi = spi;

	/*
	 * Interface 1 is used for DFU. Ignore it in this driver to avoid
	 * attaching to both interfaces
	 */
        if (interface == udev->actconfig->interface[1]) {
                dev_info(&udev->dev,
                         "Ignoring interface 1 reserved for DFU\n");
                return -ENODEV;
        }

	/*
	 * Get the static info from the device and save it ...
	 */
	retval = atusb_get_static_info(atusb);
	if (retval) {
		dev_err(&interface->dev, "%s: Failed to get static info: %d\n",
			__func__,
			retval);
		goto err_master;
	}

	dev_info(&udev->dev, "Firmware: %s\n", atusb->atusb_build);
	dev_info(&udev->dev, "Major: %u, Minor: %u, HW Type: %u\n",
			atusb->ep0_atusb_major, atusb->ep0_atusb_minor,
			atusb->atusb_hw_type);
	/*
	 * Create the sysfs files
	 */
	retval = device_create_file(&interface->dev, &dev_attr_rf_part_num);
	if (retval)
		goto err_master;

	retval = device_create_file(&interface->dev, &dev_attr_rf_version_num);
	if (retval)
		goto err_part;

	retval = spi_register_master(master);
	if (retval)
		goto err_version;

	dev_info(&interface->dev, "ATUSB ben-wpan device now attached\n");
	return 0;

err_version:
	device_remove_file(&interface->dev, &dev_attr_rf_version_num);
err_part:
	device_remove_file(&interface->dev, &dev_attr_rf_part_num);
err_master:
	spi_master_put(atusb->master);
//	kfree(atusb);
	return retval;
}

static void atusb_disconnect(struct usb_interface *interface)
{
	struct atusb_local *atusb;

	atusb = usb_get_intfdata(interface);

	/*
	 * Remove sys files
	 */
	device_remove_file(&interface->dev, &dev_attr_rf_version_num);
	device_remove_file(&interface->dev, &dev_attr_rf_part_num);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(atusb->udev);

	spi_unregister_master(atusb->master);
	spi_master_put(atusb->master);

//	kfree(atusb);

	dev_info(&interface->dev, "ATUSB ben-wpan device now disconnected\n");
}

static struct usb_driver atusb_driver = {
	.name         = "atusb_ben-wpan",
	.probe        = atusb_probe,
	.disconnect = atusb_disconnect,
	.id_table   = atusb_device_table,
};

static int __init atusb_init(void)
{
	int retval = 0;

	retval = usb_register(&atusb_driver);
	if (retval)
		err("usb_register failed. Error number %d", retval);

	return retval;
}

static void __exit atusb_exit(void)
{
	usb_deregister(&atusb_driver);
}

module_init (atusb_init);
module_exit (atusb_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
