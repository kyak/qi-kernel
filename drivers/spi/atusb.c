/*
 * ATUSB driver 0.1
 *
 * Copyright (c) 2011 Richard Sharpe (realrichardsharpe@gmail.com)
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
	unsigned int some_other_stuff;
	/* The RF part info below */
	uint8_t rf_part_num;
	uint8_t rf_version_num;
	/* The interface to the RF part info, if applicable */
	uint8_t ep0_atusb_major;
	uint8_t ep0_atusb_minor;
	uint8_t atusb_hw_type;
	char atusb_build[ATUSB_BUILD_SIZE + 1];
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

static ssize_t atusb_show_id(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct atusb_local *atusb = usb_get_intfdata(intf);

	return sprintf(buf, "Major: %u, Minor: %u, HW Type: %u\n",
			atusb->ep0_atusb_major,
			atusb->ep0_atusb_minor,
			atusb->atusb_hw_type);
}

static DEVICE_ATTR(atusb_id, S_IRUGO, atusb_show_id, NULL);

static ssize_t atusb_show_build(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct atusb_local *atusb = usb_get_intfdata(intf);

	return sprintf(buf, "%s\n", atusb->atusb_build);
}

static DEVICE_ATTR(atusb_build, S_IRUGO, atusb_show_build, NULL);

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

static int atusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct atusb_local *atusb = NULL;
	int retval = -ENOMEM;

	atusb = kzalloc(sizeof(struct atusb_local), GFP_KERNEL);
	if (!atusb) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error_mem;
	}

	atusb->udev = usb_get_dev(udev);
	usb_set_intfdata(interface, atusb);



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
		goto error;
	}

	dev_info(&udev->dev, "Firmware: %s\n", atusb->atusb_build);

	/*
	 * Create the sysfs files
	 */

	retval = device_create_file(&interface->dev, &dev_attr_atusb_id);
	if (retval)
		goto error;

	retval = device_create_file(&interface->dev, &dev_attr_atusb_build);
	if (retval)
		goto error;

	retval = device_create_file(&interface->dev, &dev_attr_rf_part_num);
	if (retval)
		goto error;

	retval = device_create_file(&interface->dev, &dev_attr_rf_version_num);
	if (retval)
		goto error;

	dev_info(&interface->dev, "ATUSB ben-wpan device now attached\n");
	return 0;

error:
	device_remove_file(&interface->dev, &dev_attr_rf_version_num);
	device_remove_file(&interface->dev, &dev_attr_rf_part_num);
	device_remove_file(&interface->dev, &dev_attr_atusb_build);
	device_remove_file(&interface->dev, &dev_attr_atusb_id);
	kfree(atusb);
error_mem:
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
	device_remove_file(&interface->dev, &dev_attr_atusb_build);
	device_remove_file(&interface->dev, &dev_attr_atusb_id);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(atusb->udev);

	kfree(atusb);

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
