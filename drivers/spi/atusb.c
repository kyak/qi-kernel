/*
 * atusb - SPI host look-alike for ATSUB
 *
 * Copyright (c) 2011 Richard Sharpe <realrichardsharpe@gmail.com>
 * Copyright (c) 2011 Stefan Schmidt <stefan@datenfreihafen.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/at86rf230.h>

#include "../ieee802154/at86rf230.h"	/* dirty */

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
	/* The interface to the RF part info, if applicable */
	uint8_t ep0_atusb_major;
	uint8_t ep0_atusb_minor;
	uint8_t atusb_hw_type;
	unsigned char *atusb_build;
	struct spi_master *master;
	int slave_irq;
	int usb_irq;
	struct at86rf230_platform_data platform_data;
	/* copy platform_data so that we can adapt .reset_data */
	struct spi_device *spi;
	struct urb *ctrl_urb;
	spinlock_t		err_lock;		/* lock for errors */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	bool			processed_urb;		/* indicates we haven't processed the urb */
	struct completion	urb_completion;
	unsigned char *buffer;
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

static void atusb_usb_cb(struct urb *urb)
{
	struct atusb_local *atusb;

	atusb = urb->context;

	spin_lock(&atusb->err_lock);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&atusb->udev->dev, "nonzero write bulk status received: %d",
			    urb->status);

	} else {
		atusb->bulk_in_filled = urb->actual_length;
	}
	spin_unlock(&atusb->err_lock);
	complete(&atusb->urb_completion);
}

static void atusb_read1_cb(struct urb *urb)
{
	struct spi_message *msg;
	msg = urb->context;

	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			printk("Async USB failed with error %i\n", urb->status);

	} else {
		printk("Async USB succeeded with length %i\n", urb->actual_length);
	}
//	printk("RX buffer %i\n", msg->transfers->rx_buf);
	msg->status = 0;
	msg->complete(msg->context);
}
static int atusb_get_static_info(struct atusb_local *atusb)
{
	int retval;
	struct usb_ctrlrequest *req;

	atusb->buffer = kzalloc(3, GFP_KERNEL);
	if (!atusb->buffer) {
		dev_err(&atusb->udev->dev, "out of memory\n");
		retval = -ENOMEM;
	}

	atusb->atusb_build = kzalloc(ATUSB_BUILD_SIZE+1, GFP_KERNEL);
	if (!atusb->buffer) {
		dev_err(&atusb->udev->dev, "out of memory\n");
		retval = -ENOMEM;
	}

	/*
	 * Get a couple of the ATMega Firmware values as well
	 */
	atusb->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!atusb->ctrl_urb) {
		retval = -ENOMEM;
	}
	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
        req->bRequest = ATUSB_ID;
        req->bRequestType = ATUSB_FROM_DEV;
        req->wValue = cpu_to_le16(0x00);
        req->wIndex = cpu_to_le16(0x00);
        req->wLength = cpu_to_le16(3);

	usb_fill_control_urb(atusb->ctrl_urb,
			atusb->udev,
			usb_rcvbulkpipe(atusb->udev, 0),
			(unsigned char *)req,
			atusb->buffer,
			3,
			atusb_usb_cb,
			atusb);

	retval = usb_submit_urb(atusb->ctrl_urb, GFP_KERNEL);
	if (retval < 0) {
		dev_info(&atusb->udev->dev, "failed submitting read urb, error %d",
			retval);
		retval = (retval == -ENOMEM) ? retval : -EIO;
	}
	wait_for_completion_interruptible(&atusb->urb_completion);
	INIT_COMPLETION(atusb->urb_completion);
	usb_free_urb(atusb->ctrl_urb);
	kfree(req);

	atusb->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!atusb->ctrl_urb) {
		retval = -ENOMEM;
	}
	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
        req->bRequest = ATUSB_BUILD;
        req->bRequestType = ATUSB_FROM_DEV;
        req->wValue = cpu_to_le16(0x00);
        req->wIndex = cpu_to_le16(0x00);
        req->wLength = cpu_to_le16(ATUSB_BUILD_SIZE+1); /* Either size length is wrong... */

	usb_fill_control_urb(atusb->ctrl_urb,
			atusb->udev,
			usb_rcvbulkpipe(atusb->udev, 0),
			(unsigned char *)req,
			atusb->atusb_build,
			ATUSB_BUILD_SIZE+1, /* ... or size length is wrong */
			atusb_usb_cb,
			atusb);

	retval = usb_submit_urb(atusb->ctrl_urb, GFP_KERNEL);
	if (retval < 0) {
		dev_info(&atusb->udev->dev, "failed submitting read urb, error %d",
			retval);
		retval = (retval == -ENOMEM) ? retval : -EIO;
	}
	wait_for_completion_interruptible(&atusb->urb_completion);
	usb_free_urb(atusb->ctrl_urb);
	kfree(req);

	return retval;
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
				0,
				1000);
	if (retval < 0) {
		dev_info(&atusb->udev->dev,
			"%s: error doing reset retval = %d\n",
			__func__, retval);
	}
}

static void atusb_read1(struct atusb_local *atusb, const uint8_t *tx, uint8_t *rx, int len, struct spi_message *msg)
{
	int retval;
	struct usb_ctrlrequest *req;

	dev_info(&atusb->udev->dev, "atusb_read1: tx = 0x%x\n", tx[0]);
	atusb->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!atusb->ctrl_urb) {
		retval = -ENOMEM;
	}
	/* ->host	ATUSB_SPI_READ1		byte0		-	#bytes */
	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
        req->bRequest = ATUSB_SPI_READ1;
        req->bRequestType = ATUSB_FROM_DEV;
        req->wValue = cpu_to_le16(tx[0]);
        req->wIndex = cpu_to_le16(0x00);
        req->wLength = cpu_to_le16(0x01);

	usb_fill_control_urb(atusb->ctrl_urb,
			atusb->udev,
			usb_rcvbulkpipe(atusb->udev, 0),
			(unsigned char *)req,
			rx+1,
			0x01,
			atusb_read1_cb,
			msg);

	retval = usb_submit_urb(atusb->ctrl_urb, GFP_KERNEL);
	if (retval < 0) {
		dev_info(&atusb->udev->dev, "failed submitting read urb, error %d",
			retval);
		retval = (retval == -ENOMEM) ? retval : -EIO;
	}
	usb_free_urb(atusb->ctrl_urb);
	kfree(req);
}

static void atusb_read2(struct atusb_local *atusb, uint8_t *buf, int len)
{
	/* ->host	ATUSB_SPI_READ2		byte0		byte1	#bytes */
}

static void atusb_write(struct atusb_local *atusb, const uint8_t *tx, uint8_t *rx, int len, struct spi_message *msg)
{
	int retval;
	struct usb_ctrlrequest *req;

	dev_info(&atusb->udev->dev, "atusb_write: tx[0] = 0x%x\n", tx[0]);
	dev_info(&atusb->udev->dev, "atusb_write: tx[1] = 0x%x\n", tx[1]);
	atusb->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!atusb->ctrl_urb) {
		retval = -ENOMEM;
	}
	/* host->	ATUSB_SPI_WRITE		byte0		byte1	#bytes */
	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
        req->bRequest = ATUSB_SPI_WRITE;
        req->bRequestType = ATUSB_TO_DEV;
        req->wValue = cpu_to_le16(tx[0]);
        req->wIndex = cpu_to_le16(tx[1]);
        req->wLength = cpu_to_le16(0x0);

	usb_fill_control_urb(atusb->ctrl_urb,
			atusb->udev,
			usb_rcvbulkpipe(atusb->udev, 0),
			(unsigned char *)req,
			0,
			0, /* ... or size length is wrong */
			atusb_read1_cb,
			msg);

	retval = usb_submit_urb(atusb->ctrl_urb, GFP_KERNEL);
	if (retval < 0) {
		dev_info(&atusb->udev->dev, "failed submitting read urb, error %d",
			retval);
		retval = (retval == -ENOMEM) ? retval : -EIO;
	}
	usb_free_urb(atusb->ctrl_urb);
	kfree(req);
}

static int atusb_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct atusb_local *atusb = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	struct spi_transfer *x[2];
	int n;
	const uint8_t *tx;
	uint8_t *rx;

	if (unlikely(list_empty(&msg->transfers))) {
		dev_err(&atusb->udev->dev, "transfer is empty\n");
		return -EINVAL;
	}

	/*
	 * Classify the request.
	 */
	n = 0;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (n == ARRAY_SIZE(x)) {
			dev_info(&atusb->udev->dev, "too many transfers\n");
			return -EINVAL;
		}
		x[n] = xfer;
		//dev_info(&atusb->udev->dev, "%s transfer %i\n", __func__, n);
		n++;
	}

	msg->actual_length = 0;

	if (!x[0]->tx_buf || x[0]->len != 2)
		goto bad_req;
	if (n == 1) {
		if (x[0]->rx_buf) {
			dev_info(&atusb->udev->dev, "read 1\n");
			tx = x[0]->tx_buf;
			rx = x[0]->rx_buf;
			msg->actual_length += x[0]->len;
			atusb_read1(atusb, tx, rx, x[0]->len, msg);
		} else {
			dev_info(&atusb->udev->dev, "write 2\n");
			tx = x[0]->tx_buf;
			rx = x[0]->rx_buf;
			msg->actual_length += x[0]->len;
			atusb_write(atusb, tx, rx, x[0]->len, msg);
		}
	} else {
		if (x[0]->rx_buf) {
			if (x[1]->tx_buf || !x[1]->rx_buf)
				goto bad_req;
			dev_info(&atusb->udev->dev, "read 1+\n");
		} else {
			if (!x[1]->tx_buf ||x[1]->rx_buf)
				goto bad_req;
			dev_info(&atusb->udev->dev, "write 2+n\n");
		}
	}
#if 0
	msg->actual_length = 0;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		tx = xfer->tx_buf;
		rx = xfer->rx_buf;
		msg->actual_length += xfer->len;
		atusb_read1(atusb, tx, rx, xfer->len, msg);
	}
#endif
#if 0
	/*
	 * The AT86RF230 driver sometimes requires a transceiver state
	 * transition to be an interrupt barrier. This is the case after
	 * writing FORCE_TX_ON to the TRX_CMD field in the TRX_STATE register.
	 *
	 * Since there is no other means of notification, we just decode the
	 * transfer and do a bit of pattern matching.
	 */
	xfer = list_first_entry(&msg->transfers, struct spi_transfer,
	    transfer_list);
	tx = xfer->tx_buf;
	if (tx && xfer->len == 2 &&
	    tx[0] == (CMD_REG | CMD_WRITE | RG_TRX_STATE) &&
	    (tx[1] & 0x1f) == STATE_FORCE_TX_ON)
		synchronize_irq(atusb->gpio_irq);
#endif
//	dev_info(&atusb->udev->dev, "atusb_transfer: tx = %i, rx = %i\n", *tx, *rx);

	return 0;

bad_req:
	dev_info(&atusb->udev->dev, "unrecognized request:\n");
	list_for_each_entry(xfer, &msg->transfers, transfer_list)
		dev_info(&atusb->udev->dev, "%stx %srx len %u\n",
		    xfer->tx_buf ? "" : "!", xfer->rx_buf ? " " : "!",
		    xfer->len);
	return -EINVAL;
}

static int atusb_setup(struct spi_device *spi)
{
	return 0;
}
#if 0
static irqreturn_t atusb_irq(int irq, void *data)
{
	struct atusb_local *atusb = data;

	generic_handle_irq(atusb->slave_irq);
	return IRQ_HANDLED;
}
#endif
static void atusb_irq_mask(struct irq_data *data)
{
//	struct atben_local *atusb = irq_data_get_irq_chip_data(data);

//	disable_irq_nosync(atusb->usb_irq);
}

static void atusb_irq_unmask(struct irq_data *data)
{
//	struct atben_local *atusb = irq_data_get_irq_chip_data(data);

//	enable_irq(atusb->usb_irq);
}

static struct irq_chip atusb_irq_chip = {
	.name		= "atusb-slave",
	.irq_mask	= atusb_irq_mask,
	.irq_unmask	= atusb_irq_unmask,
};

struct at86rf230_platform_data at86rf230_platform_data = {
	.rstn	= -1,
	.slp_tr	= -1,
	.dig2	= -1,
	.reset	= atben_reset,
	/* set .reset_data later */
};

static int atusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct spi_board_info board_info = {
		.modalias = "at86rf230",
		/* set .irq later */
		.chip_select = 0,
		.bus_num = -1,
		.max_speed_hz = 8 * 1000 * 1000,
	};

	struct usb_device *udev = interface_to_usbdev(interface);
	struct atusb_local *atusb = NULL;
	struct spi_master *master;
	int retval;

	/*
	 * Interface 1 is used for DFU. Ignore it in this driver to avoid
	 * attaching to both interfaces
	 */
        if (interface == udev->actconfig->interface[1]) {
                dev_info(&udev->dev,
                         "Ignoring interface 1 reserved for DFU\n");
                return -ENODEV;
        }

	master = spi_alloc_master(&udev->dev, sizeof(*atusb));
	if (!master)
		return -ENOMEM;

	atusb = spi_master_get_devdata(master);

	spin_lock_init(&atusb->err_lock);
	init_completion(&atusb->urb_completion);

	atusb->udev = usb_get_dev(udev);
	usb_set_intfdata(interface, atusb);

	atusb->master = spi_master_get(master);

	master->mode_bits	= SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bus_num		= -1;
	master->num_chipselect	= 1;
	master->setup		= atusb_setup;
	master->transfer	= atusb_transfer;

	atusb->slave_irq = irq_alloc_desc(numa_node_id());
	if (atusb->slave_irq < 0) {
		dev_info(&udev->dev, "can't allocate slave irq\n");
		retval = -ENXIO;
		goto err_free;
	}

	set_irq_chip_data(atusb->slave_irq, atusb);
	set_irq_chip_and_handler(atusb->slave_irq, &atusb_irq_chip,
	    handle_level_irq);

	/* FIXME prepare USB IRQ */

	retval = spi_register_master(master);
	if (retval < 0) {
		dev_info(&udev->dev, "can't register spi master\n");
		goto err_slave_irq;
	}

	atusb->platform_data = at86rf230_platform_data;
	atusb->platform_data.reset_data = atusb;
	board_info.platform_data = &atusb->platform_data;
	board_info.irq = atusb->slave_irq;

	atusb->spi = spi_new_device(master, &board_info);
	if (!atusb->spi) {
		dev_info(&udev->dev, "can't create new device for %s\n",
		    board_info.modalias);
		goto err_master;
	}

	dev_info(&atusb->spi->dev, "ATUSB ready for mischief (IRQ %d)\n", board_info.irq);
#if 1
	/*
	 * Get the static info from the device and save it ...
	 */
	retval = atusb_get_static_info(atusb);
	if (retval) {
		dev_info(&interface->dev, "%s: Failed to get static info: %d\n",
			__func__,
			retval);
		goto err_master;
	}

	dev_info(&udev->dev, "Firmware: build %s\n", atusb->atusb_build);
	atusb->ep0_atusb_major = atusb->buffer[0];
	atusb->ep0_atusb_minor = atusb->buffer[1];
	atusb->atusb_hw_type   = atusb->buffer[2];
	dev_info(&udev->dev, "Firmware: major: %u, minor: %u, hardware type: %u\n",
		atusb->ep0_atusb_major, atusb->ep0_atusb_minor, atusb->atusb_hw_type);

	kfree(atusb->buffer);
	kfree(atusb->atusb_build);
#endif
	return 0;

err_master:
	spi_master_put(atusb->master);
err_slave_irq:
	set_irq_chained_handler(atusb->slave_irq, NULL);
	set_irq_chip_data(atusb->slave_irq, NULL);
	irq_free_desc(atusb->slave_irq);
err_free:
	return retval;
}

static void atusb_disconnect(struct usb_interface *interface)
{
	/* atben should come out of master devdata */
	struct atusb_local *atusb = usb_get_intfdata(interface);
	struct spi_master *master = atusb->master;

	usb_set_intfdata(interface, NULL);
	usb_put_dev(atusb->udev);

	spi_dev_put(atusb->spi);

	spi_unregister_master(master);

	set_irq_chained_handler(atusb->slave_irq, NULL);
	set_irq_chip_data(atusb->slave_irq, NULL);
	irq_free_desc(atusb->slave_irq);

	spi_master_put(master);
}

void atusb_release(struct device *dev)
{
	return;
}

static struct usb_driver atusb_driver = {
	.name         = "atusb_ben-wpan",
	.probe        = atusb_probe,
	.disconnect = atusb_disconnect,
	.id_table   = atusb_device_table,
};
#if 0
static struct resource atusb_resources[] = {
	{
		.start  = JZ4740_GPIO_BASE_ADDR+0x300,
		.end    = JZ4740_GPIO_BASE_ADDR+0x3ff,
		.flags  = IORESOURCE_MEM,
	},
	{
		/* set start and end later */
		.flags  = IORESOURCE_IRQ,
	},
};
#endif
static struct platform_device atusb_device = {
	.name = "spi_atusb",
	.id = -1,
//	.num_resources = ARRAY_SIZE(atben_resources),
//	.resource = atben_resources,
	.dev.release = atusb_release,
};

static int __init atusb_init(void)
{
	int retval;

	retval = platform_device_register(&atusb_device);
	if (retval)
		return retval;

//	atusb_resources[1].start = atusb_resources[1].end =
//	    gpio_to_irq(JZ_GPIO_PORTD(12));

	return usb_register(&atusb_driver);
}

static void __exit atusb_exit(void)
{
	usb_deregister(&atusb_driver);
	platform_device_unregister(&atusb_device);
}

module_init (atusb_init);
module_exit (atusb_exit);

MODULE_AUTHOR("Richard Sharpe <realrichardsharpe@gmail.com>, Stefan Schmidt \
		<stefan@datenfreihafen.org>");
MODULE_DESCRIPTION( "ATUSB ben-wpan Driver");
MODULE_LICENSE("GPL");
