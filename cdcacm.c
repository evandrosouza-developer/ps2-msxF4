/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>

#include "cdcacm.h"
#include "serial.h"
#include "port_def.h"

extern struct ring tx_ring;
extern struct ring rx_ring;
usbd_device *global_usbd_dev;

bool usb_aligned;
char serial_no[LEN_SERIAL_No + 1];
#define USBD_CONTROL_BUFFER_SIZE					64

//Prototypes
//void USB_ISR(void);

//To place strings entirely into Flash/read-only memory, use
//static const * const strings[] = { ... };

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	//.idVendor = 0x0483,
	//.idProduct = 0x5740,
	.idVendor = 0x0ACE,		//ZyXEL Omni FaxModem 56K Plus
	.idProduct = 0x1611,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
} };

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
} };

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
} };

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
} };

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
} };

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

/*static const char * usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	"DEMO",
};*/
static const char *usb_strings[] = {
	"Evandro Souza Technologies",
	"PS/2 keyboard Interface for MSX",
	serial_no,
	"Console Port",
	"UART1 Port",
};




void serialno_read(char *s)
{
	int i, ii;
	uint64_t unique_id, u_id_p1, u_id_p2;

#ifdef STM32F103
	volatile uint32_t *unique_id_p = (volatile uint32_t *)0x1FFFF7E8;
#endif
#ifdef STM32F401
	volatile uint32_t *unique_id_p = (volatile uint32_t *)0x1FFF7A10;
#endif
	u_id_p1 = *(unique_id_p + 1);
	u_id_p2 = *(unique_id_p + 2);
	unique_id = (*unique_id_p)	+ (u_id_p2) +
				((u_id_p1 + u_id_p2) << 32);
	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < LEN_SERIAL_No; i++)
	{
		ii = LEN_SERIAL_No-1-i;
		s[ii] = ((unique_id >> (4*i)) & 0xF) + '0';
		if(s[ii] > '9')
			s[ii] += 'A' - '9' - 1;
	}
	s[LEN_SERIAL_No] = 0;

	return; //s
}


/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[USBD_CONTROL_BUFFER_SIZE];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return USBD_REQ_NOTSUPP;
		}

		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}


static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	/*(void)ep;

	char buf[USBD_CONTROL_BUFFER_SIZE];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, USBD_CONTROL_BUFFER_SIZE);

	if (len) {
		while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
	}*/
	(void)ep;
	uint16_t len, i;
	uint8_t buf[USBD_CONTROL_BUFFER_SIZE];
	len = usbd_ep_read_packet(usbd_dev, 0x01, buf, USBD_CONTROL_BUFFER_SIZE);

	ep &= 0xFF;		//to avoid warnong of unused variable

	if (len)
	{
		for(i = 0; i == len - 1; i++)
		{
			if(ring_put_ch(&rx_ring, buf[i]) == 0)
				//rx_ring is full
				break;
		}
	}
}


static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	uint16_t i, qty_in_buffer, len;
	ep *= 1;

	uint8_t buf[USBD_CONTROL_BUFFER_SIZE];
	len = (ring_avail_get_ch(&tx_ring));
	if (len > USBD_CONTROL_BUFFER_SIZE)
		len = USBD_CONTROL_BUFFER_SIZE;

	if (len) {
		for(i = 0; i <= (len - 1); i++)
		{
				buf[i] = (uint8_t)ring_get_ch(&tx_ring, &qty_in_buffer);
				if(qty_in_buffer == 0)
					//rx_ring is full
					break;
		}
		usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
	}
}


static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE,
			cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE,
			cdcacm_data_tx_cb);
	/*usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE, NULL);*/
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}


void cdcacm_init(void)
{
	serialno_read(serial_no);

	usbd_device *usbd_dev;

	rcc_periph_clock_enable(RCC_OTGFS);

#ifdef STM32F103
	/* Setup GPIOC Pin 12 to pull up the D+ high, so autodect works
	 * with the bootloader.  The circuit is active low. */
	//gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	//gpio_clear(GPIOC, GPIO12);
#endif
#ifdef STM32F401
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
#endif

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3,
			usbd_control_buffer, sizeof(usbd_control_buffer));

	global_usbd_dev	= usbd_dev;

	usb_aligned = true;

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	//while (1) {
	//	usbd_poll(usbd_dev);
	//nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	//nvic_enable_irq(USB_IRQ);
}


//void USB_ISR(void)
//{
//	usbd_poll(usbdev);
//}
