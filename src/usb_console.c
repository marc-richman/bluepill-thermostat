#include "usb_console.h"
#include <string.h>
#include <stdio.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* Shared with main: provide access to system_millis for usb polling if desired */
extern volatile uint32_t system_millis;

static usbd_device *usbdev = NULL;
static bool configured = false;

/* TX queue */
#define USB_TX_BUF_SIZE 512
static uint8_t txbuf[USB_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

static void (*cmd_handler)(const char *cmd) = NULL;
static char rx_cmd_buf[64];
static uint8_t rx_cmd_len = 0;

/* USB descriptors and callbacks (standard CDC ACM) */
static uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

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
}};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header = {
        .bFunctionLength   = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType   = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC            = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength   = sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType   = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities    = 0,
        .bDataInterface    = 1,
    },
    .acm = {
        .bFunctionLength   = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType   = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities    = 0,
    },
    .cdc_union = {
        .bFunctionLength       = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType       = CS_INTERFACE,
        .bDescriptorSubtype    = USB_CDC_TYPE_UNION,
        .bControlInterface     = 0,
        .bSubordinateInterface0= 1,
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
    .extralen = sizeof(cdcacm_functional_descriptors),
}};

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
}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config_descr = {
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

static const char *usb_strings[] = {
    "Bluepill",
    "Heater SHT40 controller",
    "0001",
};

static enum usbd_request_return_codes
cdcacm_control_request(usbd_device *usbd_dev,
                       struct usb_setup_data *req,
                       uint8_t **buf, uint16_t *len,
                       usbd_control_complete_callback *complete)
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
        return USBD_REQ_HANDLED;
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding)) return USBD_REQ_NOTSUPP;
        return USBD_REQ_HANDLED;
    default:
        return USBD_REQ_NOTSUPP;
    }
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;
    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));
    if (len <= 0) return;

    for (int i = 0; i < len; i++) {
        char c = buf[i];
        if (c == '\r' || c == '\n') {
            if (rx_cmd_len > 0) {
                rx_cmd_buf[rx_cmd_len] = '\0';
                /* echo */
                usb_console_write("> ");
                usb_console_write(rx_cmd_buf);
                usb_console_write("\r\n");
                if (cmd_handler) cmd_handler(rx_cmd_buf);
                rx_cmd_len = 0;
            }
        } else {
            if (rx_cmd_len < (sizeof(rx_cmd_buf) - 1)) {
                rx_cmd_buf[rx_cmd_len++] = c;
            }
        }
    }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;
    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        cdcacm_control_request);

    configured = true;
}

void usb_console_init(void)
{
    /* Force re-enumeration by pulling D+ low briefly */
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);
    for (volatile uint32_t i = 0; i < 800000; i++) __asm__("nop");
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11 | GPIO12);

    rcc_periph_clock_enable(RCC_USB);

    usbdev = usbd_init(&st_usbfs_v1_usb_driver,
                       &dev_descr, &config_descr,
                       usb_strings, 3,
                       usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbdev, cdcacm_set_config);
}

int usb_console_write(const char *s)
{
    if (!configured) return 0;
    int enqueued = 0;
    while (*s) {
        uint16_t next = (tx_head + 1) % USB_TX_BUF_SIZE;
        if (next == tx_tail) break; /* full */
        txbuf[tx_head] = (uint8_t)(*s++);
        tx_head = next;
        ++enqueued;
    }
    return enqueued;
}

void usb_console_poll(void)
{
    if (usbdev) usbd_poll(usbdev);

    if (!configured) return;

    while (tx_tail != tx_head) {
        uint16_t tail = tx_tail;
        uint16_t head = tx_head;
        uint16_t len = (head >= tail) ? (head - tail) : (USB_TX_BUF_SIZE - tail);
        if (len == 0) break;
        if (len > 64) len = 64;
        int written = usbd_ep_write_packet(usbdev, 0x82, &txbuf[tail], len);
        if (written <= 0) break;
        tx_tail = (tx_tail + written) % USB_TX_BUF_SIZE;
    }
}

void usb_console_set_cmd_handler(void (*handler)(const char *cmd))
{
    cmd_handler = handler;
}

bool usb_console_configured(void)
{
    return configured;
}