/*
 * bluepill-thermostat.c
 *
 * Bluepill heater controller with SHT40 sensor and USB CDC console.
 *
 * Rewritten to use libopencm3 low-level I2C helpers (for STM32F103):
 *  - i2c_send_start
 *  - i2c_send_7bit_address
 *  - i2c_send_stop
 *
 * Uses register-level checks (I2C_SR1/I2C_SR2/I2C_DR) for robust error reporting
 * and replaces prior use of i2c_transfer7. Intended for libopencm3 + STM32F1.
 *
 * Retains improvements:
 *  - Command prefix ordering fixes
 *  - USB non-blocking outbound queue
 *  - Parsing bounds for seconds/temperature
 *  - Single GPIOB RCC enable
 *  - I2C bus re-init limiting and improved retry logging
 *
 * Note: STM32F1 I2C read semantics for the last 1-2 bytes are delicate. This
 * implementation takes a pragmatic approach: it issues STOP before the final
 * RXNE read to force the peripheral to NACK further bytes. For absolute
 * corner-case correctness on all toolchain versions, further device-specific
 * handling of ACK/POS may be required.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

/* ------------ Application configuration ------------ */

/* Heater control pin: PB0, push-pull output, HIGH = ON */
#define HEATER_PORT                     GPIOB
#define HEATER_PIN                      GPIO0

/* I2C1 for SHT40: PB6 = SCL, PB7 = SDA */
#define SHT40_I2C                       I2C1
#define SHT40_ADDR                      0x44
#define SHT40_CMD_MEAS_HIGH_PREC        0xFD

/* I2C timing constants (STM32F103: HSE 8MHz -> SYSCLK 72MHz, PCLK1 = 36MHz) */
#define I2C_PCLK1_FREQ_MHZ              36U
#define I2C_CCR_SM_100KHZ_36MHZ         180U
#define I2C_TRISE_SM_36MHZ              (I2C_PCLK1_FREQ_MHZ + 1U)

/* SHT40 measurement timing & retries */
#define SHT40_MEAS_MAX_TIME_MS          9U
#define SHT40_MEAS_DELAY_MARGIN_MS      11U
#define SHT40_MEAS_DELAY_MS             (SHT40_MEAS_MAX_TIME_MS + SHT40_MEAS_DELAY_MARGIN_MS) /* 20ms */
#define SHT40_I2C_MAX_RETRIES           3U
#define SHT40_I2C_RETRY_DELAY_MS        2U
/* cap of how many times we will fully re-initialize I2C across the entire run of program */
#define SHT40_I2C_MAX_BUS_REINITS       4U

/* Defaults stored internally as centi-°C and milliseconds */
#define DEFAULT_TEMP_THRESHOLD_CENTI    1670    /* 16.70 °C */
#define HYSTERESIS_CENTI                50      /* 0.50 °C hysteresis band */

#define DEFAULT_HEAT_MAX_TIME_MS        (300UL * 1000UL)
#define DEFAULT_COOLDOWN_SLEEP_MS       (300UL * 1000UL)

/* Console/reporting cadence (also used as "verbose print" cadence) */
#define CHECK_INTERVAL_MS               (30UL * 1000UL)

/* Watchdog feeding cadence: must be < watchdog timeout (20s). */
#define WDT_FEED_STEP_MS                (10UL * 1000UL)

/* Flash config (last 1K page of 64K flash): 0x0800FC00 (STM32F103C8 64K/1K pages) */
#define CONFIG_FLASH_ADDR               0x0800FC00
#define CONFIG_MAGIC                    0xDEADBEEF

/* Parsing bounds */
#define PARSE_SECS_MAX                  604800UL   /* 7 days cap for safety */
#define PARSE_CENTI_MAX_ABS             100000     /* 1000.00 °C absolute cap */

/* USB outbound queue size */
#define USB_TX_BUF_SIZE                 512
#define USB_EP_BULK_MAXPACKET           64

/* I2C helper timeouts */
#define I2C_FLAG_TIMEOUT_MS             5U   /* per-flag small timeout for responsiveness */
#define I2C_BYTE_TIMEOUT_MS             5U

/* Local SR1 bit masks (explicit values for STM32F1) */
#define I2C_SR1_SB_MASK      (1U << 0)
#define I2C_SR1_ADDR_MASK    (1U << 1)
#define I2C_SR1_BTF_MASK     (1U << 2)
#define I2C_SR1_ADD10_MASK   (1U << 3)
#define I2C_SR1_STOPF_MASK   (1U << 4)
#define I2C_SR1_RXNE_MASK    (1U << 6)
#define I2C_SR1_TXE_MASK     (1U << 7)
#define I2C_SR1_BERR_MASK    (1U << 8)
#define I2C_SR1_ARLO_MASK    (1U << 9)
#define I2C_SR1_AF_MASK      (1U << 10)
#define I2C_SR1_OVR_MASK     (1U << 11)
#define I2C_SR1_PECERR_MASK  (1U << 12)
#define I2C_SR1_TIMEOUT_MASK (1U << 14)
#define I2C_SR1_SMBALERT_MASK (1U << 15)

/* ------------ Globals ------------ */

static volatile uint32_t system_millis = 0;

/* Configurable at run-time via USB CDC console */
static int32_t  temp_threshold_centi = DEFAULT_TEMP_THRESHOLD_CENTI; /* centi-°C */
static uint32_t heat_max_time_ms     = DEFAULT_HEAT_MAX_TIME_MS;
static uint32_t cooldown_sleep_ms    = DEFAULT_COOLDOWN_SLEEP_MS;

/* USB CDC */
static usbd_device *usbdev;
static bool usb_configured = false;

/* Small outbound queue (non-blocking): producers enqueue, consumer flushes in usb_poll path */
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];
static volatile uint16_t usb_tx_head = 0; /* next write index */
static volatile uint16_t usb_tx_tail = 0; /* next read index */

/* Command buffer for incoming CDC */
static char cmd_buf[64];
static uint8_t cmd_len = 0;

static char boot_cause_msg[128] = "Boot cause: unknown";

/* ------------ Flash config struct ------------ */
typedef struct {
    uint32_t magic;
    int32_t  threshold_centi;     /* centi-°C */
    uint32_t heat_max_time_ms;
    uint32_t cooldown_sleep_ms;
} config_data_t;

/* ------------ Prototypes ------------ */
void sys_tick_handler(void);

static uint32_t millis(void);
static void delay_ms(uint32_t ms);

static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);
static void i2c_setup(void);
static void watchdog_setup(void);
static void usb_setup(void);
static void usb_poll(void);
static void boot_cause_init(void);

static void config_load(void);
static void config_save(void);
static void print_config(void);

static void hard_reset_via_watchdog(const char *reason);

static void handle_command(const char *cmd);

/* USB TX queue helpers */
static size_t usb_tx_enqueue(const char *s);
static void usb_tx_service(void);

/* Reworked usb_write_str to use enqueue */
static void usb_write_str(const char *s);

static bool parse_centi(const char *s, int32_t *out);
static bool parse_uint_seconds(const char *s, uint32_t *out);

static void centi_to_str(int32_t centi, char *buf, size_t len);
static int32_t centiC_to_centiF(int32_t cC);
static int32_t centiF_to_centiC(int32_t cF);

static void i2c1_soft_reset(uint32_t i2c);
static int i2c_write_bytes(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t len,
                           uint32_t timeout_ms, char *err, size_t errlen);
static int i2c_read_bytes(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t len,
                          uint32_t timeout_ms, char *err, size_t errlen);
static bool sht40_read_temperature_centi(int32_t *temp_centi);

typedef enum {
    THERM_IDLE = 0,
    THERM_HEATING,
    THERM_COOLDOWN
} therm_state_t;

static bool temp_sample_compare_and_feed(bool verbose,
                                         bool *want_heat,   /* temp <= setpoint-hyst */
                                         bool *stop_heat);  /* temp >= setpoint */

/* ------------ Timebase ------------ */
static uint32_t millis(void)
{
    return system_millis;
}

void sys_tick_handler(void)
{
    system_millis++;
}

/* delay_ms polls USB but DOES NOT feed watchdog */
static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        usb_poll();
        /* DO NOT iwdg_reset() here */
    }
}

/* ------------ Clock/SysTick ------------ */
static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1);
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();
}

/* ------------ Boot cause ------------ */
static void boot_cause_init(void)
{
    uint32_t csr = RCC_CSR;

    strcpy(boot_cause_msg, "Boot cause:");
    bool any = false;

    if (csr & RCC_CSR_PORRSTF) { strcat(boot_cause_msg, " POR"); any = true; }
    if (csr & RCC_CSR_PINRSTF) { strcat(boot_cause_msg, " PIN"); any = true; }
    if (csr & RCC_CSR_SFTRSTF) { strcat(boot_cause_msg, " SW");  any = true; }
    if (csr & RCC_CSR_IWDGRSTF){ strcat(boot_cause_msg, " IWDG");any = true; }
    if (csr & RCC_CSR_WWDGRSTF){ strcat(boot_cause_msg, " WWDG");any = true; }
    if (csr & RCC_CSR_LPWRRSTF){ strcat(boot_cause_msg, " LPWR");any = true; }

    if (!any) strcat(boot_cause_msg, " unknown");

    RCC_CSR |= RCC_CSR_RMVF;
}

/* ------------ GPIO/heater ------------ */
static void gpio_setup(void)
{
    /* Keep the single enable for GPIOB here (removed duplicate in i2c_setup). */
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_mode(HEATER_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, HEATER_PIN);
    gpio_clear(HEATER_PORT, HEATER_PIN);
}

static inline void heater_on(void)  { gpio_set(HEATER_PORT, HEATER_PIN); }
static inline void heater_off(void) { gpio_clear(HEATER_PORT, HEATER_PIN); }

/* ------------ Intentional hard reset via watchdog ------------ */
static void hard_reset_via_watchdog(const char *reason)
{
    heater_off();

    if (usb_configured && reason) {
        usb_write_str("FATAL: ");
        usb_write_str(reason);
        usb_write_str(" -> watchdog reset\r\n");
    }

    /* Stop feeding watchdog permanently; it will reset us. */
    while (1) {
        usb_poll();
        for (volatile uint32_t i = 0; i < 20000; i++) __asm__("nop");
    }
}

/* ------------ I2C low-level helpers (register-based) ------------ */

/* wait for a SR1 bit mask to be set, returns true if set before timeout_ms, false otherwise */
static bool i2c_wait_flag(uint32_t i2c, uint32_t sr1_mask, uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (!(I2C_SR1(i2c) & sr1_mask)) {
        uint32_t sr1 = I2C_SR1(i2c);
        /* check for fatal errors: bus error, arbitration lost, ack failure */
        if (sr1 & (I2C_SR1_BERR_MASK | I2C_SR1_ARLO_MASK | I2C_SR1_AF_MASK)) {
            return false;
        }
        if ((millis() - start) >= timeout_ms) return false;
        usb_poll(); /* keep USB alive while waiting */
    }
    return true;
}

/* Write len bytes to 'addr' (7-bit) on bus 'i2c'.
 * Returns 0 on success, negative on error.
 * err (optional) will be populated with an ASCII brief message.
 */
static int i2c_write_bytes(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t len,
                           uint32_t timeout_ms, char *err, size_t errlen)
{
    /* Send START */
    i2c_send_start(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB_MASK, timeout_ms)) {
        if (err) snprintf(err, errlen, "START_TO");
        goto err_no_stop;
    }

    /* Send address (write) */
    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR_MASK, timeout_ms)) {
        /* NACK or timeout */
        if (I2C_SR1(i2c) & I2C_SR1_AF_MASK) {
            if (err) snprintf(err, errlen, "ADDR_NACK");
            i2c_send_stop(i2c);
            return -2;
        }
        if (err) snprintf(err, errlen, "ADDR_TO");
        i2c_send_stop(i2c);
        return -1;
    }
    /* Clear ADDR by reading SR1 then SR2 */
    (void)I2C_SR1(i2c);
    (void)I2C_SR2(i2c);

    /* Send bytes */
    for (size_t i = 0; i < len; i++) {
        I2C_DR(i2c) = data[i];
        /* Wait for TXE (data register empty) */
        if (!i2c_wait_flag(i2c, I2C_SR1_TXE_MASK, timeout_ms)) {
            if (I2C_SR1(i2c) & I2C_SR1_AF_MASK) {
                if (err) snprintf(err, errlen, "DATA_NACK");
                i2c_send_stop(i2c);
                return -2;
            }
            if (err) snprintf(err, errlen, "DATA_TO");
            i2c_send_stop(i2c);
            return -1;
        }
    }

    /* Wait for BTF if available (transfer finished) */
    i2c_wait_flag(i2c, I2C_SR1_BTF_MASK, timeout_ms);

    i2c_send_stop(i2c);
    return 0;

err_no_stop:
    /* best-effort stop */
    i2c_send_stop(i2c);
    return -1;
}

/* Read len bytes from 'addr' (7-bit) on bus 'i2c'.
 * Returns 0 on success, negative on error.
 * err (optional) will be populated with an ASCII brief message.
 *
 * Note: For STM32F1, proper handling of 1/2 byte reads can require special ACK/POS handling.
 * This implementation issues STOP before reading the final byte to force the peripheral to send NACK.
 */
static int i2c_read_bytes(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t len,
                          uint32_t timeout_ms, char *err, size_t errlen)
{
    if (len == 0) return 0;

    /* Ensure ACK is enabled initially */
    I2C_CR1(i2c) |= I2C_CR1_ACK;

    i2c_send_start(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB_MASK, timeout_ms)) {
        if (err) snprintf(err, errlen, "START_TO");
        goto rd_err_no_stop;
    }

    i2c_send_7bit_address(i2c, addr, I2C_READ);
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR_MASK, timeout_ms)) {
        if (I2C_SR1(i2c) & I2C_SR1_AF_MASK) {
            if (err) snprintf(err, errlen, "ADDR_NACK");
            i2c_send_stop(i2c);
            return -2;
        }
        if (err) snprintf(err, errlen, "ADDR_TO");
        i2c_send_stop(i2c);
        return -1;
    }

    /* Clear ADDR by reading SR1 then SR2 */
    (void)I2C_SR1(i2c);
    (void)I2C_SR2(i2c);

    for (size_t i = 0; i < len; i++) {
        /* For the last byte, prepare to NACK and STOP */
        if (i == (len - 1)) {
            /* Disable ACK so the controller NACKs the final byte */
            I2C_CR1(i2c) &= ~I2C_CR1_ACK;
            /* Send STOP early to ensure the peripheral releases the bus after last byte */
            i2c_send_stop(i2c);
        }

        /* Wait for RXNE */
        if (!i2c_wait_flag(i2c, I2C_SR1_RXNE_MASK, timeout_ms)) {
            uint32_t sr1 = I2C_SR1(i2c);
            if (sr1 & (I2C_SR1_BERR_MASK | I2C_SR1_ARLO_MASK)) {
                if (err) snprintf(err, errlen, "BUS_ERR");
                return -3;
            }
            if (err) snprintf(err, errlen, "RXNE_TO");
            return -1;
        }
        /* Read data register */
        buf[i] = (uint8_t)I2C_DR(i2c);
    }

    /* Re-enable ACK for future ops */
    I2C_CR1(i2c) |= I2C_CR1_ACK;

    return 0;

rd_err_no_stop:
    i2c_send_stop(i2c);
    return -1;
}

static void i2c1_soft_reset(uint32_t i2c)
{
    i2c_peripheral_disable(i2c);

    I2C_CR1(i2c) |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 100; i++) __asm__("nop");
    I2C_CR1(i2c) &= ~I2C_CR1_SWRST;

    /* leave disabled for clean re-init */
}

static void i2c_setup(void)
{
    /* GPIOB clock already enabled in gpio_setup() - avoid duplicate enable. */
    rcc_periph_clock_enable(RCC_I2C1);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);

    i2c_peripheral_disable(SHT40_I2C);
    i2c1_soft_reset(SHT40_I2C);

    i2c_set_clock_frequency(SHT40_I2C, I2C_PCLK1_FREQ_MHZ);
    i2c_set_standard_mode(SHT40_I2C);
    i2c_set_ccr(SHT40_I2C, I2C_CCR_SM_100KHZ_36MHZ);
    i2c_set_trise(SHT40_I2C, I2C_TRISE_SM_36MHZ);

    /* Enable ACK by default */
    I2C_CR1(SHT40_I2C) |= I2C_CR1_ACK;

    i2c_peripheral_enable(SHT40_I2C);
}

/* ------------ Watchdog ------------ */
static void watchdog_setup(void)
{
    iwdg_set_period_ms(20000);
    iwdg_start();
}

/* ------------ USB CDC ACM ------------ */

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
                       usbd_control_complete_callback *complete);

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);

static void usb_force_reenumeration(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);

    for (volatile uint32_t i = 0; i < 800000; i++) __asm__("nop");

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO11 | GPIO12);
}

static void usb_setup(void)
{
    usb_force_reenumeration();
    rcc_periph_clock_enable(RCC_USB);

    usbdev = usbd_init(&st_usbfs_v1_usb_driver,
                       &dev_descr, &config_descr,
                       usb_strings, 3,
                       usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbdev, cdcacm_set_config);
}

static void usb_tx_service(void)
{
    if (!usb_configured || !usbdev) return;

    /* Flush the queue in EP-sized chunks */
    while (usb_tx_tail != usb_tx_head) {
        uint16_t tail = usb_tx_tail;
        uint16_t head = usb_tx_head;
        uint16_t len;

        if (head > tail) len = head - tail;
        else len = USB_TX_BUF_SIZE - tail;

        if (len == 0) break;

        if (len > USB_EP_BULK_MAXPACKET) len = USB_EP_BULK_MAXPACKET;

        int written = usbd_ep_write_packet(usbdev, 0x82, &usb_tx_buf[tail], len);
        if (written <= 0) {
            /* Can't write now, back off and retry later */
            break;
        }

        usb_tx_tail = (uint16_t)((usb_tx_tail + written) % USB_TX_BUF_SIZE);
    }
}

/* Enqueue as many bytes of s as will fit; returns number enqueued */
static size_t usb_tx_enqueue(const char *s)
{
    size_t enq = 0;
    while (*s) {
        uint16_t next_head = (uint16_t)((usb_tx_head + 1) % USB_TX_BUF_SIZE);
        if (next_head == usb_tx_tail) {
            /* buffer full */
            break;
        }
        usb_tx_buf[usb_tx_head] = (uint8_t)*s++;
        usb_tx_head = next_head;
        enq++;
    }
    return enq;
}

static void usb_write_str(const char *s)
{
    if (!usb_configured) return;

    /* Non-blocking enqueue; if queue is full, we drop the remainder */
    usb_tx_enqueue(s);
}

/* Ensure usb_poll calls usbd_poll and flushes outbound queue */
static void usb_poll(void)
{
    if (usbdev) usbd_poll(usbdev);
    usb_tx_service();
}

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
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';

                usb_write_str("> ");
                usb_write_str(cmd_buf);
                usb_write_str("\r\n");

                handle_command(cmd_buf);
                cmd_len = 0;
            }
        } else {
            if (cmd_len < (sizeof(cmd_buf) - 1)) {
                cmd_buf[cmd_len++] = c;
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

    usb_configured = true;

    /* Welcome messages use enqueue (non-blocking) */
    usb_write_str("\r\nBluepill heater controller with SHT40\r\n");
    usb_write_str(boot_cause_msg);
    usb_write_str("\r\nCommands:\r\n");
    usb_write_str("  t=<degF> / threshold=<degF> (default)\r\n");
    usb_write_str("  tF=<degF> / thresholdF=<degF>\r\n");
    usb_write_str("  tC=<degC> / thresholdC=<degC>\r\n");
    usb_write_str("  run=<seconds> or r=<seconds>\r\n");
    usb_write_str("  sleep=<seconds> or s=<seconds>\r\n");
    usb_write_str("  status\r\n\r\n");
    print_config();
}

/* ------------ Formatting & parsing ------------ */

static void centi_to_str(int32_t centi, char *buf, size_t len)
{
    bool neg = false;
    /* guard INT32_MIN by using int64_t */
    int64_t val = centi;
    if (val < 0) { neg = true; val = -val; }
    int32_t ip = (int32_t)(val / 100);
    int32_t fp = (int32_t)(val % 100);

    snprintf(buf, len, "%s%ld.%02ld", neg ? "-" : "", (long)ip, (long)fp);
}

static int32_t centiC_to_centiF(int32_t cC)
{
    int64_t n = (int64_t)cC * 9;
    n = (n >= 0) ? (n + 2) / 5 : (n - 2) / 5;
    return (int32_t)n + 3200;
}

static int32_t centiF_to_centiC(int32_t cF)
{
    int64_t n = (int64_t)(cF - 3200) * 5;
    n = (n >= 0) ? (n + 4) / 9 : (n - 4) / 9;
    return (int32_t)n;
}

static bool parse_centi(const char *s, int32_t *out)
{
    bool neg = false;
    int32_t ip = 0, fp = 0;
    int fp_digits = 0;

    while (*s == ' ' || *s == '\t') s++;

    if (*s == '+' || *s == '-') {
        if (*s == '-') neg = true;
        s++;
    }

    if (*s < '0' || *s > '9') return false;

    while (*s >= '0' && *s <= '9') {
        /* avoid overflow: cap integer part to PARSE_CENTI_MAX_ABS/100 */
        if (ip > (PARSE_CENTI_MAX_ABS / 100)) return false;
        ip = ip * 10 + (*s - '0');
        s++;
    }

    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9' && fp_digits < 2) {
            fp = fp * 10 + (*s - '0');
            fp_digits++;
            s++;
        }
        while (*s >= '0' && *s <= '9') s++;
    }

    if (fp_digits == 1) fp *= 10;
    if (fp_digits == 0) fp = 0;

    int32_t val = ip * 100 + fp;
    if (val > (int32_t)PARSE_CENTI_MAX_ABS) return false;
    if (neg) val = -val;

    *out = val;
    return true;
}

static bool parse_uint_seconds(const char *s, uint32_t *out)
{
    uint32_t val = 0;
    while (*s == ' ' || *s == '\t') s++;
    if (*s < '0' || *s > '9') return false;

    while (*s >= '0' && *s <= '9') {
        uint32_t digit = (uint32_t)(*s - '0');
        /* check overflow and cap */
        if (val > (PARSE_SECS_MAX / 10)) return false;
        val = val * 10 + digit;
        if (val > PARSE_SECS_MAX) return false;
        s++;
    }

    *out = val;
    return true;
}

/* ------------ Config load/save ------------ */

static void config_load(void)
{
    const config_data_t *cfg = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (cfg->magic == CONFIG_MAGIC) {
        temp_threshold_centi = cfg->threshold_centi;
        heat_max_time_ms     = cfg->heat_max_time_ms;
        cooldown_sleep_ms    = cfg->cooldown_sleep_ms;
    }
}

static void config_save(void)
{
    config_data_t cfg = {
        .magic             = CONFIG_MAGIC,
        .threshold_centi   = temp_threshold_centi,
        .heat_max_time_ms  = heat_max_time_ms,
        .cooldown_sleep_ms = cooldown_sleep_ms,
    };

    const config_data_t *old = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (old->magic == CONFIG_MAGIC &&
        old->threshold_centi == cfg.threshold_centi &&
        old->heat_max_time_ms == cfg.heat_max_time_ms &&
        old->cooldown_sleep_ms == cfg.cooldown_sleep_ms) {
        usb_write_str("Config unchanged; not writing flash.\r\n");
        return;
    }

    flash_unlock();
    flash_erase_page(CONFIG_FLASH_ADDR);

    const uint32_t *p = (const uint32_t *)&cfg;
    unsigned words = sizeof(cfg) / 4;

    for (unsigned i = 0; i < words; i++) {
        flash_program_word(CONFIG_FLASH_ADDR + (i * 4U), p[i]);
        /* DO NOT feed watchdog here (strict rule) */
    }

    flash_lock();
    usb_write_str("Config saved to flash.\r\n");
}

static void print_config(void)
{
    char buf[32];

    usb_write_str("\r\nCurrent config:\r\n");

    int32_t thr_f = centiC_to_centiF(temp_threshold_centi);
    centi_to_str(thr_f, buf, sizeof(buf));
    usb_write_str("  threshold = ");
    usb_write_str(buf);
    usb_write_str(" F\r\n");

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(heat_max_time_ms / 1000UL));
    usb_write_str("  run       = ");
    usb_write_str(buf);
    usb_write_str(" s\r\n");

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(cooldown_sleep_ms / 1000UL));
    usb_write_str("  sleep     = ");
    usb_write_str(buf);
    usb_write_str(" s\r\n\r\n");
}

/* ------------ Command handling ------------ */

static void handle_command(const char *cmd)
{
    if (!cmd || cmd[0] == '\0') return;

    /* Dual-unit threshold input:
     * default: t= / threshold= => Fahrenheit
     * explicit F: tF= / thresholdF=
     * explicit C: tC= / thresholdC=
     *
     * IMPORTANT: check specific variants before generic ones.
     */
    if (strncmp(cmd, "thresholdC=", 11) == 0 ||
        strncmp(cmd, "thresholdF=", 11) == 0 ||
        strncmp(cmd, "threshold=", 10) == 0 ||
        strncmp(cmd, "tC=", 3) == 0 ||
        strncmp(cmd, "tF=", 3) == 0 ||
        strncmp(cmd, "t=", 2) == 0) {

        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;

        int32_t in_centi;
        if (!valstr || !parse_centi(valstr, &in_centi)) {
            usb_write_str("ERR: bad threshold value\r\n");
            return;
        }

        bool is_celsius = false;
        if (strncmp(cmd, "tC=", 3) == 0 || strncmp(cmd, "thresholdC=", 11) == 0) {
            is_celsius = true;
        }
        /* default (unqualified) and F-suffixed inputs are treated as Fahrenheit */

        int32_t new_thr_c = is_celsius ? in_centi : centiF_to_centiC(in_centi);

        if (new_thr_c != temp_threshold_centi) {
            temp_threshold_centi = new_thr_c;
            usb_write_str("OK: threshold updated\r\n");
            config_save();
        } else {
            usb_write_str("No change: threshold unchanged\r\n");
        }
        print_config();
        return;
    }

    if (strncmp(cmd, "run=", 4) == 0 || strncmp(cmd, "r=", 2) == 0) {
        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;

        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0 && secs <= PARSE_SECS_MAX) {
            uint32_t new_ms = secs * 1000UL;
            if (new_ms != heat_max_time_ms) {
                heat_max_time_ms = new_ms;
                usb_write_str("OK: run time updated\r\n");
                config_save();
            } else {
                usb_write_str("No change: run time unchanged\r\n");
            }
            print_config();
        } else {
            usb_write_str("ERR: bad run value\r\n");
        }
        return;
    }

    if (strncmp(cmd, "sleep=", 6) == 0 || strncmp(cmd, "s=", 2) == 0) {
        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;

        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0 && secs <= PARSE_SECS_MAX) {
            uint32_t new_ms = secs * 1000UL;
            if (new_ms != cooldown_sleep_ms) {
                cooldown_sleep_ms = new_ms;
                usb_write_str("OK: sleep time updated\r\n");
                config_save();
            } else {
                usb_write_str("No change: sleep time unchanged\r\n");
            }
            print_config();
        } else {
            usb_write_str("ERR: bad sleep value\r\n");
        }
        return;
    }

    if (strcmp(cmd, "status") == 0 || strcmp(cmd, "config") == 0) {
        print_config();
        return;
    }

    usb_write_str("Unknown command. Supported:\r\n");
    usb_write_str("  t=<degF> / threshold=<degF> (default)\r\n");
    usb_write_str("  tF=<degF> / thresholdF=<degF>\r\n");
    usb_write_str("  tC=<degC> / thresholdC=<degC>\r\n");
    usb_write_str("  run=<seconds> or r=<seconds>\r\n");
    usb_write_str("  sleep=<seconds> or s=<seconds>\r\n");
    usb_write_str("  status\r\n\r\n");
}

/* ------------ SHT40 temperature read (with retries + CRC) ------------ */

static uint8_t sht40_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static bool sht40_read_temperature_centi(int32_t *temp_centi)
{
    uint8_t cmd;
    uint8_t buf[6];

    unsigned int bus_reinit_count = 0;

    for (uint8_t attempt = 0; attempt < SHT40_I2C_MAX_RETRIES; attempt++) {
        if (bus_reinit_count > SHT40_I2C_MAX_BUS_REINITS) {
            /* too many bus reinitializations: give up early */
            break;
        }

        cmd = SHT40_CMD_MEAS_HIGH_PREC;

        /* Send measurement command via low-level write */
        char i2c_err[64] = {0};
        int wres = i2c_write_bytes(SHT40_I2C, SHT40_ADDR, &cmd, 1, I2C_FLAG_TIMEOUT_MS, i2c_err, sizeof(i2c_err));
        if (wres != 0) {
            if (usb_configured) {
                char msg[128];
                snprintf(msg, sizeof(msg), "SHT40 write err %d (%s) try%u\r\n", wres, i2c_err, (unsigned)(attempt + 1));
                usb_write_str(msg);
            }
            i2c1_soft_reset(SHT40_I2C);
            i2c_setup();
            bus_reinit_count++;
            delay_ms(2);
            continue;
        }

        uint32_t tstart = millis();
        uint32_t read_attempts = 0;
        uint32_t crc_fail_count = 0;

        while ((millis() - tstart) < SHT40_MEAS_DELAY_MS) {
            memset(buf, 0, sizeof(buf));
            int rres = i2c_read_bytes(SHT40_I2C, SHT40_ADDR, buf, sizeof(buf),
                                      I2C_BYTE_TIMEOUT_MS, i2c_err, sizeof(i2c_err));
            read_attempts++;

            if (rres == 0) {
                if ((sht40_crc8(&buf[0], 2) == buf[2]) &&
                    (sht40_crc8(&buf[3], 2) == buf[5])) {

                    uint16_t t_raw = (uint16_t)((buf[0] << 8) | buf[1]);

                    /* T(°C) = -45 + 175 * t_raw / 65535
                     * T(m°C) = ((21875 * t_raw) >> 13) - 45000
                     */
                    int32_t t_milli = ((21875 * (int32_t)t_raw) >> 13) - 45000;

                    if (t_milli >= 0) *temp_centi = (t_milli + 5) / 10;
                    else              *temp_centi = (t_milli - 5) / 10;

                    return true;
                }
                crc_fail_count++;
            } else {
                /* failed read -> count as failure; log later */
                crc_fail_count++;
                if (usb_configured) {
                    char msg[128];
                    snprintf(msg, sizeof(msg), "SHT40 read err %d (%s) try%u\r\n", rres, i2c_err, (unsigned)(attempt + 1));
                    usb_write_str(msg);
                }
            }

            delay_ms(SHT40_I2C_RETRY_DELAY_MS);
        }

        if (usb_configured) {
            char msg[128];
            snprintf(msg, sizeof(msg), "SHT40 TO %lums crc%lu readAttempt%lu try%u\r\n",
                     (unsigned long)SHT40_MEAS_DELAY_MS,
                     (unsigned long)crc_fail_count,
                     (unsigned long)read_attempts,
                     (unsigned)(attempt + 1));
            usb_write_str(msg);
        }

        /* Recover I2C and retry */
        i2c1_soft_reset(SHT40_I2C);
        i2c_setup();
        bus_reinit_count++;
        delay_ms(2);
    }

    return false;
}

/* ------------ Sample+compare+feed (ONLY place iwdg_reset() occurs) ------------ */

static bool temp_sample_compare_and_feed(bool verbose,
                                         bool *want_heat,
                                         bool *stop_heat)
{
    int32_t tC;

    if (!sht40_read_temperature_centi(&tC)) {
        return false;
    }

    /* Proper hysteresis */
    bool wh = (tC <= (temp_threshold_centi - HYSTERESIS_CENTI));
    bool sh = (tC >= temp_threshold_centi);

    if (want_heat) *want_heat = wh;
    if (stop_heat) *stop_heat = sh;

    if (verbose && usb_configured) {
        char buf[32];
        int32_t tF = centiC_to_centiF(tC);
        centi_to_str(tF, buf, sizeof(buf));
        usb_write_str("Temp = ");
        usb_write_str(buf);
        usb_write_str(" F\r\n");
    }

    /* STRICT RULE: feed watchdog ONLY after temp read + compare */
    iwdg_reset();
    return true;
}

/* ------------ Main ------------ */

int main(void)
{
    clock_setup();
    boot_cause_init();
    gpio_setup();
    i2c_setup();
    systick_setup();
    config_load();
    usb_setup();
    watchdog_setup();

    therm_state_t state = THERM_IDLE;
    uint32_t state_start_ms = millis();
    uint32_t last_verbose_ms = 0;

    /* Ensure heater starts off */
    heater_off();

    while (1) {
        usb_poll();

        bool want_heat = false;
        bool stop_heat = false;

        /* Print cadence control */
        bool verbose_now = false;
        if ((millis() - last_verbose_ms) >= CHECK_INTERVAL_MS) {
            verbose_now = true;
            last_verbose_ms = millis();
        }

        switch (state) {

        case THERM_IDLE:
            heater_off();

            /* Pre-check sensor before ever heating */
            if (!temp_sample_compare_and_feed(verbose_now, &want_heat, &stop_heat)) {
                hard_reset_via_watchdog("SHT40 fail (IDLE)");
            }

            if (want_heat) {
                if (usb_configured) usb_write_str("State: IDLE -> HEATING\r\n");
                heater_on();
                state = THERM_HEATING;
                state_start_ms = millis();
            }
            break;

        case THERM_HEATING:
            /* Enforce max heat time */
            if ((millis() - state_start_ms) >= heat_max_time_ms) {
                heater_off();
                if (usb_configured) usb_write_str("Heat max time -> COOLDOWN\r\n");
                state = THERM_COOLDOWN;
                state_start_ms = millis();
                break;
            }

            if (!temp_sample_compare_and_feed(verbose_now, &want_heat, &stop_heat)) {
                hard_reset_via_watchdog("SHT40 fail (HEATING)");
            }

            if (stop_heat) {
                heater_off();
                if (usb_configured) usb_write_str("Reached setpoint -> COOLDOWN\r\n");
                state = THERM_COOLDOWN;
                state_start_ms = millis();
            }
            break;

        case THERM_COOLDOWN:
            heater_off();

            if (!temp_sample_compare_and_feed(false, &want_heat, &stop_heat)) {
                hard_reset_via_watchdog("SHT40 fail (COOLDOWN)");
            }

            if ((millis() - state_start_ms) >= cooldown_sleep_ms) {
                if (usb_configured) usb_write_str("Cooldown done -> IDLE\r\n");
                state = THERM_IDLE;
                state_start_ms = millis();
            }
            break;

        default:
            hard_reset_via_watchdog("Bad state");
            break;
        }

        /* Sleep until next watchdog-legal sample */
        delay_ms(WDT_FEED_STEP_MS);
    }
}
