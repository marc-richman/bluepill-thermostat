#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <libopencm3/cm3/common.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <libopencm3/cm3/systick.h>

/* ------------ Application configuration ------------ */

/* Heater control pin: PB0, push-pull output, HIGH = ON */
#define HEATER_PORT                     GPIOB
#define HEATER_PIN                      GPIO0

/* I2C1 for SHT40: PB6 = SCL, PB7 = SDA */
#define SHT40_I2C                       I2C1
#define SHT40_ADDR                      0x44    /* 7-bit address (typical SHT40) */
#define SHT40_CMD_MEAS_HIGH_PREC        0xFD    /* High-precision single-shot */

/* I2C timing constants (for STM32F103: HSE 8MHz -> SYSCLK 72MHz, PCLK1 = 36MHz) */
#define I2C_PCLK1_FREQ_MHZ              36U
#define I2C_BUS_FREQ_HZ                 100000U /* 100 kHz standard mode */
#define I2C_CCR_SM_100KHZ_36MHZ         180U    /* 36MHz / (2 * 100kHz) */
#define I2C_TRISE_SM_36MHZ              (I2C_PCLK1_FREQ_MHZ + 1U)

/* SHT40 measurement timing & retries */
#define SHT40_MEAS_MAX_TIME_MS          9U      /* from datasheet, high-precision */
#define SHT40_MEAS_DELAY_MARGIN_MS      11U     /* safety margin */
#define SHT40_MEAS_DELAY_MS             (SHT40_MEAS_MAX_TIME_MS + SHT40_MEAS_DELAY_MARGIN_MS) /* 20 ms total */

#define SHT40_I2C_MAX_RETRIES           3U      /* simple retry-on-CRC-fail */
#define SHT40_I2C_RETRY_DELAY_MS        2U      /* short delay between retries */

/* Defaults: in centi-degrees and milliseconds */
#define DEFAULT_TEMP_THRESHOLD_CENTI    1670    /* 16.70 °C initial threshold */
#define HYSTERESIS_CENTI                50      /* 0.50 °C: turn off at threshold - 0.5 °C */

#define DEFAULT_HEAT_MAX_TIME_MS        (300UL * 1000UL)  /* 300 s run */
#define DEFAULT_COOLDOWN_SLEEP_MS       (300UL * 1000UL)  /* 300 s sleep */

/* Temperature check interval */
#define CHECK_INTERVAL_MS               (30UL * 1000UL)   /* 30 s */

/* Config storage in flash (last 1K page of 64K flash): 0x0800FC00
 * NOTE: This assumes an STM32F103C8 with 64K flash and 1K pages.
 * If code grows too large or you use a different part, adjust this.
 */
#define CONFIG_FLASH_ADDR               0x0800FC00
#define CONFIG_MAGIC                    0xDEADBEEF

/* ------------ Global state ------------ */

static volatile uint32_t system_millis = 0;

/* Configurable at run-time via USB CDC console */
static int32_t temp_threshold_centi = DEFAULT_TEMP_THRESHOLD_CENTI;
static uint32_t heat_max_time_ms    = DEFAULT_HEAT_MAX_TIME_MS;
static uint32_t cooldown_sleep_ms   = DEFAULT_COOLDOWN_SLEEP_MS;

/* USB CDC stuff */
static usbd_device *usbdev;
static bool usb_configured = false;

/* Simple command line buffer */
static char cmd_buf[64];
static uint8_t cmd_len = 0;

/* Boot cause message (printed after USB configuration) */
static char boot_cause_msg[128] = "Boot cause: unknown";

/* ------------ Config struct in flash ------------ */

typedef struct {
    uint32_t magic;
    int32_t  threshold_centi;
    uint32_t heat_max_time_ms;
    uint32_t cooldown_sleep_ms;
} config_data_t;

/* ------------ Timebase / delays ------------ */

/* Prototype to satisfy -Wmissing-prototypes for ISR */
void sys_tick_handler(void);

static uint32_t millis(void)
{
    return system_millis;
}

void sys_tick_handler(void)
{
    system_millis++;
}

/* delay_ms keeps calling usb_poll() and feeding watchdog */
static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        if (usbdev) {
            usbd_poll(usbdev);  /* keep USB CDC alive */
        }
        iwdg_reset();          /* feed the watchdog */
    }
}

/* ------------ Forward declarations for internal helpers ------------ */

static void usb_poll(void);
static void print_config(void);
static void config_load(void);
static void config_save(void);
static void boot_cause_init(void);
static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);
static void i2c_setup(void);
static void watchdog_setup(void);
static void usb_setup(void);

/* ------------ Clock & SysTick setup ------------ */

static void clock_setup(void)
{
    /* Bluepill: 8MHz HSE -> 72MHz system clock using new PLL helper */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

static void systick_setup(void)
{
    /* SysTick at 1kHz (1ms) from AHB clock (72MHz) */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1);  /* 72MHz / 72000 = 1kHz */
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();
}

/* ------------ Boot cause decoding ------------ */

static void boot_cause_init(void)
{
    /* Read and decode RCC_CSR reset flags */
    uint32_t csr = RCC_CSR;

    strcpy(boot_cause_msg, "Boot cause:");

    bool any = false;
    if (csr & RCC_CSR_PORRSTF) {
        strcat(boot_cause_msg, " POR");
        any = true;
    }
    if (csr & RCC_CSR_PINRSTF) {
        strcat(boot_cause_msg, " PIN");
        any = true;
    }
    if (csr & RCC_CSR_SFTRSTF) {
        strcat(boot_cause_msg, " SW");
        any = true;
    }
    if (csr & RCC_CSR_IWDGRSTF) {
        strcat(boot_cause_msg, " IWDG");
        any = true;
    }
    if (csr & RCC_CSR_WWDGRSTF) {
        strcat(boot_cause_msg, " WWDG");
        any = true;
    }
    if (csr & RCC_CSR_LPWRRSTF) {
        strcat(boot_cause_msg, " LPWR");
        any = true;
    }

    if (!any) {
        strcat(boot_cause_msg, " unknown");
    }

    /* Clear reset flags */
    RCC_CSR |= RCC_CSR_RMVF;
}

/* ------------ GPIO & heater setup ------------ */

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Heater pin: push-pull output */
    gpio_set_mode(HEATER_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, HEATER_PIN);
    gpio_clear(HEATER_PORT, HEATER_PIN); /* heater OFF */
}

static inline void heater_on(void)
{
    gpio_set(HEATER_PORT, HEATER_PIN);
}

static inline void heater_off(void)
{
    gpio_clear(HEATER_PORT, HEATER_PIN);
}

/* ------------ I2C1 reset helper (used by SHT40) ------------ */

/* Improved software reset: disable peripheral, toggle SWRST, leave disabled so
 * caller can re-configure cleanly.
 */
static void i2c1_soft_reset(uint32_t i2c)
{
    /* Ensure peripheral is disabled while we toggle SWRST */
    i2c_peripheral_disable(i2c);

    /* Toggle software reset */
    I2C_CR1(i2c) |= I2C_CR1_SWRST;
    /* short delay to ensure reset propagates */
    for (volatile int i = 0; i < 100; i++) __asm__("nop");
    I2C_CR1(i2c) &= ~I2C_CR1_SWRST;

    /* Leave peripheral disabled — caller will configure and enable */
}

/* ------------ I2C1 setup for SHT40 ------------ */

static void i2c_setup(void)
{
    /* Enable clocks */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);

    /* PB6 = SCL, PB7 = SDA, alt-function open-drain */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);

    /* Best practice: disable, soft-reset, then configure and enable */
    i2c_peripheral_disable(SHT40_I2C);
    i2c1_soft_reset(SHT40_I2C);

    /* PCLK1 is 36MHz in the HSE8→72MHz setup */
    i2c_set_clock_frequency(SHT40_I2C, I2C_PCLK1_FREQ_MHZ);

    /* Standard mode (100kHz) */
    i2c_set_standard_mode(SHT40_I2C);

    /* CCR and TRISE chosen for 100kHz @ 36MHz PCLK1 */
    i2c_set_ccr(SHT40_I2C, I2C_CCR_SM_100KHZ_36MHZ);
    i2c_set_trise(SHT40_I2C, I2C_TRISE_SM_36MHZ);

    i2c_peripheral_enable(SHT40_I2C);
}

/* ------------ Watchdog setup (≈20s timeout) ------------ */

static void watchdog_setup(void)
{
    /* ~20s timeout using libopencm3 helper */
    iwdg_set_period_ms(20000);
    iwdg_start();
}

/* ------------ USB CDC-ACM setup & helpers ------------ */

static uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,       /* generic ST-like VID */
    .idProduct = 0x5740,      /* generic VCP PID */
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,           /* EP3 IN (interrupt) */
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,           /* EP1 OUT */
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,           /* EP2 IN */
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
        .bFunctionLength      = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType      = CS_INTERFACE,
        .bDescriptorSubtype   = USB_CDC_TYPE_UNION,
        .bControlInterface    = 0,
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
    .bmAttributes = 0x80, /* bus powered */
    .bMaxPower = 0x32,    /* 100 mA */
    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Bluepill",
    "Heater SHT40 controller",
    "0001",
};

/* Forward declarations for USB callbacks with correct types */
static enum usbd_request_return_codes
cdcacm_control_request(usbd_device *usbd_dev,
                       struct usb_setup_data *req,
                       uint8_t **buf, uint16_t *len,
                       usbd_control_complete_callback *complete);
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);

/* On many Bluepill variants the D+ pull-up is tied to PA12. Briefly drive
 * PA12 low as a GPIO so the host reliably sees a USB disconnect/reconnect
 * when the firmware starts. On boards with a fixed pull-up this just sinks
 * a couple of mA and is harmless.
 */
static void usb_force_reenumeration(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Drive PA12 low for a short time */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);

    for (volatile uint32_t i = 0; i < 800000; i++) {
        __asm__("nop");
    }

    /* Let USB peripheral take over PA11/PA12 */
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

static void usb_poll(void)
{
    if (usbdev) {
        usbd_poll(usbdev);
    }
}

/* simple write helper */
static void usb_write_str(const char *s)
{
    if (!usb_configured) return;

    while (*s) {
        char buf[64];
        int i = 0;
        while (*s && i < 64) {
            buf[i++] = *s++;
        }
        usbd_ep_write_packet(usbdev, 0x82, buf, i);
    }
}

/* ------------ Config printing & parsing helpers ------------ */

/* Convert signed centi-degrees to "[-]xx.yy" string */
static void centi_to_str(int32_t centi, char *buf, size_t len)
{
    bool neg = false;
    if (centi < 0) {
        neg = true;
        centi = -centi;
    }
    int32_t ip = centi / 100;
    int32_t fp = centi % 100;
    if (neg) {
        snprintf(buf, len, "-%ld.%02ld", (long)ip, (long)fp);
    } else {
        snprintf(buf, len, "%ld.%02ld", (long)ip, (long)fp);
    }
}

/* Parse "<int>[.fraction]" into centi-degrees (0.01 units) */
static bool parse_centi(const char *s, int32_t *out)
{
    bool neg = false;
    int32_t ip = 0;
    int32_t fp = 0;
    int fp_digits = 0;

    while (*s == ' ' || *s == '\t') s++;

    if (*s == '+' || *s == '-') {
        if (*s == '-') neg = true;
        s++;
    }

    if (*s < '0' || *s > '9') {
        return false;
    }

    while (*s >= '0' && *s <= '9') {
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
        while (*s >= '0' && *s <= '9') {
            s++;
        }
    }

    if (fp_digits == 0) {
        fp = 0;
    } else if (fp_digits == 1) {
        fp *= 10;  /* ".5" => 50 */
    }

    int32_t val = ip * 100 + fp;
    if (neg) val = -val;

    *out = val;
    return true;
}

/* Parse unsigned integer seconds */
static bool parse_uint_seconds(const char *s, uint32_t *out)
{
    uint32_t val = 0;

    while (*s == ' ' || *s == '\t') s++;
    if (*s < '0' || *s > '9') return false;

    while (*s >= '0' && *s <= '9') {
        val = val * 10 + (uint32_t)(*s - '0');
        s++;
    }

    *out = val;
    return true;
}

/* Print current configuration */
static void print_config(void)
{
    char buf[32];

    usb_write_str("\r\nCurrent config:\r\n");

    centi_to_str(temp_threshold_centi, buf, sizeof(buf));
    usb_write_str("  threshold = ");
    usb_write_str(buf);
    usb_write_str(" C\r\n");

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(heat_max_time_ms / 1000UL));
    usb_write_str("  run       = ");
    usb_write_str(buf);
    usb_write_str(" s\r\n");

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(cooldown_sleep_ms / 1000UL));
    usb_write_str("  sleep     = ");
    usb_write_str(buf);
    usb_write_str(" s\r\n\r\n");
}

/* ------------ Config load/save to flash ------------ */

static void config_load(void)
{
    const config_data_t *cfg = (const config_data_t *)CONFIG_FLASH_ADDR;

    if (cfg->magic == CONFIG_MAGIC) {
        temp_threshold_centi = cfg->threshold_centi;
        heat_max_time_ms     = cfg->heat_max_time_ms;
        cooldown_sleep_ms    = cfg->cooldown_sleep_ms;
    } else {
        /* No valid config in flash; keep defaults */
    }
}

static void config_save(void)
{
    config_data_t cfg;
    cfg.magic             = CONFIG_MAGIC;
    cfg.threshold_centi   = temp_threshold_centi;
    cfg.heat_max_time_ms  = heat_max_time_ms;
    cfg.cooldown_sleep_ms = cooldown_sleep_ms;

    /* If the flash already contains identical config, skip write to save wear */
    const config_data_t *old = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (old->magic == CONFIG_MAGIC &&
        old->threshold_centi == cfg.threshold_centi &&
        old->heat_max_time_ms == cfg.heat_max_time_ms &&
        old->cooldown_sleep_ms == cfg.cooldown_sleep_ms) {
        usb_write_str("Config unchanged; not writing flash.\r\n");
        return;
    }

    const uint32_t *p = (const uint32_t *)&cfg;
    unsigned int words = sizeof(cfg) / 4;

    flash_unlock();
    flash_erase_page(CONFIG_FLASH_ADDR);

    for (unsigned int i = 0; i < words; i++) {
        flash_program_word(CONFIG_FLASH_ADDR + (i * 4U), p[i]);
        /* Feed watchdog in case flash programming takes long */
        iwdg_reset();
    }

    flash_lock();

    usb_write_str("Config saved to flash.\r\n");
}

/* ------------ Command handling ------------ */

static void handle_command(const char *cmd)
{
    if (cmd[0] == '\0') return;

    /* threshold / t= */
    if (strncmp(cmd, "threshold=", 10) == 0 ||
        strncmp(cmd, "t=", 2) == 0) {

        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;

        int32_t new_thr;
        if (valstr && parse_centi(valstr, &new_thr)) {
            if (new_thr != temp_threshold_centi) {
                temp_threshold_centi = new_thr;
                usb_write_str("OK: threshold updated\r\n");
                config_save();
            } else {
                usb_write_str("No change: threshold unchanged\r\n");
            }
            print_config();
        } else {
            usb_write_str("ERR: bad threshold value\r\n");
        }
        return;
    }

    /* run / r= */
    if (strncmp(cmd, "run=", 4) == 0 ||
        strncmp(cmd, "r=", 2) == 0) {

        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;

        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0) {
            uint32_t new_run_ms = secs * 1000UL;
            if (new_run_ms != heat_max_time_ms) {
                heat_max_time_ms = new_run_ms;
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

    /* sleep / s= */
    if (strncmp(cmd, "sleep=", 6) == 0 ||
        strncmp(cmd, "s=", 2) == 0) {

        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;

        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0) {
            uint32_t new_sleep_ms = secs * 1000UL;
            if (new_sleep_ms != cooldown_sleep_ms) {
                cooldown_sleep_ms = new_sleep_ms;
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
    usb_write_str("  threshold=<degC> or t=<degC>\r\n");
    usb_write_str("  run=<seconds>    or r=<seconds>\r\n");
    usb_write_str("  sleep=<seconds>  or s=<seconds>\r\n");
    usb_write_str("  status\r\n\r\n");
}

/* USB CDC control request handler (updated signature) */
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
        /* Could inspect req->wValue for DTR/RTS here if needed */
        return USBD_REQ_HANDLED;

    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding)) {
            return USBD_REQ_NOTSUPP;
        }
        /* Could parse and apply baud/parity/etc. here if desired */
        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}

/* RX callback: read data, accumulate into cmd_buf, process on \r/\n
 * Echo the command back to the host (ACK/echo) before processing.
 */
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

                /* Echo/ACK the command */
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

/* Called when host sets configuration */
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

    usb_write_str("\r\nBluepill heater controller with SHT40\r\n");
    usb_write_str(boot_cause_msg);
    usb_write_str("\r\n");
    usb_write_str("Commands:\r\n");
    usb_write_str("  threshold=<degC> or t=<degC>\r\n");
    usb_write_str("  run=<seconds>    or r=<seconds>\r\n");
    usb_write_str("  sleep=<seconds>  or s=<seconds>\r\n");
    usb_write_str("  status\r\n\r\n");
    print_config();
}

/* ------------ SHT40 temperature read ------------ */

/* SHT4x frame: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC.
 * CRC-8: polynomial 0x31, init 0xFF.
 */
static uint8_t sht40_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/* Read temperature from SHT40 and return in centi-degrees Celsius.
 * Behavior:
 *  - Send measurement command (high precision)
 *  - Repeatedly attempt reading the 6-byte frame until a timeout (SHT40_MEAS_DELAY_MS)
 *  - If no valid frame obtained, soft-reset I2C and retry (up to SHT40_I2C_MAX_RETRIES)
 *
 * Note: libopencm3's i2c_transfer7 is used (may be blocking). We handle
 * time-based retries around reads and perform I2C soft-reset on persistent failure.
 */
static bool sht40_read_temperature_centi(int32_t *temp_centi)
{
    uint8_t cmd;
    uint8_t buf[6];

    for (uint8_t attempt = 0; attempt < SHT40_I2C_MAX_RETRIES; attempt++) {
        cmd = SHT40_CMD_MEAS_HIGH_PREC;

        /* Issue one-shot measurement command (no immediate check of transfer) */
        i2c_transfer7(SHT40_I2C, SHT40_ADDR, &cmd, 1, NULL, 0);

        /* Wait up to SHT40_MEAS_DELAY_MS for a valid 6-byte frame.
         * We repeatedly attempt the read; if the sensor NACKs or returns
         * invalid CRC we retry until timeout.
         */
        uint32_t tstart = millis();
        bool got_good = false;

        while ((millis() - tstart) < SHT40_MEAS_DELAY_MS) {
            /* Attempt to read 6 bytes */
            i2c_transfer7(SHT40_I2C, SHT40_ADDR, NULL, 0, buf, sizeof(buf));

            /* Validate CRC for both temperature and humidity words.
             * If CRC matches, parse and return success.
             * If CRC fails, do a short backoff and try reading again until timeout.
             */
            if ((sht40_crc8(&buf[0], 2) == buf[2]) &&
                (sht40_crc8(&buf[3], 2) == buf[5])) {

                uint16_t t_raw = (uint16_t)((buf[0] << 8) | buf[1]);

                /* Fixed-point conversion from ticks to milli-degC:
                 * T(°C) = -45 + 175 * t_raw / 65535
                 * T(m°C) = ((21875 * t_raw) >> 13) - 45000
                 */
                int32_t t_milli = ((21875 * (int32_t)t_raw) >> 13) - 45000;

                /* Convert to centi-degrees with rounding */
                if (t_milli >= 0) {
                    *temp_centi = (t_milli + 5) / 10;
                } else {
                    *temp_centi = (t_milli - 5) / 10;
                }

                return true;
            }

            /* short pause & keep USB/WDT alive */
            delay_ms(SHT40_I2C_RETRY_DELAY_MS);
        }

        /* If we get here, this attempt timed out without a valid frame.
         * Try a soft reset of I2C and reconfigure before next attempt.
         */
        iwdg_reset();
        i2c1_soft_reset(SHT40_I2C);
        i2c_setup();
        delay_ms(2);
    }

    /* All attempts failed */
    return false;
}

/* ------------ Main control logic ------------ */

int main(void)
{
    clock_setup();
    boot_cause_init();    /* decode reset flags before they're cleared */
    gpio_setup();
    i2c_setup();
    systick_setup();
    config_load();        /* load persisted settings (if any) */
    usb_setup();
    watchdog_setup();     /* ≈20s watchdog */

    while (1) {
        uint32_t start_time;
        bool reached_threshold = false;
        int32_t temp_centi;

        usb_poll();
        usb_write_str("Starting heat cycle...\r\n");
        heater_on();
        start_time = millis();

        while ((millis() - start_time) < heat_max_time_ms) {
            usb_poll();

            if (sht40_read_temperature_centi(&temp_centi)) {
                char buf[32];
                centi_to_str(temp_centi, buf, sizeof(buf));
                usb_write_str("Temp = ");
                usb_write_str(buf);
                usb_write_str(" C\r\n");

                /* Turn off heater if T >= threshold - 0.5°C */
                if (temp_centi >= (temp_threshold_centi - HYSTERESIS_CENTI)) {
                    reached_threshold = true;
                    usb_write_str("Threshold reached, stopping heat.\r\n");
                    break;
                }
            } else {
                usb_write_str("Sensor error, stopping heat.\r\n");
                break;
            }

            /* Wait 30 seconds before next check (heater stays ON) */
            delay_ms(CHECK_INTERVAL_MS);
        }

        heater_off();
        usb_write_str("Heat off.\r\n");

        /* Always sleep for cooldown, whether or not we met the setpoint */
        if (reached_threshold) {
            usb_write_str("Setpoint reached; sleeping cooldown...\r\n");
        } else {
            usb_write_str("Threshold not reached; sleeping cooldown...\r\n");
        }
        delay_ms(cooldown_sleep_ms);
    }
}
