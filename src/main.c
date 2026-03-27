/*
 * src/main.c
 *
 * Main thermostat state machine for Bluepill with SHT40.
 *
 * Uses modules:
 *  - i2c_sht40: SHT40 sensor access over I2C
 *  - usb_console: non-blocking USB CDC console + command handling
 *  - config: flash-stored configuration
 *  - utils: parsing/formatting helpers and shared millis
 *
 * The shared millisecond timebase is in utils.c as 'volatile uint32_t system_millis'.
 * sys_tick_handler is declared in include/utils.h and defined below to increment it.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "i2c_sht40.h"
#include "usb_console.h"
#include "config.h"
#include "utils.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>

#include "pinmap.h"

/* Timing */
#define CHECK_INTERVAL_MS   (30UL * 1000UL)
#define WDT_FEED_STEP_MS    (10UL * 1000UL)

/* Hysteresis (centi-degrees C) */
#define HYSTERESIS_CENTI    50

/* SysTick handler: increments shared millis (prototype in utils.h) */
void sys_tick_handler(void)
{
    system_millis++;
}

/* Simple heater control wrappers */
static inline void heater_on(void)  { gpio_set(HEATER_PORT, HEATER_PIN); }
static inline void heater_off(void) { gpio_clear(HEATER_PORT, HEATER_PIN); }
/* Fan2 control wrappers */
static inline void fan2_on(void)  { gpio_set(FAN2_PORT, FAN2_PIN); }
static inline void fan2_off(void) { gpio_clear(FAN2_PORT, FAN2_PIN); }

/* Clock + systick setup */
static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1); /* 1 ms tick at 72 MHz AHB */
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();
}

/* Boot cause text */
static char boot_cause_msg[128] = "Boot cause: unknown";
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

/* GPIO setup for heater pin */
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(HEATER_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, HEATER_PIN);
    heater_off();

    /* Fan2 pin (optional) */
    gpio_set_mode(FAN2_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, FAN2_PIN);
    fan2_off();
}


/* Watchdog setup */
static void watchdog_setup(void)
{
    iwdg_set_period_ms(20000); /* 20s timeout */
    iwdg_start();
}

/* Polling delay that also services USB console (does not feed watchdog) */
static void delay_ms_poll_usb(uint32_t ms)
{
    uint32_t start = system_millis;
    while ((system_millis - start) < ms) {
        usb_console_poll();
    }
}

/* Global configuration instance */
static config_data_t cfg;
static bool fan2_enabled = true;

/* Command handler registered with usb_console: updates cfg and saves when needed */
static void on_command(const char *cmd)
{
    if (!cmd || *cmd == '\0') return;

    /* Threshold handling: default (unqualified) is Fahrenheit */
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
            usb_console_write("ERR: bad threshold value\r\n");
            return;
        }

        bool is_celsius = false;
        if (strncmp(cmd, "tC=", 3) == 0 || strncmp(cmd, "thresholdC=", 11) == 0)
            is_celsius = true;

        int32_t new_thr_c = is_celsius ? in_centi : centiF_to_centiC(in_centi);

        if (new_thr_c != cfg.threshold_centi) {
            cfg.threshold_centi = new_thr_c;
            usb_console_write("OK: threshold updated\r\n");
            config_save(&cfg);
        } else {
            usb_console_write("No change: threshold unchanged\r\n");
        }

        /* Print config */
        {
            char buf[32];
            usb_console_write("\r\nCurrent config:\r\n");
            int32_t thr_f = centiC_to_centiF(cfg.threshold_centi);
            centi_to_str(thr_f, buf, sizeof(buf));
            usb_console_write("  threshold = "); usb_console_write(buf); usb_console_write(" F\r\n");
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)(cfg.heat_max_time_ms / 1000UL));
            usb_console_write("  run       = "); usb_console_write(buf); usb_console_write(" s\r\n");
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)(cfg.cooldown_sleep_ms / 1000UL));
            usb_console_write("  sleep     = "); usb_console_write(buf); usb_console_write(" s\r\n");
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.fan2_interval_s);
            usb_console_write("  fan2_int  = "); usb_console_write(buf); usb_console_write(" s\r\n");
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.fan2_duration_s);
            usb_console_write("  fan2_dur  = "); usb_console_write(buf); usb_console_write(" s\r\n\r\n");
        }

        return;
    }

    if (strncmp(cmd, "run=", 4) == 0 || strncmp(cmd, "r=", 2) == 0) {
        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;
        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0) {
            uint32_t new_ms = secs * 1000UL;
            if (new_ms != cfg.heat_max_time_ms) {
                cfg.heat_max_time_ms = new_ms;
                usb_console_write("OK: run time updated\r\n");
                config_save(&cfg);
            } else {
                usb_console_write("No change: run time unchanged\r\n");
            }
        } else {
            usb_console_write("ERR: bad run value\r\n");
        }
        return;
    }

    if (strncmp(cmd, "sleep=", 6) == 0 || strncmp(cmd, "s=", 2) == 0) {
        const char *valstr = strchr(cmd, '=');
        if (valstr) valstr++;
        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0) {
            uint32_t new_ms = secs * 1000UL;
            if (new_ms != cfg.cooldown_sleep_ms) {
                cfg.cooldown_sleep_ms = new_ms;
                usb_console_write("OK: sleep time updated\r\n");
                config_save(&cfg);
            } else {
                usb_console_write("No change: sleep time unchanged\r\n");
            }
        } else {
            usb_console_write("ERR: bad sleep value\r\n");
        }
        return;
    }

    if (strncmp(cmd, "fan2_interval=", 14) == 0 || strncmp(cmd, "fi=", 3) == 0) {
        const char *valstr = strchr(cmd, '='); if (valstr) valstr++;
        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0) {
            if (secs != cfg.fan2_interval_s) {
                cfg.fan2_interval_s = secs;
                usb_console_write("OK: fan2 interval updated\r\n");
                config_save(&cfg);
            } else {
                usb_console_write("No change: fan2 interval unchanged\r\n");
            }
        } else {
            usb_console_write("ERR: bad fan2 interval value\r\n");
        }
        return;
    }

    if (strncmp(cmd, "fan2_duration=", 14) == 0 || strncmp(cmd, "fd=", 3) == 0) {
        const char *valstr = strchr(cmd, '='); if (valstr) valstr++;
        uint32_t secs;
        if (valstr && parse_uint_seconds(valstr, &secs) && secs > 0) {
            if (secs != cfg.fan2_duration_s) {
                cfg.fan2_duration_s = secs;
                usb_console_write("OK: fan2 duration updated\r\n");
                config_save(&cfg);
            } else {
                usb_console_write("No change: fan2 duration unchanged\r\n");
            }
        } else {
            usb_console_write("ERR: bad fan2 duration value\r\n");
        }
        return;
    }

    if (strcmp(cmd, "fan enable") == 0) {
        if (!fan2_enabled) {
            fan2_enabled = true;
            usb_console_write("OK: fan enabled\r\n");
        } else {
            usb_console_write("No change: fan already enabled\r\n");
        }
        return;
    }

    if (strcmp(cmd, "fan disable") == 0) {
        if (fan2_enabled) {
            fan2_enabled = false;
            fan2_off();
            usb_console_write("OK: fan disabled\r\n");
        } else {
            usb_console_write("No change: fan already disabled\r\n");
        }
        return;
    }

    if (strcmp(cmd, "status") == 0 || strcmp(cmd, "config") == 0) {
        char buf[64];
        usb_console_write("\r\nCurrent config:\r\n");
        int32_t thr_f = centiC_to_centiF(cfg.threshold_centi);
        centi_to_str(thr_f, buf, sizeof(buf));
        usb_console_write("  threshold = "); usb_console_write(buf); usb_console_write(" F\r\n");
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)(cfg.heat_max_time_ms / 1000UL));
        usb_console_write("  run       = "); usb_console_write(buf); usb_console_write(" s\r\n");
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)(cfg.cooldown_sleep_ms / 1000UL));
        usb_console_write("  sleep     = "); usb_console_write(buf); usb_console_write(" s\r\n");
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.fan2_interval_s);
            usb_console_write("  fan2_int  = "); usb_console_write(buf); usb_console_write(" s\r\n");
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.fan2_duration_s);
            usb_console_write("  fan2_dur  = "); usb_console_write(buf); usb_console_write(" s\r\n\r\n");
            usb_console_write("  fan       = "); usb_console_write(fan2_enabled ? "enabled\r\n\r\n" : "disabled\r\n\r\n");
        return;
    }

    usb_console_write("Unknown command. Supported:\r\n");
    usb_console_write("  t=<degF> / threshold=<degF> (default)\r\n");
    usb_console_write("  tF=<degF> / thresholdF=<degF>\r\n");
    usb_console_write("  tC=<degC> / thresholdC=<degC>\r\n");
    usb_console_write("  run=<seconds> or r=<seconds>\r\n");
    usb_console_write("  sleep=<seconds> or s=<seconds>\r\n");
    usb_console_write("  fan2_interval=<seconds> or fi=<seconds>\r\n");
    usb_console_write("  fan2_duration=<seconds> or fd=<seconds>\r\n");
    usb_console_write("  fan enable / fan disable\r\n");
    usb_console_write("  status\r\n\r\n");
}

/* Sample sensor, apply hysteresis and feed watchdog on success */
static bool sample_compare_and_feed(bool verbose, bool *want_heat, bool *stop_heat)
{
    int32_t tC;
    if (sht40_read_temperature_centi(&tC) != SHT40_OK) {
        return false;
    }

    bool wh = (tC <= (cfg.threshold_centi - HYSTERESIS_CENTI));
    bool sh = (tC >= cfg.threshold_centi);

    if (want_heat) *want_heat = wh;
    if (stop_heat) *stop_heat = sh;

    if (verbose && usb_console_configured()) {
        char buf[32];
        int32_t tF = centiC_to_centiF(tC);
        centi_to_str(tF, buf, sizeof(buf));
        usb_console_write("Temp = ");
        usb_console_write(buf);
        usb_console_write(" F\r\n");
    }

    /* Feed watchdog only after successful read/compare */
    iwdg_reset();
    return true;
}

int main(void)
{
    clock_setup();
    boot_cause_init();
    gpio_setup();
    systick_setup();

    /* Module initializations */
    sht40_i2c_init();
    usb_console_init();
    usb_console_set_cmd_handler(on_command);

    watchdog_setup();

    /* Load persistent config */
    config_load(&cfg);

    /* Print welcome (note: usb_console may not be configured yet; messages are queued) */
    usb_console_write("\r\nBluepill heater controller with SHT40\r\n");
    usb_console_write(boot_cause_msg);
    usb_console_write("\r\nCommands:\r\n");
    usb_console_write("  t=<degF> / threshold=<degF> (default)\r\n");
    usb_console_write("  tF=<degF> / thresholdF=<degF>\r\n");
    usb_console_write("  tC=<degC> / thresholdC=<degC>\r\n");
    usb_console_write("  run=<seconds> or r=<seconds>\r\n");
    usb_console_write("  sleep=<seconds> or s=<seconds>\r\n");
    usb_console_write("  fan2_interval=<seconds> or fi=<seconds>\r\n");
    usb_console_write("  fan2_duration=<seconds> or fd=<seconds>\r\n");
    usb_console_write("  fan enable / fan disable\r\n");
    usb_console_write("  status\r\n\r\n");

    /* Ensure heater is off */
    heater_off();

    /* Thermostat state machine */
    enum { IDLE = 0, HEATING, COOLDOWN } state = IDLE;
    uint32_t state_start_ms = system_millis;
    uint32_t last_verbose_ms = 0;
    /* Fan2 state */
    uint32_t fan2_last_ms = system_millis; /* last time fan2 completed or timer started */
    bool fan2_active = false;
    uint32_t fan2_on_start_ms = 0;
    bool fan2_pending = false; /* interval elapsed while heater running */
    bool heat_pending = false; /* temp called for heat while fan2 was running */

    while (1) {
        usb_console_poll();

        bool want_heat = false, stop_heat = false;
        bool verbose_now = false;
        if ((system_millis - last_verbose_ms) >= CHECK_INTERVAL_MS) {
            verbose_now = true;
            last_verbose_ms = system_millis;
        }

        switch (state) {
        case IDLE:
            heater_off();
            if (!sample_compare_and_feed(verbose_now, &want_heat, &stop_heat)) {
                usb_console_write("SHT40 fail (IDLE)\r\n");
                /* Stop feeding watchdog so device resets */
                while (1) { usb_console_poll(); }
            }
            if (want_heat) {
                if (fan2_active) {
                    /* Fan2 and heater are mutually exclusive. Defer heat until fan2 stops. */
                    heat_pending = true;
                    if (usb_console_configured()) usb_console_write("Heat deferred: FAN2 active\r\n");
                } else {
                    heat_pending = false;
                    if (usb_console_configured()) usb_console_write("State: IDLE -> HEATING\r\n");
                    heater_on();
                    state = HEATING;
                    state_start_ms = system_millis;
                }
            } else {
                heat_pending = false;
            }
            break;

        case HEATING:
            if ((system_millis - state_start_ms) >= cfg.heat_max_time_ms) {
                heater_off();
                if (usb_console_configured()) usb_console_write("Heat max time -> COOLDOWN\r\n");
                state = COOLDOWN;
                state_start_ms = system_millis;
                break;
            }
            if (!sample_compare_and_feed(verbose_now, &want_heat, &stop_heat)) {
                usb_console_write("SHT40 fail (HEATING)\r\n");
                while (1) { usb_console_poll(); }
            }
            if (stop_heat) {
                heater_off();
                if (usb_console_configured()) usb_console_write("Reached setpoint -> COOLDOWN\r\n");
                state = COOLDOWN;
                state_start_ms = system_millis;
            }
            break;

        case COOLDOWN:
            heater_off();
            if (!sample_compare_and_feed(false, &want_heat, &stop_heat)) {
                usb_console_write("SHT40 fail (COOLDOWN)\r\n");
                while (1) { usb_console_poll(); }
            }
            if ((system_millis - state_start_ms) >= cfg.cooldown_sleep_ms) {
                if (usb_console_configured()) usb_console_write("Cooldown done -> IDLE\r\n");
                state = IDLE;
                state_start_ms = system_millis;
            }
            break;

        default:
            /* Fatal: stop feeding watchdog */
            while (1) { usb_console_poll(); }
        }

        /* --- Fan2 periodic control ---
         * Behavior:
         *  - Set FAN2_PIN high for cfg.fan2_duration_s seconds every
         *    cfg.fan2_interval_s seconds.
         *  - If the interval elapses while heater is running, defer
         *    the fan until heater stops; when heater stops, immediately
         *    run the fan for the configured duration and restart the
         *    interval counter.
         */
        if (!fan2_enabled) {
            if (fan2_active) {
                fan2_off();
                fan2_active = false;
            }
            fan2_pending = false;
            heat_pending = false;
            fan2_last_ms = system_millis;
        } else if (fan2_active) {
            /* turn off after duration */
            if ((system_millis - fan2_on_start_ms) >= (cfg.fan2_duration_s * 1000UL)) {
                fan2_off();
                fan2_active = false;
                fan2_last_ms = system_millis; /* restart interval from now */

                /* If heat was requested while FAN2 was active, allow heater to start on the next loop. */
                if (heat_pending && usb_console_configured()) {
                    usb_console_write("FAN2 done: heat may resume\r\n");
                }
            }
        } else {
            /* Not currently running */
            if (fan2_pending && state != HEATING) {
                /* heater stopped — run fan now */
                fan2_on();
                fan2_active = true;
                fan2_on_start_ms = system_millis;
                fan2_pending = false;
            } else if (!fan2_pending) {
                /* check interval expiry */
                uint32_t interval_ms = cfg.fan2_interval_s * 1000UL;
                if ((system_millis - fan2_last_ms) >= interval_ms) {
                    if (state == HEATING) {
                        /* defer until heater stops */
                        fan2_pending = true;
                    } else {
                        /* start fan now */
                        fan2_on();
                        fan2_active = true;
                        fan2_on_start_ms = system_millis;
                    }
                }
            }
        }

        /* Sleep until next watchdog-legal sample; usb is polled inside */
        delay_ms_poll_usb(WDT_FEED_STEP_MS);
    }

    return 0;
}
