#include "config.h"
#include <libopencm3/stm32/flash.h>
#include <string.h>

/* Default values */
static const config_data_t default_cfg = {
    .magic = CONFIG_MAGIC,
    .threshold_centi = 1670,
    .heat_max_time_ms = 300000,
    .cooldown_sleep_ms = 300000,
    .fan2_interval_s = 3600,
    .fan2_duration_s = 20,
};

static bool config_is_valid(const config_data_t *cfg)
{
    if (cfg->magic != CONFIG_MAGIC) return false;
    if (cfg->threshold_centi < CONFIG_THRESHOLD_CENTI_MIN ||
        cfg->threshold_centi > CONFIG_THRESHOLD_CENTI_MAX) return false;
    if (cfg->heat_max_time_ms < (CONFIG_RUNTIME_SECONDS_MIN * 1000UL) ||
        cfg->heat_max_time_ms > (CONFIG_RUNTIME_SECONDS_MAX * 1000UL)) return false;
    if (cfg->cooldown_sleep_ms < (CONFIG_RUNTIME_SECONDS_MIN * 1000UL) ||
        cfg->cooldown_sleep_ms > (CONFIG_RUNTIME_SECONDS_MAX * 1000UL)) return false;
    if (cfg->fan2_interval_s < CONFIG_FAN2_INTERVAL_SECONDS_MIN ||
        cfg->fan2_interval_s > CONFIG_FAN2_INTERVAL_SECONDS_MAX) return false;
    if (cfg->fan2_duration_s < CONFIG_FAN2_DURATION_SECONDS_MIN ||
        cfg->fan2_duration_s > CONFIG_FAN2_DURATION_SECONDS_MAX) return false;
    return true;
}

void config_load(config_data_t *out)
{
    const config_data_t *cfg = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (config_is_valid(cfg)) {
        memcpy(out, cfg, sizeof(config_data_t));
    } else {
        memcpy(out, &default_cfg, sizeof(config_data_t));
    }
}

int config_save(const config_data_t *in)
{
    config_data_t sanitized = *in;
    if (!config_is_valid(&sanitized)) {
        sanitized = default_cfg;
    }

    const config_data_t *old = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (old->magic == sanitized.magic &&
        old->threshold_centi == sanitized.threshold_centi &&
        old->heat_max_time_ms == sanitized.heat_max_time_ms &&
        old->cooldown_sleep_ms == sanitized.cooldown_sleep_ms &&
        old->fan2_interval_s == sanitized.fan2_interval_s &&
        old->fan2_duration_s == sanitized.fan2_duration_s) {
        return 0; /* unchanged */
    }

    flash_unlock();
    flash_erase_page(CONFIG_FLASH_ADDR);
    const uint32_t *p = (const uint32_t *)&sanitized;
    unsigned words = sizeof(*in) / 4;
    for (unsigned i = 0; i < words; i++) {
        flash_program_word(CONFIG_FLASH_ADDR + (i * 4U), p[i]);
    }
    flash_lock();
    return 0;
}
