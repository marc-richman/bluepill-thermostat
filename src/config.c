#include "config.h"
#include <libopencm3/stm32/flash.h>
#include <string.h>

/* Default values */
static const config_data_t default_cfg = {
    .magic = CONFIG_MAGIC,
    .threshold_centi = 1670,
    .heat_max_time_ms = 300000,
    .cooldown_sleep_ms = 300000,
};

void config_load(config_data_t *out)
{
    const config_data_t *cfg = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (cfg->magic == CONFIG_MAGIC) {
        memcpy(out, cfg, sizeof(config_data_t));
    } else {
        memcpy(out, &default_cfg, sizeof(config_data_t));
    }
}

int config_save(const config_data_t *in)
{
    const config_data_t *old = (const config_data_t *)CONFIG_FLASH_ADDR;
    if (old->magic == in->magic &&
        old->threshold_centi == in->threshold_centi &&
        old->heat_max_time_ms == in->heat_max_time_ms &&
        old->cooldown_sleep_ms == in->cooldown_sleep_ms) {
        return 0; /* unchanged */
    }

    flash_unlock();
    flash_erase_page(CONFIG_FLASH_ADDR);
    const uint32_t *p = (const uint32_t *)in;
    unsigned words = sizeof(*in) / 4;
    for (unsigned i = 0; i < words; i++) {
        flash_program_word(CONFIG_FLASH_ADDR + (i * 4U), p[i]);
    }
    flash_lock();
    return 0;
}