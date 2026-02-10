#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define CONFIG_FLASH_ADDR 0x0800FC00UL
#define CONFIG_MAGIC 0xDEADBEEF

typedef struct {
    uint32_t magic;
    int32_t threshold_centi;
    uint32_t heat_max_time_ms;
    uint32_t cooldown_sleep_ms;
} config_data_t;

void config_load(config_data_t *out);
int config_save(const config_data_t *in);

#endif /* CONFIG_H */