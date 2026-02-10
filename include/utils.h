#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

bool parse_centi(const char *s, int32_t *out);
bool parse_uint_seconds(const char *s, uint32_t *out);
void centi_to_str(int32_t centi, char *buf, size_t len);
int32_t centiC_to_centiF(int32_t cC);
int32_t centiF_to_centiC(int32_t cF);

/* Expose system millis to modules */
extern volatile uint32_t system_millis;

/* Provide prototype for systick handler (defined in main.c) */
void sys_tick_handler(void);

#endif /* UTILS_H */