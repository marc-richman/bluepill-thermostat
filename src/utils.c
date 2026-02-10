#include "utils.h"
#include <stdio.h>
#include <stdint.h>

volatile uint32_t system_millis = 0; /* defined here; sys_tick_handler in main increments it */

/* Format centi value like -12.34 or 56.78 */
void centi_to_str(int32_t centi, char *buf, size_t len)
{
    int64_t v = centi;
    bool neg = false;
    if (v < 0) { neg = true; v = -v; }
    int32_t ip = (int32_t)(v / 100);
    int32_t fp = (int32_t)(v % 100);
    snprintf(buf, len, "%s%ld.%02ld", neg ? "-" : "", (long)ip, (long)fp);
}

int32_t centiC_to_centiF(int32_t cC)
{
    int64_t n = (int64_t)cC * 9;
    n = (n >= 0) ? (n + 2) / 5 : (n - 2) / 5;
    return (int32_t)n + 3200;
}

int32_t centiF_to_centiC(int32_t cF)
{
    int64_t n = (int64_t)(cF - 3200) * 5;
    n = (n >= 0) ? (n + 4) / 9 : (n - 4) / 9;
    return (int32_t)n;
}

bool parse_centi(const char *s, int32_t *out)
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
        if (ip > (100000 / 100)) return false;
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
    if (val > 100000) return false;
    if (neg) val = -val;
    *out = val;
    return true;
}

bool parse_uint_seconds(const char *s, uint32_t *out)
{
    uint32_t val = 0;
    while (*s == ' ' || *s == '\t') s++;
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') {
        uint32_t digit = (uint32_t)(*s - '0');
        if (val > (604800U / 10U)) return false;
        val = val * 10 + digit;
        if (val > 604800U) return false;
        s++;
    }
    *out = val;
    return true;
}