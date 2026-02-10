#ifndef USB_CONSOLE_H
#define USB_CONSOLE_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize USB CDC console */
void usb_console_init(void);

/* Poll/flush; call often from main loop */
void usb_console_poll(void);

/* Non-blocking write; returns number of bytes queued */
int usb_console_write(const char *s);

/* Register command handler callback */
void usb_console_set_cmd_handler(void (*handler)(const char *cmd));

/* Query configured state */
bool usb_console_configured(void);

#endif /* USB_CONSOLE_H */