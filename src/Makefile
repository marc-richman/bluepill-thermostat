OPENCM3_DIR = ../libopencm3

PROJECT     = bluepill-thermostat
BUILD_DIR   = bin
CFILES      = bluepill-thermostat.c
DEVICE      = stm32f103c8t6

# Extra flags
CSTD    = -std=gnu99         # override default -std=c99 from rules.mk

# Extra warnings on top of libopencm3 defaults
CFLAGS += -Wall -Wextra -Wshadow -Wundef \
          -Wstrict-prototypes -Wmissing-prototypes -Wredundant-decls \
          -Wpointer-arith -Wcast-align

INCLUDES += -I.
VPATH    += .

# OpenOCD
OOCD_FILE = /usr/share/openocd/scripts/board/stm32f103c8_blue_pill.cfg

# libopencm3 build machinery
include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

# (optional) make default target explicit
.DEFAULT_GOAL := all

# Optional: build a HEX as well
#$(PROJECT).hex: $(PROJECT).elf
#	$(OBJCOPY) -O ihex $< $@
