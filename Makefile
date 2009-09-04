# orb makefile <h.zeller@acm.org>
##

USBTINY=../usbtiny/usbtiny

TARGET_ARCH = -mmcu=attiny44
AVRDUDE     = avrdude -p t44 -c avrusb500
FLASH_CMD   = $(AVRDUDE) -e -U flash:w:main.hex
#STACK       = 42
STACK       = 128
FLASH       = 4096
SRAM        = 256
OBJECTS     = orb.o

CFLAGS=-O -mmcu=$(TARGET_ARCH) -Wall -Wstrict-prototypes -O3 -mcall-prologues -I. -I$(USBTINY)

include $(USBTINY)/common.mk

ifdef USBTINY_SERIAL
CFLAGS+=-DUSBTINY_SERIAL=\"$(USBTINY_SERIAL)\"
endif

# (attiny 2313 page 163ff)
### Fuse high byte: 0xDB
# 7 dwen	1    debug wire: yes.
# 6 eesave	1    save eeprom on chip erase: disabled.
# 5 spien	0    serial programming: enabled.
# 4 wdton	1    watchdog timer on: disabled.
#
# 3 bodlevel2	1\
# 2 bodlevel1	0 +  brown out detection 2.7 Volt (page 38)
# 1 bodlevel0	1/
# 0 rstdisbl	1   disable external reset: disabled (i.e.: reset enabled).
#
### Fuse low byte: 0xEF
# 7 ckdiv8	1   divide by 8: no
# 6 ckout	1   clk out: disabled.
# 5 sut1	1-+ crystal oscillator, fast rising power.
# 4 sut0	0/
#
# 3 cksel3	1\
# 2 cksel2	1 + crystal >=8 Mhz  (page 26)
# 1 cksel1	1/
# 0 cksel0	1   chrystal with SUT 10 -> crystal oscillator, fast rising power.
fuse_2313:
	$(AVRDUDE) -U hfuse:w:0xdb:m -U lfuse:w:0xef:m

### Fuse high byte: 0xDD
# 7 rstdisbl    1    disable external reset: disabled (i.e.: reset enabled).
# 6 dwen	1    debug wire: yes.
# - spien	0    serial programming: enabled.
# 4 wdton	1    watchdog timer on: disabled.
#
# 3 eesave	1    save eeprom on chip erase: disabled.
# 2 bodlevel2	1\
# 1 bodlevel1	0 +  brown out detection 2.7 Volt (page 38)
# 0 bodlevel0	1/
#
### Fuse low byte: 0xCE
# 7 ckdiv8	1   divide by 8: no
# 6 ckout	1   clk out: disabled.
# 5 sut1	0-+ ceramic oscillator, slow rising power (page 28)
# 4 sut0	0/
#
# 3 cksel3	1\
# 2 cksel2	1 + crystal >=8 Mhz  (page 26)
# 1 cksel1	1/
# 0 cksel0	0 ceramic osc SUT 10 -> crystal oscillator, fast rising power.
fuse_44:
	$(AVRDUDE) -U hfuse:w:0xdd:m -U lfuse:w:0xce:m
