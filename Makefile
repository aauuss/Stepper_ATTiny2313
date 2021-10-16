
# Project name.
# Main C file without extension, sample for 'project1.c': 'project1'.
PROJECT = Stepper_ATTiny2313

# Chip type for AVR GCC. 
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
GCC_MCU = attiny2313
CLOCK_HZ   = 1000000

# AVRDUDE settings.
# https://ph0en1x.net/77-avrdude-full-howto-samples-options-gui-linux.html
AVRDUDE_MCU             = t2313
AVRDUDE_PROGRAMMER      = avrisp
AVRDUDE_PROGRAMMER_PORT = /dev/ttyUSB0
AVRDUDE_PROGRAMMER_BITRATE = 19200

# Fuses
FUSE_L = 0x7F
FUSE_H = 0xDF
FUSE_E = 0xFF

# List of additional C files for compilation.
C_FILES = TM1637_lib.c EEPROM_lib.c


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
CFLAGS        = -Os -mmcu=$(GCC_MCU) 
FUSES         = -U lfuse:w:$(FUSE_L):m -U hfuse:w:$(FUSE_H):m -U efuse:w:$(FUSE_E):m
FLASH         = -U flash:w:$(PROJECT).hex

AVR_GCC       = `which avr-gcc`
AVR_OBJCOPY   = `which avr-objcopy`
AVR_SIZE      = `which avr-size`
AVR_OBJDUMP   = `which avr-objdump`
AVRDUDE       = `which avrdude`
REMOVE        = `which rm`
NANO          = `which nano`
TAR           = `which tar`
DATETIME      = `date +"%d-%m-%Y"`

AVRDUDE_CMD   = $(AVRDUDE) -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROGRAMMER) -P $(AVRDUDE_PROGRAMMER_PORT) -b $(AVRDUDE_PROGRAMMER_BITRATE)

%.elf: %.c
	$(AVR_GCC) $(CFLAGS) $< $(C_FILES) -o $@

%.hex: %.elf
	$(AVR_OBJCOPY) -R .eeprom -O ihex $< $@

all: clean hex program

program: $(PROJECT).hex
	$(AVRDUDE_CMD) $(FLASH)

fuses:
	$(AVRDUDE_CMD) $(FUSES)

elf: $(PROJECT).elf

hex: $(PROJECT).hex

size: $(PROJECT).elf
	$(AVR_SIZE) $(PROJECT).elf

disasm: $(PROJECT).elf
	$(AVR_OBJDUMP) -d $(PROJECT).elf

clean:
	$(REMOVE) -f *.hex *.elf *.o

edit:
	$(NANO) $(PROJECT).c

tar:
	$(TAR) -zcf $(PROJECT)_$(DATETIME).tgz ./* 

