###############################################################################
# Makefile for the project QuadCtrl
###############################################################################

## General Flags
PROJECT = QuadCtrl
MCU = atmega32
TARGET = QuadCtrl.elf
CC = avr-gcc.exe

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99       -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=QuadCtrl.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"D:\projects\avr\start\uQuad\..\..\avrlib" -I"D:\projects\avr\start\uQuad\." 

## Objects that must be built in order to link
OBJECTS = QuadCtrl.o timer.o uart.o buffer.o rprintf.o lis3lv02_spi.o spi.o ITG3200.o ctrl.o a2d.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) QuadCtrl.hex QuadCtrl.eep QuadCtrl.lss size

## Compile
QuadCtrl.o: ../QuadCtrl.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

timer.o: ../../../avrlib/timer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart.o: ../../../avrlib/uart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

buffer.o: ../../../avrlib/buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

rprintf.o: ../../../avrlib/rprintf.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

lis3lv02_spi.o: ../lis3lv02_spi.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spi.o: ../../../avrlib/spi.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ITG3200.o: ../ITG3200.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ctrl.o: ../ctrl.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

a2d.o: ../../../avrlib/a2d.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) QuadCtrl.elf dep/* QuadCtrl.hex QuadCtrl.eep QuadCtrl.lss QuadCtrl.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

