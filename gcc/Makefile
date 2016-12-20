TARGET=h8mini
EXECUTABLE=h8mini.elf

DBG_TARGET=h8mini_debug
DBG_EXECUTABLE=h8mini_debug.elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
DB=arm-none-eabi-gdb


DEFS = -DUSE_STDPERIPH_DRIVER
STARTUP = startup.s

MCU = cortex-m3
topdir = ..
INCLUDES = -I$(topdir)/H8mini-test/src/ \
	-I$(topdir)/H8mini-acro/src/ \
	-I$(topdir)/CMSIS/ \
	-I$(topdir)/Peripherals/inc/

BASE_CFLAGS    = -mcpu=$(MCU) -nostartfiles -mthumb -mlittle-endian -mno-thumb-interwork -ffast-math -fdata-sections -ffunction-sections -MD $(DEFS) $(INCLUDES) -std=gnu99
DBG_CFLAGS     = -Og -g3 -DDEBUG
RELEASE_CFLAGS = -Os -flto

AFLAGS = $(MCFLAGS)
BASE_LDFLAGS    = --specs=nano.specs --specs=nosys.specs -lc_nano -lm -Wl,--no-wchar-size-warning,-T,flash.ld,-Map,$(TARGET).map,--gc-sections 
DBG_LDFLAGS     = --specs=rdimon.specs -lrdimon -u _printf_float
RELEASE_LDFLAGS = -flto -lnosys


SRC = $(wildcard $(topdir)/H8mini_*/src/*.c) \
	$(wildcard $(topdir)/H8mini_*/src/config.h) \
	$(wildcard $(topdir)/H8mini_*/src/*.cpp) \
	$(wildcard $(topdir)/CMSIS/*.c) \
	$(wildcard $(topdir)/Peripherals/src/*.c)
HEADERS = $(wildcard $(topdir)/H8mini_*/src/*.h)

OBJDIR = .
OBJ = $(patsubst %.c,$(OBJDIR)/%.o,$(filter %.c,$(SRC)))
OBJ += $(patsubst %.cpp,$(OBJDIR)/%.o,$(filter %.cpp,$(SRC)))
OBJ += Startup.o



release: CFLAGS = $(RELEASE_CFLAGS)

release: LDFLAGS = $(RELEASE_LDFLAGS)

release: $(TARGET)


debug: CFLAGS = $(DBG_CFLAGS)

debug: LDFLAGS = $(DBG_LDFLAGS)

debug: $(DBG_TARGET)


all: debug release


$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@


$(EXECUTABLE): $(SRC) $(STARTUP) $(HEADERS)
	$(CC) $(BASE_CFLAGS) $(CFLAGS) $^ $(BASE_LDFLAGS) $(LDFLAGS) -o $@
	arm-none-eabi-size $(EXECUTABLE)

$(DBG_TARGET).hex: $(DBG_EXECUTABLE)
	$(CP) -O ihex $^ $@

$(DBG_TARGET): $(DBG_EXECUTABLE)
	$(CP) -O binary $^ $@


$(DBG_EXECUTABLE): $(SRC) $(STARTUP) $(HEADERS)
	$(CC) $(BASE_CFLAGS) $(CFLAGS) $^ $(BASE_LDFLAGS) $(LDFLAGS) -o $@
	arm-none-eabi-size $(DBG_EXECUTABLE)

clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(AUTOGEN) \
		$(TARGET).out $(TARGET).hex  $(TARGET).map \
		$(TARGET).dmp $(EXECUTABLE) \
		$(DBG_TARGET) $(DBG_TARGET).lst \
		$(DBG_TARGET).out $(DBG_TARGET).hex  $(DBG_TARGET).map \
		$(DBG_TARGET).dmp $(DBG_EXECUTABLE)

flash:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c init -c "reset halt" -c "flash write_image erase h8mini 0x08000000" -c "verify_image h8mini 0x08000000" -c "reset run" -c shutdown

flash-erase-bm:
	$(DB) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "monitor erase_mass" -ex "detach" -ex "quit"

flash-bm:
	$(DB) $(EXECUTABLE) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

flash-debug-bm:
	$(DB) $(DBG_EXECUTABLE) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

debug-bm:
	$(DB) $(DBG_EXECUTABLE) -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1"

