TARGETS=h8mini fq777-124
DEBUG_TARGETS = $(addsuffix _debug, $(TARGETS))

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
BASE_LDFLAGS    = --specs=nano.specs -lc_nano -lm -Wl,--no-wchar-size-warning,-T,flash.ld,-Map,$(TARGET).map,--gc-sections 
DBG_LDFLAGS     = --specs=rdimon.specs -lrdimon -u _printf_float
RELEASE_LDFLAGS = --specs=nosys.specs -flto 


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



uppercase_TABLE:=a,A b,B c,C d,D e,E f,F g,G h,H i,I j,J k,K l,L m,M n,N o,O p,P q,Q r,R s,S t,T u,U v,V w,W x,X y,Y z,Z

define uppercase_internal
$(if $1,$$(subst $(firstword $1),$(call uppercase_internal,$(wordlist
2,$(words $1),$1),$2)),$2)
endef

define uppercase
$(eval uppercase_RESULT:=$(call
uppercase_internal,$(uppercase_TABLE),$1))$(uppercase_RESULT)
endef


release: $(TARGETS)

debug: $(DEBUG_TARGETS)


all: release debug



define MAKE_TARGET


$(1).hex: $(1).elf
	$$(CP) -O ihex $$^ $$@

$(1): $(1).elf
	$$(CP) -O binary $$^ $$@

ifneq (,$(findstring debug,$(1)))
$(1).elf: CFLAGS = $$(DBG_CFLAGS) -DHARDWARE_HEADER=hardware_$(subst _debug,,$(1)).h

$(1).elf: LDFLAGS = $$(DBG_LDFLAGS)

else
$(1).elf: CFLAGS = $$(RELEASE_CFLAGS) -DHARDWARE_HEADER=hardware_$(1).h

$(1).elf: LDFLAGS = $$(RELEASE_LDFLAGS)

endif

$(1).elf: $$(SRC) $$(STARTUP) $$(HEADERS)
	$$(CC) $$(BASE_CFLAGS) $$(CFLAGS) $$^ $$(BASE_LDFLAGS) $$(LDFLAGS) -o $$@
	arm-none-eabi-size $$@

$(1)-flash:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c init -c "reset halt" -c "flash write_image erase $(1) 0x08000000" -c "verify_image $(1) 0x08000000" -c "reset run" -c shutdown

$(1)-flash-bm:
	$$(DB) $(1).elf -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

$(1)-debug:
	$$(DB) $(1).elf -ex "target remote | openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c 'gdb_port pipe; log_output $(1)-debug.log' " -ex "monitor arm semihosting enable"

$(1)-debug-bm:
	$$(DB) $(1).elf -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1"


endef

$(foreach target,$(TARGETS),$(eval $(call MAKE_TARGET,$(target))))
$(foreach target,$(DEBUG_TARGETS),$(eval $(call MAKE_TARGET,$(target))))



all: debug release

clean:
	rm -f Startup.lst $(TARGETS) $(addsuffix .lst, $(TARGETS)) $(AUTOGEN) \
		$(addsuffix .out, $(TARGETS)) $(addsuffix .hex, $(TARGETS)) $(addsuffix .map, $(TARGETS)) \
		$(addsuffix .dmp, $(TARGETS)) $(addsuffix .elf, $(TARGETS)) \
		$(addsuffix .d, $(TARGETS)) \
		$(DEBUG_TARGETS) $(addsuffix .lst, $(DEBUG_TARGETS)) \
		$(addsuffix .out, $(DEBUG_TARGETS)) $(addsuffix .hex, $(DEBUG_TARGETS)) $(addsuffix .map, $(DEBUG_TARGETS)) \
		$(addsuffix .dmp, $(DEBUG_TARGETS)) $(addsuffix .elf, $(DEBUG_TARGETS))\
		$(addsuffix .d, $(DEBUG_TARGETS))

flash-erase:
	openocd -s /usr/share/openocd/scripts -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg - debug_level 3 -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" -c "mww 0x40022010 0x2220" -c "sleep 100" -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" -c shutdown


flash-erase-bm:
	$(DB) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "monitor erase_mass" -ex "detach" -ex "quit"

