# Compile option defines:
# VISION : substitutes GPS for visual odometry
# LOG_FLT_CTRL_DEBUG_TO_SD : only perform SD card logging

# Add library files as needed only:
SOURCES   = libfatfs/src/ff.c
SOURCES  += libfatfs/src/unicode.c
SOURCES  += libstr91x/src/91x_gpio.c
SOURCES  += libstr91x/src/91x_i2c.c
SOURCES  += libstr91x/src/91x_scu.c
SOURCES  += libstr91x/src/91x_ssp.c
SOURCES  += libstr91x/src/91x_tim.c
SOURCES  += libstr91x/src/91x_uart.c
SOURCES  += libstr91x/src/91x_vic.c
SOURCES  += libstr91x/src/91x_wdg.c
SOURCES  += libstr91x/src/91x_wiu.c

TARGET := $(notdir $(shell pwd))

MCU      := arm9e

LIBRARY     := $(wildcard lib*)
LIBRARY_INC := $(addsuffix /include, $(LIBRARY))
LIBRARY_SRC := $(addsuffix /src, $(LIBRARY))

DEPFLAGS  = -MM -MT '$(addprefix $(BUILD_PATH)/, $(<:.c=.o)) $@' $< -MF $@
CFLAGS    = -c -g $(LDFLAGS)
CCFLAGS   = -std=gnu99 -Wstrict-prototypes
CPPFLAGS  = -std=c++11 -fno-exceptions
LSTFLAGS  = -Wa,-adhlns=$(addprefix $(BUILD_PATH)/,$(addsuffix .lst, $<))
LDFLAGS   = -Ofast -pedantic -Wall -Wextra -Werror -Wundef -ffreestanding \
            -Wl,--relax -Tstr911fam.ld
ALLFLAGS  = -mcpu=$(MCU) -I. $(addprefix -I, $(LIBRARY_INC)) \
            -DVISION

CC   := arm-none-eabi-gcc
CCP  := arm-none-eabi-g++
CP   := arm-none-eabi-objcopy
DUMP := arm-none-eabi-objdump
SIZE := arm-none-eabi-size
MKP  := mk-programmer

# If the environment variable DEV_BUILD_PATH is set, then the build files will
# be placed there in a named sub-folder, otherwise a build directory will be
# created in the current directory
ifneq ($(DEV_BUILD_PATH),)
  BUILD_PATH := $(DEV_BUILD_PATH)/build/$(TARGET)
else
  BUILD_PATH := build
endif

SOURCES  += $(wildcard *.S)
SOURCES  += $(wildcard *.c)
SOURCES  += $(wildcard *.cpp)
DEPENDS   = $(addsuffix .d, $(addprefix $(BUILD_PATH)/, $(SOURCES)))
OBJECTS   = $(addsuffix .o, $(addprefix $(BUILD_PATH)/, $(SOURCES)))
ASSEMBL   = $(addsuffix .lst, $(addprefix $(BUILD_PATH)/, $(SOURCES)))

ELF    := $(BUILD_PATH)/$(TARGET).elf
HEX    := $(BUILD_PATH)/$(TARGET).hex

# Rules to make dependency "makefiles"
$(BUILD_PATH)/%.c.d: %.c
	mkdir -p $(dir $@)
	$(CC) $(DEPFLAGS) $(ALLFLAGS)

$(BUILD_PATH)/%.S.d: %.S
	mkdir -p $(dir $@)
	$(CC) $(DEPFLAGS) $(ALLFLAGS)

$(BUILD_PATH)/%.cpp.d: %.cpp
	mkdir -p $(dir $@)
	$(CPP) $(DEPFLAGS) $(ALLFLAGS)

# Rules to make the compiled objects
$(BUILD_PATH)/%.c.o: %.c $(BUILD_PATH)/%.c.d
	$(CC) $(CFLAGS) $(CCFLAGS) $(LSTFLAGS) $(ALLFLAGS) -o $@ $<

$(BUILD_PATH)/%.S.o: %.S $(BUILD_PATH)/%.S.d
	$(CC) $(CFLAGS) $(CCFLAGS) $(LSTFLAGS) $(ALLFLAGS) -o $@ $<

$(BUILD_PATH)/%.cpp.o: %.cpp $(BUILD_PATH)/%.cpp.d
	$(CPP) $(CFLAGS) $(CPPFLAGS) $(LSTFLAGS) $(ALLFLAGS) -o $@ $<

# Declare targets that are not files
.PHONY: program clean


# Note that without an argument, make simply tries to build the first target
# (not rule), which in this case is this target to build the .hex
$(HEX): $(ELF)
	$(CP) -O ihex $< $@
	$(SIZE) --target=ihex $@

# Target to build the .elf file
# NOTE: -lm includes the math library (libm.a)
$(ELF): $(OBJECTS)
	$(CC) $(LDFLAGS) $(CCFLAGS) $(ALLFLAGS) -o $@ $(OBJECTS) -lm
	$(SIZE) -A $@

# Include the dependency "makefiles"
ifneq ($(MAKECMDGOALS),clean)
-include $(DEPENDS)
endif

# Target to program the board
program: $(HEX)
	$(MKP) $(HEX)

# Target to clean up the directory (leaving only source)
clean: $(addprefix $(BUILD_PATH)/, $(LIBRARY_SRC))
	rm -f $(HEX) $(ELF) $(OBJECTS) $(DEPENDS) $(ASSEMBL)
	rmdir $(addprefix $(BUILD_PATH)/, $(LIBRARY_SRC))
	rmdir $(addprefix $(BUILD_PATH)/, $(LIBRARY))
	rmdir $(BUILD_PATH)

$(addprefix $(BUILD_PATH)/, $(LIBRARY_SRC)):
	mkdir -p $@
