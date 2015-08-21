TARGET := fatfs

MCU      := arm9e

DEPFLAGS  = -MM -MT '$(addprefix $(BUILD_PATH)/, $(<:.c=.o)) $@' $< -MF $@
CFLAGS    = -c -g $(LDFLAGS)
CCFLAGS   = -std=gnu99 -Wstrict-prototypes
CPPFLAGS  = -std=c++11 -fno-exceptions
LSTFLAGS  = -Wa,-adhlns=$(addprefix $(BUILD_PATH)/,$(addsuffix .lst, $<))
LDFLAGS   = -Ofast -pedantic -Wall -Wextra -Werror -Wundef -ffreestanding \
            -Wl,--relax -Tstr911fam.ld
ALLFLAGS  = -mcpu=$(MCU)

AR   := arm-none-eabi-ar
CC   := arm-none-eabi-gcc
CCP  := arm-none-eabi-g++

# If the environment variable DEV_BUILD_PATH is set, then the build files will
# be placed there in a named sub-folder, otherwise a build directory will be
# created in the current directory
ifneq ($(DEV_BUILD_PATH),)
  BUILD_PATH := $(DEV_BUILD_PATH)/build/$(TARGET)
else
  BUILD_PATH := build
endif
INSTALL_PATH ?= /opt/arm/arm-none-eabi

HEADERS   = $(wildcard *.h)
HEADERS  += $(wildcard *.hpp)
SOURCES   = $(wildcard *.S)
SOURCES  += $(wildcard *.c)
SOURCES  += $(wildcard *.cpp)
DEPENDS   = $(addsuffix .d, $(addprefix $(BUILD_PATH)/, $(SOURCES)))
OBJECTS   = $(addsuffix .o, $(addprefix $(BUILD_PATH)/, $(SOURCES)))
ASSEMBL   = $(addsuffix .lst, $(addprefix $(BUILD_PATH)/, $(SOURCES)))
ARCHIVE   = lib$(TARGET).a

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
# (not rule), which in this case is this target to build the .a file
$(BUILD_PATH)/$(ARCHIVE): $(OBJECTS)
	$(AR) cr $@ $<

install: $(INSTALL_PATH)
	mkdir -p $(INSTALL_PATH)/include/$(TARGET)
	cp $(HEADERS) $(INSTALL_PATH)/include/$(TARGET)/.
	cp $(BUILD_PATH)/$(ARCHIVE) $(INSTALL_PATH)/lib/.

# Include the dependency "makefiles"
ifneq ($(MAKECMDGOALS),clean)
-include $(DEPENDS)
endif

# Target to clean up the directory (leaving only source)
clean: $(BUILD_PATH)
	rm -f $(OBJECTS) $(DEPENDS) $(ASSEMBL) $(BUILD_PATH)/$(ARCHIVE)
	rmdir $(BUILD_PATH)

uninstall:
	rm -rf $(INSTALL_PATH)/inlcude/$(TARGET)
	rm $(INSTALL_PATH)/lib/$(ARCHIVE)

$(BUILD_PATH):
	mkdir -p $(BUILD_PATH)
