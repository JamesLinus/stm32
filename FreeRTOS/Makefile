###
TOOLCHAIN_DIR=/home/toolchain/stm32/gcc-arm-none-eabi-4_9-2015q3
# GNU ARM Embedded Toolchain
CC=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-gcc
LD=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-ld
AR=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-ar
AS=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-as
CP=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-objcopy
OD=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-objdump
SIZE=$(TOOLCHAIN_DIR)/bin/arm-none-eabi-size

###
# Directory Structure
BINDIR=bin
INCDIR=inc
SRCDIR=src
LIB_DIR=../lib

###
# Find source files
ASOURCES=$(shell find -L $(SRCDIR) -name '*.s')
CSOURCES+=$(shell find -L $(SRCDIR) -name '*.c')
CSRC_LIB+=$(shell find -L $(LIB_DIR)/src -name '*.c')
# Find header directories
INC=$(shell find -L . -name '*.h' -exec dirname {} \; | uniq)
INC+= $(LIB_DIR)/include
INCLUDES=$(INC:%=-I%)
# Create object list
OBJECTS+=$(CSRC_LIB:%.c=%.o)
OBJECTS+=$(ASOURCES:%.s=%.o)
OBJECTS+=$(CSOURCES:%.c=%.o)
# Define output files ELF & IHEX
BINELF=outp.elf
BINHEX=outp.hex
BIN=outp.bin

###
# MCU FLAGS
MCFLAGS=-mcpu=cortex-m4 -mthumb -mlittle-endian \
-mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork
# COMPILE FLAGS
DEFS=-DUSE_STDPERIPH_DRIVER -DSTM32F4XX
CFLAGS  =-c $(MCFLAGS) $(DEFS) $(INCLUDES)
# LINKER FLAGS
LDSCRIPT= stm32_flash.ld
LDFLAGS =-T $(LDSCRIPT) --specs=nosys.specs $(MCFLAGS) -Wl,-Map=app.map

###
# Optimizations
OPT?='O1 O2 O3 O4 O6 O7' # O5 disabled by default, because it breaks code

#ifneq ($(findstring release-memopt,$(MAKECMDGOALS)),)
ifneq ($(filter O1,$(OPT)),)
CXXFLAGS+=-fno-exceptions # Uncomment to disable exception handling
DEFS+=-DNO_EXCEPTIONS # The source code has to comply with this rule
endif

ifneq ($(filter O2,$(OPT)),)
CFLAGS+=-O2 # Optimize for size https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
CXXFLAGS+=-O2
LDFLAGS+=-O2 # Optimize for size https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
endif

ifneq ($(filter O3,$(OPT)),)
CFLAGS+=-ffunction-sections -fdata-sections # Place each function or data item into its own section in the output file
CXXFLAGS+=-ffunction-sections -fdata-sections # -||-
LDFLAGS+=-Wl,-gc-sections # Remove isolated unused sections
endif

ifneq ($(filter O4,$(OPT)),)
CFLAGS+=-fno-builtin # Disable C++ exception handling
CXXFLAGS+=-fno-builtin # Disable C++ exception handling
endif

ifneq ($(filter O5,$(OPT)),)
CFLAGS+=-flto # Enable link time optimization
CXXFLAGS+=-flto # Enable link time optimization
LDFLAGS+=-flto # Enable link time optimization
endif

ifneq ($(filter O6,$(OPT)),)
CXXFLAGS+=-fno-rtti # Disable type introspection
endif

ifneq ($(findstring O7,$(OPT)),)
LDFLAGS+=--specs=nano.specs # Use size optimized newlib
endif
#endif

###
# Build Rules
.PHONY: all release release-memopt debug clean

all: release

release: $(BINDIR)/$(BINHEX)

release-memopt: release

debug: CFLAGS+=-g
debug: LDFLAGS+=-g
debug: release

$(BINDIR)/$(BINHEX): $(BINDIR)/$(BINELF)
	@ $(CP) -O ihex $< $@ 
	@echo "Objcopy from ELF to IHEX complete"
$(BINDIR)/$(BIN): $(BINDIR)/$(BINELF)
	@ $(CP) -O binary $< $@ 
	@echo "Objcopy from B to BIN complete"	

$(BINDIR)/$(BINELF): $(OBJECTS)
	@ $(CC) $(LDFLAGS) $(OBJECTS) -o $@
	@echo "Linking complete!"
	$(SIZE) $(BINDIR)/$(BINELF)

%.o: %.c
	@ $(CC) $(CFLAGS) $< -o $@
	@echo "CC "$<""

%.o: %.s
	@ $(CC) $(CFLAGS) $< -o $@
	@echo "ASM "$<""

clean:
	@ rm -f $(OBJECTS) $(BINDIR)/$(BINELF) $(BINDIR)/$(BINHEX)

deploy:
ifeq ($(wildcard /opt/openocd/bin/openocd),)
	/usr/bin/openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg -c "program bin/"$(BINELF)" verify reset"
else
	/opt/openocd/bin/openocd -f /opt/openocd/share/openocd/scripts/board/stm32f4discovery.cfg -c "program bin/"$(BINELF)" verify reset"
endif
flash: $(BINDIR)/$(BIN)
	st-flash write $(BINDIR)/$(BIN) 0x08000000
