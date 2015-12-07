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
SRC_DIR=.
OS_DIR=../opensource/uCOS/uCOS-III
OS_CPU_DIR=../opensource/uCOS/uC-CPU
OS_LIB_DIR=../opensource/uCOS/uC-LIB
OS_SERIAL_DIR=../opensource/uCOS/uC-Serial
STM32F4_HAL_DIR=../opensource/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR=../opensource/STM32CubeF4/Drivers/CMSIS
EXTRA_LIB_DIR=../extralib
###
# Find source files
ASOURCES+=$(shell find -L $(SRC_DIR) -name '*.s')
ASOURCES+=$(shell find -L $(OS_DIR) -name '*.s')
ASOURCES+=$(shell find -L $(OS_CPU_DIR) -name '*.s')
ASOURCES+=$(shell find -L $(OS_LIB_DIR) -name '*.s')
ASOURCES+=$(shell find -L $(OS_SERIAL_DIR) -name '*.s')
ASOURCES+=$(shell find -L $(CMSIS_DIR) -name '*.s')


CSOURCES+=$(shell find -L $(SRC_DIR) -name '*.c')
CSOURCES+=$(shell find -L $(OS_DIR) -name '*.c')
CSOURCES+=$(shell find -L $(OS_CPU_DIR) -name '*.c')
CSOURCES+=$(shell find -L $(OS_LIB_DIR) -name '*.c')
CSOURCES+=$(shell find -L $(OS_SERIAL_DIR) -name '*.c')
CSOURCES+=$(shell find -L $(STM32F4_HAL_DIR) -name '*.c')
CSOURCES+=$(shell find -L $(CMSIS_DIR) -name '*.c')
# Find header directories
INC=$(shell find -L $(SRC_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(OS_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(OS_CPU_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(OS_LIB_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(OS_SERIAL_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(STM32F4_HAL_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(CMSIS_DIR) -name '*.h' -exec dirname {} \; | uniq)
INC+=$(shell find -L $(EXTRA_LIB_DIR) -name '*.h' -exec dirname {} \; | uniq)

CSOURCES+=$(EXTRA_LIB_DIR)/src/drv/drv_core.c
CSOURCES+=$(EXTRA_LIB_DIR)/src/drv/drv_gpio.c
#INC+= $(DRIVER_DIR)/include
INCLUDES=$(INC:%=-I%)
# Create object list
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
DEFS=-DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DSTM32F407xx -DHSE_VALUE=8000000 -DSTM32CUBEF4 -DOS_UCOS
CFLAGS  =-c $(MCFLAGS) $(DEFS) $(INCLUDES)
# LINKER FLAGS
LDSCRIPT= stm32f4_flash.ld
LDFLAGS =-T $(LDSCRIPT) --specs=nosys.specs $(MCFLAGS) -Wl,-Map=bin/app.map

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

release: $(BINDIR)/$(BINHEX) $(BINDIR)/$(BIN)

release-memopt: release

debug: CFLAGS+=-g
debug: LDFLAGS+=-g
debug: release

$(BINDIR)/$(BINHEX): $(BINDIR)/$(BINELF)
	@ $(CP) -O ihex $< $@ 
	@echo "Objcopy from ELF to IHEX complete"
$(BINDIR)/$(BIN): $(BINDIR)/$(BINELF)
	@ $(CP) -O binary $< $@ 
	@ $(OD) -S $(BINDIR)/$(BINELF) > $(BINDIR)/$(BINELF).lst
	@echo "Objcopy from ELF to BIN complete"	

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
	@ rm -f $(OBJECTS) $(BINDIR)/*

deploy:
ifeq ($(wildcard /opt/openocd/bin/openocd),)
	/usr/bin/openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg -c "program bin/"$(BINELF)" verify reset"
else
	/opt/openocd/bin/openocd -f /opt/openocd/share/openocd/scripts/board/stm32f4discovery.cfg -c "program bin/"$(BINELF)" verify reset"
endif
flash: $(BINDIR)/$(BIN)
	st-flash --reset write $(BINDIR)/$(BIN) 0x08000000
