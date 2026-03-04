CROSS_COMPILE ?= aarch64-none-elf-

CC      = $(CROSS_COMPILE)gcc
LD      = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

# Use generic ARMv8-A flags to avoid host-specific -mcpu parsing issues.
CFLAGS  := -O2 -ffreestanding -nostdlib -Wall -Wextra -march=armv8-a
LDFLAGS := -T link.ld

TARGET  := vlpi_baremetal
OBJS    := start.o vlpi_baremetal.o

CC_MACHINE := $(shell $(CC) -dumpmachine 2>/dev/null)

.PHONY: all clean run-direct run-trap check-toolchain

all: check-toolchain $(TARGET).elf $(TARGET).bin

check-toolchain:
	@if [ -z "$(CC_MACHINE)" ]; then \
		echo "ERROR: compiler '$(CC)' not found."; \
		echo "Hint: install AArch64 cross toolchain and run:"; \
		echo "  make CROSS_COMPILE=aarch64-none-elf-"; \
		echo "or"; \
		echo "  make CROSS_COMPILE=aarch64-linux-gnu-"; \
		exit 1; \
	fi
	@if ! echo "$(CC_MACHINE)" | grep -q 'aarch64'; then \
		echo "ERROR: CC='$(CC)' targets '$(CC_MACHINE)', not AArch64."; \
		echo "Please use an AArch64 cross compiler, e.g.:"; \
		echo "  make CROSS_COMPILE=aarch64-none-elf-"; \
		echo "or"; \
		echo "  make CROSS_COMPILE=aarch64-linux-gnu-"; \
		exit 1; \
	fi

start.o: start.S check-toolchain
	$(CC) $(CFLAGS) -c $< -o $@

vlpi_baremetal.o: vlpi_baremetal.c check-toolchain
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS) link.ld
	$(LD) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

run-direct: all
	build/ARM/gem5.opt \
	  --outdir=m5out-vlpi-baremetal-direct \
	  configs/example/arm/baremetal.py \
	  --kernel tests/gem5/arm_vlpi_tests/baremetal/$(TARGET).elf \
	  --cpu atomic --num-cores 1 --machine-type VExpress_GEM5_V2 \
	  --param "system.realview.gic.gicv4=True" \
	  --param "system.realview.gic.its.direct_vlpi=True" \
	  --exit-on-uart-eot

run-trap: all
	build/ARM/gem5.opt \
	  --outdir=m5out-vlpi-baremetal-trap \
	  configs/example/arm/baremetal.py \
	  --kernel tests/gem5/arm_vlpi_tests/baremetal/$(TARGET).elf \
	  --cpu atomic --num-cores 1 --machine-type VExpress_GEM5_V2 \
	  --param "system.realview.gic.gicv4=True" \
	  --param "system.realview.gic.its.direct_vlpi=False" \
	  --exit-on-uart-eot

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin
