CROSS_COMPILE ?= aarch64-none-elf-

CC      := $(CROSS_COMPILE)gcc
LD      := $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy

CFLAGS  := -O2 -ffreestanding -nostdlib -Wall -Wextra -mcpu=cortex-a57
LDFLAGS := -T link.ld

TARGET  := vlpi_baremetal
OBJS    := start.o vlpi_baremetal.o

.PHONY: all clean run-direct run-trap

all: $(TARGET).elf $(TARGET).bin

start.o: start.S
	$(CC) $(CFLAGS) -c $< -o $@

vlpi_baremetal.o: vlpi_baremetal.c
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
	  --param "system.realview.gic.its.direct_vlpi=True"

run-trap: all
	build/ARM/gem5.opt \
	  --outdir=m5out-vlpi-baremetal-trap \
	  configs/example/arm/baremetal.py \
	  --kernel tests/gem5/arm_vlpi_tests/baremetal/$(TARGET).elf \
	  --cpu atomic --num-cores 1 --machine-type VExpress_GEM5_V2 \
	  --param "system.realview.gic.gicv4=True" \
	  --param "system.realview.gic.its.direct_vlpi=False"

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin
