# 
# Copyright (c) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
# Copyright (c) 2017 Texas Instruments - Jason Kridner <jdk@ti.com>
#
PROJ_NAME=detector

# PRU_CGT environment variable must point to the TI PRU compiler directory.
# PRU_SUPPORT points to pru-software-support-package
# Both are set in setup.sh
PRU_CGT:=/usr/share/ti/cgt-pru
PRU_SUPPORT:=/usr/lib/ti/pru-software-support-package

LINKER_COMMAND_FILE=./AM335x_PRU.cmd
LIBS=--library=$(PRU_SUPPORT)/lib/rpmsg_lib.lib
INCLUDE=--include_path=$(PRU_SUPPORT)/include --include_path=$(PRU_SUPPORT)/include/am335x
STACK_SIZE=0x100
HEAP_SIZE=0x100

CFLAGS=-v3 -O2 --display_error_number --endian=little --hardware_mac=on --obj_directory=$(GEN_DIR) --pp_directory=$(GEN_DIR) -ppd -ppa --asm_listing --c_src_interlist # --absolute_listing
LFLAGS=--reread_libs --warn_sections --stack_size=$(STACK_SIZE) --heap_size=$(HEAP_SIZE) -m $(GEN_DIR)/file.map

GEN_DIR=gen

PRU1_FW		=$(GEN_DIR)/$(PROJ_NAME).out

# -----------------------------------------------------
# Variable to edit in the makefile

# add the required firmwares to TARGETS
TARGETS		=$(PRU1_FW) $(GEN_DIR)/decay95.out

#------------------------------------------------------

.PHONY: all
all: $(TARGETS)
	@echo '-	Generated firmwares are : $^'

$(GEN_DIR)/decay95.out:  $(GEN_DIR)/decay95.obj
	@echo 'LD	$^' 
	@lnkpru -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^

$(GEN_DIR)/decay95.obj: $(PROJ_NAME).c 
	@mkdir -p $(GEN_DIR)
	@echo 'CC	$<'
	@clpru --define=DECAY_RATE=95 --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

$(PRU1_FW): $(GEN_DIR)/$(PROJ_NAME).obj
	@echo 'LD	$^' 
	@lnkpru -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^

$(GEN_DIR)/$(PROJ_NAME).obj: $(PROJ_NAME).c 
	@mkdir -p $(GEN_DIR)
	@echo 'CC	$<'
	@clpru --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

.PHONY: install run run95

install:
	@echo '-	copying firmware file $(PRU1_FW) to /lib/firmware/am335x-pru1-fw'
	@cp $(PRU1_FW) /lib/firmware/am335x-pru1-fw

run: install
	@echo '-	rebooting pru core 1'
	$(shell echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind 2> /dev/null)
	$(shell echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind)
	@echo "-	pru core 1 is now loaded with $(PRU1_FW)"

run95:
	@echo '-	copying firmware file $(GEN_DIR)/decay95.out to /lib/firmware/am335x-pru1-fw'
	@cp $(GEN_DIR)/decay95.out /lib/firmware/am335x-pru1-fw
	@echo '-	rebooting pru core 1'
	$(shell echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind 2> /dev/null)
	$(shell echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind)
	@echo "-	pru core 1 is now loaded with $(GEN_DIR)/decay95.out"

.PHONY: clean
clean:
	@echo 'CLEAN	.'
	@rm -rf $(GEN_DIR) $(PROJ_NAME).asm
