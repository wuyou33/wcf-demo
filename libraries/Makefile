LIB = libSTM32F4xx_StdPeriph_Driver.a

ifndef COMPILE_OPTS
 ERROR...
endif

SRC_DIR  = ./ST/STM32F4xx_StdPeriph_Driver/src

SRC_DIR += ./ST/STM32_USB_OTG_Driver/src 
SRC_DIR += ./ST/STM32_USB_Device_Library/Core/src
SRC_DIR += ./ST/STM32_USB_Device_Library/Class/cdc/src

BLD_DIR := obj

VPATH =. 
VPATH += $(SRC_DIR)

SRCS = $(foreach dir, $(SRC_DIR), $(wildcard $(dir)/*.c))
OBJS = $(patsubst %.c,$(BLD_DIR)/%.o,$(notdir $(SRCS)))



CC = $(ARMGCC_PATH)arm-none-eabi-gcc
AR = $(ARMGCC_PATH)arm-none-eabi-ar
RANLIB = $(ARMGCC_PATH)arm-none-eabi-ranlib


CFLAGS = $(COMPILE_OPTS)
CFLAGS += -I ../inc
CFLAGS += -I ST/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -I ST/CMSIS/Include
CFLAGS += -I ST/CMSIS/Device/ST/STM32F4xx/Include

CFLAGS += -I ST/STM32_USB_OTG_Driver/inc 
CFLAGS += -I ST/STM32_USB_Device_Library/Core/inc
CFLAGS += -I ST/STM32_USB_Device_Library/Class/cdc/inc
  

OBJS := $(filter-out obj/stm32f4xx_fmc.o,$(OBJS))
OBJS := $(filter-out obj/usb_hcd.o,$(OBJS))
OBJS := $(filter-out obj/usb_hcd_int.o,$(OBJS))
OBJS := $(filter-out obj/usb_otg.o,$(OBJS))

## FatFs
#CFLAGS += -I libfatfs
#CFLAGS += -I libfatfs/inc
#CFLAGS += -I libusb/host/msc/storage/ctrl_access/module_config
#CFLAGS += -I libusb/host/msc/storage/ctrl_access

PR=1
all: $(PR) $(LIB) 

$(PR):
	@echo DIRECT. $(ASFLAGS)
	@echo $(OBJS)
$(BLD_DIR)/%.o : %.c  |$(BLD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB): $(OBJS)
	$(AR) -r $(LIB)  $(OBJS) 
	$(RANLIB) $(LIB)

$(BLD_DIR):
	mkdir $@

clean:
	-rm -rf $(BLD_DIR)
	-rm -f $(LIB) 
#find ../ -type f -name *.o -delete; find ../ -type f -name *.a -delete
.PHONY: all clean