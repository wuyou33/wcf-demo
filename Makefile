ARMGCC_PATH = /usr/local/gcc-arm-none-eabi-6_2-2016q4/bin/


LinkerFile = STM32F407xGx_FLASH.ld

#arm-none-eabi-gcc --print-multi-lib
#armv7e-m/fpu/fpv5-sp-d16;@mthumb@march=armv7e-m@mfloat-abi=hard@mfpu=fpv5-sp-d16

COMPILE_OPTS= -std=gnu11 -g3  -O3  -fsingle-precision-constant   -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv5-sp-d16
COMPILE_OPTS+=-mlittle-endian 
COMPILE_OPTS+=-ffreestanding -fdata-sections -ffunction-sections  # -nodefaultlibs -nostartfiles -nostdlib we use __libc_init_array in startupxxx.s

# ST specific definitions..
COMPILE_OPTS+=-DSTM32F40_41xxx
COMPILE_OPTS+=-DUSE_STDPERIPH_DRIVER
COMPILE_OPTS+=-DUSE_USB_OTG_FS
COMPILE_OPTS+=-DHSE_VALUE=12000000
COMPILE_OPTS+=-DARM_MATH_CM4=1
COMPILE_OPTS+=-D__FPU_PRESENT=1

export ARMGCC_PATH
export COMPILE_OPTS

OUTPUT = main
BIN_DIR = bin
SRC_DIR = ./src
BLD_DIR = obj

INC_DIR = inc
INC_DIR += libraries/ST/STM32F4xx_StdPeriph_Driver/inc

INC_DIR += libraries/ST/STM32_USB_OTG_Driver/inc
INC_DIR += libraries/ST/STM32_USB_Device_Library/Core/inc
INC_DIR += libraries/ST/STM32_USB_Device_Library/Class/cdc/inc

INC_DIR += libraries/ST/CMSIS/Include
INC_DIR += libraries/ST/CMSIS/Device/ST/STM32F4xx/Include 

VPATH = $(SRC_DIR)



SRCS = $(foreach dir, $(SRC_DIR), $(wildcard $(dir)/*.c))
OBJS = $(patsubst %.c,$(BLD_DIR)/%.o,$(notdir $(SRCS)))
OBJS += $(BLD_DIR)/startup_stm32f40xx.o




LIBS = -L. -Lobj -L libraries   -lSTM32F4xx_StdPeriph_Driver
LIBS += -lm -lc   
LIBS += -lgcc -lg -lnosys 
#-lnosys

INCS = $(foreach dir, $(INC_DIR),-I$(dir))

MAIN_OUT = main
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_BIN = $(MAIN_OUT).bin
MAIN_OUT_LST = $(MAIN_OUT).lst 

CC = $(ARMGCC_PATH)arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS) $(INCS)  
AS = $(ARMGCC_PATH)arm-none-eabi-gcc
ASFLAGS = -g0 -Wall -mcpu=cortex-m4 -mthumb -c 
LD = $(ARMGCC_PATH)arm-none-eabi-gcc
LDFLAGS = $(CFLAGS) -u _scanf_float -u _printf_float  -Wl,-T,"$(LinkerFile)" -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align  

  
OBJCP = $(ARMGCC_PATH)arm-none-eabi-objcopy
OBJCPFLAGS = -O binary 
OBJDUMP = $(ARMGCC_PATH)arm-none-eabi-objdump
SIZE = $(ARMGCC_PATH)arm-none-eabi-size 

#STLINK=1
OPENOCD=1
#TELNET=1
COMMANDS = -c init -c targets -c "reset halt" -c "flash write_image erase $(OUTPUT).elf" -c "verify_image $(OUTPUT).elf" -c "reset run" -c shutdown

 

OUTPUT := $(BIN_DIR)/$(OUTPUT)

all: $(BIN_DIR) $(OUTPUT).elf
	$(OBJCP) $(OBJCPFLAGS) $(OUTPUT).elf  $(OUTPUT).bin
	$(OBJDUMP) -h -S -C -r $(OUTPUT).elf > $(OUTPUT).lst
	$(SIZE) $(OUTPUT).elf
		
$(OUTPUT).elf: $(OBJS)
	$(MAKE) -C libraries
	$(CC) $(LDFLAGS) -Wl,-Map=$@.map $^ -o $@   $(LIBS)


$(BLD_DIR)/%.o : %.c  |$(BLD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@
$(BLD_DIR)/%.o : %.s  |$(BLD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BLD_DIR):
	mkdir $@
$(BIN_DIR):
	mkdir $@


#$(CC) $(CFLAGS) -Wl,-Map=$(OUTPUT).map  -o $(OUTPUT).elf   $(LIBS)
#$(CC) $(LDFLAGS) $(LD_OPTIONAL) $(OUTPUT).map -o $(OUTPUT).elf 
# $(LIBS)	

# flash

flash: all
ifdef OPENOCD
	#openocd -f stm32f4discoveryRESETNONE.cfg  $(COMMANDS)
	openocd -f ocd_flash.cfg  $(COMMANDS)
endif
ifdef TELNET 
	(echo "reset halt; flash write_image erase $(OUTPUT).elf; verify_image $(OUTPUT).elf; reset run; exit")|nc localhost 4444
endif


clean:
	-rm -rf $(BLD_DIR) $(BIN_DIR)

cleanall:
	$(MAKE) -C libraries clean
	-rm -rf $(BLD_DIR) $(BIN_DIR)
	

