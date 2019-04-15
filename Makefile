-include local.mk

ifeq ($(CMSISDIR),)
$(error CMSISDIR must be defined.)
endif

CROSS		=	arm-none-eabi-
CC	=		$(CROSS)gcc
LD		=	$(CROSS)ld
OBJCOPY		=	$(CROSS)objcopy
OBJDUMP		=	$(CROSS)objdump
AS		=	$(CROSS)as
SIZE		=	$(CROSS)size
MFLAGS		=	-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS		= 	-Os -ggdb -falign-functions=16 -ffunction-sections -fdata-sections -fno-common -flto $(MFLAGS)
ASFLAGS		=	$(MFLAGS)

LDFLAGS		=	\
			-Wl,-Map=./$(@:.elf=.map),--cref,--no-warn-mismatch,--script=./STM32F334R8Tx_FLASH.ld,--gc-sections,--defsym=__process_stack_size__=0x400,--defsym=__main_stack_size__=0x400 --specs=nosys.specs

DEFINES		=	-DSTM32F334x8
INCLUDES	=	-I$(CMSISDIR)/Include -I$(CMSISDIR)/Device/ST/STM32F3xx/Include
CPPFLAGS	=	$(INCLUDES) $(DEFINES)

LIBS		=	-L/usr/lib/newlib-nano/arm-none-eabi/lib/thumb/v7e-m -lc -lgcc

BL_TARGET	=	bl.elf bl.map
BL_CFILES	=	main.c system_stm32f3xx.c
BL_SFILES	=	startup_stm32f334x8.s
BL_SOURCE	=	$(BL_CFILES) $(BL_SFILES)
BL_OBJECTS 	=	$(BL_CFILES:.c=.o) $(BL_SFILES:.s=.o)
OBJECTS		+=	$(BL_OBJECTS)
TARGETS		+=	$(BL_TARGET) $(BL_TARGET:.elf=.bin)


all: $(TARGETS)

$(BL_TARGET): $(BL_OBJECTS) Makefile
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(BL_OBJECTS) $(LIBS)
	$(SIZE) $@

flash:	$(BL_TARGET:.elf=.bin)
	st-flash write $< 0x08000000

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

clean:
	$(RM) $(TARGETS) $(OBJECTS)
