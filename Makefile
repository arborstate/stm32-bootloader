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
CFLAGS		= 	-Os -g -falign-functions=16 -ffunction-sections -fdata-sections -fno-common -flto $(MFLAGS)
ASFLAGS		=	$(MFLAGS)

LDFLAGS		=	\
			-Wl,-Map=./$(@:.elf=.map),--cref,--no-warn-mismatch,--script=./STM32F334R8Tx_FLASH.ld,--gc-sections --specs=nosys.specs

DEFINES		=	-DSTM32F334x8
INCLUDES	=	-I$(CMSISDIR)/Include -I$(CMSISDIR)/Device/ST/STM32F3xx/Include
CPPFLAGS	=	$(INCLUDES) $(DEFINES)

LIBS		=	-L/usr/lib/newlib-nano/arm-none-eabi/lib/thumb/v7e-m -lc -lgcc

BL_TARGET	=	bl.elf bl.map
BL_CFILES	=	system_stm32f3xx.c bxcan.c xmodem.c bl.c
BL_SFILES	=	startup_stm32f334x8.s
BL_SOURCE	=	$(BL_CFILES) $(BL_SFILES)
BL_OBJECTS 	=	$(BL_SFILES:.s=.o) $(BL_CFILES:.c=.o)
OBJECTS		+=	$(BL_OBJECTS)
TARGETS		+=	$(BL_TARGET) $(BL_TARGET:.elf=.bin) $(BL_TARGET:.elf=.list)


all: $(TARGETS)

$(BL_TARGET): $(BL_OBJECTS) Makefile
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(BL_OBJECTS) $(LIBS)
	$(SIZE) $@

flash:	$(BL_TARGET:.elf=.bin)
	st-flash write $< 0x08000000

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.list: %.elf
	$(OBJDUMP) -S $< > $@

clean:
	$(RM) $(TARGETS) $(OBJECTS)
