DEFCCPATH = "/opt/ti/gcc"
CCPATHFILE = ccpath.local
CCPATH ="$$(cat $(CCPATHFILE) 2>/dev/null || echo $(DEFCCPATH))"

CCINCLUDES = ${CCPATH}/include
WINE    =
CC      = $(CCPATH)/bin/msp430-elf-gcc
LINK    = $(CCPATH)/bin/msp430-elf-gcc
OBJCOPY = $(CCPATH)/bin/msp430-elf-objcopy
ECHO	= echo
OBJDIR  = objs.msp430

BUILDNUM = buildnum
FWVERSION ="hello.$$(touch $(BUILDNUM);cat $(BUILDNUM))"

LSCRIPT = iwatch-msp430f5438a.ld

CFLAGS  = -g -std=c99 --debug -mlarge -ffunction-sections -fdata-sections -fno-builtin -I${CCPATH}/include -D__MSP430F5438A__ -mmcu=MSP430F5438A -DFWVERSION=\"$(FWVERSION)\"
CFLAGS2 = -Os -MMD
LDFLAGS = ${CFLAGS} -Wl,-m,msp430X -L${CCPATH}/include/ -T${LSCRIPT} -Wl,--gc-sections -Wl,-Map=$(OBJDIR)/watch.map

ALL_DEFINES = RELEASE_VERSION AUTOSTART_ENABLE=1 HAVE_BLE=0 AMX_ANSIONLY=1 BYTE_ORDER=LITTLE_ENDIAN PAWN_CELL_SIZE=16 HAVE_ALLOCA_H=0 PRODUCT_W001=1 AMX_NATIVETABLE=window_natives

#######################################
# source files
CORE   = \
	hello.c

SRCS = $(CORE)
OBJS0 = $(SRCS:.c=.o)
OBJS = $(addprefix $(OBJDIR)/, $(OBJS0))

#####################
# rules to build the object files
$(OBJDIR)/%.o: %.c
	@$(ECHO) "Compiling $<"
	@test -d $(OBJDIR) || mkdir -pm 775 $(OBJDIR)
	@test -d $(@D) || mkdir -pm 775 $(@D)
	@-$(RM) $@
	$(CC) $< -c $(CFLAGS) $(CFLAGS2) $(ALL_DEFINES:%=-D%) $(ALL_INCLUDEDIRS:%=-I%) -o $@

# create firmware image from common objects and example source file

all: buildnum $(OBJS) $(OBJDIR)/watch.elf

$(OBJDIR)/watch.elf: ${OBJS}
	@echo "============================================================"
	@echo "Link $@"
	${LINK} $^ $(LIBS) ${LDFLAGS} -o$@
	@echo "Convert to verilog format"
	${OBJCOPY} -O verilog $@ $(OBJDIR)/watch.txt
# compatibility with Kreyos' BSL.exe
	@${ECHO} q >>$(OBJDIR)/watch.txt

buildnum: $(OBJS)
	@echo "Build Number: $$(cat buildnum)"
	@echo $$(($$(cat buildnum) + 1)) > buildnum

.SILENT:
.PHONY:	clean
clean:
	rm -Rf $(OBJDIR)/
