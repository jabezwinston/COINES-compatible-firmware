BOARD ?= PICO

PROJ_NAME = COINES_FW_$(BOARD)

EXE =  $(PROJ_NAME).elf
BIN =  $(PROJ_NAME).bin
HEX =  $(PROJ_NAME).hex
UF2 =  $(PROJ_NAME).uf2

C_SRCS += main.c
C_SRCS += coines_protocol/coines_protocol_parser.c

INCLUDE_PATHS += coines_protocol

include boards.mk

ASM_SRCS      += $(addprefix boards/,$(BRD_ASM_SRCS))
C_SRCS        += $(addprefix boards/,$(BRD_C_SRCS))
INCLUDE_PATHS += $(addprefix boards/,$(BRD_INCLUDE_PATHS))

ASM_SRCS      += $(addprefix sdk/,$(SDK_ASM_SRCS))
C_SRCS        += $(addprefix sdk/,$(SDK_C_SRCS))
INCLUDE_PATHS += $(addprefix sdk/,$(SDK_INCLUDE_PATHS))
LD_SCRIPT      = $(addprefix sdk/,$(SDK_LD_SCRIPT))

CFLAGS  += -D $(BOARD) -D PC
LDFLAGS += -T $(LD_SCRIPT) -Wl,-Map="$(OBJ_DIR)/coines_fw.map"

######################################################################

OBJ_DIR = _build/$(BOARD)

ifeq ($(OS),Windows_NT)
    IS_CC_FOUND = $(shell where $(CC))
    $(info Platform: Windows)
    ifeq ($(notdir $(MAKE)),mingw32-make)
        SHELL = cmd
        CP  = copy
        RM  = del /s /q
        MKDIR = mkdir
        syspath = $(subst /,\,$(1))
    else
        CP = cp
        RM = rm -rf
        MKDIR = mkdir -p
        syspath = $(subst /,/,$(1))
    endif
    TOOL_DIR = ../tools/win32/
else
    IS_CC_FOUND = $(shell which $(CC))
    $(info Platform: Linux / macOS)
    RM = rm -rf
    MKDIR = mkdir -p
    syspath = $(subst /,/,$(1))
    TOOL_DIR = 
endif

ASM_FILES = $(notdir $(ASM_SRCS))
ASM_OBJS += $(addprefix $(OBJ_DIR)/, $(ASM_FILES:.S=.S.o))
ASM_PATHS = $(sort $(dir $(ASM_SRCS)))
vpath %.S $(ASM_PATHS)

C_FILES = $(notdir $(C_SRCS))
C_OBJS += $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.c.o))
C_PATHS = $(sort $(dir $(C_SRCS)))
DEP += $(C_OBJS:%.o=%.d)
vpath %.c $(C_PATHS)

####################################################################
# Make BOARDs                                                     #
####################################################################
all: $(ARTIFACTS)

$(OBJ_DIR):
	@echo [ MKDIR ] $@
	@$(MKDIR) $(call syspath,$@)

$(BIN): $(EXE)
	@echo [ BIN ] $@
	@$(OBJCOPY) -O binary $< $@

$(HEX): $(EXE)
	@echo [ HEX ] $@
	@$(OBJCOPY) -O ihex $< $@

$(UF2): $(EXE)
	@echo [ UF2 ] $@
	@$(ELF2UF2) $< $@

$(EXE): $(OBJ_DIR) $(C_OBJS) $(ASM_OBJS)
	@echo [ LD ] $@
	@$(CC) $(LDFLAGS) -o "$@" $(C_OBJS) $(ASM_OBJS) $(addprefix -L,$(LIBPATHS)) $(addprefix -l,$(LIBS))

-include $(DEP)

$(OBJ_DIR)/%.S.o: %.S
	@echo [ AS ] $<
	@$(CC) $(ASMFLAGS) -MMD $(addprefix -I,$(INCLUDE_PATHS)) -o "$@" "$<"

$(OBJ_DIR)/%.c.o: %.c
	@echo [ CC ] $<
	@$(CC) $(CFLAGS) -MMD $(addprefix -I,$(INCLUDE_PATHS)) -o "$@" "$<"

flash: $(FLASH_FILE)
	@echo [ FLASH ] $<
	@$(FLASH_TOOL) $(FLASH_TOOL_PARAMS)

clean:
	@echo "Cleaning..."
	@$(RM) $(ARTIFACTS) $(call syspath,$(OBJ_DIR))
