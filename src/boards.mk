ifeq ($(BOARD), PICO)
    include boards/mcu_pico.mk
    include sdk/pico-sdk.mk
    ELF2UF2 = $(TOOL_DIR)elf2uf2
    ARTIFACTS = $(EXE) $(UF2)
    FLASH_TOOL = $(TOOL_DIR)picotool
    FLASH_FILE = $(UF2)
    FLASH_TOOL_PARAMS = load -x $(FLASH_FILE)
endif

ifeq ($(BOARD), BLUEPILL)
    include boards/mcu_bluepill.mk
    include sdk/stm32cubef1.mk
    ARTIFACTS = $(EXE) $(BIN)
    FLASH_TOOL = $(TOOL_DIR)st-flash
    FLASH_FILE = $(BIN)
    FLASH_TOOL_PARAMS = write $(FLASH_FILE) 0x8000000
endif

ifeq ($(BOARD), BLACKPILL)
    include boards/mcu_blackpill.mk
    include sdk/stm32cubef4.mk
    ARTIFACTS = $(EXE) $(BIN)
    FLASH_TOOL = $(TOOL_DIR)st-flash
    FLASH_FILE = $(BIN)
    FLASH_TOOL_PARAMS = write $(FLASH_FILE) 0x8000000
endif

