##########################################################################
#
# Build variables
#
##########################################################################

BUILD_NAME = bootrom
BUILD_DIR = ./Debug
BINARY_PATH = $(BUILD_DIR)/bin

##########################################################################
#
# Build system
#
##########################################################################

ifeq ($(OS), Windows_NT)
	PRESET = C:\Windows\sysnative\wsl.exe bash -ic "
	ENDSET = "
	MKDIR = if not exist $(@D) mkdir $(subst /,\,$(@D))
else
	MKDIR = @mkdir -p $(@D)
endif

##########################################################################
#
# Sources
#
##########################################################################
-include Makefile.include

##########################################################################
#
# Compiler definitions
#
##########################################################################
CROSS_COMPILE 	= riscv64-unknown-elf-
COMPILER 		= $(CROSS_COMPILE)gcc
ASSEMBLER 		= $(CROSS_COMPILE)as
LD				= $(CROSS_COMPILE)gcc
OBJCOPY			= $(CROSS_COMPILE)objcopy
OBJDUMP			= $(CROSS_COMPILE)objdump
SIZE			= $(CROSS_COMPILE)size

OBJECTS = $(patsubst %.c, $(BUILD_DIR)/%.o,$(patsubst %.S, $(BUILD_DIR)/%.o,$(SRC)))
#OBJECTS = $(patsubst %.c, %.o,$(patsubst %.S, %.o,$(SRC)))
##########################################################################
#
# Make Targets
#
##########################################################################
all: $(BINARY_PATH)/$(BUILD_NAME).elf

# Compile the elf file
# First make the directory
# Link the object files and create the .elf file
# Do the object dump, which create the list file
# Create a hex file from the .elf file
# Print out the size of the .elf file in the terminal
%.elf: $(OBJECTS)
	@echo 'Building target: $@'
	$(MKDIR)	
	$(PRESET) $(LD) $(OBJECTS) -T $(LINKER)  $(LDFLAGS) -Wl,-Map=$(BINARY_PATH)/$(BUILD_NAME).map -o $@ $(ENDSET)
	$(PRESET) $(OBJDUMP) --source --all-headers --demangle --line-numbers --wide "$(BINARY_PATH)/$(BUILD_NAME).elf" > "$(BINARY_PATH)/$(BUILD_NAME).lst" $(ENDSET)
	$(PRESET) $(OBJCOPY) -O ihex "$(BINARY_PATH)/$(BUILD_NAME).elf" "$(BINARY_PATH)/$(BUILD_NAME).hex" $(ENDSET)
	$(PRESET) $(SIZE) --format=berkeley "$(BINARY_PATH)/$(BUILD_NAME).elf" $(ENDSET)
	perl hex2verilog $(BINARY_PATH)/$(BUILD_NAME).hex $(HEX2VERILOG_DATA_SIZE) $(HEX2VERILOG_OFFSET)

# compile .c (c source) files to object files
$(BUILD_DIR)/%.o: %.c	
	@echo "Compiling Source $< into object $@"
	$(MKDIR)
	$(PRESET) $(COMPILER) $(CFLAGS) -c $< -o $@ $(ENDSET)

# compile .S (assembler) files to object files
$(BUILD_DIR)/%.o: %.S
	@echo "Compiling Source $< into object $@"
	$(MKDIR)
	$(PRESET) $(COMPILER) $(CFLAGS) -c $< -o $@ $(ENDSET)

.PHONY: clean
clean:
	$(PRESET) rm -r -f $(BUILD_DIR) all $(ENDSET)
