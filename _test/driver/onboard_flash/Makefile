################################################################
#
# onboard_flash unit tests (based on gcc)
#
################################################################


################################################################
# target
################################################################
TARGET = onboard_flash

################################################################
# build variables
################################################################

# Debug build?
DEBUG = 0
# Optimization
OPT   = -Og


################################################################
# paths
################################################################

# build folder
BUILD_DIR = build

# base directory (this Makefile lives in driver/_test/driver/onboard_flash)
ROOT_DIR  = ../../../../
DRIVER_DIR = ../../../

################################################################
# source
################################################################

# C sources (test, mocks, stubs)
TEST_SOURCES =                                              \
test_onboard_flash.c                                        \
mock/test_onboard_flash_stubs.c

# Tested source files (coverage targets)
COV_C_SOURCES =                                             \
$(DRIVER_DIR)/onboard_flash/onboard_flash.c

# Framework (do not modify)
FRAMEWORK_SOURCES =                                         \
$(DRIVER_DIR)/_test/framework/src/test_assert.c                  \
$(DRIVER_DIR)/_test/framework/src/test_runner.c

C_SOURCES = $(TEST_SOURCES) $(COV_C_SOURCES) $(FRAMEWORK_SOURCES)

################################################################
# Compiler
################################################################

CC = gcc

################################################################
# C flags
################################################################

C_INCLUDES =                                                \
-Imock                                                      \
-I.                                                         \
-I$(DRIVER_DIR)/onboard_flash                         		\
-I$(ROOT_DIR)/mod/common                                    \
-I$(ROOT_DIR)/mod/error_sdr                                \
-I$(ROOT_DIR)/mod/math_sdr                                \
-I$(DRIVER_DIR)/_test/framework/src

C_DEFS =                                                    \
-DSTM32H750xx                                               \
-DUNIT_TEST													\
-DRELBLD													\
-DA0005

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g

CFLAGS += -Wno-pointer-to-int-cast
CFLAGS += -Wno-int-to-pointer-cast
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable

CFLAGS += -ftest-coverage
CFLAGS += -fprofile-arcs

################################################################
# Build all
################################################################
all: clean $(BUILD_DIR)/$(TARGET)

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ -lgcov -lm

$(BUILD_DIR):
	mkdir $@

################################################################
# Build test
################################################################
test:
	@echo THIS TEST MUST BE EXECUTED IN A BASH TERMINAL. CMD/PS do not work.
	-rm -fR $(BUILD_DIR)
	$(MAKE) all
	$(BUILD_DIR)/$(TARGET)
	tail -n 7 "results.txt"
	mkdir -p coverage
	gcovr --filter $(COV_C_SOURCES) --html-details coverage/coverage.html

################################################################
# clean up
################################################################
clean:
	-rm -fR $(BUILD_DIR)

################################################################
# EOF
################################################################
