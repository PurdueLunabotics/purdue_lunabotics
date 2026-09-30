# ---------------------------------------------------------------------------------------
# Target System Configuration
# ---------------------------------------------------------------------------------------
# "Generic" tells CMake we are targeting a bare-metal/embedded system without an OS
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Force cross-compilation active
set(CMAKE_CROSSCOMPILING TRUE)

# ---------------------------------------------------------------------------------------
# Toolchain Executable Definitions
# ---------------------------------------------------------------------------------------
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# System-dependent executable extension (.exe for Windows)
if(WIN32)
    set(TOOLCHAIN_EXT ".exe")
else()
    set(TOOLCHAIN_EXT "")
endif()

# Find the compilers and tools on the system PATH
find_program(CMAKE_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXT} REQUIRED)
find_program(CMAKE_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_EXT} REQUIRED)
find_program(CMAKE_ASM_COMPILER NAMES ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXT} REQUIRED)
find_program(CMAKE_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_EXT} REQUIRED)
find_program(CMAKE_OBJDUMP NAMES ${TOOLCHAIN_PREFIX}objdump${TOOLCHAIN_EXT} REQUIRED)
find_program(CMAKE_SIZE NAMES ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_EXT} REQUIRED)

# ---------------------------------------------------------------------------------------
# CMake Try-Compile Behavior
# ---------------------------------------------------------------------------------------
# Prevent CMake from trying to compile and link a standard test executable.
# Bare-metal programs require custom linker scripts and startup code, causing the default 
# test to fail. Compiling as a static library skips the link step during validation.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---------------------------------------------------------------------------------------
# Search Path Policies
# ---------------------------------------------------------------------------------------
# Look for tools (like compilers) exclusively on the host system, 
# but isolate libraries, headers, and packages to the target environment.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---------------------------------------------------------------------------------------
# Default Core Flags (Bare-Metal Bare Minimums)
# ---------------------------------------------------------------------------------------
# Note: You should pass your specific hardware architecture flags (like -mcpu=cortex-m4) 
# in your primary CMakeLists.txt or target configurations.

set(MCU "IMXRT1062")
set(MCU_LD "${teensy_package_SOURCE_DIR}/cores/teensy4/imxrt1062_t41.ld")
set(MCU_DEF "ARDUINO_TEENSY41")

set(OPTIONS "-DF_CPU=600000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -D__${MCU}__ -DARDUINO=10813 -DTEENSYDUINO=159 -D${MCU_DEF}")
set(CPUOPTIONS "-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb")

set(CMAKE_C_FLAGS_INIT   "-Wall -g -O2 ${CPUOPTIONS} -MMD ${OPTIONS} -I. -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "-std=gnu++17 -felide-constructors -fno-exceptions -fpermissive -fno-rtti -Wno-error=narrowing ${CMAKE_C_FLAGS_INIT}")

# Common bare-metal linker specifications
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Os -Wl,--gc-sections,--relax ${CPUOPTIONS} -T${MCU_LD} -lm -lstdc++") # missing -larm_cortexM7lfsp_math 
