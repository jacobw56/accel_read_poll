#include(CMakeForceCompiler)

# Usage:
# cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ../

# Set the toolchain name.
set(TOOLCHAIN arm-none-eabi)
set(TOOLCHAIN_PREFIX ${TOOLCHAIN}-)

set(TOOLCHAIN_INCLUDE_DIR ${ARM_NONE_EABI_PATH}/include)

# Generic is used for embedded targets w/o OS.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR armv7e-m)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# Specify the cross compiler.
set(CMAKE_AR_COMPILER ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(OBJDUMP ${TOOLCHAIN_PREFIX}objdump)

set(OPTIMIZATION_FLAGS "-O2")
#set(OPTIMIZATION_FLAGS "-Og -g3")
#set(COMMON_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs")
set(COMMON_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs")

set(C_FLAGS "${OPTIMIZATION_FLAGS} ${COMMON_FLAGS}")
set(C_FLAGS "${C_FLAGS} -Wall -Werror")
set(C_FLAGS "${C_FLAGS} -ffunction-sections -fdata-sections -fno-builtin -fshort-enums -fstack-usage")
set(C_FLAGS "${C_FLAGS} -D__HEAP_SIZE=0x0000 -D__STACK_SIZE=0x2000")

set(CXX_FLAGS "${OPTIMIZATION_FLAGS}")

set(ASM_FLAGS "${COMMON_FLAGS}")
set(ASM_FLAGS "${ASM_FLAGS} -D__HEAP_SIZE=0x0000 -D__STACK_SIZE=0x2000")

set(LINKER_FLAGS "${OPTIMIZATION_FLAGS} ${COMMON_FLAGS}")
set(LINKER_FLAGS "${LINKER_FLAGS} -Wl,-emain,--gc-sections,--print-memory-usage,-Map,bootloader_template.map")
set(LINKER_FLAGS "${LINKER_FLAGS} -specs=nano.specs")

set(LINKER_LIBRARIES_DIR ${SDK_ROOT_DIR}/modules/nrfx/mdk)
set(LINKER_LIBRARIES "-L${LINKER_LIBRARIES_DIR}")

set(LINKER_SCRIPT "linker_script.ld")
set(LINKER_SCRIPTS "-T${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT}")

set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -std=c++11" CACHE INTERNAL "")
set(CMAKE_C_FLAGS "${C_FLAGS} -std=c11" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS "${ASM_FLAGS}" CACHE INTERNAL "")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS} ${LINKER_LIBRARIES} ${LINKER_SCRIPTS}" CACHE INTERNAL "")
