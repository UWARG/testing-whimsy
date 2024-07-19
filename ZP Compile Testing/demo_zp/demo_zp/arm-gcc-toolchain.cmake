# Define the C compiler
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross compiler
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE INTERNAL "")
set(CMAKE_SIZE arm-none-eabi-size CACHE INTERNAL "")

set(MCU_CPU cortex-m33)  # Adjust as per your ARM architecture
set(MCU_FLOAT_ABI hard) # Adjust based on your ABI requirement
set(MCU_FLAGS "-mcpu=${MCU_CPU} -mthumb -mfloat-abi=${MCU_FLOAT_ABI}")

set(COMMON_FLAGS "${MCU_FLAGS} -g -Wall -Wextra -Wno-unused-parameter -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu11" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -Wno-implicit-fallthrough -Wno-register -std=c++17 -fno-rtti -fno-exceptions -fno-unwind-tables" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "")

set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -specs=nosys.specs -specs=nano.specs -Wl,-gc-sections" CACHE INTERNAL "")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DDEBUG")


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)