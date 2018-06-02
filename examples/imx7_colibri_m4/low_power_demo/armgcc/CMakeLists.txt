INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

 
# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})
SET(BspRootDirPath ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)

# DEBUG LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${BspRootDirPath}/platform/devices/MCIMX7D/linker/gcc/MCIMX7D_M4_ocram.ld  -static")

# RELEASE LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${BspRootDirPath}/platform/devices/MCIMX7D/linker/gcc/MCIMX7D_M4_ocram.ld  -static")

# DEBUG ASM FLAGS
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# DEBUG C FLAGS
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g  -O0  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# DEBUG LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs")

# RELEASE ASM FLAGS
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# RELEASE LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs")

# ASM MACRO
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

# C MACRO
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -D__DEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DCPU_MCIMX7D_M4")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DCPU_MCIMX7D_M4")

# CXX MACRO

# INCLUDE_DIRECTORIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../..)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/include)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/startup)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/drivers/inc)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/rtos/FreeRTOS/Source/include)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/utilities/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../..)
    INCLUDE_DIRECTORIES(${ProjDirPath}/..)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../..)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/include)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/startup)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/drivers/inc)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/rtos/FreeRTOS/Source/include)
    INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/utilities/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../..)
    INCLUDE_DIRECTORIES(${ProjDirPath}/..)
ENDIF()

# ADD_EXECUTABLE
ADD_EXECUTABLE(low_power_demo
    "${BspRootDirPath}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h"
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/gcc/startup_MCIMX7D_M4.S"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c"
    "${BspRootDirPath}/platform/utilities/src/debug_console_imx.c"
    "${BspRootDirPath}/platform/utilities/inc/debug_console_imx.h"
    "${BspRootDirPath}/platform/utilities/src/print_scan.c"
    "${BspRootDirPath}/platform/utilities/src/print_scan.h"
    "${BspRootDirPath}/platform/drivers/inc/ccm_analog_imx7d.h"
    "${BspRootDirPath}/platform/drivers/inc/ccm_imx7d.h"
    "${BspRootDirPath}/platform/drivers/inc/lmem.h"
    "${BspRootDirPath}/platform/drivers/inc/rdc.h"
    "${BspRootDirPath}/platform/drivers/inc/rdc_defs_imx7d.h"
    "${BspRootDirPath}/platform/drivers/inc/wdog_imx.h"
    "${BspRootDirPath}/platform/drivers/src/ccm_analog_imx7d.c"
    "${BspRootDirPath}/platform/drivers/src/ccm_imx7d.c"
    "${BspRootDirPath}/platform/drivers/src/lmem.c"
    "${BspRootDirPath}/platform/drivers/src/rdc.c"
    "${BspRootDirPath}/platform/drivers/src/wdog_imx.c"
    "${BspRootDirPath}/platform/drivers/src/ecspi.c"
    "${BspRootDirPath}/platform/drivers/inc/ecspi.h"
    "${BspRootDirPath}/platform/drivers/src/mu_imx.c"
    "${BspRootDirPath}/platform/drivers/inc/mu_imx.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/croutine.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/event_groups.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/FreeRTOS.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/list.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/mpu_wrappers.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/portable.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/projdefs.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/queue.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/semphr.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/StackMacros.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/task.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/include/timers.h"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/croutine.c"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/event_groups.c"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/list.c"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/queue.c"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/tasks.c"
    "${BspRootDirPath}/rtos/FreeRTOS/Source/timers.c"
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.c"
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.h"
    "${ProjDirPath}/../../pin_mux.c"
    "${ProjDirPath}/../../pin_mux.h"
    "${ProjDirPath}/../../board.c"
    "${ProjDirPath}/../../board.h"
    "${ProjDirPath}/../../clock_freq.c"
    "${ProjDirPath}/../../clock_freq.h"
    "${ProjDirPath}/../FreeRTOSConfig.h"
    "${ProjDirPath}/../hardware_init.c"
    "${ProjDirPath}/../gpio_pins.c"
    "${ProjDirPath}/../gpio_pins.h"
    "${ProjDirPath}/../ugui/ugui.h"
    "${ProjDirPath}/../ugui/ugui.c"
    "${ProjDirPath}/../lpm_mcore.c"
    "${ProjDirPath}/../lpm_mcore.h"
    "${ProjDirPath}/../main.c"
    "${ProjDirPath}/../main.c"
    "${BspRootDirPath}/platform/drivers/src/gpio_imx.c"
    "${BspRootDirPath}/platform/drivers/inc/gpio_imx.h"
    "${BspRootDirPath}/platform/drivers/src/uart_imx.c"
    "${BspRootDirPath}/platform/drivers/inc/uart_imx.h"
)
SET_TARGET_PROPERTIES(low_power_demo PROPERTIES OUTPUT_NAME "low_power_demo.elf")

TARGET_LINK_LIBRARIES(low_power_demo -Wl,--start-group)
# LIBRARIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
ENDIF()

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(low_power_demo m)
TARGET_LINK_LIBRARIES(low_power_demo c)
TARGET_LINK_LIBRARIES(low_power_demo gcc)
TARGET_LINK_LIBRARIES(low_power_demo nosys)
TARGET_LINK_LIBRARIES(low_power_demo -Wl,--end-group)

# MAP FILE
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}  -Xlinker -Map=debug/low_power_demo.map")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}  -Xlinker -Map=release/low_power_demo.map")

# BIN AND HEX
ADD_CUSTOM_COMMAND(TARGET low_power_demo POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/low_power_demo.elf ${EXECUTABLE_OUTPUT_PATH}/low_power_demo.hex)
ADD_CUSTOM_COMMAND(TARGET low_power_demo POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/low_power_demo.elf ${EXECUTABLE_OUTPUT_PATH}/low_power_demo.bin)
