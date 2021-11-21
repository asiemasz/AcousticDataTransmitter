cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE STM32F4Toolchain.cmake)

#set C and CPP standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_ASM_FLAGS "${CFLAGS} -x assembler-with-cpp")

enable_language(C ASM)

project(AcousticDataTransmitter)

add_compile_options(
  -Wall
  -Wextra
)

add_compile_definitions(
  DEBUG
)

include_directories(
  "Drivers/CMSIS/Include"
  "Drivers/CMSIS/DSP/Include"
  "Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  "inc"
  "DSP_own/src"
  "MCP4822"
)

add_compile_definitions(
  STM32F401xE
  ARM_MATH_CM4
  __FPU_PRESENT=1U
)

file(GLOB C_SRCS CMAKE_CONFIGURE_DEPENDS
  "src/*.c"
  "Drivers/CMSIS/Device/ST/STM32F4xx/Source/*.c"
  "Drivers/CMSIS/DSP/Source/BasicMathFunction/*.c"
  "Drivers/CMSIS/DSP/Source/CommonTables/*.c"  
  "Drivers/CMSIS/DSP/Source/ComplexMathFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/ControllerFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/FastMathFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/FilteringFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/MatrixFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/StatisticsFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/SupportFunctions/*.c"  
  "Drivers/CMSIS/DSP/Source/TransformFunctions/*.c"  
  "MCP4822/*.c"
  "DSP_own/src/*.c"
  "main.c"
)

file(GLOB ASM_SRCS CMAKE_CONFIGURE_DEPENDS
  "startup/*.s"
  "Drivers/CMSIS/DSP/Source/TransformFunctions/*.s"
)

add_arm_executable(AcousticDataTransmitter ${CPP_SRCS} ${C_SRCS} ${ASM_SRCS})


# Print executable size
add_custom_command(TARGET AcousticDataTransmitter
        POST_BUILD
        COMMAND arm-none-eabi-size ${elf_file})

set(LINKER_SCRIPT "STM32F401RETx_FLASH.ld")

find_program(OPENOCD_EXECUTABLE "openocd" PATHS DOC "OpenOCD debugger and flash utility")

get_filename_component(OPENOCD_CONFIG openocd.cfg ABSOLUTE)

add_custom_target(flash COMMAND ${OPENOCD_EXECUTABLE}
 -f ${OPENOCD_CONFIG}
 --command "program ${elf_file} reset exit")

 add_custom_target(reset COMMAND ${OPENOCD_EXECUTABLE}
 -f ${OPENOCD_CONFIG}
 --command "init" --command "reset" --command "exit")