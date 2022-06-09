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
  -Wdouble-promotion
)

include_directories(
  "Drivers/CMSIS/Include"
  "Drivers/CMSIS/DSP/Include"
  "Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  "inc"
)


file(GLOB C_SRCS
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
  "main.c"
)

file(GLOB ASM_SRCS
  "startup/*.s"
  "Drivers/CMSIS/DSP/Source/TransformFunctions/*.s"
)

add_subdirectory(MCP4822)

####################################################################################
#This must be set explicitly before including DSP_own as it's dependend on CMSIS_DSP
add_compile_definitions(
  STM32F401xE
  ARM_MATH_CM4
  __FPU_PRESENT=1U
  __FPU_USED=1U
)
add_compile_options(
  -mfloat-abi=hard
  -mfpu=fpv4-sp-d16
)
add_subdirectory(DSP_own)
#####################################################################################

add_arm_executable(AcousticDataTransmitter ${CPP_SRCS} ${C_SRCS} ${ASM_SRCS})
arm_link_libraries(AcousticDataTransmitter MCP4822 DSP_own)

# Print executable size
add_custom_command(TARGET AcousticDataTransmitter
        POST_BUILD
        COMMAND arm-none-eabi-size ${elf_file})

find_program(OPENOCD_EXECUTABLE "openocd" PATHS DOC "OpenOCD debugger and flash utility")

get_filename_component(OPENOCD_CONFIG openocd.cfg ABSOLUTE)

add_custom_target(flash COMMAND ${OPENOCD_EXECUTABLE}
 -f ${OPENOCD_CONFIG}
 --command "program ${elf_file} reset exit")

 add_custom_target(reset COMMAND ${OPENOCD_EXECUTABLE}
 -f ${OPENOCD_CONFIG}
 --command "init" --command "reset" --command "exit")