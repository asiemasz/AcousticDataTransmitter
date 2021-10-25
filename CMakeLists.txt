cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE STM32F4Toolchain.cmake)

#set C and CPP standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_ASM_FLAGS "${CFLAGS} -x assembler-with-cpp")

enable_language(C ASM)

project(AcousticDataTransmitter)

add_compile_options(
  -Wall
  -Wextra
  -g3
  -Og
)

add_compile_definitions(
  DEBUG
)

include_directories(
  "Drivers/CMSIS/Include"
  "Drivers/CMSIS/DSP/Include"
  "Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  "inc"
)

add_compile_definitions(
  STM32F401xE
  ARM_MATH_CM4
  __FPU_PRESENT
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

)

file(GLOB ASM_SRCS CMAKE_CONFIGURE_DEPENDS
  "startup/*.s"
)

add_arm_executable(AcousticDataTransmitter ${CPP_SRCS} ${C_SRCS} ${ASM_SRCS})

set(LINKER_SCRIPT "STM32F401RETx_FLASH.ld")
