set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION Cortex-M4-STM32F401)

#Compilers
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

#skip checking compiler 
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
