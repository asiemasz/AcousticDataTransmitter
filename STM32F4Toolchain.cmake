# System Generic - no OS bare-metal application
set(CMAKE_SYSTEM_NAME Generic)

# Setup arm processor and bleeding edge toolchain
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar) #Creates archives or static libraries
set(CMAKE_OBJCOPY arm-none-eabi-objcopy) #Converts obj file format
set(CMAKE_OBJDUMP arm-none-eabi-objdump) #Displays information from object files
set(CMAKE_NM arm-none-eabi-nm) #Lists symbols from object files
set(CMAKE_STRIP arm-none-eabi-strip) #Discard symbols and other data from object files
set(CMAKE_RANLIB arm-none-eabi-ranlib) #Generates an index to an archive

# When trying to link cross compiled test program, error occurs, so setting test compilation to static library
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Remove default static libraries for win32
set(CMAKE_C_STANDARD_LIBRARIES "")

set(ARM_OPTIONS 
   -mcpu=cortex-m4 
   -mfloat-abi=hard 
   -mthumb 
   --specs=nano.specs 
   -mfpu=fpv4-sp-d16
)

add_compile_options(
  ${ARM_OPTIONS}
  -fmessage-length=0
  -funsigned-char
  -fsingle-precision-constant
  -ffast-math
  -ffunction-sections
  -fdata-sections
  -MMD
  -MP
  -gdwarf-2
  -g3
  -Ofast
)

add_link_options(
  ${ARM_OPTIONS}
  --specs=nosys.specs
  -u_printf_float
  -u_scanf_float
#  -nostartfiles
  -T../STM32F401RETx_FLASH.ld
  LINKER:--gc-sections
  LINKER:--build-id
  LINKER:--print-memory-usage
)

macro(add_arm_executable target_name)

# Output files
set(elf_file ${target_name}.elf)
set(map_file ${target_name}.map)
set(hex_file ${target_name}.hex)
set(bin_file ${target_name}.bin)
set(lss_file ${target_name}.lss)
set(dmp_file ${target_name}.dmp)

add_executable(${elf_file} ${ARGN})

#generate hex file
add_custom_command(
	OUTPUT ${hex_file}

	COMMAND
		${CMAKE_OBJCOPY} -O ihex ${elf_file} ${hex_file}

	DEPENDS ${elf_file}
)

# #generate bin file
add_custom_command(
	OUTPUT ${bin_file}

	COMMAND
		${CMAKE_OBJCOPY} -O binary ${elf_file} ${bin_file}

	DEPENDS ${elf_file}
)

# #generate extended listing
add_custom_command(
	OUTPUT ${lss_file}

	COMMAND
		${CMAKE_OBJDUMP} -h -S ${elf_file} > ${lss_file}

	DEPENDS ${elf_file}
)

# #generate memory dump
add_custom_command(
	OUTPUT ${dmp_file}

	COMMAND
		${CMAKE_OBJDUMP} -x --syms ${elf_file} > ${dmp_file}

	DEPENDS ${elf_file}
)

#postprocessing from elf file - generate hex bin etc.
add_custom_target(
	${CMAKE_PROJECT_NAME}
	ALL
	DEPENDS ${hex_file} ${bin_file} ${lss_file} ${dmp_file}
)

set_target_properties(
	${CMAKE_PROJECT_NAME}

	PROPERTIES
		OUTPUT_NAME ${elf_file}
)

endmacro(add_arm_executable)

macro(arm_link_libraries target_name)

target_link_libraries(${target_name}.elf ${ARGN})

endmacro(arm_link_libraries)