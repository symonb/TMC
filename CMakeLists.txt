cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_STANDARD 11)

set(TARGET "main")

set(TARGET_ELF "${TARGET}.elf")
set(TARGET_HEX "${TARGET}.hex")
set(TARGET_BIN "${TARGET}.bin")

project(TMC C ASM)
set(MAIN_SRC_DIR "${CMAKE_SOURCE_DIR}/Src")



set(LD_include "-lc -lm -lnosys")

set(linker_script "${CMAKE_SOURCE_DIR}/STM32F407VGTx_FLASH.ld")

set(MCU_flags "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb ")
    
set(C_flags "${MCU_flags} -Wall -fdata-sections -ffunction-sections -DARM_MATH_CM4 -D__FPU_PRESENT")
set(AS_flags "${MCU_flags} -Wall -fdata-sections -ffunction-sections")
set(LD_flags "${MCU_flags} -specs=nano.specs -specs=nosys.specs  -T${linker_script} ${LD_include} -Wl,--print-memory-usage")



macro(main_sources var) 
    set(${var} ${ARGN})
    list(TRANSFORM ${var} PREPEND "${MAIN_SRC_DIR}/")
endmacro()

add_subdirectory("${MAIN_SRC_DIR}")
main_sources(src "${MAIN_SRC}")

add_executable(${TARGET_ELF}
    Drivers/CMSIS/startup_stm32f407xx.s
    Drivers/CMSIS/system_stm32f4xx.c
    "${src}")

target_include_directories(${TARGET_ELF} PUBLIC
    Src/Std_Periph/inc
    Src
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include)

set(CMAKE_C_FLAGS "${C_flags}")
set(CMAKE_ASM_FLAGS "${AS_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${LD_flags}") 
set(OBJCOPY "/bin/arm-none-eabi-objcopy")
target_link_libraries(${TARGET_ELF} PUBLIC "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/libarm_cortexM4lf_math.a")

add_custom_target(${TARGET_BIN} ALL COMMAND ${OBJCOPY} -O binary -S ${TARGET_ELF} ${TARGET_BIN})
add_dependencies(${TARGET_BIN} ${TARGET_ELF})

