# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zgrywcio/TMC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zgrywcio/TMC/build

# Include any dependencies generated for this target.
include CMakeFiles/main.elf.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.elf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.elf.dir/flags.make

CMakeFiles/main.elf.dir/Drivers/CMSIS/startup_stm32f407xx.s.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Drivers/CMSIS/startup_stm32f407xx.s.o: ../Drivers/CMSIS/startup_stm32f407xx.s
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building ASM object CMakeFiles/main.elf.dir/Drivers/CMSIS/startup_stm32f407xx.s.o"
	/bin/arm-none-eabi-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -o CMakeFiles/main.elf.dir/Drivers/CMSIS/startup_stm32f407xx.s.o -c /home/zgrywcio/TMC/Drivers/CMSIS/startup_stm32f407xx.s

CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.o: ../Drivers/CMSIS/system_stm32f4xx.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.o   -c /home/zgrywcio/TMC/Drivers/CMSIS/system_stm32f4xx.c

CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Drivers/CMSIS/system_stm32f4xx.c > CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.i

CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Drivers/CMSIS/system_stm32f4xx.c -o CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.s

CMakeFiles/main.elf.dir/Src/setup.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/setup.c.o: ../Src/setup.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/main.elf.dir/Src/setup.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/setup.c.o   -c /home/zgrywcio/TMC/Src/setup.c

CMakeFiles/main.elf.dir/Src/setup.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/setup.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/setup.c > CMakeFiles/main.elf.dir/Src/setup.c.i

CMakeFiles/main.elf.dir/Src/setup.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/setup.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/setup.c -o CMakeFiles/main.elf.dir/Src/setup.c.s

CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.o: ../Src/drivers/GPIO.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.o   -c /home/zgrywcio/TMC/Src/drivers/GPIO.c

CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/drivers/GPIO.c > CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.i

CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/drivers/GPIO.c -o CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.s

CMakeFiles/main.elf.dir/Src/drivers/USART1.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/drivers/USART1.c.o: ../Src/drivers/USART1.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/main.elf.dir/Src/drivers/USART1.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/drivers/USART1.c.o   -c /home/zgrywcio/TMC/Src/drivers/USART1.c

CMakeFiles/main.elf.dir/Src/drivers/USART1.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/drivers/USART1.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/drivers/USART1.c > CMakeFiles/main.elf.dir/Src/drivers/USART1.c.i

CMakeFiles/main.elf.dir/Src/drivers/USART1.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/drivers/USART1.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/drivers/USART1.c -o CMakeFiles/main.elf.dir/Src/drivers/USART1.c.s

CMakeFiles/main.elf.dir/Src/drivers/USART3.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/drivers/USART3.c.o: ../Src/drivers/USART3.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/main.elf.dir/Src/drivers/USART3.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/drivers/USART3.c.o   -c /home/zgrywcio/TMC/Src/drivers/USART3.c

CMakeFiles/main.elf.dir/Src/drivers/USART3.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/drivers/USART3.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/drivers/USART3.c > CMakeFiles/main.elf.dir/Src/drivers/USART3.c.i

CMakeFiles/main.elf.dir/Src/drivers/USART3.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/drivers/USART3.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/drivers/USART3.c -o CMakeFiles/main.elf.dir/Src/drivers/USART3.c.s

CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.o: ../Src/drivers/SPI3.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.o   -c /home/zgrywcio/TMC/Src/drivers/SPI3.c

CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/drivers/SPI3.c > CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.i

CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/drivers/SPI3.c -o CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.s

CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.o: ../Src/Sensors/BNO080/bno080.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.o   -c /home/zgrywcio/TMC/Src/Sensors/BNO080/bno080.c

CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/Sensors/BNO080/bno080.c > CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.i

CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/Sensors/BNO080/bno080.c -o CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.s

CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.o: ../Src/Sensors/BNO080/hw.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.o   -c /home/zgrywcio/TMC/Src/Sensors/BNO080/hw.c

CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/Sensors/BNO080/hw.c > CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.i

CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/Sensors/BNO080/hw.c -o CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.s

CMakeFiles/main.elf.dir/Src/interrupts.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/interrupts.c.o: ../Src/interrupts.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object CMakeFiles/main.elf.dir/Src/interrupts.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/interrupts.c.o   -c /home/zgrywcio/TMC/Src/interrupts.c

CMakeFiles/main.elf.dir/Src/interrupts.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/interrupts.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/interrupts.c > CMakeFiles/main.elf.dir/Src/interrupts.c.i

CMakeFiles/main.elf.dir/Src/interrupts.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/interrupts.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/interrupts.c -o CMakeFiles/main.elf.dir/Src/interrupts.c.s

CMakeFiles/main.elf.dir/Src/main.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/main.c.o: ../Src/main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object CMakeFiles/main.elf.dir/Src/main.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/main.c.o   -c /home/zgrywcio/TMC/Src/main.c

CMakeFiles/main.elf.dir/Src/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/main.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/main.c > CMakeFiles/main.elf.dir/Src/main.c.i

CMakeFiles/main.elf.dir/Src/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/main.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/main.c -o CMakeFiles/main.elf.dir/Src/main.c.s

CMakeFiles/main.elf.dir/Src/time/time.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/time/time.c.o: ../Src/time/time.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object CMakeFiles/main.elf.dir/Src/time/time.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/time/time.c.o   -c /home/zgrywcio/TMC/Src/time/time.c

CMakeFiles/main.elf.dir/Src/time/time.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/time/time.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/time/time.c > CMakeFiles/main.elf.dir/Src/time/time.c.i

CMakeFiles/main.elf.dir/Src/time/time.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/time/time.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/time/time.c -o CMakeFiles/main.elf.dir/Src/time/time.c.s

CMakeFiles/main.elf.dir/Src/IO/LED.c.o: CMakeFiles/main.elf.dir/flags.make
CMakeFiles/main.elf.dir/Src/IO/LED.c.o: ../Src/IO/LED.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object CMakeFiles/main.elf.dir/Src/IO/LED.c.o"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.elf.dir/Src/IO/LED.c.o   -c /home/zgrywcio/TMC/Src/IO/LED.c

CMakeFiles/main.elf.dir/Src/IO/LED.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.elf.dir/Src/IO/LED.c.i"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zgrywcio/TMC/Src/IO/LED.c > CMakeFiles/main.elf.dir/Src/IO/LED.c.i

CMakeFiles/main.elf.dir/Src/IO/LED.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.elf.dir/Src/IO/LED.c.s"
	/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zgrywcio/TMC/Src/IO/LED.c -o CMakeFiles/main.elf.dir/Src/IO/LED.c.s

# Object files for target main.elf
main_elf_OBJECTS = \
"CMakeFiles/main.elf.dir/Drivers/CMSIS/startup_stm32f407xx.s.o" \
"CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.o" \
"CMakeFiles/main.elf.dir/Src/setup.c.o" \
"CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.o" \
"CMakeFiles/main.elf.dir/Src/drivers/USART1.c.o" \
"CMakeFiles/main.elf.dir/Src/drivers/USART3.c.o" \
"CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.o" \
"CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.o" \
"CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.o" \
"CMakeFiles/main.elf.dir/Src/interrupts.c.o" \
"CMakeFiles/main.elf.dir/Src/main.c.o" \
"CMakeFiles/main.elf.dir/Src/time/time.c.o" \
"CMakeFiles/main.elf.dir/Src/IO/LED.c.o"

# External object files for target main.elf
main_elf_EXTERNAL_OBJECTS =

main.elf: CMakeFiles/main.elf.dir/Drivers/CMSIS/startup_stm32f407xx.s.o
main.elf: CMakeFiles/main.elf.dir/Drivers/CMSIS/system_stm32f4xx.c.o
main.elf: CMakeFiles/main.elf.dir/Src/setup.c.o
main.elf: CMakeFiles/main.elf.dir/Src/drivers/GPIO.c.o
main.elf: CMakeFiles/main.elf.dir/Src/drivers/USART1.c.o
main.elf: CMakeFiles/main.elf.dir/Src/drivers/USART3.c.o
main.elf: CMakeFiles/main.elf.dir/Src/drivers/SPI3.c.o
main.elf: CMakeFiles/main.elf.dir/Src/Sensors/BNO080/bno080.c.o
main.elf: CMakeFiles/main.elf.dir/Src/Sensors/BNO080/hw.c.o
main.elf: CMakeFiles/main.elf.dir/Src/interrupts.c.o
main.elf: CMakeFiles/main.elf.dir/Src/main.c.o
main.elf: CMakeFiles/main.elf.dir/Src/time/time.c.o
main.elf: CMakeFiles/main.elf.dir/Src/IO/LED.c.o
main.elf: CMakeFiles/main.elf.dir/build.make
main.elf: ../Drivers/CMSIS/libarm_cortexM4lf_math.a
main.elf: CMakeFiles/main.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zgrywcio/TMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking C executable main.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.elf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.elf.dir/build: main.elf

.PHONY : CMakeFiles/main.elf.dir/build

CMakeFiles/main.elf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.elf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.elf.dir/clean

CMakeFiles/main.elf.dir/depend:
	cd /home/zgrywcio/TMC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zgrywcio/TMC /home/zgrywcio/TMC /home/zgrywcio/TMC/build /home/zgrywcio/TMC/build /home/zgrywcio/TMC/build/CMakeFiles/main.elf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.elf.dir/depend

