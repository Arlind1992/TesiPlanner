################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/grid/Grid.cpp 

OBJS += \
./src/grid/Grid.o 

CPP_DEPS += \
./src/grid/Grid.d 


# Each subdirectory must supply rules for building sources it contributes
src/grid/%.o: ../src/grid/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/arlind/git/TesiPlanner/include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


