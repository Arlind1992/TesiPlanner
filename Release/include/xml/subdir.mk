################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../include/xml/tinyxml2.cpp 

OBJS += \
./include/xml/tinyxml2.o 

CPP_DEPS += \
./include/xml/tinyxml2.d 


# Each subdirectory must supply rules for building sources it contributes
include/xml/%.o: ../include/xml/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/arlind/git/TesiPlanner/include" -O3 -Wall -c -fmessage-length=0 -std=c++11 -g -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


