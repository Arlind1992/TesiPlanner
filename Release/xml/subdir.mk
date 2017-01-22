################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../xml/tinyxml2.cpp 

OBJS += \
./xml/tinyxml2.o 

CPP_DEPS += \
./xml/tinyxml2.d 


# Each subdirectory must supply rules for building sources it contributes
xml/%.o: ../xml/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


