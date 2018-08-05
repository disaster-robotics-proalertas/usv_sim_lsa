################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/freefloating_gazebo_control.cpp \
../src/freefloating_gazebo_fluid.cpp \
../src/freefloating_pids.cpp \
../src/freefloating_pids_body.cpp \
../src/freefloating_pids_joint.cpp \
../src/freefloating_pids_main.cpp \
../src/test_pid.cpp 

OBJS += \
./src/freefloating_gazebo_control.o \
./src/freefloating_gazebo_fluid.o \
./src/freefloating_pids.o \
./src/freefloating_pids_body.o \
./src/freefloating_pids_joint.o \
./src/freefloating_pids_main.o \
./src/test_pid.o 

CPP_DEPS += \
./src/freefloating_gazebo_control.d \
./src/freefloating_gazebo_fluid.d \
./src/freefloating_pids.d \
./src/freefloating_pids_body.d \
./src/freefloating_pids_joint.d \
./src/freefloating_pids_main.d \
./src/test_pid.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/gazebo-7 -I/opt/ros/kinetic/include -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


