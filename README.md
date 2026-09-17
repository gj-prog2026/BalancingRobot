# Balancing Robot

## Introduction
This is an ongoing personal projected started on 02/08/2026. Up to 17/09/2026, the budget was controlled to within £600. The aim of this project was to build a steady balancing robot in the hopes of learning about PID control. 


## Hardware
Both the PCB and robot frame were designed by myself with PCB manufactured by JLC Lab and the frame 3D printed at home. The bottom layer and motor brackets have to withstand the most vibration so carbon fibre PETG was chosen for its rigidity. For the remaining parts of the frame, PLA was chosen as it was easier to print and there was no special requirements for material strength. 


## Software
The development was divided into several stages:
|Stage|Purpose|
|-----|-------|
|0| Confirm the clock ticks are generated correctly|
|1| Confirm the MPU6050 is connected to I2C of BluePill and can receive data from measurements made by gyroscope|
|2| Confirm the BluePill can receive data from MPU6050 via I2C and send messages to laptop through UART by HC-05|
|3| Confirm BluePill can receive commands from laptop via bluetooth communication by checking motors respond to commands|
|4| Adjust PID to achieve preliminary balance (robot should be able to balance on its own at this stage)|


>Note
>I am currently stuck at stage 4 as it is proving to be very difficult to adjust to the right PID.
>I tried making I and D equal 0 and use only P to facilitate me in finding the deadband which I hypothesize is the root cause of inability to balance despite the maths working.
