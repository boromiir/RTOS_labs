# RTOS_labs
Task solutions for RTOS laboratory (run on STM32F446 with CMSIS V1 FreeRTOS)

## Lab 1
Basic introduction
+ Prepare a simple FreeRTOS application, capable of blinking led light with a frequency equal to 10Hz.
+ Prepare an application that has 2 tasks running in parallel:
  - the first task - blinks a LED with the frequency defined as a modifiable parameter (global variable) with some initial value e.g. 10,
  - the second task - runs periodically every 1s for checking whether the button has ever been pressed (no matter what amount of times). When the press was detected, the modifiable parameter shall be increased with an arbitrarily chosen value.
+ Add interrupt on button. When given the GPIO pin will be shortcut with GND, then the setting of the modifiable parameter shall return to the initial value.

## Lab 2
Inter Process Communication (ocassionaly done using STM32F303 and CMSIS V2 FreeRTOS)
+ Take a sample from the ADC from the temperature channel of the microcontroller and from any additional channel connected to the selected pin. Send the values to the main task using queue so it can calculate average of temperature and frequency of chosen pin; fullfill 3 scenarios:
  - collect data from ADC using a software timer provided by FrreRTOS,
  - collect data from ADC using a hardware timer that drives ADC,
  - collect data from ADC using DMA in circular buffer configuration.
+ Add task printing data over UART.
  
## Lab 3 
Concurrency
+ Implement 3 tasks on different levels of priority calculating mean and mean square of 1000 randomly generated values in range of <-1.0, 1.0>; put the calculations in global stucture.
+ Make highest priority task which prints the structure over UART.
+ Solve the concurrency problems using:
  - semaphore,
  - mutex.

