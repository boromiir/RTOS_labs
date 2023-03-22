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
