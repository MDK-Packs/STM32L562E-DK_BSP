Platform Project
================

The 'Platform' project configures the hardware of the evaluation board
and is a CMSIS-RTOS2 based software template that can be further expanded.

This implementation contains the following configured interface drivers:

  - CMSIS-Driver LPUART1 routed to Arduino UNO R3 connector
    - TX: CN12 pin 2
    - RX: CN12 pin 1

  - CMSIS-Driver SPI3 routed to Arduino UNO R3 connector
    - SCK:  CN11 pin 6
    - MISO: CN11 pin 5
    - MOSI: CN11 pin 4
    - SSN:  CN11 pin 3

  - CMSIS-Driver VIO with the following board hardware mapping
    - vioBUTTON0:        PC13 - Button USER
    - vioLED0:           PD3  - LD9 RED
    - vioLED1:           PG12 - LD10 GREEN
    - vioMotionGyro:     iNEMO 3D gyroscope (LSM6DSO)
    - vioMotionAccelero: iNEMO 3D accelerometer (LSM6DSO)

The CMSIS-RTOS2 is based on RTX5 with the following configuration settings:

   - Global Dynamic Memory size: 32768 bytes

   - Default Thread Stack size: 3072 bytes
