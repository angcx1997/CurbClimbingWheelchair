# Curb Climbing Wheelchair

This repo focus on the program  in the embedded system(STM32F429VI) on the wheelchair.

The objective of the project is to allow the wheelchair user climbing up and down a curb with maximum height of 20.0 cm safely.

## Requirement
- STM32CubeIDE > 1.7.0
  
  All OSes: [click here for installation instructions](https://www.st.com/en/development-tools/stm32cubeide.html)


## Installation
STM32CubeIDE is used to generated the overall code and structure.
In Project Manager/Code Generator, select "Generate peripheral initialization ..."

Tips: Add the code in between the section 
- `/* USER CODE BEGIN XX */` and 
- `/* USER CODE END XX */`

Therefore, if the code regenerated through CubeMX, the code added will be saved.

## Hardware and Peripheral Used
| Hardware | Peripheral Used |
| --- | ----------- |
| Joystick (AD7606) | SPI1 | 
| Climbing motor (BD25L) | Rear: TIM8 CH4  Back: TIM1 CH2 |
| Climbing motor encoder | CAN1| 
| MPU6050 | I2C1 | 
| Hub Motor (X2_6010S) | RS485/UART3 | 
| Base motor (Sabertooth | TIM3 (Right)CH1  (Left)CH2| 
| Driving Encoder | RS485/UART4 |

## Usage
In normal operation mode, the wheelchair could be used as usual by controlled through joystick input.

To start climbing mode, user required to push the button to initiate sequence of action while the action could not be stopped in the middle of the process.

The climbing action can be briefly explained as following step:
- Both climbing wheel landed on the ground
-   climbing process would be determined as `CLIMB_UP` or `CLIMB_DOWN` depends on which leg touches ground first
- The wheelchair will start lifting itself up to appropriate height according to the curb height calculated from the encoder input
- During lifting process, the wheelchair will maintain its position and make sure it would be lifted straight up by controlling the hub motor
- Then, the wheelchair will start moving forward by distance of the wheelchair length to make sure both its wheel has passed the curb before retracting the leg
- Last, both leg will be retract to its initial position and mark the end of the climbing process.
