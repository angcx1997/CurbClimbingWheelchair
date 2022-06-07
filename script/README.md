# DataLogger_SerialSniffer

Used to log data from MCU to PC. 

Highly customizable and easy to use.

# Installation

1. Python Package Dependency Installation
    - `pyyaml`,`pyserial`,`pyserial`, `csv`, `struct`
2. Copy the script file to the MCU workspace
    - Remember to include the script into compiler include path if using STM32CubeIDE

# Manual

## PC

1. Use the `serial_config.yaml` to customise some important parameter.
2. Following are important parameter to be provided user:
    
    `csv-filename`: The filename that user want to store the logged data
    
    `serial/name`: port name. Can be search by `dmesg | grep tty` with the MCU connected
    
    `serial/baudrate`: total bit transfer per second.
    
    `total_number_receive`: size of byte user want to send
    
    `data/name`: name will be used in the header
    
    `data/type`: supported data type, `float`, `uint8/26/32_t`, and `int8/16/32_t`
3. Run the script by making the file executable `chmod +x serial_sniffer.py`
4. Execute the file `python3 serial_sniffer.py`
    

## MCU

Example code:

```c
#include "main.h"
#include "../script/DataLogger.h"

//1. Declare variable
DataLogger_Msg_t datalog_msg;

int main(void)
{
	//2. Initialize variable
	DataLogger_Init(&datalog_msg);
 
  while (1)
  {
		//example:
		//Use data structure specified inside script/UnionDataStruct.h
		//If user is not providing array directly but casting from type other than uint8_t
	  float_byte_u float_data;
	  float_data.data = 1.0;

	  uint32_byte_u uint32_data;
	  uint32_data.data = 0x12345678;
		
		//3. Add the array into datalog message
	  DataLogger_AddMessage(&datalog_msg, float_data.array, sizeof(float_data.array));
	  DataLogger_AddMessage(&datalog_msg, uint32_data.array, sizeof(uint32_data.array));
	  //4. After added all the message, call complete tx message to add check to ensure data integrity
		DataLogger_CompleteTxMessage(&datalog_msg);
		//5. Send the TX msg using serial protocol such as UART, USB, CAN, etc
	  HAL_UART_Transmit(&huart3, datalog_msg.pMsg, datalog_msg.size, 1);
  }
}

```
