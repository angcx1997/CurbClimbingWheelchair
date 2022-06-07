#!/usr/bin/python3
import csv
from os import sync
import struct
import serial
import time
import yaml

def setupPort(ser, config):
    # Initialize port
    ser.port=config['serial']['name']
    ser.baudrate=config['serial']['baudrate']
    ser.stopbits=config['serial']['stopbit']
    ser.parity=config['serial']['parity']
    ser.bytesize=config['serial']['bytesize']
    ser.timeout=config['serial']['timeout']
    try:
        ser.open()
        ser.flushInput()
    except serial.SerialException as var:
        print("An exception occurred when try to open serial port")
        print("Exception details-> ", var)
        return False
    else:
        print("Serial port opened")
        return True

def checkIntegrity(packet,config):
    # Check start of transmission
    try:
        if (packet[0] != config['soi']):
            return False
        #Get data length and calculate checksum
        data_length = packet[1]
        checksum = 0
        # Checksum = data[0] + data[1] + data[2:data_length]
        for i in packet[:2+data_length]:
            checksum += i
        #Checksum
        if ((checksum >> 8 & 0xFF) != packet[len(packet) - 1 -2] and 
            (checksum & 0xFF) != packet[len(packet) - 1 - 1]):
            return False
        
        #Check end of transmission
        if (packet[len(packet)-1] != config['eoi']):
            return False
        return True  
    except:
        return False

def byteFormatter(type, packet, start_idx):
    if(type == "float"):
        end_idx = start_idx + 4
        return struct.unpack('f', packet[start_idx:end_idx])[0], end_idx
    elif(type == "uint8_t"):
        end_idx = start_idx + 1
        return struct.unpack('B', packet[start_idx:end_idx])[0], end_idx
    elif(type == "uint16_t"):
        end_idx = start_idx + 2
        return struct.unpack('H', packet[start_idx:end_idx])[0], end_idx
    elif(type == "uint32_t"):
        end_idx = start_idx + 4
        return struct.unpack('I', packet[start_idx:end_idx])[0], end_idx
    elif(type == "int8_t"):
        end_idx = start_idx + 1
        return struct.unpack('b', packet[start_idx:end_idx])[0], end_idx
    elif(type == "int16_t"):
        end_idx = start_idx + 2
        return struct.unpack('h', packet[start_idx:end_idx])[0], end_idx
    elif(type == "int32_t"):
        end_idx = start_idx + 4
        return struct.unpack('i', packet[start_idx:end_idx])[0], end_idx


def main():
    # Load file from yaml
    with open('serial_config.yaml') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    # Store size of packet
    packet_length = config['total_number_receive']   

    # Load csv
    csv_name = config['csv-filename']
    # f = open("../log/"+csv_name, "w")
    f = open(csv_name, "w")
    writer = csv.writer(f, delimiter=",")
    
    # Input header into csv
    header = ['Index', 'Timestamp']
    for key in config['data-logging']:
        name = config['data-logging'][key]['name']
        header.append(name)
    print(header)
    writer.writerow(header)

    # Setup serial port
    ser = serial.Serial()
    port_open = False
    while(not port_open):
        port_open = setupPort(ser,config)
        if (port_open == False):
            # Reset serial port
            ser.close()
            time.sleep(1)
        
    time.sleep(1)
    # while (setupPort(ser,config) != True):
    #     pass

    # Data logger
    index = 1
    false_reception = 0

    sync_flag = False
    
    # Start reading the serial port
    while True:
        command = 23
        # print(b'\xff')
        ser.write(b'\xff')
        # ser.write(bytes(command, 'utf-8'))
        # time.sleep(0.05)
        
        line = ser.read(packet_length)
        data_integrity = checkIntegrity(line,config)

        if(data_integrity):
            if(index%100 == 0):
                print("[%d, %d]Data received"%(index,time.time()))
            logged_data = [index,time.time_ns() // 1000000 ]
            start_idx = 2
            for key in config['data-logging']:
                type = config['data-logging'][key]['type']
                data, start_idx = byteFormatter(type, line, start_idx)
                logged_data.append(data)
            writer.writerow(logged_data)
            index+=1
        else:
            false_reception += 1

        if(false_reception > 10):
            # Reset serial port
            ser.close()
            # Setup serial port
            ser = serial.Serial()
            port_open = False
            while(not port_open):
                port_open = setupPort(ser,config)
                if (port_open == False):
                    # Reset serial port
                    ser.close()
                    time.sleep(1)
            false_reception = 0




#Run main() function
if __name__ == '__main__':
    main()