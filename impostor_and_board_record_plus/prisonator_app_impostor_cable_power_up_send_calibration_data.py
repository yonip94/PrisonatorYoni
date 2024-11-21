import time
import serial
#import binascii
import sys
from threading import Thread
from multiprocessing import Process
from multiprocessing import Pool
import os
import time

def send_data(data):
    try:
        sent = packet.write(data)
        #if sent == len(data):
        #    print(f"Sent: {data}")
        #else:
        #    print(f"Only sent {sent} bytes of {len(data)}")
    except serial.SerialTimeoutException:
        print("Write timeout occurred.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")

#connect uart dongle tx to rx, check com number and put on port
def task_uart_get():
    # Buffer to store incoming data
    buffer = bytearray()
    counter_ = 0
    while True:
        # Read data from the serial port
        if (packet.in_waiting > 0):
            data = packet.read(packet.in_waiting)
            buffer.extend(data)
            
            counter_ = counter_ + 1
            if (counter_ >= 100):
                counter_ = 0
                #print(f"{buffer[:10000].decode('utf-8', errors='ignore')}")
                #print(buffer[0])
                
            # Process the buffer (for demonstration, just print the buffer content)
            # print(f"{buffer[:10000].decode('utf-8', errors='ignore')}")
            # print(buffer[0])

            # Clear the buffer if needed after processing
            buffer.clear()

        # Sleep for a short time to prevent high CPU usage
        time.sleep(0.01)

        


data_ka = bytes([0x08, 0x05])
data_ignore_ka = bytes([0x08, 0x0b])
data_not_ignore_ka = bytes([0x08, 0x0c])
data_reset = bytes([0x08, 0x06])
data_start_navigation = bytes([0x08, 0x01])
data_start_calibration = bytes([0x08, 0x03])
data_ack = bytes([0x02])
data_p0 = bytes([0x0C, 1,2,3,4,5,6,7,8,9,0x08])
data_pn = bytes([0x0C, 1,2,3,4,5,6,7,8,9,0x0a,0x08,1])
data_pn_temp = bytearray([0x0C, 1,2,3,4,5,6,7,8,9,0x0a,0x08,1])

port = "COM3"
baudrate = 460800
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE
get_timeout = 0.5
xonxoff = False
rtscts = False
dsrdtr = True #False here will reset board when close window
send_timeout = 1
inter_byte_timeout = None
exclusive = None

packet = serial.Serial(
    port=port,
    baudrate=baudrate,
    bytesize=bytesize,
    parity=parity,
    stopbits=stopbits,
    timeout=get_timeout,
    xonxoff=xonxoff,
    rtscts=rtscts,
    dsrdtr=dsrdtr,
    write_timeout=send_timeout,
    inter_byte_timeout=inter_byte_timeout,
    exclusive=exclusive
)

if packet.is_open:
    print(f"Serial port {port} opened successfully.")
else:
    print(f"Failed to open serial port {port}.")
    
task_uart_get = Thread(target=task_uart_get)
task_uart_get.daemon = True
task_uart_get.start()


while True:
    cnt = 0
    while cnt<12:
        cnt=cnt+1
    
        time.sleep(1)
        send_data(data_ka)
        print("KA_SEND")
        
    time.sleep(1)
    send_data(data_ack)
    print("ACK_SEND")

    time.sleep(1)
    send_data(data_ignore_ka)
    print("IGNORE_KA_SEND")

    time.sleep(1)
    send_data(data_start_calibration)
    print("START CALIBRATION")
    
    while True:
       #print("STOP")
       send_data(data_p0)
       time.sleep(0.07)#time.sleep(0.07)

       for i in range(1, 90):
           data_pn_temp[12] = i
           data_pn = bytes(data_pn_temp)
           #print(data_pn)
           send_data(data_pn)
           time.sleep(0.07)#time.sleep(0.07)
       time.sleep(0.2)




