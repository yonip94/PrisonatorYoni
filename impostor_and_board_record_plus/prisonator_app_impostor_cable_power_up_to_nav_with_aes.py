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
    global ack_type16
    global ack_rand1
    global ack_encrypted_rand2
    global rand2_arrived
    # Buffer to store incoming data
    buffer = bytearray()
    counter_ = 0
    while True:
        # Read data from the serial port
        if (packet.in_waiting > 0):
            data = packet.read(packet.in_waiting)
            buffer.extend(data)
            if(len(data) == 6):
                print(f"{buffer[:5].decode('utf-8', errors='ignore')}")
                if(data_base64_type17_ack == data):
                    print("board generated private key (sends ack on id and blk)")
                    ack_type16 = 1 #id,blk ack
                    
                elif(data_base64_type17_nack == data):
                    print("board not generated private key (sends nack on id and blk)")
                    ack_type16 = 0 #id,blk nack
    
                elif(data_base64_type23_ack == data):
                    print("board sends operation done - ack on rand2")
                    ack_encrypted_rand2 = 1
                elif(data_base64_type23_nack == data):
                    print("board sends operation fail - nack on rand2")
                    ack_encrypted_rand2 = 0
                        
            elif(len(data) == 46): #decryption of rand1 number base 64 including header
                
                if(data_rand1_encrypted_including_header_base64 == data):
                    print("board sends encrypted rand1 with header")
                    print(f"{buffer[0:45].decode('utf-8', errors='ignore')}")  
                    ack_rand1 = 1
                    rand2_arrived = 0
                elif (data_rand2_including_header_base64 == data):
                    print("board sends rand2 number + header")
                    print(f"{buffer[0:45].decode('utf-8', errors='ignore')}")  
                    rand2_arrived = 1
                    ack_rand1 = 0
                else:
                    ack_rand1 = 0
                    rand2_arrived = 0
        
            counter_ = counter_ + 1
            if (counter_ >= 100):
                counter_ = 0
                #print(f"{buffer[:10000].decode('utf-8', errors='ignore')}")
                #print(buffer[0])
                
            # Process the buffer (for demonstration, just print the buffer content)
            #print(buffer[0])
            #print(f"{buffer[:10000].decode('utf-8', errors='ignore')}")
            #print(len(data))
            
            # Clear the buffer if needed after processing
            buffer.clear()

        # Sleep for a short time to prevent high CPU usage
        time.sleep(0.01)
              
ack_rand1 = 2
ack_encrypted_rand2 = 2
ack_type16 = 2
rand2_arrived = 0

data_ka = bytes([0x08, 0x05])
data_ignore_ka = bytes([0x08, 0x0b])
data_not_ignore_ka = bytes([0x08, 0x0c])
data_reset = bytes([0x08, 0x06])
data_start_navigation = bytes([0x08, 0x01])
data_start_calibration = bytes([0x08, 0x03])
data_ack = bytes([0x02])

data_blk_iv_mac  = bytes([ 0x10,
                           0x02,
                           0x01,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00,
                           0xe1,0x0a,0x58,0x41,
                           0xd2,0x3f,0x50,0x04,
                           0x32,0x13,0x2a,0xb7,
                           0xe3,0x0f,0x87])#type16, blk 2, iv vector, imei


data_base64_type17_ack  = bytes([0x45,0x51,0x45,0x3D,0x0D,0x0A])#type17 ack  0x11,0x01  EQE=
data_base64_type17_nack = bytes([0x45,0x51,0x41,0x3D,0x0D,0x0A])#type17 nack 0x11,0x00  EQA=

data_rand1_with_header = bytes([0x12,0xe3,0x96,0x72,
                                0x18,0xfa,0xdc,0x50,
                                0x07,0xff,0xc2,0x1e,
                                0x2b,0x43,0x9c,0xa1,
                                0x0e,0x13,0x82,0xca,
                                0x6b,0x42,0x20,0xd4,
                                0xed,0x54,0xfa,0x78,
                                0x66,0xaa,0x9c,0x21,
                                0x85])#type18, 32bytes rand1 number + 1 header

data_rand1_encrypted = bytes([0x12,0xC5,0x06,0xD0,
                              0x2B,0x63,0x2D,0x27,
                              0x26,0x87,0xAF,0x60,
                              0x45,0x82,0x29,0xEC,
                              0x03,0x65,0xBC,0xD4,
                              0x55,0x02,0x0C,0x33,
                              0x52,0x54,0x52,0xB2,
                              0xEB,0xF3,0xDE,0x2E])#32bytes enc rand1 number

data_rand1_encrypted_including_header = bytes([0x13,
                                               0x12,0xC5,0x06,0xD0,
                                               0x2B,0x63,0x2D,0x27,
                                               0x26,0x87,0xAF,0x60,
                                               0x45,0x82,0x29,0xEC,
                                               0x03,0x65,0xBC,0xD4,
                                               0x55,0x02,0x0C,0x33,
                                               0x52,0x54,0x52,0xB2,
                                               0xEB,0xF3,0xDE,0x2E])#1 header + 32bytes enc rand1 number

data_rand1_encrypted_including_header_base64 = bytes([0x45, 0x78, 0x4C, 0x46,
                                                      0x42, 0x74, 0x41, 0x72,
                                                      0x59, 0x79, 0x30, 0x6E,
                                                      0x4A, 0x6F, 0x65, 0x76, 
                                                      0x59, 0x45, 0x57, 0x43,
                                                      0x4B, 0x65, 0x77, 0x44,
                                                      0x5A, 0x62, 0x7A, 0x55,
                                                      0x56, 0x51, 0x49, 0x4D, 
                                                      0x4D, 0x31, 0x4A, 0x55,
                                                      0x55, 0x72, 0x4C, 0x72,
                                                      0x38, 0x39, 0x34, 0x75,
                                                      0x0D,0x0A])#1 header + 32bytes enc base64 res is ExLFBtArYy0nJoevYEWCKewDZbzUVQIMM1JUUrLr894u
data_rand1_ack = bytes([0x14,0x01])#type18
data_rand1_nack = bytes([0x14,0x00])#type18

data_rand2_encrypted_including_header = bytes([0x16,
                                               0x82,0x30,0xB5,0xF5,
                                               0xB7,0xDB,0x96,0x28,
                                               0x11,0x05,0xD8,0x57,
                                               0xED,0xEF,0x95,0x1A,
                                               0x49,0x20,0x76,0xD5,
                                               0xB8,0x22,0xF1,0x8C,
                                               0x80,0xCF,0x1E,0xB5,
                                               0xAD,0x44,0x85,0x5C])#type19, 32bytes rand2 enc + header number

data_rand2_including_header_base64  = bytes([0x46, 0x6F, 0x49, 0x77,
                                             0x74, 0x66, 0x57, 0x33, 
                                             0x32, 0x35, 0x59, 0x6F, 
                                             0x45, 0x51, 0x58, 0x59, 
                                             0x56, 0x2B, 0x33, 0x76, 
                                             0x6C, 0x52, 0x70, 0x4A, 
                                             0x49, 0x48, 0x62, 0x56, 
                                             0x75, 0x43, 0x4C, 0x78, 
                                             0x6A, 0x49, 0x44, 0x50, 
                                             0x48, 0x72, 0x57, 0x74, 
                                             0x52, 0x49, 0x56, 0x63,
                                             0x0D,0x0A])#type19, #1 header + 32bytes rand2 base64 number is FoIwtfW325YoEQXYV+3vlRpJIHbVuCLxjIDPHrWtRIVc

data_base64_type23_ack  = bytes([0x46,0x77,0x45,0x3D,0x0D,0x0A])#type23 ack  0x17,0x01  FwE=
data_base64_type23_nack = bytes([0x46,0x77,0x61,0x3D,0x0D,0x0A])#type23 nack 0x17,0x00  FwA=

port = "COM3"
baudrate = 460800
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE
get_timeout = 0.5
xonxoff = False
rtscts = False
dsrdtr = False #False here will reset board when close window, better to be true
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
        print("app sends ka")

    counter_timeout = 0
    ack_type16 = 2
    while (ack_type16 == 2):
        send_data(data_blk_iv_mac)
        print("app sends header+blk+mac")
        time.sleep(1)
        counter_timeout = counter_timeout + 1
        if(counter_timeout >= 15):
            os._exit(0)
            
    if (ack_type16 == 1):
        print("app continues to the next stage - to send rand1 + header")
    else:
        os._exit(0)
        
    counter_timeout = 0
    while (ack_rand1 == 2):
        send_data(data_rand1_with_header)
        time.sleep(1)
        counter_timeout = counter_timeout + 1
        if(counter_timeout >= 15):
            os._exit(0)

    if (ack_rand1 == 1):
        print("app decrypted rand1 and send ack to board (agree with result)")
        send_data(data_rand1_ack)
    else:
        print("app decrypted rand1 and send nack to board (not-agree with result)")
        send_data(data_rand1_nack)
        os._exit(0)

    counter_timeout = 0
    while (rand2_arrived == 0):
        time.sleep(1)
        counter_timeout = counter_timeout + 1
        if(counter_timeout >= 15):
            os._exit(0)

    if (rand2_arrived == 1):
        print("app got rand2 number, continues to the next state")
    else:
        os._exit(0) 
    
    counter_timeout = 0
    while (ack_encrypted_rand2 == 2):
        print("app sends enctypted rand2 + header to board")
        send_data(data_rand2_encrypted_including_header)
        time.sleep(1)
        counter_timeout = counter_timeout + 1
        if(counter_timeout >= 15):
            os._exit(0)

    if (ack_encrypted_rand2 == 1):
        print("AES PROCESS PASS")

        print("app sends ACK CALIBRATION")
        send_data(data_ack)
        time.sleep(1)

        print("app send IGNORE KA")
        send_data(data_ignore_ka)
        time.sleep(1)
        
        print("app send START NAVIGATE")
        send_data(data_start_navigation)
        time.sleep(1)
        
        while(True):
            pass
    else:
        os._exit(0)

    


