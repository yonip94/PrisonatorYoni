import bluetooth
import time
import threading
import datetime  # Import the datetime module
import os
import queue
import keyboard
import psutil  # Import the psutil library
import numpy as np

#pip install pybluez
#pip install git+https://github.com/pybluez/pybluez.git#egg=pybluez
#C:\Users\mgroup2\AppData\Local\Programs\Python\Python311\Lib\site-packages

ALLOW_IDLE = 0

# Define the target MAC address as a string
#target_mac_address = "44:17:93:F7:8C:B2"
target_mac_address = "78:21:84:EE:F9:2A"
#target_mac_address = "44:17:93:F7:8C:66"
#target_mac_address = "44:17:93:F7:8C:3A"
#target_mac_address = "44:17:93:F7:8C:A6"
#target_mac_address = "44:17:93:F7:8C:8A"
keypress_queue = queue.Queue()
packet_index_to_get = 0
disconnection_number = 0
idle_mode_flag = 0
send_ack_flag = 0

def set_affinity(thread, core):
    pid = os.getpid()
    psutil.Process(pid).cpu_affinity([core])
    print(f"Thread {thread} pinned to core {core}")

# Function to set the flag to True
def set_flag():
    global reset_board_flag
    reset_board_flag = 1
    
def read_data(sock):
    global disconnection_number
    global idle_mode_flag
    global send_ack_flag
    bat_voltage = 0
    bat_current = 0
    bat_current_avg = 0
    last_bat_current = 0
    counter_of_bat_current_samples = 0
    bat_current_total = 0
    bat_remain_capacity = 0
    fail_to_get_packet_size = 0
    fail_to_get_packet_x = 0
    #last_packet_timestamp = 0
    first_packet_to_read_flag = 0
    error_flag_imu = 0
    error_flag_mag = 0
    packet_counter = 0
    current_sign = 0
    disconnection_number = disconnection_number + 1

    # Initialize a variable to store the start time
    #current_timestamp = time.time()
    while True:
        try:
            data = sock.recv(850*6)  # Receive data from the device (adjust buffer size as needed)
            if not data:
                print("Disconnected from the device.")
                break
            if(data[0]==0x01):
               send_ack_flag = 1
                
            if((data[0]!=0x00) and (data[0]!=0x0a) and (data[0]!=0x06)):
                print(data[0])
                continue

            if (first_packet_to_read_flag == 0):
                first_packet_to_read_flag = 1
                packet_index_to_get = ((data[3] << 16) | (data[2] << 8) | data[1])
                packet_counter = packet_counter + 1

            ##cs calc
            cs_val = 0;
            for i in range(0,850,1):
                if (i==747):
                    continue
                cs_val =  cs_val+(i+1)*data[i]
            if (data[747]==(cs_val&0xff)):
                print(f"packet cs OK = {data[747]}")
            else:
                print(f"packet cs FAIL = {data[747]}") 
            ##
            
            packet_elapsed_us1 = ((data[80] << 32) | (data[79] << 24) | (data[78] << 16) | (data[77] << 8) | data[76])
            if (packet_counter>=2):
                if (packet_elapsed_us8>packet_elapsed_us1):
                    print("imu 8>1")
                    print(f"packet_elapsed_us1 = {packet_elapsed_us1}[us]")
                    print(f"{data[80]:02X}{data[79]:02X}{data[78]:02X}{data[77]:02X}{data[76]:02X}")
                    print(f"packet_elapsed_us2 = {packet_elapsed_us2}[us]")
                    print(f"{data[157]:02X}{data[156]:02X}{data[155]:02X}{data[154]:02X}{data[153]:02X}")
                    print(f"packet_elapsed_us3 = {packet_elapsed_us3}[us]")
                    print(f"{data[234]:02X}{data[233]:02X}{data[232]:02X}{data[231]:02X}{data[230]:02X}")
                    print(f"packet_elapsed_us4 = {packet_elapsed_us4}[us]")
                    print(f"{data[311]:02X}{data[310]:02X}{data[309]:02X}{data[308]:02X}{data[307]:02X}")
                    print(f"packet_elapsed_us5 = {packet_elapsed_us5}[us]")
                    print(f"{data[388]:02X}{data[387]:02X}{data[386]:02X}{data[385]:02X}{data[384]:02X}")
                    print(f"packet_elapsed_us6 = {packet_elapsed_us6}[us]")
                    print(f"{data[465]:02X}{data[464]:02X}{data[463]:02X}{data[462]:02X}{data[461]:02X}")
                    print(f"packet_elapsed_us7 = {packet_elapsed_us7}[us]")
                    print(f"{data[542]:02X}{data[541]:02X}{data[540]:02X}{data[539]:02X}{data[538]:02X}")
                    print(f"packet_elapsed_us8 = {packet_elapsed_us8}[us]")
                    print(f"{data[619]:02X}{data[618]:02X}{data[617]:02X}{data[616]:02X}{data[615]:02X}")
                    error_flag_imu = 1
                    print("code exit")
                    os._exit(0)

            packet_elapsed_us2 = ((data[157] << 32) | (data[156] << 24) | (data[155] << 16) | (data[154] << 8) | data[153])
            packet_elapsed_us3 = ((data[234] << 32) | (data[233] << 24) | (data[232] << 16) | (data[231] << 8) | data[230])
            packet_elapsed_us4 = ((data[311] << 32) | (data[310] << 24) | (data[309] << 16) | (data[308] << 8) | data[307])
            packet_elapsed_us5 = ((data[388] << 32) | (data[387] << 24) | (data[386] << 16) | (data[385] << 8) | data[384])
            packet_elapsed_us6 = ((data[465] << 32) | (data[464] << 24) | (data[463] << 16) | (data[462] << 8) | data[461])
            packet_elapsed_us7 = ((data[542] << 32) | (data[541] << 24) | (data[540] << 16) | (data[539] << 8) | data[538])
            packet_elapsed_us8 = ((data[619] << 32) | (data[618] << 24) | (data[617] << 16) | (data[616] << 8) | data[615])

            mag_ts1= ((data[651] << 32) | (data[650] << 24) | (data[649] << 16) | (data[648] << 8) | data[647])
            if (packet_counter>=2):
                if (mag_ts4>mag_ts1):
                    print("mag 4>1")
                    print(f"mag_ts1 = {mag_ts1}[us]")
                    print(f"{data[651]:02X}{data[650]:02X}{data[649]:02X}{data[648]:02X}{data[647]:02X}")
                    print(f"mag_ts2 = {mag_ts2}[us]")
                    print(f"{data[680]:02X}{data[679]:02X}{data[678]:02X}{data[677]:02X}{data[676]:02X}")
                    print(f"mag_ts3 = {mag_ts3}[us]")
                    print(f"{data[709]:02X}{data[708]:02X}{data[707]:02X}{data[706]:02X}{data[705]:02X}")
                    print(f"mag_ts4 = {mag_ts4}[us]")
                    print(f"{data[738]:02X}{data[737]:02X}{data[736]:02X}{data[735]:02X}{data[734]:02X}")
                    error_flag_mag = 1
                    print("code exit")
                    os._exit(0)
            mag_ts2= ((data[680] << 32) | (data[679] << 24) | (data[678] << 16) | (data[677] << 8) | data[676])
            mag_ts3= ((data[709] << 32) | (data[708] << 24) | (data[707] << 16) | (data[706] << 8) | data[705])
            mag_ts4= ((data[738] << 32) | (data[737] << 24) | (data[736] << 16) | (data[735] << 8) | data[734])

            if (first_packet_to_read_flag == 1):
                if (packet_elapsed_us1>packet_elapsed_us2):
                    print("imu 1>2")
                    error_flag_imu = 1
                elif (packet_elapsed_us2>packet_elapsed_us3):
                    print("imu 2>3")
                    error_flag_imu = 1
                elif (packet_elapsed_us3>packet_elapsed_us4):
                    print("imu 3>4")
                    error_flag_imu = 1
                elif (packet_elapsed_us4>packet_elapsed_us5):
                    print("imu 4>5")
                    error_flag_imu = 1
                elif (packet_elapsed_us5>packet_elapsed_us6):
                    print("imu 5>6")
                    error_flag_imu = 1
                elif (packet_elapsed_us6>packet_elapsed_us7):
                    print("imu 6>7")
                    error_flag_imu = 1
                elif (packet_elapsed_us7>packet_elapsed_us8):
                    print("imu 7>8")
                    error_flag_imu = 1

                if (mag_ts1>mag_ts2):
                    print("mag 1>2")
                    error_flag_mag = 1
                elif (mag_ts2>mag_ts3):
                    print("mag 2>3")
                    error_flag_mag = 1
                    print("mag 3>4")
                elif (mag_ts3>mag_ts4):
                    error_flag_mag = 1

                if ((error_flag_imu==1) or (error_flag_mag == 1)):
                    print(f"packet_elapsed_us1 = {packet_elapsed_us1}[us]")
                    print(f"{data[80]:02X}{data[79]:02X}{data[78]:02X}{data[77]:02X}{data[76]:02X}")
                    print(f"packet_elapsed_us2 = {packet_elapsed_us2}[us]")
                    print(f"{data[157]:02X}{data[156]:02X}{data[155]:02X}{data[154]:02X}{data[153]:02X}")
                    print(f"packet_elapsed_us3 = {packet_elapsed_us3}[us]")
                    print(f"{data[234]:02X}{data[233]:02X}{data[232]:02X}{data[231]:02X}{data[230]:02X}")
                    print(f"packet_elapsed_us4 = {packet_elapsed_us4}[us]")
                    print(f"{data[311]:02X}{data[310]:02X}{data[309]:02X}{data[308]:02X}{data[307]:02X}")
                    print(f"packet_elapsed_us5 = {packet_elapsed_us5}[us]")
                    print(f"{data[388]:02X}{data[387]:02X}{data[386]:02X}{data[385]:02X}{data[384]:02X}")
                    print(f"packet_elapsed_us6 = {packet_elapsed_us6}[us]")
                    print(f"{data[465]:02X}{data[464]:02X}{data[463]:02X}{data[462]:02X}{data[461]:02X}")
                    print(f"packet_elapsed_us7 = {packet_elapsed_us7}[us]")
                    print(f"{data[542]:02X}{data[541]:02X}{data[540]:02X}{data[539]:02X}{data[538]:02X}")
                    print(f"packet_elapsed_us8 = {packet_elapsed_us8}[us]")
                    print(f"{data[619]:02X}{data[618]:02X}{data[617]:02X}{data[616]:02X}{data[615]:02X}")
                    print(f"mag_ts1 = {mag_ts1}[us]")
                    print(f"{data[651]:02X}{data[650]:02X}{data[649]:02X}{data[648]:02X}{data[647]:02X}")
                    print(f"mag_ts2 = {mag_ts2}[us]")
                    print(f"{data[680]:02X}{data[679]:02X}{data[678]:02X}{data[677]:02X}{data[676]:02X}")
                    print(f"mag_ts3 = {mag_ts3}[us]")
                    print(f"{data[709]:02X}{data[708]:02X}{data[707]:02X}{data[706]:02X}{data[705]:02X}")
                    print(f"mag_ts4 = {mag_ts4}[us]")
                    print(f"{data[738]:02X}{data[737]:02X}{data[736]:02X}{data[735]:02X}{data[734]:02X}")
                    
                    print("code exit")
                    os._exit(0)

            elapsed_s = (((data[3] << 16) | (data[2] << 8) | data[1]))/(25)
            bat_voltage = (data[638]<<8) | (data[637])
            bat_current = int((data[640]<<8) | (data[639]))
            bat_temp = (int((data[644]<<8) | (data[643]))/100)
            Battery_precents = data[850-48-48-2] 
            #print(f"{data[640]:02X}{data[639]:02X}")
            #print(np.int16(bat_current))
            #print(np.uint16(bat_current))
            current_sign = 0
            if ((np.uint16(bat_current) >= 0x8000)): #if its negative current (discharge current)
                current_sign=1 
                if ((last_bat_current) != (-(np.uint16(~np.uint16(bat_current)) + 1)) and (data[850-48-48-2]<=100)):
                    bat_current_total = (int)((int)(bat_current_total) + (int)(bat_current))
                    counter_of_bat_current_samples=counter_of_bat_current_samples+1
                    bat_current_avg = float(bat_current_total/counter_of_bat_current_samples)
                    last_bat_current = -(np.uint16(~np.uint16(bat_current)) + 1)
                
            bat_remain_capacity = (data[642]<<8) | (data[641])
            idle_mode_flag = data[850-48-48-1]

            print(f"Disconnect num                 = {disconnection_number-1}")
            if (Battery_precents==0x7F):#127
                print("Battery is not connected")
            else:
                print(f"Battery voltage                = {bat_voltage}[mV]")
                
                if(current_sign == 1):#if its negative current (discharge current)
                    print(f"Battery current                = -{np.uint16(~np.uint16(bat_current)) + 1}[mA]")
                else:#if its positive current (charge current)
                    print(f"Battery current                = {np.uint16(bat_current)}[mA]")
                    
                print(f"Battery discharge current_avg  = {np.int16(bat_current_avg)}[mA]")
                print(f"Battery remain capacity        = {bat_remain_capacity}[mAh]")
                print(f"Battery Temp                   = {bat_temp}[Celsius]")
                if((Battery_precents&0x80)!=0x00):
                    print(f"Battery precents not directly  = {Battery_precents&0x7f}[%]")	
                else:
                    print(f"Battery precents yes directly  = {Battery_precents&0x7f}[%]")
                
            print(f"Idle status                    = {idle_mode_flag}") 
            print(f"PACKET TS                      = {packet_elapsed_us1}[us]")
            print(f"TS                             = {elapsed_s}[s]")
            print(f"Type                           = {data[0]:02X}") 
            print(f"Packet SN                      = {data[3]:02X}{data[2]:02X}{data[1]:02X}")
        
            if (packet_index_to_get != ((data[3] << 16) | (data[2] << 8) | data[1])):
                print(packet_index_to_get)
                print(f"{data[3]:02X}{data[2]:02X}{data[1]:02X} Packet SN")
                fail_to_get_packet_x = 1
                
            packet_index_to_get = packet_index_to_get + 1;
    
            if ((len(data)%850) != 0):
                print(len(data))
                fail_to_get_packet_size = 1
            else:
                counter_packets = 0
                temp_size = len(data)
                while ((temp_size/850)!=1):
                    
                    counter_packets = counter_packets + 1
                    if (packet_index_to_get != ((data[(counter_packets*850)+3] << 16) | (data[(counter_packets*850)+2] << 8) | data[(counter_packets*850)+1])):
                        print(packet_index_to_get)
                        print(f"{data[3]:02X}{data[2]:02X}{data[1]:02X} Packet SN")
                        fail_to_get_packet_x = 1
                        packet_index_to_get = packet_index_to_get + 1 
                        break

                    print(f"Type       = {data[(counter_packets*850)]:02X}") 
                    print(f"Packet SN  = {data[(counter_packets*850)+3]:02X}{data[(counter_packets*850)+2]:02X}{data[(counter_packets*850)+1]:02X}")
                    print(f"Pulse Byte = {data[(counter_packets*850)+750]:02X}")
                    ##cs calc
                    cs_val = 0;
                    for i in range(0,850,1):
                        if (i==747):
                            continue
                        cs_val =  cs_val+(i+1)*data[(counter_packets*850)+i]
                    if (data[(counter_packets*850)+747]==(cs_val&0xff)):
                        print(f"packet cs OK = {data[(counter_packets*850)+747]}")
                    else:
                        print(f"packet cs FAIL = {data[(counter_packets*850)+747]}") 
                    ##
                    packet_index_to_get = packet_index_to_get + 1
                    temp_size = temp_size - 850
                    
            #fails print
            if (fail_to_get_packet_x != 0):
                print(f"failed to get buffer x")
            if (fail_to_get_packet_size != 0):
                print(f"failed to get 850 bytes")
                
            print("")
            
        except Exception as e:
            print(f"Error reading data: {str(e)}")
            break

def send_data(sock):
    data_ka = bytes([0x08, 0x05])
    data_reset = bytes([0x08, 0x06])
    data_start_navigation = bytes([0x08, 0x01])
    data_ack = bytes([0x02])
    idle_bt_off_command = bytes([0x07])
    
    time.sleep(1)
    sock.send(data_ka)
    time.sleep(1)
    sock.send(data_ack)
    time.sleep(1)
    sock.send(data_start_navigation)
    time.sleep(1)
    global idle_mode_flag
    global ALLOW_IDLE
    global send_ack_flag
    while True:
        try:
            
            if (send_ack_flag == 1):
                time.sleep(1)
                print("send ack")
                sock.send(data_ack)
                time.sleep(1)
                send_ack_flag = 0
                
            if ((idle_mode_flag == 1) and (ALLOW_IDLE==1)):
                sock.send(idle_bt_off_command)
                time.sleep(1)
                
            if (reset_board_flag == 1):
                #time.sleep(1)  # Wait for 1 second before sending the next data
                sock.send(data_reset)
                time.sleep(1)
                
            # Data to send (0x08, 0x05)
            sock.send(data_ka)
            #print(f"Sent data: {data}")
            time.sleep(1)  # Wait for 1 second before sending the next data
                
        except Exception as e:
            print(f"Error sending data: {str(e)}")
            break

def scan_and_connect():
    nearby_devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True)

    for address, name in nearby_devices:
        if address == target_mac_address:
            print(f"Found target device: {name} ({address})")
            print("Connecting to the device...")
            while True:
                try:
                    # Attempt to connect to the target device
                    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                    sock.connect((address, 1))  # The second argument is the Bluetooth channel (usually 1)
                    print("Connected to the device!")
                    
                    # Start sending and reading data in separate threads or processes as needed
                    send_thread = threading.Thread(target=send_data, args=(sock,))
                    send_thread.daemon = True
                    
                    read_thread = threading.Thread(target=read_data, args=(sock,))
                    read_thread.daemon = True
                    
                    keypress_thread = threading.Thread(target=keypress_listener)
                    keypress_thread.daemon = True
                    
                    cpu_count = psutil.cpu_count()
                    print("number of cores = " + str(cpu_count))
                    #print(cpu_count)
                    
                    set_affinity(keypress_thread, 0)  # Pin keypress_thread to core 0
                    set_affinity(send_thread, 1)      # Pin send_thread to core 1
                    set_affinity(read_thread, 2)      # Pin read_thread to core 2
                    
                    keypress_thread.start()    
                    send_thread.start()
                    read_thread.start()
                    
                    send_thread.join()
                    read_thread.join()

                    # Don't forget to close the socket when you're done.
                    sock.close()
                    print("Disconnected from the device.")
                    break  # Exit the loop after connecting to the target device

                except Exception as e:
                    print(f"Error connecting to the device: {str(e)}")
                    # Add a delay before attempting to reconnect
                    print("Retrying in 5 seconds...")
                    time.sleep(5)  # Wait for 5 seconds before retrying
        else:
            print(f"nearby target device: {name} ({address})")
    else:
        print("Target device not found in nearby devices.")
        
# Function to catch keypresses from the keyboard
def keypress_listener():
    while True:
        event = keyboard.read_event()
        if event.event_type == keyboard.KEY_DOWN:
            keypress_queue.put(event.name)
            key = keypress_queue.get_nowait()
            if ((key == "q") or (key == "Q")):
                print("code done")
                os._exit(0)
                    
            elif (key == "6"):
                set_flag()
                print("Board reset will occuured soon")          
                    
if __name__ == "__main__":
    # Start the keypress listener thread
    while True:
        reset_board_flag = 0
        scan_and_connect()






        
