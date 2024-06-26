#from pymodbus.client.sync import ModbusSerialClient
from pymodbus.client import ModbusSerialClient
from pymodbus.transaction import ModbusRtuFramer
import time

def hex_groups_to_decimal(hex_groups):
    if len(hex_groups) != 4 or any(not (0 <= x < 256) for x in hex_groups):
        raise ValueError("Input must be a list of 4 integers between 0 and 255.")
    
    # Combine the hex groups into a single 32-bit hexadecimal string
    hex_number = ''.join(f"{x:02X}" for x in hex_groups)
    
    # Convert the hex string to a decimal number
    number = int(hex_number, 16)
    
    # Check if the number is in the range of a 32-bit signed integer
    if number >= (1 << 31):
        number -= (1 << 32)
    
    return number

def decimal_to_hex_groups(number):
    if not (-2**31 <= number < 2**31):
        raise ValueError("Number out of range for 32-bit signed integer.")
    
    # Convert the number to a 32-bit two's complement hexadecimal
    if number < 0:
        number = (1 << 32) + number

    hex_number = f"{number:08X}"

    hex_groups = [int(hex_number[i:i+2], 16) for i in range(0, 8, 2)]
    
    return hex_groups

def get_position(client):
    feedback_request_packet = bytearray([0x01, # Slave address
                                        0x03, # Read operation
                                        0x10, 0x06, # feedback position
                                        0x00, 0x02]) # No. of registers
    # Calculate the CRC for the packet
    crc = calculate_crc(feedback_request_packet)
    crc_bytes = crc.to_bytes(2, byteorder='little')
    feedback_request_packet.extend(crc_bytes)
    # Send the request packet
    client.socket.write(feedback_request_packet)
    #print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
    # Receive the response packet
    feedback_response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
    pos_feedback_hexs = [feedback_response_packet[3], feedback_response_packet[4], feedback_response_packet[5], feedback_response_packet[6]]
    position = hex_groups_to_decimal(pos_feedback_hexs)
    
    return position


def calculate_crc(data):
    """Calculate CRC-16 Modbus for the given data."""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def main():
    # Define the serial port parameters
    port = 'COM12'          # Change this to the correct COM port on your machine
    baudrate = 38400 #  9600        # Set the baud rate for the serial connection
    parity = 'E'           # Set parity ('N' for None, 'E' for Even, 'O' for Odd)
    stopbits = 1           # Set the number of stop bits (1 or 2)
    bytesize = 8           # Set the number of data bits (usually 8)

    # Create a Modbus serial client for RTU
    client = ModbusSerialClient(method='rtu', port=port, baudrate=baudrate,
                                parity=parity, stopbits=stopbits, bytesize=bytesize,
                                timeout=1, framer=ModbusRtuFramer)

    # Connect to the server
    connection = client.connect()
    if not connection:
        print("Unable to connect to the server")
        return

    try:
        # Construct the packet with specified bytes values
        # Here we use a custom Modbus request to send the raw bytes
        #request_packet = bytearray([0x01, 0x08, 0x00, 0x00, 0x00, 0x00]) # Maintenance echo message
        #request_packet = bytearray([0x01, 0x03, 0x10, 0x00, 0x00, 0x02]) # Read query

        # PA1_02    INC/ABS selection (0=INC, 1=ABS) [P4-7]

        # Operation sequence taken from an example in Alpha5 manual page 13-36

        # print("-------------------")
        # print("Assign [S-ON] to CONT9")
        # # CONT9 at PA3_09 address (HEX) is 4208 (p.13-17)
        # request_packet = bytearray([0x01, # Slave address
        #                             0x10, # Write operation
        #                             0x42, 0x08, # PA3_09 address
        #                             0x00, 0x02, # Number of registers
        #                             0x04, # No. of data bytes = 4
        #                             0x00, 0x00, 0x00, 0x01]) # Data=1 [S-ON]}
        # # Calculate the CRC for the packet
        # crc = calculate_crc(request_packet)
        # crc_bytes = crc.to_bytes(2, byteorder='little')
        # request_packet.extend(crc_bytes)
        # # Send the request packet
        # client.socket.write(request_packet)
        # print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # # Receive the response packet
        # response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        # print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        # time.sleep(0.005)# Not sure if this is needed?

        # print("-------------------")
        # print("Assign [START] to CONT10")
        # # CONT10 at PA3_10 address (HEX) is 4209 (p.13-17)
        # request_packet = bytearray([0x01, # Slave address
        #                             0x10, # Write operation
        #                             0x42, 0x09, # PA3_10 address
        #                             0x00, 0x02, # Number of registers
        #                             0x04, # No. of data bytes = 4
        #                             0x00, 0x00, 0x00, 0x04]) # Data=4 [START]}
        # # Calculate the CRC for the packet
        # crc = calculate_crc(request_packet)
        # crc_bytes = crc.to_bytes(2, byteorder='little')
        # request_packet.extend(crc_bytes)
        # # Send the request packet
        # client.socket.write(request_packet)
        # print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # # Receive the response packet
        # response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        # print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        # time.sleep(0.005) # Not sure if this is needed?

    #    # Assign [INP] to OUT6. ･･･ PA3_56: OUT6 signal assignment =2: [INP]
    #     print("-------------------")
    #     print("Assign [INP] to OUT6")
    #     # OUT6 at PA3_56 address (hex) is 4255 (p.13-17)
    #     request_packet = bytearray([0x01, # Slave address
    #                                 0x10, # Write operation
    #                                 0x42, 0x55, # PA3_56 address
    #                                 0x00, 0x02, # Number of registers
    #                                 0x04, # No. of data bytes = 4
    #                                 0x00, 0x00, 0x00, 0x02]) # Data=2 [INP]}
    #     # Calculate the CRC for the pack et
    #     crc = calculate_crc(request_packet)
    #     crc_bytes = crc.to_bytes(2, byteorder='little')
    #     request_packet.extend(crc_bytes)
    #     # Send the request packet
    #     client.socket.write(request_packet)
    #     print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
    #     # Receive the response packet
    #     response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
    #     print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
    #     time.sleep(0.005) # Not sure if this is needed?

        print("-------------------")
        print("Set [S-ON] and [START] to off")        
        # Write “0” (OFF) to bit0 and bit1 to turn [S-ON] and [START] off
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x00, 0x00, # Address of CONT
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0b00000000]) # bit assignment. See page 13-16
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        time.sleep(0.005) # Not sure if this is needed?

        print("-------------------")
        print("Set [S-ON] to on")        
        # Write “1” (ON) to bit0 to turn [S-ON] on
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x00, 0x00, # Address of CONT
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0b00000001]) # bit assignment. See page 13-16
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        time.sleep(0.005) # Not sure if this is needed?

        # print("-------------------")
        # print("Set PA1_01 for positioning operation")
        # # Set PA1_01 to 7 (Control mode selection = Positioning operation)
        # # PA1_01 address (HEX) is 4000 (p.13-17)
        # request_packet = bytearray([0x01, # Slave address
        #                             0x10, # Write operation
        #                             0x40, 0x00, # PA1_01 address
        #                             0x00, 0x02, # Number of registers
        #                             0x04, # No. of data bytes = 4
        #                             0x00, 0x00, 0x00, 0x07]) # Data=7
        # # Calculate the CRC for the packet
        # crc = calculate_crc(request_packet)
        # crc_bytes = crc.to_bytes(2, byteorder='little')
        # request_packet.extend(crc_bytes)
        # # Send the request packet
        # client.socket.write(request_packet)
        # print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # # Receive the response packet
        # response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        # print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        # time.sleep(0.005) # Not sure if this is needed?

        print("-------------------")
        print("Send Immediate operation setting 1")        
        # Setting 1: Designation method = ABS. 
        # Immediate value position = 500000 units. 
        # Immediate value speed = 500.00 r/min
        # Query: 01 10 5100 0006 0C 00000000 0007A120 0000C350 D9EC (21 bytes)
        # Response: 01 10 5100 0006 50F7 (8 bytes)
     

        # Set number of units
        units =-5000
        hex_units = decimal_to_hex_groups(units)
        
        # Set speed
        speed = 300000
        hex_speed = decimal_to_hex_groups(speed)
        
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x51, 0x01, # Address (Immediate value status)
                                    0x00, 0x04, # Number of register
                                    0x08, # NUmber of data bytes (C=12) 
                                    # 0x00, 0x00, 0x00, 0x00, # ?
                                    # 0x00, 0x00, 0xC3, 0x50, # 50000 units
                                    hex_units[0], hex_units[1], hex_units[2], hex_units[3], 
                                    hex_speed[0], hex_speed[1], hex_speed[2], hex_speed[3]])
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        time.sleep(0.005) # Not sure if this is needed?

        print("-------------------")
        print("Set [S-ON] and [START] to on")        
        # Write “1” (ON) to bit9 and bit0 to turn [S-ON] and [START] on
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x00, 0x00, # Address of CONT
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0b00000011]) # bit assignment. See page 13-16
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        time.sleep(0.005) # Not sure if this is needed?

        print("-------------------")
        print("Set [START] to off")  
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x00, 0x00, # Address of CONT
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0b00000001]) # bit assignment. See page 13-16
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
        time.sleep(0.005) # Not sure if this is needed?

        # Loop until INP is ON (When in progress, INP is off)
        # Query: 01 03 0100 0002 C5F7 (8 bytes)
        # Response: 01 03 04 00 00 00 00 FA33 (9 bytes)
        print("-------------------")
        print("Loop until [INP] is ON")  
        # Block until opertion is complete (until INP is 1)
        # Initialize the response_packet to 0 up to byte 6
        show_feecback = True
        response_packet = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        while response_packet[6] == 0:
            request_packet = bytearray([0x01, # Slave address
                                        0x03, # Read operation
                                        0x01, 0x00, # Address of OUT
                                        0x00, 0x02]) # No. of registers
            # Calculate the CRC for the packet
            crc = calculate_crc(request_packet)
            crc_bytes = crc.to_bytes(2, byteorder='little')
            request_packet.extend(crc_bytes)
            # Send the request packet
            client.socket.write(request_packet)
            #print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
            # Receive the response packet
            response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
            #print(f"Response:\t{[hex(byte) for byte in response_packet]}")
            #print(f"Response: {hex(response_packet[2])}")
            if show_feecback:
                position = get_position(client)
                print("position =", position)

        print("Complete!")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the connection
        client.close()

if __name__ == "__main__":
    main()
