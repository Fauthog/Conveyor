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
    # print(f"Response:\t\t{[hex(byte) for byte in feedback_response_packet]}")
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

        print("-------------------")
        print("Assign [S-ON] to CONT9")
        # CONT9 at PA3_09 address (HEX) is 4208 (p.13-17)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x42, 0x08, # PA3_09 address
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0x01]) # Data=1 [S-ON]}
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
        time.sleep(0.005)# Not sure if this is needed?

        # print("-------------------")
        # print("Assign [ORG] to CONT11")
        # # CONT11 at PA3_11 address (HEX) is 4210 (p.13-17)
        # request_packet = bytearray([0x01, # Slave address
        #                             0x10, # Write operation
        #                             0x42, 0x10, # PA3_10 address
        #                             0x00, 0x02, # Number of registers
        #                             0x04, # No. of data bytes = 4
        #                             0x00, 0x00, 0x00, 0x05]) # data=5 [ORG]}
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
        print("Set [S-ON] to on")        
        # Write “1” (ON) to bit9 and bit0 to turn [S-ON] and [START] on
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

        print("-------------------")
        print("Set [ORG] to on")  
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x00, 0x00, # Address of CONT
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0b00000101]) # bit assignment. See page 13-16
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
                # feedback_request_packet = bytearray([0x01, # Slave address
                #                             0x03, # Read operation
                #                             0x10, 0x06, # feedback position
                #                             0x00, 0x02]) # No. of registers
                # # Calculate the CRC for the packet
                # crc = calculate_crc(feedback_request_packet)
                # crc_bytes = crc.to_bytes(2, byteorder='little')
                # feedback_request_packet.extend(crc_bytes)
                # # Send the request packet
                # client.socket.write(feedback_request_packet)
                # #print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
                # # Receive the response packet
                # feedback_response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
                # pos_feedback_hexs = [feedback_response_packet[4], feedback_response_packet[5], feedback_response_packet[6], feedback_response_packet[7]]
                # position = hex_groups_to_decimal(pos_feedback_hexs)
                position = get_position(client)
                formatted_position = format(position, ",")
                print("position =", formatted_position)

        print("Complete!")

        # for i in range(100):
        #     position = get_position(client)
        #     formatted_position = format(position, ",")
        #     print("position =", formatted_position)

        # Set to absolute position
        # PA1_02 = 1 (0=incremental, 1=absolute)

        # print("-------------------")
        # print("Set PA1_02 for absolute operation")
        # # Set PA1_02 to 1 (0=incremental, 1=absolute)
        # # PA1_02 address (HEX) is 4001 (p.13-17)
        # request_packet = bytearray([0x01, # Slave address
        #                             0x10, # Write operation
        #                             0x40, 0x01, # PA1_02 address
        #                             0x00, 0x02, # Number of registers
        #                             0x04, # No. of data bytes = 4
        #                             0x00, 0x00, 0x00, 0x00]) # Data=1 (absolute)
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



    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the connection
        client.close()

if __name__ == "__main__":
    main()
    # hexs = decimal_to_hex_groups(100)
    # print(hex(hexs[0]))
    # print(hex(hexs[1]))
    # print(hex(hexs[2]))
    # print(hex(hexs[3]))
