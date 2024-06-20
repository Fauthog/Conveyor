#from pymodbus.client.sync import ModbusSerialClient
from pymodbus.client import ModbusSerialClient
from pymodbus.transaction import ModbusRtuFramer
import time



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
    port = 'COM4'          # Change this to the correct COM port on your machine
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

        print("-------------------")
        print("Assign [START] to CONT10")
        # CONT10 at PA3_10 address (HEX) is 4209 (p.13-17)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x42, 0x09, # PA3_10 address
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0x04]) # Data=4 [START]}
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

        print("-------------------")
        print("Set PA1_01 for positioning operation")
        # Set PA1_01 to 7 (Control mode selection = Positioning operation)
        # PA1_01 address (HEX) is 4000 (p.13-17)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x40, 0x00, # PA1_01 address
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0x07]) # Data=7
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
        print("Send Immediate operation setting 1")        
        # Setting 1: Designation method = ABS. 
        # Immediate value position = 500000 units. 
        # Immediate value speed = 500.00 r/min
        # Query: 01 10 5100 0006 0C 00000000 0007A120 0000C350 D9EC (21 bytes)
        # Response: 01 10 5100 0006 50F7 (8 bytes)
     
        
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x51, 0x01, # Address (Immediate value status)
                                    0x00, 0x04, # Number of register
                                    0x08, # NUmber of data bytes (C=12) 
                                    # 0x00, 0x00, 0x00, 0x00, # ?
                                    # 0x00, 0x00, 0xC3, 0x50, # 50000 units
                                    0x00, 0x01, 0x86, 0xA0, # 500000 units
                                    0x00, 0x03, 0x0D, 0x40]) # 500.00 RPM
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
        
    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the connection
        client.close()

if __name__ == "__main__":
    main()
