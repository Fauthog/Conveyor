#from pymodbus.client.sync import ModbusSerialClient
from pymodbus.client import ModbusSerialClient
from pymodbus.transaction import ModbusRtuFramer

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
    baudrate = 9600        # Set the baud rate for the serial connection
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

        # Operation sequence taken from an example in Alpha5 manual page 13-36

        print("-------------------")
        print("Set CONT1 to turn S-ON on")
        # CONT1 at PA3_01 address (HEX) is 4200 (p.13-17)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x42, 0x00, # PA3_01 address
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0x01]) # Data=1
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:    {[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response: {[hex(byte) for byte in response_packet]}")

        print("-------------------")
        print("Set PA1_01 for posinioning operation")
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
        print(f"Query:    {[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response: {[hex(byte) for byte in response_packet]}")

        print("-------------------")
        print("Set PA3_09 to START")
        # Set PA3_09 to 4 (CONT9 signal assignment = start)
        # PA3_09 address (HEX) is 4208 (p.13-17)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x42, 0x08, # PA3_09 address
                                    0x00, 0x02, # Number of registers
                                    0x04, # No. of data bytes = 4
                                    0x00, 0x00, 0x00, 0x04]) # Data=4
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:    {[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response: {[hex(byte) for byte in response_packet]}")

        print("-------------------")
        print("Send Immediate operation setting 1")        
        # Setting 1: Designation method = ABS. 
        # Immediate value position = 500000 units. 
        # Immediate value speed = 500.00 r/min
        # Query: 01 10 5100 0006 0C 00000000 0007A120 0000C350 D9EC (21 bytes)
        # Response: 01 10 5100 0006 50F7 (8 bytes)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x51, 0x00, # Address (Immediate value status)
                                    0x00, 0x06, # Number of register
                                    0x0C, # NUmber of data bytes (C=12) 
                                    0x00, 0x00, 0x00, 0x00, # ?
                                    0x00, 0x07, 0xA1, 0x20, # 500000 units
                                    0x00, 0x00, 0xC3, 0x50]) # 500.00 RPM
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:    {[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response: {[hex(byte) for byte in response_packet]}")

        print("-------------------")
        print("Send START")        
        # Write “1” (ON) to [START] to start positioning operation. (Immediate value data operation 1
        # based on immediate value data setting 1 starts.)
        # Query: 01 10 00 00 00 02 04 00 00 00 01 326F (13 bytes)
        # Response: 01 10 0000 0002 41C8 (8 bytes)
        request_packet = bytearray([0x01, # Slave address
                                    0x10, # Write operation
                                    0x00, 0x00, # Address of Communication CONT signal?
                                    0x00, 0x02, # No. of registers
                                    0x04, # No. of data
                                    0x00, 0x00, 0x00, 0x01])
        # Calculate the CRC for the packet
        crc = calculate_crc(request_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        request_packet.extend(crc_bytes)
        # Send the request packet
        client.socket.write(request_packet)
        print(f"Query:    {[hex(byte) for byte in request_packet]}")
        # Receive the response packet
        response_packet = client.socket.read(1024)  # Adjust buffer size if necessary
        print(f"Response: {[hex(byte) for byte in response_packet]}")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the connection
        client.close()

if __name__ == "__main__":
    main()
