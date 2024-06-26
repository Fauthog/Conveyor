from pymodbus.client import ModbusSerialClient
from pymodbus.transaction import ModbusRtuFramer
import time

class Alpha5Client:
    def __init__(self, port, baudrate=38400, parity='E', stopbits=1, bytesize=8, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout = timeout
        self.client = None
        self.client_buffer_size = 40 # Should be enough for usual operation. Increase if needed

        self.show_query_response = True

        # CONT function list (P.4-96 in Alpha5 PDF)
        self.fn_list = {
            "[NONE]": 0,    # None            
            "[S-ON]": 1,    # Servo-on
            "[FWD]": 2,     # Forware command
            "[REV]": 3,     # Reverse command
            "[START]": 4,   # Start positioning
            "[ORG]": 5,     # Homing
            "[LS]": 6,      # Home position LS
            "[+OT]": 7,     # +Over travel
            "[-OT]": 8,     # -Over travel
            "[EMG]": 10,    # Force stop
            "[RST]": 11,     # Alarm reset
            #...
            "[INP]": 2
        }

        # Addresses (P.13-16 in Alpha5 PDF)
        self.address_list = {
            "COMM_CONT_SIGNAL": 0x0000,
            "COMM_OUT_SIGNAL": 0x0100,
            "FEEDBACK_SPEED": 0x1000,
            #...
            "FEEDBACK_POSITION": 0x1006,
            #...
            "PA1_01": 0x4000,
            "PA2_01": 0x4100,
            "PA3_01": 0x4200, # CONT9 is PA3_09 and its address is 4208, increase the address by 1 to get to the next CONT
            # Also define address according to CONT and OUT names instead of parameter numbers
            "CONT1": 0x4200,
            "CONT2": 0x4201,
            "CONT3": 0x4202,
            "CONT4": 0x4203,
            "CONT5": 0x4204,
            "CONT9": 0x4208,
            "CONT10": 0x4209,
            "CONT11": 0x4210,
            "OUT6": 0x4255,

            #...
            "IMMEDIATE_VAL_STATUS": 0x5100,
            "IMMEDIATE_VAL_POS": 0x5101,
            "IMMEDIATE_VAL_SPEED": 0x5102
            #...
        }

        # CONT output bits
        self.cont_bit_data = [
            {"ID":1, "byte1": 0b00000000, "byte0": 0b00000000},
            {"ID":2, "byte1": 0b00000000, "byte0": 0b00000000},
            {"ID":3, "byte1": 0b00000000, "byte0": 0b00000000},
            {"ID":4, "byte1": 0b00000000, "byte0": 0b00000000}
        ]


    def calculate_crc(self, data):
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

    def create_client(self):
        # Create a Modbus serial client for RTU
        self.client = ModbusSerialClient(method='rtu', port=self.port, baudrate=self.baudrate,
                                         parity=self.parity, stopbits=self.stopbits, bytesize=self.bytesize,
                                         timeout=self.timeout, framer=ModbusRtuFramer)
        return self.client

    def connect(self):
        if self.client is None:
            self.create_client()
        # Connect to the server
        connection = self.client.connect()
        return connection

    def set_show_query_response(self, showflag=False):
        self.show_query_response = showflag

    def hex_groups_to_dec(self, hex_groups):
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

    def dec_to_hex_groups(self, number):
        if not (-2**31 <= number < 2**31):
            raise ValueError("Number out of range for 32-bit signed integer.")
        
        # Convert the number to a 32-bit two's complement hexadecimal
        if number < 0:
            number = (1 << 32) + number

        # Convert the decimal number to hexadecimal
        hex_number = f"{number:08X}"

        # Format as a group of 4 hex numbers
        hex_groups = [int(hex_number[i:i+2], 16) for i in range(0, 8, 2)]
        
        return hex_groups

    def get_feedback_position(self, id):
        if self.client is not None:
            feedback_request_packet = bytearray([id, # Slave address
                                                0x03, # Read operation
                                                0x10, 0x06, # feedback position
                                                0x00, 0x02]) # No. of registers
            # Calculate the CRC for the packet
            crc = self.calculate_crc(feedback_request_packet)
            crc_bytes = crc.to_bytes(2, byteorder='little')
            feedback_request_packet.extend(crc_bytes)
            # Send the request packet
            self.client.socket.write(feedback_request_packet)
            #print(f"Query:\t\t{[hex(byte) for byte in request_packet]}")
            # Receive the response packet
            feedback_response_packet = self.client.socket.read(self.client_buffer_size)
            pos_feedback_hexs = [feedback_response_packet[3], feedback_response_packet[4], feedback_response_packet[5], feedback_response_packet[6]]
            position = self.hex_groups_to_dec(pos_feedback_hexs)
            
            return position

    def signal_assignment(self, id, signal, function):
        # Get the CONT address, return if not valid
        if signal not in self.address_list:
            print("The given CONT or OUT name is invalid")
            return False
        # Get the address if given name is valid
        signal_address = self.address_list.get(signal)
        # Convert cont_no to byte array
        byte_array_signal_address = signal_address.to_bytes(2, byteorder='big')

        # Get fn_no, return if not valid
        if function not in self.fn_list:
            print("The given function is invalid")
            return False
        # Get the function number if given function is valid
        function_no = self.fn_list.get(function)

        # Construct query packet (p.13-17)
        query_packet = bytearray([id, # Slave address
                                  0x10, # Write operation
                                  byte_array_signal_address[0], byte_array_signal_address[1], # PA3_x address
                                  0x00, 0x02, # Number of registers
                                  0x04, # No. of data bytes = 4
                                  0x00, 0x00, 0x00, function_no]) # Data=fn_no
        # Calculate the CRC for the packet
        crc = self.calculate_crc(query_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        query_packet.extend(crc_bytes)

        # Try sending the request packet
        try:
            self.client.socket.write(query_packet)
            if self.show_query_response:
                print(f"Query:\t\t{[hex(byte) for byte in query_packet]}")
            # Receive the response packet
            response_packet = self.client.socket.read(self.client_buffer_size)  # Adjust buffer size if necessary
            if self.show_query_response:
                print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
            time.sleep(0.005)# Not sure if this is needed?
            return True
        except Exception as e:
            print(f"An error occurred while writing to the socket: {e}")
            return None

    def manipulate_virtualcont_bits(self, id, bit_index, state):
        # Ensure the index is within range
        if not 0 <= bit_index < 16:
            raise ValueError("Index must be between 0 and 15 inclusive")
            return False

        # Retrieve the byte data
        for item in self.cont_bit_data:
            if item["ID"] == id:
                high_byte = item["byte1"]
                low_byte = item["byte0"]

                # Combine the two 8-bit numbers into a single 16-bit number
                binary_num = (high_byte << 8) | low_byte

                if state == 'ON':
                    binary_num |= (1 << bit_index)  # Turn the bit on
                elif state == 'OFF':
                    binary_num &= ~(1 << bit_index)  # Turn the bit off
                else:
                    raise ValueError("State must be 'ON' or 'OFF'")
                    return False

                # Split the 16-bit number back into two 8-bit numbers
                high_byte = (binary_num >> 8) & 0xFF
                low_byte = binary_num & 0xFF

                item["byte1"] = high_byte
                item["byte0"] = low_byte
                # print(f"Byte1: {format(high_byte, '08b')}, Byte0: {format(low_byte, '08b')}")

                # Construct query packet (p.13-17)
                query_packet = bytearray([id, # Slave address
                                        0x10, # Write operation
                                        0x00, 0x00, # CONT address
                                        0x00, 0x02, # Number of registers
                                        0x04, # No. of data bytes = 4
                                        0x00, 0x00, high_byte, low_byte])
                # Calculate the CRC for the packet
                crc = self.calculate_crc(query_packet)
                crc_bytes = crc.to_bytes(2, byteorder='little')
                query_packet.extend(crc_bytes)

                # Try sending the request packet
                try:
                    self.client.socket.write(query_packet)
                    if self.show_query_response:
                        print(f"Query:\t\t{[hex(byte) for byte in query_packet]}")
                    # Receive the response packet
                    response_packet = self.client.socket.read(self.client_buffer_size)  # Adjust buffer size if necessary
                    if self.show_query_response:
                        print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
                    time.sleep(0.005)# Not sure if this is needed?
                    return True
                except Exception as e:
                    print(f"An error occurred while writing to the socket: {e}")
                    return None

    def send_immediate_operation_setting(self, id, units, speed):
        hex_units = self.dec_to_hex_groups(units)
        hex_speed = self.dec_to_hex_groups(speed)

        # Construct query packet
        query_packet = bytearray([id, # Slave address
                                0x10, # Write operation
                                0x51, 0x01, # Address for immdiate value status
                                0x00, 0x04, # Number of registers
                                0x08, # No. of data bytes
                                hex_units[0], hex_units[1], hex_units[2], hex_units[3], 
                                hex_speed[0], hex_speed[1], hex_speed[2], hex_speed[3]])

        # Calculate the CRC for the packet
        crc = self.calculate_crc(query_packet)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        query_packet.extend(crc_bytes)

        # Try sending the request packet
        try:
            self.client.socket.write(query_packet)
            if self.show_query_response:
                print(f"Query:\t\t{[hex(byte) for byte in query_packet]}")
            # Receive the response packet
            response_packet = self.client.socket.read(self.client_buffer_size)  # Adjust buffer size if necessary
            if self.show_query_response:
                print(f"Response:\t\t{[hex(byte) for byte in response_packet]}")
            time.sleep(0.005)# Not sure if this is needed?
            return True
        except Exception as e:
            print(f"An error occurred while writing to the socket: {e}")
            return None

    def wait_operation_complete(self, id):
        # Block until opertion is complete (until INP is 1)
        # Initialize the response_packet to 0 up to byte 6
        show_feedback_position = True
        oc_response_packet = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        while oc_response_packet[6] == 0:
            oc_request_packet = bytearray([id, # Slave address
                                        0x03, # Read operation
                                        0x01, 0x00, # Address of OUT
                                        0x00, 0x02]) # No. of registers
            # Calculate the CRC for the packet
            crc = self.calculate_crc(oc_request_packet)
            crc_bytes = crc.to_bytes(2, byteorder='little')
            oc_request_packet.extend(crc_bytes)
            # Send the request packet
            self.client.socket.write(oc_request_packet)
            #print(f"Query:\t\t{[hex(byte) for byte in oc_request_packet]}")
            # Receive the response packet
            oc_response_packet = self.client.socket.read(self.client_buffer_size)  # Adjust buffer size if necessary
            #print(f"Response:\t{[hex(byte) for byte in oc_response_packet]}")
            # time.sleep(0.005)# Not sure if this is needed?

            if show_feedback_position:
                position = self.get_feedback_position(id=id)
                # print("feedback position =", position)
