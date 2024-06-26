from alpha5 import Alpha5Client
import time

# Main function
def main():
    # Create an instance of the Alpha5 client
    alpha5 = Alpha5Client("COM12")
    alpha5.create_client()
    alpha5.connect()
    alpha5.set_show_query_response(False)
    

    # Assign [S-ON] to CONT9
    #alpha5.signal_assignment(id=1, signal="CONT9", function="[S-ON]")

    # Assign [START] to CONT10
    #alpha5.signal_assignment(id=1, signal="CONT10", function="[START]")

    # Assign [INP] to OUT6
    #alpha5.signal_assignment(id=1, signal="OUT6", function="[INP]")

    print('Set [S-ON] to ON')
    alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9
    
    # print('Issue homing command')
    # alpha5.manipulate_virtualcont_bits(id=1, bit_index=2, state='ON') # [ORG]->CONT11

    # print("Homing...")
    # print('Wait for the operatio to complete')
    # alpha5.wait_operation_complete(id=1)
    # print('done!')

    print('Send immediate operation setting')
    alpha5.send_immediate_operation_setting(id=1, units=-13777, speed=5000)
    print('Set [START] to ON')
    alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='ON') # [START]->CONT10
    print('Set [START] to OFF')
    alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='OFF') # [START]->CONT10


    print('Wait for the operatio to complete')
    alpha5.wait_operation_complete(id=1)


    print('Set [S-ON] to OFF')
    alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
 
 

if __name__ == "__main__":
    main()
    # Example usage:
