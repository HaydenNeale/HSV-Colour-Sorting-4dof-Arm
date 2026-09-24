import cv2 
import time 
import serial 
import serial.tools.list_ports 
import IKtest as ik 
import hsvtest as hsv 
 
# serial pico control  
Port = 'COM10' # com for the pico  
baud = 115200 
# Initialize serial connection 
def connect_serial(port, baud_rate): 
    try: 
        ser = serial.Serial(port, baud_rate, timeout=1) 
        print(f"Connected to {port} at {baud_rate} baud.") 
        return ser 
    except serial.SerialException as e: 
        print(f"Error connecting to {port}: {e}") 
        return None 

def send_angles(ser, angles): 
    if ser is None: 
            print(f"[PC SIMULATION] Moving to {angles}") 
            time.sleep(1) 
            return 
    elif not ser.is_open:
            print("Serial connection is not open.")
            return 
 
# Convert angles array to a CSV string format: "0,45,60,90\n" 
    command_str = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]}\n" 
    ser.write(command_str.encode('utf-8')) 
    print(f"[PC -> PICO] Sent angles: {angles}") 
 
    # Wait for completion handshake from Pico 
    while True: 
        if ser.in_waiting > 0: 
            response = ser.readline().decode('utf-8').strip() 
            if response == "DONE": 
                print("[PICO -> PC] Target reached successfully.\n") 
                break 
            elif response.startswith("LOG:"): 
                print(f"[PICO LOG] {response}") 
        time.sleep(0.01) 
 
 
def main(): 
 
# variables for block positions and offsets 
    R , G , B = [0,0,0,0], [0,0,0,0], [0,0,0,0] # arrays for joint angles at each color detection Claw Elbo Shoulder Base 
    Roff , Goff , Boff = [100,0,0,0], [100,0,0,0], [100,0,0,0] # arrays for joint offsets at each color detection Claw Elbo Shoulder Base for gripping the block 
    Drop = [100,90,0,0] # array for drop angles of blocks
 
# Flags to track reachable IK solutions 
    valid_targets = {'Red': False, 'Green': False, 'Blue': False} 
 
# code start 
    # Load camera calibration 
    H, table_z_cm = hsv.load_calibration() 
 
    # Load test image 
    img = cv2.imread(hsv.IMAGE_PATH) 
 
    if img is None: 
        raise FileNotFoundError(f"Image not found: {hsv.IMAGE_PATH}") 
 
    # Find blocks in the image 
    blocks = hsv.find_blocks(img, H, table_z_cm) 
 
    # Store block positions 
    for color, (x, y, z) in blocks.items(): 
        match color: 
            case 'Red': 
                R = [0, x, y, z] 
                Roff = [0, x, y, z + 4] # Offset for gripping 
            case 'Green': 
                G = [0, x, y, z] 
                Goff = [0, x, y, z + 4]  
            case 'Blue': 
                B = [0, x, y, z] 
                Boff = [0, x, y, z + 4]  
    # IK maths for location of blocks 
    try: 
        R[3], R[2], R[1] = ik.solver(R[1], R[2], R[3]) 
        G[3], G[2], G[1] = ik.solver(G[1], G[2], G[3]) 
        B[3], B[2], B[1] = ik.solver(B[1], B[2], B[3]) 
 
        Roff[3], Roff[2], Roff[1] = ik.solver(Roff[1], Roff[2], Roff[3]) 
        Goff[3], Goff[2], Goff[1] = ik.solver(Goff[1], Goff[2], Goff[3]) 
        Boff[3], Boff[2], Boff[1] = ik.solver(Boff[1], Boff[2], Boff[3]) 
    except Exception as e: 
        print(f"Error occurred while solving IK: {e}") 
 
    print("Red IK:", R) 
    print("Green IK:", G) 
    print("Blue IK:", B) 
 
    print("Red Offset IK:", Roff) 
    print("Green Offset IK:", Goff) 
    print("Blue Offset IK:", Boff) 
 
#fk Test for angles and offsets 
    Rfk = ik.fk(R[1], R[2], R[3]) 
    Gfk = ik.fk(G[1], G[2], G[3]) 
    Bfk = ik.fk(B[1], B[2], B[3]) 
 
    print("Red FK:", Rfk) 
    print("Green FK:", Gfk) 
    print("Blue FK:", Bfk) 
 
# logic fowarding to the pico 
# Order Roff R Drop Goff G Drop Boff B Drop  
# also add error handling for unreachable targets  
    ser = connect_serial(Port, baud) 

# code run order 
    sequence = [ 
        ('Red', Roff, R,Drop), 
        ('Green', Goff, G,Drop), 
        ('Blue', Boff, B,Drop) 
    ] 

#starting robot sequence 
    for color, offset_angles, target_angles in sequence: 
      
        # Check that the IK angles were successfully calculated
        if all(isinstance(angle, (int, float)) for angle in offset_angles[1:]) and \
           all(isinstance(angle, (int, float)) for angle in target_angles[1:]):
            valid_targets[color] = True
        else:
            valid_targets[color] = False

        if valid_targets[color]: 
            print(f"Moving to {color} block offset angles: {offset_angles}") 
            send_angles(ser, offset_angles) 
            time.sleep(1)  # Wait for the robot to reach the position 
 
            print(f"Moving to {color} block target angles: {target_angles}") 
            send_angles(ser, target_angles) 
            time.sleep(1)  # Wait for the robot to reach the position 
 
        else: 
            print(f"Skipping {color} block because valid IK angles were not generated.") 

main()