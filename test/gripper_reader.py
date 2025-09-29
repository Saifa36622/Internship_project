#!/usr/bin/python3
from gripper_state import GripperState
import time

# This file reads the gripper state
def main():
    gripper = GripperState()  # Initialize the state file
    print("Gripper State Reader running...")
    last_printed_state = None
    
    try:
        while True:
            current = GripperState.get_current()
            if current is not None:  # Only process if we got new data
                last_printed_state = current
            elif last_printed_state is not None:
                print(f"Waiting... (Last state: {last_printed_state})")
            else:
                print("Waiting for initial state...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping gripper reader...")

if __name__ == '__main__':
    main()