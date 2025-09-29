#!/usr/bin/python3
from gripper_state import GripperState
import time
import random

# This file simulates setting the gripper state
def main():
    gripper = GripperState()  # Initialize the state file
    print("Gripper State Setter running...")
    try:
        while True:
            # Simulate changing gripper state
            new_state = random.randint(0, 100)
            GripperState.set_current(new_state)
            print(f"Set gripper state to: {new_state}")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nStopping gripper setter...")

if __name__ == '__main__':
    main()