from gripper_state import GripperState
import time
import os
import json
import fcntl
from datetime import datetime

class GripperStateLogger:
    def __init__(self):
        self.log_file = os.path.join(os.path.expanduser('~'), 'gripper_state.json')
        self._initialize_log_file()
        
    def _initialize_log_file(self):
        """Initialize or clear the log file with proper locking"""
        try:
            with open(self.log_file, 'w') as f:
                # Acquire exclusive lock for writing
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                try:
                    json.dump([], f)
                finally:
                    # Release lock in finally block to ensure it's always released
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
            print(f"Initialized empty log file at: {self.log_file}")
        except Exception as e:
            print(f"Error initializing log file: {e}")
            raise

    def log_state(self, state):
        """Log the gripper state with timestamp using file locking"""
        try:
            # First read with shared lock
            with open(self.log_file, 'r') as f:
                fcntl.flock(f.fileno(), fcntl.LOCK_SH)
                try:
                    data = json.load(f)
                finally:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
            
            # Add new state
            data.append({
                'timestamp': datetime.now().isoformat(),
                'state': state
            })
            
            # Write with exclusive lock
            with open(self.log_file, 'w') as f:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                try:
                    json.dump(data, f, indent=2)
                finally:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
            
        except BlockingIOError:
            print("File is locked by another process, skipping this update")
        except Exception as e:
            print(f"Error logging state: {e}")

def main():
    try:
        logger = GripperStateLogger()
        print("Starting gripper state monitoring...")
        print(f"Logging to: {logger.log_file}")
        
        while True:
            current = GripperState.get_current()
            logger.log_state(current)
            print(f"Logged Gripper State: {current}")
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nGripper state monitoring stopped.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()