import os
import json
import time

class GripperState:
    _state_file = os.path.expanduser('~/gripper_state.json')
    _last_read_index = -1     # Track the last read index
    _cached_state = None      # Cache the last read state
    _write_index = 0          # Counter for writes

    def __init__(self):
        # Initialize the state file if it doesn't exist
        if not os.path.exists(self._state_file):
            self._write_state(0)

    @classmethod
    def _write_state(cls, value):
        """Write state to file with incremented index"""
        try:
            cls._write_index += 1  # Increment the index for each write
            with open(cls._state_file, 'w') as f:
                data = {
                    'current': value,
                    'timestamp': time.time(),
                    'index': cls._write_index
                }
                json.dump(data, f)
                print(f"State updated: {value} (index: {cls._write_index})")
        except Exception as e:
            print(f"Error writing state: {e}")

    @classmethod
    def _read_state(cls):
        """Read state from file only if index has changed"""
        try:
            # Read current state from file
            with open(cls._state_file, 'r') as f:
                data = json.load(f)
                current_index = data.get('index', 0)
                current_value = data.get('current', 0)
                
                # If index hasn't changed, return None to indicate no new data
                if current_index <= cls._last_read_index:
                    return None
                
                # Update cache and last read index
                cls._cached_state = current_value
                cls._last_read_index = current_index
                
                print(f"New state detected: {current_value} (index: {current_index})")
                return current_value
                
        except Exception as e:
            print(f"Error reading state: {e}")
            return None

    @classmethod
    def set_current(cls, current):
        """Set the current state"""
        cls._write_state(int(current))

    @classmethod
    def get_current(cls):
        """Get the current state"""
        return cls._read_state()