#!usr/bin/env python

import requests
from PIL import Image
import io
import base64
import json
import os
class MirRequestor:
    def __init__(self, mir_ip="http://192.168.12.20", api_version="v2.0.0"):
        self.header = {"Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
        self.url = mir_ip + "/api/" + api_version + "/"

    def abort_all_missions(self):
        end_point = "mission_queue"
        response = requests.delete(self.url + end_point, headers=self.header)
        return response

    def is_mission_done(self, id):
        end_point = "mission_queue/" + str(id)
        response = requests.get(self.url + end_point, headers=self.header).json()
        if (response["state"] == "Done"):
            return True
        else:
            return False

    def dock_pick_point(self):
        end_point = "mission_queue"
        data = {
            "mission_id": "mirconst-guid-0000-0005-actionlist00",  # "Dock"
            "parameters": [
                {
                    "id": "Marker",
                    "value": "34915c39-1700-11eb-84ba-0001299df266"
                }
            ]
        }
        response = requests.post(self.url + end_point, headers=self.header, json=data)
        return response.json()

    def dock_place_point(self):
        end_point = "mission_queue"
        data = {
            "mission_id": "mirconst-guid-0000-0005-actionlist00",
            "parameters": [
                {
                    "id": "Marker",
                    "value": "3041b829-1942-11eb-827e-0001299df266"
                }
            ]
        }
        response = requests.post(self.url + end_point, headers=self.header, json=data)
        return response.json()

    def go_standby(self):
        end_point = "mission_queue"
        data = {
            "mission_id": "mirconst-guid-0000-0001-actionlist00",  # "Move"
            "parameters": [
                {
                    "id": "Position",
                    "value": "3586a1d7-e5a6-11ea-ac58-0001299df266"
                }
            ]
        }
        response = requests.post(self.url + end_point, headers=self.header, json=data)
        return response.json()

    def relative_move(self, x=0, y=0, orientation=0):
        end_point = "mission_queue"
        data = {
            "mission_id": "fcf41a69-0e2a-11eb-a64b-0001299df266",  # "RelativeMove"
            "parameters": [
                {
                    "id": "x",
                    "value": x
                },
                {
                    "id": "y",
                    "value": y
                },
                {
                    "id": "orientation",
                    "value": orientation
                }
            ]
        }
        response = requests.post(self.url + end_point, headers=self.header, json=data)
        return response.json()

    def robot_pause(self):
        pass

    def robot_play(self):
        pass

    def get_status(self):
        end_point = "status"
        response = requests.get(self.url + end_point, headers=self.header)
        return response.json()

    def get_maps(self):
        """Get list of all available maps"""
        try:
            end_point = "maps"
            response = requests.get(self.url + end_point, headers=self.header)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error getting maps: {e}")
            return []

    def get_active_map_id(self):
        """Get the currently active map ID from robot status"""
        try:
            status = self.get_status()
            return status.get('map_id', None)
        except Exception as e:
            print(f"Error getting active map ID: {e}")
            return None

    def get_map_info(self, map_id=None):
        """Get information about a specific map or the active map"""
        try:
            if map_id is None:
                map_id = self.get_active_map_id()
            
            if map_id is None:
                return {"error": "No active map found"}
            
            end_point = f"maps/{map_id}"
            response = requests.get(self.url + end_point, headers=self.header)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error getting map info: {e}")
            return {"error": str(e)}

    def get_map_image_raw(self, map_id=None):
        """Download the raw map image data"""
        try:
            if map_id is None:
                map_id = self.get_active_map_id()
            
            if map_id is None:
                return None
            
            end_point = f"maps/{map_id}/map"
            response = requests.get(self.url + end_point, headers=self.header)
            response.raise_for_status()
            return response.content
        except requests.exceptions.RequestException as e:
            print(f"Error getting map image: {e}")
            return None

    def save_map_image(self, filename="current_map.png", map_id=None):
        """Save map image to file"""
        try:
            image_data = self.get_map_image_raw(map_id)
            if image_data:
                with open(filename, 'wb') as f:
                    f.write(image_data)
                print(f"Map image saved as: {filename}")
                return True
            else:
                print("No image data to save")
                return False
        except Exception as e:
            print(f"Error saving map image: {e}")
            return False

    def get_positions_on_map(self, map_id=None):
        """Get all positions (markers) on the map"""
        try:
            if map_id is None:
                map_id = self.get_active_map_id()
            
            if map_id is None:
                return []
            
            end_point = "positions"
            response = requests.get(self.url + end_point, headers=self.header)
            response.raise_for_status()
            all_positions = response.json()
            
            # Filter positions for the specific map
            map_positions = [pos for pos in all_positions if pos.get('map_id') == map_id]
            return map_positions
        except requests.exceptions.RequestException as e:
            print(f"Error getting positions: {e}")
            return []

    def get_current_map_data(self):
        """Get comprehensive data about the current active map"""
        try:
            # Get basic info
            map_info = self.get_map_info()
            if 'error' in map_info:
                return map_info
            
            # Get positions
            positions = self.get_positions_on_map()
            
            # Get robot status
            status = self.get_status()
            
            # Compile comprehensive data
            map_data = {
                'timestamp': self.get_current_timestamp(),
                'map_info': map_info,
                'positions': positions,
                'position_count': len(positions),
                'robot_status': {
                    'position': status.get('position', {}),
                    'state': status.get('state_text', 'Unknown'),
                    'mode': status.get('mode_text', 'Unknown'),
                    'battery': status.get('battery_percentage', 0),
                    'speed': status.get('velocity', {})
                }
            }
            
            return map_data
            
        except Exception as e:
            print(f"Error getting current map data: {e}")
            return {"error": str(e)}

    def get_current_timestamp(self):
        """Get current timestamp string"""
        import datetime
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def print_map_data(self):
        """Print formatted map data to console"""
        data = self.get_current_map_data()
        
        if 'error' in data:
            print(f"Error: {data['error']}")
            return
        
        print("\n" + "="*60)
        print("MIR100 CURRENT MAP DATA")
        print("="*60)
        print(f"Timestamp: {data['timestamp']}")
        
        # Map Information
        map_info = data['map_info']
        print(f"\nMAP INFORMATION:")
        print(f"  Name: {map_info.get('name', 'Unknown')}")
        print(f"  ID: {map_info.get('guid', 'Unknown')}")
        print(f"  Resolution: {map_info.get('resolution', 'Unknown')} m/pixel")
        print(f"  Created by: {map_info.get('created_by_name', 'Unknown')}")
        print(f"  Session ID: {map_info.get('session_id', 'Unknown')}")
        
        # Robot Status
        robot = data['robot_status']
        print(f"\nROBOT STATUS:")
        print(f"  State: {robot['state']}")
        print(f"  Mode: {robot['mode']}")
        print(f"  Battery: {robot['battery']}%")
        
        # Robot Position
        pos = robot['position']
        print(f"  Position: X={pos.get('x', 0):.3f}m, Y={pos.get('y', 0):.3f}m")
        print(f"  Orientation: {pos.get('orientation', 0):.3f} rad ({pos.get('orientation', 0) * 57.2958:.1f}°)")
        
        # Velocity
        vel = robot.get('speed', {})
        if vel:
            print(f"  Velocity: X={vel.get('vx', 0):.3f}m/s, Y={vel.get('vy', 0):.3f}m/s")
            print(f"  Angular Velocity: {vel.get('omega', 0):.3f} rad/s")
        
        # Positions on Map
        print(f"\nPOSITIONS ON MAP: {data['position_count']} total")
        if data['positions']:
            print("  Top 10 positions:")
            for i, pos in enumerate(data['positions'][:10], 1):
                pos_type = pos.get('type_id', 0)
                type_name = self.get_position_type_name(pos_type)
                print(f"    {i:2d}. {pos.get('name', 'Unnamed'):20s} [{type_name:10s}] "
                      f"X={pos.get('pos_x', 0):7.3f}m Y={pos.get('pos_y', 0):7.3f}m")
            
            if len(data['positions']) > 10:
                print(f"    ... and {len(data['positions']) - 10} more positions")
        
        print("="*60)

    def get_position_type_name(self, type_id):
        """Convert position type ID to readable name"""
        type_names = {
            0: "Position",
            1: "Charging",
            2: "Docking", 
            3: "VL marker",
            4: "Area",
            5: "Shelf",
            6: "Cart"
        }
        return type_names.get(type_id, f"Type_{type_id}")

    def save_map_data_json(self, filename="map_data.json"):
        """Save current map data to JSON file"""
        try:
            data = self.get_current_map_data()
            if 'error' not in data:
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2)
                print(f"Map data saved to: {filename}")
                return True
            else:
                print(f"Cannot save data: {data['error']}")
                return False
        except Exception as e:
            print(f"Error saving JSON: {e}")
            return False

    def export_positions_csv(self, filename="positions.csv"):
        """Export positions to CSV file"""
        try:
            data = self.get_current_map_data()
            if 'error' in data:
                print(f"Cannot export positions: {data['error']}")
                return False
            
            positions = data['positions']
            if not positions:
                print("No positions to export")
                return False
            
            with open(filename, 'w') as f:
                # Header
                f.write("name,type,x,y,orientation,map_id\n")
                
                # Data
                for pos in positions:
                    name = pos.get('name', '').replace(',', ';')  # Handle commas in names
                    type_name = self.get_position_type_name(pos.get('type_id', 0))
                    x = pos.get('pos_x', 0)
                    y = pos.get('pos_y', 0)
                    orientation = pos.get('orientation', 0)
                    map_id = pos.get('map_id', '')
                    
                    f.write(f"{name},{type_name},{x:.3f},{y:.3f},{orientation:.3f},{map_id}\n")
            
            print(f"Positions exported to: {filename}")
            return True
            
        except Exception as e:
            print(f"Error exporting CSV: {e}")
            return False

    def create_simple_map_report(self, output_dir="map_report"):
        """Create a complete map report with all data and image"""
        try:
            # Create output directory
            os.makedirs(output_dir, exist_ok=True)
            
            # Get map data
            data = self.get_current_map_data()
            if 'error' in data:
                print(f"Cannot create report: {data['error']}")
                return False
            
            map_name = data['map_info'].get('name', 'unknown_map')
            safe_name = "".join(c for c in map_name if c.isalnum() or c in (' ', '-', '_')).rstrip()
            
            # Save map image
            image_file = os.path.join(output_dir, f"{safe_name}_map.png")
            self.save_map_image(image_file)
            
            # Save JSON data
            json_file = os.path.join(output_dir, f"{safe_name}_data.json")
            with open(json_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            # Save positions CSV
            csv_file = os.path.join(output_dir, f"{safe_name}_positions.csv")
            self.export_positions_csv(csv_file)
            
            # Create text summary
            summary_file = os.path.join(output_dir, f"{safe_name}_summary.txt")
            with open(summary_file, 'w') as f:
                f.write("MIR100 MAP REPORT\n")
                f.write("="*50 + "\n")
                f.write(f"Generated: {data['timestamp']}\n")
                f.write(f"Map: {map_name}\n")
                f.write(f"Positions: {data['position_count']}\n")
                f.write(f"Robot State: {data['robot_status']['state']}\n")
                f.write(f"Battery: {data['robot_status']['battery']}%\n")
                f.write("\nFiles in this report:\n")
                f.write(f"- {safe_name}_map.png (map image)\n")
                f.write(f"- {safe_name}_data.json (complete data)\n")
                f.write(f"- {safe_name}_positions.csv (position list)\n")
                f.write(f"- {safe_name}_summary.txt (this file)\n")
            
            print(f"\nComplete map report created in: {output_dir}")
            print(f"Map: {map_name}")
            print(f"Files: 4 files generated")
            return True
            
        except Exception as e:
            print(f"Error creating report: {e}")
            return False

ip = "http://192.168.12.20"
mir = MirRequestor(ip)




