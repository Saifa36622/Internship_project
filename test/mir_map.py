from api import *

mir_req = MirRequestor()
# Create instance
mir = MirRequestor("http://192.168.12.20")

print("MIR100 Map Data Retrieval")
print("-" * 30)

# Option 1: Print current map data to console
print("1. Current Map Data:")
mir.print_map_data()

mir.save_map_image("current_map.png")