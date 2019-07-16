from utils.in_memory_map import InMemoryMap
import carla

ip = "10.51.6.52"
port = 2006

client = carla.Client(ip, port)
world = client.get_world()
world_map = world.get_map()

print("Setting up local map ...")
local_map = InMemoryMap(world_map)
local_map.setup()

count = 0
for swp in local_map.get_dense_topology():
    if len(swp.get_next_waypoints()) == 0:
        count += 1

print("Number of loose ends : ", count)