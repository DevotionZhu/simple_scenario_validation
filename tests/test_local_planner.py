import carla
from utils.local_planner import LocalPlanner
from utils.in_memory_map import InMemoryMap

ip = "10.51.6.52"
port = 2006

client = carla.Client(ip, port)
world = client.get_world()
world_map = world.get_map()

local_map = InMemoryMap(world_map)
local_map.setup()

vehicle = world.get_actors().filter('vehicle.*')[0]

local_planner = LocalPlanner(vehicle, local_map, target_velocity=20)

while True:
    throttle, steer = local_planner.update()
    control = carla.VehicleControl()
    control.throttle = throttle
    control.steer = steer
    vehicle.apply_control(control)
    print('Throttle : ', throttle, 'Steer : ', steer)
