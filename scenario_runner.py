import math
import random

import numpy as np
import carla
from carla import ColorConverter as cc
import pygame
import py_trees

from scenarios.lane_cut_in import LaneCutIn

ip = "10.51.6.52"
port = 2006

client = carla.Client(ip, port)
world = client.get_world()
world_map = world.get_map()

pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)

vehicle = None
traffic_vehicle = None
camera_sensor = None


class surface_holder:
    def __init__(self):
        self.surface = None
camera_holder = surface_holder()
lidar_holder = surface_holder()

scenario_data = {
    "location1": {
        "ego_vehicle": {
            "model": "vehicle.tesla.model3",
            "position": [387.8, -32.5, 5]
        },
        "traffic_vehicle": {
            "model": "vehicle.tesla.model3",
            "position": [391.3, -63.4, 5]
        }
    }
}


def spawn_vehicle(data):
    model = data["model"]
    loc = data["position"]
    wp = world_map.get_waypoint(carla.Location(loc[0], loc[1], loc[2]))
    blueprint = random.choice(
        world.get_blueprint_library().filter(model))
    return world.try_spawn_actor(blueprint, wp.transform)


def print_tree(tree):
    print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))


def magnitude(vector):
    return math.sqrt(vector.x**2+vector.y**2+vector.z**2)

try:

    # Spawn vehicle

    vehicle = spawn_vehicle(
        scenario_data["location1"]["ego_vehicle"]
    )
    traffic_vehicle = spawn_vehicle(
        scenario_data["location1"]["traffic_vehicle"]
    )
    if vehicle is None or traffic_vehicle is None:
        print("Couldn't spawn vehicles !")
        exit()
    # Create behaviour tree

    scenario = LaneCutIn(world, "LaneCutInDemo", vehicle, [traffic_vehicle])
    root = scenario.create_tree()
    behaviour_tree = py_trees.trees.BehaviourTree(root=root)
    behaviour_tree.tick()

    # Set sensor

    bp_library = world.get_blueprint_library()
    bp = bp_library.find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(1920//2))
    bp.set_attribute('image_size_y', str(1080//2))

    camera_transform = carla.Transform(
        carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera_sensor = world.spawn_actor(
        bp,
        camera_transform,
        attach_to=vehicle)

    bp = bp_library.find('sensor.lidar.ray_cast')
    bp.set_attribute('range', '5000')
    lidar_transform = carla.Transform(
        carla.Location(z=3), carla.Rotation(pitch=0))
    lidar_sensor = world.spawn_actor(
        bp,
        lidar_transform,
        attach_to=vehicle)

    # Camera callback funtion

    def parse_image(image):
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        camera_holder.surface = surface

    camera_sensor.listen(lambda image: parse_image(image))

    # Lidar callback
    def parse_lidar(image):
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
        lidar_data = np.array(points[:, :2])
        lidar_data *= 540 / 100.0
        lidar_data += (0.5 * 540, 0.5 * 540)
        lidar_data = np.fabs(lidar_data)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (540, 540, 3)
        lidar_img = np.zeros(lidar_img_size)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        lidar_holder.surface = pygame.surfarray.make_surface(lidar_img)

    lidar_sensor.listen(lambda image: parse_lidar(image))

    # Project camera image on pygame

    display = pygame.display.set_mode(
        (1920//2 + 1080//2, 1080//2),
        pygame.HWSURFACE | pygame.DOUBLEBUF)

    clock = pygame.time.Clock()
    behaviour_tree.tick()
    while behaviour_tree.root.status == py_trees.common.Status.RUNNING:
        # clock.tick_busy_loop(100)
        while camera_holder.surface is None or lidar_holder.surface is None:
            # print 'waiting for surface'
            pass
        behaviour_tree.tick(post_tick_handler=print_tree)
        display.blit(camera_holder.surface, (0, 0))
        display.blit(lidar_holder.surface, (1920//2, 0))
        ev_velocity = vehicle.get_velocity()
        tv_velocity = traffic_vehicle.get_velocity()
        textsurface = myfont.render(
            "EV Speed: " +
            str(round(magnitude(ev_velocity)*3.6)) +
            " TV Speed:" +
            str(round(magnitude(tv_velocity)*3.6)),
            True, (255, 255, 255)
        )
        display.blit(textsurface, (1920*0.5*0.1, 1080*0.5*0.9))
        pygame.display.flip()

finally:
    if vehicle is not None:
        vehicle.destroy()
    if traffic_vehicle is not None:
        traffic_vehicle.destroy()
    if camera_sensor is not None:
        camera_sensor.destroy()
