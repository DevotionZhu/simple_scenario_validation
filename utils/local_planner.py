import numpy as np
import math
import carla

from utils.in_memory_map import InMemoryMap


class LocalPlanner:

    def __init__(
        self, vehicle, local_map, target_velocity=10,
        buffer_size=50, kv=1, ks=1, plan=None
    ):
        self._vehicle = vehicle
        self._map = local_map
        self._buffer = []
        self._buffer_size = buffer_size
        self._min_distance = 0.5*target_velocity
        self._target_velocity = target_velocity
        self._kv = kv
        self._ks = ks
        self._plan = plan
        if self._plan:
            self._buffer += plan

    def set_plan(self, plan):
        self._plan = plan
        if self._plan:
            self._buffer = plan

    def purge_buffer(self):
        while self._buffer and self._vehicle.get_location().distance(
                self._buffer[0].get_location()) < self._min_distance:
            self._buffer.pop(0)

    def populate_buffer(self):
        if len(self._buffer) == 0:
            self._buffer.append(self._map.get_waypoint(self._vehicle.get_location()))
        while len(self._buffer) < self._buffer_size:
            self._buffer.append(self._buffer[-1].get_next_waypoints()[0])

    def controls(self, current_transform, current_velocity):
        throttle = self._kv*(self._target_velocity - current_velocity)#/\
            #self._target_velocity
        throttle = np.clip(throttle, 0, 1)
        heading_vector = current_transform.get_forward_vector()
        heading_vector.z = 0
        heading_vector = self.unit_vector(heading_vector)
        target_location = self._buffer[0].get_location()
        current_location = current_transform.location
        target_vector = target_location - current_location
        target_vector.z = 0
        target_vector = self.unit_vector(target_vector)
        theta = math.acos(
            np.clip(
                target_vector.x*heading_vector.x + target_vector.y*heading_vector.y,
                -1, 1
            )
        )
        cross_product = heading_vector.x*target_vector.y-target_vector.x*heading_vector.y
        steer = self._ks * 2.0 * theta / math.pi
        steer = np.clip(steer, -1, 1)
        if cross_product < 0:
            steer *= -1
        return (throttle, steer)

    def update(self):
        self.purge_buffer()
        if self._plan is None:
            self.populate_buffer()
        throttle, steer = self.controls(
            self._vehicle.get_transform(),
            self.vector_length(self._vehicle.get_velocity()))

        return throttle, steer

    def vector_length(self, vector):
        return math.sqrt(vector.x**2+vector.y**2+vector.z**2)

    def unit_vector(self, vector):
        magnitude = self.vector_length(vector)
        return carla.Vector3D(
            vector.x/magnitude,
            vector.y/magnitude,
            vector.z/magnitude
        )
