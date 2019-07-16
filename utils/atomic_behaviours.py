import math

import py_trees
import carla

from utils.in_memory_map import InMemoryMap
from utils.local_planner import LocalPlanner

# ======================= Actions ============================= #


class FollowLane(py_trees.behaviour.Behaviour):
    def __init__(self, vehicle, local_planner, name="FollowLane"):
        super(FollowLane, self).__init__(name)
        self._vehicle = vehicle
        self._local_planner = local_planner
        self._control = carla.VehicleControl()

    def update(self):
        throttle, steer = self._local_planner.update()
        self._control.throttle = throttle
        self._control.steer = steer
        self._vehicle.apply_control(self._control)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._control.throttle = 0
        self._control.steer = 0
        self._control.brake = 0
        self._vehicle.apply_control(self._control)


class StopVehicle(py_trees.behaviour.Behaviour):
    def __init__(self, vehicle, max_brake=1.0, name="StopVehicle"):
        super(StopVehicle, self).__init__(name)
        self._vehicle = vehicle
        self._max_brake = max_brake
        self._control = carla.VehicleControl()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        velocity = self._vehicle.get_velocity()
        velocity = math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
        if velocity > 0.1:      #Make this a constant
            self._control.throttle = 0
            self._control.steer = 0
            self._control.brake = self._max_brake
            self._vehicle.apply_control(self._control)
        else:
            new_status = py_trees.common.Status.SUCCESS
        return new_status


class LaneChange(py_trees.behaviour.Behaviour):
    def __init__(
            self, vehicle, local_map, local_planner,
            direction=1, sampling_resolution=2, name="LaneChange"
            ):
        super(LaneChange, self).__init__(name)
        self._vehicle = vehicle
        self._map = local_map
        self._local_planner = local_planner
        self._direction = direction
        self._plan = []
        self._control = carla.VehicleControl()

    def setup(self):
        kn = 2
        current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        current_waypoint = current_waypoint.get_waypoint()
        velocity = self._vehicle.get_velocity()
        velocity = math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
        if self._direction == 1:
            right_waypoint = current_waypoint.get_right_lane()
            lane_start = right_waypoint.next(kn*velocity)[0]
        elif self._direction == -1:
            left_waypoint = current_waypoint.get_left_lane()
            lane_start = left_waypoint.next(kn*velocity)[0]
        lane_start = self._map.get_waypoint(lane_start.transform.location)
        self._plan.append(lane_start)
        for _ in range(int(kn*velocity)):        #Account for this constant
            self._plan.append(self._plan[-1].get_next_waypoints()[0])
        self._local_planner.set_plan(self._plan)

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._plan:
            throttle, steer = self._local_planner.update()
            self._control.throttle = throttle
            self._control.steer = steer
            self._vehicle.apply_control(self._control)
            if len(self._plan) <= 1:
                new_status = py_trees.common.Status.SUCCESS
        else:
            self.setup()
        return new_status

    def terminate(self, new_status):
        self._control.throttle = 0
        self._control.steer = 0
        self._control.brake = 0
        self._vehicle.apply_control(self._control)


# ======================= Checks ============================= #


class DriveDistance(py_trees.behaviour.Behaviour):
    def __init__(self, vehicle, distance, name="DriveDistance"):
        super(DriveDistance, self).__init__(name)
        self._vehicle = vehicle
        self._distance = distance
        self._initial_location = None

    def setup(self):
        self._initial_location = self._vehicle.get_location()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._initial_location is not None:
            current_distance = self._vehicle.get_location().distance(self._initial_location)
            if current_distance > self._distance:
                new_status = py_trees.common.Status.SUCCESS
        else:
            self.setup()
        return new_status


class TriggerDistanceToVehicle(py_trees.behaviour.Behaviour):
    def __init__(
        self, reference_vehicle, other_vehicle,
        distance, comparision=1, name="TriggerDistanceToVehicle"
    ):
        super(TriggerDistanceToVehicle, self).__init__(name)
        self._reference_vehicle = reference_vehicle
        self._other_vehicle = other_vehicle
        self._distance = distance
        self._comparision = comparision

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._comparision == 1 and \
                self._reference_vehicle.get_location().distance(
                    self._other_vehicle.get_location()
                ) < self._distance:
            new_status = py_trees.common.Status.SUCCESS
        elif self._comparision == -1 and \
                self._reference_vehicle.get_location().distance(
                    self._other_vehicle.get_location()
                ) > self._distance:
            new_status = py_trees.common.Status.SUCCESS
        return new_status
