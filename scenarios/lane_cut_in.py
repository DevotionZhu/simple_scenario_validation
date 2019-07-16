import py_trees

from utils.atomic_behaviours import FollowLane
from utils.atomic_behaviours import DriveDistance
from utils.atomic_behaviours import LaneChange
from utils.atomic_behaviours import StopVehicle
from utils.atomic_behaviours import TriggerDistanceToVehicle
from scenarios.basic_scenario import BasicScenario
from utils.in_memory_map import InMemoryMap
from utils.local_planner import LocalPlanner


class LaneCutIn(BasicScenario):
    def __init__(self, world, name, ego_vehicle, other_vehicles):
        super(LaneCutIn, self).__init__(
            world, name, ego_vehicle, other_vehicles
        )
        self._traffic_vehicle = other_vehicles[0]

    def create_tree(self):
        tv_speed = 30
        ego_speed = 20
        ego_drive_distance = 600
        tv_seq = py_trees.composites.Sequence()
        local_map = InMemoryMap(self._world.get_map())
        local_map.setup()
        drive_tv = py_trees.composites.Parallel(
            name="root", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        drive_tv.add_child(
            FollowLane(
                self._traffic_vehicle,
                LocalPlanner(
                    self._traffic_vehicle, local_map,
                    target_velocity=tv_speed, kv=2
                )
            )
        )
        drive_past = py_trees.composites.Sequence()
        drive_past.add_child(
            TriggerDistanceToVehicle(
                self._ego_vehicle, self._traffic_vehicle,
                5, comparision=1
            )
        )
        drive_past.add_child(
            TriggerDistanceToVehicle(
                self._ego_vehicle, self._traffic_vehicle,
                5, comparision=-1
            )
        )
        drive_tv.add_child(drive_past)
        tv_seq.add_child(drive_tv)
        tv_seq.add_child(
            LaneChange(
                self._traffic_vehicle, local_map,
                LocalPlanner(
                    self._traffic_vehicle, local_map,
                    target_velocity=tv_speed, kv=2
                ), direction=1
            )
        )
        tv_seq.add_child(
            FollowLane(
                self._traffic_vehicle,
                LocalPlanner(
                    self._traffic_vehicle, local_map,
                    target_velocity=tv_speed, kv=2
                )
            )
        )

        drive_ev = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        follow_lane_ev = FollowLane(
            self._ego_vehicle,
            LocalPlanner(
                self._ego_vehicle, local_map,
                target_velocity=ego_speed, kv=2
            )
        )
        drive_distance_ev = DriveDistance(
            self._ego_vehicle, ego_drive_distance
        )
        drive_ev.add_child(follow_lane_ev)
        drive_ev.add_child(drive_distance_ev)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        root.add_child(tv_seq)
        root.add_child(drive_ev)

        return root
