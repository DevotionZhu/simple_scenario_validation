import py_trees

from utils.atomic_behaviours import FollowLane
from utils.atomic_behaviours import DriveDistance
from utils.atomic_behaviours import LaneChange
from utils.atomic_behaviours import StopVehicle
from scenarios.basic_scenario import BasicScenario
from utils.in_memory_map import InMemoryMap
from utils.local_planner import LocalPlanner


class LaneChanging(BasicScenario):
    def __init__(self, world, name, ego_vehicle, other_vehicles):
        super(LaneChanging, self).__init__(
            world, name, ego_vehicle, other_vehicles
        )

    def create_tree(self):
        root = py_trees.composites.Sequence()
        local_map = InMemoryMap(self._world.get_map())
        local_map.setup()
        drive = py_trees.composites.Parallel(
            name="root", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        drive.add_child(
            FollowLane(
                self._ego_vehicle, 
                LocalPlanner(
                    self._ego_vehicle, local_map, target_velocity=20,
                    kv=2
                )
            )
        )
        drive.add_child(DriveDistance(self._ego_vehicle, 20))
        root.add_child(drive)
        root.add_child(
            LaneChange(
                self._ego_vehicle, local_map,
                LocalPlanner(self._ego_vehicle, local_map, target_velocity=20, kv=2),
                direction=-1
            )
        )
        root.add_child(StopVehicle(self._ego_vehicle, max_brake=1.0))
        return root
