import py_trees

from scenarios.basic_scenario import BasicScenario
from utils.atomic_behaviours import FollowLane
from utils.atomic_behaviours import DriveDistance
from utils.atomic_behaviours import StopVehicle
from utils.local_planner import LocalPlanner
from utils.in_memory_map import InMemoryMap


class LaneFollowing(BasicScenario):
    def __init__(self, world, name, ego_vehicle, other_vehicles):
        super(LaneFollowing, self).__init__(
            world, name, ego_vehicle, other_vehicles
        )

    def create_tree(self):
        root = py_trees.composites.Sequence()
        drive = py_trees.composites.Parallel(
            name="root", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        local_map = InMemoryMap(self._world.get_map())
        local_map.setup()
        local_planner = LocalPlanner(self._ego_vehicle, local_map)
        drive.add_child(
            FollowLane(self._ego_vehicle, local_planner)
        )
        drive.add_child(DriveDistance(self._ego_vehicle, 20))

        root.add_child(drive)
        root.add_child(StopVehicle(self._ego_vehicle))
        return root
