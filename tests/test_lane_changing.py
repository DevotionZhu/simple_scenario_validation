import carla
import py_trees

from scenarios.lane_changing import LaneChanging

ip = "10.51.6.52"
port = 2006

client = carla.Client(ip, port)
world = client.get_world()

vehicle = world.get_actors().filter('vehicle.*')[0]
scenario = LaneChanging(world, "LaneFollowingEgo", vehicle, None)

root = scenario.create_tree()
behaviour_tree = py_trees.trees.BehaviourTree(root=root)
behaviour_tree.tick()


def print_tree(tree):
    print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))

while behaviour_tree.root.status == py_trees.common.Status.RUNNING:
    behaviour_tree.tick(post_tick_handler=print_tree)
