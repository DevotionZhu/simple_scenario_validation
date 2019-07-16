
class SimpleWaypoint():
    def __init__(self, waypoint):
        self._waypoint = waypoint
        self._next_waypoints = []

    def get_waypoint(self):
        return self._waypoint

    def set_next_waypoint(self, waypoint):
        self._next_waypoints.append(waypoint)

    def get_next_waypoints(self):
        return self._next_waypoints

    def distance(self, waypoint):
        wp = waypoint.get_waypoint()
        return wp.transform.location.distance(self._waypoint.transform.location)

    def get_location(self):
        return self._waypoint.transform.location