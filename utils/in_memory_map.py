from utils.simple_waypoint import SimpleWaypoint


class InMemoryMap:

    def __init__(self, world_map):
        self._world_map = world_map
        self._dense_topology = []

    def setup(self, sampling_resolution=2):
        entry_waypoints = []
        exit_waypoints = []

        sparse_toplogy = self._world_map.get_topology()
        for segment in sparse_toplogy:
            begin_waypoint = segment[0]
            end_waypoint = segment[1]

            if begin_waypoint.transform.location.distance(
                    end_waypoint.transform.location) > sampling_resolution:

                current_waypoint = begin_waypoint
                self._dense_topology.append(SimpleWaypoint(current_waypoint))
                entry_waypoints.append(self._dense_topology[-1])

                while current_waypoint.transform.location.distance(
                        end_waypoint.transform.location) > sampling_resolution:
                    previous_waypoint = self._dense_topology[-1]
                    current_waypoint = current_waypoint.next(
                        sampling_resolution)[0]
                    self._dense_topology.append(
                        SimpleWaypoint(current_waypoint))
                    previous_waypoint.set_next_waypoint(self._dense_topology[-1])

                previous_waypoint = self._dense_topology[-1]
                self._dense_topology.append(SimpleWaypoint(end_waypoint))
                previous_waypoint.set_next_waypoint(self._dense_topology[-1])
                exit_waypoints.append(self._dense_topology[-1])

        i, j = 0, 0
        for end_point in exit_waypoints:
            i += 1
            for begin_point in entry_waypoints:
                j += 1
                if end_point.distance(begin_point) <= sampling_resolution and \
                        i != j:
                    end_point.set_next_waypoint(begin_point)

    def get_waypoint(self, location):
        min_distance = float('inf')
        closest_waypoint = None
        for swp in self._dense_topology:
            waypoint = swp.get_waypoint()
            if waypoint.transform.location.distance(location) < min_distance:
                min_distance = waypoint.transform.location.distance(location)
                closest_waypoint = swp

        return closest_waypoint

    def get_dense_topology(self):
        return self._dense_topology
