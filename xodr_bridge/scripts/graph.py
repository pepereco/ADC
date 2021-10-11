#!/usr/bin/python

# Run this script with Python 2.7

"""
ADC_2021 - UPC Driverless - Eric Roy
This module manages a graph, used to parse the xodr files.
"""

from dijkstra import fastest_route
from math import cos, pi, sin, sqrt

DIVIDE_BY = 200
DISTANCE_TOLERANCE = 15 # meters
YAW_TOLERANCE = pi/20 # = 9 degrees

class Graph(object):
    def __init__(self):
        self.nodes = []
        self.connections = []

    def get_continuation_nodes(self, current_node, previous_node = None):
        """
        Returns a list of Nodes from where we can go after the current node.

        We also use previous_node because we wouldn't normally want to go back to where we were.
        """
        nodelist = []

        for path in self.connections:
            if path.start == current_node and path.end != previous_node:
                nodelist.append(path.end)

        return nodelist

    def get_continuation_connections(self, current_connection):
        """
        Returns a list of Connections where we use after the current Connection.
        """
        connlist = []

        for conn in self.connections:
            if conn != current_connection and conn.reverse != current_connection and conn.start == current_connection.end:
                connlist.append(conn)

        return connlist

    def find_node(self, id):
        """
        Returns a Node with the id provided, as a string.
        Raises KeyError if not found.
        """
        for single_node in self.nodes:
            if single_node.id == id:
                return single_node

        raise KeyError( "Node not found. ID: {0}".format(id) )

    def find_connection(self, id):
        """
        Returns a Connection with the id provided, as a integer.
        Raises KeyError if not found.
        """
        for single_conn in self.connections:
            if single_conn.id == id:
                return single_conn

        raise KeyError( "Connection not found. ID: {0}".format(id) )

    def __repr__(self):
        return "GRAPH: {0} Nodes, {1} Connections.".format(len(self.nodes), len(self.connections))

    def conn_conn_route(self, orig_conn, dest_conn):
        """
        Uses the algorithm. Returns a list of connections where the first connection is orig_conn and the last_conn is dest_conn.
        The connection list creates a path to follow.
        """
        return fastest_route(self, orig_conn, dest_conn)

    def find_xodr_connection(self, coords):
        """
        Given a point, we return the Connection and the distance to go through this connection in order to get to that point.

        coords = Coords object: (x, y, yaw)

        returns: best_connection, distance_in_connection, exact_coords

        Where:
        - Best_connection: A Connection instance of the most precise connection.
        - Distance_in_connection: The distance to drive in that connection before reaching that poing.
        - exact_coords. The real exact coordinates of the calculated point. Used to correct the car position if needed.
        """
        best_result = None
        best_connection = None

        for conn in self.connections:
            result = conn.awayfrom(coords)

            if best_result is None:
                best_result = result
                best_connection = conn

            elif best_result[0] > result[0]:
                best_result = result
                best_connection = conn

        return best_connection, best_result[2], best_result[1]

    def from_point_to_point(self, corrds_from, coords_to, find_parkings=False):
        """
        Main function, runned in other programs.

        TODO: Make find_parkings functionnality
        """
        start_conn, start_dist, start_coords = self.find_xodr_connection(corrds_from)
        end_conn, end_dist, end_coords = self.find_xodr_connection(coords_to)

        if start_conn != end_conn or start_dist>end_dist:
            connection_list = self.conn_conn_route(start_conn, end_conn)
            car_path = Path(connection_list, end_dist)

        else: # In case of a just-go-forward case
            car_path = Path([], end_dist-start_dist)

        return car_path

    def create_attached_connection(self, similar_conn):
        """
        Creates a connection to the graph that matches in all stuff the first one except the start and the instance.
        """

        tmp_conn = Connection(Node(self, "tempnode"), similar_conn.end, similar_conn.length, reverse=similar_conn.reverse )
        tmp_conn.end_coords = similar_conn.end_coords

        self.connections.append( tmp_conn )
        return tmp_conn

    def can_not_turn(self, car_coords, direction):
        """
        Returns 0 if the Connection that starts pointing at ``direction`` in the nearest intersection with ``car_coords`` is removed.

        Returns another number when an accurate enough Connection is not found.
        """
        car_conn, dist, exact_coords = self.find_xodr_connection(car_coords)

        # Getting intersection data.
        if car_conn.length-dist > dist:
            intersection = car_conn.start
            inter_coords = car_conn.start_coords
        else:
            intersection = car_conn.end
            inter_coords = car_conn.end_coords

        # Too far away from the intersections
        if car_coords.distance(inter_coords) > DISTANCE_TOLERANCE: return 1
        
        best_connection = None
        best_angle_diff = float('inf')

        for candidate in self.get_continuation_connections(car_conn):
            angle_diff = abs(Coords.rectify_yaw(candidate.start_coords.yaw - direction))

            if angle_diff < best_angle_diff:
                best_angle_diff = angle_diff
                best_connection = candidate

        # Non accurate connection
        if best_angle_diff > YAW_TOLERANCE: return 2

        # Delete that connection
        self.connections.remove(best_connection)
        return 0


class Node(object):
    def __init__(self, graph, id):
        """
        We give the graph when creating a Node so we can use it to comunicate with others.
        """
        self.graph = graph
        self.id = id

    def connects(self, other):
        """
        Returns whether this node has a direct connection with the other node.
        """
        for path in self.graph.connections:
            if path.start == self and path.end == other:
                return True
        
        return False

    def dual_connection(self, other):
        """
        Returns whether this node can connect with the oher in both ways.
        """
        return self.connects(other) and other.connects(self)

    def __repr__(self):
        return "NODE(id:{0})".format(self.id)

class Connection(object):
    """
    Start and End are the nodes in the connection. A connection is UNIDIRECTIONAL, so if you have a
    road that can be driven in both ways, you need to create both connections.

    Then you can use Node.dual_connection(other) to check if a road is unidirectional or not.
    """
    last_id = 0

    def __init__(self, node_start, node_end, length, roads=None, reverse = None):
        self.start = node_start
        self.end = node_end
        self.length = length

        self.reverse = reverse

        Connection.last_id += 1
        self.id = self.last_id

        if roads is not None:
            self.roads = roads
            self.start_coords = self.roads[0].startcoords
            self.end_coords = self.roads[-1].endcoords

        else:
            self.roads = []
            self.start_coords = None
            self.end_coords = None

    def __repr__(self):
        return "CONNECTION(Id:{0}): From {1} To {2}. Length: {3}".format(self.id, self.start, self.end, self.length)            

    def awayfrom(self, coords):
        """
        Returns the distance multiplied by the yaw difference from the nearest point of coords and the line of the road connection.

        coords: Coords object (x, y, yaw)

        returns: (error, exact_coords, meters_in_connection)

        error: the distance from the point and the road.
        exact_cords: coords - error
        meters_in_connection: how much we have to go through the connection to get to that point.
        """
        # If this road is "fake", we don't want to use it.
        if self.roads == []:
            #raise NotImplementedError ('You need to add some xodr Roads!')
            return float('inf'), Coords(0, 0, 0), 0

        best_result = None
        accumulated_distance = 0

        for road in self.roads:

            pointlist = split_line(road.startcoords, road.curvature, road.length)
            pointseparation = road.length / float(DIVIDE_BY)

            i = 0
            error = float('inf')
            while i < len(pointlist):

                current_error = distance_two_points(pointlist[i], coords)
                current_error += abs(coords.yaw - pointlist[i].yaw)*4 # I put *4 to give more strength to the orientation.

                if current_error < error:
                    error = current_error
                    exact_coords = pointlist[i]
                    distance_in_road = pointseparation*i
                    meters_in_connection = accumulated_distance + distance_in_road

                i += 1

            # Merge both roads and calculate
            accumulated_distance += road.length

            if best_result is None:
                best_result = (error, exact_coords, meters_in_connection)
            elif best_result[0] > error:
                best_result = (error, exact_coords, meters_in_connection)

        assert accumulated_distance == self.length
        return best_result


class Coords(object):
    """
    This class has been made to hold 3 items in an object. There's not really much into it.
    """
    def __init__(self, x, y, yaw):
        """
        REMEMBER!! Yaw angle in radians! And clockwise: (+pi/2 = left turn, -pi/2 = right turn)
        """
        self.x = x
        self.y = y
        self.yaw = Coords.rectify_yaw(yaw)

    @staticmethod
    def rectify_yaw(yaw):
        """
        This non-class method returns the reduced yaw.

        No need to run this as a Coords instance.
        """
        while yaw > pi:
            yaw -= 2*pi

        while yaw < -pi:
            yaw += 2*pi

        return yaw

    def __repr__(self):
        return "(x:{0}, y:{1}, yaw:{2})".format(self.x, self.y, self.yaw)

    def invert_yaw(self):
        """
        Returns new Coords object with the yaw inverted.
        """
        return Coords(self.x, self.y, self.yaw + pi)

    def distance(self, other):
        """
        Returns the distance between two coordinates.
        """
        return sqrt( (self.x - other.x)**2 + (self.y - other.y)**2 )


class Path(object):
    """
    A list of yaws and the last meters to go. This is what we pass to the car.
    """
    def __init__(self, connlist, lastmeters):
        self.connections = connlist
        self.lastmeters = lastmeters

        # Calculate the angles to turn in each node
        i = 1
        self.angles = []
        while i < len(self.connections):
            entering = self.connections[i-1].end_coords.yaw
            exiting = self.connections[i].start_coords.yaw

            diff = Coords.rectify_yaw(exiting-entering)

            self.angles.append(diff)
            i += 1

        # The car should use self.angles and self.lastmeters
        # or the integrated functions, defined below.

    def __getitem__(self, index):
        return self.angles[index]

    def __repr__(self):
        return "PATH: Yaws: {0} LastMeters: {1}".format(self.angles, self.lastmeters)

    def __iter__(self):
        return self.angles.__iter__()


class RoadDumper(object):
    """
    A list of Road objects.
    """
    def __init__(self, graph):
        self.roads = []
        self.graph = graph

    def add(self, e):
        self.roads.append(e)

    def is_ended(self):
        """
        Checks if all the roads of this instance goes from junction to junction.
        """
        for road in self.roads:
            if not road.is_full_road():
                return False
        
        return True

    def connect(self):
        """
        This function connects all the Roads with the Nodes of the graph, and returns a list of connections, already added to the graph.

        So, after running this, the graph should be created and working.
        """
        self.join_all()

        for road in self.roads:
            pointa = self.graph.find_node(road.pred_id)
            pointb = self.graph.find_node(road.succ_id)

            temp_conn1 = Connection(pointa, pointb, road.length, road.composed)
            self.graph.connections.append( temp_conn1 )

            if road.two_sides:
                temp_conn2 = Connection(pointb, pointa, road.length, road.invert_composed(), temp_conn1)
                self.graph.connections.append( temp_conn2 )
                temp_conn1.reverse = temp_conn2

    def join_all(self):
        """
        This "recursive" function will run until until all roads are joined or an error occurred.
        """
        while not self.is_ended():

            # We find the first road that has not an end
            for road in self.roads:
                if road.succ_type != "junction":
                    break
            else:
                raise KeyError('A road does not start in a junction!')

            # We check for a candidate road
            for secondroad in self.roads:
                if road.can_join(secondroad):
                    self.roads.append(road.join(secondroad))
                    self.roads.remove(road)
                    self.roads.remove(secondroad)
                    break
            else:
                raise KeyError('A road does not end in a junction!')

class Road(object):
    """
    An XODR Road with all the necessary information used to connect them to the nodes.
    """
    def __init__(self, id, pred_type, succ_type, pred_id, succ_id, two_sides, length, startcoords, curvature, endcoords, id2 = None, composed = None):
        """
        ID2 is used when joining roads.
        """
        self.id = id
        self.pred_type = pred_type
        self.succ_type = succ_type
        self.pred_id = pred_id
        self.succ_id = succ_id
        self.two_sides = two_sides
        self.length = length
        self.startcoords = startcoords
        self.curvature = curvature
        self.endcoords = endcoords
        self.id2 = id2 if id2 is not None else id

        self.composed = [self] if composed is None else composed

    def is_full_road(self):
        """
        Returns whether a road is full (goes from junc to junc).
        """
        return self.pred_type == "junction" and self.succ_type == "junction"

    def can_join(self, other):
        """
        Returns whether two roads can be joined.

        I'm lazy so i use the join function raises.
        """
        if self.succ_type != "road" or other.pred_type != "road" or (self.two_sides != other.two_sides):
            return False

        if self.succ_id != other.id or other.pred_id != self.id2:
            return False

        return True

    def join(self, other):
        """
        Joins two roads that have the same junction. The first road's successor must be the second road, and the second road's predecessor must be the first road, otherwise an exception is raised.
        """
        if self.succ_type != "road" or other.pred_type != "road" or (self.two_sides != other.two_sides):
            raise TypeError("Can't join! Both roads must connect!")

        if self.succ_id != other.id or other.pred_id != self.id2:
            raise TypeError("Can't join! Both ids must connect!")

        return Road(self.id, self.pred_type, other.succ_type, self.pred_id, other.succ_id, self.two_sides, self.length + other.length, self.startcoords, None, other.endcoords, other.id2, self.composed + other.composed)

    def invert_composed(self):
        """
        This function will invert the composed list of a road, as well as the start, end and yaw of it's components.
        """
        inv_list = self.composed[::-1]
        result = []

        for sr in inv_list:
            new_road = Road(sr.id, sr.succ_type, sr.pred_type, sr.succ_id, sr.pred_id, False, sr.length, sr.endcoords.invert_yaw(), sr.curvature, sr.startcoords.invert_yaw())
            result.append(new_road)

        return result

    def __repr__(self):
        return "\nROAD(Id: {0}): Start: {1}, End: {2}, Curv: {3}".format(self.id, self.startcoords, self.endcoords, self.curvature)


### GEOMETRY FUNCTIONS

def split_line(start_coords, curvature, length, divide_by = DIVIDE_BY):
    """
    Splits a straight or curved line in ``divide_by`` smaller lines.

    Returns ``divide_by + 1`` Coords.
    """
    segment_length = length/float(divide_by)

    if curvature != 0:

        # while curvature < 0:
        #     curvature += 2*pi

        yaw_rate = segment_length*curvature
        radius = 2*pi / curvature

    else:
        new_yaw = start_coords.yaw

    newPoints = [start_coords]
    last_coords = start_coords

    for i in range(1, divide_by+1):
        

        if curvature == 0:
            new_x = last_coords.x + segment_length*cos(new_yaw)
            new_y = last_coords.y + segment_length*sin(new_yaw)

        else:
            new_yaw = (last_coords.yaw + yaw_rate) % (2*pi)

            # Finding new point

            # We need to:
            # - Rotate yaw 90 degrees and move radius meters

            current_yaw = last_coords.yaw + (pi/2)

            new_x = last_coords.x + radius*cos(current_yaw)
            new_y = last_coords.y + radius*sin(current_yaw)

            current_yaw += pi + yaw_rate # Half a turn + yaw_rate

            new_x += radius*cos(current_yaw)
            new_y += radius*sin(current_yaw)

            # - Half turn yaw + move yaw rate
            # - Move radius meters

        tmp = Coords(new_x, new_y, new_yaw)
        newPoints.append(tmp)
        last_coords = tmp

    assert len(newPoints) == divide_by+1
    return newPoints

def distance_two_points(pointA, pointB):
    """
    Returns the distance between two points, in meters.
    """
    return sqrt( (pointA.x-pointB.x)**2 + (pointA.y-pointB.y)**2 )

if __name__ == "__main__":
    print("Debug!")

    # print(split_line(Coords(10, 0, 1.59), -0.1, 15.9, 20))
    # print(split_line(Coords(100, 50, 0.2), 0, 50, 20))