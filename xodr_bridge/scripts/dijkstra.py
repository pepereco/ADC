#!/usr/bin/python

# Run this script with Python 2.7

"""
ADC_2021 - UPC Driverless - Arnau Beltra Bisa
This module looks for the fastest path to get to a certain position
"""

def fastest_route(graph, orig_conn, dest_conn):
    """
    Looks for the fastest route to get to a certain position, looking node by node.

    DONE: If orig_conn == dest_conn, we look for a way to loop trough both nodes.
    """
    #Debug
    #print(orig_conn.id, dest_conn.id)

    # This dict will hold the distance needed to get trough the nodes.
    # Assigns to each node the distance inf, but not for the origin node.
    conns_distance = { conn : float('inf') for conn in graph.connections }
    conns_distance[orig_conn] = 0

    # This one will save the shortest path to use in each case.
    conns_paths = { conn : [] for conn in graph.connections }
    conns_paths[orig_conn] = [orig_conn]

    # The used variables during each iteration.
    current_conn = orig_conn
    current_conn_path = [orig_conn]

    # We create a list with all the connections
    not_seen_connections = graph.connections[:]
    calculated_connections = [orig_conn]

    if orig_conn == dest_conn:

        tmp_conn = graph.create_attached_connection(orig_conn)

        conns_distance[orig_conn] = float('inf')
        conns_distance[tmp_conn] = 0

        conns_paths[orig_conn] = []
        conns_paths[tmp_conn] = [orig_conn]

        current_conn = tmp_conn
        current_conn_path = [tmp_conn]

        not_seen_connections.append(tmp_conn)
        calculated_connections = [tmp_conn]
        

    while True:
        #Debug
        # print();print()
        # print("NOT", not_seen_connections);print()
        # print("CALC", calculated_connections);print()
        # print("CURR", current_conn)
        # print(current_conn_path)

        # Stores the fastest node_path that has to be followed to get to certain coordinates
        #current_conn_path.append(current_conn)
        not_seen_connections.remove(current_conn)

        if (current_conn == dest_conn):
            return current_conn_path

        # Applies the relaxation procedure to the closest following neighbours to recalculate the distance
        connlist = graph.get_continuation_connections(current_conn)

        for following_conn in connlist:

            # We add it to the list of possible connections to see.
            if following_conn not in calculated_connections:
                calculated_connections.append(following_conn)

            # We want to update nodes_conns if nodes_distance changes, so we save the current value.
            previous_distance = conns_distance[following_conn]
            assert following_conn.start == current_conn.end

            conns_distance[following_conn] = relaxation(conns_distance[current_conn], conns_distance[following_conn], following_conn.length)

            if conns_distance[following_conn] != previous_distance:
                conns_paths[following_conn] = current_conn_path + [following_conn]


        # Getting the next connection to loop at
        # Looks for the connection with the smallest distance
        smallest_distance = float("inf")
        changes = False
        for conn in calculated_connections:
            if conns_distance[conn] < smallest_distance and conn in not_seen_connections:
                changes = True
                smallest_distance = conns_distance[conn]

                # Assigns current_node for next iteration
                current_conn_path = conns_paths[conn]
                current_conn = conn

        if not changes:
            raise IndexError('There is not a solution to that route!')

def relaxation(dist_u, dist_v, cost_u2v):
    """
    Applies the relaxation procedure which cosists into recalculating the distance
    of the following nodes based on the previous followed path and the cost of
    reaching the next node.
    """
    if ((dist_u + cost_u2v) < dist_v):
        dist_v = dist_u + cost_u2v
    return dist_v
