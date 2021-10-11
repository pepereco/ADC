#!/usr/bin/python

# Run this script with Python 2.7

from graph import *

if __name__ == '__main__1':
    graph = Graph()

    n1 = Node(graph, 1)
    n2 = Node(graph, 2)
    n3 = Node(graph, 3)
    n4 = Node(graph, 4)
    n5 = Node(graph, 5)
    n6 = Node(graph, 6)

    c1 = Connection(n1, n2, 2)
    c2 = Connection(n1, n3, 4)
    c3 = Connection(n2, n4, 7)
    c4 = Connection(n2, n3, 1)
    c5 = Connection(n3, n5, 3)
    c6 = Connection(n4, n6, 1)
    c7 = Connection(n5, n4, 2)
    c8 = Connection(n5, n6, 5)

    graph.nodes = [n1, n2, n3, n4, n5, n6]
    graph.connections = [c1, c2, c3, c4, c5, c6, c7, c8]

    print( "\n".join([repr(x) for x in graph.conn_conn_route(c1, c6)]) )


if __name__ == '__main__':
    graph = Graph()

    n1 = Node(graph, 1)
    n2 = Node(graph, 2)
    n3 = Node(graph, 3)
    n4 = Node(graph, 4)
    n5 = Node(graph, 5)
    n6 = Node(graph, 6)
    n7 = Node(graph, 7)

    c1_1 = Connection(n1, n2, 2)
    c1_2 = Connection(n2, n1, 2)
    c2_1 = Connection(n1, n3, 4)
    c2_2 = Connection(n3, n1, 4)
    c3_1 = Connection(n2, n4, 7)
    c3_2 = Connection(n4, n2, 7)
    c4_1 = Connection(n2, n3, 1)
    c4_2 = Connection(n3, n2, 1)
    c5_1 = Connection(n3, n5, 3)
    c5_2 = Connection(n5, n3, 3)
    c6_1 = Connection(n4, n6, 1)
    c6_2 = Connection(n6, n4, 1)
    c7_1 = Connection(n5, n4, 2)
    c7_2 = Connection(n4, n5, 2)
    c8_1 = Connection(n5, n6, 5)
    c8_2 = Connection(n6, n5, 5)
    c9_1 = Connection(n6, n7, 1)
    c9_2 = Connection(n7, n6, 1)

    graph.nodes = [n1, n2, n3, n4, n5, n6, n7]
    graph.connections = [c1_1, c1_2, c2_1, c2_2, c3_1, c3_2, c4_1, c4_2, c5_1, c5_2, c6_1, c6_2, c7_1, c7_2, c8_1, c8_2, c9_1, c9_2]

    print( "\n".join([repr(x) for x in graph.conn_conn_route(c1_1, c9_1)]) )


if __name__ == '__main__1':
    graph = Graph()

    n1 = Node(graph, 1)
    n2 = Node(graph, 2)
    n3 = Node(graph, 3)
    n4 = Node(graph, 4)

    c1_1 = Connection(n1, n2, 2)
    c1_2 = Connection(n2, n1, 2)
    c2_1 = Connection(n1, n3, 4)
    c2_2 = Connection(n3, n1, 4)
    c3_1 = Connection(n2, n3, 3)
    c3_2 = Connection(n3, n2, 3)
    c4_1 = Connection(n3, n4, 1)
    c4_2 = Connection(n4, n3, 1)

    graph.nodes = [n1, n2, n3, n4]
    graph.connections = [c1_1, c1_2, c2_1, c2_2, c3_1, c3_2, c4_1, c4_2]

    print( "\n".join([repr(x) for x in graph.conn_conn_route(c1_1, c4_1)]) )


