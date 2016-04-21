#!/usr/bin/env python

import rospy

from pygraph.classes.graph import graph
from pygraph.classes.exceptions import NodeUnreachable
from pygraph.algorithms.heuristics.euclidean import euclidean
from pygraph.algorithms.minmax import heuristic_search

from math import sqrt
from random import uniform

#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty RandomizedRoadmapPlanner class.
#               An instance of this class will be created by the path_planner
#               node. It should maintain a graph of points and the connections
#               between them.
#               The 'plan()' function should find a path between the requested
#               points, inserting new nodes and edges if necessary. Make sure
#               that it stops at some point in time if no path between the
#               points exists.
#
# Remark: it will be necessary to test points and line segments for emptiness.
#         The class is (as usual) ROS-independent, so the actual mechanism of
#         performing these tests is abstracted by two callback functions, which
#         the object receives during the construction. In order to test whether
#         e.g. the point (1, 4) is free you should do:
#
#             free = self.point_free_cb((1, 4))
#
# Hint: use the standard function 'math.uniform()' to generate the coordinates
#       for random points.
#
# Hint: if you decided to use 'pygraph' library for graph and search
#       implementations, make sure that the graph object is stored in a member
#       field called 'graph'. If this is the case, the nodes and edges of the
#       graph will be automatically visualized by the path_planner node after
#       each planning request.

class RandomizedRoadmapPlanner:

    def __init__(self, point_free_cb, line_free_cb, dimensions):
        """
        Construct a randomized roadmap planner.

        'point_free_cb' is a function that accepts a point (two-tuple) and
        outputs a boolean value indicating whether the point is in free space.

        'line_free_cb' is a function that accepts two points (the start and the
        end of a line segment) and outputs a boolen value indicating whether
        the line segment is free from obstacles.

        'dimensions' is a tuple of tuples that define the x and y dimensions of
        the world, e.g. ((-8, 8), (-8, 8)). It should be used when generating
        random points.
        """
        self.point_free_cb = point_free_cb
        self.line_free_cb = line_free_cb
        self.dimensions = dimensions

        self.search_algorithm = euclidean()
        self.number_of_trials = 500
        self.search_result = False
        self.output_path = None
        self.id_generator = 10000
        self.graph = graph()


    def next_id(self):
        self.id_generator=self.id_generator+1
        return self.id_generator


    def is_valid_points(self, point1, point2):
        return self.point_free_cb(point1) and self.point_free_cb(point2)


    def plan(self, point1, point2):
        """
        Plan a path which connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Return a list of tuples where each tuple represents a point in the
        planned path, the first point is the start point, and the last point is
        the end point. If the planning algorithm failed the returned list
        should be empty.
        """
        #reset number of trials
        self.number_of_trials = 50

        self.search_result = False
        self.output_path = list()
        start_node_id = self.next_id()
        end_node_id = self.next_id()

        if not self.is_valid_points(point1, point2):
            rospy.logwarn(self.__class__.__name__ + ": invalid points")
            return self.output_path #return empty list
        else:
            # add point1, point2 to the graph
            self.graph.add_node(start_node_id, attrs=[('position', point1)])
            self.graph.add_node(end_node_id, attrs=[('position', point2)])

            # connect point1, point2
            if self.line_free_cb(point1, point2):
                edge_weight = sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))
                self.graph.add_edge((start_node_id, end_node_id), wt = edge_weight)

            # add more edges between point1, point2 and the rest of graph
            for temp_id, attr in self.graph.node_attr.iteritems():
                # 'temp_id' contains node temp_id, 'attr' contains array of attributes
                temp_position = attr[0][1]
                if point1 != temp_position and self.line_free_cb(point1, temp_position):
                    edge_weight = sqrt(pow(point1[0] - temp_position[0], 2) + pow(point1[1] - temp_position[1], 2))
                    self.graph.add_edge((start_node_id, temp_id), wt = edge_weight)

                if point2 != temp_position and self.line_free_cb(point2, temp_position):
                    edge_weight = sqrt(pow(point2[0] - temp_position[0], 2) + pow(point2[1] - temp_position[1], 2))
                    self.graph.add_edge((end_node_id, temp_id), wt = edge_weight)

        while not self.search_result and self.number_of_trials > 0:
            try:
                #find a path (sequence of node ids) between start and end
                self.search_algorithm.optimize(self.graph)
                node_ids = heuristic_search(self.graph, start_node_id, end_node_id, self.search_algorithm)

                for temp_id in node_ids:
                    node_position = self.graph.node_attributes(temp_id)
                    self.output_path.append(node_position[0][1])

                self.search_result = True

            except NodeUnreachable:
                # create a random point if there is no path
                x = uniform(self.dimensions[0][0] , self.dimensions[0][1])
                y = uniform(self.dimensions[1][0] , self.dimensions[1][1])
                new_point_position = (x , y)
                # add the new point to graph
                if self.point_free_cb(new_point_position):
                    new_point_id = self.next_id()
                    self.graph.add_node(new_point_id, attrs=[('position', new_point_position)])
                    # connect the new_point with older nodes
                    for temp_id, attr in self.graph.node_attr.iteritems():
                        temp_position = attr[0][1]
                        if new_point_position != temp_position and self.line_free_cb(new_point_position, temp_position):
                            edge_weight = sqrt(pow(new_point_position[0] - temp_position[0], 2) + pow(new_point_position[1] - temp_position[1], 2))
                            self.graph.add_edge((new_point_id, temp_id), wt = edge_weight)

            # decrease number of trials
            self.number_of_trials = self.number_of_trials-1

        return self.output_path


    def remove_edge(self, point1, point2):
        """
        Remove the edge of the graph that connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Has an effect only if both points have a corresponding node in the
        graph and if those nodes are connected by an edge.
        """
        found_node1=False
        found_node2=False
        # search for nodes in graph
        for temp_node_id, attr in self.graph.node_attr.iteritems():
            temp_position = attr[0][1]
            if(point1 == temp_position):
                node1 = temp_node_id;
                found_node1=True
            elif(point2 == temp_position):
                node2 = temp_node_id;
                found_node2=True
            if(found_node1 and found_node2):
                break
        #delete edge if there is an edge that connects the 2 nodes
        if(found_node1 and found_node2 and self.graph.has_edge((node1,node2))):
            self.graph.del_edge((node1,node2));

