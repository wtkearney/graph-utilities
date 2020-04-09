
# from datetime import datetime

import queue
import pickle
import os
import random

#
# Graph data struct, DFS/BFS implementations, and functions to check if graph is fully-connected and generate a rank matrix.
#
# Will Kearney
# 

class Node():
	
	def __init__(self, name):
		self.name = name
		self.visited = False
		self.neighbors = []
		self.rank = 0

	def get_name(self):
		'''Returns the name of the node'''
		return self.name

	def get_rank(self):
		'''Returns the rank'''
		return self.rank

	def set_rank(self, rank):
		'''Sets the rank'''
		self.rank = rank

	def mark_visited(self):
		'''Marks the node as visited'''
		self.visited = True

	def mark_unvisited(self):
		'''Marks the node as unvisited'''
		self.visited = False

	def is_visited(self):
		'''Checks if the node is visited'''
		return self.visited

	def add_neighbor(self, neighbor):
		'''Adds a neighbor to the nodes neighbor list. Throws error if neighbor already exists in neighbor list'''
		self.neighbors.append(neighbor)

	def get_neighbors(self):
		'''Returns list of neighbors'''
		return self.neighbors

	def get_neighbors_names(self):
		'''Returns list of neighbors names'''
		return map(lambda node: node.get_name(), self.neighbors)

class Graph():

	def __init__(self):
		self.nodeList = []

	def add_node(self, node):
		'''Adds a new node to the graph'''
		self.nodeList.append(node)

	def size(self):
		'''Returns the size of the graph (number of nodes)'''
		return len(self.nodeList)

	def clean(self):
		'''Marks each node in graph as unvisited and sets the rank to 0'''
		for node in self.nodeList:
			node.mark_unvisited()
			node.set_rank(0)

	def get_nodes(self):
		'''Returns the list of nodes'''
		return self.nodeList

	def get_node(self, nodeName):
		'''Returns a node from the node's name'''
		for node in self.nodeList:
			if node.get_name() == nodeName:
				return node

	def add_connection(self, toNode, fromNode):
		'''Adds a (directional) connection between two nodes'''
		#print("Adding connection to " + str(toNode.get_name()) + " from " + str(fromNode.get_name()))
		fromNode.add_neighbor(toNode)

	def get_node_names(self):
		'''Returns the list of node names (in the most pythonic way possible)'''
		return map(lambda node: node.get_name(), self.nodeList)

	def connected(self):
		'''returns True if all nodes have been visited'''
		for node in self.nodeList:
			if node.visited == False:
				return False
		return True

def create_graph(edge_list):
	'''Creates a graph from an edge list structured [(node0, node1), (node0, node3), (node0, node4), (node1, node0), ...]'''
	graph = Graph()

	# populate graph
	print("Populating graph...", end=' ')
	for edge in edge_list:
		nodeName = edge[0]
		neighborName = edge[1]
		# make sure both nodes are in graph already
		if nodeName not in graph.get_node_names():
			node = Node(nodeName)
			graph.add_node(node)
		else:
			node = graph.get_node(nodeName)
		if neighborName not in graph.get_node_names():
			node2 = Node(neighborName)
			graph.add_node(node2)
		else:
			node2 = graph.get_node(neighborName)
		graph.add_connection(node, node2)

	print("done. Created graph with " + str(graph.size()) + " nodes.")

	return graph

def check_graph_connectivity(graph):
	'''Checks if the graph is connected by running a DFS from each vertex'''

	count = 0.0
	status = 0.0
	for node in graph.get_nodes():

		# run DFS
		DFS(graph, node)

		# check if all nodes have been visited
		if graph.connected() == False:
			print("Node {} is not fully connected to nodes {}".format(node.name, [x.name for x in graph.get_nodes() if x.visited == False]))
			return False

	# 	print '{}% of nodes checked\r'.format(status),
	# 	status = round((count / float(graph.size()))*100, 2)
	# 	count += 1.0
	# print ""

	return True

def compute_shortest_paths(graph):
	'''Computes the shortest path from the each vertex to every other vertex in the graph, returned in a distance matrix as a dictionary'''

	# create 2D matrix to store distance values
	distance_matrix = {}

	# status = 0.0
	# count = 0.0
	for node in graph.get_nodes():
		BFS(graph, node)
		for destination in graph.get_nodes():
			distance_matrix[node.get_name(), destination.get_name()] = destination.get_rank()

			# print('{}% of paths calculated\r'.format(status), end='')
			# status = round((count / float(graph.size()*graph.size()))*100, 2)
			# count += 1.0

	return distance_matrix

def DFS(graph, v):
	'''Depth-first search.'''
	stack = []
	graph.clean()

	stack.append(v)
	while stack:
		n = stack.pop()
		if not n.is_visited():
			n.mark_visited()
			for neighbor in n.get_neighbors():
				stack.append(neighbor)

def BFS(graph, v):
	'''Breadth-first search.'''
	q = queue.Queue()
	graph.clean()
	rank = 0

	q.put(v)
	v.mark_visited()
	v.set_rank(rank)
	while not q.empty():
		n = q.get()
		for neighbor in n.get_neighbors():
			if not neighbor.is_visited():
				q.put(neighbor)
				neighbor.mark_visited()
				neighbor.set_rank(n.get_rank()+1)

def testBFS():

	edge_list = [("A", "B"),
				("A", "D"),
				("B", "A"),
				("B", "D"),
				("B", "H"),
				("C", "D"),
				("D", "A"),
				("D", "B"),
				("D", "C"),
				("D", "F"),
				("E", "F"),
				("F", "D"),
				("F", "E"),
				("F", "G"),
				("G", "F"),
				("H", "B"),
				("H", "I"),
				("I", "H")]

	graph = create_graph(edge_list)

	conn = compute_shortest_paths(graph)

	print("Nodes: {}".format( [node.get_name() for node in graph.get_nodes()] ))

	for toNode in graph.get_nodes():
		for fromNode in graph.get_nodes():
			print("{} -> {}; {} edges.".format(fromNode.get_name(),
				toNode.get_name(),
				conn[toNode.get_name(), fromNode.get_name()])
			)

def testDFS():

	edge_list = [("A", "B"),
				("A", "D"),
				("B", "A"),
				("B", "D"),
				("C", "D"),
				("D", "A"),
				("D", "B"),
				("D", "C"),
				("D", "F"),
				("E", "F"),
				("F", "D"),
				("F", "E"),
				("F", "G"),
				("G", "F")]

	graph = create_graph(edge_list)
	# print graph.get_node_names()
	# for node in graph.get_nodes():
	# 	print node.get_name(),
	# 	print node.get_neighbors_names()

	print(check_graph_connectivity(graph))

def check_connectivity():
	'''Gets the connectivity of the planning areas in database'''

	# get edge list from database
	edge_list = edge_list()

	# make graph
	graph = create_graph(edge_list)

	# get distance matrix
	return check_graph_connectivity(graph)

def get_rank_matrix(edgeList):
	'''Gets the distance matrix for the planning areas in database'''

	# get edge list from database
	# edgeList = edge_list()

	# make graph
	graph = create_graph(edgeList)

	# get distance matrix
	rank_matrix = compute_shortest_paths(graph)

	# write to local file
	print("Rank matrix is " + str(int(len(rank_matrix) ** (0.5))) + " by " + str(int(len(rank_matrix) ** (0.5))) + ", with " + str(len(rank_matrix)) + " entries.")

	print("Writing to r'../storage/rank_matrix.p'.")
	pickle.dump(rank_matrix, open(r'../storage/rank_matrix.p', 'wb') )

	return rank_matrix

def generate_fully_connected_edge_list(start, end):
	'''Generates a fully connected edge list of nodes, numbered from start to end.'''

	# create a list of N unique numbers in range
	node_list = range(start, end)
	# random.shuffle(node_list)

	# create an edge between each node and their neighbors
	edge_list = []
	for idx, node in enumerate(node_list):

		# get left and right neighbors
		neighborLeft = node_list[idx-1]
		if idx+1 >= len(node_list):
			# this is the last node; connect with the first one
			neighborRight = node_list[0]
		else:
			neighborRight = node_list[idx+1]

		# print "Node: {}".format(node)
		# print "\tLeft neighbor: {}".format(neighborLeft)
		# print "\tRight neighbor: {}".format(neighborRight)

		edge_list.append( (node, neighborLeft) )
		edge_list.append( (node, neighborRight) )

	return edge_list

def test_graph_connectivity():
	edge_list = generate_fully_connected_edge_list(0, 1000)
	graph = create_graph(edge_list)
	print("Is graph connected? (Should be True): {}".format(check_graph_connectivity(graph)), end="\n\n")

	edge_list = generate_fully_connected_edge_list(0,1000)
	island_edge_list = generate_fully_connected_edge_list(1000,1015)
	edge_list = edge_list + island_edge_list
	graph = create_graph(edge_list)
	print("Is graph connected? (Should be False): {}".format(check_graph_connectivity(graph)), end="\n\n")

def main():

	# testBFS()
	# testDFS()
	test_graph_connectivity()
	

if __name__ == '__main__':
	main()

