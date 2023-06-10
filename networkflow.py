import collections
import math

#Question 1
class Graph:
    def __init__(self, v: tuple, maxIn: list[int], maxOut:list[int], targets:list[int]) -> None:
        """
        This is the initialization function for a graph class that creates vertices and edges based on
        input parameters. If there are more than 1 target, we create a super node that connects to every target.
        
        :param v: a list of tuples representing edges in the graph, where each tuple contains the IDs of
        the two vertices connected by the edge and the weight of the edge
        :type v: tuple
        :param maxIn: The `maxIn` parameter is a list of integers representing the maximum number of
        incoming capacity that each vertex in the graph can have
        :type maxIn: list[int]
        :param maxOut: The `maxOut` parameter is a list of integers representing the maximum number of
        outgoing capacity allowed for each vertex in the graph
        :type maxOut: list[int]
        :param targets: The `targets` parameter is a list of integers representing the vertices in the
        graph that needs to be reached
        :type targets: list[int]

        Time complexity = O(n) where n is the length of the tuple
        Aux Space complexity = O(V) where V is the length of the tuple
        """
        self.maxIn = maxIn
        self.maxOut = maxOut
        self.targets = targets
        max_id = max(max(pair[0],pair[1]) for pair in v)
        self.vertices = [None] * (max_id + 1)
        for i in range(len(v)):
            self.vertices[v[i][0]] = Vertex(v[i][0])
            self.vertices[v[i][1]] = Vertex(v[i][1])
        for i in range(len(v)):
            self.add_edge(self.vertices[v[i][0]], self.vertices[v[i][1]], v[i][2])
        
        if len(self.targets) > 0:
            self.vertices.append(Vertex(len(self.vertices)))

    def __len__(self) -> int:
        return len(self.vertices)
    
    def __str__(self) -> str:
        output = ""
        for vertex in self.vertices:
            output += "Vertex = " + str(vertex) + "\n" + " Neighbours " + ", ".join(str(edge) + " weight = "+str(edge.max_through) for edge in vertex.edges) + "\n"
        return output

    
    def add_edge(self, source: 'Vertex', dest: 'Vertex', max_through: int):
        source.edges.append(Edge(dest, max_through))
    
    
    def setFlow(self, connections: list[tuple[int, int, int]]):
        """
        The function sets the flow of a network based on the given connections and maximum capacities.
        We are building a proper residual network here using the information of every vertex's maxIn and maxOut and the 
        connection weight to properly set up the flow
        
        :param connections: The `connections` parameter is a list of tuples, where each tuple represents
        a connection between two vertices in a graph. Each tuple contains three integers: the index of
        the source vertex, the index of the destination vertex, and the maximum flow capacity of the
        connection.
        :type connections: list[tuple[int, int, int]]

        Time complexity = O(n) where n is the length of the tuple
        Aux Space complexity = O(V) where V is the length of the tuple
        """
        if len(self.targets) > 1:
            for i in self.targets:
                self.add_edge(self.vertices[i], self.vertices[-1],self.maxIn[i] )
        for i in connections:
            curr_edge = self.vertices[i[0]].getEdgeFromID(i[1])
            if curr_edge != False:
                capacity = curr_edge.max_through
                cap = min(self.maxOut[i[0]], self.maxIn[i[1]], capacity)
                self.vertices[i[0]].getEdgeFromID(i[1]).max_through = cap
                self.maxOut[i[0]] -= cap
                self.maxIn[i[1]] -= cap

    def initBackwards(self, connections: list[tuple[int, int, int]]):
        """
        This function initializes the current graph with reverse connections withedges to the graph.
        
        :param connections: The `connections` parameter is a list of tuples, where each tuple represents
        a directed edge in the graph. The first element of the tuple is the starting vertex, the second
        element is the ending vertex, and the third element is the weight of the edge
        :type connections: list[tuple[int, int, int]]

        Time complexity = O(n) where n is the length of the tuple
        Aux Space complexity = O(V) where V is the length of the tuple
        """
        for i in connections:
            self.add_edge(self.vertices[i[1]], self.vertices[i[0]], 0)
        if len(self.targets) > 0:
            for i in self.targets:
                self.add_edge(self.vertices[-1], self.vertices[i], 0)

    def bfs_path(self, start_id: int, target_id: int):
        """
        This is a breadth-first search algorithm that finds the shortest path between two vertices in a
        graph.
        
        :param start_id: The ID of the starting vertex for the breadth-first search algorithm
        :type start_id: int
        :param target_id: The target_id parameter is an integer representing the ID of the vertex that
        we want to find a path to in the breadth-first search algorithm
        :type target_id: int
        :return: a list of vertices representing the shortest path from the start vertex to the target
        vertex using the breadth-first search algorithm. If the target vertex is not found, an empty
        list is returned.

        Time Complexity:
        O(|E| + |V|) where |V| and |E| is the set of vertices(data centers) and edges(communication channels) respectively.
        Aux Space Complexity:
        O(|V|) where |V| is the set of vertices(data centers)
        """
        start = self.vertices[start_id]
        target = self.vertices[target_id]
        queue = collections.deque()
        queue.append(start)
        start.visited = True

        while queue:
            current_vertex = queue.popleft()

            if current_vertex == target:
                path = [current_vertex]
                current = current_vertex
                while current != start:
                    if current == None:
                        return list(reversed(path))
                    path.append(current.previous)
                    current = current.previous
    
                return list(reversed(path))
            
            for edge in current_vertex.edges:
                next_vertex = edge.dest
                if not next_vertex.visited and edge.max_through > 0:
                    queue.append(next_vertex)
                    next_vertex.visited = True
                    next_vertex.previous = current_vertex

        # Target vertex not found
        return []

    def updateEdge(self, path: list['Vertex'], change: int):
        """
        This function updates the maximum flow through edges in a given path and removes edges that are
        no longer available, Both the normal edges and the reverse edges
        
        :param path: A list of Vertex objects representing a path in a graph
        :type path: list['Vertex']
        :param change: The `change` parameter is an integer value that represents the amount by which
        the maximum flow through the edges in the given `path` needs to be updated. It can be positive
        or negative depending on whether the flow needs to be increased or decreased
        :type change: int

        Time complexity = O(n) where n is the length of the path (changing the edges between them)
        Aux Space complexity = O(V) where V is the length of the path
        """
        for i in range(len(path) - 1):
            vertex = path[i]
            edge = vertex.getEdgeFromID(path[i+1].id)
            # print(vertex, path[i+1])
            edge.max_through -= change
            if edge.max_through <= 0:
                edge.available = False
                for i in vertex.edges:
                    if i == edge:
                        vertex.edges.remove(i)
        
        rev = list(reversed(path))
        for i in range(len(rev) - 1):
            vertex = rev[i]
            backedge = vertex.getEdgeFromID(rev[i+1].id)
            backedge.max_through += change
            if backedge.max_through > 0:
                backedge.available = True
            else:
                for i in vertex.edges:
                    if i == edge:
                        vertex.edges.remove(i)
    
    def ford_fulkerson(self, start: int, finish: int):
        """
        The function implements the Ford-Fulkerson algorithm to find the maximum flow in a network flow
        graph.

        In this method we take the residual network in this class, we keep updating the edges in the augmented paths.
        in every path we found, we get the minimum capacity and increment it to our output. and before every new path is found
        we reset the previous vertex and the visited boolean of every vertex in the graph
        
        :param start: The starting node for the Ford-Fulkerson algorithm to find the maximum flow in a
        flow network
        :type start: int
        :param finish: The "finish" parameter in the "ford_fulkerson" function is an integer
        representing the node in the graph where the flow needs to be maximized. This is the destination
        node or the sink node in the graph
        :type finish: int
        :return: the maximum flow that can be sent from the source node (specified by the `start`
        parameter) to the sink node (specified by the `finish` parameter) using the Ford-Fulkerson
        algorithm.

        Time Complexity:  O(|V|*|E|^2) is the set of vertices(data centers) and edges(communication channels) respectively.
        Space Complexity:  O(|V|*|E|) is the set of vertices(data centers) and edges(communication channels) respectively.
        """
        flow = 0
        while self.has_augmenting_path(start, finish) != False:
            self.resetStatus()
            path = self.bfs_path(start, finish)
            minimum = self.get_minimum(path)
            flow += minimum
            self.updateEdge(path, minimum)
            self.resetStatus()
        return flow
    
    def has_augmenting_path(self, source: int, target: int) -> bool:
        """
        This function checks if there is an augmenting path between a source and target node in a graph
        using a breadth-first search algorithm.
        
        :param source: The starting node for the path search
        :type source: int
        :param target: The "target" parameter in the given code refers to the node in the graph to which
        we want to find an augmenting path from the source node. An augmenting path is a path in a flow
        network that starts from the source node and ends at the target node, and has the property that
        :type target: int
        :return: a boolean value. If there exists a path from the source node to the target node in the
        graph, the function returns True. Otherwise, it returns False.

        Time Complexity:
        O(|E| + |V|) where |V| and |E| is the set of vertices(data centers) and edges(communication channels) respectively.
        Aux Space Complexity:
        O(|V|) where |V| is the set of vertices(data centers)

        The complexities follows bfs because it only runs bfs in this function
        """
        path = self.bfs_path(source, target)
        if len(path) > 0:
            return True
        else:
            return False
    
    def resetStatus(self):
        """
        The function resets the "previous" and "visited" attributes of all vertices in a graph.

        Time complexity = O(n) where n is the amount of vertices in the graph
        """
        for vertex in self.vertices:
            vertex.previous = None
            vertex.visited = False

    
    def get_minimum(self, path: list['Vertex']):
        """
        This function calculates the minimum value of the maximum capacity of edges in a given path.
        
        :param path: The `path` parameter is a list of `Vertex` objects representing a path in a graph.
        Each `Vertex` object has an `id` attribute that uniquely identifies it in the graph. The
        `get_minimum` method calculates the minimum weight of the edges in the path by iterating over
        the vertices
        :type path: list['Vertex']
        :return: The function `get_minimum` is returning the minimum value of the `max_through`
        attribute of the edges in the given path, which is a list of `Vertex` objects. The minimum value
        is calculated by iterating over the edges in the path and comparing their `max_through`
        attribute to the current minimum value. The function returns the final minimum value found.

        Time Complexity: O(|D|) |D| is the path of vertices
        Aux Space Complexity: O(1)
        """
        minimum = math.inf
        for i in range(len(path) - 1):
            vertex = self.vertices[path[i].id]
            edge = vertex.getEdgeFromID(path[i + 1].id)
            if edge.max_through < minimum:
                minimum = edge.max_through
        return minimum
    
# The Vertex class represents a vertex in a graph and contains various attributes such as edges,
# visited status, previous vertex, distance, cycle status, and change.
class Vertex:
    def __init__(self, id:int) -> None:
        """
        This is a constructor function that initializes an object with an ID, an empty list of edges, a
        boolean value for visited status, and a previous attribute set to None.
        
        :param id: The "id" parameter is an integer that represents the unique identifier of a node in a
        graph. Each node in a graph must have a unique id
        :type id: int
        """
        self.id = id
        self.edges = []
        self.visited = False
        self.previous = None
    
    def __str__(self) -> str:
        output = str(self.id)
        return output

    def getEdgeFromID(self, id: int):
        """
        This function searches for an edge in the current vertex in a list of edges based on a given destination ID.
        
        :param id: The parameter "id" is an integer representing the ID of a node in a graph. The
        function "getEdgeFromID" searches through the edges of the graph to find the edge that connects
        the node with the given ID to another node. If such an edge is found, it is returned
        :type id: int
        :return: either an edge object or a boolean value (False). If an edge object with a destination
        node ID matching the input ID is found in the list of edges, it will be returned. If no such
        edge is found, the function will return False.

        Time Complexity: O(n) where n is the number of edges in a specific vertex
        """
        for i in self.edges:
            if i.dest.id == id:
                return i
        return False

class Edge:
    def __init__(self, dest: 'Vertex', capacity: int):
        """
        This is a constructor function for a class that initializes the destination vertex and capacity of
        an object, and sets its availability to True.
        
        :param dest: The `dest` parameter is a reference to a `Vertex` object, which represents the
        destination vertex of an edge
        :type dest: 'Vertex'
        :param capacity: The capacity parameter is an integer that represents the maximum amount of flow
        that can be sent through this edge. In a graph, an edge can be thought of as a connection between
        two vertices, and the capacity of an edge determines how much flow can be sent through it
        :type capacity: int
        """
        self.dest = dest
        self.capacity = capacity
        self.available = True
    
    def __str__(self) -> str:
        output = "dest = " +  str(self.dest)
        return output

def maxThroughput(connections: list[tuple[int, int, int]], maxIn: int, maxOut: int, origin: int, targets: list[int]):
    """
    This function calculates the maximum flow through a graph given a set of connections, maximum input
    and output capacities, an origin node, and a list of target nodes using the Ford-Fulkerson
    algorithm.
    
    Approach Description:
        Firstly, we create a graph from all the information we have. We create a super node for the targets to attach to.
        After that, we update the graph using setFlow to update all the edges using the bottlenecks we have from 
        maxIn: int and maxOut: int. After that is done, We create reverse edges using the information in the connections
        tuple. 

        With all that done, we already have our residual network. Then we can use the ford fulkerson algorithm to get our maximum
        flow. 
        In ford-fulkerson we take the residual network in this class, we keep updating the edges in the augmented paths.
        in every path we found, we get the minimum capacity and increment it to our output. and before every new path is found
        we reset the previous vertex and the visited boolean of every vertex in the graph.

    :param connections: A list of tuples representing the connections between vertices in the graph.
    Each tuple contains three integers: the source vertex, the destination vertex, and the weight of the 
    edge between them
    :type connections: list[tuple[int, int, int]]
    :param maxIn: The maximum amount of flow that can enter a vertex in the graph
    :type maxIn: int
    :param maxOut: maxOut is a parameter that specifies the maximum outgoing flow from a vertex in the
    graph. In other words, it limits the amount of flow that can leave a vertex and go to its
    neighboring vertices. This parameter is used in the construction of the graph and in the
    implementation of the Ford-Fulkerson
    :type maxOut: int
    :param origin: The starting node in the graph from which the maximum flow needs to be calculated
    :type origin: int
    :param targets: The `targets` parameter is a list of integers representing the destination nodes in
    the graph. The `maxThroughput` function takes in a list of connections, which are tuples of three
    integers representing the source node, destination node, and maximum flow capacity between them. The
    `maxIn` and `
    :type targets: list[int]
    :return: The function `maxThroughput` returns the maximum flow that can be sent from the `origin`
    vertex to the vertices in the `targets` list, given the `connections` list, `maxIn` and `maxOut`
    capacities for each vertex, using the Ford-Fulkerson algorithm.
    """
    graph = Graph(connections, maxIn, maxOut, targets)
    graph.setFlow(connections)
    graph.initBackwards(connections)
    output = graph.ford_fulkerson(origin, graph.vertices[-1].id)
    return output
