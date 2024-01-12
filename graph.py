import re
import networkx as nx
import math

def read_data(filename):
    data = {}
    with open(filename, "r") as f:
        while True:
            line = f.readline()
            if not line:
                break
            line = re.split('[,():]', line)
            neighbors = line[4:]
            for i in range(len(neighbors)):
                neighbors[i] = int(neighbors[i])
            data[int(line[0])] = {"x": float(line[1]), "y": float(line[2]), "neighbors": neighbors}
            
    return data

def create_graph(data):
    # create graph
    G = nx.Graph()
    # get id and check if in graph
    for node in data:
        x1 = data[node]['x']
        y1 = data[node]['y']
        if not G.has_node(node):
            # if not in graph add it and its neighbors
            G.add_node(node)
            for neighbor in data[node]['neighbors']:
                x2 = data[neighbor]['x']
                y2 = data[neighbor]['y']
                if not G.has_node(neighbor):
                    G.add_node(neighbor)
                # we need to add egde
                # first calculate distance
                G.add_edge(node,neighbor,weight = calculate_distance(x1,y1,x2,y2))
    return G
def calculate_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)
    
graph = create_graph(read_data("data_path_nodes.txt"))
