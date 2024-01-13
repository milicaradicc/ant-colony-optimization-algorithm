import re
import networkx as nx
import math
import random

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
        x1 = float(data[node]['x'])
        y1 = float(data[node]['y'])
        if not G.has_node(node):
            # if not in graph add it and its neighbors
            G.add_node(node,x=x1,y=y1)
            for neighbor in data[node]['neighbors']:
                x2 =float(data[neighbor]['x'])
                y2 = float(data[neighbor]['y'])
                if not G.has_node(neighbor):
                    G.add_node(neighbor,x=x2,y=y2)
                # we need to add egde
                # first calculate distance
                G.add_edge(node,neighbor,distance = calculate_distance(x1,y1,x2,y2),pheromones=10)
    return G

def calculate_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

id_start = 1675129468
id_end = 1517125651
m = 30
iterations = 100
alpha = 1
beta = 2
gamma = 2
Q = 50
rho = 0.8 #persistence coefficient



def algorithm(id_start,id_end):
    data = read_data("data_path_nodes.txt")
    graph = create_graph(data)
    node= graph.nodes[id_start]
    print(node)
    node_end, attr = graph.nodes[id_end]  
    for iter in range(iterations):
        paths= []
        for ant in range(m):
            path = {'nodes':[],'distance':0}
            path['nodes'].append(node)
            while node != graph.nodes[id_end]:
                print(graph.neighbors(id_start))
                neighbors = graph.neighbors(id_start)
                # ceo cvor sa id x i y
                node, distance = get_next_node(node,neighbors,graph,node_end)
                path['nodes'].append(node)
                path['distance'] += distance
            paths.append(path)
        update_pheromones(graph,paths)
    return paths[0]
     
def update_pheromones(graph,paths):
    paths = sorted(paths, key=lambda x: x['distance'])[:10]
    delta = {}
    for path in paths:
        if  not delta[nodes[i]][nodes[i+1]]:
             delta[nodes[i]][nodes[i+1]] = {'pher':0, 'status':0}
        nodes = path['nodes']
        for i in range(len(nodes)-1):
            edge = graph.edges[nodes[i], nodes[i+1]]
            delta[nodes[i]][nodes[i+1]] += Q / edge['distance']
    for path in paths:
        nodes = path['nodes']
        for i in range(len(nodes)-1):
            if  delta[nodes[i]][nodes[i+1]]['status'] == 1:
                continue
            pheromones = graph.edges[nodes[i], nodes[i+1]]['pheromones']
            pheromones = rho * pheromones + delta[nodes[i]][nodes[i+1]]
    

def get_probabilities(node,neighbors,graph,node_end):
    probabilities = {}
    sum = 0
    for neighbor in neighbors:
        edge = graph.edges[node, neighbor]
        probabilities[neighbor] = get_probability_numerator(edge,node,neighbor,node_end)
        sum += probabilities[neighbor]
    sum_of_probabilities = 0
    for neighbor in probabilities:
        probabilities[neighbor] /= sum
        sum_of_probabilities += probabilities[neighbor]
    return probabilities , sum_of_probabilities
    
def get_probability_numerator(edge,node,neighbor,node_end):
    return (edge['pheromones']**alpha) * (1/edge['distance']**beta) * (1/(1+get_teta(node['x'],node['y'],neighbor['x'],neighbor['y'],node_end)))**gamma

def get_teta(x1,y1,x2,y2,node_end):
    k1 = (y2 - y1)/(x2 - x1)
    k2 = (node_end['y'] - y1)/(node_end['x'] - x1)
    return math.atan((k1 - k2)/(1 + k1*k2))
    
def get_next_node(node,neighbors,graph,node_end):
    # 0.1 0.2 0.3
    # sum = 0.6
    
    # 0.17 0.33 0.5
    # 0 - 0.17
    # 0.18 - 0.5
    # 0.51 - 1
    # random [0,1] = 0.24
    random_number = random.random()
    probabilities,sum = get_probabilities(node,neighbors,graph,node_end)
    probabilities = {k: v for k, v in sorted(probabilities.items(), key=lambda item: item[1]/sum)}
    for neighbor in probabilities:
        if probabilities[neighbor] >= random_number:
            break
    return neighbor, graph.edges[node, neighbor]['distance']


def main():
    algorithm(3653296222, 3653134376)

if __name__ == '__main__':
    main()