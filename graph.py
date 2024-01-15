import re
import networkx as nx
import math
import random
import time
from collections import Counter

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
    G = nx.Graph()
    for node in data:
        x1 = float(data[node]['x'])
        y1 = float(data[node]['y'])
        
        if not G.has_node(node):
            G.add_node(node, id=node, x=x1, y=y1, status = 0)
            
        for neighbor in data[node]['neighbors']:
            x2 = float(data[neighbor]['x'])
            y2 = float(data[neighbor]['y'])
            
            if not G.has_node(neighbor):
                G.add_node(neighbor, id = neighbor, x=x2, y=y2, status=0)
                
            G.add_edge(node, neighbor, distance=calculate_distance(x1, y1, x2, y2), pheromones=10)
    return G

def calculate_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

m = 30
iterations = 1
alpha = 1
beta = 2
gamma = 2
Q = 50
rho = 0.8 #persistence coefficient


def algorithm(id_start,id_end):
    with open('nodes.txt', 'w') as f:
        data = read_data("data_path_nodes.txt")
        graph = create_graph(data)
        node= graph.nodes[id_start]
        node_end = graph.nodes[id_end] 
        opt_paths = [] 
        for iter in range(iterations):
            paths= []
            number=[]
            for ant in range(m):
                print(ant)

                path = {'nodes':[],'distance':0}
                path['nodes'].append(node)
                i = 0
                while node != graph.nodes[id_end]:
                    print(i)
                    i+=1
                    if len(path['nodes']) >= 3 and path['nodes'][-1]['id'] == path['nodes'][-2]['id']:
                        print('')
                    neighbors =  list(graph.neighbors(node['id']))
                    if graph.nodes[id_end] in neighbors:
                        path['nodes'].append(graph.nodes[id_end])
                        path['distance'] += graph.edges[path['nodes'][-1]['id'], id_end]['distance']
                        break

                    # ceo cvor sa id x i y
                    next_node_id, distance = get_next_node(node,neighbors,graph,node_end)
                    if len(list(graph.neighbors(next_node_id)))==1:
                        while len(list(graph.neighbors(next_node_id)))==1:
                            for neighbor in neighbors:
                                print(neighbor)
                                if neighbor != next_node_id and len(list(graph.nodes(neighbor)))!=1:
                                    next_node_id = neighbor
                                    node = graph.nodes[next_node_id]
                                    path['nodes'].append(node)
                                    path['distance'] += graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                    break
                            path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                            del path['nodes'][-1]
                            next_node_id = path['nodes'][-1]['id']

                    elif len(path['nodes']) >= 4:
                        if graph.nodes[next_node_id] == path['nodes'][-2]:
                            if len(neighbors) != 1:
                                while True:
                                    found = 0
                                    for neighbor in neighbors:
                                        if graph.nodes[id_end] in neighbors:
                                            path['nodes'].append(graph.nodes[id_end])
                                            path['distance'] += graph.edges[path['nodes'][-1]['id'], id_end]['distance']
                                            break

                                        next_node_id = graph.nodes[neighbor]['id']
                                        if next_node_id != path['nodes'][-2]['id'] and next_node_id != path['nodes'][-3]['id']:
                                            node = graph.nodes[next_node_id]
                                            path['nodes'].append(node)
                                            path['distance'] += graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']

                                            found = 1
                                            break
                                    if not found:
                                        path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                        del path['nodes'][-1]
                                    break
                            else:
                                path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                del path['nodes'][-1]
                                # path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                # del path['nodes'][-1]
                                print(len(list(graph.neighbors(path['nodes'][-1]['id']))))
                                print(len(list(graph.neighbors(path['nodes'][-1]['id'])))==1)
                                while len(list(graph.neighbors(path['nodes'][-1]['id'])))==1:
                                    path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                    del path['nodes'][-1]
                                    i+=1
                                next_node_id = path['nodes'][-1]['id']
                                node = graph.nodes[next_node_id]
                        elif(graph.nodes[next_node_id] == path['nodes'][-3]):
                            print(path['nodes'][-1]['id'])
                            neighbors = list(graph.neighbors(path['nodes'][-1]['id']))
                            if len(neighbors) == 1:
                                path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                del path['nodes'][-1]
                                i = 0
                                while len(list(graph.neighbors(path['nodes'][-1-i]['id'])))==1:
                                    path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                    del path['nodes'][-1]
                                    i+=1
                                next_node_id = path['nodes'][-4-i]['id']
                                node = graph.nodes[next_node_id]



                            else:
                                for neighbor in neighbors:
                                    if graph.nodes[id_end] in neighbors:
                                        path['nodes'].append(graph.nodes[id_end])
                                        path['distance'] += graph.edges[path['nodes'][-1]['id'], id_end]['distance']
                                        break

                                    next_node_id = graph.nodes[neighbor]['id']
                                    if next_node_id != path['nodes'][-2]['id'] :
                                        if  next_node_id != path['nodes'][-4]['id']:
                                            node = graph.nodes[next_node_id]
                                            path['nodes'].append(node)
                                            path['distance'] += graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']

                                            break
                                        else:
                                            i = 5
                                            while len(list(graph.neighbors(path['nodes'][-i]['id'])))==1:
                                                path['distance'] -=  graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                                                del path['nodes'][-1]
                                                i+=1
                                            next_node_id = path['nodes'][-1]['id']
                                            node = graph.nodes[next_node_id]
                                            path['nodes'].append(node)
                                            path['distance'] += graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']

                                            break
                        else:
                            node = graph.nodes[next_node_id]
                            path['nodes'].append(node)
                            path['distance'] += distance
                    else:
                        node = graph.nodes[next_node_id]
                        path['nodes'].append(node)
                        path['distance'] += distance
                    number.append(next_node_id)
                    f.write(str(next_node_id)+'\n')
                    if i == 1000:
                        number_counts = Counter(number)

                        for number, count in number_counts.items():
                            print(f"{number}: {count} times")
                        return
                            

                    #         if len(new_next_node_ids) == 0:
                    #             num_ret_steps = 2
                    #             # if num_returns > 2 and len(new_next_node_ids) > 10:
                    #             #     num_ret_steps = 10
                    #             node_id = path['nodes'][-1-num_ret_steps]['id']
                    #             node = graph.nodes[node_id]
                    #             for i in range(num_ret_steps):
                    #                 path['distance'] -= graph.edges[path['nodes'][-1]['id'], path['nodes'][-2]['id']]['distance']
                    #                 del path['nodes'][-1]
                    #             num_returns += 1
                    #             continue
                    #         if len(new_next_node_ids) == 1:
                    #             next_node_id, distance = new_next_node_ids[0], graph.edges[node['id'], new_next_node_ids[0]]['distance']
                    #         else:
                    #             next_node_id, distance = get_next_node(node,new_next_node_ids,graph,node_end)
                    
                    
                paths.append(path)
                node= graph.nodes[id_start]
            if paths:
                opt_paths.append(update_pheromones(graph,paths)[0])
            
    return opt_paths[len(opt_paths)-1]
    
def check_for_stop(opt_paths):
    number_of_opt_paths = len(opt_paths)
    if number_of_opt_paths >= 10:
        for i in range(1,11):
            if opt_paths[number_of_opt_paths-i] != opt_paths[number_of_opt_paths-i-1]:
                return False
        return True
    return False
     
def update_pheromones(graph,paths):
    paths = sorted(paths, key=lambda x: x['distance'])[:10]
    delta = {}
    for path in paths:
        nodes = path['nodes']
        for i in range(len(nodes)-1):
            if (nodes[i]['id'] not in delta):
                delta[nodes[i]['id']] = {}
            if (nodes[i+1]['id'] not in  delta[nodes[i]['id']]):
                delta[nodes[i]['id']][nodes[i+1]['id']] = {'pher':0, 'status':0}
            edge = graph.edges[nodes[i]['id'], nodes[i+1]['id']]
            delta[nodes[i]['id']][nodes[i+1]['id']]['pher'] += Q / edge['distance']
            
    for path in paths:
        nodes = path['nodes']
        for i in range(len(nodes)-1):
            if  delta[nodes[i]['id']][nodes[i+1]['id']]['status'] == 1:
                continue
            pheromones = graph.edges[nodes[i]['id'], nodes[i+1]['id']]['pheromones']
            pheromones = rho * pheromones + delta[nodes[i]['id']][nodes[i+1]['id']]['pher']
            delta[nodes[i]['id']][nodes[i+1]['id']]['status'] = 1

    return paths

def get_probabilities(node,neighbors,graph,node_end):
    probabilities = {}
    sum = 0
    for neighbor in neighbors:
        neighbor = graph.nodes[neighbor]
        neighbor_id = neighbor['id']
        edge = graph.edges[node['id'],neighbor_id]
        pheromones = edge['pheromones']
        distance = edge['distance']

        probabilities[neighbor_id] = get_probability_numerator(pheromones,distance,node,neighbor,node_end)
        sum += probabilities[neighbor_id]
    for neighbor_id in probabilities:
        probabilities[neighbor_id] /= sum
    return probabilities
    
def get_probability_numerator(pheromones,distance,node,neighbor,node_end):
    return (pheromones**alpha) * ((1/distance)**beta) * ((1/(1+get_teta(node['x'],node['y'],neighbor['x'],neighbor['y'],node_end)))**gamma)

def get_teta(x1,y1,x2,y2,node_end):
    if x1 == x2:
        return 0
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
    random_number = random.uniform(0.2, 1.0)
    probabilities = get_probabilities(node,neighbors,graph,node_end)
    probabilities = {k: v for k, v in sorted(probabilities.items(), key=lambda item: item[1])}
    last = 0
    current = 0
    for neighbor_id in probabilities:
        current = probabilities[neighbor_id] + last
        if current >= random_number:
            break
        else:
            last = current
    return neighbor_id, graph.edges[node['id'], neighbor_id]['distance']

def main():
    pocetno = time.time()
    path = algorithm(3653296222, 3653134376)
    krajnje = time.time()

    print("OPTIMALNA PUTANJA : ",len(path['nodes']))
    print('DISTANCA: ', path['distance'])
    print('VREME: ', krajnje - pocetno)

if __name__ == '__main__':
    main()