import networkx as nx

def compute_normalized_tree_connectivity(graph):
    # Step 1: Compute the minimum spanning tree
    mst = nx.minimum_spanning_tree(graph)

    # Step 2: Calculate the sum of edge weights in the MST
    total_weight = sum(graph.get_edge_data(*e)['weight'] for e in mst.edges())

    # Step 3: Compute shortest path trees and calculate the sum of edge weights
    shortest_path_weights = []
    for node in graph.nodes():
        path_lengths = nx.single_source_dijkstra_path_length(graph, node)
        shortest_path_tree = nx.Graph()
        for v, length in path_lengths.items():
            if v != node:
                path = nx.shortest_path(graph, source=node, target=v)
                shortest_path_tree.add_edges_from(zip(path, path[1:]))
        weight = sum(graph.get_edge_data(*e)['weight'] for e in shortest_path_tree.edges())
        shortest_path_weights.append(weight)

    # Step 4: Compute the normalized tree connectivity
    n = graph.number_of_nodes()
    ntc = (total_weight - sum(shortest_path_weights)) / (n * (n - 1))
    return ntc

# Create a sample graph
G = nx.Graph()
G.add_edge('A', 'B', weight=2)
G.add_edge('B', 'C', weight=3)
G.add_edge('C', 'D', weight=4)
G.add_edge('D', 'E', weight=5)
G.add_edge('E', 'A', weight=1)

# Compute normalized tree connectivity
ntc = compute_normalized_tree_connectivity(G)
print("Normalized Tree Connectivity:", ntc)
