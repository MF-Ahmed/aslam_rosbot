import networkx as nx

# Create a sample graph
G = nx.Graph()
G.add_edges_from([(1, 2), (2, 3), (3, 4), (4, 1), (2,4)])

# Compute the average degree
degrees = dict(G.degree())
print(G.degree())
print(degrees)
print(degrees.values)
print(G.number_of_nodes())
avg_degree = sum(degrees.values()) / G.number_of_nodes()

print("Average Degree:", avg_degree)