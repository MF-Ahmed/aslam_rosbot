import networkx as nx
import numpy as np

'''
The number of spanning trees of a graph can be computed using Kirchhoff's theorem, 
which relates the number of spanning trees to the Laplacian matrix of the graph.
Here's an approach to compute the number of spanning trees of a graph in Python
'''


# Create a sample graph
G = nx.Graph()
G.add_edges_from([(1, 2), (2, 3), (3, 4), (4, 1)])

# Compute the Laplacian matrix
L = nx.laplacian_matrix(G).toarray()

# Remove the last row and column to obtain the reduced Laplacian matrix
L_reduced = np.delete(np.delete(L, -1, axis=0), -1, axis=1)

# Compute the number of spanning trees using Kirchhoff's theorem
num_spanning_trees = int(np.linalg.det(L_reduced))

print("Number of Spanning Trees:", num_spanning_trees)