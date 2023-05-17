import networkx as nx
import numpy as np
from scipy.linalg import eigvals

# Create a sample graph
G = nx.Graph()
G.add_edges_from([(1, 2), (2, 3), (3, 4), (4, 1)])

# Compute the Laplacian matrix
L = nx.laplacian_matrix(G).toarray()

# Calculate the eigenvalues of the Laplacian matrix
eigenvalues = eigvals(L)

# Sort the eigenvalues in ascending order
sorted_eigenvalues = np.sort(eigenvalues)

# Get the second smallest eigenvalue, which is the algebraic connectivity
algebraic_connectivity = sorted_eigenvalues[1]

print("Algebraic Connectivity:", algebraic_connectivity)
