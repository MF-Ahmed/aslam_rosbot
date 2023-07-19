#!/usr/bin/python
#
# Julio Placed. University of Zaragoza. 2022.
# jplaced@unizar.es
import os
import networkx as nx
import numpy as np
import heapq
import scipy
import sys
python_version = sys.version_info
from constants import NMINEIG, EIG_TH



def wait_enterKey():
    input("Press Enter to continue...") #if (python_version >= (3, 0)) else raw_input("Press Enter to continue...")

def enforce_symmetry_list(A):
    A = np.array(A)
    return np.tril(A.T) + np.triu(A, 1) # returns a lowe and upper triangular matrix of A

def read_graph(options, args):
    nodes_o = None
    edges_o = None
    edges_oo = None

    if options.graph_name != 'FRH_P_toro':
        print('Not default graph: ' + format(options.graph_name))
        if options.graph_name[-3:] == 'g2o':
            options.initial_nodes = options.graph_name + "_nodes.txt"
            options.initial_edges = options.graph_name + "_edges.txt"
            options.optimized_nodes = options.graph_name + "_opt_nodes.txt"
            options.optimized_edges = options.graph_name + "_opt_edges.txt"
        elif options.graph_name[-4:] == 'toro':
            options.initial_nodes = options.graph_name + "_nodes.txt"
            options.initial_edges = options.graph_name + "_edges.txt"
            options.optimized_nodes = options.graph_name + "_opt_nodes.txt"
            options.optimized_edges = options.graph_name + "_opt_edges.txt"
    else:
        print('Default graph.')

    if options.initial_nodes != '':
        nodes_o = np.genfromtxt(options.initial_nodes, usecols=(0, 1, 2, 3))
        #print("nodes_o = {}".format(nodes_o[1]))
    if options.initial_edges != '':
        edges_o = np.genfromtxt(options.initial_edges, usecols=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11))
        #print("edges_o = {}".format(edges_o[1]))
    if options.optimized_nodes != '':
        nodes_opt = np.genfromtxt(options.optimized_nodes, usecols=(0, 1, 2, 3))
    if options.optimized_edges != '':
        edges_oo = np.genfromtxt(options.optimized_edges, usecols=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11))

    return nodes_o, edges_o, nodes_opt, edges_oo

def compute_optimality(A, e_opt='max', invert_matrix=False):
    if invert_matrix:
        A = np.linalg.pinv(A)

    eigv2 = scipy.linalg.eigvalsh(A)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")

    eigv = eigv2[eigv2 > EIG_TH]
    n = np.size(A, 1)
    t_opt = np.sum(eigv) / n
    d_opt = np.exp(np.sum(np.log(eigv)) / n)
    a_opt = n / np.sum(1. / eigv)

    if e_opt == 'min':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        return t_opt, d_opt, a_opt, e_opt
    elif e_opt == 'max':
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, tilde_opt
    elif e_opt == 'both':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, e_opt, tilde_opt

def compute_optimality_sparse(A, e_opt='max', invert_matrix=False):
    if invert_matrix: A = np.linalg.pinv(A)

    eigv2 = scipy.sparse.linalg.eigsh(A, k= min(len(A.todense())-1,6), return_eigenvectors=False)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")

    eigv = eigv2[eigv2 > EIG_TH]
    n = np.size(A, 1)
    t_opt = np.sum(eigv) / n
    d_opt = np.exp(np.sum(np.log(eigv)) / n)
    a_opt = n / np.sum(1. / eigv)

    if e_opt == 'min':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        return t_opt, d_opt, a_opt, e_opt
    elif e_opt == 'max':
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, tilde_opt
    elif e_opt == 'both':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, e_opt, tilde_opt

def build_fullFIM(graph):
    graph_nodes = nx.number_of_nodes(graph.graph)
    graph_edges = nx.number_of_edges(graph.graph)
    print("graph has {} nodes and {} edges" .format(graph_nodes, graph_edges))
    dim = 3
    A = np.zeros(((graph_nodes) * dim, (graph_nodes) * dim))

    for i in range(0, graph_nodes):
        edge_Info = graph.graph.edges([i], 'information')
        #print("edge_Info = {}".format(edge_Info))
        for (id1, id2, fisher) in edge_Info:
            node1 = int(id1)
            node2 = int(id2)
            #print("node1 = {}".format(node1))
            #print("node2 = {}".format(node2))

            if node2 > node1:
                FIM = fisher
                FIM = np.array(FIM)
                #print("A[(node2) * dim:(node2 + 1) * dim, (node2) * dim:(node2 + 1) * dim] = {}".format(A[(node2) * dim:(node2 + 1) * dim, (node2) * dim:(node2 + 1) * dim]))
                A[(node2) * dim:(node2 + 1) * dim, (node2) * dim:(node2 + 1) * dim] += FIM
                A[(node1) * dim:(node1 + 1) * dim, (node1) * dim:(node1 + 1) * dim] += FIM
                #print("A = {}".format(A))
                A[(node1) * dim:(node1 + 1) * dim, (node2) * dim:(node2 + 1) * dim] = - FIM
                A[(node2) * dim:(node2 + 1) * dim, (node1) * dim:(node1 + 1) * dim] = - FIM
    diff = A - A.T
    if not np.all(np.abs(diff.data) < 1e-8):
        print("Error in build_fullFIM: Fisher Information matrix is not symmetric.")
    return A
def save_value_to_txt(value, filename):
    filepath = os.path.join(os.getcwd(), filename)
    with open(filename, 'w') as file:
        file.write(str(value))
    print(f"Value {value} saved to {filename}.")

def plot_data_from_txt(filename):
    filepath = os.path.join(os.getcwd(), filename)
    with open(filepath, 'r') as file:
        value = float(file.read())
        return value

