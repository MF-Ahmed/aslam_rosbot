#!/usr/bin/python3
#
# Julio Placed. University of Zaragoza. 2022.
# jplaced@unizar.es
import os
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import heapq
import scipy
import utils as ut
from operator import itemgetter
from constants import NMINEIG, EIG_TH
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class weighted_pose_graph:
    def __init__(self, nodes=None, edges=None, criteria='d_opt'):
        self.graph = nx.Graph()
        filepath_opti = os.path.join(os.getcwd(), 'd-opt.txt')
        filepath_poses = os.path.join(os.getcwd(), 'poses.csv')
        with open(filepath_opti, 'w') as file:
            pass
        with open(filepath_poses, 'w') as file:
            pass

        if (nodes is not None) and (edges is not None):

            for i in range(0, np.size(nodes, 0)):
                p = [nodes[i][1], nodes[i][2], nodes[i][3]]
                self.graph.add_node(nodes[i][0], pose=p, theta=nodes[i][3])

                with open(filepath_poses, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([str(p[0]),str(p[1]),str(p[2])])

            for i in range(0, np.size(edges, 0)):
                edge = (edges[i][0], edges[i][1])
                delta = edges[i][3:6]
                I = edges[i][6:12]
                A = [[I[0], I[1], I[2]],
                     [I[1], I[3], I[4]],
                     [I[2], I[4], I[5]]]
                #print("edge  = {}, delta  = {}, I  = {}, A  = {}".format(edge,delta,I,A))

                A = ut.enforce_symmetry_list(A)
                eigv2 = np.linalg.eigvals(A)
                eigv = eigv2[eigv2 > EIG_TH]
                n = np.size(A, 1)
                if criteria == 'd_opt':
                    opt_cri = np.exp(np.sum(np.log(eigv)) / n)
                    with open(filepath_opti, 'a') as file:
                        file.writelines('{:.3f}'.format(opt_cri)+'\r')
                elif criteria == 'e_opt':
                    opt_cri = heapq.nsmallest(NMINEIG, eigv)[-1]
                elif criteria == 't_opt':
                    opt_cri = np.sum(eigv) / n
                elif criteria == 'tilde_opt':
                    opt_cri = np.max(eigv)
                #print("edges[i][2]  = {}".format(edges[i][2]))
                self.graph.add_edge(*edge, type=edges[i][2], delta=delta, information=A, weight=opt_cri)  # edge[i][2] loop closure
            #with open(filepath_opti, 'r') as file:
                #content = file.readlines()
            #print("Opti-content = {}".format(content))

            #data = np.array(np.loadtxt(filepath_opti))   # Read data from file
            #print("data = {}".format(data))
            #data = data/np.max(data) # normalize
            #plt.xlim(0,len(edges))  # Set x-axis range
            #plt.ylim(0, np.max(data))  # Set x-axis range
            #plt.plot(data)
            #plt.show()
            poses = []
            x=[]
            y=[]
            z=[]
            with open(filepath_poses, 'r') as file:
                # Create a CSV reader object
                reader = csv.reader(file)
                # Read each row in the CSV file
                for row in reader:
                    # Access the columns by index
                    x.append(float(row[0]))
                    y.append(float(row[1]))
                    z.append(float(row[2]))
                    # Do something with the data
        else:
            print("Graph initialized to None.")
    def compute_anchored_L(self):
        L = nx.laplacian_matrix(self.graph, weight='weight')
        idx_to_drop = np.random.randint(0, np.shape(L)[1], 1)
        # convert a sparse  matrix to a coordinate format
        C = L.tocoo()
        keep = ~np.in1d(C.col, idx_to_drop)
        C.data, C.row, C.col = C.data[keep], C.row[keep], C.col[keep]
        C.col -= idx_to_drop.searchsorted(C.col)
        C._shape = (C.shape[0], C.shape[1] - len(idx_to_drop))
        keep = ~np.in1d(C.row, idx_to_drop)
        C.data, C.row, C.col = C.data[keep], C.row[keep], C.col[keep]
        C.row -= idx_to_drop.searchsorted(C.row)
        C._shape = (C.shape[0] - len(idx_to_drop), C.shape[1])
        return C.tocsr()

    def compute_L(self):
        return nx.laplacian_matrix(self.graph, weight='weight')

    def compute_algcon(self):
        return nx.algebraic_connectivity(self.graph, weight='weight', normalized=False, tol=1e-08, method='tracemin')

    def compute_maxE(self):
        L = self.compute_L()
        n = self.get_no_nodes() - 1
        return scipy.linalg.eigvalsh(L.todense(), eigvals=[n, n])

    def get_no_edges(self):
        return nx.number_of_edges(self.graph)

    def get_no_nodes(self):
        return nx.number_of_nodes(self.graph)

    def get_LC_edges(self):
        edges = self.graph.edges.data('type')
        edges_LC = []
        for (u, v, wt) in edges:
            edges_LC.append([u, v]) if wt == 1 else True
        return sorted(edges_LC, key=itemgetter(1))

    def plot_graph(self, label='Data', color='Blue', draw_l=False):
        if self.graph is not None:
            nodes = self.graph.nodes.data('pose')
            #print("graph Nodes  = {}".format(nodes))
            poses_x = [el[1][0] for el in nodes]
            print('poses_x = {}'.format(poses_x))
            poses_y = [el[1][1] for el in nodes]
            plt.plot(poses_x, poses_y, '-', label=label, alpha=1, color=color)
            plt.suptitle('Trajectory')
            if draw_l:
                plt.plot(poses_x[0], poses_y[0], '*', label='Start', color='black', alpha=1, markersize=15) # start
                plt.plot(poses_x[-1], poses_y[-1], 'd', label='End', color='blue', alpha=1, markersize=12)  # end
                label_added = False
                edges_LC = self.get_LC_edges()                
                for (u, v) in edges_LC:
                    poses_LC_x = []
                    poses_LC_y = []
                    poses_LC_x.append(self.graph.nodes[u]['pose'][0])
                    poses_LC_x.append(self.graph.nodes[v]['pose'][0])
                    poses_LC_y.append(self.graph.nodes[u]['pose'][1])
                    poses_LC_y.append(self.graph.nodes[v]['pose'][1])
                    if not label_added:
                        plt.plot(poses_LC_x, poses_LC_y, linestyle='-', color='green', label='Loop Closure', alpha=1, marker='o', markeredgecolor='b', markersize=2.5)
                        label_added = True
                    else:
                        plt.plot(poses_LC_x, poses_LC_y, linestyle='-', color='green', alpha=1, marker='o', markeredgecolor='b', markersize=2.5)
                
                
                
                