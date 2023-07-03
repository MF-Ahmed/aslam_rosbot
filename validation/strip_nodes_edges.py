import sys
import difflib

def strip_nodes_and_edges_SE2(input_file='pose_graph_hospital_mpc.g2o', nodes_output_file = 'stripped_nodes.txt', edges_output_file='stripped_edges.py'):
    # Read the g2o file
    with open(input_file, 'r') as file:
        lines = file.readlines()
    # Parse the g2o file
    nodes = []
    edges = []
    for line in lines:
        if line.startswith('VERTEX_SE2'):
            nodes.append(line.replace('VERTEX_SE2',''))
        elif line.startswith('EDGE_SE2'):
            edges.append(line.replace('EDGE_SE2',''))

    # Save the stripped nodes into a .txt file
    with open(nodes_output_file, 'w') as file:
        file.writelines(nodes)

    # Save the stripped edges into a .txt file
    with open(edges_output_file, 'w') as file:
        file.writelines(edges)

    with open(edges_output_file, 'r') as file:
        lines = file.readlines()

    # Open the file again in write mode
    with open(edges_output_file, 'w') as file:
        # Iterate over the lines and modify each line by adding the desired string at a specific location
        for line in lines:
            parts = line.split()
            node1 = int(parts[0])
            node2 = int(parts[1])
            if (abs(node2-node1))>1:
                loopclosure = '1'
            else:
                loopclosure = '0'


            file.write(f'{parts[0]} {parts[1]} {loopclosure} {parts[2]} {parts[3]} {parts[4]} {parts[5]} {parts[6]} {parts[7]} {parts[8]} {parts[9]} {parts[10]}  \n')

# Check if both input and output file arguments are provided
#if len(sys.argv) < 3:
    #print("Please provide both the input file path and output file path as arguments.")
    #sys.exit(1)

#input_file = sys.argv[1]
#nodes_output_file = sys.argv[2]
#edges_output_file = sys.argv[3]

input_file = 'entropy_frontier_5min.g2o'
nodes_output_file = 'entropy_frontier_5min_nodes.txt'
edges_output_file = 'entropy_frontier_5min_edges.txt'



strip_nodes_and_edges_SE2(input_file, nodes_output_file, edges_output_file)



