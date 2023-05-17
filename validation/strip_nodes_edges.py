def strip_nodes_and_edges_SE2(input_file, nodes_output_file, edges_output_file):
    # Read the g2o file
    with open(input_file, 'r') as file:
        lines = file.readlines()
    # Parse the g2o file
    nodes = []
    edges = []
    #i=0
    for line in lines:
        if line.startswith('VERTEX'):
            nodes.append(line.replace('VERTEX_SE2',''))
        elif line.startswith('EDGE'):
            #edges.append(line.replace('EDGE_SE2',str(i)))
            edges.append(line.replace('EDGE_SE2',''))

    # Save the stripped nodes into a .txt file
    with open(nodes_output_file, 'w') as file:
        file.writelines(nodes)

    # Save the stripped edges into a .txt file
    with open(edges_output_file, 'w') as file:
        file.writelines(edges)

# Usage
input_file = 'pose_graph_hospital_mpc.g2o'
nodes_output_file = 'pose_graph_hospital_mpc_stripped_nodes.txt'
edges_output_file = 'pose_graph_hospital_mpc_stripped_edges.txt'
strip_nodes_and_edges_SE2(input_file, nodes_output_file, edges_output_file)


