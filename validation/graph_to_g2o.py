def convert_graph_to_g2o(input_file, output_file):
    # Open the input file in read mode
    with open(input_file, 'r') as f:
        lines = f.readlines()

    # Open the output file in write mode
    with open(output_file, 'w') as f:
        for line in lines:
            line = line.strip()  # Remove leading/trailing whitespaces
            if line.startswith('VERTEX2'):
                # Process vertex data
                parts = line.split()
                vertex_id, x, y, theta = parts[1], parts[2], parts[3], parts[4]
                f.write(f'VERTEX_SE2 {vertex_id} {x} {y} {theta}\n')
            elif line.startswith('EDGE2'):
                # Process edge data
                parts = line.split()
                vertex1_id, vertex2_id, x, y, theta, q11, q12, q13, q22, q23, q33 = parts[1], parts[2], parts[3], parts[4], parts[5],parts[6], parts[7], parts[8], parts[9], parts[10], parts[11]

                f.write(f'EDGE_SE2 {vertex1_id} {vertex2_id} {x} {y} {theta} {q11} {q12} {q23} {q13} {q33} {q22} \n')

    print('Conversion complete!')

convert_graph_to_g2o('FRH_P_toro.graph', 'output.g2o')

# Usage:
