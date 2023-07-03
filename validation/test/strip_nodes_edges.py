import sys
import difflib
#file_list_path = "/home/usr/data/catkin_ws/src/aslam_rosbot/aslam_rosbot/maps/file_to_convert.txt"
file_list_path = "/home/usr/data/catkin_ws/src/aslam_rosbot/validation/test/filename.txt"

def strip_nodes_and_edges_SE2(input_file, nodes_output_file , edges_output_file):
    # Read the g2o file
    print("input file = {}".format(input_file))
    try:
        with open(input_file, 'r') as file:
           lines = file.readlines()
    except Exception as e:
        print("Error reading the file:", str(e))
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

with open(file_list_path, "r") as file_list:
    # Read the lines of the file_list.txt file
    files = file_list.readlines()


# Remove leading/trailing whitespaces and newlines from the file paths/names
files = [file.strip() for file in files]

for i in range (0, len(files)):
   input_file = files[i]
   nodes_output_file = input_file +'_nodes.txt'
   edges_output_file = input_file +'_edges.txt'
   
   strip_nodes_and_edges_SE2(input_file, nodes_output_file, edges_output_file)



