import matplotlib.pyplot as plt
import networkx as nx
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

# Create a directed graph
G = nx.DiGraph()

# Add nodes
nodes = ['s1', 's2', 's3', 's4', '{s2, s3}', '{s2, s4}']
G.add_nodes_from(nodes)

# Add edges
edges = [('s1', '{s2, s3}'), ('s2', 's1'), ('s3', '{s2, s4}'), ('s4', 's1')]
G.add_edges_from(edges)

# Define node positions for a visually appealing layout
pos = {'s1': (0, 0), 's2': (1, -1), 's3': (2, 1), 's4': (2, -1), '{s2, s3}': (1.5, 0), '{s2, s4}': (1.5, -1)}

# Draw the graph
plt.figure(figsize=(3, 3))
nx.draw(G, pos, with_labels=True, node_size=5000, node_color="lightblue", arrowsize=20, font_size=15, font_weight='bold')

# # Define regions and their colors
# regions = [
#     {'nodes': ['s2', 's3'], 'color': 'red'},
#     {'nodes': ['s2', 's4'], 'color': 'blue'},
# ]

# # Add elliptical regions around nodes
# ax = plt.gca()
# for region in regions:
#     nodes_in_region = region['nodes']
#     color = region['color']
    
#     # Compute the center of the region by averaging node positions
#     x_coords = [pos[node][0] for node in nodes_in_region]
#     y_coords = [pos[node][1] for node in nodes_in_region]
#     center_x, center_y = sum(x_coords) / len(x_coords), sum(y_coords) / len(y_coords)
    
#     # Determine width and height of the ellipse
#     width = (max(x_coords) - min(x_coords)) + 1.5
#     height = (max(y_coords) - min(y_coords)) + 1.5
    
#     # Create and add the ellipse patch
#     ellipse = Ellipse((center_x, center_y), width, height, color=color, alpha=0.3)
#     ax.add_patch(ellipse)

# Add dashed ellipse to represent the set node {s2, s3}
ax = plt.gca()
ellipse = Ellipse((1.5, 0), width=1.8, height=2.2, edgecolor='red', facecolor='none', linestyle='--')
ellipse2 = Ellipse((1.5, -1), width=1.8, height=2.2, edgecolor='blue', facecolor='none', linestyle='--')
ax.add_patch(ellipse)
ax.add_patch(ellipse2)



# Display the plot
plt.show()
