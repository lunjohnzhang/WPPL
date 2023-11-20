import numpy as np
import os
import matplotlib.pyplot as plt
script_dir = os.path.dirname(os.path.realpath(__file__))

# Build the file path relative to the script's location
file_path = os.path.join(script_dir, "frequency.txt")

with open(file_path, 'r') as file:
    data = file.read()

# Parsing the data into a 2D numpy array
data_array = [list(map(int, line.split())) for line in data.split('\n') if line.strip()]
data_matrix = np.array(data_array)

# Using a masked array to plot 0 values as white and other frequencies with a normal scale
masked_data = np.ma.masked_where(data_matrix == 0, data_matrix)

# Using the 'Viridis' colormap and setting zero values to white
cmap_viridis = plt.cm.get_cmap('viridis')
cmap_viridis.set_bad(color='white')  # Warning: Modifying a global colormap is deprecated

# Plotting the heatmap with the 'Viridis' colormap
plt.figure(figsize=(12, 8))
plt.imshow(masked_data, cmap=cmap_viridis, interpolation='nearest')
plt.colorbar()
plt.title('Heatmap of Frequency')
plt.show()
