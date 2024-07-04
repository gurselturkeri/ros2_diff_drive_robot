from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

def read_pgm(pgm_file):
    """
    Reads a PGM file and converts it to a numpy array.
    
    :param pgm_file: Path to the PGM file.
    :return: Numpy array representing the PGM image.
    """
    image = Image.open(pgm_file)
    pgm_array = np.array(image)
    return pgm_array

def pgm_to_occupancy_grid(pgm_array, free_threshold=254, occupied_threshold=0):
    """
    Converts a PGM array to an occupancy grid.
    
    :param pgm_array: Numpy array of the PGM image.
    :param free_threshold: Threshold for free space.
    :param occupied_threshold: Threshold for occupied space.
    :return: Numpy array representing the occupancy grid.
    """
    occupancy_grid = np.zeros(pgm_array.shape, dtype=np.int8)

    # Debugging: Print some statistics about the pgm_array
    print(f"PGM array statistics: min={pgm_array.min()}, max={pgm_array.max()}, mean={pgm_array.mean()}")

    occupancy_grid[pgm_array > free_threshold] = 0       # Free space
    occupancy_grid[pgm_array <= occupied_threshold] = 100  # Occupied space
    occupancy_grid[(pgm_array > occupied_threshold) & (pgm_array <= free_threshold)] = -1  # Unknown space
    
    return occupancy_grid

# Path to the PGM file
pgm_file = 'my_map.pgm'  # Replace with the path to your PGM file

# Read the PGM file
pgm_array = read_pgm(pgm_file)

# Convert the PGM array to an occupancy grid
occupancy_grid = pgm_to_occupancy_grid(pgm_array)

# Print the occupancy grid
print(occupancy_grid)

# Visualize the occupancy grid
plt.imshow(occupancy_grid, cmap='gray', origin='lower')
plt.colorbar()
plt.show()
