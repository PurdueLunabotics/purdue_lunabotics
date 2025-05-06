import matplotlib.pyplot as plt

def read_float_costmap(filename):
    """Reads a 2D cost map of floats from a text file into a nested list."""
    costmap = []
    with open(filename, 'r') as f:
        for line in f:
            if line.strip():
                row = [float(val) for val in line.strip().split()]
                costmap.append(row)
    return costmap

def get_min_max(costmap):
    """Finds the minimum and maximum values in a nested list of floats."""
    min_val = float('inf')
    max_val = float('-inf')
    for row in costmap:
        for val in row:
            if val < min_val:
                min_val = val
            if val > max_val:
                max_val = val
    return min_val, max_val

def plot_costmap(costmap):
    """Displays the float cost map using matplotlib."""
    min_val, max_val = get_min_max(costmap)
    
    plt.imshow(costmap, cmap='viridis', origin='upper', vmin=min_val, vmax=700)
    plt.colorbar(label='Cost (Float)')
    plt.title("2D Float Cost Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(False)
    plt.show()

if __name__ == "__main__":
    filename = "/home/purdue/.ros/nodevalues.txt"  # Change this to your actual file
    costmap = read_float_costmap(filename)
    plot_costmap(costmap)
