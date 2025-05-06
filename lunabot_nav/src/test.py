import matplotlib.pyplot as plt

def read_costmap(filename):
    """Reads a 2D cost map from a text file into a nested list."""
    costmap = []
    with open(filename, 'r') as f:
        for line in f:
            if line.strip():  # Ignore empty lines
                row = [int(val) for val in line.strip().split()]
                costmap.append(row)
    return costmap

def plot_costmap(costmap):
    """Displays the cost map using matplotlib."""
    # Matplotlib requires a 2D array-like structure; nested list works
    plt.imshow(costmap, cmap='viridis', origin='upper')
    plt.colorbar(label='Cost')
    plt.title("2D Cost Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(False)
    plt.show()

if __name__ == "__main__":
    filename = "/home/purdue/.ros/map.txt"  # Replace with your actual filename
    costmap = read_costmap(filename)
    plot_costmap(costmap)
