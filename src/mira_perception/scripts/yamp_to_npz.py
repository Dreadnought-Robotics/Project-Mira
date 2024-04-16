import yaml
import numpy as np

# Load data from YAML file
with open('/home/shasankgunturu/catkin_ws/src/perception/scripts/my_camera_node.yaml', 'r') as file:
    data = yaml.load(file, Loader=yaml.FullLoader)

# Convert data to numpy arrays
arrays = {key: np.array(value) for key, value in data.items()}

# Save arrays to a .npz file
np.savez('data.npz', **arrays)
