from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np

def numpy_to_float32_multiarray(np_array):
    """
    Converts a NumPy array to a ROS2 Float32MultiArray message, including correct strides.

    :param np_array: NumPy array of any shape
    :return: Float32MultiArray message with proper layout and data
    """
    # Ensure the NumPy array is of type float32
    np_array = np_array.astype(np.float32)

    # Create the Float32MultiArray message
    msg = Float32MultiArray()

    # Flatten the NumPy array and assign it to the data field
    msg.data = np_array.flatten().tolist()

    # Create the MultiArrayLayout
    layout = MultiArrayLayout()
    layout.dim = []
    layout.data_offset = 0

    # Get the shape of the NumPy array
    shape = np_array.shape
    n_dims = np_array.ndim

    # Compute the strides
    # Stride for dimension D is the product of the sizes of the dimensions after D
    strides = []
    stride = 1
    for size in reversed(shape):
        strides.insert(0, stride)
        stride *= size

    # Create MultiArrayDimension messages for each dimension
    for i in range(n_dims):
        dim = MultiArrayDimension()
        dim.label = f"dim_{i}"
        dim.size = shape[i]
        dim.stride = strides[i]
        layout.dim.append(dim)

    # Assign the layout to the message
    msg.layout = layout

    return msg


def float32_multiarray_to_numpy(msg):
    """
    Converts a ROS2 Float32MultiArray message to a NumPy array using its layout information.

    :param msg: Float32MultiArray message
    :return: NumPy array reconstructed from the Float32MultiArray message
    """
    if msg.layout.dim == []:
        # If no layout information is provided, assume a 1D array
        return np.array(msg.data, dtype=np.float32)
    # Extract the shape of the array from the layout dimensions
    shape = [dim.size for dim in msg.layout.dim]

    # Convert the flattened data back to a NumPy array and reshape it
    np_array = np.array(msg.data, dtype=np.float32).reshape(shape)

    return np_array