import sys
import glob
import serial.tools.list_ports
import time
import datetime
import threading
import numpy as np

class NamedArrayElementDescriptor:
    def __init__(self, name, arr_name, index):
        self.name = name
        self.index = index
        self.arr_name = arr_name

    def __get__(self, obj, objtype=None):
        return getattr(obj, self.arr_name)[self.index]

    def __set__(self, obj, value):
        getattr(obj, self.arr_name)[self.index] = value

    def __delete__(self, obj):
        getattr(obj, self.arr_name)[self.index] = 0  # or any default value

class Rate:  # pylint: disable=too-few-public-methods
    """Rate object to sleep in a loop to maintain a constant rate."""

    def __init__(self, period_sec: float, warn_threshold: float = 0.1, context: str = "", warn_probability: float = 0.05):
        """Initialize the rate object.

        :param period_sec: The period in seconds.
        :param warn_threshold: The threshold to print a warning if the rate is
            slower than the period.
        """
        self._period_sec = period_sec
        self._warn_threshold_sec = -1 * warn_threshold * period_sec
        self._last_time = time.monotonic()
        self._context = context
        self._warn_check = lambda: np.random.rand() < warn_probability

    def sleep(self):
        """Sleep to maintain the rate."""
        time_since = time.monotonic() - self._last_time
        sleep_time = self._period_sec - time_since
        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < self._warn_threshold_sec and self._warn_check():
            # print(f"{self._context} WARNING: Rate is too slow! {time_since:.3f} s instead of {self._period_sec:.3f} s")
            print(f"{self._context} Loop rate: {(1.0/time_since):.3f} Hz")
        self._last_time = time.monotonic()


class ThreadSafeWrapper:
    def __init__(self, obj):
        self._obj = obj
        self._lock = threading.RLock()

    def __getattr__(self, name):
        with self._lock:
            attr = getattr(self._obj, name)
            if callable(attr):
                def wrapper(*args, **kwargs):
                    with self._lock:
                        return attr(*args, **kwargs)
                return wrapper
            return attr


def get_datetime_str(time_start=None):
    if time_start is None:
        time_start = datetime.datetime.now()
    return time_start.strftime("%Y%m%d%H%M%S")

import os
import glob
import json
import tempfile

def find_tty_port(identifier: str=""):
    # Determine the OS and set the appropriate glob pattern
    if os.name == 'nt':  # Windows
        ports = glob.glob('COM[0-9]*')
    else:  # Unix-like systems (Linux, macOS)
        ports = glob.glob('/dev/tty[.A-Za-z]*')

    if identifier == "":
        identifier = "tty_device"

    # Filter out non-TTY devices
    tty_ports = [port for port in ports if 'tty' in port.lower()]

    if not tty_ports:
        print("No TTY devices found.")
        return None

    # Path for the temporary file
    temp_file = os.path.join(tempfile.gettempdir(), f"{identifier}.json")

    # Check if we have a saved choice
    if os.path.exists(temp_file):
        with open(temp_file, 'r') as f:
            saved_choice = json.load(f)
        if saved_choice in tty_ports:
            print(f"Using saved port: {saved_choice}")
            return saved_choice
        else:
            print(f"Saved port {saved_choice} not found.")

    # If we have multiple ports or no saved choice, ask the user
    if len(tty_ports) > 1:
        print("Multiple TTY devices found. Please choose:")
        for i, port in enumerate(tty_ports):
            print(f"{i + 1}. {port}")
        
        while True:
            try:
                choice = int(input("Enter the number of your choice: ")) - 1
                if 0 <= choice < len(tty_ports):
                    selected_port = tty_ports[choice]
                    break
                else:
                    print("Invalid choice. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number.")
    else:
        selected_port = tty_ports[0]

    # Save the choice
    with open(temp_file, 'w') as f:
        json.dump(selected_port, f)

    print(f"Selecting port: {selected_port}")
    return selected_port

def scan_tty_ports():
    """
    Scan for available TTY ports on the system.
    
    Returns:
    list: A list of available TTY port names.
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass

    return result


from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
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
    # Extract the shape of the array from the layout dimensions
    shape = [dim.size for dim in msg.layout.dim]

    # Convert the flattened data back to a NumPy array and reshape it
    np_array = np.array(msg.data, dtype=np.float32).reshape(shape)

    return np_array