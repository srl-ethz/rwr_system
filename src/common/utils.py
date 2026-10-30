import sys
import glob
import time
import datetime
import threading
import numpy as np
from tqdm import trange

import os
import json
import tempfile
import platform
import shutil
import subprocess
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


class NamedArrayElementDescriptor:
    def __init__(self, name, arr_name, index):
        self.name = name
        self.index = index
        self.arr_name = arr_name

    def __get__(self, obj, objtype=None):
        return getattr(obj, self.arr_name)[self.index].copy()

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
            print(f"{self._context} Loop rate slower than desired: {(1.0/time_since):.3f} Hz < {1.0/self._period_sec:.3f} Hz")
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


def print_warn_block(msg: str) -> None:
    """Print a big yellow warning block."""
    YELLOW = "\033[38;5;226m" # "\033[93m" --> light mustard
    RESET  = "\033[0m"
    banner = "=" * 80
    print(f"{YELLOW}\n{banner}\n⚠️  {msg}\n{banner}{RESET}\n", file=sys.stderr, flush=True)


def print_ok_block(msg: str) -> None:
    """Print a big green success block."""
    GREEN  = "\033[38;5;46m"
    RESET  = "\033[0m"
    banner = "=" * 80
    print(f"{GREEN}\n{banner}\n✅  {msg}\n{banner}{RESET}\n", flush=True)


def print_info_block(msg: str) -> None:
    """Print an informational block in light blue."""
    # 256-colour palette index 117 → #87AFFf, a clear sky-blue
    BLUE  = "\033[38;5;117m"
    RESET = "\033[0m"
    banner = "=" * 80
    print(f"{BLUE}\n{banner}\nℹ️  {msg}\n{banner}{RESET}\n", flush=True)


def print_error_block(msg: str) -> None:
    """Print a big red error block."""
    RED = "\033[38;5;196m"  # Bright red from 256-colour palette
    RESET = "\033[0m"
    banner = "=" * 80
    print(f"{RED}\n{banner}\n❌  {msg}\n{banner}{RESET}\n", file=sys.stderr, flush=True)


def check_low_latency(port: str) -> bool:
    """
    Returns:
        True  - flag is set
        False - flag missing or cannot be verified
        None  - not applicable on this OS
    Side-effect:
        Prints a yellow warning block when flag is missing or check is impossible.
    """
    system = platform.system()

    if system == "Linux":
        # 1. Port sanity
        if not os.path.exists(port):
            print_warn_block(f"Serial port {port!r} doesn't exist.")
            return False

        # 2. Need setserial
        if not shutil.which("setserial"):
            print_warn_block("'setserial' isn't installed - can't verify low_latency.")
            return False

        # 3. Ask setserial
        try:
            result = subprocess.check_output(
                ["setserial", "-g", port],
                text=True,
                stderr=subprocess.STDOUT,
            ).strip()
        except subprocess.CalledProcessError as exc:
            print_warn_block(f"setserial blew up:\n{exc.output.strip()}")
            return False

        if "low_latency" in result:
            return True

        print_warn_block(
            f"low_latency flag is NOT set on {port}.\n"
            f"Run:\n    sudo setserial {port} low_latency"
        )
        return False

    elif system == "Windows":
        # There's no generic low-latency switch; FTDI and friends expose
        # registry tweaks instead. We simply inform the caller.
        print_warn_block(
            "Windows doesn't expose a generic low-latency flag for COM ports.\n"
            "If you're chasing FTDI latency, adjust the driver's LatencyTimer in "
            "Device Manager or the registry."
        )
        return False

    else:
        # macOS / BSD - unsupported.
        print_warn_block(f"{system} is not supported for this check.")
        return False


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


# -----------------------------------------------------------------------------
# Dynamixel helpers
# -----------------------------------------------------------------------------

def scan_bus_for_motors(port: str = "", baudrate: int = 3000000, id_range: tuple = (0, 252)) -> list:
    """Scan a Dynamixel bus and return a sorted list of responding motor IDs.

    Args:
        port: Serial device name. If empty, :func:`find_tty_port` is used to
              choose one automatically/interactively.
        baudrate: Baudrate to configure on the port while scanning.
        id_range: `(min_id, max_id)` tuple limiting IDs that are pinged.

    Returns:
        Sorted list of responding motor IDs. Empty if none found or on error.
    """
    try:
        # Import lazily to keep utils lightweight for non-motor code paths.
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS  # type: ignore
    except ImportError:
        print_error_block(
            "dynamixel_sdk is not installed - cannot scan for Dynamixel motors.\n"
            "Install it with `pip install dynamixel_sdk`."
        )
        return []

    PROTOCOL_VERSION = 2.0

    if port == "":
        port = find_tty_port("dynamixel")
    if port is None:
        return []

    check_low_latency(port)

    ph = PortHandler(port)
    pkt = PacketHandler(PROTOCOL_VERSION)

    if not ph.openPort():
        print_error_block(f"Failed to open port {port}")
        return []
    if not ph.setBaudRate(baudrate):
        print_error_block(f"Failed to set baudrate {baudrate} on {port}")
        ph.closePort()
        return []

    found_ids = []
    for mid in trange(id_range[0], id_range[1] + 1):
        _, comm_result, error = pkt.ping(ph, mid)
        if comm_result == COMM_SUCCESS and error == 0:
            found_ids.append(mid)
    ph.closePort()

    if found_ids:
        print_ok_block(f"Scan complete - found motors: {found_ids}")
    else:
        print_warn_block("No motors responded on the Dynamixel bus.")
    return sorted(found_ids)