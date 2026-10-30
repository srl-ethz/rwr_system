from scipy.spatial.transform import Rotation
import numpy as np

def rigid_transform(pos, quat):
    """
    Compute the 4x4 rigid transformation matrix from position and quaternion.

    Parameters:
    pos: array-like, shape (3,)
        Position vector [x, y, z]
    quat: array-like, shape (4,)
        Quaternion [x, y, z, w] where w is the scalar component

    Returns:
    T: ndarray, shape (4,4)
        The 4x4 rigid transformation matrix
    """
    # Ensure inputs are numpy arrays
    pos = np.asarray(pos)
    quat = np.asarray(quat)

    assert pos.size == 3, "Position must have size 3"
    assert quat.size == 4, "Quaternion must have size 4"
    
    # Create rotation matrix from quaternion
    rot_matrix = Rotation.from_quat(quat).as_matrix()
    
    # Create 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = rot_matrix
    T[:3, 3] = pos
    
    return T


def complete_rotation_matrix(x=None, y=None, z=None): 
    """
    Compute a 3x3 matrix from three axises. Return a rotation matrix.
    There should be exactly one None value in the input.
    """
    assert sum([x is None, y is None, z is None]) == 1, "Exactly one axis should be None"
    x = np.array(x) if x is not None else None
    y = np.array(y) if y is not None else None
    z = np.array(z) if z is not None else None
    x = x / np.linalg.norm(x) if x is not None else None
    y = y / np.linalg.norm(y) if y is not None else None
    z = z / np.linalg.norm(z) if z is not None else None
    if x is None:
        x = np.cross(y, z)
    elif y is None:
        y = np.cross(z, x)
    elif z is None:
        z = np.cross(x, y)
    return np.vstack((x, y, z)).T
