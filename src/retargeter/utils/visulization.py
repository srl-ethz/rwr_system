import numpy as np
import matplotlib.pyplot as plt


def plot_mano_hand(joints, fig_ax=None):
    """
    Visualizes MANO hand joints in 3D.

    Parameters:
    - joints: numpy array of shape (21, 3)
    """
    # For simplicity, we'll visualize the first hand in the batch
    # Define connections between joints (bones)
    bones = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),  # Thumb
        (0, 5),
        (5, 6),
        (6, 7),
        (7, 8),  # Index finger
        (0, 9),
        (9, 10),
        (10, 11),
        (11, 12),  # Middle finger
        (0, 13),
        (13, 14),
        (14, 15),
        (15, 16),  # Ring finger
        (0, 17),
        (17, 18),
        (18, 19),
        (19, 20),  # Pinky
    ]
    if fig_ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig, ax = fig_ax
    # Plot joints
    ax.scatter(joints[:, 0], joints[:, 1], joints[:, 2], c="r", s=20)

    # Plot bones
    for bone in bones:
        joint_start = joints[bone[0]]
        joint_end = joints[bone[1]]
        xs = [joint_start[0], joint_end[0]]
        ys = [joint_start[1], joint_end[1]]
        zs = [joint_start[2], joint_end[2]]
        ax.plot(xs, ys, zs, c="b")

    # Set plot labels and aspect ratio
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("MANO Hand Joints Visualization")
    ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio

    return fig, ax


def draw_frame(origin, x_axis, y_axis, z_axis, fig_ax=None):
    if fig_ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig, ax = fig_ax

    # Plot the axes as arrows
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        x_axis[0],
        x_axis[1],
        x_axis[2],
        length=1.0,
        color="r",
        label="X-axis",
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        length=1.0,
        color="g",
        label="Y-axis",
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        z_axis[0],
        z_axis[1],
        z_axis[2],
        length=1.0,
        color="b",
        label="Z-axis",
    )

    return fig, ax
