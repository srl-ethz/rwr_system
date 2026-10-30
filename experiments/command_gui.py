#!/bin/env python3

"""
A ROS2 node that provides a GUI for controlling multiple policy command dimensions via sliders.
Example usage:
    ros2 run experiments command_gui.py -n 17 -t policy_output
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import Scale, HORIZONTAL, Canvas, Frame
import argparse

class PolicyCommandGUI(Node):
    def __init__(self, num_commands=1, topic_name='policy_output', publish_freq=10.0):
        super().__init__('policy_command_gui')
        self.declare_parameter('num_commands', num_commands)
        self.num_commands = self.get_parameter('num_commands').value
        self.declare_parameter("hand_controller/hand_chirality", "right")
        self.chirality = self.get_parameter("hand_controller/hand_chirality").value
        self.declare_parameter("topic_name", topic_name)
        self.topic_name = self.get_parameter("topic_name").value
        self.publish_freq = float(publish_freq)
        self.get_logger().info(f'Policy Command GUI Node starting with {self.num_commands} commands. Topic: {self.topic_name}, freq: {self.publish_freq} Hz')

        self.publisher_ = self.create_publisher(Float32MultiArray, f"/{self.chirality}/hand/{self.topic_name}", 10)

        self.root = tk.Tk()
        self.root.title("Policy Command GUI")

        # Create a Canvas for scrolling
        self.canvas = Canvas(self.root)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.scrollbar = tk.Scrollbar(self.root, orient=tk.VERTICAL, command=self.canvas.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.scrollable_frame = Frame(self.canvas)
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.scrollable_frame.bind('<Configure>', lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        # Enable mouse wheel scrolling
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", lambda e: self.canvas.yview_scroll(-1, "units"))
        self.canvas.bind_all("<Button-5>", lambda e: self.canvas.yview_scroll(1, "units"))

        self.command_sliders = []
        self.command_values = [0.0] * self.num_commands # Initialize all to 0.5

        for i in range(self.num_commands):
            row = tk.Frame(self.scrollable_frame)
            row.pack(fill=tk.X, pady=2)

            label = tk.Label(row, text=f"Command {i+1}:")
            label.pack(side=tk.LEFT, padx=4)

            minus_btn = tk.Button(row, text='-', width=2, command=lambda index=i: self.adjust_range(index, -1))
            minus_btn.pack(side=tk.LEFT, padx=2)

            plus_btn = tk.Button(row, text='+', width=2, command=lambda index=i: self.adjust_range(index, +1))
            plus_btn.pack(side=tk.LEFT, padx=2)

            slider = Scale(self.scrollable_frame, from_=-1.0, to=1.0, resolution=0.01,
                           orient=HORIZONTAL, length=300,
                           command=lambda value, index=i: self.on_scale_change(index, float(value)))
            slider.set(self.command_values[i])
            slider.pack(pady=2)
            self.command_sliders.append(slider)

        # Zero button to reset all dimensions to zero
        self.zero_button = tk.Button(self.root, text="Zero", command=self.zero_all)
        self.zero_button.pack(pady=10)

        # Start periodic publishing at the specified frequency
        if self.publish_freq and self.publish_freq > 0:
            interval_ms = int(1000.0 / self.publish_freq)
            self.root.after(interval_ms, lambda: self._publish_periodic(interval_ms))

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.update_gui)

    def on_scale_change(self, index, value):
        self.command_values[index] = value
        # Optional: You can uncomment the line below if you want to publish immediately on *any* slider change
        # self.publish_all_commands()

    def publish_all_commands(self):
        msg = Float32MultiArray()
        msg.data = self.command_values
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing: "{msg.data}"')

    def _publish_periodic(self, interval_ms):
        # Publish and reschedule
        try:
            self.publish_all_commands()
        finally:
            if self.publish_freq and self.publish_freq > 0:
                self.root.after(interval_ms, lambda: self._publish_periodic(interval_ms))

    def zero_all(self):
        # Set all sliders and values to zero
        for i, slider in enumerate(self.command_sliders):
            slider.set(0.0)
            self.command_values[i] = 0.0

    def adjust_range(self, index, direction):
        # direction: +1 to expand range by 1.0 on both ends, -1 to shrink by 1.0 on both ends
        slider = self.command_sliders[index]
        current_from = float(slider.cget('from'))
        current_to = float(slider.cget('to'))
        if direction > 0:
            new_from = current_from - 1.0
            new_to = current_to + 1.0
        else:
            new_from = current_from + 1.0
            new_to = current_to - 1.0
            # enforce a minimal span of 1.0
            if (new_to - new_from) < 1.0:
                center = (current_to + current_from) / 2.0
                new_from = center - 0.5
                new_to = center + 0.5

        # Clamp current value into the new range
        val = self.command_values[index]
        if val < new_from:
            val = new_from
        if val > new_to:
            val = new_to

        slider.config(from_=new_from, to=new_to)
        slider.set(val)
        self.command_values[index] = val

    def _on_mousewheel(self, event):
        # Windows/MacOS MouseWheel uses event.delta
        if event.delta:
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def update_gui(self):
        self.root.update_idletasks()
        self.root.update()
        self.root.after(100, self.update_gui)

    def on_closing(self):
        self.get_logger().info("Closing Policy Command GUI.")
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="ROS2 Policy Command GUI")
    parser.add_argument('-n', '--num_commands', type=int, default=1,
                        help='Number of command sliders to display (default: 1)')
    parser.add_argument('-t', '--topic', type=str, default='policy_command',
                        help='ROS topic name to publish the commands to (default: policy_command)')
    parser.add_argument('-f', '--freq', type=float, default=10.0,
                        help='Publishing frequency in Hz for all commands (default: 10.0)')
    parsed_args, ros_args = parser.parse_known_args()

    # Pass args to the node constructor
    node = PolicyCommandGUI(num_commands=parsed_args.num_commands,
                            topic_name=parsed_args.topic,
                            publish_freq=parsed_args.freq)

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Node clean shutdown.")
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()