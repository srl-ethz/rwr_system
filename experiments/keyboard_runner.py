#!/bin/env python3
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String as ROSString
import argparse
import rclpy
from rclpy.node import Node

from faive_system.src.common.enums import TeleopState, HandChirality

parser = argparse.ArgumentParser(description="A runner that coordinates the nodes with keyboard input")

parser.add_argument("--right_hand", action="store_true", help="If right part of the robot is used")
parser.add_argument("--left_hand", action="store_true", help="If left part of the robot is used")
parser.add_argument("--retargeter", action="store_true", help="If retargeter is used")
parser.add_argument("--teleop_pause_and_resume", action="store_true", help="Pause and resume teleop with Enter")
parser.add_argument("--teleop_back_to_init", action="store_true", help="Move to init pose with Enter")
parser.add_argument("--demo_logger", action="store_true", help="Trigger demo logger with Enter")
parser.add_argument("--prompt_task_description", action="store_true", help="Prompt user to enter a task description")
parser.add_argument("--run_policy_inference", action="store_true", help="Trigger policy inference (e.g. for DAgger)")

args, unknown = parser.parse_known_args()

RIGHT_HAND = args.right_hand
LEFT_HAND = args.left_hand
TELEOP_PAUSE_AND_RESUME = args.teleop_pause_and_resume
TELEOP_BACK_TO_INIT = args.teleop_back_to_init
TELEOP_RETARGETER = args.retargeter
DEMO_LOGGER = args.demo_logger
PROMPT_TASK_DESCRIPTION = args.prompt_task_description
RUN_POLICY_INFERENCE = args.run_policy_inference

print(
"#### A runner that coordinates the nodes with keyboard input ####"
"\nRIGHT_HAND", RIGHT_HAND,
"\nLEFT_HAND", LEFT_HAND,
"\nTELEOP_PAUSE_AND_RESUME", TELEOP_PAUSE_AND_RESUME,
"\nTELEOP_BACK_TO_INIT", TELEOP_BACK_TO_INIT,
"\nTELEOP_RETARGETER", TELEOP_RETARGETER,
"\nDEMO_LOGGER", DEMO_LOGGER,
"\nPROMPT_TASK_DESCRIPTION", PROMPT_TASK_DESCRIPTION,
"\nRUN_POLICY_INFERENCE", RUN_POLICY_INFERENCE,
)

task_description_topic = "/task_description"
model_inference_command_topic = "/model_inference_command"


class RunnerNode(Node):
    """
    A node that runs the teleop and demo logger in a loop, allowing the user to control the robot with keyboard input.
    """
    def __init__(self):
        super().__init__(f"runner_node")
        self.move2init_clients, self.pause_clients, self.resume_clients  = [], [], []
        self.chiralities = [HandChirality.RIGHT if RIGHT_HAND else None, HandChirality.LEFT if LEFT_HAND else None]

        for chirality in self.chiralities:
            if chirality is not None:
                if TELEOP_BACK_TO_INIT:
                    self.move2init_clients.append(self.create_client(Trigger, f"/franka_teleop/{chirality}/move2init"))
                if TELEOP_PAUSE_AND_RESUME:
                    self.pause_clients.append(self.create_client(Trigger, f"/franka_teleop/{chirality}/pause"))
                    self.resume_clients.append(self.create_client(Trigger, f"/franka_teleop/{chirality}/resume"))
        if TELEOP_PAUSE_AND_RESUME and TELEOP_RETARGETER:
            self.retargeter_set_enable_clients = [
                self.create_client(SetBool, f"/{chirality}/retargeter/set_enabled")
                for chirality in self.chiralities
                if chirality is not None
            ]

        if DEMO_LOGGER:
            # create client for demo logger node
            self.start_recording_client = self.create_client(Trigger, "/demo_logger/start_recording")
            self.stop_recording_client = self.create_client(Trigger, "/demo_logger/stop_recording")
            self.save_recording_client = self.create_client(Trigger, "/demo_logger/save_recording")

        if PROMPT_TASK_DESCRIPTION:
            self.description_pub = self.create_publisher(ROSString, task_description_topic, 10)

        if RUN_POLICY_INFERENCE:
            self.get_logger().info(f"Model inference command topic: {model_inference_command_topic}")
            self.model_inference_cmd_pub = self.create_publisher(ROSString, model_inference_command_topic, 10)

    def loop_body(self):
        if TELEOP_BACK_TO_INIT:
            answer = None
            while answer not in ['y', 'n', '']:
                answer = input("Move to init pose? ([y]/n): ").lower().strip()
            if answer in ['y', '']:
                for move2init_client in self.move2init_clients:
                    move2init_client.call(Trigger.Request())

        if DEMO_LOGGER and PROMPT_TASK_DESCRIPTION:
            task_description = input("Enter a description for the task: ")
            self.get_logger().info(f"Task description published: {task_description}")


        if RUN_POLICY_INFERENCE:
            input("Press Enter to run policy inference...")
            if TELEOP_PAUSE_AND_RESUME:
                for pause_client in self.pause_clients:
                    pause_client.call(Trigger.Request())
                if TELEOP_RETARGETER:
                    for retargeter_client in self.retargeter_set_enable_clients:
                        retargeter_client.call(SetBool.Request(data=False))
            self.model_inference_cmd_pub.publish(ROSString(data="start"))
            self.get_logger().info("Model inference command published.")

            input("Press Enter to stop policy inference")
            self.model_inference_cmd_pub.publish(ROSString(data="stop"))

        if TELEOP_PAUSE_AND_RESUME:
            input("Press Enter to start teleop...")
            for resume_client in self.resume_clients:
                resume_client.call(Trigger.Request())
            if TELEOP_RETARGETER:
                for retargeter_client in self.retargeter_set_enable_clients:
                    retargeter_client.call(SetBool.Request(data=True))

        if DEMO_LOGGER:
            input("Press Enter to start recording...")
            self.start_recording_client.call(Trigger.Request())

            if PROMPT_TASK_DESCRIPTION:
                msg = ROSString()
                msg.data = task_description
                self.description_pub.publish(msg)

        input("Press Enter to stop and pause teleop...")
        if DEMO_LOGGER:
            self.stop_recording_client.call(Trigger.Request())

        if TELEOP_PAUSE_AND_RESUME:
            for pause_client in self.pause_clients:
                pause_client.call(Trigger.Request())
            if TELEOP_RETARGETER:
                for retargeter_client in self.retargeter_set_enable_clients:
                    retargeter_client.call(SetBool.Request(data=False))
        

        if DEMO_LOGGER:
            # Create a publisher for the task description topic
            # Ask to save or discard
            answer = None
            while answer not in ['y', 'n', '']:
                answer = input("Save recording? ([y]/n): ").lower().strip()
            if answer in ['y', '']:
                self.save_recording_client.call(Trigger.Request())
                self.get_logger().info("Recording saved.")
            else:
                self.get_logger().info("Recording discarded")
                

def main():    
    rclpy.init()
    runner_node = RunnerNode()

    import threading
    threading.Thread(target=rclpy.spin, args=(runner_node,)).start()

    keep_running = True
    while keep_running:
        runner_node.loop_body()
        
    runner_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
