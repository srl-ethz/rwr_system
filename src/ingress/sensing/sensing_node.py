#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float32
from faive_system.src.ingress.sensing.sensing_ingress import ArduinoDriver

class SensorPublisher(Node):
    def __init__(self):
        super().__init__("sensor_publisher")
        self.sensing_data_lock = threading.Lock()
        
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("device", "/dev/ttyACM0")
        
        self.baud_rate = self.get_parameter("baud_rate").value
        self.device = self.get_parameter("device").value
        
        
        self.sensing_data = {"pressure": {}, "fsr": {}}
        self.sensor_publishers = {"pressure": {}, "fsr": {}}
        
        self.get_logger().info("Started sensor publisher node")

        self.init_sensors()
    
    def init_sensors(self):
        self.get_logger().info("Initializing sensors")
        if not self.device:
            self.get_logger().warn("No device specified. Using random sensor data.")
        self.arduino_driver = ArduinoDriver(callback=self.recv_sensing_data, device=self.device, baud_rate=self.baud_rate)
        self.get_logger().info("Starting Arduino driver with device")
        self.get_logger().info(self.device)
        driver_thread = threading.Thread(target=self.arduino_driver.run)
        driver_thread.start()
        
        fsr_sensor_names = ["thumb", "index", "middle", "ring", "pinky"]
        for sensor_name in fsr_sensor_names:
            self.sensing_data["fsr"][sensor_name] = 0.0
            topic_name = f"fsr/{sensor_name}"
            self.sensor_publishers["fsr"][sensor_name] = self.create_publisher(Float32, topic_name, 10)
        
        pressure_sensor_names = ["thumb", "index", "middle", "ring", "pinky"]
        for sensor_name in pressure_sensor_names:
            self.sensing_data["pressure"][sensor_name] = 0.0
            topic_name = f"pressure/{sensor_name}"
            self.sensor_publishers["pressure"][sensor_name] = self.create_publisher(Float32, topic_name, 10)
        
    def recv_sensing_data(self, data):
        with self.sensing_data_lock:
            for sensor_type, sensor_data in data.items():
                if sensor_type in self.sensing_data:
                    for sensor_name, sensor_value in sensor_data.items():
                        if not isinstance(sensor_value, float):
                            sensor_value = float(sensor_value)
                        if sensor_name in self.sensing_data[sensor_type]:
                            self.sensing_data[sensor_type][sensor_name] = sensor_value
                        else:
                            self.get_logger().warn(f"Sensor name {sensor_name} not found in {sensor_type}")
                else:
                    self.get_logger().warn(f"Sensor type {sensor_type} not found in sensing data")

    def publish_sensing_data(self):
        with self.sensing_data_lock:
            # Publish pressure sensor data
            for sensor_name, sensor_value in self.sensing_data["pressure"].items():
                msg = Float32()
                msg.data = sensor_value
                self.sensor_publishers["pressure"][sensor_name].publish(msg)

            # Publish FSR sensor data
            for sensor_name, sensor_value in self.sensing_data["fsr"].items():
                msg = Float32()
                msg.data = sensor_value
                self.sensor_publishers["fsr"][sensor_name].publish(msg)

def main():
    rclpy.init()
    sensor_publisher = SensorPublisher()
    spin_thread = threading.Thread(target=rclpy.spin, args=(sensor_publisher,))
    spin_thread.start()

    try:
        while rclpy.ok():
            sensor_publisher.publish_sensing_data()
            time.sleep(0.1)  # Publish rate of 10 Hz
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    spin_thread.join()

if __name__ == "__main__":
    main()