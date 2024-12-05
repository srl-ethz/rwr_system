#!/usr/bin/env python3

import random
import time
import threading
import serial
import json

class ArduinoDriver:
    def __init__(self, callback, baud_rate=9600, device=None, debug=False):
        self.callback = callback
        self.baud_rate = baud_rate
        self.device = device
        self.running = False
        self.serial_connection = None
        self.debug = debug

    def run(self):
        self.running = True
        if self.device:
            self.serial_connection = serial.Serial(self.device, self.baud_rate)
            self.read_from_device()
        else:
            # Generate random values if no device is available
            self.generate_random_values()

    def stop(self):
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()


    def read_from_device(self):
        while self.running:
            if self.serial_connection.in_waiting > 0:
                try:
                    # Read a line of data from the serial port
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    # Assuming the data is in JSON format
                    data = json.loads(line)
                    self.callback(data)
                except Exception as e:
                    print(f"Error reading from device: {e}")
            time.sleep(0.01)

    def generate_random_values(self):
        while self.running:
            # Generate random sensor data
            data = {
                "pressure": {
                    "thumb": random.uniform(0, 1),
                    "index": random.uniform(0, 1),
                    "middle": random.uniform(0, 1),
                    "ring": random.uniform(0, 1),
                    "pinky": random.uniform(0, 1)
                },
                "fsr": {
                    "thumb": random.uniform(0, 1),
                    "index": random.uniform(0, 1),
                    "middle": random.uniform(0, 1),
                    "ring": random.uniform(0, 1),
                    "pinky": random.uniform(0, 1)
                }
            }
            self.callback(data)
            time.sleep(0.1)

if __name__ == "__main__":
    def example_callback(data):
        print(f"Received data: {data}")

    device = "/dev/ttyACM0"
    baud_rate = 115200

    driver = ArduinoDriver(callback=example_callback, device=device, baud_rate=baud_rate, debug=True)
    driver_thread = threading.Thread(target=driver.run)
    driver_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        driver.stop()
        driver_thread.join()