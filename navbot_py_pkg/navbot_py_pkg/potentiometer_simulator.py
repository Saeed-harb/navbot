#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk

class PotentiometerSimulator(Node):
    def __init__(self):
        super().__init__('potentiometer_simulator')
        self.publisher_ = self.create_publisher(Float64, 'setpoint', 10)

        # Create GUI
        self.root = tk.Tk()
        self.root.title("Potentiometer Speed Simulator")

        self.scale = tk.Scale(self.root,
                              from_=-200.0,  # Min speed
                              to=400.0,       # Max speed
                              orient='horizontal',
                              length=400,
                              resolution=0.1,
                              label='Speed (RPM)')
        self.scale.pack(padx=20, pady=20)

        # Update at regular intervals
        self.timer = self.create_timer(0.1, self.publish_speed)  # 10 Hz

    def publish_speed(self):
        value = float(self.scale.get())
        msg = Float64()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing speed: {value:.2f}")

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                self.root.update_idletasks()
                self.root.update()
        except tk.TclError:
            pass  # Happens when GUI window is closed
        finally:
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = PotentiometerSimulator()
    node.run()

if __name__ == '__main__':
    main()
