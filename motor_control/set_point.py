# Import libraries
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult  # Import required class

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Declare dynamic parameters for rqt_reconfigure
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('signal_type', 'sine')  # Options: 'sine', 'square', 'step'

        # Initialize parameters
        self.update_parameters()

        # Create publisher
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)

        # Create a timer
        self.timer = self.create_timer(0.1, self.timer_cb)

        # Start time for time-based signals
        self.start_time = self.get_clock().now()

        # Listen for parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("SetPoint Node Started 🚀")

    def update_parameters(self):
        """ Updates set point parameters from YAML or rqt_reconfigure """
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
        self.signal_type = self.get_parameter('signal_type').value
        self.get_logger().info(f"Updated Params - Amplitude: {self.amplitude}, Omega: {self.omega}, Signal Type: {self.signal_type}")

    def timer_cb(self):
        """ Generates and publishes the selected type of signal """
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Ensure signal_type is valid
        if self.signal_type not in ["sine", "square", "step"]:
            self.get_logger().error(f"Invalid signal type: {self.signal_type}. Defaulting to sine wave.")
            self.signal_type = "sine"

        # Select the appropriate signal type
        if self.signal_type == "sine":
            signal_value = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.signal_type == "square":
            signal_value = self.amplitude if np.sin(self.omega * elapsed_time) >= 0 else -self.amplitude
        elif self.signal_type == "step":
            signal_value = self.amplitude if elapsed_time >= 1.0 else 0.0  # Step function at t=1s

        # Publish the signal
        self.signal_msg = Float32()
        self.signal_msg.data = signal_value
        self.signal_publisher.publish(self.signal_msg)

    def parameter_callback(self, params):
        """ Updates parameters dynamically when changed via rqt_reconfigure """
        for param in params:
            if param.name == "amplitude":
                self.amplitude = param.value
            elif param.name == "omega":
                self.omega = param.value
            elif param.name == "signal_type":
                self.signal_type = param.value

        self.get_logger().info(f"Parameter Updated - Amplitude: {self.amplitude}, Omega: {self.omega}, Signal Type: {self.signal_type}")

        return SetParametersResult(successful=True)  # Fix: Correct return type

def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()