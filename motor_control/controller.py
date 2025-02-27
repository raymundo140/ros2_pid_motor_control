# Import libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult  # Import required class

class PIDController(Node):
    def __init__(self):
        super().__init__('ctrl')

        # Declare parameters with rqt_reconfigure support
        self.declare_parameter('Kp', 5.5)
        self.declare_parameter('Ki', 5.0)
        self.declare_parameter('Kd', 0.02)
        self.declare_parameter('sample_time', 0.02)

        # Initialize PID parameters
        self.update_parameters()

        # PID variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.control_signal = 0.0
        self.set_point = 0.0
        self.motor_speed = 0.0

        # Publishers & Subscribers
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.motor_speed_sub = self.create_subscription(Float32, 'motor_speed_y', self.motor_speed_callback, 10)

        # Timer for PID loop
        self.timer = self.create_timer(self.sample_time, self.control_loop)

        # Listen for parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("PID Controller Node Started 🚀")

    def update_parameters(self):
        """ Updates PID parameters from YAML or rqt_reconfigure """
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.sample_time = self.get_parameter('sample_time').value

    def control_loop(self):
        """ PID Control Algorithm with Square Wave Handling """
        error = self.set_point - self.motor_speed

        # Limit the integral term
        integral_limit = 5.0  
        self.integral += error * self.sample_time
        self.integral = max(min(self.integral, integral_limit), -integral_limit)

        # Ignore large jumps for derivative term
        if abs(error - self.prev_error) > self.set_point:
            derivative = 0
        else:
            derivative = (error - self.prev_error) / self.sample_time

        # PID Control Equation
        raw_control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Smooth the control output using a low-pass filter
        alpha = 0.2  
        self.control_signal = alpha * raw_control_signal + (1 - alpha) * self.control_signal

        # Publish the control signal
        motor_input_msg = Float32()
        motor_input_msg.data = self.control_signal
        self.motor_input_pub.publish(motor_input_msg)

        self.prev_error = error

    def set_point_callback(self, msg):
        """ Updates set point value """
        self.set_point = msg.data

    def motor_speed_callback(self, msg):
        """ Updates motor speed value """
        self.motor_speed = msg.data

    def parameter_callback(self, params):
        """ Updates parameters dynamically when changed via rqt_reconfigure """
        for param in params:
            if param.name in ["Kp", "Ki", "Kd", "sample_time"]:
                setattr(self, param.name, param.value)

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    try:
        rclpy.spin(pid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pid_controller.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()