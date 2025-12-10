import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStampedNode(Node):
    def __init__(self):
        super().__init__('twist_to_stamped_node')
        # Create a subscriber for the unstamped Twist message
        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        # Create a publisher for the stamped Twist message
        self.stamped_twist_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value

    def twist_callback(self, msg):
        stamped_twist_msg = TwistStamped()
        stamped_twist_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_twist_msg.header.frame_id = self.frame_id
        stamped_twist_msg.twist = msg
        self.stamped_twist_pub.publish(stamped_twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToStampedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
