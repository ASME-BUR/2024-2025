import rclpy
from rclpy.node import Node

from bur_msgs.msg import ThrusterCommand
from std_msgs.msg import Float64

class ThrusterMiddleware(Node):
    def __init__(self):
        super().__init__('thruster_middleware')

        self.thruster_cmd_listener = self.create_subscription(
            ThrusterCommand,
            '/thrust/cmd_pub_topic',
            self.listener_callback,
            10)

        self.thruster_pub = []

        for i in range(8):
            self.thruster_pub.append(self.create_publisher(Float64, '/model/bur/joint/thruster{}/cmd_pos'.format(i), 10))


    def listener_callback(self, thrust_cmd):
        msg = Float64()
        for i in range(8):
            msg.data = thrust_cmd.thrusters[i]
            self.thruster_pub[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)

    thrusters_middleware = ThrusterMiddleware()

    rclpy.spin(thrusters_middleware)

    thrusters_middleware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
