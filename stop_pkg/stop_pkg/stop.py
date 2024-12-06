import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Stop(Node):

    def __init__(self):
        
        super().__init__('stop')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        
        timer_period = 0.5
        self.laser_forward = float('inf')
        self.laser_left = float('inf')
        self.laser_right = float('inf')
       

        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.motion)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[0] 
        self.laser_left = min([r for r in msg.ranges[0:45] if r != float('inf')], default=float('inf'))
        self.laser_right = min([r for r in msg.ranges[-45:] if r != float('inf')], default=float('inf'))
        self.safe_distance = 0.5
    
    def motion(self):
        self.get_logger().info('Laser forward: "%s"' % str(self.laser_forward))
        self.get_logger().info('Laser left: "%s"' % str(self.laser_left))
        self.get_logger().info('Laser right: "%s"' % str(self.laser_right))


        if self.laser_forward <0.3:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z=0.0
        else:
            self.cmd.linear.x=0.2
            self.cmd.angular.z=0.0

        self.publisher_.publish(self.cmd)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    stop = Stop()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(stop)
    # Explicity destroys the node
    stop.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()