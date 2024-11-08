import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class  Lidar(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('lidar')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359] 
        self.laser_left = min (msg.ranges[0:90])
        self.laser_right = min(msg.ranges[269:359])
        self.safe_distance= 1.0

        self.inf_distance = float('inf')

        
        
        
    def motion(self):
        # print the data
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        self.get_logger().info('On my left :"%s"' % str(self.laser_left))
        self.get_logger().info('On my right :"%s"' % str(self.laser_right))
        # Logic of move
        if self.laser_forward >1:
            self.cmd.linear.x = 0.2  #move vooruit 
            self.cmd.angular.z= 0.0
       
        if (self.laser_left < 1):
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -0.5  #move right when the left side is smaller than 1
        if (self.laser_right <1):
            self.cmd.linear.x =0.1
            self.cmd.angular.z = 0.5  #move left when the right side is smaller than 1

        if (self.laser_forward > self.safe_distance):  # blijf vooruit bewegen
            self.cmd.linear.x= 0.2 
            self.cmd.angular.z= 0.0

        elif(self.laser_left > self.laser_forward and self.laser_left > self.safe_distance): #meer plaats links dan voorwaards en het is vijlig --> move left
            self.cmd.linear.x = 0.1
            self.cmd.angular.z= -0.2

        elif(self.laser_right > self.laser_forward and self.laser_left > self.safe_distance): #meer plaats rechts dan voorwaards en het is vijlig --> move right
            self.cmd.linear.x = 0.1
            self.cmd.angular.z= 0.2

        elif(self.laser_left == self.laser_forward and self.laser_right == self.laser_forward): #move vooruit 
            self.cmd.linear.x = 0.2  
            self.cmd.angular.z= 0.0
        else:
            if (self.laser_forward <1 and self.laser_forward >= 0.4):
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = 0.5
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.1


        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    lidar = Lidar()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(lidar)
    # Explicity destroy the node
    lidar.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()