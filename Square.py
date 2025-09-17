import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Square(Node):
    def __init__(self):
        super().__init__('square_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Move turtle forward for set amount of time
    def move_forward(self, speed=1.0, duration=2.0):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.get_logger().info("Moving forward...")
        self._publish_for_duration(msg, duration)

    # Turns turtle approximately 90 degrees
    def turn(self, angular_speed=1.57, duration=1.0):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        self.get_logger().info("Turning...")
        self._publish_for_duration(msg, duration)

    def stop(self):
        msg = Twist()
        self.publisher_.publish(msg)

    def _publish_for_duration(self, msg, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)  # publish at ~10Hz
        self.stop()
        time.sleep(1.0)  # small pause

def main(args=None):
    rclpy.init(args=args)
    node = Square()

    try:
        for i in range(4):   # A loop repeated 4 times, each loop creating one side and turning 90 degrees
            node.move_forward(speed=1.0, duration=2.0)
            node.turn(angular_speed=1.57, duration=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
