import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time

class AdaptivePublisher(Node):
    def __init__(self):
        super().__init__('adaptive_publisher')
        self.publisher_ = self.create_publisher(String, '/data_topic', 10)
        self.feedback_sub_ = self.create_subscription(Float32, '/feedback_topic', self.feedback_callback, 10)
        
        self.publish_rate = 10.0  # initial 10 Hz
        self.last_feedback_time = time.time()

        self.get_logger().info(f"Starting with rate {self.publish_rate:.1f} Hz")

        # Timer for dynamic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def feedback_callback(self, msg):
        feedback = msg.data
        now = time.time()
        time_since_feedback = now - self.last_feedback_time
        
        if feedback > 0.5: 
            self.publish_rate = max(1.0, self.publish_rate * 0.8)
        elif feedback < 0.2: 
            self.publish_rate = min(20.0, self.publish_rate * 1.1)

        self.get_logger().info(f"Feedback={feedback:.2f}, new rate={self.publish_rate:.2f} Hz")

        self.timer.cancel()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.last_feedback_time = now

    def timer_callback(self):
        msg = String()
        msg.data = f"Message at {time.time():.2f}"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
