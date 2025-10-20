import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time

class FeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('feedback_subscriber')
        self.sub = self.create_subscription(String, '/data_topic', self.callback, 10)
        self.feedback_pub = self.create_publisher(Float32, '/feedback_topic', 10)

        self.last_msg_time = time.time()
        self.processed_count = 0
        self.backlog_estimate = 0.0

    def callback(self, msg):
        now = time.time()
        delay = now - self.last_msg_time
        self.last_msg_time = now
        if delay < 1.0 / 10.0:  
            self.backlog_estimate = min(1.0, self.backlog_estimate + 0.1)
        else:
            self.backlog_estimate = max(0.0, self.backlog_estimate - 0.05)

        self.get_logger().info(f"Received: {msg.data}, backlog={self.backlog_estimate:.2f}")

        feedback_msg = Float32()
        feedback_msg.data = self.backlog_estimate
        self.feedback_pub.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
