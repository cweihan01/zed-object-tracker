import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped

class ZEDObjectTracker(Node):
    def __init__(self):
        super().__init__('zed_object_tracker')
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        for obj in msg.objects:
            # Extract confidence
            confidence = obj.confidence

            # Extract top-left and bottom-right from bounding_box_2d
            corners = obj.bounding_box_2d.corners
            if len(corners) == 4:
                top_left = corners[0].kp
                bottom_right = corners[2].kp
            else:
                self.get_logger().warn("Invalid number of corners in bounding_box_2d.")
                continue

            # Extract velocity
            velocity = obj.velocity  # velocity[0]=vx, [1]=vy, [2]=vz

            # Print to terminal
            self.get_logger().info(
                f"Label: {obj.label} | Confidence: {confidence:.1f} | "
                f"Top-left: {top_left} | Bottom-right: {bottom_right} | "
                f"Velocity: ({velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f})"
            )

            # Optionally write to file
            with open('/tmp/zed-object-tracker-log.csv', 'a') as f:
                f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec},"
                        f"{obj.label},{confidence:.1f},"
                        f"{top_left[0]:.2f},{top_left[1]:.2f},"
                        f"{bottom_right[0]:.2f},{bottom_right[1]:.2f},"
                        f"{velocity[0]:.2f},{velocity[1]:.2f},{velocity[2]:.2f}\n")

def main(args=None):
    rclpy.init(args=args)
    node = ZEDObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()