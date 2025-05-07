import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
import torch
import numpy as np 
# because NP 1.20+ removed np.float and byteTracker uses it, we alias it to np.float64
if not hasattr(np, 'float'): np.float = np.float64
from yolox.tracker.byte_tracker import BYTETracker
from types import SimpleNamespace

class ZEDObjectTracker(Node):
    def __init__(self):
        # arguments for the BYTETracker
        args = SimpleNamespace(
            track_thresh=0.5,   # confidence threshold for tracking a bounding box
            track_buffer=30,    # how many frames to keep in the buffer in case of occlusion
            match_thresh=0.8,   # confidence threshold for matching a bounding box to earlier ones for consistent ID-ing
            mot20=False,        # we aer not using the MOT20 dataset
            min_box_area=10,    # minimum area of a bounding box to be considered for tracking
        )
        self.tracker = BYTETracker(args, frame_rate=30)
        self.img_size = (360, 640)  
        self.info_imgs = (360, 640) 
        super().__init__('zed_object_tracker')
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        dets = []
        for obj in msg.objects:
            # Extract confidence
            confidence = obj.confidence

            # Extract top-left and bottom-right from bounding_box_2d
            corners = obj.bounding_box_2d.corners
            if len(corners) == 4:
                top_left = corners[0].kp
                bottom_right = corners[2].kp
                x1, y1 = top_left[0], top_left[1]
                x2, y2 = bottom_right[0], bottom_right[1]
                dets.append([x1, y1, x2, y2, confidence])
            else:
                self.get_logger().warn("Invalid number of corners in bounding_box_2d.")
                continue

            # Extract velocity
            velocity = obj.velocity  # velocity[0]=vx, [1]=vy, [2]=vz

            # Print message to terminal
            self.get_logger().info(
                f"Label: {obj.label} | Confidence: {confidence:.1f} | "
                f"Top-left: {top_left} | Bottom-right: {bottom_right} | "
                f"Velocity: ({velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f})"
            )

            if not dets:
                return

            # Convert detections to PyTorch tensor
            dets_tensor = torch.tensor(dets)

            # Run tracker
            online_targets = self.tracker.update(dets_tensor, self.info_imgs, self.img_size)

            for t in online_targets:
                tlwh = t.tlwh
                tid = t.track_id
                score = t.score

                x1, y1, w, h = tlwh
                x2 = x1 + w
                y2 = y1 + h

                self.get_logger().info(
                    f"[TRACK ID {tid}] Score: {score:.2f} | "
                    f"Top-left: ({x1:.1f}, {y1:.1f}) | Bottom-right: ({x2:.1f}, {y2:.1f}) | "
                    f"Width: {w:.1f}, Height: {h:.1f}"
                )
            


def main(args=None):
    rclpy.init(args=args)
    node = ZEDObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()