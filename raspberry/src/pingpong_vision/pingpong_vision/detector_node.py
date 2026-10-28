"""detector_node - Stage S3 Detection.               OWNER: Sofian

Doc: section 4.2 (S3), section 13 (OpenCV or YOLO is an open decision).

Subscribes:  /camera/image_raw     sensor_msgs/Image
Publishes:   /vision/detections    pingpong_interfaces/DetectionArray
             (TODO) /vision/debug_image  image with the box drawn, for the live view

Parameter `method`:
    'color' (v1, default)  colour threshold on orange. Works with the synthetic camera.
    'yolo'  (v2)           TODO(Sofian)

Both methods MUST publish the same DetectionArray. Nothing after this node may
care which method is used.
Rule: copy the image header stamp into the detections (time of the image, not now).
"""
import numpy as np
import rclpy
from pingpong_interfaces.msg import Detection, DetectionArray
from rclpy.node import Node
from sensor_msgs.msg import Image


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameter('method', 'color')
        self.declare_parameter('min_pixels', 20)
        self.method = self.get_parameter('method').value
        if self.method == 'yolo':
            # TODO(Sofian): load the YOLO model here (ultralytics or Hailo), once.
            raise NotImplementedError("method:=yolo is not implemented yet (Sofian)")

        self.pub = self.create_publisher(DetectionArray, '/vision/detections', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.get_logger().info(f'detector_node started, method={self.method}')

    def on_image(self, img):
        frame = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3)
        out = DetectionArray()
        out.header = img.header          # keep the IMAGE timestamp
        det = self.detect_color(frame)
        if det is not None:
            det.header = img.header
            out.detections.append(det)
        self.pub.publish(out)

    def detect_color(self, rgb):
        # TODO(Sofian): replace with cv2: convert to HSV, inRange, morphology, findContours,
        # pick the most circular blob. Tune thresholds on real images under lab lighting.
        r = rgb[:, :, 0].astype(np.int16)
        g = rgb[:, :, 1].astype(np.int16)
        b = rgb[:, :, 2].astype(np.int16)
        mask = (r > 180) & (g > 60) & (g < 180) & (b < 80)
        ys, xs = np.nonzero(mask)
        if xs.size < self.get_parameter('min_pixels').value:
            return None
        det = Detection()
        det.u = float(xs.mean())
        det.v = float(ys.mean())
        det.width = float(xs.max() - xs.min() + 1)
        det.height = float(ys.max() - ys.min() + 1)
        # filled fraction of the bounding box; a perfect disc gives pi/4
        fill = xs.size / (det.width * det.height)
        det.confidence = float(min(1.0, fill / (np.pi / 4)))
        det.class_id = 0
        return det


def main():
    rclpy.init()
    rclpy.spin(DetectorNode())


if __name__ == '__main__':
    main()
