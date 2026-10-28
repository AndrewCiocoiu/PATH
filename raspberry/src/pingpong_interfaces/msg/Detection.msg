# One detected ball in one image. Produced by detector_node (stage S3).
# header.stamp = time the IMAGE was taken (copied from the image), not the time of detection.
std_msgs/Header header

float32 u            # pixel x of the box centre
float32 v            # pixel y of the box centre
float32 width        # box width  [pixels]
float32 height       # box height [pixels]
float32 confidence   # 0 to 1
int32 class_id       # 0 = ping pong ball target (extend for YOLO classes)
