import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge, CvBridgeError  # Import CvBridge to convert ROS2 Image messages to OpenCV format
import numpy as np
import os
import itertools

K = 2
path = "/home/robot/greenline/src/alignment/alignment/debug-images"

queue = []
debug_iter = 0

def remove_noise(image: np.ndarray) -> np.ndarray:
    image = cv2.GaussianBlur(image, (3, 3), 10)
    # image = cv2.medianBlur(image, 5)
    return image


def detect_edges(image: np.ndarray) -> np.ndarray:
    low_threshold = 50
    high_threshold = 150

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    binarized_image = cv2.Canny(gray_image, low_threshold, high_threshold)
    return binarized_image


def find_lines(binarized_image: np.ndarray) -> np.ndarray:
    threshold = 150
    rho_step = np.pi / 180

    lines = cv2.HoughLines(binarized_image, 1, rho_step, threshold=threshold)

    if lines is None:
        return []
    else:
        return [line[0] for line in lines]
    

def cluster_lines(lines: list[tuple[float, float]]) -> list[tuple[float, float]]:
    lines_array = np.array(lines).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    _, labels, _ = cv2.kmeans(lines_array.astype(np.float32), K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    
    # Group the lines by their cluster
    clustered_lines = [[] for _ in range(K)]
    for i, label in enumerate(labels.flatten()):
        clustered_lines[label].append(lines[i])
    
    # Calculate the mean (rho, theta) for each cluster
    centroid_lines = []
    for cluster in clustered_lines:
        if cluster:
            mean_rho = np.mean([line[0] for line in cluster])
            mean_theta = np.mean([line[1] for line in cluster])
            centroid_lines.append((mean_rho, mean_theta))
    
    return centroid_lines


def is_aligned(centroid_lines: tuple[float, float]) -> bool:
    expected_angles = [(-35, 10), (35, 10)]

    # Define the expected angles and their tolerances
    thetas = [np.degrees(line[1]) - 90 for line in centroid_lines]

    print(thetas)

    count = 0
    for expected, tolerance in expected_angles:
        for theta in thetas:
            if abs(theta - expected) < tolerance:
                count += 1
                break

    return count == 2


def debug_lines(image: np.ndarray, lines: list[tuple[float, float]], line_weight=2, color=(0, 0, 255)):
    for line in lines:
        rho, theta = line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = int(a * rho)
        y0 = int(b * rho)
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(image, (x1, y1), (x2, y2), color, line_weight)
    return image

def filter_horizontal(lines):
    thetas = [np.degrees(line[1]) - 90 for line in lines]
    filtered_lines = []
    margin = 5
    for theta, line in zip(thetas, lines):
        if theta > margin or theta < -margin:
            filtered_lines.append(line)
    return filtered_lines


class IsAlignedNode(Node):
    def __init__(self):
        super().__init__('is_aligned_node')

        self.image_subscriber = self.create_subscription(
            Image,
            #'/robot1/D435i/depth/image_rect_raw',
            '/robot1/D435i/color/image_raw',
            self.check_alignment_callback,
            10
        )
        self.aligned_publisher = self.create_publisher(Bool, '/is_aligned', 100)
        self.bridge = CvBridge()
        
        self.declare_parameter("debug", True)
        self.get_logger().info("is_aligned_node has been started.")

    def check_alignment_callback(self, msg):
        global debug_iter
        debug_iter += 1
        debug_iter %= 30

        debug = self.get_parameter("debug").value
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Converting image to OpenCV format
            
            # Cut the top portion of the image
            height, _ = image.shape[:2]
            image = image[height // 2:]

            alignment_msg = Bool()

            # Line detection pipeline
            denoised_image = remove_noise(image)
            binarized_image = detect_edges(denoised_image)
            lines = find_lines(binarized_image)
            lines = filter_horizontal(lines)

            queue.append(lines)
            if len(queue) > 10:
                queue.pop(0)
            lines = list(itertools.chain.from_iterable(queue)) # flatten the list of line lists

            if debug and debug_iter == 0:
                #cv2.imwrite(os.path.join(path, "original_image.png"), image)
                #cv2.imwrite(os.path.join(path, "denoised_image.png"), denoised_image)
                cv2.imwrite(os.path.join(path, "binarized_image.png"), binarized_image)
                #cv2.imwrite(os.path.join(path, "detected_lines.png"), debug_lines(image, lines))

            if len(lines) < K:
                alignment_status = False
                alignment_msg.data = False
            else:
                centroid_lines = cluster_lines(lines)

                alignment_status = is_aligned(centroid_lines)
                alignment_msg.data = alignment_status

                if debug and debug_iter == 0:
                    if alignment_status:
                        color = (0, 255, 0)
                    else:
                        color = (0, 0, 255)
                    cv2.imwrite(os.path.join(path, "clustered_lines.png"), debug_lines(image, centroid_lines, line_weight=15, color=color))

            self.aligned_publisher.publish(alignment_msg)
            self.get_logger().info(f'Alignment status: {alignment_status}, {len(lines)} lines detected')
        
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

        
def main(args=None):
    rclpy.init(args=args)
    is_aligned_node = IsAlignedNode()
    try:
        rclpy.spin(is_aligned_node)
    except KeyboardInterrupt:
        pass

    is_aligned_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
