#!/usr/bin/env python3
import os
import cv2
import numpy as np
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String ,Float64
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node


class ImageSaverAnd3DDetector(Node):
    def __init__(self):
        super().__init__('image_saver_and_3d_detector')

        # Subscription to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/depth_cam/image_raw',
            self.listener_callback,
            10
        )

        # Subscription to the /capture topic
        self.capture_subscription = self.create_subscription(
            Bool,
            '/capture',
            self.capture_callback,
            10
        )

        self.ball_pose_pub = self.create_publisher(Point, '/ball_pose', 10)
        self.ball_color_pub = self.create_publisher(String, '/ball_color', 10)

        self.bridge = CvBridge()

        # Directory to save the processed images
        self.save_folder = os.path.join(os.getcwd(), 'config', 'picture')
        os.makedirs(self.save_folder, exist_ok=True)
        self.clear_folder(self.save_folder)

        # Camera intrinsic parameters
        self.fx = 525.0  # Focal length x
        self.fy = 525.0  # Focal length y
        self.cx = 319.5  # Principal point x
        self.cy = 239.5  # Principal point y

        # State variables for /capture
        self.last_capture_state = False
        self.current_capture_state = False
        self.last_save_time = time.time()

        # A variable to store the last processed image
        self.last_processed_image = None

        # A variable to store the last detected info
        self.last_detected_info = []

        # Dictionary to store image sets (color/position info)
        self.image_sets = {}
        # Counter for the saved images
        self.image_count = 0

        # Minimum radius for a ball to be considered "big enough" to store
        self.min_radius = 20

        # จำนวนภาพที่ต้องการถ่าย ก่อนจะสรุปผล
        self.target_image_count = 8

    @staticmethod
    def clear_folder(folder_path):
        """Clear all files in the folder."""
        for file_name in os.listdir(folder_path):
            file_path = os.path.join(folder_path, file_name)
            if os.path.isfile(file_path):
                os.remove(file_path)

    def capture_callback(self, msg):
        """Callback for the /capture topic."""
        self.current_capture_state = msg.data

    def listener_callback(self, msg):
        """Callback for receiving images."""
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Always detect shapes (log X, Y, Z if found)
            processed_image, detected_info = self.detect_shapes(cv_image)

            # Calculate and display 3D distances (just for visualization)
            self.calculate_and_draw_3d_distances(processed_image, detected_info)

            # Publish ball positions that are large enough
            for ball in detected_info:
                if ball['radius'] > self.min_radius:
                    p = Point()
                    c = String()
                    p.x, p.y, p.z = ball['3D_position']
                    c.data = ball['color']
                    print(c)
                    self.ball_pose_pub.publish(p)
                    self.ball_color_pub.publish(c)

            # Store the processed image and detected_info in memory
            self.last_processed_image = processed_image
            self.last_detected_info = detected_info

            # If /capture was just triggered (False -> True), save the image
            if self.current_capture_state and not self.last_capture_state:
                self.save_image(processed_image, detected_info)

            # Update the capture state
            self.last_capture_state = self.current_capture_state

        except Exception as e:
            self.get_logger().error(f'Failed to process the image: {e}')

    def save_image(self, image, detected_info):
        """Save the currently processed image and record sorted ball info."""
        current_time = time.time()
        filename = os.path.join(self.save_folder, f'detect_circle_{int(current_time)}.jpg')
        cv2.imwrite(filename, image)
        self.get_logger().info(f'Image saved with detection: {filename}')

        # Filter to include only balls that are large enough
        large_balls = [ball for ball in detected_info if ball['radius'] > self.min_radius]

        # If no large balls, do not store any info for this image
        if not large_balls:
            self.get_logger().info("No large enough balls found in this image.")
            self.image_count += 1
        else:
            # Sort large balls by their X-position
            sorted_balls = sorted(large_balls, key=lambda b: b['3D_position'][0])

            # Create a list of dicts with color and position
            balls_info = []
            for ball in sorted_balls:
                color = ball['color']
                position = ball['3D_position']
                balls_info.append({
                    'color': color,
                    'position': position
                })

            # Increment image count
            self.image_count += 1
            # Store the list under this image count
            self.image_sets[self.image_count] = balls_info

            # Log the stored information
            self.get_logger().info(f'Image Set {self.image_count}: {self.image_sets[self.image_count]}')

        if self.image_count == self.target_image_count:
            self.summarize_balls()

    def summarize_balls(self):
        """Summarize how many balls of each color from all captured sets."""
        color_counts = {}
        for _, balls_info in self.image_sets.items():
            for ball in balls_info:
                c = ball['color']
                color_counts[c] = color_counts.get(c, 0) + 1

        self.get_logger().info("===== Summary of Balls by Color =====")
        for color, count in color_counts.items():
            self.get_logger().info(f"Color {color}: {count} ball(s)")

    def detect_shapes(self, frame):
        """Detect circles, annotate them, and log their color and position."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = {
            "Red": cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) +
                   cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255])),
            "Green": cv2.inRange(hsv, np.array([35, 100, 100]), np.array([85, 255, 255])),
            "Blue": cv2.inRange(hsv, np.array([100, 100, 100]), np.array([140, 255, 255])),
            "Yellow": cv2.inRange(hsv, np.array([25, 100, 100]), np.array([35, 255, 255])),
            "Pink": cv2.inRange(hsv, np.array([145, 100, 100]), np.array([160, 255, 255]))
        }

        combined_mask = sum(masks.values())
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        detected_info = []

        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

            if radius > 10:
                depth_value = self.get_depth_value(center)
                X, Y, Z = self.calculate_3d_position(center, depth_value)
                color_name = self.get_color_name(hsv[int(y), int(x)])

                # Draw detection results
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.putText(frame, f'{color_name} (X={X:.3f}, Y={Y:.3f}, Z={Z:.3f})',
                            (center[0] - 50, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                # Log the detected ball info (X, Y, Z)
                self.get_logger().info(
                    f'Detected ball: Color={color_name}, Position=(X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}), Radius={radius}'
                )

                detected_info.append({
                    'center': center,
                    'radius': radius,
                    'color': color_name,
                    'depth': depth_value,
                    '3D_position': (X, Y, Z)
                })

        return frame, detected_info

    def calculate_3d_position(self, center, depth):
        """Calculate 3D coordinates from 2D pixel coordinates."""
        x_pixel, y_pixel = center
        Z = depth
        X = (x_pixel - self.cx) * Z / self.fx
        Y = (y_pixel - self.cy) * Z / self.fy
        return X, Y, Z

    def calculate_and_draw_3d_distances(self, frame, detected_info):
        """Calculate and display 3D distances between objects."""
        num_balls = len(detected_info)
        if num_balls < 2:
            return

        for i in range(num_balls):
            for j in range(i + 1, num_balls):
                ball1 = detected_info[i]
                ball2 = detected_info[j]

                X1, Y1, Z1 = ball1['3D_position']
                X2, Y2, Z2 = ball2['3D_position']

                distance = np.sqrt((X2 - X1)**2 + (Y2 - Y1)**2 + (Z2 - Z1)**2)
                mid_point = ((ball1['center'][0] + ball2['center'][0]) // 2,
                             (ball1['center'][1] + ball2['center'][1]) // 2)

                cv2.putText(frame, f'Dist: {distance:.3f}m', mid_point,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def get_depth_value(self, center):
        """Return fixed depth value of 0.262 meters."""
        return 0.262

    @staticmethod
    def get_color_name(hsv_color):
        """Determine color name based on HSV values."""
        h, s, v = hsv_color
        if 0 <= h <= 10 or 160 <= h <= 180:
            return "Red"
        elif 35 <= h <= 85:
            return "Green"
        elif 100 <= h <= 140:
            return "Blue"
        elif 25 <= h <= 35:
            return "Yellow"
        elif 145 <= h <= 160:
            return "Pink"
        return "Unknown"


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverAnd3DDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()