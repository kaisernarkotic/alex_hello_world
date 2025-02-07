import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from cv_bridge import CvBridge
import cv2
import numpy as np

class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('alex_hello_world')

        self.logger = self.get_logger()

        self.zed_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_cb,
            10)

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.bridge = CvBridge()

        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])

        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        self.logger.info('Zed camera node started')
    
    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            red_mask = cv2.inRange(hsv_image, self.lower_red, self.upper_red)
            blue_mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)

            #total_pixels = hsv_image.shape[0] * hsv_image.shape[1]
            red_pixels = np.sum(red_mask > 0)
            blue_pixels = np.sum(blue_mask > 0)
            
            #red_ratio = red_pixels/total_pixels
            #blue_ratio = blue_pixels/total_pixels
            detected_color = "none"
            if red_pixels > 3000:
                detected_color = "red"
            elif blue_pixels > 3000:
                detected_color = "blue"
            
            self.logger.info('Color processing done')

            self.turn(detected_color)

        except Exception as e:
            self.logger.error(f"Error processing image: {e}")
    
    def turn(self, msg):
        stamped_msg = AckermannDriveStamped()
        stamped_msg.drive = AckermannDrive()
        if (msg == "red"):  
            stamped_msg.drive.steering_angle = 30.0
            self.logger.info("Red detected. Turning left.")
        elif (msg == "blue"):
            stamped_msg.drive.steering_angle = -30.0
            self.logger.info("Blue detected. Turning right.")
        else:
            stamped_msg.drive.steering_angle = 0.0
            self.logger.info("No significant color detected. Keeping straight.")
        
        self.publisher.publish(stamped_msg)
        
        

def main(args=None):
    rclpy.init(args=args)
    ackermann_publisher = AckermannPublisher()
    try:
        rclpy.spin(ackermann_publisher)
    except KeyboardInterrupt:
        ackermann_publisher.logger.info("Interrupted by user.")
    finally:
        ackermann_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
