import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.create_subscription(
            CompressedImage,
            'AMR_image',
            self.image_callback,
            10  # 큐 사이즈 10으로 설정함
        )

        self.bridge = CvBridge()

    def image_callback(self, msg) :
    # ROS 메시지를 OpenCV 이미지로 변환
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1) 
        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
