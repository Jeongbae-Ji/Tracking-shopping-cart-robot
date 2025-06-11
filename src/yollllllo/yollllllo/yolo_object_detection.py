import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
import threading

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'AMR_image', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.bridge = CvBridge()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        model_path = os.path.join(parent_dir, 'best.pt')
        self.model = YOLO(model_path)
        self.camera_index = 0
        self.max_camera_index = 5
        self.cap = None
        self.boundery = 0
        self.initialize_camera()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.carheight = None # 객체인식된 자동차 높이
        self.dum_height = None # 객체인식된 더미 높이
        self.obstacle_avoiding = False  # 장애물 회피 상태 플래그
        self.obstacle_timer = None     # 회피 동작 종료 타이머
    
    def initialize_camera(self):
        while self.camera_index < self.max_camera_index:
            self.cap = cv2.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                self.get_logger().info(f"카메라 {self.camera_index} 초기화 성공")
                return
            else:
                self.get_logger().warn(f"카메라 {self.camera_index} 초기화 실패, 다음 카메라 시도")
                self.camera_index += 1

        self.get_logger().error("사용 가능한 카메라를 찾을 수 없습니다.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("웹캠이 인식되지 않아 프레임 캡쳐에 실패했습니다...")
            return

        camera_center_x = frame.shape[1] // 2 #카메라 중심점
        results = self.model.track(source=frame, show=False, tracker='bytetrack.yaml')
        carcenter_x = None
        carcenter_y = None
        dumcenter_x = None
        twist = Twist() #속도제어

        for result in results:
            for detection in result.boxes.data:# 검출된 객체 데이터 출력(x1, y1, x2, y2, confidence, class_id)
                if len(detection) < 7:
                    detection = list(detection) + [100] * (7 - len(detection))
                if len(detection) >= 6:
                    x1, y1, x2, y2, track_id, confidence, class_id = detection[:7]
                    class_confidence = detection[5]  # 클래스 confidence 값
                    if class_confidence < 0.85: #confidence 0.85미만일 경우 무시
                        continue 
                    
                    if int(class_id) == 1:
                        dumcenter_x = int((x1 + x2) / 2)
                        self.dum_height = int(y2 - y1)
                        self.get_logger().info(f"dum_height: {self.dum_height}")                      
                        if self.obstacle_avoiding:                        # 장애물 회피 중이라면 회피 작업 수행 
                            twist = Twist()
                            self.avoid_obstacle(twist, dumcenter_x, camera_center_x,frame,x1, y1, x2, y2, track_id, confidence, class_id)
                            return  
                        elif self.obstacle_avoiding==False and self.dum_height>=300:        # 장애물을 인식했는데 충돌할 위험이 있음에도 불구하고 장애물 회피 중이 아닐 때
                            self.get_logger().info("장애물이 탐지되었습니다!! 회피를 시작합니다...")
                            self.obstacle_avoiding = True
                            self.avoid_obstacle(twist, dumcenter_x, camera_center_x,frame,x1, y1, x2, y2, track_id, confidence, class_id)
                            self.start_obstacle_timer()  # 회피 상태 타이머 시작
                            break

                    if int(class_id) == 0 and int(track_id) == 1:
                        carcenter_x = int((x1 + x2) / 2)
                        carcenter_y = int((y1 + y2) / 2)
                        self.boundery = detection[3]              
                        angular_edit = carcenter_x - camera_center_x
                        self.get_logger().info(f"angular_edit: {angular_edit}")

                        self.carheight = int(y2 - y1)
                        self.get_logger().info(f'carheight: {self.carheight}')
                        if self.carheight is not None and not self.obstacle_avoiding:
                            self.adjust_speed(self.carheight,twist)
                            self.adjust_angular(angular_edit,twist)
                        
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                    if carcenter_x is not None and carcenter_y is not None:
                        cv2.circle(frame, (carcenter_x, carcenter_y), 5, (0, 0, 255), -1)

                    label_text = f'Conf: {confidence:.2f} Class: {int(class_id)}'
                    if track_id is not None:
                        label_text = f'Track_ID: {int(track_id)}, ' + label_text

                    cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        _, compressed_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        compressed_img_msg = CompressedImage()
        compressed_img_msg.header.stamp = self.get_clock().now().to_msg() 
        compressed_img_msg.format = "jpeg"
        compressed_img_msg.data = compressed_frame.tobytes()
        self.publisher_.publish(compressed_img_msg)
        self.cmd_vel_publisher.publish(twist)


    
    
    
    
    
    def avoid_obstacle(self, twist, dumcenter_x, camera_center_x, frame, x1, y1, x2, y2, track_id, confidence, class_id):
        """
        장애물 회피 로직: 장애물의 위치에 따라 우회 방향 설정
        """
        if dumcenter_x < camera_center_x: 
            twist.linear.x = 0.1
            twist.angular.z = -0.2
            self.get_logger().info("장애물 회피: 우회전중...")
           
        else: 
            twist.linear.x = 0.1
            twist.angular.z = 0.2
            self.get_logger().info("장애물 회피: 좌회전중...")

        self.cmd_vel_publisher.publish(twist)

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        label_text = f'Conf: {confidence:.2f} Class: {int(class_id)}'
        if track_id is not None:
            label_text = f'Track_ID: {int(track_id)}, ' + label_text

        cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        _, compressed_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        compressed_img_msg = CompressedImage()
        compressed_img_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_img_msg.format = "jpeg"
        compressed_img_msg.data = compressed_frame.tobytes()
        self.publisher_.publish(compressed_img_msg)
          
           
    

    def start_obstacle_timer(self):
        """
        회피 동작 후 장애물 무시 시간을 설정
        """
        if  self.obstacle_timer is not None and self.obstacle_timer.is_alive():
                return
        self.obstacle_timer = threading.Timer(3.0, self.reset_obstacle_avoidance) #별도 thread 에서 3초간 장애물 무시
        self.obstacle_timer.start()

    def reset_obstacle_avoidance(self):
        """
        회피 상태 플래그를 초기화하여 장애물 감지를 재개
        """
        self.obstacle_avoiding = False
        self.get_logger().info("장애물 회피 완료, 객체 정상 추적중...")
    
    def adjust_speed(self, height, twist):
        """
        객체의 높이(=객체와의 거리)에 따라 전진 속도를 세밀하게 조정.
        """
        if 470 < self.boundery < 485:
            speed = 0.0  # 정지
            self.get_logger().info("box가 닿을정도로 가까움 : 멈춤")
        elif height > 280:  # 매우 가까움
            speed = 0.0  # 정지
            self.get_logger().info("거리가 너무 가까움: 멈춤")
        elif 200 < height <= 280:  # 가까움
            speed = 0.05  # 매우 느린 속도
            self.get_logger().info("거리가 가까움: 매우천천히")
        elif 170 < height <= 200:  # 중간 거리
            speed = 0.1  # 느린 속도
            self.get_logger().info("거리가 조금 가까움: 천천히")
        elif 125 < height <= 170:  # 적정 거리
            speed = 0.15  # 적당한 속도
            self.get_logger().info("거리가 적당함: 표준속도")
        elif 100 < height <= 125:  # 먼 거리
            speed = 0.2  # 빠른 속도
            self.get_logger().info("거리가 멈: 빠르게")
        else:  # 매우 먼 거리
            speed = 0.3  # 최대 속도
            self.get_logger().info("거리가 매우 멈: 매우 빠르게")

        twist.linear.x = speed


    def adjust_angular(self, angular_edit, twist):
        """
        중심점 차이에 따라 좌/우회전을 세밀하게 조정.
        """
        if angular_edit > 200:  # 중심점이 오른쪽으로 크게 치우침
            twist.angular.z = -0.2  # 빠른 우회전
            self.get_logger().info("빠른 우회전")
        elif 150 < angular_edit <= 200:  # 중심점이 오른쪽으로 약간 치우침
            twist.angular.z = -0.1  # 중간 우회전
            self.get_logger().info("중간 우회전")
        elif 20 < angular_edit <= 150:  # 중심점이 오른쪽으로 조금 치우침
            twist.angular.z = -0.05  # 느린 우회전
            self.get_logger().info("느린 우회전")
        elif -20 <= angular_edit <= 20:  # 중심점이 거의 정중앙
            twist.angular.z = 0.0  # 회전 없음
            self.get_logger().info("회전 없음")
        elif -150 <= angular_edit < -20:  # 중심점이 왼쪽으로 조금 치우침
            twist.angular.z = 0.05  # 느린 좌회전
            self.get_logger().info("느린 좌회전")
        elif -200 <= angular_edit < -150:  # 중심점이 왼쪽으로 약간 치우침
            twist.angular.z = 0.1  # 중간 좌회전
            self.get_logger().info("중간 좌회전")
        else:  # 중심점이 왼쪽으로 크게 치우침
            twist.angular.z = 0.2  # 빠른 좌회전
            self.get_logger().info("빠른 좌회전")
    
    def destroy_node(self):
        super().destroy_node()
        self.cap.release()   



def main(args=None):
    rclpy.init(args=args)
    node = YOLOTrackingPublisher()
    
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    thread.join()
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






