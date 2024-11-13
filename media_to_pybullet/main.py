import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import mediapipe as mp
import numpy as np
from cv_bridge import CvBridge
import roslibpy

import os
os.environ['ROS_DOMAIN_ID'] = '1'

class MediapipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_node')
        
        # 從參數伺服器獲取 ROS bridge IP 地址，若未設置則默認為 127.0.0.1
        rosbridge_ip = self.declare_parameter('rosbridge_ip', '127.0.0.1').value  
        
        # 初始化 rosbridge 連接
        self.ros_client = roslibpy.Ros(host=rosbridge_ip, port=9090)
        self.ros_client.run()
        
        # 初始化訂閱者
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # CvBridge 用於轉換 ROS 影像訊息
        self.bridge = CvBridge()
        
        # MediaPipe 初始化
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, 
                                         min_detection_confidence=0.5)
        
        # 初始化 Pose 模組
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        # 設定要發布的話題
        self.coordinate_pub = roslibpy.Topic(self.ros_client, '/mediapipe_data', 'geometry_msgs/Point')

        # 深度圖像變數
        self.depth_image = None
        self.center_depth = None

        self.get_logger().info(f"MediapipeNode initialized and connected to ROS bridge at IP: {rosbridge_ip}")

    def draw_axis(self, image, center_x, center_y, length=60, step=10):
        """在畫面上繪製 Y 和 Z 軸的刻度線，使得範圍為 -60 ~ 60 並等比例分佈"""
        scale_x = image.shape[1] // (2 * length)
        scale_y = image.shape[0] // (2 * length)

        for i in range(-length, length + 1, step):
            # Z 軸刻度（水平）
            x_pos = center_x + i * scale_x
            cv2.line(image, (x_pos, center_y - 5), (x_pos, center_y + 5), (255, 0, 0), 1)
            cv2.putText(image, str(i), (x_pos, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
            
            # Y 軸刻度（垂直）
            y_pos = center_y + i * scale_y
            cv2.line(image, (center_x - 5, y_pos), (center_x + 5, y_pos), (0, 0, 255), 1)
            cv2.putText(image, str(-i), (center_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
        y_pos = center_y + (40) * scale_y
        cv2.circle(image, (center_x, int(y_pos)), 5, (0, 0, 255), -1)  # 畫紅點

    def color_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        # 設定畫面的中心點（十字架交點）
        center_x_px = color_image.shape[1] // 2
        center_y_px = color_image.shape[0] // 2

        # 畫上十字架
        cv2.drawMarker(color_image, (center_x_px, center_y_px), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # 畫出 Y 和 Z 軸的刻度線
        self.draw_axis(color_image, center_x_px, center_y_px)

        # 更新中心點的深度
        if self.depth_image is not None:
            self.center_depth = self.depth_image[center_y_px, center_x_px] / 1000.0

            # 使用 Hands 模組檢測手掌座標
            hand_results = self.hands.process(rgb_image)
            if hand_results.multi_hand_landmarks and hand_results.multi_handedness:
                for hand_landmarks, handedness in zip(hand_results.multi_hand_landmarks, hand_results.multi_handedness):
                    if handedness.classification[0].label == "Left":
                        fingertip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                        
                        # 將手掌像素位置轉換為深度影像中的位置
                        fingertip_x = int(fingertip.x * self.depth_image.shape[1])
                        fingertip_y = int(fingertip.y * self.depth_image.shape[0])

                        # 確保手掌位置在圖像邊界內
                        if 0 <= fingertip_x < self.depth_image.shape[1] and 0 <= fingertip_y < self.depth_image.shape[0]:
                            # 使用深度影像獲取手掌的深度，單位為毫米，轉換為米
                            fingertip_depth = self.depth_image[fingertip_y, fingertip_x] / 1000.0
                            
                            # 計算手掌相對於中心點的刻度座標
                            relative_y = (center_y_px - fingertip_y) * 60 / (color_image.shape[0] // 2)
                            relative_z = (fingertip_x - center_x_px) * 60 / (color_image.shape[1] // 2)
                            relative_x = fingertip_depth - self.center_depth

                            

                            tmp = relative_y
                            relative_y = -relative_z /100.0
                            relative_z = tmp / 100.0

                            if relative_z < 0.1:
                                relative_z = 0.1
                            # 構建消息並通過 rosbridge 發布
                            point_message = roslibpy.Message({
                                'x': relative_x,
                                'y': relative_y,
                                'z': relative_z
                            })
                            self.coordinate_pub.publish(point_message)
                            self.get_logger().info(f"發布座標: x={relative_x:.2f}, y={relative_y:.2f}, z={relative_z:.2f}")

                            # 在圖像上顯示手腕的像素座標和刻度座標
                            cv2.putText(color_image, f"Hand (y, z): ({relative_y:.2f}, {relative_z:.2f})", 
                                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                            self.mp_drawing.draw_landmarks(color_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        
        cv2.imshow("Color Image with Mediapipe", color_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        # 將 ROS 深度影像轉為 OpenCV 格式
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        
    def destroy_node(self):
        # 關閉 rosbridge 連接
        self.coordinate_pub.unadvertise()
        self.ros_client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MediapipeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MediapipeNode.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
