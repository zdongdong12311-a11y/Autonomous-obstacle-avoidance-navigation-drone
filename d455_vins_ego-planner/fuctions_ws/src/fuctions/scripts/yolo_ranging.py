#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from ultralytics import YOLO

class YoloRangingNode:
    def __init__(self):
        rospy.init_node('yolo_ranging_node')
        self.bridge = CvBridge()
        
        rospy.loginfo("🧠 正在加载 YOLOv8 模型...")
        self.model = YOLO("yolov8n.pt") 

        # 订阅彩色图和“对齐后的深度图”
        color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

        # 毫秒级时间同步器，确保彩色和深度画面绝对匹配
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.result_pub = rospy.Publisher('/yolo/result_image', Image, queue_size=1)
        rospy.loginfo("🎯 YOLOv8 视觉测距已启动！")

    def sync_callback(self, color_msg, depth_msg):
        cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        # RealSense 深度图格式为 16位无符号整型，单位毫米
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1") 

        # 推理：只看人 (classes=[0])，置信度 0.5
        results = self.model(cv_color, conf=0.5, classes=[0], verbose=False)
        
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # 防止坐标越界
                if 0 <= cx < cv_depth.shape[1] and 0 <= cy < cv_depth.shape[0]:
                    distance_mm = cv_depth[cy, cx]
                    distance_m = distance_mm / 1000.0
                    
                    if distance_m > 0:
                        text = f"Person: {distance_m:.2f}m"
                    else:
                        text = "Distance Unknown"
                else:
                    text = "Out of bounds"

                # 绘制绿色框和红色准星
                cv2.rectangle(cv_color, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(cv_color, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(cv_color, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        result_img_msg = self.bridge.cv2_to_imgmsg(cv_color, "bgr8")
        self.result_pub.publish(result_img_msg)

if __name__ == '__main__':
    try:
        YoloRangingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass