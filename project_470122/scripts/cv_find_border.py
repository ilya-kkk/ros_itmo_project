#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import cv2
import numpy as np
from cv_bridge import CvBridge

class sensor_datahandler:

    def __init__(self):
        # Подписка на исходное изображение
        sub_topic_name = "/diff_drive_robot/camera1/image_raw"
        self.camera_subscriber = rospy.Subscriber(sub_topic_name, Image, self.camera_cb)
        
        # Паблишеры для обработки изображений
        self.image_publisher_processed = rospy.Publisher("/diff_drive_robot/camera1/image_processed", Image, queue_size=10)
        self.image_publisher_binarized = rospy.Publisher("/diff_drive_robot/camera1/image_binarized", Image, queue_size=10)

        # Конвертер между ROS и OpenCV
        self.bridge = CvBridge()
        self.curent_image = None

    def camera_cb(self, data):
        # Преобразуем изображение из формата ROS в OpenCV
        frame = self.bridge.imgmsg_to_cv2(data)
        # print(frame.shape)

        # Пример обработки изображения (например, детектирование границ)
        edge_frame = cv2.Canny(frame, 100, 200)
        
        # Публикуем обработанное изображение
        self.publish_image(self.image_publisher_processed, edge_frame)
        
        # Создание биаризованного изображения на основе фильтрации по цвету
        binarized_frame = self.create_binarized_image(frame)
        
        # Публикуем биаризованное изображение
        self.publish_image(self.image_publisher_binarized, binarized_frame)

        # Отображаем обработанные изображения
        # cv2.imshow("edge_frame", edge_frame)
        # cv2.imshow("binarized_frame", binarized_frame)
        cv2.waitKey(1)

    def create_binarized_image(self, frame):
        """
        Создание биаризованного изображения, где белая область соответствует
        пикселям с цветом близким к (r=55, g=55, b=55) с разбросом в 5.
        """
        lower_bound = np.array([50, 50, 50], dtype=np.uint8)  # Нижний предел
        upper_bound = np.array([60, 60, 60], dtype=np.uint8)  # Верхний предел

        # Преобразуем изображение в маску, где пиксели в указанном диапазоне цвета равны 255 (белый), остальное - 0 (черный)
        mask = cv2.inRange(frame, lower_bound, upper_bound)

        # Применяем маску к изображению (результат - черно-белое изображение)
        binarized_image = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Конвертируем в оттенки серого для ROS-сообщения
        gray_binarized_image = cv2.cvtColor(binarized_image, cv2.COLOR_BGR2GRAY)

        return gray_binarized_image

    def publish_image(self, publisher, image):
        """
        Публикуем изображение в заданный топик.
        """
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        ros_image.header = Header()
        ros_image.header.stamp = rospy.Time.now()
        
        #publisher.publish(ros_image)

if __name__ == '__main__':
    rospy.init_node("camera_data_processing")
    sensor_datahandler()
    rospy.spin()
