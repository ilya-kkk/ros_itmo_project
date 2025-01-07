#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import math
import tf


class MapToImagePublisher:
    def __init__(self):
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.image_publisher = rospy.Publisher("/diff_drive_robot/map_img", Image, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.listener = tf.TransformListener()


        self.bridge = CvBridge()

        self.resolution = 0.05
        self.origin = Point()
        self.point = [0.0, 0.0]
    

    def map_callback(self, msg):
        data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
    

        input_image = self.convert_map_to_image(data)
        output_image = self.transform(input_image)
        
        # theta = 45
        theta = self.calc_angle(self.point)
        self.publish_goal(self.point, theta)
        self.publish_image(output_image)


    def calc_angle(self, image_point):
        # Преобразуем точку на изображении в координаты на карте
        map_point_x, map_point_y = self.image_to_map_coordinates(image_point[0], image_point[1])
        # Инициализируем слушатель tf
        listener = tf.TransformListener()
        try:
            # Ждём доступности трансформации
            listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

            # Получаем трансформацию от map к base_link
            (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            robot_x, robot_y, _ = trans  # Извлекаем координаты робота
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("Failed to get transform from map to base_link.")
            return None
        # Вычисляем разницу в координатах
        delta_x = map_point_x - robot_x
        delta_y = map_point_y - robot_y
        # Вычисляем угол в радианах
        angle_rad = math.atan2(delta_y, delta_x)
        # Конвертируем угол в градусы
        angle_deg = math.degrees(angle_rad)

        return angle_deg


    def convert_map_to_image(self, data):
        # Создаем изображение на основе данных карты
        image = np.zeros_like(data, dtype=np.uint8)
        
        # Заполняем картинку в зависимости от значений на карте
        image[data == 100] = 0    # Занятая клетка
        image[data == 0] = 255    # Свободная клетка
        image[data == -1] = 127   # Неизвестная клетка

        # Переворачиваем изображение по вертикали (отражение сверху вниз)
        image = cv2.flip(image, 0)

        return image


    def find_boundaries(self, image):
        # создаем маску по черному цвету (стена) расширяем ее границы и накладываем 
        #  обратно на изобразжение чтобы следующий аглоритм не захватывал лишнего
        black_mask = cv2.inRange(image, 0, 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        black_mask = cv2.dilate(black_mask, kernel, iterations=1)
        image[black_mask == 255] = [0]
        # создаем маски по свободным 255 и неразведанным 127 областям 
        # расширяем их, затем ищем пересечения - это и есть искомые фронтиры
        gray_mask = cv2.inRange(image, 126, 128)
        white_mask = cv2.inRange(image, 253, 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        gray_mask = cv2.dilate(gray_mask, kernel, iterations=1)
        white_mask = cv2.dilate(white_mask, kernel, iterations=1)
        border = cv2.bitwise_and(gray_mask, white_mask)
        return border


    def find_centroids(self, binary_image):
        """
        Находит центры масс областей со значением 255 на бинаризованном изображении.

        :param binary_image: np.array, бинарное изображение (0 и 255)
        :return: list of tuples, массив координат центров (x, y)
        """
        # Проверяем входное изображение
        if len(binary_image.shape) != 2:
            raise ValueError("Изображение должно быть одноканальным (grayscale).")
        
        # Находим контуры областей со значением 255
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Список для хранения координат центров
        centroids = []
        
        for contour in contours:
            # Вычисляем моменты области
            M = cv2.moments(contour)
            if M["m00"] != 0:  # Проверяем, что площадь не равна нулю
                # Вычисляем координаты центра масс
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))
        
        return centroids


    def create_mask_with_crosses(self, binary_image, point, size=5):
        mask = np.zeros_like(binary_image, dtype=np.uint8)

        x, y = point[0], point[1]
        cv2.line(mask, (x - size, y - size), (x + size, y + size), 255, 1)
        cv2.line(mask, (x + size, y - size), (x - size, y + size), 255, 1)
        # cv2.putText(mask, f"({x},{y})", (x + size + 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, 255, 1)

        return mask


    def transform(self, input_image):   ##########################################
        color_image = cv2.cvtColor(input_image, cv2.COLOR_GRAY2BGR)  
        boundaries = self.find_boundaries(input_image)
        color_image[boundaries == 255] = [0, 255, 0]  

        points = self.find_centroids(boundaries)
        for point in points:
            circles = self.create_mask_with_crosses(boundaries, point, 2)
            color_image[circles == 255] = [255, 0, 0]

        self.point = points[0]          ##########################################

        circles = self.create_mask_with_crosses(boundaries, self.point, 2)
        color_image[circles == 255] = [0, 0, 255] 

        output_image = self.resize_image(self.crop_image(color_image))
        return output_image


    def image_to_map_coordinates(self, px, py):
        # Преобразуем координаты пикселя в координаты на карте
        map_x = px * self.resolution + self.origin.x
        map_y = py * self.resolution + self.origin.y

        return map_x, map_y


    def angle_to_quaternion(self, angle_deg):
        # Преобразуем угол из градусов в радианы
        angle_rad = math.radians(angle_deg)
        
        # Вычисляем компоненты кватерниона
        qw = math.cos(angle_rad / 2)
        qz = math.sin(angle_rad / 2)
        
        # Создаем объект Quaternion
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = qz
        quaternion.w = qw
        
        return quaternion


    def publish_goal(self, point, angle): ########################
        x, y = self.image_to_map_coordinates(point[0], point[1])
        y = -y
        rospy.loginfo("Points: (%s, %s)\n\r", x, y)
        # x, y = 5, -1.1 ###########################################

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  # Система координат карты
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position = Point(x, y, 0)
        goal_msg.pose.orientation = self.angle_to_quaternion(angle)

        self.goal_publisher.publish(goal_msg)


    def crop_image(self, image):
        # Проверяем, есть ли вообще ненулевые элементы в изображении
        if not np.any(image):
            # Если все пиксели равны 0, возвращаем исходное изображение
            return image

        # Находим строки и столбцы, содержащие ненулевые пиксели
        rows = np.any(image == 255, axis=1)
        cols = np.any(image == 255, axis=0)

        # Находим минимальные и максимальные индексы с ненулевыми пикселями
        row_indices = np.where(rows)[0]
        col_indices = np.where(cols)[0]

        row_min, row_max = row_indices[0], row_indices[-1]
        col_min, col_max = col_indices[0], col_indices[-1]

        # Добавляем отступы
        offset = 20  # Отступ в пикселях
        row_min = max(0, row_min - offset)
        row_max = min(image.shape[0], row_max + offset)
        col_min = max(0, col_min - offset)
        col_max = min(image.shape[1], col_max + offset)

        # Вычисляем размеры области обрезки
        cropped_height = row_max - row_min
        cropped_width = col_max - col_min

        # Вычисляем разницу, чтобы сделать области квадратными
        if cropped_height > cropped_width:
            diff = cropped_height - cropped_width
            col_min = max(0, col_min - diff // 2)
            col_max = min(image.shape[1], col_max + diff // 2)
        elif cropped_width > cropped_height:
            diff = cropped_width - cropped_height
            row_min = max(0, row_min - diff // 2)
            row_max = min(image.shape[0], row_max + diff // 2)

        # Обрезаем изображение
        cropped_image = image[row_min:row_max, col_min:col_max]

        return cropped_image


    def resize_image(self, image, target_size=(2000, 2000)): 
        resized_image = cv2.resize(image, target_size, interpolation=cv2.INTER_NEAREST)
        return resized_image


    def publish_image(self, image): 
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8") 
        ros_image.header = Header()
        ros_image.header.stamp = rospy.Time.now()

        self.image_publisher.publish(ros_image)


if __name__ == '__main__':
    rospy.init_node('map_to_image_node', anonymous=True)
    map_to_image_publisher = MapToImagePublisher()
    rospy.spin()
