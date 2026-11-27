
#!/usr/bin/env python3
import json
import time

import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from r8_aruco_detector.marker_filter import MarkerFilter


class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        # Словарь ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters_create()

        # Топики пяти камер
        default_camera_topics = [
            '/camera/image_raw',
            '/camera_chin/image_raw',
            '/camera_left/image_raw',
            '/camera_right/image_raw',
            '/camera_rearDown/image_raw'
        ]

        self.declare_parameter(
            'camera_topics',
            default_camera_topics
        )

        self.camera_topics = self.get_parameter(
            'camera_topics'
        ).value

        # Отдельный фильтр для каждой камеры
        self.marker_filters = {
            topic: MarkerFilter(required_frames=3)
            for topic in self.camera_topics
        }

        # Время последнего подтверждённого обнаружения каждого ID
        self.marker_last_seen = {}

        # Окно актуальности обнаруженных маркеров: 0,75 секунды
        self.marker_lifetime = 0.75

        # QoS для пяти камер
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Общий publisher
        self.publisher = self.create_publisher(
            String,
            '/aruco/detections',
            10
        )

        # Подписки на камеры
        self.subscriptions = []

        for topic in self.camera_topics:
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, camera_topic=topic:
                    self.image_callback(msg, camera_topic),
                camera_qos
            )

            self.subscriptions.append(subscription)

            self.get_logger().info(
                f'Подписка на камеру: {topic}'
            )

        self.get_logger().info(
            f'ArUco-нода запущена. Камер: {len(self.camera_topics)}'
        )

    def image_callback(self, msg, camera_topic):

        try:
            # Конвертация изображения в BGR8
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Обнаружение ArUco
            corners, ids, rejected = cv2.aruco.detectMarkers(
                image,
                self.aruco_dict,
                parameters=self.parameters
            )

            if ids is None:
                detected_ids = []
            else:
                detected_ids = ids.flatten().tolist()

            # Фильтрация по трём последовательным кадрам
            confirmed_ids = self.marker_filters[
                camera_topic
            ].update(detected_ids)

            # Обновление времени обнаружения ID
            current_time = time.monotonic()

            for marker_id in confirmed_ids:
                self.marker_last_seen[int(marker_id)] = current_time

            # Удаление устаревших ID
            self.marker_last_seen = {
                marker_id: timestamp
                for marker_id, timestamp
                in self.marker_last_seen.items()
                if current_time - timestamp <= self.marker_lifetime
            }

            # Объединение ID всех камер без дубликатов
            unique_ids = sorted(set(self.marker_last_seen.keys()))

            # Публикация JSON
            output = String()
            output.data = json.dumps({
                'ids': unique_ids
            })

            self.publisher.publish(output)

        except Exception as error:
            self.get_logger().error(
                f'Ошибка обработки камеры {camera_topic}: {error}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
