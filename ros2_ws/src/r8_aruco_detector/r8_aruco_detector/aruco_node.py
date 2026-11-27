
#!/usr/bin/env python3
import json

import cv2
import rclpy

from rclpy.node import Node
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

        # Список топиков камер.
        # Пока указан известный нам топик одной камеры.
        # После ответа R2 добавим остальные четыре.
        self.declare_parameter(
            'camera_topics',
            ['/camera/image_raw']
        )

        self.camera_topics = self.get_parameter(
            'camera_topics'
        ).value

        # Отдельный фильтр для каждой камеры
        self.marker_filters = {
            topic: MarkerFilter(required_frames=3)
            for topic in self.camera_topics
        }

        # Общий ROS 2 publisher
        self.publisher = self.create_publisher(
            String,
            '/aruco/detections',
            10
        )

        # Подписки на изображения всех камер
        self.subscriptions = []

        for topic in self.camera_topics:
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, camera_topic=topic:
                    self.image_callback(msg, camera_topic),
                10
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
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            corners, ids, rejected = cv2.aruco.detectMarkers(
                image,
                self.aruco_dict,
                parameters=self.parameters
            )

            if ids is None:
                detected_ids = []
            else:
                detected_ids = ids.flatten().tolist()

            # Используем фильтр именно этой камеры
            confirmed_ids = self.marker_filters[
                camera_topic
            ].update(detected_ids)

            # Формируем JSON
            output = String()
            output.data = json.dumps({
                'ids': confirmed_ids
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
