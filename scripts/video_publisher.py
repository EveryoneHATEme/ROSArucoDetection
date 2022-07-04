import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge


class ImagePublisher:
    def __init__(self, source: int, publisher_name: str) -> None:
        rospy.init_node('image_publisher')
        self.publisher = rospy.Publisher(publisher_name, Image, queue_size=10)
        self.camera = cv.VideoCapture(source)
        self.bridge = CvBridge()
    
    def run_loop(self, rate_hz: int) -> None:
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown() and self.camera.isOpened():
            ret, image = self.camera.read()
            if not ret:
                continue

            message = self.bridge.cv2_to_imgmsg(image)
            self.publisher.publish(message)

            rate.sleep()


if __name__ == '__main__':
    try:
        publisher = ImagePublisher(0, 'image_publisher')
        publisher.run_loop(30)
    except rospy.ROSInterruptException:
        pass
