import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


image = None
cvbridge = CvBridge()


def callback(msg):
    global image
    image = cvbridge.imgmsg_to_cv2(msg, "bgr8")


def main():

    rospy.init_node("digit_ros_node_read", anonymous=True)
    # pub = rospy.Subscriber("digit/camera/image_color", Image, callback)
    pub = rospy.Subscriber("gel/camera/image_color", Image, callback)

    print("Reading from digit")

    while True:

        if image is not None:
            cv2.imshow("window", image)
            cv2.waitKey(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



if __name__ == '__main__':
    main()