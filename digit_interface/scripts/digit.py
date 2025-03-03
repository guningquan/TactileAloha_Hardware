#!/home/ubuntu20/miniforge3/envs/aloha/bin/python
# first of all, you need to source the virtualenv. Then source the devel setup.bash file.

# ROS IMPORTS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import cv2

# DIGIT LIBRARY
from digit_interface import Digit

'''
    setup_digit function: reads the sensor id from the launch file and initializes the sensor.
    Arguments:
        - None
    Returns: object class of the sensor library.
'''

def setup_digit_multi():

    # get sensor id from launch file
    id = rospy.get_param("/sensor_id")
    id55, id45 = id.split("_")[0], id.split("_")[1]

    # initialize digit sensor with the usb serial
    d55 = Digit(id55)
    d45 = Digit(id45)
    d55.connect()
    d45.connect()

    return d55, d45, id55, id45


def setup_digit():

    # get sensor id from launch file
    id = rospy.get_param("/sensor_id")
    id55 = id

    # initialize digit sensor with the usb serial
    d55 = Digit(id55)

    d55.connect()


    return d55, id55


'''
    main function: initialize the ros node and the publisher. Runs an infinite loop
        where reads images from the sensor and publish them in the topic.
    Arguments:
        - None
    Returns: None
'''
def main():

    # initialize digit sensor
    d55, id55 = setup_digit()


    d55.set_resolution(Digit.STREAMS["VGA"])  # VGA 分辨率：640×480
    d55.set_fps(Digit.STREAMS["VGA"]["fps"]["30fps"])

    # d55.set_resolution(Digit.STREAMS["QVGA"])  # VGA 分辨率：640×480
    # d55.set_fps(Digit.STREAMS["QVGA"]["fps"]["60fps"])


    # initialize ros node
    rospy.init_node(f"digit_ros_node", anonymous=True)
    # create publisher with the topic and the type of the msg
    pub55 = rospy.Publisher(f"gel/camera/image_color", Image, queue_size=10) # 10 -> 5

    # cvbridge to transform the images from np to ros type
    cvbridge = CvBridge()

    # infinite loop
    while True:

        # comment if you want
        # rospy.loginfo("Publishing Digit images!")

        # get image from the sensor
        digit55_img = d55.get_frame()

        # cv2.imshow(f"Digit View {id55}", digit55_img)
        # if cv2.waitKey(1) == 27:
        #     cv2.destroyAllWindows()
        #     break

        digit55_img = cv2.resize(digit55_img, (480, 640))
        digit55_img = cv2.rotate(digit55_img, cv2.ROTATE_90_CLOCKWISE)

        # cv2.imshow("digit55", digit55_img)
        # cv2.waitKey(1)

        # create the ros msg for the images and fill with all the info
        image_msg55 = Image()
        image_msg55.header.stamp = rospy.Time.from_sec(time.time())
        image_msg55.header.frame_id = "digit_camera"
        image_msg55.height = (digit55_img.shape)[0]
        image_msg55.width = (digit55_img.shape)[1]
        image_msg55.step = digit55_img.strides[0]

        # bgr8
        # image_msg55.data = digit55_img.flatten().tolist()
        # image_msg55.encoding = "bgr8"

        # rgb8
        digit55_img_rgb = cv2.cvtColor(digit55_img, cv2.COLOR_BGR2RGB)
        image_msg55.data = digit55_img_rgb.flatten().tolist()
        image_msg55.encoding = "rgb8"

        # transform to ros type and publish img
        #pub.publish(cvbridge.cv2_to_imgmsg(digit_img))
        pub55.publish(image_msg55)
        # 100 hz rate of publication
        rospy.Rate(100) # modify


        # rospy.Rate(60)  # modify 60 -> 30  need to be consider



if __name__ == '__main__':
    main()
