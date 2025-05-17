import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import numpy as np

class ImageRecorder:
    def __init__(self, init_node=True, is_debug=False):
        from collections import deque
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self.is_debug = is_debug
        self.bridge = CvBridge()

        # Add "gel" to our list of cameras
        self.camera_names = [
            'cam_high',
            'cam_low',
            'cam_left_wrist',
            'cam_right_wrist',
            'gel',   # @gnq important !
        ]

        if init_node:
            rospy.init_node('image_recorder', anonymous=True)

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)

            # Select callback based on camera name
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
                topic_name = "/usb_cam_high/image_raw"

            elif cam_name == 'cam_low':
                callback_func = self.image_cb_cam_low
                topic_name = "/usb_cam_low/image_raw"

            elif cam_name == 'cam_left_wrist':
                callback_func = self.image_cb_cam_left_wrist
                topic_name = "/usb_cam_left_wrist/image_raw"

            elif cam_name == 'cam_right_wrist':
                callback_func = self.image_cb_cam_right_wrist
                topic_name = "/usb_cam_right_wrist/image_raw"

            elif cam_name == 'gel':
                callback_func = self.image_cb_gel
                # Make sure this matches the topic used in digit.py
                topic_name = "/gel/camera/image_color"

            else:
                # If you add more names later, handle them here
                raise NotImplementedError

            # Subscribe to the correct topic with the selected callback
            rospy.Subscriber(topic_name, Image, callback_func)

            # If debug, store the last 50 timestamps for frequency diagnostics
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))

        time.sleep(0.5)

    def image_cb(self, cam_name, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        # 转换颜色通道，从 BGR 转为 RGB
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        setattr(self, f'{cam_name}_image', cv_image)
        setattr(self, f'{cam_name}_secs', data.header.stamp.secs)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nsecs)

        if self.is_debug:
            ts_list = getattr(self, f'{cam_name}_timestamps')
            ts_list.append(data.header.stamp.secs + data.header.stamp.nsecs * 1e-9)

    def image_cb_cam_high(self, data):
        return self.image_cb('cam_high', data)

    def image_cb_cam_low(self, data):
        return self.image_cb('cam_low', data)

    def image_cb_cam_left_wrist(self, data):
        return self.image_cb('cam_left_wrist', data)

    def image_cb_cam_right_wrist(self, data):
        return self.image_cb('cam_right_wrist', data)

    def image_cb_gel(self, data):
        return self.image_cb('gel', data)

    def get_images(self):
        image_dict = {}
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        def dt_helper(list_of_timestamps):
            arr = np.array(list_of_timestamps)
            diff = arr[1:] - arr[:-1]
            return np.mean(diff)

        for cam_name in self.camera_names:
            ts_list = getattr(self, f'{cam_name}_timestamps', [])
            if len(ts_list) > 1:
                image_freq = 1 / dt_helper(ts_list)
                print(f'{cam_name} frequency: {image_freq:.2f} Hz')
            else:
                print(f'{cam_name} no timestamps recorded yet.')
        print()

class Recorder:
    def __init__(self, side, init_node=True, is_debug=False):
        from collections import deque
        import rospy
        from sensor_msgs.msg import JointState
        from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand

        self.secs = None
        self.nsecs = None
        self.qpos = None
        self.effort = None
        self.arm_command = None
        self.gripper_command = None
        self.is_debug = is_debug

        if init_node:
            rospy.init_node('recorder', anonymous=True)
        rospy.Subscriber(f"/puppet_{side}/joint_states", JointState, self.puppet_state_cb)
        rospy.Subscriber(f"/puppet_{side}/commands/joint_group", JointGroupCommand, self.puppet_arm_commands_cb)
        rospy.Subscriber(f"/puppet_{side}/commands/joint_single", JointSingleCommand, self.puppet_gripper_commands_cb)
        if self.is_debug:
            self.joint_timestamps = deque(maxlen=50)
            self.arm_command_timestamps = deque(maxlen=50)
            self.gripper_command_timestamps = deque(maxlen=50)
        time.sleep(0.1)

    def puppet_state_cb(self, data):
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data
        if self.is_debug:
            self.joint_timestamps.append(time.time())

    def puppet_arm_commands_cb(self, data):
        self.arm_command = data.cmd
        if self.is_debug:
            self.arm_command_timestamps.append(time.time())

    def puppet_gripper_commands_cb(self, data):
        self.gripper_command = data.cmd
        if self.is_debug:
            self.gripper_command_timestamps.append(time.time())

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)

        joint_freq = 1 / dt_helper(self.joint_timestamps)
        arm_command_freq = 1 / dt_helper(self.arm_command_timestamps)
        gripper_command_freq = 1 / dt_helper(self.gripper_command_timestamps)

        print(f'{joint_freq=:.2f}\n{arm_command_freq=:.2f}\n{gripper_command_freq=:.2f}\n')

def main():
    # Initialize the image recorder
    recorder = ImageRecorder(init_node=True)

    # Main loop
    while not rospy.is_shutdown():
        # Get images from all cameras
        images = recorder.get_images()

        # Set the target size for resizing and create a blank (black) image
        target_size = (640, 480)  # Resize all images to 640x480
        blank_image_size = (480, 640, 3)  # Dimensions for the blank image
        blank_image = np.zeros(blank_image_size, dtype=np.uint8)  # Create a black image


        # Define the text content and style
        text = "NO SIGNAL"  # The text to display
        font = cv2.FONT_HERSHEY_SIMPLEX  # Font type
        font_scale = 2  # Font size
        font_color = (255, 255, 255)  # White color for the text (BGR format)
        thickness = 3  # Thickness of the text

        # Get the size of the text to center it on the image
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        # Calculate the starting coordinates to center the text
        text_x = (blank_image.shape[1] - text_width) // 2  # Center horizontally
        text_y = (blank_image.shape[0] + text_height) // 2  # Center vertically

        # Add text to the blank image
        cv2.putText(blank_image, text, (text_x, text_y), font, font_scale, font_color, thickness)


        # Check if all images are received; if not, fill with the blank image
        filled_images = {}
        for name in ['cam_high', 'cam_low', 'gel', 'cam_left_wrist', 'cam_right_wrist']:  # could expend gel02
            if images.get(name) is not None:  # If the image exists
                filled_images[name] = cv2.resize(images[name], target_size)  # Resize the image
            else:  # If the image is missing
                filled_images[name] = blank_image  # Use the blank (black) image

        # Combine images into a 2x2 grid and add the "gel" image
        top_row = np.hstack((filled_images['cam_high'], filled_images['cam_low'], filled_images['gel']))
        bottom_row = np.hstack((filled_images['cam_left_wrist'], filled_images['cam_right_wrist'], blank_image))
        grid_3x2 = np.vstack((top_row, bottom_row))
        final_output = grid_3x2

        # Display the combined image
        cv2.namedWindow("Multi-camera View", cv2.WINDOW_NORMAL)  # Create a resizable window
        cv2.setWindowProperty("Multi-camera View", cv2.WND_PROP_TOPMOST, 1)  # Set the window to always stay on top
        cv2.imshow("Multi-camera View", final_output)  # Show the combined image

        # Exit when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()