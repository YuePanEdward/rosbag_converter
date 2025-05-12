try:
    from rosbags.typesys.types import sensor_msgs__msg__CompressedImage
except ImportError as e:
    raise ImportError('rosbags library not installed, run "pip install -U rosbags"') from e

try:
    from rosbags.image import compressed_image_to_cvimage
except ImportError as e:
    raise ImportError('rosbags-image library not installed, run "pip install -U rosbags-image"') from e

# import rospy
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import CompressedImage


class CompressedImage:
    def __init__(self):
        self.image = None
        self.encoding = None
        ...

    @staticmethod
    def from_ros(msg: sensor_msgs__msg__CompressedImage) -> "CompressedImage":
        im = CompressedImage()
        im.image = compressed_image_to_cvimage(msg)
        
        # im.encoding = msg.encoding

        return im