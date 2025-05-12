from typing import Any, Union

from rosbags.typesys.types import (
    sensor_msgs__msg__Imu,
    sensor_msgs__msg__Image,
    sensor_msgs__msg__CompressedImage,
    sensor_msgs__msg__PointCloud2
)
from rosbag_converter.tools.image import Image
from rosbag_converter.tools.comp_image import CompressedImage
from rosbag_converter.tools.imu import Imu
from rosbag_converter.tools.point_cloud2 import PointCloudXf

conversion_dict = {
    "sensor_msgs/msg/PointCloud2": PointCloudXf.from_ros,
    "sensor_msgs/msg/Image": Image.from_ros,
    "sensor_msgs/msg/CompressedImage": CompressedImage.from_ros,
    "sensor_msgs/msg/Imu": Imu.from_ros
}


def convert_msg_to_datum(
        msg: Any,
        msg_type: str) -> Union[PointCloudXf, Image, CompressedImage, Imu]:
    
    # for others msg type, just skip
    if msg_type in conversion_dict:
        return conversion_dict[msg_type](msg)
    else:
        return None
