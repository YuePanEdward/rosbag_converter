from pathlib import Path
import json
from pathlib import Path
from typing import Union
import numpy as np
import cv2
from datetime import datetime

import open3d as o3d

from rosbag_converter.tools import PointCloudXf, Image, Imu, CompressedImage

IMU_CSV_HEADER = ("acc_x,acc_y,acc_z,"
                  "gyr_x,gyr_y,gyr_z,"
                  "quat_w,quat_x,quat_y,quat_z,"
                  "cov_acc_xx,cov_acc_xy,cov_acc_xz,"
                  "cov_acc_yx,cov_acc_yy,cov_acc_zz,"
                  "cov_acc_zx,cov_acc_zy,cov_acc_zz,"
                  "cov_gyr_xx,cov_gyr_xy,cov_gyr_xz,"
                  "cov_gyr_xx,cov_gyr_xy,cov_gyr_xz,"
                  "cov_gyr_yx,cov_gyr_yy,cov_gyr_zz,"
                  "cov_rot_zx,cov_rot_zy,cov_rot_zz,"
                  "cov_rot_yx,cov_rot_yy,cov_rot_zz,"
                  "cov_rot_zx,cov_rot_zy,cov_rot_zz\n")

# TODO: also make it work with the compressed_img

class KittiTopicHandler:
    def __init__(self, data_dir: Path, topic: str, format_fn=None):
        self.metadata_f = data_dir / ".metadata.json"
        self.timestamps_f = data_dir / "timestamps.txt"
        self.data_f = data_dir / "data"

        self.metadata = {}
        self.timestamps = []

        self.format_fn = format_fn

        self.imu_dest = None

        self.save_fn = {
            PointCloudXf.__name__: self._save_cloud,
            Image.__name__: self._save_image,
            CompressedImage.__name__: self._save_image,
            Imu.__name__: self._save_imu

        }

        if self.metadata_f.exists():
            # Try loading the metadata file to get info
            with self.metadata_f.open("r") as f:
                self.metadata = json.load(f)

            if topic:
                assert topic == self.metadata["topic"]
        else:
            self.data_f.mkdir(exist_ok=True, parents=True)
            self.metadata["topic"] = topic
            self.metadata["num_messages"] = 0

    def push_back(self, data: Union[PointCloudXf, Image, CompressedImage, Imu], timestamp, *args, **kwargs):
        if "msg_type" not in self.metadata:
            self.metadata["msg_type"] = data.__class__.__name__

        if self.metadata["msg_type"] != data.__class__.__name__:
            raise RuntimeError(
                f"TopicHandler is bound to {self.metadata['msg_type']}. Cannot handle data of type {type(data)}")

        self.save_fn[self.metadata["msg_type"]](data, timestamp, *args, **kwargs)
        self.timestamps.append(timestamp)
        self.metadata["num_messages"] += 1

    def _save_cloud(self, data: PointCloudXf, timestamp, *args, **kwargs):
        # dest_path = self.data_f / Path(self.format_fn(self.metadata["num_messages"]) + ".bin")

        dest_path = str(self.data_f / Path(self.format_fn(self.metadata["num_messages"]) + ".ply")) # save as ply for more visualization

        device = o3d.core.Device("CPU:0")
        dtype = o3d.core.float32

        out_ply = o3d.t.geometry.PointCloud(device)
        
        xyz = np.array([data.points["x"], data.points["y"], data.points["z"]])
        xyz = xyz.T
        
        # print(data.points.dtype.names)

        out_ply.point["positions"] = o3d.core.Tensor(xyz, dtype, device)

        properties = list(data.points.dtype.names)
        properties.remove("x")
        properties.remove("y")
        properties.remove("z")

        # print(properties)
        time_fields = ["t", "timestamp", "ts", "time", "times", "timestamps"]


        for field in properties:
            if field in time_fields:
                out_ply.point[field] = o3d.core.Tensor(np.expand_dims(data.points[field] * 1e-9, axis=1), dtype, device)
            else:
                out_ply.point[field] = o3d.core.Tensor(np.expand_dims(data.points[field] * 1.0, axis=1), dtype, device)

        o3d.t.io.write_point_cloud(dest_path, out_ply)


        # write to a ply file using open3d
        
        # Save fields to metadata to recover it later.
        # We assume fields to remain constant through data of this topic
        # if "fields" not in self.metadata.keys():
        #     self.metadata["fields"] = [
        #         f.__dict__ for f in data.fields
        #     ]
        #     # Dump data.points.datatype in pickle (this shit is a workaround and should be fixed asap)
        #     import pickle
        #     with open(self.data_f / ".dtype.pkl", "wb") as f:
        #         pickle.dump(data.points.dtype, f)

        # if "pcloud_kitti_format" in kwargs: # TODO: is this t available for livox ? fix it tomorrow, and also the wrong opencv dependency
        #     if kwargs.get("pcloud_kitti_format"):
        #         # print(data.points["t"]) # t is proportional to point idx
        #         ts = data.points["t"] * 1e-9 # in second

        #         print(ts)

        #         # clip_points = np.stack([data.points["x"], data.points["y"], data.points["z"], data.points["intensity"], ts],
        #         #                        axis=1)

        #         clip_points = np.stack([data.points["x"], data.points["y"], data.points["z"], ts],
        #                                 axis=1)
        #         clip_points.tofile(dest_path)
        #         return

        # data.points.tofile(dest_path)

    def _save_image(self, data: Image, timestamp: float, *args, **kwargs):
        dest_path = self.data_f / Path(self.format_fn(self.metadata["num_messages"]) + ".png")

        if "rgb_convert" in kwargs:
            if kwargs.get("rgb_convert"):
                data.image = cv2.cvtColor(data.image, cv2.COLOR_BAYER_RG2RGB)
                data.encoding = "rgb8"

        if not "encoding" in self.metadata.keys():
            self.metadata["encoding"] = data.encoding

        cv2.imwrite(str(dest_path), data.image)

    def _save_imu(self, data: Imu, timestamp: float, *args, **kwargs):
        if not self.imu_dest:
            self.imu_dest = (self.data_f / "imu.txt").open("w")
            self.imu_dest.write(IMU_CSV_HEADER)

        imu_line = np.concatenate((data.linear_acceleration, data.angular_velocity, data.orientation,
                                   data.linear_acceleration_covariance, data.angular_velocity_covariance,
                                   data.orientation_covariance), axis=0)
        self.imu_dest.write(",".join(map(str, imu_line.tolist())) + "\n")

    def topic(self) -> str:
        return self.metadata["topic"]

    def close(self):
        with self.metadata_f.open("w") as f:
            json.dump(self.metadata, f)
        with self.timestamps_f.open("a") as f:
            f.writelines([
                str(np.datetime64(t, "ns")) + "\n" for t in self.timestamps])


class KittiWriter:
    def __init__(self, data_dir: Path, rgb_convert: bool = True, *args, **kwargs):
        data_dir.mkdir(parents=True, exist_ok=True)
        self.destination_dir = data_dir
        self.data_handles = {}
        self.rgb_convert = rgb_convert

    def __enter__(self):
        return self

    def publish(self, timestamp, topic: str, message: Union[PointCloudXf, Image, Imu]):
        if topic not in self.data_handles.keys():
            # Infer path to store stuff
            # Remove first / on topic
            # Remove /image_raw if present
            handle_dir = topic[1:].replace("/image_raw", "").replace("/data", "").replace("/", "_")
            # print(handle_dir)
            self.data_handles[topic] = KittiTopicHandler(self.destination_dir / Path(handle_dir), topic,
                                                         lambda x: f"{x:010d}")

        self.data_handles[topic].push_back(message, timestamp, rgb_convert=self.rgb_convert)

    def __exit__(self, exc_type, exc_val, exc_tb):
        for handle in self.data_handles:
            self.data_handles[handle].close()
