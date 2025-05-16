<div align="center">
    <h1>ROS Bag Converter</h1>
</div>

This kit contains utilities to convert rosbags to KITTI format, involving LiDARs, images, GPS and IMU information.

This is adapted from the [VBR dataset devkit](https://github.com/rvp-group/vbr-devkit). All credits to the original authors of the [VBR dataset](https://rvp-group.net/slam-dataset.html).

## Install

After cloning the repository, install the python package by running:

```shell
cd python
pip install .
```

## Usage

### Convert format

The sequences are provided in ROS1 format. We offer a convenient tool to change representation if you prefer working on a different format like in KITTI dataset.
You can see the supported formats by typing:

```shell
rosbag_converter --help
```

To convert a bag or a sequence of bags, type:
```shell
rosbag_converter <desired_format> <input_directory/input_bag> <output_directory>
```

for instance, we could convert the `campus_train0_00` sequence to `kitti` format as follows:

```shell
rosbag_converter kitti ./rosbag/campus_train0_00.bag ./extracted_data/
```

Note that the input can also be a folder containing multiple bags. The converter will merge them and ouput into a single folder.

Besides, if the RGB images are already debayered, please set the `--no-rgb-conversion` flag, for example:

```shell
rosbag_converter --no-rgb-conversion kitti ./rosbag/campus_train0_00.bag ./extracted_data/
```

You can expect the following result:

```
data
  - your_output_dir
    - camera_left
      - timestamps.txt
      - data
        - 0000000000.png
        - 0000000001.png
        - ...
    - camera_right
      - timestamps.txt
      - data
        - 0000000000.png
        - 0000000001.png
        - ...
    - ouster_points
      - timestamps.txt
      - data
        - 0000000000.ply
        - 0000000001.ply
        - ...
    - ... 
```

The extracted data can be then used as the input to systems such as [PINGS](https://github.com/PRBonn/PINGS) together with the provided calibration information.