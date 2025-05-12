import sys

import typer
from pathlib import Path
from rich.console import Group
from rich.panel import Panel
from rich.progress import track
from typing import Sequence
from typing_extensions import Annotated
from rosbag_converter.datasets import RosReader
from rosbag_converter.datasets.convert_bag import OutputDataInterface, OutputDataInterface_lut
from rosbag_converter.tools.console import console
from rosbags import convert as rosconvert

app = typer.Typer()


@app.command(help="Convert a sequence from ROS1 to other known formats")
def convert(to: Annotated[OutputDataInterface, typer.Argument(help="Desired data format", show_default=False)],
            input_dir: Annotated[
                Path, typer.Argument(help="Input bag or directory containing multiple bags", show_default=False)],
            output_dir: Annotated[
                Path, typer.Argument(help="Output directory in which the data will be stored", show_default=False)],
            rgb_conversion: Annotated[
                bool, typer.Option(
                    help="Enable BayerRG8->RGB conversion during conversion in KITTI format."
                         " Disable this flag to reduce the memory footprint of the converted folder.",
                    show_default=True)] = True) -> None:
    console.print(f"Converting {input_dir} to {to} format at {output_dir}")
    if to == OutputDataInterface.ros2:
        if not input_dir.is_dir():
            print("Processing...")
            rosconvert.convert(input_dir, output_dir / input_dir.stem)
        else:
            for item in track(list(input_dir.iterdir()), description="Processing..."):
                if item.suffix == '.bag':
                    rosconvert.convert(item, output_dir / item.stem)
    else:
        with RosReader(input_dir) as reader:
            with OutputDataInterface_lut[to](output_dir, rgb_convert=rgb_conversion) as writer:
                for timestamp, topic, message in track(reader, description="Processing..."):
                    if timestamp is not None:
                        writer.publish(timestamp, topic, message)
    console.print(":tada: Completed")


if __name__ == "__main__":
    app()
