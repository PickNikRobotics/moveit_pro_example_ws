from typing import IO, Any, Iterable, Iterator, Optional, Union, List

import copy
import multiprocessing as mp
from io import BytesIO
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages, McapROS2Message
from pathlib import Path

from collections import defaultdict
from torch.utils.data import Dataset, DataLoader
import torch
import zstandard as zstd
import io
import cv2 as cv
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

start_indicator_str = "Starting demonstration"
stop_indicator_str = "Stopping demonstration"
demonstration_indicator_topic = "/demonstration_indicator"

def get_mcap_files(data_dir="/media/paul/DATA/pushing_data"):
    directory_path = Path(data_dir)
    mcap_files = list(directory_path.glob("*.mcap.zstd"))
    return mcap_files


def get_ros_bag_reader(mcap_file: Path):
    byte_data = bytearray()
    dctx = zstd.ZstdDecompressor()
    with open(mcap_file, "rb") as compressed_file:
        with dctx.stream_reader(compressed_file) as reader:
            while chunk := reader.read(16384):  # Read in chunks
                byte_data.extend(chunk)
    reader = read_ros2_messages(
        io.BytesIO(byte_data), topics=[demonstration_indicator_topic]
    )
    return reader


def get_demonstration_meta_data(mcap_files: List[Path]):
    metadata = []
    demonstration_data = dict()
    for mcap_file in mcap_files:
        reader = get_ros_bag_reader(mcap_file)

        for mcap_ros_msg in reader:
            msg = mcap_ros_msg.ros_msg
            if msg.data == start_indicator_str:
                demonstration_data.clear()
                demonstration_data["start_time"] = mcap_ros_msg.publish_time_ns
                demonstration_data["files"] = {mcap_file}
            elif msg.data == stop_indicator_str and "start_time" in demonstration_data:
                demonstration_data["files"].add(mcap_file)
                demonstration_data["stop_time"] = mcap_ros_msg.publish_time_ns
                metadata.append(copy.deepcopy(demonstration_data))
                demonstration_data.clear()

    return metadata


mcap_files = get_mcap_files()
metadata = get_demonstration_meta_data(mcap_files)

for data in metadata:
    print(data)

exit(1)


class MCAPDataset(Dataset):
    def __init__(self, mcap_file_path, topic_name):
        self.mcap_file_path = mcap_file_path
        self.topic_name = topic_name
        self.messages = self._load_messages()

    def _load_messages(self):
        messages = []
        with open(self.mcap_file_path, "rb") as f:
            reader = make_reader(f)
            for schema, channel, message in reader.iter_messages(
                    topics=[self.topic_name]
            ):
                messages.append(message)
        return messages

    def __len__(self):
        return len(self.messages)

    def __getitem__(self, idx):
        # Extract data from the message
        message = self.messages[idx]
        return message.data  # Or parse the data based on schema


# Usage
mcap_file_path = "your_mcap_file.mcap"
topic_name = "your_topic_name"
dataset = MCAPDataset(mcap_file_path, topic_name)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# Iterate through the dataloader
for batch in dataloader:
    # Process the batch of data
    print(batch)
