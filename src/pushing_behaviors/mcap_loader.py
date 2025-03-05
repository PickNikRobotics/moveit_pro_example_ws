import time
from typing import IO, Any, Iterable, Iterator, Optional, Union, List

import copy
import multiprocessing as mp
from io import BytesIO
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages, McapROS2Message
from pathlib import Path
import numpy as np

from collections import defaultdict
from torch.utils.data import Dataset, IterableDataset, DataLoader
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
image_topic = "/wrist_camera/color"
joint_states_topic = "/joint_states"


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
    return reader, byte_data


def interpolate_joint_state(last_msg, mcap_ros_msg, target_time):
    joints1 = np.array(last_msg.ros_msg.position)
    joints2 = np.array(mcap_ros_msg.ros_msg.position)
    t1 = last_msg.publish_time_ns
    t2 = mcap_ros_msg.publish_time_ns
    return (joints2 - joints1) * (t2 - target_time) / (t2 - t1)


def fill_data(demonstration_data, source_map, delta_time=0.2):
    demonstration_data["wrist_rgb"] = []
    demonstration_data["joint_states"] = []
    for ind, mcap_file in enumerate(demonstration_data['files']):
        byte_data = source_map[mcap_file]
        reader = read_ros2_messages(io.BytesIO(byte_data), topics=[image_topic],
                                    start_time=demonstration_data['start_time'],
                                    end_time=demonstration_data['end_time'])
        mcap_ros_msg = next(reader)
        image_times = []
        last_time = mcap_ros_msg.publish_time_ns
        for mcap_ros_msg in reader:
            if (mcap_ros_msg.publish_time_ns - last_time) / 1E6 > delta_time * 1E3:
                last_time = mcap_ros_msg.publish_time_ns
                cv_image = bridge.imgmsg_to_cv2(mcap_ros_msg.ros_msg, desired_encoding='rgb8')
                demonstration_data["wrist_rgb"].append(cv_image)
                image_times.append(mcap_ros_msg.publish_time_ns)

        reader = read_ros2_messages(io.BytesIO(byte_data), topics=[joint_states_topic],
                                    start_time=demonstration_data['start_time'],
                                    end_time=demonstration_data['end_time'])

        mcap_ros_msg = next(reader)
        last_msg = mcap_ros_msg
        image_index = 0
        for mcap_ros_msg in reader:
            if image_index >= len(image_times):
                break
            target_time = image_times[image_index]
            if last_msg.publish_time_ns < target_time <= mcap_ros_msg.publish_time_ns:
                image_index += 1
                joint_state = interpolate_joint_state(last_msg, mcap_ros_msg, target_time)
                demonstration_data["joint_states"].append(joint_state)

            last_msg = mcap_ros_msg



def get_demonstration_meta_data(mcap_files: List[Path]):
    demonstration_data = dict()
    source_map = dict()
    for mcap_file in mcap_files:
        reader, source = get_ros_bag_reader(mcap_file)
        source_map[mcap_file] = source
        for mcap_ros_msg in reader:
            msg = mcap_ros_msg.ros_msg
            if msg.data == start_indicator_str:
                demonstration_data.clear()
                demonstration_data["start_time"] = mcap_ros_msg.publish_time_ns

            if "files" in demonstration_data:
                demonstration_data["files"].add(mcap_file)
            else:
                demonstration_data["files"] = {mcap_file}

            if msg.data == stop_indicator_str and "start_time" in demonstration_data:
                demonstration_data["end_time"] = mcap_ros_msg.publish_time_ns
                fill_data(demonstration_data, source_map)
                source_map.clear()
                source_map[mcap_file] = source
                yield copy.deepcopy(demonstration_data)

    yield None


def foo(q):
    mcap_files = get_mcap_files()
    metadata = get_demonstration_meta_data(mcap_files)
    while True:
        if q.qsize() >= 2:
            time.sleep(0.1)
        else:
            val = next(metadata)
            if val is None:
                break
            q.put(val)

    q.put(None)


class MCAPDataset(IterableDataset):
    def __init__(self):
        self.ctx = mp.get_context('spawn')
        self.q = self.ctx.Queue()
        self.p = self.ctx.Process(target=foo, args=(self.q,))
        self.p.start()

    def __iter__(self):
        return self

    def __next__(self):
        data_point = self.q.get()
        if data_point is None:
            raise StopIteration
        return data_point


if __name__ == '__main__':
    loader = MCAPDataset()
    for i in loader:
        print("i")
