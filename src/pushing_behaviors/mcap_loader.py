import time
from typing import List

import multiprocessing as mp
from mcap_ros2.reader import read_ros2_messages
from pathlib import Path
import numpy as np

from torch.utils.data import IterableDataset
import torch
import zstandard as zstd
import io
import cv2
from cv_bridge import CvBridge


bridge = CvBridge()


class ROSConfigData:
    def __init__(self, start_indicator_str,
                 stop_indicator_str, demonstration_indicator_topic,
                 image_topic, joint_states_topic, data_dir):
        self.start_indicator_str = start_indicator_str
        self.stop_indicator_str = stop_indicator_str
        self.demonstration_indicator_topic = demonstration_indicator_topic
        self.image_topic = image_topic
        self.joint_states_topic = joint_states_topic
        self.data_dir = data_dir


def get_mcap_files(data_dir):
    directory_path = Path(data_dir)
    mcap_files = list(directory_path.glob("*.mcap.zstd"))
    return mcap_files


def get_ros_bag_reader(mcap_file: Path, ros_config_data: ROSConfigData):
    byte_data = bytearray()
    dctx = zstd.ZstdDecompressor()
    with open(mcap_file, "rb") as compressed_file:
        with dctx.stream_reader(compressed_file) as reader:
            while chunk := reader.read(16384):  # Read in chunks
                byte_data.extend(chunk)
    reader = read_ros2_messages(
        io.BytesIO(byte_data), topics=[ros_config_data.demonstration_indicator_topic]
    )
    return reader, byte_data


def interpolate_joint_state(last_msg, mcap_ros_msg, target_time):
    joints1 = np.array(last_msg.ros_msg.position)
    joints2 = np.array(mcap_ros_msg.ros_msg.position)
    t1 = last_msg.publish_time_ns
    t2 = mcap_ros_msg.publish_time_ns
    return (joints2 - joints1) * (t2 - target_time) / (t2 - t1)


def fill_data(demonstration_data, source_map, ros_config_data: ROSConfigData, delta_time=0.2):
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240

    data_point = []
    for ind, mcap_file in enumerate(demonstration_data['files']):
        byte_data = source_map[mcap_file]
        reader = read_ros2_messages(io.BytesIO(byte_data), topics=[ros_config_data.image_topic],
                                    start_time=demonstration_data['start_time'],
                                    end_time=demonstration_data['end_time'])
        mcap_ros_msg = next(reader)
        image_times = []
        last_time = mcap_ros_msg.publish_time_ns
        for mcap_ros_msg in reader:
            if (mcap_ros_msg.publish_time_ns - last_time) / 1E6 > delta_time * 1E3:
                last_time = mcap_ros_msg.publish_time_ns
                cv_image = bridge.imgmsg_to_cv2(mcap_ros_msg.ros_msg, desired_encoding='rgb8')
                resized_image = cv2.resize(cv_image, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
                img = np.array(resized_image, dtype=np.float32) / 255.0
                data_point.append({"wrist_rgb": img})
                image_times.append(mcap_ros_msg.publish_time_ns)

        reader = read_ros2_messages(io.BytesIO(byte_data), topics=[ros_config_data.joint_states_topic],
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
                joint_state = interpolate_joint_state(last_msg, mcap_ros_msg, target_time)
                data_point[image_index]["joint_states"] = joint_state
                image_index += 1

            last_msg = mcap_ros_msg

    return data_point


def get_demonstration_meta_data(mcap_files: List[Path], ros_config_data: ROSConfigData):
    demonstration_data = dict()
    source_map = dict()
    for mcap_file in mcap_files:
        reader, source = get_ros_bag_reader(mcap_file, ros_config_data)
        source_map[mcap_file] = source
        for mcap_ros_msg in reader:
            msg = mcap_ros_msg.ros_msg
            if msg.data == ros_config_data.start_indicator_str:
                demonstration_data.clear()
                demonstration_data["start_time"] = mcap_ros_msg.publish_time_ns

            if "files" in demonstration_data:
                demonstration_data["files"].add(mcap_file)
            else:
                demonstration_data["files"] = {mcap_file}

            if msg.data == ros_config_data.stop_indicator_str and "start_time" in demonstration_data:
                # TODO Apparently ROS bags skip frames when splitting. We therefore want to skip over those?
                if len(demonstration_data["files"]) > 1:
                    continue
                demonstration_data["end_time"] = mcap_ros_msg.publish_time_ns
                data_point = fill_data(demonstration_data, source_map, ros_config_data)
                source_map.clear()
                source_map[mcap_file] = source
                yield data_point
    yield None


def process_ros_bags(q, ros_config_data):
    mcap_files = get_mcap_files(ros_config_data.data_dir)
    metadata = get_demonstration_meta_data(mcap_files, ros_config_data)
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
    def __init__(self, batch=4, start_indicator_str="Starting demonstration",
                 stop_indicator_str="Stopping demonstration", demonstration_indicator_topic="/demonstration_indicator",
                 image_topic="/wrist_camera/color", joint_states_topic="/joint_states",
                 data_dir="/media/paul/DATA/pushing_data"):

        self.ros_config_data = ROSConfigData(start_indicator_str, stop_indicator_str, demonstration_indicator_topic,
                                             image_topic, joint_states_topic, data_dir)

        self.ctx = mp.get_context('spawn')
        self.q = self.ctx.Queue()
        self.p = self.ctx.Process(target=process_ros_bags, args=(self.q, self.ros_config_data))
        self.p.start()
        self.batch = batch
        self.demonstration = []

    def __iter__(self):
        return self

    def __next__(self):
        if len(self.demonstration) < self.batch:
            self.demonstration = self.q.get()
        if self.demonstration is None:
            raise StopIteration

        data_point = self.demonstration[0:self.batch]
        self.demonstration = self.demonstration[self.batch:]

        return data_point


if __name__ == '__main__':
    loader = MCAPDataset()
    for data_point in loader:
        print(data_point)
