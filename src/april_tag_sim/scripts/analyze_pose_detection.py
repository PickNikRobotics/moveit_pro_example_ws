#!/usr/bin/env python3

# Copyright 2025 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import yaml


DIR_0DEG = "../objectives/0degree_runs"
DIR_45DEG = "../objectives/45degree_runs"
NUM_TAGS = 6


def load_yaml_file(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def save_yaml_file(data, file_path):
    with open(file_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)


def extract_yaml_data(tag_dfs, base_path, num_runs):
    base_path = Path(base_path)

    for tag_num in range(1, NUM_TAGS + 1):
        tag_file = base_path / f"tag{tag_num}.yaml"
        runs_yaml = load_yaml_file(tag_file)
        for run_num in runs_yaml.keys():
            pose_yaml = runs_yaml[run_num]
            position = pose_yaml["pose"]["position"]
            orientation = pose_yaml["pose"]["orientation"]

            # Extract position and orientation
            tag_dfs[tag_num].loc[run_num] = {
                "x": position["x"],
                "y": position["y"],
                "z": position["z"],
                "qw": orientation["w"],
                "qx": orientation["x"],
                "qy": orientation["y"],
                "qz": orientation["z"],
            }


def get_deviations(row):
    pos_mag = np.linalg.norm([row["x"], row["y"], row["z"]])
    r = R.from_quat([row["qx"], row["qy"], row["qz"], row["qw"]])
    rot_mag = r.magnitude()
    rot_mag_deg = np.degrees(rot_mag)
    rot_vec = r.as_rotvec()
    return {
        "x": row["x"],
        "y": row["y"],
        "z": row["z"],
        "pos_mag": pos_mag,
        "rx": rot_vec[0],
        "ry": rot_vec[1],
        "rz": rot_vec[2],
        "rot_mag": rot_mag_deg,
    }


def create_dfs(num_runs):
    tag_dfs = {
        tag_num: pd.DataFrame(
            columns=["x", "y", "z", "qx", "qy", "qz", "qw"],
            index=range(1, num_runs + 1),
        )
        for tag_num in range(1, NUM_TAGS + 1)
    }
    deviation_dfs = {
        tag_num: pd.DataFrame(
            columns=["x", "y", "z", "pos_mag", "rx", "ry", "rz", "rot_mag"],
            index=range(1, num_runs + 1),
        )
        for tag_num in range(1, NUM_TAGS + 1)
    }
    mean_df = pd.DataFrame(
        columns=["x", "y", "z", "pos_mag", "rx", "ry", "rz", "rot_mag"],
        index=range(1, NUM_TAGS + 1),
    )
    stdev_df = pd.DataFrame(
        columns=["x", "y", "z", "pos_mag", "rx", "ry", "rz", "rot_mag"],
        index=range(1, NUM_TAGS + 1),
    )
    return tag_dfs, deviation_dfs, mean_df, stdev_df


def fill_deviations_dfs(deviation_dfs, tag_dfs):
    for tag_num in range(1, NUM_TAGS + 1):
        results = tag_dfs[tag_num].apply(lambda row: get_deviations(row), axis=1)
        for idx, res in results.items():
            deviation_dfs[tag_num].loc[idx] = res


def fill_summary_dfs(mean_df, stdev_df, deviation_dfs):
    for tag_num in range(1, NUM_TAGS + 1):
        mean_df.loc[tag_num] = deviation_dfs[tag_num].mean()
        stdev_df.loc[tag_num] = deviation_dfs[tag_num].std()


def create_plot(y1, err1, y2, err2, ylabel, title):
    plt.figure(figsize=(10, 6))
    segments = [0, 1, 2, 3, 4, 5]
    plt.errorbar(
        segments,
        y1,
        yerr=err1,
        fmt="o",
        capsize=5,
        elinewidth=2,
        markerfacecolor="black",
        label="Parallel (0°) detections",
    )
    plt.errorbar(
        segments,
        y2,
        yerr=err2,
        fmt="o",
        capsize=5,
        elinewidth=2,
        markerfacecolor="black",
        label="Angled (45°) detections",
    )

    # X-axis
    tag_sizes = [0.48, 0.32, 0.24, 0.16, 0.12, 0.08]
    plt.xticks(segments, [f"Tag {i+1}, size: {tag_sizes[i]}" for i in segments])

    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


if __name__ == "__main__":
    sample_yaml = load_yaml_file(DIR_0DEG + "/tag1.yaml")
    num_runs = len(sample_yaml)

    tag_dfs_0deg, deviation_dfs_0deg, mean_df_0deg, stdev_df_0deg = create_dfs(num_runs)
    tag_dfs_45deg, deviation_dfs_45deg, mean_df_45deg, stdev_df_45deg = create_dfs(
        num_runs
    )

    # Extract data from YAML files into the tag DataFrames.
    extract_yaml_data(tag_dfs_0deg, DIR_0DEG, num_runs)
    extract_yaml_data(tag_dfs_45deg, DIR_45DEG, num_runs)

    # Fill deviations DataFrame.
    fill_deviations_dfs(deviation_dfs_0deg, tag_dfs_0deg)
    fill_deviations_dfs(deviation_dfs_45deg, tag_dfs_45deg)

    # Fill summary (avg & stdev) DataFrames.
    fill_summary_dfs(mean_df_0deg, stdev_df_0deg, deviation_dfs_0deg)
    fill_summary_dfs(mean_df_45deg, stdev_df_45deg, deviation_dfs_45deg)

    # Positional deviation plot
    pos_mag_means_0deg = mean_df_0deg["pos_mag"].values
    pos_mag_stdevs_0deg = stdev_df_0deg["pos_mag"].values
    pos_mag_means_45deg = mean_df_45deg["pos_mag"].values
    pos_mag_stdevs_45deg = stdev_df_45deg["pos_mag"].values
    create_plot(
        pos_mag_means_0deg,
        pos_mag_stdevs_0deg,
        pos_mag_means_45deg,
        pos_mag_stdevs_45deg,
        "AprilTag Position Deviation (m)",
        f"AprilTag Position Deviation by Tag (n={num_runs})",
    )

    # Rotational deviation plot
    # y-values: mean rot_mag for each segment
    rot_mag_means_0deg = mean_df_0deg["rot_mag"].values
    rot_mag_stdevs_0deg = stdev_df_0deg["rot_mag"].values
    rot_mag_means_45deg = mean_df_45deg["rot_mag"].values
    rot_mag_stdevs_45deg = stdev_df_45deg["rot_mag"].values
    create_plot(
        rot_mag_means_0deg,
        rot_mag_stdevs_0deg,
        rot_mag_means_45deg,
        rot_mag_stdevs_45deg,
        "AprilTag Orientation Deviation (°)",
        f"AprilTag Orientation Deviation by Tag (n={num_runs})",
    )

    plt.show()
