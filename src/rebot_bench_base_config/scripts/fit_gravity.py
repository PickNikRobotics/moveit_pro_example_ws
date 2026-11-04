#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Offline static gravity fit from collection MCAPs; never publishes or calls ROS.

Repeat --dataset RECORDING_BAG HEALTH_BAG to combine runs. Supports the original
paired-approach markers and the named-waypoint collection markers. The output
URDF is an equivalent static model, not uniquely identified physical inertias.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as KDL

JOINTS = [f"joint{i}" for i in range(1, 7)]
GRAVITY = np.array([0.0, 0.0, -9.81])


def xyz(element, attribute):
    text = element.get(attribute, "0 0 0") if element is not None else "0 0 0"
    return np.array([float(x) for x in text.split()])


def rotation(rpy):
    value = KDL.Rotation.RPY(*rpy)
    return np.array([[value[i, j] for j in range(3)] for i in range(3)])


class GravityModel:
    """Linear mass/first-moment regressor for the controller's serial chain."""

    def __init__(self, description):
        self.root = ET.fromstring(description)
        self.links = {link.get("name"): link for link in self.root.findall("link")}
        parents = {j.find("child").get("link"): j for j in self.root.findall("joint")}
        link = "gripper_end"
        chain = []
        while link != "base_link":
            joint = parents[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        self.chain = list(reversed(chain))
        self.names = [j.find("child").get("link") for j in self.chain]
        active = [j.get("name") for j in self.chain if j.get("type") != "fixed"]
        if active != JOINTS:
            raise ValueError("Expected serial base_link -> gripper_end joints joint1…joint6")
        self.parameters = []
        for name in self.names:
            inertial = self.links[name].find("inertial")
            mass = float(inertial.find("mass").get("value"))
            com = xyz(inertial.find("origin"), "xyz")
            if not np.isfinite(mass) or mass <= 0 or not np.all(np.isfinite(com)):
                raise ValueError(f"Invalid nominal mass/COM: {name}")
            self.parameters.extend([mass, *(mass * com)])
        self.parameters = np.asarray(self.parameters)
        self.first_moments = np.array([i for i in range(len(self.parameters)) if i % 4])

    def geometry(self):
        # Inertials may legitimately differ after a prior fit; frames, including
        # fixed jaw/camera attachments, must still describe the same setup.
        return [(j.get("name"), j.get("type"), j.find("parent").get("link"),
                 j.find("child").get("link"), xyz(j.find("origin"), "xyz").tolist(),
                 xyz(j.find("origin"), "rpy").tolist(), xyz(j.find("axis"), "xyz").tolist())
                for j in sorted(self.root.findall("joint"), key=lambda j: j.get("name"))]

    def regressor(self, q):
        q = np.asarray(q)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("Expected six finite measured joint positions, radians")
        transform = np.eye(4)
        result = np.zeros((6, len(self.parameters)))
        axes, centers = [], []
        index = 0
        for segment, joint in enumerate(self.chain):
            frame = np.eye(4)
            frame[:3, :3] = rotation(xyz(joint.find("origin"), "rpy"))
            frame[:3, 3] = xyz(joint.find("origin"), "xyz")
            transform = transform @ frame
            if joint.get("type") != "fixed":
                if joint.get("type") not in ("revolute", "continuous"):
                    raise ValueError("Only this robot's revolute arm is supported")
                axis = xyz(joint.find("axis"), "xyz")
                axis /= np.linalg.norm(axis)
                axes.append(transform[:3, :3] @ axis)
                centers.append(transform[:3, 3].copy())
                value = KDL.Rotation.Rot(KDL.Vector(*axis), float(q[index]))
                frame = np.eye(4)
                frame[:3, :3] = [[value[a, b] for b in range(3)] for a in range(3)]
                transform = transform @ frame
                index += 1
            for row, (axis, center) in enumerate(zip(axes, centers)):
                result[row, 4 * segment] = -axis @ np.cross(transform[:3, 3] - center, GRAVITY)
                for coordinate in range(3):
                    result[row, 4 * segment + 1 + coordinate] = -axis @ np.cross(
                        transform[:3, coordinate], GRAVITY)
        return result

    def write_calibration(self, path, parameters):
        if not np.all(np.isfinite(parameters)):
            raise ValueError("Refusing nonfinite calibration")
        root = copy.deepcopy(self.root)
        root.insert(0, ET.Comment(
            " Static gravity calibration v1: nominal masses and inertia tensors; fitted "
            "gravity-equivalent first moments, NOT uniquely identified physical COMs. "
            "base_link -> gripper_end, J1-J6, empty closed jaw; SI units. "))
        for index, name in enumerate(self.names):
            mass = parameters[4 * index]
            com = parameters[4 * index + 1:4 * index + 4] / mass
            origin = root.find(f"link[@name='{name}']/inertial/origin")
            origin.set("xyz", " ".join(f"{x:.17g}" for x in com))
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def messages(path, topics):
    # Lazy imports keep the model and window policy independently testable.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in topics}
    if set(topics) - types.keys():
        raise ValueError(f"Missing topics in {path}: {set(topics) - types.keys()}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        yield topic, deserialize_message(data, types[topic]), stamp / 1e9


def window_attempt(samples, health, start):
    """One second, approximately100Hz; exact bounded rejection policy."""
    a = samples[(samples[:, 0] >= start) & (samples[:, 0] < start + 1)]
    h = health[(health[:, 0] >= start - .03) & (health[:, 0] <= start + 1)]
    errors = []
    if not 90 <= len(a) <= 120:
        errors.append("sample count outside90…120")
    if len(a) < 2 or (len(a) and (np.max(np.diff(a[:, 0])) > .03 or
                                 a[0, 0] - start > .03 or start + 1 - a[-1, 0] > .03)):
        errors.append("ROS sample gap")
    if len(h) < 2 or (len(h) and (np.max(np.diff(h[:, 0])) > .03 or
                                 h[0, 0] > start + .03 or h[-1, 0] < start + .97)):
        errors.append("health sample gap")
    if len(h) and np.any(h[:, 1:] != 0):
        errors.append("comm_state/fault/disabled")
    if not np.all(np.isfinite(a)):
        errors.append("nonfinite sample")
    if len(a):
        if np.any(np.ptp(a[:, 1:7], axis=0) > .003):
            errors.append("position drift exceeds0.003rad")
        constant = [JOINTS[j] for j in range(6)
                    if np.all(a[:, 8 + j].view(np.uint64) == a[0, 8 + j].view(np.uint64))]
        if constant:
            errors.append("bit-identical effort: " + ",".join(constant))
        if np.max(np.abs(a[:, 7])) > .03:
            errors.append("jaw is not closed")
    return {"start_epoch_s": start, "count": len(a), "errors": errors,
            "position_range_rad": np.ptp(a[:, 1:7], axis=0).tolist() if len(a) else [],
            "q_rad": a[:, 1:7].mean(axis=0).tolist() if len(a) else [],
            "tau_Nm": a[:, 8:14].mean(axis=0).tolist() if len(a) else [],
            "tau_std_Nm": a[:, 8:14].std(axis=0).tolist() if len(a) else []}


def load_dataset(recording, health_bag):
    descriptions = [msg.data for _, msg, _ in messages(recording, ["/robot_description"])]
    if len(set(descriptions)) != 1:
        raise ValueError("Need one unchanging recorded robot description per run")
    samples, health, markers, thermal = [], [], [], []
    paired_index = 0
    for topic, msg, receive in messages(health_bag, [
            "/joint_states", "/robstride_hardware_interface/robstride_state",
            "/robstride_hardware_interface/thermal_tripped", "/rosout"]):
        if topic == "/joint_states":
            order = [list(msg.name).index(j) for j in [*JOINTS, "joint7"]]
            if len(msg.effort) != len(msg.name) or len(msg.position) != len(msg.name):
                raise ValueError("JointState lacks measured position/effort")
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            samples.append([stamp, *[msg.position[j] for j in order], *[msg.effort[j] for j in order]])
        elif topic.endswith("/robstride_state"):
            # This driver uses a steady-clock header, not ROS epoch. Keep MCAP
            # receive epoch for alignment, without claiming it proves CAN freshness.
            bad = set(msg.id) != set(range(1, 8)) or len(msg.enabled) != 7 or len(msg.fault_bits) != 7
            health.append([receive, msg.comm_state, int(bad or not all(msg.enabled)), int(any(msg.fault_bits))])
        elif topic.endswith("/thermal_tripped"):
            thermal.append((receive, msg.data))
        elif "Gravity " in msg.msg and "static window begins" in msg.msg:
            match = re.search(r"Gravity waypoint (\d+): static window begins", msg.msg)
            if match:
                label, direction = int(match[1]), "single"
            else:
                direction = "positive" if "positive approach" in msg.msg else "negative"
                expected = "positive" if paired_index % 2 == 0 else "negative"
                if direction != expected:
                    raise ValueError("Unordered paired approach markers")
                label = paired_index // 2 + 1
                paired_index += 1
            stamp = msg.stamp.sec + msg.stamp.nanosec / 1e9
            markers.append((label, direction, stamp))
    if not markers or paired_index % 2:
        raise ValueError("No complete collection markers")
    keys = [(label, direction) for label, direction, _ in markers]
    if len(set(keys)) != len(keys):
        raise ValueError("Bag contains duplicate collection windows; split runs before fitting")
    samples, health = np.asarray(samples), np.asarray(health)
    if len(samples) == 0 or len(health) == 0 or not thermal or np.any(np.diff(samples[:, 0]) <= 0):
        raise ValueError("Missing data or non-increasing JointState stamps")
    audit, accepted = [], {}
    for label, direction, stamp in markers:
        attempts = []
        for retry in range(2):
            attempt = window_attempt(samples, health, stamp + retry)
            trip_window = [(time, value) for time, value in thermal
                           if stamp + retry - .15 <= time <= stamp + retry + 1]
            if not trip_window or trip_window[-1][0] < stamp + retry + .85:
                attempt["errors"].append("missing thermal feedback")
            elif any(tripped for _, tripped in trip_window):
                attempt["errors"].append("thermal trip")
            attempts.append(attempt)
            if not attempt["errors"]:
                accepted[label, direction] = attempt
                break
        audit.append({"pose": label, "direction": direction, "attempts": attempts})
    observations = []
    for label in sorted({label for label, _, _ in markers}):
        if (label, "single") in accepted:
            points = [accepted[label, "single"]]
        elif (label, "positive") in accepted and (label, "negative") in accepted:
            points = [accepted[label, "positive"], accepted[label, "negative"]]
        else:
            continue
        observations.append({"pose": label, "approaches": points})
    return descriptions[0], observations, audit, samples, health


def fit(model, observations, holdout_every=5):
    A = np.array([np.mean([model.regressor(a["q_rad"]) for a in p["approaches"]], axis=0)
                  for p in observations])
    y = np.array([np.mean([a["tau_Nm"] for a in p["approaches"]], axis=0) for p in observations])
    rng = np.random.default_rng(912)
    reference = rng.uniform([-2, .1, .1, -1.7, -.9, -2.5], [2, 2.8, 3.7, 1.3, .9, 2.5], (256, 6))
    R = np.vstack([model.regressor(q) for q in reference])
    h = model.first_moments
    _, structural_s, v = np.linalg.svd(R[:, h], full_matrices=False)
    rank = int(np.sum(structural_s > structural_s[0] * 1e-7))
    full_s = np.linalg.svd(R, compute_uv=False)
    if rank != np.sum(full_s > full_s[0] * 1e-7):
        raise ValueError("Nominal-mass representation does not span full gravity model")
    basis = v[:rank].T
    # Hold out the same waypoint IDs across datasets, preventing same-pose leakage.
    training = np.array([p["pose"] % holdout_every != 0 for p in observations])
    if training.sum() < 12 or (~training).sum() < 2:
        raise ValueError("Need at least12training and2held-out observations")
    X = (A[training][:, :, h] @ basis).reshape(-1, rank)
    residual = (y[training] - A[training] @ model.parameters).reshape(-1)
    delta, _, fitted_rank, singular = np.linalg.lstsq(X, residual, rcond=1e-7)
    if fitted_rank != rank or singular[0] / singular[-1] > 1e5:
        raise ValueError("Underdetermined or ill-conditioned recorded pose set")
    calibrated = model.parameters.copy()
    calibrated[h] += basis @ delta
    predictions = A @ calibrated
    nominal = A @ model.parameters
    fit_error = (predictions[training] - y[training]).reshape(-1)
    variance = float(fit_error @ fit_error / (len(fit_error) - rank))
    covariance = variance * np.linalg.pinv(X.T @ X)
    report = {"rank": rank, "condition": float(singular[0] / singular[-1]),
              "first_moment_std_kg_m": np.sqrt(np.maximum(0, np.diag(basis @ covariance @ basis.T))).tolist(),
              "uncertainty_note": "OLS independent equal-variance residual assumption; not a bound on friction, torque bias or model error",
              "first_moment_order": [f"{name}.h_{axis}" for name in model.names for axis in "xyz"],
              "observable_basis": basis.tolist(), "base_correction_kg_m": delta.tolist(),
              "fit_singular_values": singular.tolist(), "structural_singular_values": structural_s.tolist()}
    for name, mask in (("training", training), ("holdout", ~training)):
        report[name] = {"count": int(mask.sum()), "pose_ids": [p["pose"] for p, use in zip(observations, mask) if use]}
        for kind, value in (("nominal", nominal), ("fitted", predictions)):
            error = value[mask] - y[mask]
            report[name][kind] = {"rmse_Nm": np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
                                 "bias_Nm": error.mean(axis=0).tolist(),
                                 "max_abs_Nm": np.max(np.abs(error), axis=0).tolist()}
    report["observations"] = [dict(p, measured_Nm=y[i].tolist(), nominal_Nm=nominal[i].tolist(),
                                   fitted_Nm=predictions[i].tolist()) for i, p in enumerate(observations)]
    return calibrated, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", nargs=2, action="append", required=True,
                        metavar=("RECORDING_BAG", "HEALTH_BAG"))
    parser.add_argument("--output", type=Path, required=True, help="New output directory, never a live calibration")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; choose a new directory")
    paths = [tuple(Path(p).resolve() for p in pair) for pair in args.dataset]
    if len(set(paths)) != len(paths):
        parser.error("Duplicate dataset")
    loaded = [load_dataset(*pair) for pair in paths]
    model = GravityModel(loaded[0][0])
    observations = []
    for index, (description, points, _, _, _) in enumerate(loaded):
        if GravityModel(description).geometry() != model.geometry():
            raise ValueError(f"Dataset{index} belongs to an incompatible chain")
        observations.extend(dict(point, dataset=index) for point in points)
    parameters, report = fit(model, observations)
    args.output.mkdir(parents=True)
    calibration = args.output / "gravity_calibrated.urdf"
    model.write_calibration(calibration, parameters)
    report.update(schema_version=1, joints=JOINTS, gravity_m_s2=GRAVITY.tolist(),
                  datasets=[[str(p) for p in pair] for pair in paths],
                  calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest(),
                  note="Nominal masses/inertia tensors and unobservable first moments retained; single approaches are not friction-averaged; no controller activation")
    for index, (description, _, audit, samples, health) in enumerate(loaded):
        (args.output / f"model_{index}.urdf").write_text(description)
        (args.output / f"window_audit_{index}.json").write_text(json.dumps(audit, indent=2))
        np.savez_compressed(args.output / f"samples_{index}.npz", joint=samples, health=health)
    (args.output / "fit_report.json").write_text(json.dumps(report, indent=2))
    print(f"Calibration: {calibration}; static rank{report['rank']}; condition{report['condition']:.3f}")
    for name in ("training", "holdout"):
        print(name, report[name])


if __name__ == "__main__":
    main()
