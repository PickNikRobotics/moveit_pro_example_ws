"""Node construction and teardown. Needs rclpy, unlike the pure contract tests."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from pathlib import Path
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

import so101_camera_bridge  # noqa: E402
from so101_camera_bridge.bridge_node import RtspCameraBridge  # noqa: E402

# Unroutable, so the decoder fails fast instead of reaching a real camera.
DEAD_URL = "rtsp://127.0.0.1:1/none"


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node() -> RtspCameraBridge:
    from rclpy.parameter import Parameter

    node = RtspCameraBridge()
    node.set_parameters(
        [
            Parameter("head_url", Parameter.Type.STRING, DEAD_URL),
            Parameter("gripper_url", Parameter.Type.STRING, DEAD_URL),
        ]
    )
    return node


class _FakePublisher:
    """Reports a subscription count the test controls and records publications."""

    def __init__(self) -> None:
        self.subscription_count = 0
        self.published: list = []

    def get_subscription_count(self) -> int:
        return self.subscription_count

    def publish(self, message) -> None:
        self.published.append(message)


class _FakeWorker:
    """A decoder the test can start, stop, and hand frames to."""

    def __init__(self, spec, frame=None) -> None:
        self.spec = spec
        self.running = False
        self.stopping = False
        self.starts = 0
        self.stops = 0
        self._frame = frame
        self._sequence = 1 if frame is not None else 0

    def start(self) -> None:
        if self.running:
            return
        self.starts += 1
        self.running = True

    def stop(self) -> None:
        self.stops += 1
        self.stopping = True

    def join(self, timeout: float | None = None) -> bool:
        self.running = False
        self.stopping = False
        return True

    def newest_after(self, sequence: int):
        if self._frame is None or self._sequence <= sequence:
            return None
        return self._sequence, self._frame


def _instrument(
    node: RtspCameraBridge, frames: dict | None = None
) -> tuple[dict, dict]:
    """Swap in fake publishers and workers, leaving `_publish_latest` itself real."""
    frames = frames or {}
    publishers = {
        name: (_FakePublisher(), _FakePublisher()) for name in node._camera_publishers
    }
    workers = {
        worker.spec.name: _FakeWorker(worker.spec, frames.get(worker.spec.name))
        for worker in node._workers
    }
    node._camera_publishers = publishers
    node._workers = list(workers.values())
    return publishers, workers


def _set_subscribers(publishers: dict, count: int) -> None:
    for image_publisher, _ in publishers.values():
        image_publisher.subscription_count = count


@pytest.fixture
def ros_with_args(request):
    rclpy.init(args=request.param)
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize(
    "ros_with_args,message",
    [
        (["--ros-args", "-p", "image_width:=0"], "image_width"),
        (["--ros-args", "-p", "image_height:=-1"], "image_height"),
        (
            ["--ros-args", "-p", "decoder_linger_seconds:=-1.0"],
            "decoder_linger_seconds",
        ),
    ],
    indirect=["ros_with_args"],
)
def test_invalid_geometry_or_linger_is_rejected_at_construction(
    ros_with_args, message: str
) -> None:
    """Bad values must fail bring-up, not surface later as wrong intrinsics or a
    decoder that never stops."""
    with pytest.raises(ValueError, match=message):
        RtspCameraBridge()


def test_destroy_node_does_not_raise(ros) -> None:
    """rclpy's Node keeps its own `_publishers` list and walks it on teardown.

    Shadowing that name with a dict keyed by camera makes destroy_node() index a
    dict by integer, so every shutdown raised KeyError after the node had already
    stopped serving — visible only in the logs, and easy to mistake for noise.
    """
    node = _node()
    node.destroy_node()  # must not raise


def test_camera_publishers_do_not_shadow_the_node_attribute(ros) -> None:
    """Pins the cause, so a rename back is caught at the point of the mistake."""
    node = _node()
    try:
        assert isinstance(node._publishers, list)
        assert set(node._camera_publishers) == {"head", "gripper"}
    finally:
        node.destroy_node()


def test_no_subscribers_publishes_camera_info_without_decoding(ros) -> None:
    """The info topic stands on its own; nothing decodes until asked."""
    node = _node()
    try:
        publishers, workers = _instrument(node)
        node._publish_latest()
        for name, (image_publisher, info_publisher) in publishers.items():
            assert len(info_publisher.published) == 1, f"{name} published no CameraInfo"
            assert image_publisher.published == [], f"{name} published an image"
            assert workers[name].starts == 0
    finally:
        node.destroy_node()


def test_camera_info_before_any_frame_uses_the_declared_geometry(ros) -> None:
    """640x480 is the declared default and must describe a plausible pinhole."""
    node = _node()
    try:
        publishers, _ = _instrument(node)
        node._publish_latest()
        info = publishers["head"][1].published[0]
        assert (info.width, info.height) == (640, 480)
        assert info.k[2] == 319.5
        assert info.k[5] == 239.5
        assert info.header.frame_id == "overhead_cam_optical_frame"
    finally:
        node.destroy_node()


def test_subscriber_starts_the_decoder(ros) -> None:
    """A subscriber on the image topic is what turns decoding on."""
    node = _node()
    try:
        publishers, workers = _instrument(node)
        _set_subscribers(publishers, 1)
        node._publish_latest()
        assert workers["head"].starts == 1
        assert workers["gripper"].starts == 1
        node._publish_latest()
        assert workers["head"].starts == 1, "a running decoder was restarted"
    finally:
        node.destroy_node()


def test_frames_are_published_and_not_republished(ros) -> None:
    """Only frames newer than the last published sequence go out."""
    node = _node()
    try:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        publishers, _ = _instrument(node, frames={"head": frame, "gripper": frame})
        _set_subscribers(publishers, 1)
        node._publish_latest()
        node._publish_latest()
        image_publisher = publishers["head"][0]
        assert len(image_publisher.published) == 1
        image = image_publisher.published[0]
        assert (image.width, image.height) == (640, 480)
        assert image.encoding == "bgr8"
        assert image.step == 640 * 3
    finally:
        node.destroy_node()


def test_decoder_survives_a_brief_gap_in_subscribers(ros) -> None:
    """Regression: a subscriber that flips away and back cost a full RTSP
    teardown and reconnect (measured 1.4-2.0 s to the next frame) because the
    stop fired on the first tick that read zero."""
    node = _node()
    try:
        publishers, workers = _instrument(node)
        node._decoder_linger = 0.5
        _set_subscribers(publishers, 1)
        node._publish_latest()
        _set_subscribers(publishers, 0)
        for _ in range(5):
            node._publish_latest()
        assert workers["head"].stops == 0, "decoder stopped inside the linger window"
        assert workers["head"].running
    finally:
        node.destroy_node()


def test_returning_subscriber_cancels_the_pending_stop(ros) -> None:
    """The countdown restarts on re-subscribe, so flipping never accumulates."""
    node = _node()
    try:
        publishers, workers = _instrument(node)
        node._decoder_linger = 0.3
        _set_subscribers(publishers, 1)
        node._publish_latest()

        _set_subscribers(publishers, 0)
        node._publish_latest()
        time.sleep(0.2)
        _set_subscribers(publishers, 1)
        node._publish_latest()

        _set_subscribers(publishers, 0)
        node._publish_latest()
        time.sleep(0.2)
        node._publish_latest()
        assert workers["head"].stops == 0, "the linger window did not restart"
    finally:
        node.destroy_node()


def test_decoder_stops_once_the_linger_elapses(ros) -> None:
    """Sustained absence of subscribers still releases the upstream stream."""
    node = _node()
    try:
        publishers, workers = _instrument(node)
        node._decoder_linger = 0.2
        _set_subscribers(publishers, 1)
        node._publish_latest()
        node._last_sequences["head"] = 7

        _set_subscribers(publishers, 0)
        node._publish_latest()
        time.sleep(0.3)
        node._publish_latest()
        assert workers["head"].stops == 1
        assert node._last_sequences["head"] == 0, "sequence bookkeeping not reset"

        node._publish_latest()
        assert workers["head"].stops == 1, "stop re-issued while winding down"
    finally:
        node.destroy_node()


def test_camera_info_follows_the_stream_when_it_differs_from_the_parameters(
    ros,
) -> None:
    """Intrinsics for a size the stream does not produce silently corrupt every
    consumer that projects pixels to poses, so the decoded frame wins."""
    node = _node()
    try:
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        publishers, _ = _instrument(node, frames={"head": frame})
        _set_subscribers(publishers, 1)
        node._publish_latest()
        node._publish_latest()

        image = publishers["head"][0].published[0]
        info = publishers["head"][1].published[-1]
        assert (image.width, image.height) == (1280, 720)
        assert (info.width, info.height) == (image.width, image.height)
        assert info.k[2] == 639.5
        assert info.k[5] == 359.5
        assert "head" in node._geometry_warned
    finally:
        node.destroy_node()


def test_camera_info_and_image_share_a_stamp(ros) -> None:
    """Consumers pair them with an exact-time synchronizer."""
    node = _node()
    try:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        publishers, _ = _instrument(node, frames={"head": frame})
        _set_subscribers(publishers, 1)
        node._publish_latest()
        image = publishers["head"][0].published[0]
        info = publishers["head"][1].published[0]
        assert image.header.stamp == info.header.stamp
    finally:
        node.destroy_node()


UPSTREAM_WAIT_SET_RACE = "failed to initialize wait set"


# --- process-level signal regression: needs main() in a real child process ---

PACKAGE_ROOT = str(Path(so101_camera_bridge.__file__).resolve().parent.parent)
DEAD_URL_ARGS = [
    "-p",
    f"head_url:={DEAD_URL}",
    "-p",
    f"gripper_url:={DEAD_URL}",
]
READY_LOG = "Publishing read-only SO-101 camera taps"
SPIN_SETTLE_SECONDS = 1.0


def _spawn_bridge() -> subprocess.Popen[str]:
    """Run main() in its own process so real signals can be delivered to it."""
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        path for path in (PACKAGE_ROOT, env.get("PYTHONPATH")) if path
    )
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from so101_camera_bridge.bridge_node import main; main()",
            "--ros-args",
            *DEAD_URL_ARGS,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if not line:
            break
        if READY_LOG in line:
            time.sleep(SPIN_SETTLE_SECONDS)
            return process
    process.kill()
    pytest.fail(f"bridge never reached {READY_LOG!r}: {process.communicate()[0]}")


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_regression_signal_during_spin_exits_zero(stop_signal) -> None:
    """Regression: an ordinary stop signal made main() exit nonzero with a traceback.

    rclpy handles both SIGINT and SIGTERM by shutting the context down, so spin
    raises ExternalShutdownException and the unguarded rclpy.shutdown() in the
    `finally` then raised RCLError on top of it. Ctrl-C and a systemd stop are
    both normal, so the process must exit 0.
    """
    process = _spawn_bridge()
    process.send_signal(stop_signal)
    output = process.communicate(timeout=30)[0]
    assert "KeyError" not in output, f"teardown walked the wrong list:\n{output}"
    assert (
        "rcl_shutdown already called" not in output
    ), f"shutdown ran twice on an already-stopped context:\n{output}"
    if process.returncode != 0:
        assert UPSTREAM_WAIT_SET_RACE in output, (
            f"{stop_signal.name} is a clean stop but exited "
            f"{process.returncode}:\n{output}"
        )
