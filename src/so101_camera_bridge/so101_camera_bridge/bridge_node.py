"""Publish the existing SO-101 MediaMTX RTSP streams as ROS images."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .contract import StreamSpec, pinhole_projection, validate_stream_spec


class StreamWorker:
    """Keep one RTSP decoder alive and retain only its newest frame."""

    def __init__(
        self,
        spec: StreamSpec,
        *,
        reconnect_delay: float,
        open_timeout_ms: int,
        read_timeout_ms: int,
        log: Callable[[str], None],
    ) -> None:
        self.spec = spec
        self._reconnect_delay = reconnect_delay
        self._open_timeout_ms = open_timeout_ms
        self._read_timeout_ms = read_timeout_ms
        self._log = log
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Any | None = None
        self._sequence = 0
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def stopping(self) -> bool:
        """A stop was requested but the decoder thread has not exited yet."""
        return self.running and self._stop.is_set()

    def start(self) -> None:
        """Begin decoding. Idempotent, so the demand check can call it freely.

        A no-op while a previous run is still winding down: that thread still
        owns an RTSP connection and would keep writing into `_latest`, so a
        second decoder alongside it is a leak, not a restart. The next tick
        starts one once the old thread is gone.
        """
        if self._thread is not None:
            if self._thread.is_alive():
                return
            self._thread = None
        # Each run owns its stop event. Sharing one event across runs means a
        # later start() can clear the flag a previous run is still watching and
        # revive a thread that was already abandoned.
        stop = threading.Event()
        with self._lock:
            # Drop a frame retained from a previous run; a subscriber that
            # arrives now wants live video, not whatever was on screen when the
            # last one left.
            self._latest = None
        self._stop = stop
        self._thread = threading.Thread(
            target=self._run,
            args=(stop,),
            name=f"rtsp-{self.spec.name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Ask the decoder to stop. Returns immediately; `join` waits for it.

        The demand check calls this from the node's timer callback, which runs
        on the executor thread. Joining there stalls every other callback for
        as long as the decoder takes to notice — measured at 1.4 s against the
        real stream when the stop lands mid-connect, and the join timeout
        allows up to 4 s at the default `read_timeout_ms`.
        """
        self._stop.set()

    def join(self, timeout: float | None = None) -> bool:
        """Wait for the decoder thread to exit. True once it has."""
        thread = self._thread
        if thread is None:
            return True
        if timeout is None:
            timeout = max(2.0, self._read_timeout_ms / 1000.0 + 1.0)
        thread.join(timeout=timeout)
        if thread.is_alive():
            return False
        self._thread = None
        return True

    def newest_after(self, sequence: int) -> tuple[int, Any] | None:
        with self._lock:
            if self._latest is None or self._sequence <= sequence:
                return None
            return self._sequence, self._latest

    def _open(self) -> Any:
        capture = cv2.VideoCapture()
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self._open_timeout_ms)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self._read_timeout_ms)
        capture.open(self.spec.url, cv2.CAP_FFMPEG)
        return capture

    def _run(self, stop: threading.Event) -> None:
        announced = False
        while not stop.is_set():
            capture = self._open()
            if not capture.isOpened():
                capture.release()
                if not announced:
                    self._log(f"{self.spec.name} RTSP stream is unavailable; retrying")
                    announced = True
                stop.wait(self._reconnect_delay)
                continue
            self._log(f"{self.spec.name} RTSP stream connected")
            announced = False
            while not stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    self._log(f"{self.spec.name} RTSP stream interrupted; reconnecting")
                    break
                if len(frame.shape) != 3 or frame.shape[2] != 3:
                    self._log(
                        f"{self.spec.name} produced a non-BGR frame; reconnecting"
                    )
                    break
                with self._lock:
                    self._latest = frame
                    self._sequence += 1
            capture.release()
            stop.wait(self._reconnect_delay)


class RtspCameraBridge(Node):
    """Read two existing RTSP streams without owning camera hardware."""

    def __init__(self) -> None:
        super().__init__("so101_rtsp_camera_bridge")
        self.declare_parameter(
            "head_url", "rtsp://so101-pi.tail337068.ts.net:8554/head"
        )
        self.declare_parameter(
            "gripper_url",
            "rtsp://so101-pi.tail337068.ts.net:8554/gripper",
        )
        self.declare_parameter("publish_fps", 15.0)
        # CameraInfo used to be derived from a decoded frame, which meant the
        # decoder had to run just to describe the camera. Declaring the geometry
        # lets the info topic stand on its own; the decoder validates it against
        # the first real frame it sees.
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        # How long the image topic must stay unsubscribed before the decoder is
        # torn down. Reconnecting costs 1.4-2.0 s against the real stream, and a
        # subscriber that flips away and back (switching camera panes, one
        # Behavior handing off to the next) reads as zero subscribers for a
        # fraction of a second. Without the hold-off each of those flips costs a
        # full RTSP teardown and reconnect on the upstream camera.
        self.declare_parameter("decoder_linger_seconds", 5.0)
        self.declare_parameter("vertical_fov_degrees", 74.5)
        self.declare_parameter("reconnect_delay_seconds", 1.0)
        self.declare_parameter("open_timeout_ms", 5000)
        self.declare_parameter("read_timeout_ms", 3000)

        specs = [
            StreamSpec(
                name="head",
                url=self.get_parameter("head_url").value,
                image_topic="/so101/cameras/overhead/image_raw",
                camera_info_topic="/so101/cameras/overhead/camera_info",
                frame_id="overhead_cam_optical_frame",
            ),
            StreamSpec(
                name="gripper",
                url=self.get_parameter("gripper_url").value,
                image_topic="/so101/cameras/wrist/image_raw",
                camera_info_topic="/so101/cameras/wrist/camera_info",
                frame_id="right_wrist_cam_optical_frame",
            ),
        ]
        for spec in specs:
            validate_stream_spec(spec)

        publish_fps = float(self.get_parameter("publish_fps").value)
        self._vertical_fov = float(self.get_parameter("vertical_fov_degrees").value)
        reconnect_delay = float(self.get_parameter("reconnect_delay_seconds").value)
        open_timeout_ms = int(self.get_parameter("open_timeout_ms").value)
        read_timeout_ms = int(self.get_parameter("read_timeout_ms").value)
        if publish_fps <= 0.0:
            raise ValueError("publish_fps must be greater than zero")
        if reconnect_delay <= 0.0:
            raise ValueError("reconnect_delay_seconds must be greater than zero")
        if min(open_timeout_ms, read_timeout_ms) <= 0:
            raise ValueError("RTSP timeouts must be greater than zero")
        pinhole_projection(640, 480, self._vertical_fov)

        # Not `self._publishers`: rclpy's Node keeps its own list of publishers
        # under that name, and shadowing it with a dict makes destroy_node()
        # index a dict by integer and raise KeyError while tearing the node down.
        self._camera_publishers = {
            spec.name: (
                self.create_publisher(Image, spec.image_topic, 2),
                self.create_publisher(CameraInfo, spec.camera_info_topic, 2),
            )
            for spec in specs
        }
        self._last_sequences = {spec.name: 0 for spec in specs}
        self._workers = [
            StreamWorker(
                spec,
                reconnect_delay=reconnect_delay,
                open_timeout_ms=open_timeout_ms,
                read_timeout_ms=read_timeout_ms,
                log=self.get_logger().info,
            )
            for spec in specs
        ]
        self._width = int(self.get_parameter("image_width").value)
        self._height = int(self.get_parameter("image_height").value)
        if min(self._width, self._height) <= 0:
            raise ValueError("image_width and image_height must be positive")
        self._decoder_linger = float(self.get_parameter("decoder_linger_seconds").value)
        if not math.isfinite(self._decoder_linger) or self._decoder_linger < 0.0:
            raise ValueError("decoder_linger_seconds must be finite and non-negative")
        # Warned about once per stream rather than per frame.
        self._geometry_warned: set[str] = set()
        # Geometry of the frames actually decoded, once any have been. CameraInfo
        # follows this in preference to the declared parameters so the intrinsics
        # always describe the image published alongside them.
        self._observed_geometry: dict[str, tuple[int, int]] = {}
        # When the image topic first read zero subscribers, per camera.
        self._idle_since: dict[str, Any] = {spec.name: None for spec in specs}

        # Workers are NOT started here. Decoding begins only when something
        # subscribes to the image topic — see _publish_latest. The publishers
        # exist from construction, so the topics are discoverable and the camera
        # panes list them whether or not anything is decoding.
        self._timer = self.create_timer(1.0 / publish_fps, self._publish_latest)
        self.get_logger().info(
            "Publishing read-only SO-101 camera taps on stable ROS topics; "
            "decoding starts on the first image subscriber"
        )

    def _camera_info(self, frame_id: str, stamp, width: int, height: int) -> CameraInfo:
        """Describe the camera. Needs no decoded frame."""
        k, p = pinhole_projection(width, height, self._vertical_fov)
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id
        camera_info.height = height
        camera_info.width = width
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5
        camera_info.k = k
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = p
        return camera_info

    def _publish_latest(self) -> None:
        """Publish CameraInfo always; decode and publish images only on demand.

        The video path no longer runs through this node — an already-encoded
        camera is relayed straight to the browser — so decoding every frame
        unconditionally burned CPU and memory producing images nothing read.
        Subscriber count is the honest signal for whether anyone still wants
        them: a recording session or a perception Behavior subscribing starts
        the decoder, and it stops again once nothing has wanted them for
        `decoder_linger_seconds`.
        """
        now = self.get_clock().now()
        stamp = now.to_msg()
        for worker in self._workers:
            name = worker.spec.name
            image_publisher, info_publisher = self._camera_publishers[name]

            # Cheap, and it keeps calibration available to anything that needs
            # to describe the camera without wanting its pixels. Prefer the
            # geometry of the frames actually decoded: the parameters are only a
            # declaration, and publishing intrinsics for a size the stream does
            # not produce silently corrupts any pixel-to-pose math downstream.
            width, height = self._observed_geometry.get(
                name, (self._width, self._height)
            )
            info_publisher.publish(
                self._camera_info(worker.spec.frame_id, stamp, width, height)
            )

            if image_publisher.get_subscription_count() == 0:
                if worker.running and not worker.stopping:
                    idle_since = self._idle_since[name]
                    if idle_since is None:
                        self._idle_since[name] = now
                    elif (now - idle_since).nanoseconds * 1e-9 >= self._decoder_linger:
                        self.get_logger().info(
                            f"{name}: no image subscriber for "
                            f"{self._decoder_linger:g}s; stopping decoder"
                        )
                        worker.stop()
                        self._last_sequences[name] = 0
                        self._idle_since[name] = None
                continue

            # Someone is subscribed, so cancel any pending stop. A worker that
            # was already asked to stop is left to finish; the next tick starts
            # a fresh one rather than racing the thread that is winding down.
            self._idle_since[name] = None
            if not worker.running:
                self.get_logger().info(
                    f"{name}: image subscriber appeared; starting decoder"
                )
                worker.start()

            latest = worker.newest_after(self._last_sequences[name])
            if latest is None:
                continue
            sequence, frame = latest
            height, width = frame.shape[:2]
            self._observed_geometry[name] = (width, height)

            if (width, height) != (self._width, self._height):
                if name not in self._geometry_warned:
                    self._geometry_warned.add(name)
                    self.get_logger().warning(
                        f"{name}: stream is {width}x{height} but image_width/"
                        f"image_height say {self._width}x{self._height}; "
                        "CameraInfo now follows the stream, but set the "
                        "parameters so it is right before the first frame"
                    )

            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = worker.spec.frame_id
            image.height = height
            image.width = width
            image.encoding = "bgr8"
            image.is_bigendian = 0
            image.step = width * 3
            image.data = frame.tobytes()
            image_publisher.publish(image)
            self._last_sequences[name] = sequence

    def destroy_node(self) -> None:
        # Signal every decoder first, then wait, so teardown costs one timeout
        # rather than one per camera.
        for worker in self._workers:
            worker.stop()
        for worker in self._workers:
            if not worker.join():
                self.get_logger().warning(
                    f"{worker.spec.name}: decoder thread did not exit; it is a "
                    "daemon and will not hold up process shutdown"
                )
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RtspCameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy installs its own SIGINT and SIGTERM handlers, and they shut the
        # context down before the interpreter can raise KeyboardInterrupt. spin
        # therefore reports the stop as ExternalShutdownException on both
        # signals; KeyboardInterrupt only surfaces if those handlers are
        # disabled. Neither is a failure, so neither should exit nonzero.
        pass
    finally:
        node.destroy_node()
        # Guarded: the signal handler already shut the context down, and calling
        # shutdown a second time raises over the top of whatever stopped us.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
