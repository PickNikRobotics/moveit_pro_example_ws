import math
import threading
import time

import numpy as np
import pytest

from so101_camera_bridge.contract import (
    StreamSpec,
    pinhole_projection,
    validate_stream_spec,
)


def test_valid_stream_contract() -> None:
    validate_stream_spec(
        StreamSpec(
            "head",
            "rtsp://robot:8554/head",
            "/camera/image_raw",
            "/camera/camera_info",
            "camera_optical_frame",
        )
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("url", "https://robot/head"),
        ("image_topic", "camera/image_raw"),
        ("camera_info_topic", "/camera/info/"),
        ("frame_id", "/camera_optical_frame"),
    ],
)
def test_invalid_stream_contract(field: str, value: str) -> None:
    values = {
        "name": "head",
        "url": "rtsp://robot:8554/head",
        "image_topic": "/camera/image_raw",
        "camera_info_topic": "/camera/camera_info",
        "frame_id": "camera_optical_frame",
    }
    values[field] = value
    with pytest.raises(ValueError):
        validate_stream_spec(StreamSpec(**values))


def test_pinhole_projection_is_centered_and_finite() -> None:
    k, p = pinhole_projection(640, 480, 74.5)
    assert len(k) == 9
    assert len(p) == 12
    assert k[2] == 319.5
    assert k[5] == 239.5
    assert k[0] == k[4]
    assert all(math.isfinite(value) for value in k + p)


@pytest.mark.parametrize(
    "width,height,fov",
    [(0, 480, 74.5), (640, 0, 74.5), (640, 480, 0.0), (640, 480, 180.0)],
)
def test_invalid_projection(width: int, height: int, fov: float) -> None:
    with pytest.raises(ValueError):
        pinhole_projection(width, height, fov)


def _worker(**overrides):
    from so101_camera_bridge.bridge_node import StreamWorker

    kwargs = {
        "reconnect_delay": 0.05,
        "open_timeout_ms": 100,
        "read_timeout_ms": 100,
        "log": lambda _message: None,
    }
    kwargs.update(overrides)
    return StreamWorker(
        StreamSpec(
            name="probe",
            url="rtsp://127.0.0.1:1/none",
            image_topic="/probe/image_raw",
            camera_info_topic="/probe/camera_info",
            frame_id="probe_frame",
        ),
        **kwargs,
    )


def _decoder_threads() -> list[str]:
    return [t.name for t in threading.enumerate() if t.name.startswith("rtsp-")]


class _SlowCapture:
    """A capture whose open() outlasts the join timeout, then reads nothing."""

    def __init__(self, open_seconds: float) -> None:
        time.sleep(open_seconds)

    def isOpened(self) -> bool:  # noqa: N802 - matches the cv2 API
        return True

    def read(self):
        time.sleep(0.02)
        return False, None

    def release(self) -> None:
        pass


class _OneFrameCapture:
    """Yields a single frame, then behaves like an interrupted stream."""

    def __init__(self) -> None:
        self._sent = False

    def isOpened(self) -> bool:  # noqa: N802 - matches the cv2 API
        return True

    def read(self):
        if self._sent:
            time.sleep(0.02)
            return False, None
        self._sent = True
        return True, np.zeros((4, 4, 3), dtype=np.uint8)

    def release(self) -> None:
        pass


def test_stream_worker_start_and_stop_are_idempotent() -> None:
    """The demand check calls these on every publish tick, so repeated calls
    must not spawn a second decoder or fail on an already-stopped worker."""
    worker = _worker()
    assert not worker.running
    worker.stop()  # stopping a worker that never ran is a no-op
    assert worker.join(), "joining a worker that never ran must not block"
    worker.start()
    assert worker.running
    worker.start()  # must not spawn a second decoder
    assert len(_decoder_threads()) == 1
    worker.stop()
    assert worker.join(timeout=5.0)
    assert not worker.running


def test_stop_does_not_block_the_caller() -> None:
    """stop() runs on the executor thread, so it must not wait on the decoder.

    Regression: joining inside stop() stalled every other node callback for as
    long as the decoder took to notice — measured at 1.4 s mid-connect against
    the real stream, and bounded only by the 2-4 s join timeout.
    """
    worker = _worker()
    worker._open = lambda: _SlowCapture(3.0)  # noqa: SLF001 - seam for the test
    worker.start()
    time.sleep(0.1)
    started = time.perf_counter()
    worker.stop()
    elapsed = time.perf_counter() - started
    assert elapsed < 0.1, f"stop() blocked the caller for {elapsed:.2f}s"
    worker.join(timeout=5.0)


def test_restart_while_winding_down_does_not_spawn_a_second_decoder() -> None:
    """Regression: stop() dropped the thread handle when the join timed out, so
    the next start() cleared the stop flag under the still-live decoder and ran
    a second one beside it — two RTSP connections per camera, growing with every
    subscriber flip, and destroy_node() only ever stopped the newest.
    """
    before = len(_decoder_threads())
    worker = _worker()
    worker._open = lambda: _SlowCapture(3.0)  # noqa: SLF001 - seam for the test
    worker.start()
    time.sleep(0.1)

    worker.stop()
    assert not worker.join(timeout=0.2), "this test needs the join to time out"
    worker.start()  # the demand check does exactly this when a subscriber returns
    assert len(_decoder_threads()) - before == 1, "a second decoder was spawned"

    assert worker.join(timeout=5.0), "the abandoned decoder never exited"
    assert len(_decoder_threads()) == before


def test_restart_drops_the_frame_from_the_previous_run() -> None:
    """A subscriber that arrives after an idle period wants live video, not the
    frame that was on screen when the last one left — a stale frame republished
    with a fresh timestamp is indistinguishable from current data downstream."""
    worker = _worker()
    worker._open = _OneFrameCapture  # noqa: SLF001 - seam for the test
    worker.start()
    deadline = time.monotonic() + 5.0
    while worker.newest_after(-1) is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert worker.newest_after(-1) is not None, "the decoder never produced a frame"
    worker.stop()
    assert worker.join(timeout=5.0)
    assert worker.newest_after(-1) is not None, "a stopped worker keeps its last frame"

    worker._open = lambda: _SlowCapture(0.0)  # noqa: SLF001 - never yields a frame
    worker.start()
    time.sleep(0.1)
    assert worker.newest_after(-1) is None, "the previous run's frame was served"
    worker.stop()
    worker.join(timeout=5.0)
