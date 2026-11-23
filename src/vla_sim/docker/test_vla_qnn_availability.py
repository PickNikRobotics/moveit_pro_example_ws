#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Offline availability regressions with fake native APIs and real HTTP/tokenizer IO."""

import json
import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from lerobot.policies.rtc.configuration_rtc import RTCConfig

import test_vla_qnn_backend as qnn_fakes
from vla_inference_server import (
    INFER_ABANDONED_SECONDS,
    ServerState,
    load_policy,
    run_inference,
    watch_runtime_inputs,
)
from vla_qnn_policy import QnnPolicy


@pytest.fixture
def bundle():
    """Keep synthetic bundle/session ownership isolated to one test."""
    fixture = qnn_fakes.QnnFixture()
    fixture.setUp()
    try:
        yield fixture
    finally:
        fixture.doCleanups()


def test_http_fixture_joins_active_request_worker_after_client_timeout(bundle) -> None:
    """Fixture teardown must wait for request work before releasing native doubles."""
    runner, runtime = bundle.runner()
    state = bundle.ready_state(runner)
    model = runtime.models["action_expert"]
    native_run = model.run
    finish = threading.Event()
    entered = threading.Event()
    request_joined = threading.Event()
    request_workers = []
    original_join = threading.Thread.join

    def execute(feed):
        if not request_workers:
            request_workers.append(threading.current_thread())
            entered.set()
            assert finish.wait(5), "bounded request worker was never released"
        return native_run(feed)

    def join(worker, *args, **kwargs):
        if worker in request_workers:
            request_joined.set()
            finish.set()
        return original_join(worker, *args, **kwargs)

    try:
        with patch.object(model, "run", side_effect=execute), patch.object(
            threading.Thread, "join", join
        ):
            with qnn_fakes.http_connection(state) as connection:
                connection.request(
                    "POST",
                    "/infer",
                    body=json.dumps(bundle.payload()),
                    headers={"Authorization": "Bearer qnn-test-key"},
                )
                assert entered.wait(5), "request did not reach the native double"
                connection.sock.settimeout(0.05)
                with pytest.raises(TimeoutError):
                    connection.getresponse()
            assert request_joined.is_set(), "fixture did not join its request worker"
            assert not request_workers[0].is_alive()
            assert state.inflight_infers == 0
    finally:
        finish.set()
        for worker in request_workers:
            original_join(worker, 5)
            assert not worker.is_alive()


def zero_extent(shape):
    """Replace one camera with a decodable image whose resize rounds to zero."""
    image = qnn_fakes.encode_bgr_jpeg_b64(np.zeros((*shape, 3), dtype=np.uint8))
    return lambda payload: payload["images"].update(overview=image)


def float32_overflow(payload):
    """Finite carryover whose host RTC correction overflows Float32."""
    payload.update(
        prev_chunk_left_over=[[3e38] * 8] * 50, inference_delay=1, guidance_horizon=8
    )


@pytest.mark.parametrize(
    "change, message, reaches_native",
    [
        (zero_extent((1, 1000)), "resize", False),
        (zero_extent((1000, 1)), "resize", False),
        (float32_overflow, "RTC carryover", True),
    ],
)
def test_host_request_error_is_400_and_next_request_succeeds(
    bundle, change, message, reaches_native
) -> None:
    """No invented magnitude limit, and a host-side refusal must not poison the runner."""
    runner, runtime = bundle.runner()
    state = bundle.ready_state(runner)
    payload = bundle.payload()
    change(payload)
    with qnn_fakes.http_connection(state) as connection:
        code, error = qnn_fakes.call(connection, "/infer", payload)
        assert code == 400, error
        assert message in error["error"]
        native_calls = sum(len(model.calls) for model in runtime.models.values())
        assert (native_calls > 0) is reaches_native
        assert not (runner.failed or state.qnn_restart_required or runtime.closed)
        code, response = qnn_fakes.call(connection, "/infer", bundle.payload())
        assert code == 200, response
        assert np.shape(response["action_chunk_raw"]) == (50, 8)
        assert np.isfinite(response["action_chunk_raw"]).all()


@pytest.mark.parametrize("carryover", [None, [[0.0] * 8] * 10])
def test_nonfinite_native_output_is_still_fatal_with_or_without_rtc(
    bundle, carryover
) -> None:
    """The caller-error fix must not hide a native output failure or retry its cleanup."""
    runner, runtime = bundle.runner()
    state = bundle.ready_state(runner)
    model = runtime.models["action_expert"]
    native_run = model.run

    def nonfinite(feed):
        native_run(feed)  # NativeModel.run refuses nonfinite outputs like this.
        raise RuntimeError("Native QNN returned nonfinite outputs; discard the chunk")

    runtime.close = MagicMock(wraps=runtime.close)
    payload = bundle.payload()
    payload["prev_chunk_left_over"] = carryover
    with patch.object(model, "run", side_effect=nonfinite), qnn_fakes.http_connection(
        state
    ) as connection:
        code, error = qnn_fakes.call(connection, "/infer", payload)
        assert code == 500, error
        assert "nonfinite" in error["error"]
        assert runner.failed
        assert state.qnn_restart_required
        runtime.close.assert_called_once()
        code, error = qnn_fakes.call(connection, "/infer", bundle.payload())
        assert code == 500, error
        assert "restart the inference server explicitly" in error["error"]
        assert len(model.calls) == 1
        runtime.close.assert_called_once()


def test_effective_qnn_still_rejects_invalid_bundle_and_honors_path_override(
    bundle,
) -> None:
    """Only an unused setting is ignored; selected QNN still requires its local bundle."""
    args = bundle.parsed_args(qnn_bundle=17)
    assert "qnn_bundle" in args.config_error
    state = ServerState()
    load_policy(state, args)
    assert state.status == "error"
    assert "qnn_bundle" in state.detail
    args = bundle.parsed_args(("--qnn-bundle", str(bundle.bundle)), qnn_bundle=17)
    assert args.config_error == ""
    assert args.qnn_bundle == str(bundle.bundle)


class ObservedLock:
    """A real mutex that exposes exactly when cleanup begins waiting for inference."""

    def __init__(self, lock=None):
        self.lock = threading.Lock() if lock is None else lock
        self.cleanup_waiting = threading.Event()

    def acquire(self, *, timeout):
        self.cleanup_waiting.set()
        return self.lock.acquire(timeout=timeout)

    def release(self):
        self.lock.release()

    def __enter__(self):
        self.lock.acquire()
        return self

    def __exit__(self, *_args):
        self.lock.release()


@pytest.mark.parametrize("registration_delay", [0.0, 1.25])
@pytest.mark.parametrize(
    "native_failure, cleanup_failure", [(True, False), (True, True), (False, False)]
)
def test_reload_cleanup_observes_native_failure_but_not_healthy_cancellation(
    bundle, native_failure, cleanup_failure, registration_delay
) -> None:
    """Force the real runner-lock/handler-publication gap, not a single-caller surrogate."""
    entered = threading.Event()
    release_native = threading.Event()
    publish_failure = threading.Event()

    class DelayedPublicationState(ServerState):
        """Schedule registration and failure publication independently of cleanup."""

        def begin_infer(self):
            time.sleep(registration_delay)
            return super().begin_infer()

        def fail_qnn(self, detail, runner=None):
            if threading.current_thread().name == "test-infer":
                assert publish_failure.wait(5), "failure publication was never released"
            super().fail_qnn(detail, runner)

    runner, runtime = bundle.runner()
    runner.lock = ObservedLock()
    runtime.close = MagicMock(
        wraps=runtime.close,
        side_effect=(
            RuntimeError("contextFree failed status=23") if cleanup_failure else None
        ),
    )
    state = DelayedPublicationState()
    state.runner, state.status, state.fps = runner, "ready", 10.0
    model = runtime.models["action_expert"]
    native_run = model.run
    results = []

    def execute(feed):
        entered.set()
        assert release_native.wait(5), "native fixture was never released"
        if native_failure:
            raise RuntimeError("HTP execute failed status=17")
        return native_run(feed)

    def infer():
        assert state.begin_infer()
        try:
            run_inference(state, bundle.payload())
        except Exception as exc:
            results.append(exc)
        finally:
            state.end_infer()

    restart = MagicMock()
    watcher_errors = []

    def watch():
        try:
            bundle.watch(state, restart, clock=lambda: expired)
        except Exception as exc:
            watcher_errors.append(exc)

    worker = threading.Thread(target=infer, name="test-infer")
    watcher = threading.Thread(target=watch, name="test-watcher")
    # Cleanup must succeed here, so its lock wait is bounded by release_native's
    # own wait rather than the production one-second timeout.
    with patch.object(model, "run", side_effect=execute), patch(
        "vla_inference_server.QNN_RELEASE_TIMEOUT_SECONDS", 5.0
    ):
        try:
            worker.start()
            assert entered.wait(5)
            with state.activity_lock:
                expired = min(state.infer_starts.values()) + INFER_ABANDONED_SECONDS + 1
            watcher.start()
            assert runner.lock.cleanup_waiting.wait(5)
            release_native.set()
            watcher.join(5)
            assert not watcher.is_alive(), "watcher did not finish one cleanup attempt"
        finally:
            release_native.set()
            publish_failure.set()
            worker.join(5)
            if watcher.ident is not None:
                watcher.join(5)
    assert not worker.is_alive()
    assert not watcher_errors
    runtime.close.assert_called_once()
    assert runner.release_complete is not cleanup_failure
    assert len(results) == 1
    if native_failure:
        restart.assert_not_called()
        assert runner.failed
        assert state.qnn_restart_required
        assert state.status == "error"
        assert "HTP execute failed status=17" in state.detail
        if cleanup_failure:
            assert "contextFree failed status=23" in state.detail
            assert state.cleanup_runner is runner
    else:
        restart.assert_called_once()
        assert runtime.closed
        assert not runner.failed
        assert not state.qnn_restart_required
        assert "reload" in str(results[0]).lower()


def test_healthy_load_finishing_during_abandoned_reload_is_not_native_failure(
    bundle,
) -> None:
    """An expired load finishing while cleanup waits can be cancelled without poisoning reload."""
    state = ServerState()
    runtime = qnn_fakes.FakeRuntime(bundle.ports)
    native_load = runtime.load
    loading = threading.Event()
    release_load = threading.Event()
    locks = []
    watcher_errors = []

    def load(binary, abi, device, **kwargs):
        if not loading.is_set():
            runner = state.cleanup_runner
            runner.lock = ObservedLock(runner.lock)
            locks.append(runner.lock)
            loading.set()
            assert release_load.wait(5), "load was never released"
        return native_load(binary, abi, device, **kwargs)

    runtime.load = load
    runtime.close = MagicMock(wraps=runtime.close)
    args = bundle.parsed_args()
    restart = MagicMock()

    def watch():
        try:
            bundle.watch(state, restart, clock=lambda: state.native_load_started + 121)
        except Exception as exc:
            watcher_errors.append(exc)

    loader = threading.Thread(target=load_policy, args=(state, args))
    watcher = threading.Thread(target=watch)
    # Cleanup must succeed here, so its lock wait is bounded by release_load's
    # own wait rather than the production one-second timeout.
    with bundle.native(runtime), patch(
        "vla_inference_server.QNN_RELEASE_TIMEOUT_SECONDS", 5.0
    ):
        try:
            loader.start()
            assert loading.wait(5)
            watcher.start()
            assert locks[0].cleanup_waiting.wait(5)
            release_load.set()
            watcher.join(5)
            loader.join(5)
        finally:
            release_load.set()
            loader.join(5)
            if watcher.ident is not None:
                watcher.join(5)
    assert not loader.is_alive() and not watcher.is_alive()
    assert not watcher_errors
    runtime.close.assert_called_once()
    restart.assert_called_once()
    assert state.runner is None
    assert not state.cleanup_runner.failed
    assert not state.qnn_restart_required, state.detail
    assert state.status == "loading"
    assert state.cleanup_runner.release_complete


def test_latched_failure_during_config_debounce_does_not_restore_reload_pending(
    bundle,
) -> None:
    """The watcher cannot overwrite failure publication with a misleading pending reload."""
    state = ServerState()
    serving = bundle.root / "watched.yaml"
    serving.write_text("backend: qnn\n")
    waits = 0

    def wait(_seconds):
        nonlocal waits
        waits += 1
        serving.write_text("backend: pytorch\n")
        if waits == 2:
            state.fail_qnn("HTP execute failed")
        assert waits <= 2, "latched failure triggered another poll"

    restart = MagicMock()
    watch_runtime_inputs([str(serving)], state=state, wait=wait, restart=restart)
    assert state.status == "error"
    assert not state.reload_pending
    restart.assert_not_called()


@pytest.mark.parametrize(
    "endpoint, status_key", [("/health", "status"), ("/status", "state")]
)
def test_health_and_status_use_one_snapshot_during_failure_publication(
    bundle, endpoint, status_key
) -> None:
    """Schedule failure at the first status read; HTTP output must remain internally consistent."""
    first_read = threading.Event()
    failure_done = threading.Event()

    class ScheduledState(ServerState):
        """Expose a deterministic scheduler seam without replacing the HTTP implementation."""

        armed = False

        @property
        def status(self):
            value = self._status
            if self.armed:
                self.armed = False
                first_read.set()
                if not self.activity_lock.locked():
                    assert failure_done.wait(5), "failure writer did not run"
            return value

        @status.setter
        def status(self, value):
            self._status = value

    runner, _runtime = bundle.runner()
    state = ScheduledState()
    state.runner, state.status, state.bearer_key = runner, "ready", "qnn-test-key"

    def fail():
        assert first_read.wait(5)
        state.fail_qnn("HTP execute failed", runner)
        failure_done.set()

    writer = threading.Thread(target=fail)
    writer.start()
    try:
        with qnn_fakes.http_connection(state) as connection:
            state.armed = True
            code, body = qnn_fakes.call(connection, endpoint)
            assert code == 200
    finally:
        first_read.set()
        writer.join(5)
    assert failure_done.is_set()
    assert body[status_key] == "ready", body
    assert body["device"] == "npu:qnn_htp", body
    assert body["int8"] is False
    assert body["qnnPrecision"] == runner.precision
    assert "detail" not in body, body


def test_missing_relative_tokenizer_cannot_substitute_a_hub_repo(
    bundle, monkeypatch
) -> None:
    """org/tokenizer is also a Hub repo id: refuse it before any tokenizer or QAIRT load."""
    bundle.config["tokenizer"] = "org/tokenizer"
    bundle.stage_bundle()
    monkeypatch.chdir(bundle.root)  # A relative bundle path keeps the name relative.
    with patch("vla_qnn_policy.AutoTokenizer.from_pretrained") as load, patch(
        "vla_qnn_runtime.NativeRuntime"
    ) as native, pytest.raises(ValueError, match="existing local directory"):
        QnnPolicy("bundle.json", "test/pi05", "", 8, 10, RTCConfig(enabled=True))
    load.assert_not_called()
    native.assert_not_called()


@pytest.mark.parametrize(
    "info_status, create_status, original",
    [(0, 17, "deviceCreate(0)"), (19, 0, "deviceGetPlatformInfo")],
)
def test_platform_failure_preserves_setup_error_allocation_and_one_free(
    info_status, create_status, original, monkeypatch
) -> None:
    """Partial platform ownership survives dual failure; ordinary cleanup cannot retry its free."""
    monkeypatch.setenv("QNN_HTP_BURST", "0")
    runtime = qnn_fakes.platform_runtime(info_status, create_status, free_status=23)
    with pytest.raises(RuntimeError) as error:
        runtime.device(0)
    assert original in str(error.value)
    assert "deviceFreePlatformInfo" in str(error.value)
    assert "status=23" in str(error.value)
    assert runtime.platform_info
    assert runtime.failed
    runtime.functions["platform_free"].assert_called_once()
    for _ in range(2):
        with pytest.raises(RuntimeError, match="deviceFreePlatformInfo"):
            runtime.close()
    runtime.functions["platform_free"].assert_called_once()
    runtime.functions["backend_free"].assert_not_called()
    assert runtime.platform_info


def test_successful_platform_free_releases_only_its_pointer(monkeypatch) -> None:
    """Normal device setup still frees platform info and keeps the native device until close."""
    monkeypatch.setenv("QNN_HTP_BURST", "0")
    runtime = qnn_fakes.platform_runtime()
    assert runtime.device(0).value == 7
    assert runtime.platform_info is None
    runtime.functions["platform_free"].assert_called_once()
    assert runtime.devices[0].value == 7
    runtime.close()
    runtime.functions["platform_free"].assert_called_once()
    runtime.functions["device_free"].assert_called_once()
    runtime.functions["backend_free"].assert_called_once()
