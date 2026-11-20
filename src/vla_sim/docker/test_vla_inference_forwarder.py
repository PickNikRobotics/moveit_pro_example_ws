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

"""Tests for vla_inference_forwarder.py: worker health states and the HTTP relay.

Standard library only, so it runs under the host Python as well as the inference image.
"""

import contextlib
import http.client
import json
import os
import socket
import socketserver
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import MagicMock, patch

import vla_inference_forwarder as forwarder
from vla_inference_forwarder import (
    RELAY_ID,
    RELAY_ID_HEADER,
    WORKER_KEEPALIVE_SECONDS,
    WORKER_KEEPALIVE_TIMEOUT_SECONDS,
    WORKER_RELOAD_GRACE_SECONDS,
    WORKER_STARTUP_GRACE_SECONDS,
    WorkerLink,
    keep_worker_alive,
    main,
    make_handler,
)


class FakeClock:
    """A monotonic clock the test advances by hand."""

    def __init__(self, now: float = 0.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class ThreadingUnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


class StopKeepalive(Exception):
    """Ends keep_worker_alive after its first pass."""


def run_keepalive_once(link) -> MagicMock:
    """Run one pass of keep_worker_alive; returns the replaced sleep mock."""
    fake_time = MagicMock()
    fake_time.sleep.side_effect = StopKeepalive
    with patch.object(forwarder, "time", fake_time):
        with contextlib.suppress(StopKeepalive):
            keep_worker_alive(link)
    return fake_time.sleep


def read_until_close(sock: socket.socket) -> bytes:
    raw = b""
    while chunk := sock.recv(65536):
        raw += chunk
    return raw


class FakeWorker:
    """Stands in for inference_qnn_worker: echoes each request as JSON.

    The status and Content-Type of the next /infer reply are set by the test;
    /health always returns HEALTH_BODY verbatim. With early_reply set, /infer
    answers that (status, body) without reading the request body and closes,
    as the real worker does for 401/413/503.
    relay_ids records (method, path, X-VLA-Relay-Id) for every request.
    """

    HEALTH_BODY = b'{"status": "ready", "from": "worker"}'

    def __init__(self, socket_path: str) -> None:
        self.requests: list = []
        self.relay_ids: list = []
        self.infer_status = 200
        self.infer_content_type = "application/json"
        self.early_reply: tuple[int, bytes] | None = None
        worker = self

        class Handler(BaseHTTPRequestHandler):
            def _reply(self, code: int, content_type: str, body: bytes) -> None:
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                worker.relay_ids.append(
                    ("GET", self.path, self.headers.get(RELAY_ID_HEADER))
                )
                worker.requests.append(("GET", self.path, None))
                self._reply(200, "application/json", FakeWorker.HEALTH_BODY)

            def do_POST(self):
                worker.relay_ids.append(
                    ("POST", self.path, self.headers.get(RELAY_ID_HEADER))
                )
                if worker.early_reply is not None:
                    worker.requests.append(("POST", self.path, None))
                    code, body = worker.early_reply
                    self._reply(code, "application/json", body)
                    return
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                worker.requests.append(("POST", self.path, body))
                echo = {
                    "method": "POST",
                    "path": self.path,
                    "authorization": self.headers.get("Authorization"),
                    "body": json.loads(body) if body else None,
                }
                self._reply(
                    worker.infer_status,
                    worker.infer_content_type,
                    json.dumps(echo).encode(),
                )

            def log_message(self, *args):
                pass

        self._server = ThreadingUnixServer(socket_path, Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            kwargs={"poll_interval": 0.05},
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()


class RelayTestCase(unittest.TestCase):
    """Runs the relay's handler on a loopback TCP port in front of a link."""

    def setUp(self) -> None:
        # The relay logs each worker answer; keep the test output quiet.
        quiet = patch.object(forwarder, "log")
        quiet.start()
        self.addCleanup(quiet.stop)

    def _start_relay(
        self, link: WorkerLink, client_timeout: float | None = None
    ) -> ThreadingHTTPServer:
        handler = make_handler(link)
        if client_timeout is not None:
            handler.timeout = client_timeout
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        httpd.daemon_threads = True
        threading.Thread(
            target=httpd.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
        ).start()
        self.addCleanup(httpd.server_close)
        self.addCleanup(httpd.shutdown)
        return httpd

    def _connect(self, httpd: ThreadingHTTPServer):
        connection = http.client.HTTPConnection(
            "127.0.0.1", httpd.server_address[1], timeout=10
        )
        self.addCleanup(connection.close)
        return connection

    def _request(self, connection, method, path, body=None, headers=None):
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        payload = response.read()
        return response, payload


class TestWorkerLinkHealth(unittest.TestCase):
    """unreachable_health: loading through the grace windows, then error."""

    def setUp(self) -> None:
        quiet = patch.object(forwarder, "log")
        quiet.start()
        self.addCleanup(quiet.stop)
        self.clock = FakeClock(1000.0)
        self.link = WorkerLink("/nonexistent/worker.sock", clock=self.clock)

    def test_loading_before_startup_grace_elapses(self) -> None:
        """A worker that never answered reads as loading until the grace is over."""
        self.clock.now = 1000.0 + WORKER_STARTUP_GRACE_SECONDS - 1
        self.assertEqual(
            self.link.unreachable_health("FileNotFoundError"), {"status": "loading"}
        )

    def test_error_after_startup_grace_names_the_worker(self) -> None:
        """Past the startup grace the body is an error that names the worker service."""
        self.clock.now = 1000.0 + WORKER_STARTUP_GRACE_SECONDS
        health = self.link.unreachable_health("FileNotFoundError")
        self.assertEqual(health["status"], "error")
        self.assertIn("inference_qnn_worker", health["detail"])
        self.assertIn("/nonexistent/worker.sock", health["detail"])
        self.assertIn("has not opened", health["detail"])
        self.assertIn("--no-drivers", health["detail"])

    def test_answer_then_loading_within_reload_grace(self) -> None:
        """After one good answer, a socket gap reads as loading (serving-YAML reload)."""
        self.link.mark_reachable()
        self.clock.now += WORKER_RELOAD_GRACE_SECONDS - 1
        self.assertEqual(
            self.link.unreachable_health("ConnectionRefusedError"),
            {"status": "loading"},
        )

    def test_answer_then_error_after_reload_grace(self) -> None:
        """A worker silent past the reload grace is an error that names the worker."""
        self.link.mark_reachable()
        self.clock.now += WORKER_RELOAD_GRACE_SECONDS
        health = self.link.unreachable_health("ConnectionRefusedError")
        self.assertEqual(health["status"], "error")
        self.assertIn("inference_qnn_worker", health["detail"])
        self.assertIn("stopped answering", health["detail"])

    def test_reload_grace_is_measured_from_the_last_answer(self) -> None:
        """Each answer restarts the reload window, so a busy worker never reads as stopped."""
        self.link.mark_reachable()
        self.clock.now += WORKER_RELOAD_GRACE_SECONDS - 1
        self.link.mark_reachable()
        self.clock.now += WORKER_RELOAD_GRACE_SECONDS - 1
        self.assertEqual(
            self.link.unreachable_health("ConnectionRefusedError"),
            {"status": "loading"},
        )


class TestRelayWithWorker(RelayTestCase):
    """The relay in front of a live fake worker on a Unix socket."""

    def setUp(self) -> None:
        super().setUp()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.socket_path = os.path.join(self._tmp.name, "worker.sock")
        self.worker = FakeWorker(self.socket_path)
        self.addCleanup(self.worker.close)
        self.link = WorkerLink(self.socket_path, clock=FakeClock())
        self.relay = self._start_relay(self.link)
        self.conn = self._connect(self.relay)

    def test_get_health_is_returned_verbatim(self) -> None:
        """GET /health passes the worker's body and status through unchanged."""
        response, payload = self._request(self.conn, "GET", "/health")
        self.assertEqual(response.status, 200)
        self.assertEqual(payload, FakeWorker.HEALTH_BODY)
        self.assertEqual(response.getheader("Content-Type"), "application/json")
        self.assertEqual(self.worker.requests[-1][:2], ("GET", "/health"))

    def test_post_infer_forwards_body_and_authorization(self) -> None:
        """POST /infer reaches the worker with its body and bearer token intact."""
        body = json.dumps({"state": [0.1, 0.2], "task": "stack"}).encode()
        response, payload = self._request(
            self.conn,
            "POST",
            "/infer",
            body=body,
            headers={
                "Authorization": "Bearer secret-key",
                "Content-Type": "application/json",
            },
        )
        self.assertEqual(response.status, 200)
        echo = json.loads(payload)
        self.assertEqual(echo["path"], "/infer")
        self.assertEqual(echo["authorization"], "Bearer secret-key")
        self.assertEqual(echo["body"], {"state": [0.1, 0.2], "task": "stack"})
        self.assertEqual(self.worker.requests[-1][2], body)

    def test_worker_error_statuses_pass_through(self) -> None:
        """Non-200 statuses and their Content-Type come back to the client unchanged."""
        for status in (400, 401, 500):
            with self.subTest(status=status):
                self.worker.infer_status = status
                self.worker.infer_content_type = "application/json; charset=utf-8"
                response, payload = self._request(
                    self.conn,
                    "POST",
                    "/infer",
                    body=b"{}",
                    headers={"Authorization": "Bearer x"},
                )
                self.assertEqual(response.status, status)
                self.assertEqual(
                    response.getheader("Content-Type"),
                    "application/json; charset=utf-8",
                )
                self.assertEqual(json.loads(payload)["path"], "/infer")

    def test_body_over_limit_is_refused_locally(self) -> None:
        """A declared body above MAX_BODY_BYTES gets 413 and never reaches the worker."""
        with patch.object(forwarder, "MAX_BODY_BYTES", 16):
            # Only the header is sent: the relay refuses on the declared length.
            response, payload = self._request(
                self.conn,
                "POST",
                "/infer",
                headers={"Content-Length": "17", "Authorization": "Bearer x"},
            )
        self.assertEqual(response.status, 413)
        self.assertIn("error", json.loads(payload))
        self.assertEqual(self.worker.requests, [])

    def test_invalid_or_negative_content_length_is_400(self) -> None:
        """A non-numeric or negative Content-Length is rejected before the worker."""
        for value in ("abc", "-1"):
            with self.subTest(content_length=value):
                response, payload = self._request(
                    self.conn,
                    "POST",
                    "/infer",
                    headers={"Content-Length": value},
                )
                self.assertEqual(response.status, 400)
                self.assertIn("error", json.loads(payload))
        self.assertEqual(self.worker.requests, [])

    def test_early_worker_answer_is_relayed_not_503(self) -> None:
        """A worker that answers 401 without reading a large body has that answer relayed.

        Regression: the send of the ~2 MB body failed with BrokenPipeError once
        the worker closed, and the relay answered 503 and dropped the 401.
        """
        self.worker.early_reply = (401, b'{"error": "unauthorized", "from": "worker"}')
        body = b'"' + b"x" * (2 * 1024 * 1024) + b'"'
        response, payload = self._request(
            self.conn,
            "POST",
            "/infer",
            body=body,
            headers={
                "Authorization": "Bearer wrong",
                "Content-Type": "application/json",
            },
        )
        self.assertEqual(response.status, 401)
        self.assertEqual(
            json.loads(payload), {"error": "unauthorized", "from": "worker"}
        )

    def test_short_body_is_400_and_never_reaches_worker(self) -> None:
        """Fewer body bytes than Content-Length, then EOF, is 400 and is not forwarded."""
        relay = self._start_relay(self.link, client_timeout=2)
        with socket.create_connection(
            ("127.0.0.1", relay.server_address[1]), timeout=10
        ) as sock:
            sock.sendall(
                b"POST /infer HTTP/1.0\r\nContent-Length: 1000\r\n"
                b"Authorization: Bearer x\r\n\r\n0123456789"
            )
            sock.shutdown(socket.SHUT_WR)
            raw = read_until_close(sock)
        head, _, payload = raw.partition(b"\r\n\r\n")
        self.assertTrue(head.startswith(b"HTTP/1.0 400"), head)
        self.assertIn("error", json.loads(payload))
        self.assertEqual(self.worker.requests, [])

    def test_non_ascii_request_path_is_400_and_relay_survives(self) -> None:
        """A raw non-ASCII request line gets 400 and the relay keeps serving."""
        with socket.create_connection(
            ("127.0.0.1", self.relay.server_address[1]), timeout=10
        ) as sock:
            sock.sendall(b"GET /h\xc3\xa9alth HTTP/1.0\r\n\r\n")
            raw = read_until_close(sock)
        self.assertTrue(raw.startswith(b"HTTP/1.0 400"), raw[:80])
        self.assertIn("error", json.loads(raw.partition(b"\r\n\r\n")[2]))
        self.assertEqual(self.worker.requests, [])

        response, payload = self._request(self._connect(self.relay), "GET", "/health")
        self.assertEqual(response.status, 200)
        self.assertEqual(payload, FakeWorker.HEALTH_BODY)

    def test_every_forwarded_request_carries_the_relay_id(self) -> None:
        """Relayed requests and the keepalive poll all name this relay process."""
        self._request(self.conn, "GET", "/health")
        self._request(
            self.conn,
            "POST",
            "/infer",
            body=b"{}",
            headers={"Authorization": "Bearer x"},
        )
        run_keepalive_once(self.link)
        self.assertEqual(
            self.worker.relay_ids,
            [
                ("GET", "/health", RELAY_ID),
                ("POST", "/infer", RELAY_ID),
                ("GET", "/health", RELAY_ID),
            ],
        )

    def test_body_exactly_at_the_limit_is_relayed(self) -> None:
        """A body of exactly MAX_BODY_BYTES reaches the worker unchanged."""
        body = b'"' + b"x" * 14 + b'"'
        self.assertEqual(len(body), 16)
        with patch.object(forwarder, "MAX_BODY_BYTES", len(body)):
            response, payload = self._request(
                self.conn,
                "POST",
                "/infer",
                body=body,
                headers={"Authorization": "Bearer x"},
            )
        self.assertEqual(response.status, 200)
        self.assertEqual(json.loads(payload)["body"], "x" * 14)
        self.assertEqual(self.worker.requests[-1][2], body)

    def test_body_one_byte_over_the_limit_is_413(self) -> None:
        """MAX_BODY_BYTES + 1 is refused with 413 and never reaches the worker."""
        with patch.object(forwarder, "MAX_BODY_BYTES", 16):
            response, payload = self._request(
                self.conn,
                "POST",
                "/infer",
                body=b"x" * 17,
                headers={"Authorization": "Bearer x"},
            )
        self.assertEqual(response.status, 413)
        self.assertIn("error", json.loads(payload))
        self.assertEqual(self.worker.requests, [])


class TestRelayWithoutWorker(RelayTestCase):
    """The relay while the worker socket does not exist."""

    def setUp(self) -> None:
        super().setUp()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.socket_path = os.path.join(self._tmp.name, "absent.sock")
        self.clock = FakeClock(0.0)
        self.link = WorkerLink(self.socket_path, clock=self.clock)
        self.relay = self._start_relay(self.link)
        self.conn = self._connect(self.relay)

    def test_health_is_loading_during_startup(self) -> None:
        """/health answers 200 loading while the worker has not yet opened its socket."""
        response, payload = self._request(self.conn, "GET", "/health")
        self.assertEqual(response.status, 200)
        self.assertEqual(json.loads(payload), {"status": "loading"})

    def test_infer_is_503_with_json_error(self) -> None:
        """/infer fails fast with 503 and a JSON error when the worker is absent."""
        response, payload = self._request(
            self.conn,
            "POST",
            "/infer",
            body=b"{}",
            headers={"Authorization": "Bearer x"},
        )
        self.assertEqual(response.status, 503)
        self.assertIn("error", json.loads(payload))

    def test_health_turns_to_error_after_startup_grace(self) -> None:
        """Once the startup grace is over, /health reports error naming the worker."""
        self.clock.now = WORKER_STARTUP_GRACE_SECONDS + 1
        response, payload = self._request(self.conn, "GET", "/health")
        self.assertEqual(response.status, 200)
        body = json.loads(payload)
        self.assertEqual(body["status"], "error")
        self.assertIn("inference_qnn_worker", body["detail"])


class TestMain(unittest.TestCase):
    """main() refuses to start without a worker socket path."""

    def test_unset_or_blank_socket_env_returns_nonzero_without_binding(self) -> None:
        """Unset, empty, or whitespace-only VLA_QNN_WORKER_SOCKET exits 2 and never binds."""
        for value in (None, "", "   "):
            with self.subTest(value=value):
                with patch.dict(os.environ), patch.object(
                    forwarder, "ThreadingHTTPServer"
                ) as server_class, patch.object(forwarder, "log"):
                    os.environ.pop(forwarder.WORKER_SOCKET_ENV, None)
                    if value is not None:
                        os.environ[forwarder.WORKER_SOCKET_ENV] = value
                    result = main()
                self.assertEqual(result, 2)
                server_class.assert_not_called()

    def test_socket_env_binds_the_relay_port(self) -> None:
        """With a socket path set, main() serves on LISTEN_PORT and relays to that path."""
        fake_server = MagicMock()
        # The keepalive thread is stubbed out: a real one would poll the missing
        # socket every few seconds for the rest of the test run.
        with patch.dict(
            os.environ, {forwarder.WORKER_SOCKET_ENV: "/tmp/w.sock"}
        ), patch.object(
            forwarder, "ThreadingHTTPServer", return_value=fake_server
        ) as server_class, patch.object(
            forwarder, "log"
        ), patch.object(
            forwarder.threading, "Thread"
        ) as thread_class:
            result = main()
        self.assertEqual(result, 0)
        server_class.assert_called_once()
        self.assertEqual(
            server_class.call_args.args[0], ("0.0.0.0", forwarder.LISTEN_PORT)
        )
        fake_server.serve_forever.assert_called_once()
        thread_class.assert_called_once()
        self.assertIs(thread_class.call_args.kwargs["target"], keep_worker_alive)
        thread_class.return_value.start.assert_called_once()


class TestKeepWorkerAlive(unittest.TestCase):
    """keep_worker_alive: the poll's bound, and how a failed poll is reported."""

    def setUp(self) -> None:
        quiet = patch.object(forwarder, "log")
        quiet.start()
        self.addCleanup(quiet.stop)

    def test_poll_is_bounded_and_paced(self) -> None:
        """Each poll uses WORKER_KEEPALIVE_TIMEOUT_SECONDS, then waits WORKER_KEEPALIVE_SECONDS."""
        link = MagicMock()
        link.forward.return_value = (200, {}, b"{}")
        sleep = run_keepalive_once(link)
        link.forward.assert_called_once_with(
            "GET", "/health", {}, None, timeout=WORKER_KEEPALIVE_TIMEOUT_SECONDS
        )
        link.mark_unreachable.assert_not_called()
        sleep.assert_called_once_with(WORKER_KEEPALIVE_SECONDS)

    def test_oserror_marks_the_worker_unreachable(self) -> None:
        """A failed poll (OSError) calls mark_unreachable with the reason."""
        link = MagicMock()
        link.forward.side_effect = FileNotFoundError(2, "No such file or directory")
        run_keepalive_once(link)
        link.mark_unreachable.assert_called_once()
        self.assertIn("FileNotFoundError", link.mark_unreachable.call_args.args[0])


class TestSuperviseWorker(unittest.TestCase):
    """The worker supervisor restarts its child only for a reload exit."""

    CHILD = """
import os, sys
runs_file = sys.argv[1]
with open(runs_file, "a") as f:
    f.write(os.environ.get("VLA_QNN_WORKER_CHILD", "") + "\\n")
runs = len(open(runs_file).read().splitlines())
sys.exit(int(sys.argv[2 + runs - 1]) if 2 + runs - 1 < len(sys.argv) else 0)
"""

    def supervise(self, *exit_codes: int) -> tuple[int, list[str]]:
        with tempfile.TemporaryDirectory() as tmp:
            script = os.path.join(tmp, "child.py")
            runs_file = os.path.join(tmp, "runs.txt")
            with open(script, "w") as f:
                f.write(self.CHILD)
            argv = [script, runs_file, *(str(code) for code in exit_codes)]
            with patch("signal.signal"):
                code = forwarder.supervise_worker(argv)
            with open(runs_file) as f:
                return code, f.read().splitlines()

    def test_restarts_a_fresh_child_for_each_reload_exit(self) -> None:
        code, runs = self.supervise(
            forwarder.RESTART_EXIT_CODE, forwarder.RESTART_EXIT_CODE, 3
        )
        self.assertEqual(code, 3)
        self.assertEqual(runs, ["1", "1", "1"])

    def test_a_clean_exit_ends_supervision(self) -> None:
        code, runs = self.supervise(0)
        self.assertEqual(code, 0)
        self.assertEqual(len(runs), 1)

    def test_restart_exit_code_matches_the_server(self) -> None:
        # vla_inference_server.py imports torch, so compare the source text.
        server_source = open(
            os.path.join(os.path.dirname(__file__), "vla_inference_server.py")
        ).read()
        self.assertIn(
            f"QNN_WORKER_RESTART_EXIT_CODE = {forwarder.RESTART_EXIT_CODE}",
            server_source,
        )
        self.assertIn(
            f'QNN_WORKER_CHILD_ENV = "{forwarder.WORKER_CHILD_ENV}"', server_source
        )


if __name__ == "__main__":
    unittest.main()
