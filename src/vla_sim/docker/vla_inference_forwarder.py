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

"""Relay inference_server's port to the QNN NPU worker's socket; supervise the worker.

Why the server is split on a QNN host: README.md, "Serve on the IQ10 NPU".
Standard library only, so the relay never loads torch.
"""

import http.client
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WORKER_SOCKET_ENV = "VLA_QNN_WORKER_SOCKET"
WORKER_SERVICE = "inference_qnn_worker"
LISTEN_PORT = 8973

# Matches MAX_BODY_BYTES in vla_inference_server.py.
MAX_BODY_BYTES = 32 * 1024 * 1024

WORKER_TIMEOUT_SECONDS = 30.0
CLIENT_SOCKET_TIMEOUT_SECONDS = 30.0
WORKER_STARTUP_GRACE_SECONDS = 300.0
# A reloading worker closes its socket, so a worker that answered before reads
# as `loading`, not `error`, for this long after its last answer.
WORKER_RELOAD_GRACE_SECONDS = 180.0

# The polls also keep the worker alive: it exits after RELAY_SILENCE_EXIT_SECONDS
# (vla_inference_server.py) without one, so each poll must stay well inside that.
WORKER_KEEPALIVE_SECONDS = 5.0
WORKER_KEEPALIVE_TIMEOUT_SECONDS = 5.0

# Matches RELAY_ID_HEADER in vla_inference_server.py.
RELAY_ID_HEADER = "X-VLA-Relay-Id"
RELAY_ID = uuid.uuid4().hex

WORKER_START_HINT = (
    f"{WORKER_SERVICE} starts as a dependency of inference_server, which "
    "`moveit_pro run --no-drivers` skips; run without --no-drivers, or start it "
    f"with `docker start moveit_pro-{WORKER_SERVICE}-1` after one full run"
)

REQUEST_HEADERS = ("Authorization", "Content-Type", "Content-Length", "Accept")
RESPONSE_HEADERS = ("Content-Type", "Content-Length")


# Match QNN_WORKER_CHILD_ENV and QNN_WORKER_RESTART_EXIT_CODE in vla_inference_server.py.
WORKER_CHILD_ENV = "VLA_QNN_WORKER_CHILD"
RESTART_EXIT_CODE = 75


def log(message: str) -> None:
    print(f"[vla_inference_forwarder] {message}", flush=True)


def supervise_worker(argv: list[str]) -> int:
    """Run the QNN worker as a child, starting a fresh process on each reload exit.

    FastRPC keys sessions by host PID, so the worker cannot reload by
    re-executing in place. Returns the first other exit code; SIGTERM and
    SIGINT are passed to the child.
    """
    environment = {**os.environ, WORKER_CHILD_ENV: "1"}
    child: subprocess.Popen | None = None

    def forward_signal(signum, _frame):
        if child is not None and child.poll() is None:
            child.send_signal(signum)

    signal.signal(signal.SIGTERM, forward_signal)
    signal.signal(signal.SIGINT, forward_signal)
    while True:
        child = subprocess.Popen([sys.executable, *argv], env=environment)
        code = child.wait()
        if code != RESTART_EXIT_CODE:
            return code
        log("starting a fresh QNN worker process to reload")


class UnixHTTPConnection(http.client.HTTPConnection):
    """An http.client connection to a server listening on a Unix socket."""

    def __init__(self, path: str, timeout: float):
        super().__init__("localhost", timeout=timeout)
        self._socket_path = path

    def connect(self) -> None:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect(self._socket_path)
        except OSError:
            sock.close()
            raise
        self.sock = sock


class WorkerLink:
    """Tracks whether the worker has answered, for the /health state."""

    def __init__(self, socket_path: str, clock=time.monotonic):
        self.socket_path = socket_path
        self._clock = clock
        self._started = clock()
        self._lock = threading.Lock()
        self._last_answer: float | None = None
        self._reachable = False

    def mark_reachable(self) -> None:
        with self._lock:
            if not self._reachable:
                log(f"{WORKER_SERVICE} answered on {self.socket_path}")
            self._reachable = True
            self._last_answer = self._clock()

    def mark_unreachable(self, reason: str) -> None:
        with self._lock:
            if self._reachable:
                log(f"{WORKER_SERVICE} stopped answering ({reason})")
            self._reachable = False

    def unreachable_health(self, reason: str) -> dict:
        """The /health body to report while the worker socket cannot be reached."""
        with self._lock:
            last_answer = self._last_answer
        logs_hint = f"see `docker compose logs {WORKER_SERVICE}`"
        if last_answer is not None:
            if self._clock() - last_answer < WORKER_RELOAD_GRACE_SECONDS:
                return {"status": "loading"}
            return {
                "status": "error",
                "detail": f"the QNN NPU worker ({WORKER_SERVICE}) stopped "
                f"answering on {self.socket_path} ({reason}); {logs_hint}, "
                "then restart with `moveit_pro run --with-inference-server`",
            }
        waited = self._clock() - self._started
        if waited < WORKER_STARTUP_GRACE_SECONDS:
            return {"status": "loading"}
        return {
            "status": "error",
            "detail": f"the QNN NPU worker ({WORKER_SERVICE}) has not opened "
            f"{self.socket_path} after {waited:.0f} s ({reason}); {logs_hint}. "
            f"{WORKER_START_HINT}",
        }

    def forward(
        self,
        method: str,
        path: str,
        headers: dict,
        body: bytes | None,
        timeout: float = WORKER_TIMEOUT_SECONDS,
    ) -> tuple[int, dict, bytes]:
        """Send one request to the worker; raises OSError when it is unreachable.

        ValueError (from http.client) means the request itself cannot be sent,
        such as a path with characters a request line cannot carry.
        """
        connection = UnixHTTPConnection(self.socket_path, timeout)
        try:
            try:
                connection.request(
                    method,
                    path,
                    body=body,
                    headers={**headers, RELAY_ID_HEADER: RELAY_ID},
                )
            except (BrokenPipeError, ConnectionResetError):
                # The worker answers early errors without reading the body, so
                # a large body fails mid-send with the answer already buffered.
                if connection.sock is None:
                    raise
            response = connection.getresponse()
            payload = response.read()
            self.mark_reachable()
            kept = {
                name: response.getheader(name)
                for name in RESPONSE_HEADERS
                if response.getheader(name) is not None
            }
            return response.status, kept, payload
        except http.client.InvalidURL as exc:
            raise ValueError(str(exc)) from exc
        except http.client.HTTPException as exc:
            raise OSError(f"{type(exc).__name__}: {exc}") from exc
        finally:
            connection.close()


def keep_worker_alive(link: WorkerLink, period: float = WORKER_KEEPALIVE_SECONDS):
    """Poll the worker so it knows this relay is running."""
    while True:
        try:
            link.forward(
                "GET", "/health", {}, None, timeout=WORKER_KEEPALIVE_TIMEOUT_SECONDS
            )
        except OSError as exc:
            link.mark_unreachable(f"{type(exc).__name__}: {exc}")
        time.sleep(period)


def make_handler(link: WorkerLink):
    class Handler(BaseHTTPRequestHandler):
        timeout = CLIENT_SOCKET_TIMEOUT_SECONDS

        def _send(self, code: int, headers: dict, body: bytes) -> None:
            try:
                self.send_response(code)
                for name, value in headers.items():
                    if name != "Content-Length":
                        self.send_header(name, value)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                log(f"client disconnected before the {code} response was sent")

        def _send_json(self, code: int, obj: dict) -> None:
            self._send(
                code, {"Content-Type": "application/json"}, json.dumps(obj).encode()
            )

        def _relay(self, method: str) -> None:
            body = None
            if method == "POST":
                try:
                    length = int(self.headers.get("Content-Length", 0))
                except ValueError:
                    self._send_json(400, {"error": "invalid Content-Length"})
                    return
                if length < 0 or length > MAX_BODY_BYTES:
                    self._send_json(
                        413 if length > 0 else 400,
                        {"error": f"request body of {length} bytes is out of range"},
                    )
                    return
                body = self.rfile.read(length)
                if len(body) != length:
                    self._send_json(
                        400,
                        {
                            "error": f"request body ended after {len(body)} of "
                            f"its {length} Content-Length bytes"
                        },
                    )
                    return
            headers = {
                name: self.headers[name]
                for name in REQUEST_HEADERS
                if self.headers.get(name) is not None
            }
            try:
                code, response_headers, payload = link.forward(
                    method, self.path, headers, body
                )
            except ValueError as exc:
                self._send_json(400, {"error": f"bad request: {exc}"})
                return
            except OSError as exc:
                reason = f"{type(exc).__name__}: {exc}"
                link.mark_unreachable(reason)
                health = link.unreachable_health(reason)
                if self.path == "/health":
                    self._send_json(200, health)
                    return
                detail = health.get(
                    "detail",
                    f"the QNN NPU worker ({WORKER_SERVICE}) is starting or "
                    f"reloading ({reason}); try again shortly",
                )
                self._send_json(503, {"error": detail})
                return
            self._send(code, response_headers, payload)

        def do_GET(self):
            self._relay("GET")

        def do_POST(self):
            self._relay("POST")

        def log_message(self, *args):
            pass

    return Handler


def main() -> int:
    socket_path = os.environ.get(WORKER_SOCKET_ENV, "").strip()
    if not socket_path:
        log(f"FATAL: {WORKER_SOCKET_ENV} is not set")
        return 2
    link = WorkerLink(socket_path)
    httpd = ThreadingHTTPServer(("0.0.0.0", LISTEN_PORT), make_handler(link))
    httpd.daemon_threads = True
    threading.Thread(target=keep_worker_alive, args=(link,), daemon=True).start()
    log(
        f"listening on 0.0.0.0:{LISTEN_PORT}; relaying to {WORKER_SERVICE} "
        f"at {socket_path}"
    )
    httpd.serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
