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

"""HTTPS endpoint and transport contracts for remote inference."""

import importlib.util
import ipaddress
import json
import ssl
import threading
import time
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

import pytest
import requests
import urllib3

_SPEC = importlib.util.spec_from_file_location(
    "remote_adapter",
    Path(__file__).resolve().parents[1] / "script/get_action_chunk_adapter.py",
)
adapter = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(adapter)


@pytest.mark.parametrize(
    "url", ["https://gpu.example/infer", "https://[::1]:8443/infer"]
)
def test_https_endpoint_is_accepted(url: str) -> None:
    """HTTPS selects external inference even on loopback."""
    assert adapter.resolve_infer_url(url) == url


@pytest.mark.parametrize(
    "url",
    [
        "http://gpu.example/infer",
        "https://gpu.example/other",
        "https://gpu.example/infer?x=1",
        "https://gpu.example/infer#x",
        "https://user:secret@gpu.example/infer",
        "https://gpu.example:0/infer",
        "https://gpu.example:99999/infer",
        "https://gpu.example/infer\\x",
        # Padding is stripped, but a control character inside the URL is not.
        "https://gpu.example/in\nfer",
        "https:///infer",
    ],
)
def test_ambiguous_or_insecure_endpoint_is_rejected(url: str) -> None:
    """Configuration cannot redirect observations or credentials through URL ambiguities."""
    with pytest.raises(ValueError, match="infer_url") as error:
        adapter.resolve_infer_url(url)
    # The message is logged, so it must not echo a password from the URL.
    assert "secret" not in str(error.value)


def test_rejected_endpoint_names_the_parts_at_fault() -> None:
    """The rejection shows the host and path it read, so a typo is visible."""
    with pytest.raises(ValueError) as error:
        adapter.resolve_infer_url("https://gpu.example/infr")
    assert "'gpu.example'" in str(error.value)
    assert "'/infr'" in str(error.value)


def test_deadline_adapter_sends_one_total_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A connect/read pair reaches urllib3 as one total, not two separate bounds.

    Passed through as a tuple, the connect bound would apply per resolved
    address and the read bound per `recv`, so neither would cap the call.
    """
    # GIVEN a transport that records what the adapter forwards
    forwarded = {}

    def record(self, request, **kwargs):
        forwarded.update(kwargs)
        return "response"

    monkeypatch.setattr(requests.adapters.HTTPAdapter, "send", record)
    prepared = requests.Request("POST", "https://gpu.example/infer").prepare()

    # WHEN a request is sent with a (connect, read) pair
    result = adapter.DeadlineAdapter().send(prepared, timeout=(2.0, 7.0))

    # THEN the pair arrives as one urllib3 budget covering both
    assert result == "response"
    timeout = forwarded["timeout"]
    assert isinstance(timeout, urllib3.Timeout)
    assert timeout.total == 9.0
    assert timeout.connect_timeout == 2.0


@pytest.fixture
def tls_endpoint(tmp_path: Path, request: pytest.FixtureRequest):
    """Run a real TLS peer with controllable responses and observed requests."""
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "test-inference")])
    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(days=2))
        .not_valid_after(now + timedelta(days=getattr(request, "param", 1)))
        .add_extension(
            x509.SubjectAlternativeName(
                [x509.IPAddress(ipaddress.ip_address("127.0.0.1"))]
            ),
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )
    cert_path, key_path = tmp_path / "ca.pem", tmp_path / "key.pem"
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    key_path.write_bytes(
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        )
    )
    state = SimpleNamespace(
        status=200,
        body=b'{"action_chunk":[[0.1,0.2]],"dt":0.1}',
        requests=[],
        trickle=False,
        closed=threading.Event(),
        extra_headers={},
        stall_after=None,
        connections=0,
        sessions=[],
    )

    class Handler(BaseHTTPRequestHandler):
        # Keep-alive capable, so a client that reuses its connection is not
        # forced into a new handshake by the fixture itself.
        protocol_version = "HTTP/1.1"

        def do_POST(self):
            state.requests.append(
                (
                    self.path,
                    self.headers.get("Authorization"),
                    self.rfile.read(int(self.headers["Content-Length"])),
                )
            )
            self.send_response(state.status)
            self.send_header("Location", "/captured")
            for name, value in state.extra_headers.items():
                self.send_header(name, value)
            if state.trickle or state.stall_after is not None:
                # No length is known up front, so this body ends at close.
                self.send_header("Connection", "close")
                self.close_connection = True
            else:
                self.send_header("Content-Length", str(len(state.body)))
            self.end_headers()
            try:
                if state.stall_after is not None:
                    began = time.monotonic()
                    while time.monotonic() - began < state.stall_after:
                        self.wfile.write(b" ")
                        self.wfile.flush()
                        if state.closed.wait(0.01):
                            break
                    state.closed.wait(120)
                elif state.trickle:
                    while True:
                        self.wfile.write(b" ")
                        self.wfile.flush()
                        if state.closed.wait(0.01):
                            break
                else:
                    self.wfile.write(state.body)
            except OSError:
                state.closed.set()

        def log_message(self, *args):
            pass  # Keep fixture HTTP request logs out of test output.

    class CountingServer(ThreadingHTTPServer):
        """Record every accepted socket, so connection reuse is measurable."""

        def get_request(self):
            connection = super().get_request()
            state.connections += 1
            return connection

    server = CountingServer(("127.0.0.1", 0), Handler)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert_path, key_path)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    state.url = f"https://127.0.0.1:{server.server_port}/infer"
    state.ca_file = str(cert_path)
    try:
        yield state
    finally:
        state.closed.set()
        # Keep-alive leaves a handler thread parked on each open socket, and
        # server_close() waits for those threads.
        for session in state.sessions:
            session.close()
        server.shutdown()
        server.server_close()
        thread.join()


def remote_client(endpoint):
    """Use the adapter's actual transport without creating a ROS service."""
    client = object.__new__(adapter.GetActionChunkAdapter)
    client.infer_url = endpoint.url
    client.http_timeout = 1.0
    client._auth_hint = "AUTH-HINT"
    client._unreachable_hint = "UNREACHABLE-HINT"
    client._session = requests.Session()
    client._session.verify = endpoint.ca_file
    # DeadlineAdapter has to be mounted or these tests measure a transport the
    # node never uses. test_http_timeout_tracks_the_parameter is what checks
    # that the node mounts it.
    client._session.mount("https://", adapter.DeadlineAdapter())
    endpoint.sessions.append(client._session)
    client._session.trust_env = False
    client._session.headers["Accept-Encoding"] = "identity"
    client._session.headers["Authorization"] = "Bearer " + "a" * 64
    return client


def test_verified_tls_sends_only_inference_key(
    tls_endpoint, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Proxy/netrc settings cannot replace the destination or inference credential."""
    monkeypatch.setenv("HTTPS_PROXY", "http://127.0.0.1:1")
    monkeypatch.setenv("ALL_PROXY", "http://127.0.0.1:1")
    netrc = tmp_path / "netrc"
    netrc.write_text("machine 127.0.0.1 login unwanted password unwanted")
    monkeypatch.setenv("NETRC", str(netrc))
    payload = {"state": [0.1, 0.2], "images": {}}
    result = remote_client(tls_endpoint)._post_infer(payload)
    assert result == {"action_chunk": [[0.1, 0.2]], "dt": 0.1}
    assert len(tls_endpoint.requests) == 1
    path, auth, body = tls_endpoint.requests[0]
    assert path == "/infer"
    assert auth == "Bearer " + "a" * 64
    assert json.loads(body) == payload


@pytest.mark.parametrize("status", [301, 302, 303, 307, 308])
def test_redirect_is_refused_without_following_it(tls_endpoint, status: int) -> None:
    """A redirect fails the call instead of resending observations elsewhere."""
    tls_endpoint.status = status
    with pytest.raises(adapter.RequestError, match="redirect"):
        remote_client(tls_endpoint)._post_infer({"images": {"front": "observation"}})
    assert len(tls_endpoint.requests) == 1


@pytest.mark.parametrize("status", [400, 401, 403, 500, 503])
def test_server_error_detail_reaches_the_operator(tls_endpoint, status: int) -> None:
    """The server's own reason is relayed, not replaced by a generic failure.

    ExecutePolicy shows this message as-is, so dropping the server's text
    would leave the operator with nothing to act on.
    """
    tls_endpoint.status = status
    tls_endpoint.body = b'{"error":"checkpoint revision mismatch"}'
    with pytest.raises(adapter.RequestError) as exc:
        remote_client(tls_endpoint)._post_infer({"images": {"front": "observation"}})
    assert "checkpoint revision mismatch" in str(exc.value)
    assert len(tls_endpoint.requests) == 1
    if status in (401, 403):
        # Only a credential rejection carries the provisioning hint.
        assert "AUTH-HINT" in str(exc.value)


def test_compressed_response_is_refused(tls_endpoint) -> None:
    """An encoded body is refused rather than decoded past the size cap."""
    tls_endpoint.extra_headers = {"Content-Encoding": "gzip"}
    tls_endpoint.body = b'{"dt":0.1}'
    with pytest.raises(adapter.RequestError, match="gzip-encoded body"):
        remote_client(tls_endpoint)._post_infer({})


def test_refused_concurrent_call_names_the_second_client(tls_endpoint) -> None:
    """A proxy's 503 is distinguishable from the server's still-loading 503.

    nginx's `limit_conn inference_requests 1` answers a concurrent call with
    HTML, so there is no {"error": ...} body to relay, and the likely cause is
    another client rather than an unreachable host.
    """
    tls_endpoint.status = 503
    tls_endpoint.body = b"<html><body>503 Service Temporarily Unavailable</body></html>"
    with pytest.raises(adapter.RequestError) as exc:
        remote_client(tls_endpoint)._post_infer({})
    assert "single /infer slot" in str(exc.value)
    assert "second client" in str(exc.value)


@pytest.mark.parametrize("status", [502, 504])
def test_gateway_failure_without_json_detail_names_the_link(
    tls_endpoint, status: int
) -> None:
    """A gateway error with no JSON body points at reachability of the server."""
    tls_endpoint.status = status
    tls_endpoint.body = b"<html><body>Bad Gateway</body></html>"
    with pytest.raises(adapter.RequestError) as exc:
        remote_client(tls_endpoint)._post_infer({})
    assert "no JSON detail" in str(exc.value)
    assert "UNREACHABLE-HINT" in str(exc.value)


def test_repeated_calls_reuse_one_tls_connection(tls_endpoint) -> None:
    """Inference calls share a connection instead of re-handshaking TLS.

    A handshake per call adds two round trips to every action chunk, which the
    ExecutePolicy timing budget cannot spare on a real link.
    """
    client = remote_client(tls_endpoint)
    for _ in range(5):
        client._post_infer({"state": [0.1]})
    assert len(tls_endpoint.requests) == 5
    assert tls_endpoint.connections == 1


@pytest.mark.parametrize("failure", ["unknown-ca", "wrong-host"])
def test_certificate_failure_sends_no_observations(tls_endpoint, failure: str) -> None:
    """Untrusted or hostname-invalid peers never receive an HTTP request."""
    client = remote_client(tls_endpoint)
    if failure == "unknown-ca":
        client._session.verify = True
    else:
        client.infer_url = client.infer_url.replace("127.0.0.1", "localhost")
    with pytest.raises(adapter.RequestError, match="TLS verification"):
        client._post_infer({"state": [1]})
    assert tls_endpoint.requests == []


@pytest.mark.parametrize("tls_endpoint", [-1], indirect=True)
def test_expired_certificate_sends_no_observations(tls_endpoint) -> None:
    """A trusted peer with an expired certificate receives no HTTP request."""
    with pytest.raises(adapter.RequestError, match="TLS verification") as error:
        remote_client(tls_endpoint)._post_infer({"state": [1]})
    assert "certificate has expired" in str(error.value.__cause__)
    assert tls_endpoint.requests == []


@pytest.mark.parametrize("extra", [0, 1])
def test_response_byte_limit_is_enforced_before_parsing(
    tls_endpoint, extra: int, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The exact limit accepts JSON padding; one declared byte over fails."""
    monkeypatch.setattr(adapter, "MAX_RESPONSE_BYTES", 2048)
    prefix = b'{"dt":0.1}'
    tls_endpoint.body = prefix + b" " * (
        adapter.MAX_RESPONSE_BYTES - len(prefix) + extra
    )
    client = remote_client(tls_endpoint)
    if extra:
        with pytest.raises(adapter.RequestError, match="declared"):
            client._post_infer({})
    else:
        assert client._post_infer({}) == {"dt": 0.1}


def test_undeclared_body_is_capped_while_streaming(
    tls_endpoint, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A body with no Content-Length is still bounded, mid-read.

    The cap and read size are tiny because the fixture trickles a byte every
    10ms. With realistic values the read would block until the deadline, and
    the test would prove nothing about size.
    """
    monkeypatch.setattr(adapter, "MAX_RESPONSE_BYTES", 32)
    monkeypatch.setattr(adapter, "READ_CHUNK_BYTES", 8)
    tls_endpoint.trickle = True
    client = remote_client(tls_endpoint)
    client.http_timeout = 5.0
    start = time.monotonic()
    with pytest.raises(adapter.RequestError, match="exceeds"):
        client._post_infer({})
    assert time.monotonic() - start < 2.0


def test_unreachable_response_socket_is_announced(
    tls_endpoint, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Losing the socket costs the call-level deadline, so it cannot be silent.

    The lookup reaches through private urllib3 attributes. If a version bump
    breaks it the body read falls back to a per-recv bound, which is the hold
    this deadline exists to prevent, so the operator has to hear about it.
    """
    warnings: list[str] = []
    monkeypatch.setattr(adapter, "response_socket", lambda resp: None)
    monkeypatch.setattr(
        adapter.GetActionChunkAdapter,
        "get_logger",
        lambda self: SimpleNamespace(
            warning=lambda message, **kwargs: warnings.append(message)
        ),
    )
    assert remote_client(tls_endpoint)._post_infer({})["dt"] == 0.1

    assert len(warnings) == 1
    assert "urllib3" in warnings[0]
    assert "policy_call_timeout" in warnings[0]


def test_json_that_is_not_an_object_is_refused(tls_endpoint) -> None:
    """A 200 carrying valid JSON that is not an object names the likely cause.

    This is what another service answering on the configured URL looks like:
    the status is fine and the body parses, so only the shape shows it.
    """
    tls_endpoint.body = b"[1, 2, 3]"
    with pytest.raises(adapter.RequestError, match="not a JSON object"):
        remote_client(tls_endpoint)._post_infer({})


def test_stalled_response_expires_at_the_deadline(tls_endpoint) -> None:
    """A server that answers, then goes quiet near the deadline, cannot outlive it.

    `requests` bounds each `recv` rather than the call, so without the watchdog
    shutting the socket down the last read runs its full timeout on top of the
    budget. The stall has to start late for that to show: measured on the
    shipped 9s budget, a stall at 8.7s held the call for 14.7s, past the
    ExecutePolicy call timeout this single-threaded node must stay inside.
    """
    client = remote_client(tls_endpoint)
    client.http_timeout = 1.0
    # Read slice is 1.0 - min(2.0, 1/4) = 0.75s, so an unbounded last read
    # would end near 1.55s. The deadline has to cut it at 1.0s instead.
    tls_endpoint.stall_after = 0.8
    start = time.monotonic()
    with pytest.raises(adapter.RequestError, match="timed out"):
        client._post_infer({})
    elapsed = time.monotonic() - start

    assert elapsed < client.http_timeout + 0.25, f"held the call for {elapsed:.2f}s"


def test_trickling_response_is_cancelled_and_next_call_works(tls_endpoint) -> None:
    """Active sockets still expire at the total deadline and do not block the next call."""
    client = remote_client(tls_endpoint)
    client.http_timeout = 0.15
    tls_endpoint.trickle = True
    start = time.monotonic()
    with pytest.raises(adapter.RequestError, match="timed out"):
        client._post_infer({})
    assert time.monotonic() - start < client.http_timeout + 0.3
    assert tls_endpoint.closed.wait(
        1.0
    ), "The timed-out transport must close its socket"
    tls_endpoint.trickle = False
    client.http_timeout = 1.0
    assert client._post_infer({})["dt"] == 0.1
