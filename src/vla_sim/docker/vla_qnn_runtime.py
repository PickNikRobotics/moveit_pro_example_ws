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

"""Resident native QNN HTP sessions for QAIRT 2.49 (core API 2.38).

The ctypes layouts and function-table offsets match the QAIRT 2.49 headers.
Only fixed, dense, unquantized float/integer IO is supported. No QAIRT library
is opened at import time, and there is no ONNX Runtime or CPU/GPU fallback.
"""

import ctypes as c
import json
import os
import threading
import time
from pathlib import Path

import numpy as np

ERROR = c.c_uint64
HANDLE = c.c_void_p


class Version(c.Structure):
    _fields_ = [(name, c.c_uint32) for name in ("major", "minor", "patch")]


class ApiVersion(c.Structure):
    _fields_ = [("core", Version), ("backend", Version)]


class Interface(c.Structure):
    _fields_ = [
        ("backend_id", c.c_uint32),
        ("name", c.c_char_p),
        ("version", ApiVersion),
        ("functions", c.c_void_p * 51),
    ]


class OnChip(c.Structure):
    _fields_ = [
        ("vtcm_mb", c.c_size_t),
        ("soc_model", c.c_uint32),
        ("signed_pd", c.c_bool),
        ("dlbc", c.c_bool),
        ("arch", c.c_int),
    ]


class HtpExtension(c.Structure):
    _fields_ = [("device_type", c.c_int), ("on_chip", OnChip)]


class HardwareV1(c.Structure):
    _fields_ = [
        ("device_id", c.c_uint32),
        ("device_type", c.c_uint32),
        ("num_cores", c.c_uint32),
        ("cores", c.c_void_p),
        ("extension", c.POINTER(HtpExtension)),
    ]


class Hardware(c.Structure):
    _fields_ = [("version", c.c_int), ("v1", HardwareV1)]


class PlatformV1(c.Structure):
    _fields_ = [("num_devices", c.c_uint32), ("devices", c.POINTER(Hardware))]


class Platform(c.Structure):
    _fields_ = [("version", c.c_int), ("v1", PlatformV1)]


class ScaleOffset(c.Structure):
    _fields_ = [("scale", c.c_float), ("offset", c.c_int32)]


class QuantUnion(c.Union):
    _fields_ = [("scale_offset", ScaleOffset), ("storage", c.c_uint64 * 4)]


class QuantParams(c.Structure):
    _fields_ = [("definition", c.c_int), ("encoding", c.c_int), ("value", QuantUnion)]


class ClientBuffer(c.Structure):
    _fields_ = [("data", c.c_void_p), ("size", c.c_uint32)]


class TensorMemory(c.Union):
    _fields_ = [("client", ClientBuffer), ("handle", HANDLE)]


class SparseParams(c.Structure):
    _fields_ = [
        ("layout", c.c_int),
        ("specified", c.c_uint32),
        ("dimensions", c.c_uint32),
    ]


class TensorV2(c.Structure):
    _fields_ = [
        ("id", c.c_uint32),
        ("name", c.c_char_p),
        ("type", c.c_int),
        ("format", c.c_uint32),
        ("dtype", c.c_int),
        ("quant", QuantParams),
        ("rank", c.c_uint32),
        ("dimensions", c.POINTER(c.c_uint32)),
        ("memory_type", c.c_int),
        ("memory", TensorMemory),
        ("dynamic", c.POINTER(c.c_uint8)),
        ("sparse", SparseParams),
        ("produced", c.c_uint8),
    ]


class TensorUnion(c.Union):
    _fields_ = [("v2", TensorV2)]


class Tensor(c.Structure):
    _fields_ = [("version", c.c_int), ("data", TensorUnion)]


class DeviceConfig(c.Structure):
    _fields_ = [("option", c.c_int), ("platform", c.POINTER(Platform))]


class Dcvs(c.Structure):
    _fields_ = [
        (name, c.c_uint32)
        for name in (
            "context_id",
            "set_dcvs",
            "dcvs",
            "mode",
            "set_sleep_latency",
            "sleep_latency",
            "set_sleep_disable",
            "sleep_disable",
            "set_bus",
            "bus_min",
            "bus_target",
            "bus_max",
            "set_core",
            "core_min",
            "core_target",
            "core_max",
        )
    ]


class PowerUnion(c.Union):
    _fields_ = [("dcvs", Dcvs), ("scalar", c.c_uint32)]


class Power(c.Structure):
    _fields_ = [("option", c.c_int), ("value", PowerUnion)]


class Perf(c.Structure):
    _fields_ = [
        (name, HANDLE)
        for name in ("create_id", "destroy_id", "set_power", "set_memory")
    ]


class Infrastructure(c.Structure):
    _fields_ = [("type", c.c_int), ("perf", Perf)]


DTYPES = {
    "QNN_DATATYPE_FLOAT_32": (0x0232, np.dtype("float32")),
    "QNN_DATATYPE_FLOAT_16": (0x0216, np.dtype("float16")),
    "QNN_DATATYPE_INT_32": (0x0032, np.dtype("int32")),
    "QNN_DATATYPE_UINT_32": (0x0132, np.dtype("uint32")),
}


def function(address, result, *args):
    """Require a real API pointer before constructing a callable C ABI pointer."""
    if not address:
        raise RuntimeError("QNN required API pointer is null; use QAIRT 2.49")
    return c.CFUNCTYPE(result, *args)(address)


class NativeRuntime:
    """Own HTP backend, device, power clients and resident graph contexts."""

    def __init__(self):
        library = os.environ.get("QNN_HTP_LIBRARY", "/usr/lib/libQnnHtp.so")
        try:
            self.lib = c.CDLL(library, mode=os.RTLD_NOW | os.RTLD_LOCAL)
        except OSError as exc:
            raise RuntimeError(
                f"QNN backend requires the native QAIRT 2.49 HTP runtime; "
                f"could not load '{library}': {exc}. Install or mount the board's "
                "QAIRT libraries and set QNN_HTP_LIBRARY/LD_LIBRARY_PATH and "
                "ADSP_LIBRARY_PATH. The standard PyTorch image does not include "
                "QAIRT; CPU/GPU fallback is disabled."
            ) from exc
        get = self.lib.QnnInterface_getProviders
        get.argtypes = [
            c.POINTER(c.POINTER(c.POINTER(Interface))),
            c.POINTER(c.c_uint32),
        ]
        get.restype = ERROR
        providers = c.POINTER(c.POINTER(Interface))()
        count = c.c_uint32()
        status = get(c.byref(providers), c.byref(count))
        if status or not providers or not 1 <= count.value <= 8 or not providers[0]:
            raise RuntimeError(
                f"QNN provider discovery status={status}, count={count.value}; "
                "use the native QAIRT 2.49 HTP runtime"
            )
        self.api = providers[0].contents
        if (
            self.api.backend_id != 6
            or self.api.version.core.major != 2
            or self.api.version.core.minor != 38
        ):
            raise RuntimeError(
                "QNN backend is not the verified HTP/core 2.38 ABI; "
                "use the board's QAIRT 2.49 library"
            )
        self.functions = {
            "backend_create": function(
                self.api.functions[1], ERROR, HANDLE, HANDLE, c.POINTER(HANDLE)
            ),
            "backend_free": function(self.api.functions[8], ERROR, HANDLE),
            "context_create": function(
                self.api.functions[13],
                ERROR,
                HANDLE,
                HANDLE,
                HANDLE,
                HANDLE,
                c.c_uint64,
                c.POINTER(HANDLE),
                HANDLE,
            ),
            "context_free": function(self.api.functions[14], ERROR, HANDLE, HANDLE),
            "graph_retrieve": function(
                self.api.functions[20], ERROR, HANDLE, c.c_char_p, c.POINTER(HANDLE)
            ),
            "graph_execute": function(
                self.api.functions[21],
                ERROR,
                HANDLE,
                c.POINTER(Tensor),
                c.c_uint32,
                c.POINTER(Tensor),
                c.c_uint32,
                HANDLE,
                HANDLE,
            ),
            "platform_info": function(
                self.api.functions[37], ERROR, HANDLE, c.POINTER(c.POINTER(Platform))
            ),
            "platform_free": function(
                self.api.functions[38], ERROR, HANDLE, c.POINTER(Platform)
            ),
            "infrastructure": function(
                self.api.functions[39], ERROR, c.POINTER(c.POINTER(Infrastructure))
            ),
            "device_create": function(
                self.api.functions[40],
                ERROR,
                HANDLE,
                c.POINTER(c.POINTER(DeviceConfig)),
                c.POINTER(HANDLE),
            ),
            "device_free": function(self.api.functions[43], ERROR, HANDLE),
            "error_message": function(
                self.api.functions[48], ERROR, ERROR, c.POINTER(c.c_char_p)
            ),
        }
        self.backend = HANDLE()
        self.devices = {}
        self.power = {}
        self.models = []
        self.lock = threading.RLock()
        try:
            self.check(
                self.functions["backend_create"](None, None, c.byref(self.backend)),
                "backendCreate",
            )
            if not self.backend:
                raise RuntimeError("QNN backendCreate returned a null backend")
        except Exception:
            self.close()
            raise

    def check(self, status, operation):
        """Include the native error; never fall back after a QNN refusal."""
        if status:
            message = c.c_char_p()
            self.functions["error_message"](status, c.byref(message))
            detail = (
                message.value.decode(errors="replace")
                if message.value
                else "no backend message"
            )
            raise RuntimeError(
                f"QNN {operation} failed status={status}: {detail}; "
                "CPU/GPU fallback is disabled"
            )

    def device(self, index):
        """Select a reported HTP device, preserving its own power/context client."""
        if type(index) is not int or index not in (0, 1):
            raise ValueError("QNN HTP device must be 0 or 1")
        if index in self.devices:
            return self.devices[index]
        actual = c.POINTER(Platform)()
        self.check(
            self.functions["platform_info"](None, c.byref(actual)),
            "deviceGetPlatformInfo",
        )
        try:
            if (
                not actual
                or actual.contents.version != 1
                or not 1 <= actual.contents.v1.num_devices <= 8
                or not actual.contents.v1.devices
            ):
                raise RuntimeError("Native QNN platform version/count is unsupported")
            selected = [
                actual.contents.v1.devices[i]
                for i in range(actual.contents.v1.num_devices)
                if actual.contents.v1.devices[i].version == 1
                and actual.contents.v1.devices[i].v1.device_id == index
            ]
            if (
                len(selected) != 1
                or selected[0].v1.num_cores != 1
                or not selected[0].v1.cores
                or not selected[0].v1.extension
            ):
                raise RuntimeError(
                    f"Native QNN HTP device {index} is missing or ambiguous; "
                    "check the board's device access and platform identity"
                )
            platform = Platform(1, PlatformV1(1, c.pointer(selected[0])))
            config = DeviceConfig(1, c.pointer(platform))
            configs = (c.POINTER(DeviceConfig) * 2)(c.pointer(config), None)
            handle = HANDLE()
            self.check(
                self.functions["device_create"](None, configs, c.byref(handle)),
                f"deviceCreate({index})",
            )
            if not handle:
                raise RuntimeError("QNN deviceCreate returned a null HTP device")
            self.devices[index] = handle
        finally:
            if actual:
                self.check(
                    self.functions["platform_free"](None, actual),
                    "deviceFreePlatformInfo",
                )
        if os.environ.get("QNN_HTP_BURST", "1") == "0":
            return handle
        infra = c.POINTER(Infrastructure)()
        self.check(
            self.functions["infrastructure"](c.byref(infra)), "deviceGetInfrastructure"
        )
        if not infra or infra.contents.type != 0:
            raise RuntimeError("QNN did not expose HTP performance infrastructure")
        perf = infra.contents.perf
        create = function(
            perf.create_id, ERROR, c.c_uint32, c.c_uint32, c.POINTER(c.c_uint32)
        )
        destroy = function(perf.destroy_id, ERROR, c.c_uint32)
        set_power = function(
            perf.set_power, ERROR, c.c_uint32, c.POINTER(c.POINTER(Power))
        )
        client = c.c_uint32()
        self.check(create(index, 0, c.byref(client)), f"createPowerConfigId({index})")
        self.power[index] = (client.value, destroy)
        # QAIRT HTP DCVS v3: burst mode (0x10), sleep latency 40 us, sleep
        # disabled, and TURBO (0x80) for both bus and core voltage corners.
        dcvs = Power()
        dcvs.option = 1
        dcvs.value.dcvs = Dcvs(
            client.value,
            1,
            0,
            0x10,
            1,
            40,
            1,
            1,
            1,
            0x80,
            0x80,
            0x80,
            1,
            0x80,
            0x80,
            0x80,
        )
        values = (c.POINTER(Power) * 2)(c.pointer(dcvs), None)
        self.check(
            set_power(client.value, values), f"setPowerConfig({index},burst/TURBO)"
        )
        return handle

    def load(self, binary: bytes, abi, device, *, name="model.bin"):
        """Load verified snapshot bytes; own a partial model before native allocation."""
        if not isinstance(binary, bytes) or not binary:
            raise ValueError("QNN load requires a nonempty immutable context snapshot")
        with self.lock:
            model = NativeModel(self, abi, device)
            self.models.append(model)
            try:
                model.load(binary, name)
            except Exception:
                # If native free fails, ownership remains here and close()
                # will retry this context before releasing its dependencies.
                model.close()
                self.models.remove(model)
                raise
            return model

    def close(self):
        """Release in dependency order; retain ownership if a native free fails."""
        with self.lock:
            while self.models:
                self.models[-1].close()
                self.models.pop()
            for index, (client, destroy) in list(self.power.items()):
                self.check(destroy(client), f"destroyPowerConfigId({index})")
                del self.power[index]
            for index, handle in list(self.devices.items()):
                self.check(
                    self.functions["device_free"](handle), f"deviceFree({index})"
                )
                del self.devices[index]
            if self.backend:
                self.check(self.functions["backend_free"](self.backend), "backendFree")
                self.backend = HANDLE()


class NativeModel:
    """Run dense native tensors by name, preserving integer IDs and every output."""

    def __init__(self, runtime, abi, device):
        self.runtime = runtime
        self.device_id = device
        info = json.loads(Path(abi).read_text())["info"]
        if info["backendId"] != 6 or info["numGraphs"] != 1 or len(info["graphs"]) != 1:
            raise ValueError("Native QNN context must contain exactly one HTP graph")
        graph = info["graphs"][0]["info"]
        self.graph_name = graph["graphName"]
        self.inputs, self.input_refs = self.tensor_array(graph["graphInputs"], True)
        self.outputs, self.output_refs = self.tensor_array(graph["graphOutputs"], False)
        self.input_names = [r["info"]["name"] for r in graph["graphInputs"]]
        self.output_names = [r["info"]["name"] for r in graph["graphOutputs"]]
        self.context = HANDLE()
        self.graph = HANDLE()
        self.call_count = 0
        self.binary: bytes | None = None

    def load(self, binary: bytes, name: str):
        """Deserialize the exact checked bytes, not a re-opened artifact path."""
        began = time.perf_counter()
        self.binary = binary
        buffer = c.c_char_p(binary)
        try:
            self.runtime.check(
                self.runtime.functions["context_create"](
                    self.runtime.backend,
                    self.runtime.device(self.device_id),
                    None,
                    c.cast(buffer, HANDLE),
                    len(binary),
                    c.byref(self.context),
                    None,
                ),
                f"contextCreateFromBinary({name},device{self.device_id})",
            )
        finally:
            # QNN's default context config is nonpersistent, so it does not
            # retain this buffer and no second multi-GiB weight copy stays resident.
            self.binary = None
        if not self.context:
            raise RuntimeError(
                "Native QNN returned a null context; refuse graph retrieval"
            )
        self.runtime.check(
            self.runtime.functions["graph_retrieve"](
                self.context, self.graph_name.encode(), c.byref(self.graph)
            ),
            f"graphRetrieve({self.graph_name})",
        )
        if not self.graph:
            raise RuntimeError("Native QNN reported a null graph; refuse execution")
        self.load_s = time.perf_counter() - began

    @staticmethod
    def tensor_array(rows, is_input):
        """Support fixed dense unquantized IO; never guess a tensor representation."""
        if not rows or len(rows) > 64:
            raise ValueError("Native graph requires 1..64 input/output tensors")
        names = [row["info"]["name"] for row in rows]
        if any(not isinstance(name, str) or not name for name in names) or len(
            set(names)
        ) != len(names):
            raise ValueError("Native context has empty/duplicate port names")
        tensors = (Tensor * len(rows))()
        refs = []
        for i, row in enumerate(rows):
            spec = row["info"]
            if (
                row["version"] != "QNN_TENSOR_VERSION_2"
                or spec["dataType"] not in DTYPES
            ):
                raise ValueError(
                    f"Native tensor {spec['name']} has unsupported version/dtype"
                )
            quant = spec["quantizeParams"]
            if (
                quant["definition"] != "QNN_DEFINITION_UNDEFINED"
                or quant["quantizationEncoding"]
                != "QNN_QUANTIZATION_ENCODING_UNDEFINED"
            ):
                raise ValueError(
                    f"Native tensor {spec['name']} uses quantized IO; require float/integer IO"
                )
            shape = tuple(spec["dimensions"])
            if (
                not shape
                or len(shape) > 16
                or any(type(d) is not int or not 1 <= d <= 0xFFFFFFFF for d in shape)
            ):
                raise ValueError(
                    f"Native tensor {spec['name']} has dynamic/invalid extents"
                )
            if spec["rank"] != len(shape) or spec["type"] != (
                "QNN_TENSOR_TYPE_APP_WRITE" if is_input else "QNN_TENSOR_TYPE_APP_READ"
            ):
                raise ValueError(
                    f"Native tensor {spec['name']} rank/type disagrees with its port"
                )
            if spec["dataFormat"] != "QNN_TENSOR_DATA_FORMAT_FLAT_BUFFER":
                raise ValueError("Native runner accepts dense tensors only")
            dtype_code, dtype = DTYPES[spec["dataType"]]
            if int(np.prod(shape, dtype=object)) * dtype.itemsize > 0xFFFFFFFF:
                raise ValueError(
                    "Native tensor buffer exceeds the uint32 byte-size limit"
                )
            if type(spec["id"]) is not int or not 0 <= spec["id"] <= 0xFFFFFFFF:
                raise ValueError(
                    "Native tensor id must fit a uint32 without truncation"
                )
            name = spec["name"].encode()
            dimensions = (c.c_uint32 * len(shape))(*shape)
            tensor = tensors[i]
            tensor.version = 2
            tensor.data.v2.id = spec["id"]
            tensor.data.v2.name = name
            tensor.data.v2.type = 0 if is_input else 1
            tensor.data.v2.format = 0
            tensor.data.v2.dtype = dtype_code
            tensor.data.v2.quant.definition = 0x7FFFFFFF
            tensor.data.v2.quant.encoding = 0x7FFFFFFF
            tensor.data.v2.rank = len(shape)
            tensor.data.v2.dimensions = dimensions
            tensor.data.v2.memory_type = 0
            tensor.data.v2.sparse.layout = 0x7FFFFFFF
            refs.append(
                {
                    "name": name,
                    "dimensions": dimensions,
                    "shape": shape,
                    "dtype": dtype,
                    "buffer": None if is_input else np.empty(shape, dtype=dtype),
                }
            )
        return tensors, refs

    def run(self, feed):
        """Bind named host tensor bytes; QnnGraph_execute is the only neural path."""
        with self.runtime.lock:
            if not self.context:
                raise RuntimeError("Native context already closed; reload the server")
            if set(feed) != set(self.input_names):
                raise ValueError(f"Native {self.graph_name} input names mismatch")
            refs = []  # Keep every contiguous input alive until graphExecute returns.
            for i, name in enumerate(self.input_names):
                spec = self.input_refs[i]
                value = np.asarray(feed[name])
                if (
                    value.dtype != spec["dtype"]
                    or value.shape != spec["shape"]
                    or not np.isfinite(value).all()
                ):
                    raise ValueError(
                        f"Native input {name} {value.dtype}/{value.shape} != "
                        f"{spec['dtype']}/{spec['shape']} or nonfinite; refuse execution"
                    )
                value = np.ascontiguousarray(value)
                refs.append(value)
                self.inputs[i].data.v2.memory.client = ClientBuffer(
                    value.ctypes.data, value.nbytes
                )
            for i, spec in enumerate(self.output_refs):
                output = spec["buffer"]
                output.fill(np.nan if output.dtype.kind == "f" else 0)
                self.outputs[i].data.v2.memory.client = ClientBuffer(
                    output.ctypes.data, output.nbytes
                )
            self.runtime.check(
                self.runtime.functions["graph_execute"](
                    self.graph,
                    self.inputs,
                    len(self.input_names),
                    self.outputs,
                    len(self.output_names),
                    None,
                    None,
                ),
                f"graphExecute({self.graph_name},device{self.device_id})",
            )
            outputs = {
                spec["name"].decode(): spec["buffer"].copy()
                for spec in self.output_refs
            }
            if any(not np.isfinite(value).all() for value in outputs.values()):
                raise RuntimeError(
                    "Native QNN returned nonfinite outputs; discard the entire chunk"
                )
            self.call_count += 1
            return outputs

    def close(self):
        """Destroy only this model's context and mapped binary."""
        if self.context:
            self.runtime.check(
                self.runtime.functions["context_free"](self.context, None),
                f"contextFree({self.graph_name})",
            )
            self.context = HANDLE()
            self.graph = HANDLE()
        self.binary = None
