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

"""
Rename the fixed batch axis of every ONNX graph input to a symbolic one.

Usage: make_batch_dynamic.py <input.onnx> <output.onnx>

The SAM3 release exports fix the batch axis at 1. MoveIt Pro 9.4 creates every
SAM3 input tensor through ``ONNXTensorModel::dynamic_inputs``, which only holds
inputs with at least one dynamic dimension, so a fully static input such as
``images`` [1, 3, 1008, 1008] throws ``unordered_map::at``. Changing the first
dimension of each graph input from the value 1 to the parameter "batch" puts
those inputs back into ``dynamic_inputs``. MoveIt Pro still feeds batch 1.

The ONNX protobuf is edited at the wire level so the build needs only the
Python standard library. Everything except the input shapes is copied byte for
byte.
"""

import sys

# Field numbers from onnx/onnx.proto.
MODEL_GRAPH = 7
GRAPH_INPUT = 11
VALUE_INFO_TYPE = 2
TYPE_TENSOR_TYPE = 1
TENSOR_SHAPE = 2
SHAPE_DIM = 1
DIM_VALUE = 1

WIRE_VARINT = 0
WIRE_I64 = 1
WIRE_LEN = 2
WIRE_I32 = 5

BATCH_DIM = b"\x12\x05batch"  # Dimension.dim_param (field 2) = "batch"


def read_varint(buf, pos):
    result = 0
    shift = 0
    while True:
        byte = buf[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, pos
        shift += 7


def encode_varint(value):
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def fields(buf):
    """Yield (field number, wire type, raw field bytes, payload) for each field."""
    pos = 0
    while pos < len(buf):
        start = pos
        tag, pos = read_varint(buf, pos)
        number, wire_type = tag >> 3, tag & 7
        if wire_type == WIRE_VARINT:
            payload, pos = read_varint(buf, pos)
        elif wire_type == WIRE_LEN:
            length, pos = read_varint(buf, pos)
            end = pos + length
            payload = buf[pos:end]
            pos = end
        elif wire_type == WIRE_I64:
            payload = None
            pos += 8
        elif wire_type == WIRE_I32:
            payload = None
            pos += 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire_type}")
        yield number, wire_type, buf[start:pos], payload


def len_field(number, chunks):
    size = sum(len(chunk) for chunk in chunks)
    return [encode_varint(number << 3 | WIRE_LEN), encode_varint(size), *chunks]


def rewrite(buf, path, edit):
    """Apply edit to the LEN field reached by following path, a list of field numbers."""
    chunks = []
    for number, wire_type, raw, payload in fields(buf):
        if wire_type == WIRE_LEN and number == path[0]:
            inner = rewrite(payload, path[1:], edit) if path[1:] else edit(payload)
            chunks.extend(len_field(number, inner))
        else:
            chunks.append(raw)
    return chunks


def make_first_dim_dynamic(shape):
    chunks = []
    first = True
    for number, wire_type, raw, payload in fields(shape):
        if number == SHAPE_DIM and first:
            first = False
            dim = list(fields(payload))
            if len(dim) == 1 and dim[0][0] == DIM_VALUE and dim[0][3] == 1:
                chunks.extend(len_field(SHAPE_DIM, [BATCH_DIM]))
                continue
        chunks.append(raw)
    return chunks


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[1], "rb") as f:
        model = memoryview(f.read())
    input_shape = [GRAPH_INPUT, VALUE_INFO_TYPE, TYPE_TENSOR_TYPE, TENSOR_SHAPE]
    chunks = rewrite(model, [MODEL_GRAPH, *input_shape], make_first_dim_dynamic)
    with open(sys.argv[2], "wb") as f:
        for chunk in chunks:
            f.write(chunk)


if __name__ == "__main__":
    main()
