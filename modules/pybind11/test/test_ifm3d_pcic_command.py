#
# Copyright 2026-present ifm electronic, gmbh
# SPDX-License-Identifier: Apache-2.0
#

import pytest
from ifm3dpy.device import PCICCommand, Parameter, SetTemporaryApplicationParameter


def _command_from_payload(payload):
    class Command(PCICCommand):
        def serialize_data(self):
            return payload

    return Command()


@pytest.mark.parametrize(
    "payload",
    [
        [0x66, 0x30, 0x31],
        (0x66, 0x30, 0x31),
        bytearray(b"f01"),
    ],
)
def test_pcic_command_python_override_payload_types(payload):
    command = _command_from_payload(payload)
    assert PCICCommand.serialize_data(command) == [0x66, 0x30, 0x31]


def test_pcic_command_override_bytes_payload_rejected():
    command = _command_from_payload(b"f01")
    with pytest.raises((RuntimeError, TypeError)):
        PCICCommand.serialize_data(command)


def test_pcic_command_override_invalid_payload_raises_type_error():
    command = _command_from_payload("f01")
    with pytest.raises((RuntimeError, TypeError)):
        PCICCommand.serialize_data(command)


def test_pcic_command_without_override_raises():
    class IncompleteCommand(PCICCommand):
        pass

    command = IncompleteCommand()
    with pytest.raises(RuntimeError):
        PCICCommand.serialize_data(command)


def test_set_temporary_application_parameter_serialization_layout():
    payload = [0x01, 0x02, 0x03]
    command = SetTemporaryApplicationParameter(Parameter.ODS_MOTION_DATA,
                                               payload)

    serialized = PCICCommand.serialize_data(command)

    # Layout: 'f' + 5-digit parameter + '#00000' + payload bytes.
    assert bytes(serialized[:12]).decode("ascii") == "f02103#00000"
    assert serialized[12:] == payload


def test_set_temporary_application_parameter_ods_motion_data_prefix_encoding():
    command = SetTemporaryApplicationParameter(Parameter.ODS_MOTION_DATA,
                                               [0xAA, 0xBB])
    serialized = PCICCommand.serialize_data(command)

    assert bytes(serialized[:12]).decode("ascii") == "f02103#00000"
    assert serialized[-2:] == [0xAA, 0xBB]
