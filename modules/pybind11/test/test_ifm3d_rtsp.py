#
# Copyright 2026-present ifm electronic, gmbh
# SPDX-License-Identifier: Apache-2.0
#

from enum import IntEnum
import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest

import ifm3dpy
from ifm3dpy.rtsp import RtspClient, NalUnit, DecoderManager


def test_rtsp_enums_are_int_enums():
    assert issubclass(RtspClient.Transport, IntEnum)
    assert issubclass(RtspClient.State, IntEnum)
    assert issubclass(NalUnit.Type, IntEnum)


def test_rtsp_enum_values():
    assert int(RtspClient.Transport.INTERLEAVED) == 0
    assert int(RtspClient.Transport.UDP) == 1
    assert int(RtspClient.State.IDLE) == 0
    assert int(RtspClient.State.FAILED) == 5
    assert int(NalUnit.Type.IDR_SLICE) == 5
    assert int(NalUnit.Type.SEI) == 6
    assert hasattr(ifm3dpy.framegrabber.buffer_id, "COMPRESSED_H264_FRAME")
    assert hasattr(ifm3dpy.framegrabber.buffer_id, "RGB_IMAGE")
    assert hasattr(ifm3dpy.framegrabber.buffer_id, "YUV420_IMAGE")
    # These are implicit successors of the enumerator above them, so inserting
    # a new buffer id anywhere earlier in the run silently renumbers them and
    # breaks every already-compiled consumer. Pin the values.
    assert int(ifm3dpy.framegrabber.buffer_id.COMPRESSED_H264_FRAME) == 0x100000007
    assert int(ifm3dpy.framegrabber.buffer_id.RGB_IMAGE) == 0x100000008
    assert int(ifm3dpy.framegrabber.buffer_id.YUV420_IMAGE) == 0x100000009


def test_rtsp_client_construct_with_defaults():
    # device may be None when a url is supplied.
    client = RtspClient(None, url="rtsp://127.0.0.1:1/port1")
    assert client.is_running() is False
    assert client.get_state() == RtspClient.State.IDLE


def test_rtsp_client_construct_with_options():
    client = RtspClient(
        None,
        url="rtsp://127.0.0.1:1/port2",
        port=8554,
        stream_path="port2",
        transport=RtspClient.Transport.UDP,
        decoder="ffmpeg",
    )
    assert client.is_running() is False
    assert client.get_state() == RtspClient.State.IDLE


def test_rtsp_client_initial_state():
    client = RtspClient(None, url="rtsp://127.0.0.1:1/port1")
    assert client.is_running() is False
    assert client.get_state() == RtspClient.State.IDLE


def test_rtsp_client_callbacks_are_registerable():
    client = RtspClient(None, url="rtsp://127.0.0.1:1/port1")

    # Registering and clearing callbacks must not raise.
    client.on_new_frame(lambda buffer: None)
    client.on_nal_unit(lambda nal: None)
    client.on_error(lambda err: None)
    client.on_state_change(lambda state: None)

    client.on_new_frame()
    client.on_nal_unit()
    client.on_error()
    client.on_state_change()


def test_rtsp_client_connection_refused_transitions_to_failed():
    # Port 1 is closed; the client must surface an error and end in FAILED.
    client = RtspClient(None, url="rtsp://127.0.0.1:1/port1")

    states = []
    errors = []
    client.on_state_change(lambda state: states.append(state))
    client.on_error(lambda err: errors.append(err))

    ok, _ = client.start(
        [ifm3dpy.framegrabber.buffer_id.COMPRESSED_H264_FRAME]
    ).wait_for(5000)
    assert ok is True
    client.stop().wait()

    assert RtspClient.State.CONNECTING in states
    assert RtspClient.State.FAILED in states
    assert len(errors) >= 1
    assert client.is_running() is False


def test_decoder_manager_discover_decoders_returns_list():
    decoders = DecoderManager.discover_decoders()
    assert isinstance(decoders, list)
    # The 'null' decoder is always listed.
    assert any(p.name == "null" for p in decoders)
    for p in decoders:
        assert isinstance(p.name, str)
        assert isinstance(p.available, bool)
        assert isinstance(p.supports_h264, bool)
        assert isinstance(p.error, str)
        # available and error must be consistent.
        assert (p.error == "") == p.available


def test_decoder_manager_decoder_info_repr():
    decoders = DecoderManager.discover_decoders()
    for p in decoders:
        assert "DecoderInfo" in repr(p)


def test_ifm3d_ffmpeg_package_is_activated_on_import(tmp_path):
    if not any(p.name == "ffmpeg" for p in DecoderManager.discover_decoders()):
        pytest.skip("ifm3d was built without FFmpeg decoder support")

    marker = tmp_path / "activated"
    package = tmp_path / "ifm3d_ffmpeg"
    package.mkdir()
    package.joinpath("__init__.py").write_text(
        textwrap.dedent(
            f"""
            def abi_info():
                return {{
                    "distribution_version": "8.1.2.post1",
                    "ffmpeg_version": "8.1.2",
                    "platform": "test-platform",
                    "avcodec_major": 62,
                    "avutil_major": 60,
                }}

            def activate():
                open({str(marker)!r}, "w").close()
            """
        ),
        encoding="utf-8",
    )

    env = os.environ.copy()
    python_paths = [str(tmp_path), str(Path(ifm3dpy.__file__).parent)]
    if env.get("PYTHONPATH"):
        python_paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(python_paths)

    result = subprocess.run(
        [sys.executable, "-c", "import ifm3dpy"],
        check=True,
        env=env,
        capture_output=True,
        text=True,
    )
    assert marker.exists()
    assert "Loaded ifm3d-ffmpeg build 8.1.2.post1" in result.stderr
    assert "valid: true" in result.stderr
