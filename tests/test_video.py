from unittest.mock import Mock

import numpy as np

from aag_yam_sim.video import StreamingVideoWriter, _as_uint8


def test_as_uint8_converts_float_frames():
    frame = np.array([[[0.0, 0.5, 1.0]]], dtype=np.float32)

    converted = _as_uint8(frame)

    assert converted.dtype == np.uint8
    assert converted.tolist() == [[[0, 127, 255]]]
    assert converted.flags.c_contiguous


def test_streaming_writer_appends_without_frame_buffer(monkeypatch, tmp_path):
    backend = Mock()
    get_writer = Mock(return_value=backend)
    monkeypatch.setattr("imageio.v2.get_writer", get_writer)
    output = tmp_path / "rollout.mp4"

    writer = StreamingVideoWriter(output, fps=30)
    writer.append(np.zeros((16, 16, 3), dtype=np.uint8))
    writer.append(np.ones((16, 16, 3), dtype=np.uint8))
    writer.close()

    get_writer.assert_called_once_with(str(output), format="FFMPEG", fps=30)
    assert backend.append_data.call_count == 2
    backend.close.assert_called_once_with()
    assert writer.frame_count == 2
    assert writer._writer is None
    assert not hasattr(writer, "frames")
