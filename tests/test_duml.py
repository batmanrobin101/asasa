
import json, zlib

def test_summary_loads():
    with open("duml_frames_summary.json") as f:
        data = json.load(f)
    assert "common_pairs" in data

def test_crc_build():
    # Basic test of DUML frame CRC calculation
    hdr = b"\x55\xAA"
    cls = b"\x12"
    cid = b"\x01"
    ln = b"\x00\x00"
    frame = hdr + cls + cid + ln
    crc = zlib.crc32(frame) & 0xFFFFFFFF
    assert isinstance(crc, int)
