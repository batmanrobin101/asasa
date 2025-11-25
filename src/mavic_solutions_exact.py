#!/usr/bin/env python3
"""Mavic Solutions - Updated with extracted DUML framing and opcodes.

Discovered framing:
  - headers found: 0x55AA and 0xAA55 (we use 0x55AA by default)
  - assumed frame format: header(2) + class(1) + id(1) + len(2 little) + payload + crc32(4)
  - CRC computed as CRC32(header+class+id+len+payload) little-endian
  - summary of parsed frames available at duml_frames_summary.json

Discovered opcodes (most common pairs):
  - class 0x59, id 0xa5 : count 3
  - class 0x56, id 0x71 : count 2
  - class 0x5f, id 0x9c : count 2
  - class 0x18, id 0x80 : count 2
  - class 0x55, id 0xd0 : count 2
  - class 0xcd, id 0xdd : count 2
  - class 0x48, id 0x66 : count 2
  - class 0x16, id 0x5a : count 2
  - class 0x38, id 0x7f : count 2
  - class 0x67, id 0x0e : count 1
  - class 0xb5, id 0x61 : count 1
  - class 0x6a, id 0xdc : count 1
  - class 0x6b, id 0x2b : count 1
  - class 0xe0, id 0x28 : count 1
  - class 0x45, id 0xad : count 1
  - class 0xb1, id 0x41 : count 1
  - class 0x11, id 0x37 : count 1
  - class 0x4a, id 0xbb : count 1
  - class 0xdd, id 0xab : count 1
  - class 0x91, id 0xff : count 1

This script implements proper DUML framing using the discovered layout and example functions.
"""
import sys, time, struct, logging, binascii, zlib
import usb.core, usb.util, click

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

DEFAULT_VID = 0x2CA3
DEFAULT_PID = 0x0001

HEADER = b'\x55\xAA'  # primary header discovered
FRAME_OVERHEAD = 2+1+1+2+4  # header + class + id + len(2) + crc32(4)

def crc32_le(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF

def build_frame_exact(cls: int, cid: int, payload: bytes = b'') -> bytes:
    body = struct.pack('<BBH', cls & 0xFF, cid & 0xFF, len(payload)) + payload
    packet = HEADER + body
    chk = struct.pack('<I', crc32_le(packet))
    return packet + chk

def parse_frame_exact(raw: bytes):
    if len(raw) < FRAME_OVERHEAD:
        return None
    if not (raw.startswith(HEADER)):
        if raw.startswith(b'\xAA\x55'):
            hdr = raw[:2]
        else:
            return None
    hdr = raw[:2]
    cls, cid, length = struct.unpack_from('<BBH', raw, 2)
    end = 6 + length
    if len(raw) < end + 4:
        return None
    payload = raw[6:end]
    crc_recv = struct.unpack_from('<I', raw, end)[0]
    crc_calc = crc32_le(raw[:end])
    return {'header': hdr, 'class': cls, 'id': cid, 'len': length, 'payload': payload, 'crc_recv': crc_recv, 'crc_calc': crc_calc, 'crc_ok': crc_recv==crc_calc}

def find_device(vid=DEFAULT_VID, pid=DEFAULT_PID):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    return dev

def claim_device(dev):
    if dev is None:
        raise RuntimeError("No USB device")
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except Exception:
        pass
    usb.util.claim_interface(dev, 0)

def release_device(dev):
    try:
        usb.util.release_interface(dev, 0)
    except Exception:
        pass

def send_frame(dev, frame: bytes, out_ep=0x01):
    logging.debug("SEND: %s", binascii.hexlify(frame))
    return dev.write(out_ep, frame, timeout=5000)

def recv_frame(dev, in_ep=0x81, size=8192):
    try:
        data = dev.read(in_ep, size, timeout=5000)
        b = bytes(data)
        logging.debug("RECV: %s", binascii.hexlify(b))
        return b
    except usb.core.USBError as e:
        logging.debug("USB read error: %s", e)
        return b''

# High-level mapping (defaults)
CMD_CLASS = 0x12  # default, override if needed
CMD_ENTER = 0x01
CMD_EXIT = 0x02
CMD_READ_SN = 0x10
CMD_WRITE_SN = 0x11
CMD_CLEAR_10016 = 0x20
CMD_GIMBAL_CAL = 0x30

def enter_factory(dev, dry=False):
    frame = build_frame_exact(CMD_CLASS, CMD_ENTER, b'')
    if dry:
        print("DRY ENTER:", frame.hex())
        return None
    send_frame(dev, frame)
    time.sleep(0.2)
    r = recv_frame(dev)
    return parse_frame_exact(r) if r else None

def exit_factory(dev, dry=False):
    frame = build_frame_exact(CMD_CLASS, CMD_EXIT, b'')
    if dry:
        print("DRY EXIT:", frame.hex())
        return None
    send_frame(dev, frame)
    time.sleep(0.1)
    r = recv_frame(dev)
    return parse_frame_exact(r) if r else None

def read_sn(dev, dry=False):
    frame = build_frame_exact(CMD_CLASS, CMD_READ_SN, b'')
    if dry:
        print("DRY READ_SN:", frame.hex())
        return "DRY-SN-0000"
    send_frame(dev, frame)
    r = recv_frame(dev)
    parsed = parse_frame_exact(r) if r else None
    if not parsed:
        return None
    try:
        return parsed['payload'].decode('latin-1', errors='ignore')
    except Exception:
        return parsed['payload'].hex()

def write_sn(dev, new_sn: str, dry=False):
    payload = new_sn.encode('ascii', errors='ignore')
    frame = build_frame_exact(CMD_CLASS, CMD_WRITE_SN, payload)
    if dry:
        print("DRY WRITE_SN:", frame.hex())
        return True
    send_frame(dev, frame)
    r = recv_frame(dev)
    return parse_frame_exact(r) if r else None

def clear_10016(dev, dry=False):
    frame = build_frame_exact(CMD_CLASS, CMD_CLEAR_10016, b'')
    if dry:
        print("DRY CLEAR_10016:", frame.hex())
        return True
    send_frame(dev, frame)
    r = recv_frame(dev)
    return parse_frame_exact(r) if r else None

def gimbal_cal(dev, dry=False):
    frame = build_frame_exact(CMD_CLASS, CMD_GIMBAL_CAL, b'\\x01\\x00\\x00\\x00')
    if dry:
        print("DRY GIMBAL_CAL:", frame.hex())
        return True
    send_frame(dev, frame)
    r = recv_frame(dev)
    return parse_frame_exact(r) if r else None

import click

@click.group()
def cli():
    pass

@cli.command()
@click.option('--vid', default=DEFAULT_VID, type=int)
@click.option('--pid', default=DEFAULT_PID, type=int)
@click.option('--dry', is_flag=True)
def info(vid, pid, dry):
    dev = find_device(vid, pid)
    if not dev:
        logging.error("Device not found")
        return
    try:
        claim_device(dev)
        logging.info("Device found")
        sn = read_sn(dev, dry=dry)
        logging.info("SN: %s", sn)
    finally:
        release_device(dev)

@cli.command()
@click.option('--vid', default=DEFAULT_VID, type=int)
@click.option('--pid', default=DEFAULT_PID, type=int)
@click.option('--sn', required=True)
@click.option('--dry', is_flag=True)
def write_sn_cmd(vid, pid, sn, dry):
    dev = find_device(vid, pid)
    if not dev:
        logging.error("Device not found")
        return
    try:
        claim_device(dev)
        res = write_sn(dev, sn, dry=dry)
        logging.info("Write SN result: %s", res)
    finally:
        release_device(dev)

@cli.command()
@click.option('--vid', default=DEFAULT_VID, type=int)
@click.option('--pid', default=DEFAULT_PID, type=int)
@click.option('--dry', is_flag=True)
def clear10016(vid, pid, dry):
    dev = find_device(vid, pid)
    if not dev:
        logging.error("Device not found")
        return
    try:
        claim_device(dev)
        res = clear_10016(dev, dry=dry)
        logging.info("Clear10016 result: %s", res)
    finally:
        release_device(dev)

@cli.command()
@click.option('--vid', default=DEFAULT_VID, type=int)
@click.option('--pid', default=DEFAULT_PID, type=int)
@click.option('--dry', is_flag=True)
def enter(vid, pid, dry):
    dev = find_device(vid, pid)
    if not dev:
        logging.error("Device not found")
        return
    try:
        claim_device(dev)
        res = enter_factory(dev, dry=dry)
        logging.info("Enter result: %s", res)
    finally:
        release_device(dev)

if __name__ == '__main__':
    cli()
