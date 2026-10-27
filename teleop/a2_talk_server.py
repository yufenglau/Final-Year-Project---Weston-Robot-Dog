#!/usr/bin/env python3
"""
Unitree A2 walkie-talkie: PC2 side. Plays laptop microphone audio on the robot's speaker.

Started by a2_talk.py on the laptop over SSH (you don't run this by hand):
    cd ~/a2_wireless && PYTHONPATH=~/a2_wireless python3 -u a2_talk_server.py --volume 70

stdin  = raw PCM, 16 kHz, mono, signed 16-bit little-endian (exactly what `arecord -f S16_LE -r 16000 -c 1 -t raw` gives).
The laptop only sends while you talk. A pause in the data (> GAP_S) ends one "utterance";
the next one gets a new stream id. Audio only: this program never sends motion commands.
"""
import argparse
import json
import os
import select
import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.a2.audio.audio_client import AudioClient

NETWORK_INTERFACE = "eth0"   # PC2's wired port to the robot network
APP_NAME = "a2_talk"
RATE = 16000                 # Hz, required by the robot
CHUNK_S = 0.5                # send this much audio per PlayStream call
GAP_S = 0.4                  # no data for this long = end of utterance


def log(msg):
    print(f"[PC2] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default=NETWORK_INTERFACE)
    ap.add_argument("--volume", type=int, default=70, help="robot speaker volume 0-100")
    ap.add_argument("--gain", type=float, default=2.0, help="software gain on the mic audio (clipped)")
    ap.add_argument("--check", action="store_true", help="only connect and read the volume (no sound), then exit")
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    client = AudioClient()
    client.SetTimeout(5.0)
    client.Init()

    code, vol = client.GetVolume()
    log(f"audio service OK (GetVolume code {code}, volume {vol})")
    if args.check:
        return
    volume = max(0, min(100, args.volume))
    code = client.SetVolume(volume)
    log(f"SetVolume {volume} -> code {code}")
    log("READY")

    chunk_bytes = int(CHUNK_S * RATE) * 2
    fd = sys.stdin.fileno()
    buf = bytearray()
    stream_id = None
    last_data = time.monotonic()
    sent_s = 0.0

    def send(data):
        nonlocal stream_id, sent_s
        if not data:
            return
        if stream_id is None:
            stream_id = str(int(time.time() * 1000))
            sent_s = 0.0
            log("talking...")
        pcm = np.frombuffer(bytes(data[: len(data) // 2 * 2]), dtype="<i2").astype(np.float32)
        pcm = np.clip(pcm * args.gain, -32768, 32767).astype("<i2").tobytes()
        code, _ = client.PlayStream(APP_NAME, stream_id, pcm)
        sent_s += len(pcm) / 2 / RATE
        if code != 0:
            log(f"PlayStream error code {code}")

    while True:
        ready, _, _ = select.select([fd], [], [], 0.05)
        now = time.monotonic()
        if ready:
            data = os.read(fd, 8192)
            if not data:                      # laptop closed the pipe: quit
                send(buf)
                break
            buf += data
            last_data = now
            while len(buf) >= chunk_bytes:
                send(buf[:chunk_bytes])
                del buf[:chunk_bytes]
        elif stream_id is not None and now - last_data > GAP_S:
            send(buf)                         # flush the tail of the utterance
            buf.clear()
            log(f"sent {sent_s:.1f} s of audio")
            stream_id = None
    log("bye")


if __name__ == "__main__":
    main()
