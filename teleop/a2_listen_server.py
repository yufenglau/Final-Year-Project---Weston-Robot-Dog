#!/usr/bin/env python3
"""
Unitree A2 live microphone: PC2 side. Forwards the robot's microphone to the laptop.

Started by a2_listen.py on the laptop over SSH (you don't run this by hand):
    cd ~/a2_wireless && python3 -u a2_listen_server.py

The robot (PC1) multicasts its microphone as raw PCM, 16 kHz, mono, signed 16-bit little-endian,
in 160 ms packets on UDP 239.168.123.161:5555 (same source as Unitree's a2_audio_client_example).
This joins that group on PC2's robot-network address and writes the PCM to stdout (the SSH pipe).
Read-only: it never sends anything to the robot.
"""
import argparse
import socket
import struct
import sys

GROUP_IP = "239.168.123.161"
PORT = 5555
LOCAL_IP = "192.168.123.162"   # PC2 eth0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--local-ip", default=LOCAL_IP)
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", PORT))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4s4s", socket.inet_aton(GROUP_IP), socket.inet_aton(args.local_ip)))
    s.settimeout(3.0)
    print(f"[PC2] listening to the robot mic ({GROUP_IP}:{PORT})", file=sys.stderr, flush=True)
    out = sys.stdout.buffer
    warned = False
    while True:
        try:
            data, _ = s.recvfrom(65535)
        except socket.timeout:
            if not warned:
                print("[PC2] no mic audio for 3 s (robot audio service running?)", file=sys.stderr, flush=True)
                warned = True
            continue
        warned = False
        try:
            out.write(data)
            out.flush()
        except (BrokenPipeError, OSError):
            break                     # laptop went away


if __name__ == "__main__":
    main()
