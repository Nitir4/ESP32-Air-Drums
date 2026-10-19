"""Send fake MIDI messages over UDP to test the app locally.

Sends alternating note_on/note_off messages for a scale.
"""
from __future__ import annotations

import argparse
import socket
import time
from typing import Tuple


def make_note_on(channel: int, note: int, velocity: int) -> bytes:
    status = 0x90 | (channel & 0x0F)
    return bytes([status, note & 0x7F, velocity & 0x7F])


def make_note_off(channel: int, note: int, velocity: int = 64) -> bytes:
    status = 0x80 | (channel & 0x0F)
    return bytes([status, note & 0x7F, velocity & 0x7F])


def run(host: str, port: int, interval: float = 0.5) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # General MIDI drum notes - play different drums in sequence
    drum_pattern = [
        (36, 100, "Kick"),
        (38, 110, "Snare"),
        (42, 90, "Closed Hi-hat"),
        (46, 85, "Open Hi-hat"),
        (49, 95, "Crash"),
        (36, 120, "Kick (hard)"),
        (38, 100, "Snare"),
        (42, 80, "Hi-hat"),
        (47, 90, "Tom Low-Mid"),
        (48, 90, "Tom Hi-Mid"),
        (50, 90, "Tom High"),
        (51, 85, "Ride"),
    ]
    try:
        print(f"Sending drum MIDI notes to {host}:{port} (Ctrl+C to stop)")
        print("Pattern: Kick → Snare → Hi-hats → Crash → Toms → Ride\n")
        while True:
            for note, vel, name in drum_pattern:
                print(f"  Playing: {name} (note {note}, vel {vel})")
                sock.sendto(make_note_on(0, note, vel), (host, port))
                time.sleep(interval)
                sock.sendto(make_note_off(0, note, 64), (host, port))
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped")


def main(argv: Tuple[str, ...] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--interval", type=float, default=0.25)
    args = parser.parse_args(argv)
    run(args.host, args.port, args.interval)


if __name__ == "__main__":
    main()
