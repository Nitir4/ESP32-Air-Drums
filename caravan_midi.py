"""Send the drum pattern from "Caravan" by Duke Ellington over UDP.

Classic Latin/swing jazz pattern with distinctive rhythm.
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


def play_caravan_pattern(sock, host: str, port: int, tempo_bpm: int = 120) -> None:
    """Play the classic Caravan drum pattern.
    
    The pattern is in 4/4 time with a distinctive Latin/swing feel.
    """
    # Calculate note durations based on tempo
    beat = 60.0 / tempo_bpm  # Quarter note duration
    eighth = beat / 2
    sixteenth = beat / 4
    
    # MIDI note mapping
    kick = 36
    snare = 38
    hihat_closed = 42
    hihat_open = 46
    ride = 51
    crash = 49
    tom_low = 45
    tom_mid = 47
    tom_high = 50
    
    # Classic Caravan pattern (2-bar phrase)
    # Bar 1: Emphasis on the Latin tresillo rhythm
    pattern_bar1 = [
        # Beat 1
        [(kick, 110), (hihat_closed, 85)],  # 1
        [(hihat_closed, 70)],                # &
        [(snare, 95), (hihat_closed, 80)],   # 2
        [(hihat_closed, 65)],                # &
        # Beat 2
        [(kick, 105)],                       # 3
        [(hihat_closed, 70)],                # &
        [(snare, 100), (hihat_closed, 85)],  # 4
        [(hihat_closed, 75)],                # &
    ]
    
    # Bar 2: Continuation with ride cymbal
    pattern_bar2 = [
        # Beat 1
        [(kick, 110), (ride, 80)],           # 1
        [(ride, 65)],                        # &
        [(snare, 95), (ride, 75)],           # 2
        [(ride, 60)],                        # &
        # Beat 2
        [(kick, 105), (ride, 75)],           # 3
        [(ride, 70)],                        # &
        [(snare, 100), (ride, 80)],          # 4
        [(ride, 65)],                        # &
    ]
    
    # Fill pattern (every 8 bars) - tom cascade
    fill_pattern = [
        [(tom_high, 100)],
        [(tom_high, 95)],
        [(tom_mid, 105)],
        [(tom_mid, 100)],
        [(tom_low, 110)],
        [(tom_low, 105)],
        [(kick, 115), (crash, 110)],
        [],
    ]
    
    bar_count = 0
    
    try:
        print(f"🎺 Playing Caravan drum pattern at {tempo_bpm} BPM")
        print(f"Sending to {host}:{port}")
        print("Press Ctrl+C to stop\n")
        
        while True:
            bar_count += 1
            
            # Every 8 bars, play a fill
            if bar_count % 8 == 0:
                print(f"  [Bar {bar_count}] 🔥 FILL!")
                for step in fill_pattern:
                    for note, vel in step:
                        sock.sendto(make_note_on(0, note, vel), (host, port))
                    time.sleep(sixteenth)
                    for note, vel in step:
                        sock.sendto(make_note_off(0, note), (host, port))
            else:
                # Alternate between bar 1 and bar 2 patterns
                if bar_count % 2 == 1:
                    pattern = pattern_bar1
                    print(f"  [Bar {bar_count}] Kick-Snare groove (hi-hat)")
                else:
                    pattern = pattern_bar2
                    print(f"  [Bar {bar_count}] Kick-Snare groove (ride)")
                
                for step in pattern:
                    for note, vel in step:
                        sock.sendto(make_note_on(0, note, vel), (host, port))
                    time.sleep(eighth)
                    for note, vel in step:
                        sock.sendto(make_note_off(0, note), (host, port))
    
    except KeyboardInterrupt:
        print("\n\n🎵 Stopped playing Caravan")


def main(argv: Tuple[str, ...] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Play the drum pattern from 'Caravan' over UDP"
    )
    parser.add_argument("--host", default="127.0.0.1", help="Target host")
    parser.add_argument("--port", type=int, default=6000, help="Target port")
    parser.add_argument(
        "--tempo", type=int, default=180, help="Tempo in BPM (default: 180)"
    )
    args = parser.parse_args(argv)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    play_caravan_pattern(sock, args.host, args.port, args.tempo)


if __name__ == "__main__":
    main()
