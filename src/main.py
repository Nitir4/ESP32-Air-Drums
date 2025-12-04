from typing import Tuple
import argparse

from .ble_server import TCPServer, BLEServer
from .midi_handler import MidiHandler
from .audio_engine import AudioEngine
from .ui import MidiUI


def main(argv: Tuple[str, ...] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["tcp", "ble"], default="tcp",
                       help="Communication mode: tcp (WiFi) or ble (Bluetooth) (default: tcp)")
    parser.add_argument("--host", default="0.0.0.0", 
                       help="[TCP mode] Host address to bind TCP server (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=6000,
                       help="[TCP mode] Port to listen on (default: 6000)")
    parser.add_argument("--device-prefix", default="ESPDRUMS",
                       help="[BLE mode] BLE device name prefix to scan for (default: ESPDRUMS)")
    args = parser.parse_args(argv)

    engine = AudioEngine()

    # UI will run in main thread; run server in background thread
    ui = MidiUI(on_close=lambda: None)

    def note_on(note: int, velocity: int) -> None:
        engine.note_on(note, velocity)
        ui.show_note(note, velocity)

    def note_off(note: int) -> None:
        engine.note_off(note)

    midi = MidiHandler(note_on, note_off)

    # Start server based on mode
    if args.mode == "ble":
        print("🔵 Starting in BLE mode...")
        server = BLEServer(
            lambda data, addr: midi.handle_packet(data),
            device_name_prefix=args.device_prefix
        )
        server.start()
        ui.set_status(f"🔵 BLE mode: Scanning for '{args.device_prefix}*' devices...")
    else:
        print("🌐 Starting in TCP mode...")
        server = TCPServer(
            lambda data, addr: midi.handle_packet(data), 
            host=args.host,
            port=args.port
        )
        server.start()
        ui.set_status(f"🌐 TCP mode: Listening on {args.host}:{args.port}")

    try:
        ui.run()
    finally:
        server.stop()


if __name__ == "__main__":
    main()
