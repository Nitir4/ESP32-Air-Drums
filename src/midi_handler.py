from typing import Callable, Optional

try:
    import mido
except Exception:  # pragma: no cover - fallback when mido isn't installed
    mido = None  # type: ignore


class MidiHandler:
    """Parse raw MIDI bytes and route events to callbacks.

    Callbacks expected: note_on(note:int, velocity:int), note_off(note:int), control_change(cc:int, value:int)
    """

    def __init__(
        self,
        note_on_cb: Callable[[int, int], None],
        note_off_cb: Callable[[int], None],
        control_change_cb: Optional[Callable[[int, int], None]] = None,
    ) -> None:
        self.note_on_cb = note_on_cb
        self.note_off_cb = note_off_cb
        self.control_change_cb = control_change_cb

    def handle_packet(self, data: bytes) -> None:
        """Parse data bytes and invoke callbacks. Expects data to contain one or more MIDI messages.

        Uses mido if available; otherwise a tiny fallback supporting 0x8n, 0x9n, 0xBn messages.
        """
        if mido:
            try:
                for msg in mido.parse_all(data):
                    self._handle_mido_message(msg)
                return
            except Exception:
                # fall through to fallback parser
                pass

        # Fallback quick parser (handles most basic 3-byte messages)
        i = 0
        while i < len(data):
            status = data[i]
            if status >= 0x80 and status <= 0xEF:
                # Determine message length by high nibble
                typ = status & 0xF0
                if typ in (0x80, 0x90, 0xB0):
                    if i + 2 < len(data):
                        ch = status & 0x0F
                        a = data[i + 1]
                        b = data[i + 2]
                        if typ == 0x90:
                            if b > 0:
                                self.note_on_cb(a, b)
                            else:
                                self.note_off_cb(a)
                        elif typ == 0x80:
                            self.note_off_cb(a)
                        elif typ == 0xB0 and self.control_change_cb:
                            self.control_change_cb(a, b)
                        i += 3
                        continue
                # Unknown/unsupported or running status: skip one byte
                i += 1
            else:
                # Data byte without status — skip
                i += 1

    def _handle_mido_message(self, msg) -> None:  # msg is mido.Message
        if msg.type == "note_on":
            if msg.velocity > 0:
                self.note_on_cb(int(msg.note), int(msg.velocity))
            else:
                self.note_off_cb(int(msg.note))
        elif msg.type == "note_off":
            self.note_off_cb(int(msg.note))
        elif msg.type == "control_change" and self.control_change_cb:
            self.control_change_cb(int(msg.control), int(msg.value))
