"""
ota_ap.py

Simple HTTP uploader for MicroPython. Run this on the ESP32 AP and POST files to /upload?file=<name>&token=<token>
This writes the file and reboots the device.

Usage (PC connected to ESP32 AP):
curl -X POST --data-binary @main.py "http://192.168.4.1/upload?file=main.py&token=NitiOTA"
"""

import network, socket, time, machine, gc, ure
from micropython import const

SSID = "ESPDRUMS"
PASSWORD = "Nitirocks"
UPLOAD_TOKEN = "NitiOTA"
SERVER_PORT = const(80)

# Optional: UDP MIDI send parameters. Set DEST to the IP of the host listening for MIDI
# or use the AP broadcast address (192.168.4.255) to reach the connected client.
MIDI_DEST = "192.168.4.255"
MIDI_PORT = 6000
MIDI_NOTE = 36
MIDI_VELOCITY = 120
MIDI_CHANNEL = 9  # MIDI channel 10 (drums) is 9 when zero-based
SEND_MIDI_ON_BOOT = True

# activate AP
ap = network.WLAN(network.AP_IF)
if not ap.active():
    ap.active(True)
    ap.config(essid=SSID, password=PASSWORD, authmode=3)
while not ap.active():
    time.sleep(0.1)
print("AP active, IP:", ap.ifconfig())

# --- send a single MIDI Note On (drum 36) via UDP so listeners on the AP network
# can receive an initial hit while the OTA server keeps running.
if SEND_MIDI_ON_BOOT:
    try:
        import socket
        # UDP socket for MIDI messages
        midi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # allow broadcast so 192.168.4.255 will work
        try:
            midi_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass

        status_byte = 0x90 | (MIDI_CHANNEL & 0x0F)  # Note On on MIDI channel
        msg = bytes((status_byte, int(MIDI_NOTE) & 0x7F, int(MIDI_VELOCITY) & 0x7F))
        midi_sock.sendto(msg, (MIDI_DEST, MIDI_PORT))
        # don't rely on note-off for percussive drum hits; if you want a note-off,
        # you can send it after a short delay in a separate thread or timer.
        midi_sock.close()
        print("Sent MIDI note {} to {}:{}".format(MIDI_NOTE, MIDI_DEST, MIDI_PORT))
    except Exception as e:
        print("Failed to send MIDI note on boot:", e)


def http_response(conn, code=200, body="OK"):
    conn.write("HTTP/1.1 {} OK\r\n".format(code))
    conn.write("Content-Type: text/plain\r\n")
    conn.write("Content-Length: {}\r\n".format(len(body)))
    conn.write("Connection: close\r\n")
    conn.write("\r\n")
    conn.write(body)


def handle_upload(conn, q, content_len):
    m_file = ure.search("file=([^&]+)", q)
    m_token = ure.search("token=([^&]+)", q)
    if not m_file:
        http_response(conn, 400, "Missing 'file' parameter")
        return
    filename = m_file.group(1)
    token = m_token.group(1) if m_token else ""
    if token != UPLOAD_TOKEN:
        http_response(conn, 403, "Invalid token")
        return

    try:
        with open(filename, "wb") as f:
            remaining = content_len
            while remaining > 0:
                chunk = conn.read(min(1024, remaining))
                if not chunk:
                    break
                f.write(chunk)
                remaining -= len(chunk)
                gc.collect()
        if remaining != 0:
            http_response(conn, 400, "Incomplete upload")
            return
    except Exception as e:
        http_response(conn, 500, "Write failed: {}".format(e))
        return

    http_response(conn, 200, "Saved {} ({} bytes). Rebooting...".format(filename, content_len))
    conn.close()
    time.sleep(0.2)
    machine.reset()


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", SERVER_PORT))
s.listen(1)
print("OTA server listening on 0.0.0.0:{}".format(SERVER_PORT))

try:
    while True:
        cl, addr = s.accept()
        cl.settimeout(5)
        print("Client from", addr)
        try:
            request_line = cl.readline()
            if not request_line:
                cl.close(); continue
            request_line = request_line.decode().strip()
            method, path_q, _ = request_line.split(" ", 2)
            path_parts = path_q.split("?", 1)
            path = path_parts[0]
            q = path_parts[1] if len(path_parts) > 1 else ""

            content_len = 0
            while True:
                line = cl.readline()
                if not line:
                    break
                line = line.decode().strip()
                if line == "":
                    break
                if line.lower().startswith("content-length:"):
                    content_len = int(line.split(":")[1].strip())
            if method == "POST" and path == "/upload":
                if content_len <= 0:
                    http_response(cl, 400, "Missing Content-Length")
                    cl.close()
                else:
                    handle_upload(cl, q, content_len)
            else:
                body = "ESP32 OTA: POST /upload?file=<name>&token=<token>\n"
                http_response(cl, 200, body)
                cl.close()
        except Exception as e:
            try:
                http_response(cl, 500, "Error: {}".format(e))
            except Exception:
                pass
            cl.close()
        gc.collect()
except KeyboardInterrupt:
    s.close()
    print("Server stopped")
