"""
sta_example.py

Station-mode example: connect ESP32 to your home/office WiFi and send UDP MIDI to your PC IP.
Recommended for stable operation.
"""

import network
import socket
import time
import struct
import gc

WIFI_SSID = "YOUR_ROUTER_SSID"
WIFI_PASS = "YOUR_ROUTER_PASSWORD"
SERVER_IP = "192.168.1.100"  # set to your PC's IP on the router
SERVER_PORT = 6000


def connect_sta():
    sta = network.WLAN(network.STA_IF)
    if not sta.active():
        sta.active(True)
    if not sta.isconnected():
        sta.connect(WIFI_SSID, WIFI_PASS)
        for _ in range(60):
            if sta.isconnected():
                break
            time.sleep(0.2)
    print("STA connected:", sta.isconnected(), sta.ifconfig())
    return sta


sta = connect_sta()
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.settimeout(0.2)


def send_note_on(note, vel=127):
    try:
        udp.sendto(struct.pack("BBB", 0x90, note, vel), (SERVER_IP, SERVER_PORT))
    except Exception as e:
        print("send error", e)
        gc.collect()


def send_note_off(note):
    try:
        udp.sendto(struct.pack("BBB", 0x80, note, 0), (SERVER_IP, SERVER_PORT))
    except Exception as e:
        print("send error", e)
        gc.collect()


try:
    while True:
        if not sta.isconnected():
            print("STA lost connection, reconnecting...")
            sta = connect_sta()
        send_note_on(36)
        time.sleep(0.06)
        send_note_off(36)
        gc.collect()
        time.sleep(0.45)
except KeyboardInterrupt:
    print("Stopping")
    try:
        udp.close()
    except Exception:
        pass
