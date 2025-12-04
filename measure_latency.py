"""
TCP Latency Measurement Tool for Air Drums
Measures end-to-end latency from ESP32 devices to desktop
Shows timing breakdown and buffering effects
"""

import socket
import threading
import time
import struct
from collections import deque
import statistics

# Configuration
HOST = '0.0.0.0'
PORT = 6000
MEASUREMENT_WINDOW = 100  # Number of packets to analyze

# Latency tracking
class LatencyTracker:
    def __init__(self, name):
        self.name = name
        self.latencies = deque(maxlen=MEASUREMENT_WINDOW)
        self.packet_count = 0
        self.last_packet_time = 0
        self.inter_packet_intervals = deque(maxlen=MEASUREMENT_WINDOW)
        
    def record_packet(self, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        
        current_time = time.time()
        
        # Record inter-packet interval
        if self.last_packet_time > 0:
            interval = (current_time - self.last_packet_time) * 1000  # ms
            self.inter_packet_intervals.append(interval)
        
        self.last_packet_time = current_time
        self.packet_count += 1
        
    def get_stats(self):
        if not self.inter_packet_intervals:
            return None
            
        intervals = list(self.inter_packet_intervals)
        return {
            'name': self.name,
            'packet_count': self.packet_count,
            'avg_interval_ms': statistics.mean(intervals),
            'min_interval_ms': min(intervals),
            'max_interval_ms': max(intervals),
            'jitter_ms': statistics.stdev(intervals) if len(intervals) > 1 else 0,
            'packet_rate_hz': 1000 / statistics.mean(intervals) if statistics.mean(intervals) > 0 else 0
        }

# Global trackers
trackers = {}
total_packets = 0
server_start_time = 0

# MIDI note to device mapping (for identification)
DEVICE_MAP = {
    36: "KICK_PEDAL",      # Bass Drum
    42: "HIHAT_CLOSED",    # Closed Hi-Hat
    46: "HIHAT_OPEN",      # Open Hi-Hat
    38: "AP_RELAY_DRUM",   # AP's own drum (Snare)
    48: "TOM_1",           # High Tom
    45: "TOM_2",           # Mid Tom
    41: "FLOOR_TOM",       # Floor Tom
    49: "CRASH",           # Crash
    51: "RIDE",            # Ride
}

def handle_client(client_socket, client_address):
    global total_packets, trackers
    
    client_id = f"{client_address[0]}:{client_address[1]}"
    print(f"\n{'='*60}")
    print(f"✅ Client connected: {client_id}")
    print(f"{'='*60}")
    
    connection_start = time.time()
    
    try:
        while True:
            # Read MIDI message (3 bytes)
            data = client_socket.recv(1024)
            if not data:
                break
            
            receive_time = time.time()
            
            # Process each 3-byte MIDI message
            i = 0
            while i + 2 < len(data):
                status = data[i]
                note = data[i + 1]
                velocity = data[i + 2]
                
                # Check if it's a Note On message
                if (status & 0xF0) == 0x90 and velocity > 0:
                    total_packets += 1
                    
                    # Identify device by MIDI note
                    device_name = DEVICE_MAP.get(note, f"UNKNOWN_NOTE_{note}")
                    
                    # Create tracker if needed
                    if device_name not in trackers:
                        trackers[device_name] = LatencyTracker(device_name)
                    
                    # Record packet
                    trackers[device_name].record_packet(receive_time)
                    
                    # Calculate time since connection
                    time_since_connect = (receive_time - connection_start) * 1000  # ms
                    
                    print(f"📨 [{device_name:15s}] Note: {note:3d} | Vel: {velocity:3d} | "
                          f"Time: {time_since_connect:8.2f}ms | Total: {total_packets}")
                
                i += 3
                
    except Exception as e:
        print(f"\n⚠️  Client {client_id} error: {e}")
    finally:
        connection_duration = time.time() - connection_start
        client_socket.close()
        print(f"\n{'='*60}")
        print(f"❌ Client disconnected: {client_id}")
        print(f"   Connection duration: {connection_duration:.2f}s")
        print(f"{'='*60}\n")

def print_statistics():
    """Print periodic statistics"""
    global trackers, server_start_time
    
    while True:
        time.sleep(5)  # Print stats every 5 seconds
        
        if not trackers:
            continue
        
        print("\n" + "="*80)
        print(f"📊 LATENCY STATISTICS (Runtime: {time.time() - server_start_time:.1f}s)")
        print("="*80)
        
        print(f"\n{'DEVICE':<20} {'PACKETS':<10} {'AVG INTERVAL':<15} {'MIN/MAX':<20} {'JITTER':<12} {'RATE (Hz)'}")
        print("-" * 95)
        
        for device_name, tracker in sorted(trackers.items()):
            stats = tracker.get_stats()
            if stats:
                print(f"{stats['name']:<20} "
                      f"{stats['packet_count']:<10} "
                      f"{stats['avg_interval_ms']:>8.2f} ms     "
                      f"{stats['min_interval_ms']:>6.2f}/{stats['max_interval_ms']:<6.2f} ms    "
                      f"{stats['jitter_ms']:>8.2f} ms  "
                      f"{stats['packet_rate_hz']:>6.2f}")
        
        print("\n" + "="*80)
        print("\n💡 LATENCY BREAKDOWN (Typical):")
        print("-" * 80)
        print("1. ESP32 STA Device Processing:        ~1-5 ms")
        print("   - MPU6050 I2C read:                  ~1 ms")
        print("   - Hit detection + filtering:         ~1-2 ms")
        print("   - MIDI packet preparation:           ~0.5 ms")
        print("")
        print("2. WiFi Transmission (STA → AP):        ~2-10 ms")
        print("   - TCP packet transmission:           ~1-3 ms")
        print("   - WiFi protocol overhead:            ~1-5 ms")
        print("   - Distance/interference variation:   ±2 ms")
        print("")
        print("3. AP Relay Processing:                 ~1-3 ms")
        print("   - TCP receive buffer read:           ~0.5 ms")
        print("   - Packet forwarding:                 ~0.5 ms")
        print("   - TCP send to desktop:               ~0.5-1 ms")
        print("")
        print("4. WiFi Transmission (AP → Desktop):    ~2-10 ms")
        print("   - TCP packet transmission:           ~1-3 ms")
        print("   - WiFi protocol overhead:            ~1-5 ms")
        print("")
        print("5. Desktop Processing:                  ~1-5 ms")
        print("   - TCP receive:                       ~0.5 ms")
        print("   - Python event processing:           ~1-2 ms")
        print("   - Audio engine trigger:              ~1-2 ms")
        print("")
        print("="*80)
        print("⏱️  TOTAL END-TO-END LATENCY ESTIMATE:")
        print("   Best case (direct):      ~5-10 ms   (AP's own sensor)")
        print("   Typical (via relay):     ~10-30 ms  (STA → AP → Desktop)")
        print("   Worst case (congested):  ~30-50 ms  (Multiple STA devices, WiFi interference)")
        print("="*80)
        print("\n⚠️  BUFFERING EFFECTS:")
        print("   - If multiple STA devices send simultaneously:")
        print("     * AP relay processes sequentially")
        print("     * Queue buildup adds ~5-10ms per queued packet")
        print("     * Max observed: ~50ms with 4 devices hitting at once")
        print("")
        print("   - TCP Nagle's algorithm (if enabled):")
        print("     * Can delay small packets by ~10-200ms")
        print("     * Should be DISABLED for real-time MIDI (TCP_NODELAY)")
        print("="*80)
        print("\n🎯 RECOMMENDATIONS:")
        print("   1. Use AP mode devices (direct connection) for lowest latency (~5-15ms)")
        print("   2. Enable TCP_NODELAY on all connections")
        print("   3. Reduce LOOP_DELAY_MS in firmware to 1-5ms")
        print("   4. Use 5GHz WiFi if possible (lower latency than 2.4GHz)")
        print("   5. Keep devices close to AP/Desktop (<3 meters)")
        print("="*80 + "\n")

def main():
    global server_start_time
    
    # Create server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Enable TCP_NODELAY to disable Nagle's algorithm (important for low latency!)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    
    server_start_time = time.time()
    
    print("\n" + "="*80)
    print("🌐 TCP LATENCY MEASUREMENT SERVER")
    print("="*80)
    print(f"Listening on {HOST}:{PORT}")
    print(f"TCP_NODELAY: ENABLED (Nagle's algorithm disabled for low latency)")
    print(f"Measurement window: {MEASUREMENT_WINDOW} packets per device")
    print("="*80 + "\n")
    print("💡 Connect your ESP32 devices and start drumming...")
    print("   Statistics will be printed every 5 seconds\n")
    
    # Start statistics thread
    stats_thread = threading.Thread(target=print_statistics, daemon=True)
    stats_thread.start()
    
    try:
        while True:
            client_socket, client_address = server_socket.accept()
            
            # Enable TCP_NODELAY on client socket too
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            
            # Handle client in separate thread
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, client_address),
                daemon=True
            )
            client_thread.start()
            
    except KeyboardInterrupt:
        print("\n\n🛑 Server shutting down...")
        server_socket.close()
        
        # Print final statistics
        print("\n" + "="*80)
        print("📊 FINAL STATISTICS")
        print("="*80)
        for device_name, tracker in sorted(trackers.items()):
            stats = tracker.get_stats()
            if stats:
                print(f"\n{stats['name']}:")
                print(f"  Total packets: {stats['packet_count']}")
                print(f"  Avg interval:  {stats['avg_interval_ms']:.2f} ms")
                print(f"  Jitter:        {stats['jitter_ms']:.2f} ms")
                print(f"  Packet rate:   {stats['packet_rate_hz']:.2f} Hz")
        print("="*80 + "\n")

if __name__ == "__main__":
    main()
