"""Dual-mode server for receiving MIDI data from ESP32 drum controllers.

Supports both TCP over WiFi and Bluetooth Low Energy (BLE) communication modes.
"""

import socket
import threading
from typing import Callable, Optional, Dict
import struct
import time

# Try to import BLE dependencies (optional)
try:
    import asyncio
    from bleak import BleakScanner, BleakClient
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False
    asyncio = None
    BleakScanner = None
    BleakClient = None


# BLE UUIDs (used when in BLE mode)
DRUM_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
DRUM_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class TCPServer:
    """TCP server that receives MIDI data from ESP32 drum controllers over WiFi.
    
    Listens on a TCP port and accepts connections from multiple ESP32 devices.
    Each device sends MIDI data which is forwarded to the on_packet callback.
    """

    def __init__(self, on_packet: Callable[[bytes, str], None], host: str = "0.0.0.0", port: int = 6000):
        """Initialize TCP server.
        
        Args:
            on_packet: Callback function called with (data: bytes, client_address: str)
            host: Host address to bind to (default: "0.0.0.0" for all interfaces)
            port: Port to listen on (default: 6000)
        """
        self.on_packet = on_packet
        self.host = host
        self.port = port
        self._server_socket: Optional[socket.socket] = None
        self._client_sockets: Dict[str, socket.socket] = {}
        self._running = threading.Event()
        self._server_thread: Optional[threading.Thread] = None
        self._client_threads: Dict[str, threading.Thread] = {}

    def start(self) -> None:
        """Start the TCP server in a background thread."""
        self._running.set()
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

    def _run_server(self) -> None:
        """Run the TCP server loop accepting connections."""
        try:
            # Create TCP socket
            self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_socket.bind((self.host, self.port))
            self._server_socket.listen(5)
            self._server_socket.settimeout(1.0)  # Non-blocking with timeout
            
            print(f"🌐 TCP Server listening on {self.host}:{self.port}")
            
            while self._running.is_set():
                try:
                    client_socket, client_address = self._server_socket.accept()
                    client_addr_str = f"{client_address[0]}:{client_address[1]}"
                    print(f"✅ Client connected: {client_addr_str}")
                    
                    # Store client socket
                    self._client_sockets[client_addr_str] = client_socket
                    
                    # Start thread to handle this client
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_socket, client_addr_str),
                        daemon=True
                    )
                    self._client_threads[client_addr_str] = client_thread
                    client_thread.start()
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if self._running.is_set():
                        print(f"Accept error: {e}")
                        
        except Exception as e:
            print(f"Server error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self._server_socket:
                self._server_socket.close()

    def _handle_client(self, client_socket: socket.socket, client_address: str) -> None:
        """Handle communication with a connected client.
        
        Args:
            client_socket: Socket for this client
            client_address: String representation of client address
        """
        try:
            client_socket.settimeout(1.0)  # Timeout for recv
            buffer = b""
            
            while self._running.is_set():
                try:
                    # Receive data from client
                    data = client_socket.recv(1024)
                    
                    if not data:
                        # Client disconnected
                        print(f"❌ Client disconnected: {client_address}")
                        break
                    
                    buffer += data
                    
                    # Process complete MIDI messages (3 bytes each)
                    while len(buffer) >= 3:
                        # Extract one MIDI message
                        midi_msg = buffer[:3]
                        buffer = buffer[3:]
                        
                        # Call the callback
                        self.on_packet(midi_msg, client_address)
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error receiving from {client_address}: {e}")
                    break
                    
        except Exception as e:
            print(f"Client handler error for {client_address}: {e}")
        finally:
            # Clean up client
            try:
                client_socket.close()
            except:
                pass
            
            if client_address in self._client_sockets:
                del self._client_sockets[client_address]
            if client_address in self._client_threads:
                del self._client_threads[client_address]

    def stop(self) -> None:
        """Stop the TCP server and disconnect all clients."""
        print("🛑 Stopping TCP server...")
        self._running.clear()
        
        # Close all client connections
        for address, client_socket in list(self._client_sockets.items()):
            try:
                client_socket.close()
                print(f"Disconnected from {address}")
            except Exception as e:
                print(f"Error disconnecting {address}: {e}")
        
        self._client_sockets.clear()
        
        # Close server socket
        if self._server_socket:
            try:
                self._server_socket.close()
            except:
                pass
        
        # Wait for threads to finish
        if self._server_thread:
            self._server_thread.join(timeout=2.0)

    @property
    def connected_devices(self) -> list[str]:
        """Get list of connected client addresses."""
        return list(self._client_sockets.keys())

    @property
    def is_running(self) -> bool:
        """Check if the TCP server is running."""
        return self._running.is_set()
    
    @property
    def connection_count(self) -> int:
        """Get number of connected clients."""
        return len(self._client_sockets)


class BLEServer:
    """BLE server that scans for and connects to ESP32 drum controllers.
    
    Scans for devices with a specific name prefix and subscribes to MIDI notifications.
    """

    def __init__(self, on_packet: Callable[[bytes, str], None], device_name_prefix: str = "ESPDRUMS"):
        """Initialize BLE server.
        
        Args:
            on_packet: Callback function called with (data: bytes, device_address: str)
            device_name_prefix: Prefix to filter BLE device names (default: "ESPDRUMS")
        """
        if not BLE_AVAILABLE:
            raise RuntimeError("BLE mode requires 'bleak' library. Install with: pip install bleak")
        
        self.on_packet = on_packet
        self.device_name_prefix = device_name_prefix
        self._clients: Dict[str, BleakClient] = {}
        self._connected_addresses: set[str] = set()
        self._failed_addresses: Dict[str, float] = {}
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def start(self) -> None:
        """Start the BLE server in a background thread."""
        self._running.set()
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()

    def _run_async_loop(self) -> None:
        """Run the async event loop in the background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._scan_and_connect())
        except Exception as e:
            print(f"BLE error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._loop.close()

    async def _scan_and_connect(self) -> None:
        """Scan for drum devices and connect to them."""
        print(f"🔍 Scanning for BLE devices with name starting with '{self.device_name_prefix}'...")
        
        while self._running.is_set():
            try:
                current_time = time.time()
                
                # Clean up old failed connection attempts (after 10 seconds)
                for addr in list(self._failed_addresses.keys()):
                    if current_time - self._failed_addresses[addr] > 10.0:
                        del self._failed_addresses[addr]
                
                # Remove disconnected clients from tracking
                for address in list(self._connected_addresses):
                    if address not in self._clients or not self._clients[address].is_connected:
                        self._connected_addresses.discard(address)
                        if address in self._clients:
                            del self._clients[address]
                
                # Scan for devices if we need more connections
                if len(self._connected_addresses) < 4:  # Support up to 4 devices
                    devices = await BleakScanner.discover(timeout=5.0)
                    
                    for device in devices:
                        if device.address in self._failed_addresses:
                            continue
                            
                        if (device.name and 
                            device.name.startswith(self.device_name_prefix) and 
                            device.address not in self._connected_addresses):
                            
                            print(f"📡 Found drum device: {device.name} ({device.address})")
                            success = await self._connect_device(device)
                            if not success:
                                self._failed_addresses[device.address] = current_time
                
                await asyncio.sleep(5.0)
                
            except Exception as e:
                print(f"Scan error: {e}")
                await asyncio.sleep(5.0)

    async def _connect_device(self, device) -> bool:
        """Connect to a BLE device and subscribe to notifications."""
        try:
            print(f"🔗 Attempting connection to {device.name} ({device.address})...")
            client = BleakClient(
                device.address,
                disconnected_callback=lambda c: self._on_disconnect(device.address)
            )
            await client.connect(timeout=15.0)
            print(f"✅ Connected to {device.name} ({device.address})")
            
            self._clients[device.address] = client
            self._connected_addresses.add(device.address)
            
            await asyncio.sleep(2.0)
            
            # Find MIDI characteristic
            def normalize_uuid(uuid_str):
                return str(uuid_str).replace("-", "").lower()
            
            drum_service_norm = normalize_uuid(DRUM_SERVICE_UUID)
            drum_char_norm = normalize_uuid(DRUM_CHAR_UUID)
            
            char_uuid = None
            for service in client.services:
                if normalize_uuid(service.uuid) == drum_service_norm:
                    for char in service.characteristics:
                        if normalize_uuid(char.uuid) == drum_char_norm:
                            char_uuid = char.uuid
                            print(f"✓ Found drum characteristic!")
                            break
            
            if char_uuid:
                await client.start_notify(
                    char_uuid,
                    lambda sender, data: self._handle_notification(device.address, data)
                )
                print(f"🎵 Subscribed to MIDI notifications from {device.name}")
                return True
            else:
                print(f"⚠️ Could not find MIDI characteristic on {device.name}")
                await client.disconnect()
                self._connected_addresses.discard(device.address)
                del self._clients[device.address]
                return False
                
        except Exception as e:
            print(f"Connection error for {device.address}: {e}")
            self._connected_addresses.discard(device.address)
            if device.address in self._clients:
                try:
                    await self._clients[device.address].disconnect()
                except:
                    pass
                del self._clients[device.address]
            return False

    def _on_disconnect(self, device_address: str) -> None:
        """Handle device disconnection."""
        print(f"❌ Device disconnected: {device_address}")
        self._connected_addresses.discard(device_address)
        if device_address in self._clients:
            del self._clients[device_address]

    def _handle_notification(self, device_address: str, data: bytes) -> None:
        """Handle incoming BLE notification (MIDI data)."""
        try:
            self.on_packet(data, device_address)
        except Exception as e:
            print(f"Error in notification handler: {e}")

    def stop(self) -> None:
        """Stop the BLE server and disconnect all devices."""
        self._running.clear()
        
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._disconnect_all(), self._loop)
        
        if self._thread:
            self._thread.join(timeout=2.0)

    async def _disconnect_all(self) -> None:
        """Disconnect all connected devices."""
        for address, client in list(self._clients.items()):
            try:
                if client.is_connected:
                    await client.disconnect()
                    print(f"Disconnected from {address}")
            except Exception as e:
                print(f"Error disconnecting {address}: {e}")
        self._clients.clear()
        self._connected_addresses.clear()

    @property
    def connected_devices(self) -> list[str]:
        """Get list of connected device addresses."""
        return list(self._clients.keys())

    @property
    def is_running(self) -> bool:
        """Check if the BLE server is running."""
        return self._running.is_set()

    @property
    def connection_count(self) -> int:
        """Get number of connected clients."""
        return len(self._clients)
