"""
Real-time Air Drums Visualization
Visualizes drumstick zones, hit detection, and velocity in real-time
Shows 3D drum kit layout with live hit feedback
"""

import pygame
import sys
import socket
import threading
import struct
import math
from collections import deque
import time

# Initialize Pygame
pygame.init()

# Screen settings
WIDTH, HEIGHT = 1400, 900
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Air Drums - Live Visualization")
clock = pygame.time.Clock()

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 50, 50)
GREEN = (50, 255, 50)
BLUE = (50, 150, 255)
YELLOW = (255, 255, 50)
ORANGE = (255, 165, 0)
PURPLE = (180, 50, 255)
CYAN = (50, 255, 255)
GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)

# Fonts
font_large = pygame.font.Font(None, 48)
font_medium = pygame.font.Font(None, 36)
font_small = pygame.font.Font(None, 24)

# Zone colors
ZONE_COLORS = {
    38: (RED, "SNARE"),         # Snare
    48: (BLUE, "TOM 1"),        # High Tom
    45: (CYAN, "TOM 2"),        # Mid Tom
    41: (ORANGE, "FLOOR"),      # Floor Tom
    42: (YELLOW, "HI-HAT"),     # Closed Hi-Hat
    46: (YELLOW, "HI-HAT OPEN"), # Open Hi-Hat
    49: (PURPLE, "CRASH"),      # Crash
    51: (GREEN, "RIDE"),        # Ride
    36: (RED, "KICK"),          # Bass Drum
}

# Drum kit layout positions (x, y, radius)
DRUM_POSITIONS = {
    38: (700, 500, 80),   # Snare - center
    48: (550, 350, 60),   # Tom 1 - left-up
    45: (850, 350, 60),   # Tom 2 - right-up
    41: (700, 650, 70),   # Floor - center-down
    42: (400, 250, 70),   # Hi-Hat - far left-up
    46: (400, 250, 70),   # Hi-Hat Open - same position
    49: (1000, 200, 75),  # Crash - far right-up
    51: (700, 200, 75),   # Ride - center-up
    36: (700, 780, 60),   # Kick - bottom
}

# Hit effect class
class HitEffect:
    def __init__(self, note, velocity, x, y, radius):
        self.note = note
        self.velocity = velocity
        self.x = x
        self.y = y
        self.radius = radius
        self.current_radius = radius
        self.alpha = 255
        self.lifetime = 30  # frames
        self.age = 0
        
    def update(self):
        self.age += 1
        progress = self.age / self.lifetime
        self.current_radius = self.radius + progress * 40
        self.alpha = int(255 * (1 - progress))
        return self.age < self.lifetime
    
    def draw(self, screen):
        if self.note in ZONE_COLORS:
            color = ZONE_COLORS[self.note][0]
            # Draw expanding circle
            s = pygame.Surface((self.current_radius * 2, self.current_radius * 2), pygame.SRCALPHA)
            alpha_color = (*color, self.alpha)
            pygame.draw.circle(s, alpha_color, (self.current_radius, self.current_radius), 
                             int(self.current_radius), 3)
            screen.blit(s, (self.x - self.current_radius, self.y - self.current_radius))

# Global state
hit_effects = []
recent_hits = deque(maxlen=10)
stick_left_status = {"connected": False, "last_hit": 0, "zone": "None"}
stick_right_status = {"connected": False, "last_hit": 0, "zone": "None"}
kick_status = {"connected": False, "last_hit": 0}
hihat_status = {"connected": False, "last_hit": 0, "state": "OPEN"}
server_running = False
clients_connected = 0

# TCP Server
class DrumTCPServer:
    def __init__(self, host='0.0.0.0', port=6000):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.clients = []
        
    def start(self):
        global server_running, clients_connected
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)
        self.running = True
        server_running = True
        
        print(f"🌐 TCP Server listening on {self.host}:{self.port}")
        
        # Accept connections
        accept_thread = threading.Thread(target=self._accept_connections, daemon=True)
        accept_thread.start()
        
    def _accept_connections(self):
        global clients_connected
        while self.running:
            try:
                client_socket, client_address = self.server_socket.accept()
                print(f"✅ Client connected: {client_address}")
                clients_connected += 1
                
                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, client_address),
                    daemon=True
                )
                client_thread.start()
                self.clients.append(client_socket)
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Accept error: {e}")
                    
    def _handle_client(self, client_socket, client_address):
        global clients_connected, hit_effects, recent_hits
        global stick_left_status, stick_right_status, kick_status, hihat_status
        
        try:
            while self.running:
                # Read MIDI message (3 bytes)
                data = client_socket.recv(1024)
                if not data:
                    break
                
                # Process each 3-byte MIDI message
                i = 0
                while i + 2 < len(data):
                    status = data[i]
                    note = data[i + 1]
                    velocity = data[i + 2]
                    
                    # Check if it's a Note On message (0x90 - 0x9F)
                    if (status & 0xF0) == 0x90 and velocity > 0:
                        print(f"🥁 HIT! Note: {note}, Velocity: {velocity}")
                        
                        # Add hit effect
                        if note in DRUM_POSITIONS:
                            x, y, radius = DRUM_POSITIONS[note]
                            hit_effects.append(HitEffect(note, velocity, x, y, radius))
                            
                            # Record hit
                            zone_name = ZONE_COLORS.get(note, (WHITE, "Unknown"))[1]
                            recent_hits.append({
                                'time': time.time(),
                                'note': note,
                                'velocity': velocity,
                                'zone': zone_name
                            })
                            
                            # Update status based on note
                            if note in [38, 48, 45, 41, 49, 51]:  # Drum/cymbal notes
                                # Determine which stick (simple heuristic)
                                if note in [48, 42]:  # Left side
                                    stick_left_status["last_hit"] = time.time()
                                    stick_left_status["zone"] = zone_name
                                    stick_left_status["connected"] = True
                                else:  # Right side or center
                                    stick_right_status["last_hit"] = time.time()
                                    stick_right_status["zone"] = zone_name
                                    stick_right_status["connected"] = True
                            elif note == 36:  # Kick
                                kick_status["last_hit"] = time.time()
                                kick_status["connected"] = True
                            elif note in [42, 46]:  # Hi-hat
                                hihat_status["last_hit"] = time.time()
                                hihat_status["state"] = "CLOSED" if note == 42 else "OPEN"
                                hihat_status["connected"] = True
                    
                    i += 3
                    
        except Exception as e:
            print(f"Client {client_address} error: {e}")
        finally:
            client_socket.close()
            clients_connected -= 1
            print(f"❌ Client disconnected: {client_address}")
            
    def stop(self):
        self.running = False
        if self.server_socket:
            self.server_socket.close()

# Start server
server = DrumTCPServer()
server.start()

# Draw functions
def draw_drum_kit():
    """Draw the drum kit layout"""
    for note, (x, y, radius) in DRUM_POSITIONS.items():
        if note == 46:  # Skip duplicate hi-hat
            continue
            
        # Draw drum circle
        color = ZONE_COLORS.get(note, (WHITE, "Unknown"))[0]
        
        # Base circle
        pygame.draw.circle(screen, DARK_GRAY, (x, y), radius + 3)
        pygame.draw.circle(screen, color, (x, y), radius, 2)
        
        # Inner circle
        pygame.draw.circle(screen, DARK_GRAY, (x, y), radius - 10, 1)
        
        # Label
        name = ZONE_COLORS.get(note, (WHITE, "Unknown"))[1]
        text = font_small.render(name, True, WHITE)
        text_rect = text.get_rect(center=(x, y))
        screen.blit(text, text_rect)
        
        # Note number
        note_text = font_small.render(f"#{note}", True, GRAY)
        note_rect = note_text.get_rect(center=(x, y + 20))
        screen.blit(note_text, note_rect)

def draw_hit_effects():
    """Draw all active hit effects"""
    global hit_effects
    hit_effects = [effect for effect in hit_effects if effect.update()]
    for effect in hit_effects:
        effect.draw(screen)

def draw_status_panel():
    """Draw status information panel"""
    panel_x = 20
    panel_y = 20
    
    # Title
    title = font_large.render("AIR DRUMS - LIVE", True, GREEN)
    screen.blit(title, (panel_x, panel_y))
    
    y_offset = panel_y + 60
    
    # Server status
    server_text = f"Server: {'RUNNING' if server_running else 'STOPPED'}"
    server_color = GREEN if server_running else RED
    text = font_medium.render(server_text, True, server_color)
    screen.blit(text, (panel_x, y_offset))
    y_offset += 40
    
    # Clients connected
    clients_text = f"Clients: {clients_connected}"
    text = font_medium.render(clients_text, True, WHITE)
    screen.blit(text, (panel_x, y_offset))
    y_offset += 50
    
    # Device status
    current_time = time.time()
    
    # Left Stick
    left_active = (current_time - stick_left_status["last_hit"]) < 2.0 if stick_left_status["last_hit"] > 0 else False
    left_color = GREEN if left_active else GRAY
    left_text = f"LEFT STICK: {stick_left_status['zone']}"
    text = font_small.render(left_text, True, left_color)
    screen.blit(text, (panel_x, y_offset))
    y_offset += 30
    
    # Right Stick
    right_active = (current_time - stick_right_status["last_hit"]) < 2.0 if stick_right_status["last_hit"] > 0 else False
    right_color = GREEN if right_active else GRAY
    right_text = f"RIGHT STICK: {stick_right_status['zone']}"
    text = font_small.render(right_text, True, right_color)
    screen.blit(text, (panel_x, y_offset))
    y_offset += 30
    
    # Kick
    kick_active = (current_time - kick_status["last_hit"]) < 2.0 if kick_status["last_hit"] > 0 else False
    kick_color = GREEN if kick_active else GRAY
    kick_text = f"KICK PEDAL: {'ACTIVE' if kick_active else 'IDLE'}"
    text = font_small.render(kick_text, True, kick_color)
    screen.blit(text, (panel_x, y_offset))
    y_offset += 30
    
    # Hi-Hat
    hihat_active = (current_time - hihat_status["last_hit"]) < 2.0 if hihat_status["last_hit"] > 0 else False
    hihat_color = GREEN if hihat_active else GRAY
    hihat_text = f"HI-HAT: {hihat_status['state']}"
    text = font_small.render(hihat_text, True, hihat_color)
    screen.blit(text, (panel_x, y_offset))

def draw_recent_hits():
    """Draw recent hits log"""
    panel_x = WIDTH - 350
    panel_y = 20
    
    # Title
    title = font_medium.render("RECENT HITS", True, CYAN)
    screen.blit(title, (panel_x, panel_y))
    
    y_offset = panel_y + 50
    current_time = time.time()
    
    for hit in reversed(list(recent_hits)):
        age = current_time - hit['time']
        if age > 5.0:  # Only show last 5 seconds
            continue
            
        # Color based on age
        alpha = max(0, 255 - int(age * 50))
        
        # Hit info
        zone = hit['zone']
        vel = hit['velocity']
        color = ZONE_COLORS.get(hit['note'], (WHITE, "Unknown"))[0]
        
        # Draw
        hit_text = f"{zone}: {vel}"
        text = font_small.render(hit_text, True, color)
        screen.blit(text, (panel_x, y_offset))
        
        # Velocity bar
        bar_width = int((vel / 127) * 200)
        pygame.draw.rect(screen, color, (panel_x + 150, y_offset + 5, bar_width, 15))
        pygame.draw.rect(screen, GRAY, (panel_x + 150, y_offset + 5, 200, 15), 1)
        
        y_offset += 35
        
        if y_offset > HEIGHT - 50:
            break

def draw_instructions():
    """Draw instructions"""
    instructions = [
        "Connect your ESP32 devices to this computer",
        "Devices will auto-connect on port 6000",
        "Start drumming to see live visualization!",
    ]
    
    y_offset = HEIGHT - 100
    for instruction in instructions:
        text = font_small.render(instruction, True, GRAY)
        text_rect = text.get_rect(center=(WIDTH // 2, y_offset))
        screen.blit(text, text_rect)
        y_offset += 25

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Clear screen
    screen.fill(BLACK)
    
    # Draw everything
    draw_drum_kit()
    draw_hit_effects()
    draw_status_panel()
    draw_recent_hits()
    draw_instructions()
    
    # Update display
    pygame.display.flip()
    clock.tick(60)  # 60 FPS

# Cleanup
server.stop()
pygame.quit()
sys.exit()
