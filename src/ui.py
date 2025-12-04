from typing import Callable, Dict
import tkinter as tk
from tkinter import ttk
import math


class MidiUI:
    """Interactive drum kit UI that visualizes MIDI notes as a real drum kit with animations."""

    # Color scheme - modern dark theme with vibrant neon accents
    BG_DARK = "#0d0d0d"
    BG_STAGE = "#16213e"
    BG_GRADIENT_TOP = "#1a1a2e"
    BG_GRADIENT_BOT = "#0f3460"
    DRUM_DEFAULT = "#2d3436"
    DRUM_KICK = "#ff3838"      # Vibrant red
    DRUM_SNARE = "#00d9ff"    # Cyan blue
    DRUM_HIHAT = "#ffd700"    # Gold
    DRUM_TOM = "#a855f7"      # Purple
    DRUM_CYMBAL = "#10b981"   # Emerald green
    DRUM_HIT = "#ffffff"
    GLOW_COLOR = "#00ffff"    # Neon cyan glow
    TEXT_PRIMARY = "#ffffff"
    TEXT_SECONDARY = "#9ca3af"
    ACCENT_PRIMARY = "#00d9ff"

    def __init__(self, on_close: Callable[[], None]):
        self.root = tk.Tk()
        self.root.title("🥁 ESP32 Air Drums")
        
        # Start in windowed mode
        self.root.geometry("1200x800")
        self.root.configure(bg=self.BG_DARK)
        self.on_close = on_close
        
        # Fullscreen toggle
        self.fullscreen = False
        self.root.bind('<F11>', self._toggle_fullscreen)
        self.root.bind('<Escape>', self._exit_fullscreen)

        # Track animation states
        self.drum_animations: Dict[str, int] = {}
        # Track active ripple item ids per drum so we can clean them up
        self._active_ripples: Dict[str, list] = {}
        
        # Main container
        main_frame = tk.Frame(self.root, bg=self.BG_DARK)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Title bar
        title_frame = tk.Frame(main_frame, bg=self.BG_DARK, height=60)
        title_frame.pack(fill=tk.X, pady=(0, 15))
        
        title_label = tk.Label(
            title_frame,
            text="🥁 ESP32 AIR DRUMS",
            font=("Segoe UI", 24, "bold"),
            bg=self.BG_DARK,
            fg=self.ACCENT_PRIMARY,
        )
        title_label.pack(side=tk.LEFT)
        
        # Metrics display (FPS, hits, etc.)
        metrics_frame = tk.Frame(title_frame, bg=self.BG_DARK)
        metrics_frame.pack(side=tk.RIGHT, padx=10)
        
        self.status_var = tk.StringVar(value="⚡ Ready")
        status_label = tk.Label(
            metrics_frame,
            textvariable=self.status_var,
            font=("Consolas", 11, "bold"),
            bg=self.BG_DARK,
            fg=self.TEXT_PRIMARY,
        )
        status_label.pack(anchor=tk.E)
        
        self.metrics_var = tk.StringVar(value="Press F11 for fullscreen")
        metrics_label = tk.Label(
            metrics_frame,
            textvariable=self.metrics_var,
            font=("Consolas", 9),
            bg=self.BG_DARK,
            fg=self.TEXT_SECONDARY,
        )
        metrics_label.pack(anchor=tk.E)

        # Drum kit canvas (main stage) - will resize with window
        self.canvas = tk.Canvas(
            main_frame,
            bg=self.BG_STAGE,
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Bind resize event to redraw drum kit dynamically
        self.canvas.bind('<Configure>', self._on_resize)
        self.drum_kit_created = False

        # Draw the drum kit (will be drawn on first resize)
        # self._create_drum_kit()

        # Info panel with velocity meter
        info_frame = tk.Frame(main_frame, bg=self.BG_DARK)
        info_frame.pack(fill=tk.X, pady=(5, 0))
        
        # Left side - drum info
        info_left = tk.Frame(info_frame, bg=self.BG_DARK)
        info_left.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.info_var = tk.StringVar(value="🎵 Waiting for MIDI input...")
        info_label = tk.Label(
            info_left,
            textvariable=self.info_var,
            font=("Segoe UI", 12, "bold"),
            bg=self.BG_DARK,
            fg=self.TEXT_PRIMARY,
            anchor=tk.W,
        )
        info_label.pack(fill=tk.X)
        
        # Velocity meter canvas
        self.velocity_canvas = tk.Canvas(
            info_left,
            height=8,
            bg=self.BG_DARK,
            highlightthickness=0,
        )
        self.velocity_canvas.pack(fill=tk.X, pady=(5, 0))
        self.velocity_bar = None
        
        # Right side - quit button
        quit_btn = tk.Button(
            info_frame,
            text="✕ EXIT",
            command=self._on_quit,
            font=("Segoe UI", 10, "bold"),
            bg="#dc2626",
            fg=self.TEXT_PRIMARY,
            activebackground="#ef4444",
            activeforeground=self.TEXT_PRIMARY,
            relief=tk.FLAT,
            padx=20,
            pady=10,
            cursor="hand2",
        )
        quit_btn.pack(side=tk.RIGHT, padx=10)

        self.root.protocol("WM_DELETE_WINDOW", self._on_quit)
    
    def _toggle_fullscreen(self, event=None) -> None:
        """Toggle fullscreen mode."""
        self.fullscreen = not self.fullscreen
        self.root.attributes('-fullscreen', self.fullscreen)
        return 'break'
    
    def _exit_fullscreen(self, event=None) -> None:
        """Exit fullscreen mode."""
        if self.fullscreen:
            self.fullscreen = False
            self.root.attributes('-fullscreen', False)
        return 'break'
    
    def _on_resize(self, event=None) -> None:
        """Handle window resize - redraw drum kit to fit new size."""
        # Only redraw if canvas has meaningful size
        if event and event.width > 100 and event.height > 100:
            self.canvas.delete('all')
            self._draw_background_gradient()
            self._create_drum_kit()
            self.drum_kit_created = True
    
    def _draw_background_gradient(self) -> None:
        """Draw subtle gradient background on stage."""
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        
        # Create radial gradient effect with rectangles
        steps = 40
        for i in range(steps):
            # Interpolate from center (lighter) to edges (darker)
            ratio = i / steps
            # Simple color blend from BG_GRADIENT_TOP to BG_GRADIENT_BOT
            r1, g1, b1 = self._hex_to_rgb(self.BG_GRADIENT_TOP)
            r2, g2, b2 = self._hex_to_rgb(self.BG_GRADIENT_BOT)
            
            r = int(r1 + (r2 - r1) * ratio)
            g = int(g1 + (g2 - g1) * ratio)
            b = int(b1 + (b2 - b1) * ratio)
            
            color = f"#{r:02x}{g:02x}{b:02x}"
            
            # Draw horizontal strip
            y1 = (h / steps) * i
            y2 = (h / steps) * (i + 1)
            self.canvas.create_rectangle(
                0, y1, w, y2,
                fill=color,
                outline=""
            )

    def _create_drum_kit(self) -> None:
        """Draw the drum kit layout on canvas - scaled to current window size."""
        # Get actual canvas dimensions
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        
        # Scale everything based on current size (baseline: 900x600)
        scale = min(w / 900, h / 600)
        
        # Cymbals at the top
        # Crash left
        self.crash_id = self._draw_cymbal(
            150 * scale, 100 * scale, 70 * scale, "CRASH\n49", self.DRUM_CYMBAL
        )
        # Ride right
        self.ride_id = self._draw_cymbal(
            w - 150 * scale, 100 * scale, 70 * scale, "RIDE\n51", self.DRUM_CYMBAL
        )
        # Hi-hat top left
        self.hihat_id = self._draw_cymbal(
            220 * scale, 200 * scale, 50 * scale, "HI-HAT\n42/46", self.DRUM_HIHAT
        )
        
        # Toms in the middle-top
        center_x = w / 2
        self.tom_high_id = self._draw_drum(
            center_x - 40 * scale, 180 * scale, 60 * scale, "TOM H\n50", self.DRUM_TOM
        )
        self.tom_mid_id = self._draw_drum(
            center_x + 80 * scale, 180 * scale, 70 * scale, "TOM M\n48", self.DRUM_TOM
        )
        self.tom_low_id = self._draw_drum(
            w - 220 * scale, 250 * scale, 80 * scale, "TOM L\n45", self.DRUM_TOM
        )
        
        # Snare center
        self.snare_id = self._draw_drum(
            center_x, h * 0.5, 90 * scale, "SNARE\n38", self.DRUM_SNARE
        )
        
        # Kick drum at bottom center (largest)
        self.kick_id = self._draw_drum(
            center_x, h * 0.75, 110 * scale, "KICK\n36", self.DRUM_KICK
        )
        
        # Map MIDI notes to canvas objects
        self.note_to_drum = {
            36: ("kick", self.kick_id),
            35: ("kick", self.kick_id),
            38: ("snare", self.snare_id),
            40: ("snare", self.snare_id),
            42: ("hihat", self.hihat_id),
            44: ("hihat", self.hihat_id),
            46: ("hihat", self.hihat_id),
            49: ("crash", self.crash_id),
            52: ("crash", self.crash_id),
            51: ("ride", self.ride_id),
            53: ("ride", self.ride_id),
            50: ("tom_high", self.tom_high_id),
            48: ("tom_mid", self.tom_mid_id),
            47: ("tom_low", self.tom_low_id),
            45: ("tom_low", self.tom_low_id),
            43: ("tom_low", self.tom_low_id),
            41: ("tom_low", self.tom_low_id),
        }

    def _draw_drum(self, x: float, y: float, radius: float, label: str, color: str) -> list:
        """Draw a drum (circle with label and shadow)."""
        # Shadow (offset slightly)
        shadow = self.canvas.create_oval(
            x - radius + 4, y - radius + 4,
            x + radius + 4, y + radius + 4,
            fill="#000000",
            outline="",
        )
        
        # Main drum circle
        drum = self.canvas.create_oval(
            x - radius, y - radius,
            x + radius, y + radius,
            fill=color,
            outline=self.DRUM_HIT,
            width=4,
        )
        
        # Inner highlight for 3D effect
        highlight = self.canvas.create_oval(
            x - radius * 0.7, y - radius * 0.7,
            x + radius * 0.7, y + radius * 0.7,
            fill="",
            outline=self._lighten_color(color, 0.3),
            width=2,
        )
        
        # Label with better font
        text = self.canvas.create_text(
            x, y,
            text=label,
            font=("Segoe UI", int(radius / 5), "bold"),
            fill=self.TEXT_PRIMARY,
        )
        
        return [drum, text, shadow, highlight]

    def _draw_cymbal(self, x: float, y: float, radius: float, label: str, color: str) -> list:
        """Draw a cymbal (ellipse with shadow and highlight)."""
        # Shadow
        shadow = self.canvas.create_oval(
            x - radius * 1.3 + 3, y - radius * 0.4 + 3,
            x + radius * 1.3 + 3, y + radius * 0.4 + 3,
            fill="#000000",
            outline="",
        )
        
        # Cymbal ellipse (wider than tall)
        cymbal = self.canvas.create_oval(
            x - radius * 1.3, y - radius * 0.4,
            x + radius * 1.3, y + radius * 0.4,
            fill=color,
            outline=self.DRUM_HIT,
            width=4,
        )
        
        # Shine effect
        highlight = self.canvas.create_oval(
            x - radius * 0.9, y - radius * 0.15,
            x + radius * 0.9, y + radius * 0.15,
            fill="",
            outline=self._lighten_color(color, 0.4),
            width=2,
        )
        
        # Label
        text = self.canvas.create_text(
            x, y,
            text=label,
            font=("Segoe UI", int(radius / 5.5), "bold"),
            fill=self.TEXT_PRIMARY,
        )
        
        return [cymbal, text, shadow, highlight]

    def set_status(self, text: str) -> None:
        """Update connection status."""
        self.status_var.set(f"🔗 {text}")
        self.metrics_var.set("Press F11 for fullscreen • ESC to exit")

    def show_note(self, note: int, velocity: int) -> None:
        """Animate the drum hit when a MIDI note is received."""
        drum_info = self.note_to_drum.get(note)
        if not drum_info:
            # Unknown note - just update info
            self.info_var.set(f"🎵 Note {note} • Velocity {velocity}")
            self._update_velocity_meter(velocity)
            return
        
        drum_name, drum_objects = drum_info
        
        # Update info display
        drum_display = self._note_to_drum_name(note)
        note_name = self._note_to_name(note)
        self.info_var.set(f"🥁 {drum_display} • Note {note} ({note_name}) • Velocity {velocity}/127")
        
        # Update velocity meter
        self._update_velocity_meter(velocity)
        
        # Animate the drum hit
        self._animate_drum_hit(drum_name, drum_objects, velocity)
    
    def _update_velocity_meter(self, velocity: int) -> None:
        """Update the velocity meter bar."""
        w = self.velocity_canvas.winfo_width()
        h = self.velocity_canvas.winfo_height()
        
        if w < 10:  # Not yet rendered
            return
        
        # Clear previous bar
        self.velocity_canvas.delete('all')
        
        # Background
        self.velocity_canvas.create_rectangle(
            0, 0, w, h,
            fill="#1f2937",
            outline=""
        )
        
        # Velocity bar (colored based on intensity)
        bar_width = (velocity / 127.0) * w
        
        # Color gradient: green -> yellow -> red
        if velocity < 42:
            color = "#10b981"  # Green
        elif velocity < 85:
            color = "#fbbf24"  # Yellow
        else:
            color = "#ef4444"  # Red
        
        if bar_width > 0:
            self.velocity_canvas.create_rectangle(
                0, 0, bar_width, h,
                fill=color,
                outline=""
            )

    def _animate_drum_hit(self, drum_name: str, drum_objects: list, velocity: int) -> None:
        """Create smooth ripple and glow animation for drum hit."""
        # Cancel any existing animation for this drum
        if drum_name in self.drum_animations:
            try:
                self.root.after_cancel(self.drum_animations[drum_name])
            except Exception:
                # ignore if it's already fired or invalid
                pass
        # Remove any lingering ripple canvas items from a previous animation
        if drum_name in self._active_ripples:
            try:
                for old_r in self._active_ripples.pop(drum_name):
                    self.canvas.delete(old_r)
            except Exception:
                pass
        
        drum_obj = drum_objects[0]  # The main shape (oval)
        
        # Get original color
        if "kick" in drum_name:
            base_color = self.DRUM_KICK
        elif "snare" in drum_name:
            base_color = self.DRUM_SNARE
        elif "hihat" in drum_name:
            base_color = self.DRUM_HIHAT
        elif "tom" in drum_name:
            base_color = self.DRUM_TOM
        else:  # cymbal
            base_color = self.DRUM_CYMBAL
        
        # Get drum center for ripple
        coords = self.canvas.coords(drum_obj)
        cx = (coords[0] + coords[2]) / 2
        cy = (coords[1] + coords[3]) / 2
        radius = (coords[2] - coords[0]) / 2
        
        # Create expanding ripple circles
        ripple_ids = []
        num_ripples = 2
        for i in range(num_ripples):
            ripple = self.canvas.create_oval(
                cx - 5, cy - 5, cx + 5, cy + 5,
                outline=self.DRUM_HIT,
                width=3,
                fill=""
            )
            ripple_ids.append(ripple)
        # remember these ripples so we can delete them if a new hit arrives
        self._active_ripples[drum_name] = list(ripple_ids)
        
        # Animation parameters
        max_radius = radius * 1.8
        steps = 15
        step_count = [0]  # Use list to allow modification in nested function
        
        def animate_step():
            step_count[0] += 1
            progress = step_count[0] / steps
            
            if step_count[0] > steps:
                # Clean up ripples
                for ripple in ripple_ids:
                    try:
                        self.canvas.delete(ripple)
                    except Exception:
                        pass
                # also clear registry entry if present
                if drum_name in self._active_ripples:
                    try:
                        del self._active_ripples[drum_name]
                    except Exception:
                        pass
                
                # Final state - return to base color
                self.canvas.itemconfig(drum_obj, fill=base_color, width=3)
                
                # Clean up animation tracking
                if drum_name in self.drum_animations:
                    del self.drum_animations[drum_name]
                return
            
            # Smooth easing (ease-out)
            ease_progress = 1 - (1 - progress) ** 3
            
            # Animate ripples expanding and fading
            for idx, ripple in enumerate(ripple_ids):
                ripple_progress = ease_progress + (idx * 0.2)
                if ripple_progress > 1:
                    ripple_progress = 1
                
                r = 5 + (max_radius * ripple_progress)
                opacity = int(255 * (1 - ripple_progress))
                
                self.canvas.coords(
                    ripple,
                    cx - r, cy - r,
                    cx + r, cy + r
                )
                
                # Fade out ripple
                if opacity > 0:
                    ripple_color = f"#{opacity:02x}{opacity:02x}{opacity:02x}"
                    self.canvas.itemconfig(ripple, outline=ripple_color)
            
            # Drum color transition - flash to white then back
            if progress < 0.3:
                # Flash phase - brighten to white
                flash_amount = (progress / 0.3)
                intensity = int((velocity / 127.0) * 255)
                
                # Interpolate from base color to white
                base_rgb = self._hex_to_rgb(base_color)
                target_rgb = (intensity, intensity, intensity)
                
                r = int(base_rgb[0] + (target_rgb[0] - base_rgb[0]) * flash_amount)
                g = int(base_rgb[1] + (target_rgb[1] - base_rgb[1]) * flash_amount)
                b = int(base_rgb[2] + (target_rgb[2] - base_rgb[2]) * flash_amount)
                
                current_color = f"#{r:02x}{g:02x}{b:02x}"
            else:
                # Fade back phase
                fade_progress = (progress - 0.3) / 0.7
                intensity = int((velocity / 127.0) * 255 * (1 - fade_progress))
                
                base_rgb = self._hex_to_rgb(base_color)
                white_rgb = (intensity, intensity, intensity)
                
                r = int(white_rgb[0] + (base_rgb[0] - white_rgb[0]) * fade_progress)
                g = int(white_rgb[1] + (base_rgb[1] - white_rgb[1]) * fade_progress)
                b = int(white_rgb[2] + (base_rgb[2] - white_rgb[2]) * fade_progress)
                
                current_color = f"#{r:02x}{g:02x}{b:02x}"
            
            self.canvas.itemconfig(drum_obj, fill=current_color)
            
            # Outline pulse
            outline_width = 3 + int(3 * (velocity / 127.0) * (1 - ease_progress))
            self.canvas.itemconfig(drum_obj, width=outline_width)
            
            # Continue animation
            animation_id = self.root.after(16, animate_step)  # ~60 FPS
            self.drum_animations[drum_name] = animation_id
        
        # Start animation
        animate_step()
    
    def _hex_to_rgb(self, hex_color: str) -> tuple:
        """Convert hex color to RGB tuple."""
        hex_color = hex_color.lstrip('#')
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    
    def _lighten_color(self, hex_color: str, factor: float) -> str:
        """Lighten a hex color by a factor (0-1)."""
        r, g, b = self._hex_to_rgb(hex_color)
        r = min(255, int(r + (255 - r) * factor))
        g = min(255, int(g + (255 - g) * factor))
        b = min(255, int(b + (255 - b) * factor))
        return f"#{r:02x}{g:02x}{b:02x}"

    def _note_to_drum_name(self, note: int) -> str:
        """Convert MIDI note to drum name."""
        drum_map = {
            35: "BASS DRUM", 36: "KICK",
            38: "SNARE", 40: "E-SNARE",
            42: "HI-HAT CL", 44: "HI-HAT PD", 46: "HI-HAT OP",
            47: "TOM LOW-M", 48: "TOM HI-M", 50: "TOM HIGH",
            49: "CRASH", 51: "RIDE", 52: "CHINA", 53: "RIDE BELL",
            41: "TOM FLOOR", 43: "TOM FLOOR", 45: "TOM LOW",
        }
        return drum_map.get(note, f"NOTE {note}")

    def _note_to_name(self, note: int) -> str:
        """Convert MIDI note number to note name."""
        note_names = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
        octave = (note // 12) - 1
        name = note_names[note % 12]
        return f"{name}{octave}"

    def _on_quit(self) -> None:
        self.root.quit()
        try:
            self.on_close()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()
