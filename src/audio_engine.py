from typing import Dict
import numpy as np
import pygame.mixer
import os


class AudioEngine:
    """Audio engine that plays drum samples for MIDI notes.

    - Generates synthetic drum samples using numpy and pygame.mixer
    - Maps MIDI notes to different drum sounds (kick, snare, hihat, toms, etc.)
    - Keeps track of active Channels to stop on note_off.
    """

    def __init__(self, sample_rate: int = 44100) -> None:
        self.sample_rate = sample_rate
        pygame.mixer.init(frequency=sample_rate, size=-16, channels=2, buffer=512)
        self._active: Dict[int, pygame.mixer.Channel] = {}
        
        # Pre-generate drum samples for better performance
        self._drum_samples: Dict[str, pygame.mixer.Sound] = {}
        self._generate_drum_samples()
        
        # Map MIDI notes to drum sounds (General MIDI drum map)
        self._note_map = {
            35: "kick",      # Acoustic Bass Drum
            36: "kick",      # Bass Drum 1
            38: "snare",     # Acoustic Snare
            40: "snare",     # Electric Snare
            42: "hihat_closed",  # Closed Hi-Hat
            44: "hihat_closed",  # Pedal Hi-Hat
            46: "hihat_open",    # Open Hi-Hat
            47: "tom_low",   # Low-Mid Tom
            48: "tom_mid",   # Hi-Mid Tom
            50: "tom_high",  # High Tom
            49: "crash",     # Crash Cymbal 1
            51: "ride",      # Ride Cymbal 1
            52: "crash",     # Chinese Cymbal
            53: "ride",      # Ride Bell
            41: "tom_low",   # Low Floor Tom
            43: "tom_low",   # High Floor Tom
            45: "tom_mid",   # Low Tom
        }

    def _generate_drum_samples(self) -> None:
        """Generate synthetic drum samples using audio synthesis."""
        
        # Kick drum - deep bass thump with pitch envelope
        kick = self._create_kick()
        self._drum_samples["kick"] = kick
        
        # Snare drum - noise burst with tone
        snare = self._create_snare()
        self._drum_samples["snare"] = snare
        
        # Closed hi-hat - short metallic noise
        hihat_closed = self._create_hihat_closed()
        self._drum_samples["hihat_closed"] = hihat_closed
        
        # Open hi-hat - longer metallic noise
        hihat_open = self._create_hihat_open()
        self._drum_samples["hihat_open"] = hihat_open
        
        # Toms - tuned drums
        self._drum_samples["tom_low"] = self._create_tom(120)
        self._drum_samples["tom_mid"] = self._create_tom(180)
        self._drum_samples["tom_high"] = self._create_tom(250)
        
        # Crash cymbal - long metallic wash
        crash = self._create_crash()
        self._drum_samples["crash"] = crash
        
        # Ride cymbal - sustained metallic ping
        ride = self._create_ride()
        self._drum_samples["ride"] = ride

    def _create_kick(self) -> pygame.mixer.Sound:
        """Create a synthesized kick drum sound."""
        duration = 0.5
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # Pitch envelope - starts at 150Hz, drops to 40Hz
        freq_start = 150
        freq_end = 40
        freq = freq_start * np.exp(-8 * t)
        freq = np.maximum(freq, freq_end)
        
        # Generate tone
        phase = 2 * np.pi * np.cumsum(freq) / self.sample_rate
        wave = np.sin(phase)
        
        # Amplitude envelope - quick attack, exponential decay
        envelope = np.exp(-8 * t)
        wave = wave * envelope
        
        # Add click at the start for punch
        click = np.exp(-100 * t) * np.random.randn(samples) * 0.3
        wave = wave + click
        
        # Normalize and convert
        wave = wave / np.max(np.abs(wave)) * 0.9
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def _create_snare(self) -> pygame.mixer.Sound:
        """Create a synthesized snare drum sound."""
        duration = 0.2
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # Tone component (around 200Hz)
        tone = np.sin(2 * np.pi * 200 * t)
        tone = tone * np.exp(-15 * t)
        
        # Noise component (snare wires)
        noise = np.random.randn(samples)
        noise = noise * np.exp(-12 * t)
        
        # Mix tone and noise
        wave = tone * 0.3 + noise * 0.7
        
        # Normalize
        wave = wave / np.max(np.abs(wave)) * 0.85
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def _create_hihat_closed(self) -> pygame.mixer.Sound:
        """Create a closed hi-hat sound."""
        duration = 0.05
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # High-frequency noise (metallic)
        noise = np.random.randn(samples)
        
        # High-pass filter effect (emphasize high frequencies)
        noise = noise * (1 + 5 * np.sin(2 * np.pi * 8000 * t))
        
        # Sharp decay
        envelope = np.exp(-80 * t)
        wave = noise * envelope
        
        wave = wave / np.max(np.abs(wave)) * 0.6
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def _create_hihat_open(self) -> pygame.mixer.Sound:
        """Create an open hi-hat sound."""
        duration = 0.3
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # High-frequency noise
        noise = np.random.randn(samples)
        noise = noise * (1 + 3 * np.sin(2 * np.pi * 7000 * t))
        
        # Slower decay than closed
        envelope = np.exp(-8 * t)
        wave = noise * envelope
        
        wave = wave / np.max(np.abs(wave)) * 0.5
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def _create_tom(self, base_freq: float) -> pygame.mixer.Sound:
        """Create a tom drum sound at specified frequency."""
        duration = 0.4
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # Pitch envelope
        freq = base_freq * np.exp(-5 * t)
        phase = 2 * np.pi * np.cumsum(freq) / self.sample_rate
        wave = np.sin(phase)
        
        # Add harmonics for more realistic tone
        wave += 0.3 * np.sin(2 * phase)
        wave += 0.1 * np.sin(3 * phase)
        
        # Amplitude envelope
        envelope = np.exp(-7 * t)
        wave = wave * envelope
        
        wave = wave / np.max(np.abs(wave)) * 0.8
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def _create_crash(self) -> pygame.mixer.Sound:
        """Create a crash cymbal sound."""
        duration = 1.5
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # Complex noise with multiple frequency bands
        noise = np.random.randn(samples)
        
        # Add shimmer with modulation
        for freq in [3000, 5000, 8000, 12000]:
            noise = noise * (1 + 0.3 * np.sin(2 * np.pi * freq * t))
        
        # Long decay
        envelope = np.exp(-2 * t)
        wave = noise * envelope
        
        wave = wave / np.max(np.abs(wave)) * 0.6
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def _create_ride(self) -> pygame.mixer.Sound:
        """Create a ride cymbal sound."""
        duration = 1.0
        samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        
        # Metallic tone with noise
        tone = np.sin(2 * np.pi * 800 * t) + 0.5 * np.sin(2 * np.pi * 1600 * t)
        noise = np.random.randn(samples) * 0.3
        
        wave = tone + noise
        envelope = np.exp(-3 * t)
        wave = wave * envelope
        
        wave = wave / np.max(np.abs(wave)) * 0.65
        audio = (wave * (2 ** 15 - 1)).astype(np.int16)
        stereo = np.column_stack((audio, audio))
        return pygame.sndarray.make_sound(stereo)

    def note_on(self, note: int, velocity: int) -> None:
        """Play drum sample for the given MIDI note with velocity-sensitive volume."""
        # Map note to drum type
        drum_type = self._note_map.get(note, "snare")  # default to snare for unmapped notes
        
        # Get the sample
        sound = self._drum_samples.get(drum_type)
        if not sound:
            return
        
        # Adjust volume based on velocity with exponential curve for more natural dynamics
        # MIDI velocity ranges from 1-127
        # Use exponential curve to make quiet hits quieter and loud hits louder
        velocity_normalized = velocity / 127.0
        
        # Apply exponential curve (power of 2 makes it more responsive)
        volume = velocity_normalized ** 1.5
        
        # Ensure minimum audible volume for very soft hits
        volume = max(0.1, min(1.0, volume))
        
        sound.set_volume(volume)
        
        # Play the sound
        channel = sound.play()
        if channel:
            self._active[note] = channel

    def note_off(self, note: int) -> None:
        """Stop playing the drum sound (for cymbals/sustained sounds)."""
        channel = self._active.pop(note, None)
        if channel:
            try:
                channel.fadeout(100)  # Fade out over 100ms for smooth stop
            except Exception:
                pass