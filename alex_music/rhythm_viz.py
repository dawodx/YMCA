import pygame
import json
import time
import sys

# CONFIG
AUDIO_FILE = "alex_music/music/ymca_music.mp3"  # <--- UPDATE THIS PATH
JSON_FILE = "alex_music/music/ymca_music_analysis.json" # <--- UPDATE THIS PATH

# Load data
try:
    with open(JSON_FILE) as f:
        data = json.load(f)
    events = data["events"]
except FileNotFoundError:
    print("JSON file not found. Run analysis script first.")
    sys.exit()

pygame.init()
pygame.mixer.init()

# Setup Screen
WIDTH, HEIGHT = 600, 400
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Choreography Debugger")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Consolas", 24)

# Load Music
try:
    pygame.mixer.music.load(AUDIO_FILE)
except pygame.error:
    print(f"Could not load audio: {AUDIO_FILE}")
    sys.exit()

print("Starting playback...")
pygame.mixer.music.play()
start_time = time.time()

running = True
event_idx = 0
current_strength = 0.0
last_move_type = "WAITING"

while running:
    # 1. Handle Window Close
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 2. Get Sync Time
    # We use pygame.mixer.music.get_pos() for tighter sync than time.time()
    # get_pos returns milliseconds, so divide by 1000
    now = pygame.mixer.music.get_pos() / 1000.0

    # 3. Check for Beats
    target_strength = 0.1 # Resting state
    hit_beat = False
    
    # Process ALL events that have happened since the last frame
    while event_idx < len(events) and now >= events[event_idx]["timestamp"]:
        ev = events[event_idx]
        # We prioritize the "JUMP" if multiple events happen in one frame
        if ev["strength"] > 0.25:
            hit_beat = True
            last_move_type = ev["type"]
            current_strength = 1.0 # Force visual spike
        
        event_idx += 1

    # 4. Animation Physics (Decay)
    if not hit_beat:
        # Decay strength back to 0
        current_strength = max(0, current_strength * 0.90)

    # 5. Drawing
    screen.fill((20, 20, 20))
    
    # Draw the bar
    bar_height = int(current_strength * 300)
    
    # Color code: Red = Jump/Strong, Blue = Step/Weak
    color = (255, 50, 50) if last_move_type == "JUMP" else (0, 200, 255)
    
    # Center Bar
    pygame.draw.rect(screen, color, (WIDTH//2 - 50, HEIGHT - bar_height - 50, 100, bar_height))
    
    # Text info
    info = [
        f"Time: {now:.2f}s",
        f"Next Event: {event_idx}/{len(events)}",
        f"Last Move: {last_move_type}"
    ]
    
    for i, line in enumerate(info):
        text = font.render(line, True, (200, 200, 200))
        screen.blit(text, (10, 10 + i * 30))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()