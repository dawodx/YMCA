import pygame
import json
import argparse
import time
import os

# Colors for groups (up to 10 distinct colors)
COLORS = [
    (255, 0, 0),    # Red
    (0, 255, 0),    # Green
    (0, 0, 255),    # Blue
    (255, 255, 0),  # Yellow
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Cyan
    (255, 165, 0),  # Orange
    (128, 0, 128),  # Purple
    (255, 192, 203),# Pink
    (128, 128, 128) # Grey
]

def main(json_path, audio_path):
    pygame.init()
    pygame.mixer.init()

    # Load data
    print(f"Loading JSON: {json_path}")
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    events = data['events']
    
    # Setup Window
    WIDTH, HEIGHT = 800, 600
    try:
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(f"Beat Visualizer - {os.path.basename(json_path)}")
    except pygame.error as e:
        print(f"Error setting up display: {e}")
        print("Note: This script requires a display environment to run.")
        return
    
    font = pygame.font.Font(None, 36)
    small_font = pygame.font.Font(None, 24)

    # Load Music
    if not os.path.exists(audio_path):
        print(f"Audio file not found: {audio_path}")
        return

    print(f"Playing Audio: {audio_path}")
    pygame.mixer.music.load(audio_path)
    pygame.mixer.music.play()
    
    # start_time = time.time()
    running = True
    
    print("controls: SPACE to Pause/Resume, ESC to Quit.")

    paused = False

    while running:
        # Standard Event Loop
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    if paused:
                        pygame.mixer.music.unpause()
                        paused = False
                    else:
                        pygame.mixer.music.pause()
                        paused = True

        screen.fill((20, 20, 20)) # Dark background

        if not paused:
            # Sync logic
            music_pos_ms = pygame.mixer.music.get_pos()
            
            # get_pos returns -1 if stopped, or time in ms
            if music_pos_ms == -1: 
                # End of song logic or not started
                pass
            else:
                sim_time = music_pos_ms / 1000.0

                # Visualization Logic: Scrolling Timeline
                # Center of screen is "now"
                
                center_x = WIDTH // 2
                px_per_sec = 200 # Zoom level (pixels per second)
                
                # Draw Static Center Line (The "Now" line)
                pygame.draw.line(screen, (255, 255, 255), (center_x, 0), (center_x, HEIGHT), 2)
                
                # Iterate through beats
                for e in events:
                    t = e['timestamp']
                    diff = t - sim_time
                    
                    # Optimization: Only draw visible beats (-2s to +2s window)
                    if -2.5 < diff < 2.5:
                        x = center_x + (diff * px_per_sec)
                        
                        group_id = e.get('group_id', 0)
                        # Fallback for old 'type' based JSON if group_id missing
                        if 'group_id' not in e and 'type' in e:
                             # Hash type string to int for color
                             group_id = hash(e['type']) % len(COLORS)
                             
                        color = COLORS[group_id % len(COLORS)]
                        
                        # Radius based on strength (0.0 to 1.0)
                        strength = e.get('strength', 0.5)
                        radius = int(15 + (strength * 25))
                        
                        # Y-Position based on pitch? 
                        # Low pitch -> lower on screen, High pitch -> higher
                        # If pitch not present, center.
                        pitch = e.get('pitch', 0.5)
                        y_pos = HEIGHT - int(pitch * (HEIGHT - 100)) - 50
                        
                        # Draw Beat Circle
                        pygame.draw.circle(screen, color, (int(x), y_pos), radius)
                        
                        # Draw connecting line to ground/center?
                        # pygame.draw.line(screen, color, (int(x), y_pos + radius), (int(x), HEIGHT), 1)

                        # Check if this beat is "Active" (crossing the center line)
                        if abs(diff) < 0.05: # 50ms window
                            # Flash effect: Draw ring
                            pygame.draw.circle(screen, (255, 255, 255), (int(x), y_pos), radius + 10, 4)
                            
                            # Display Info for active beat
                            label_str = f"Group {group_id}"
                            label = font.render(label_str, True, color)
                            screen.blit(label, (center_x + 20, 50))
                            
                            # Display Features
                            rms = e.get('loudness', 0)
                            cent = e.get('spectral_centroid', 0)
                            info_str = f"Time: {t:.2f}s | Str: {strength:.2f} | Pitch: {pitch:.2f}"
                            info_surf = small_font.render(info_str, True, (200, 200, 200))
                            screen.blit(info_surf, (center_x + 20, 90))

                # Display Current Time
                time_surf = small_font.render(f"Music Time: {sim_time:.2f}s", True, (255, 255, 255))
                screen.blit(time_surf, (10, 10))
                
                # Legend
                legend_y = HEIGHT - 30
                for i in range(5): # Show first 5 groups legend
                    if i < len(COLORS):
                        pygame.draw.circle(screen, COLORS[i], (20 + i*100, legend_y), 10)
                        l_text = small_font.render(f"Grp {i}", True, (200,200,200))
                        screen.blit(l_text, (35 + i*100, legend_y - 8))

        pygame.display.flip()
        
        # Limit FPS
        pygame.time.Clock().tick(60)

    pygame.quit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Rhythm Analysis")
    parser.add_argument("--json", required=True, help="Path to analysis JSON")
    parser.add_argument("--audio", required=True, help="Path to audio file")
    args = parser.parse_args()
    
    main(args.json, args.audio)
