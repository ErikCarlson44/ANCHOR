"""
=============================================================================
RD-03D RADAR DISPLAY - PC Application (Tkinter)
=============================================================================
360° radar visualization with servo sweep.
Reads data from Pico via USB serial.

Usage:
  1. Upload pico_radar_servo.py to Pico as code.py
  2. Close Thonny
  3. Run: python radar_display.py COM5

Requirements:
  pip install pyserial
=============================================================================
"""

import tkinter as tk
from tkinter import Canvas
import math
import serial
import json
import sys
import time

class RadarDisplay:
    def __init__(self, port):
        self.root = tk.Tk()
        self.root.title("RD-03D Radar Display - 360°")
        self.root.configure(bg='black')
        
        self.width = 950
        self.height = 750
        self.root.geometry(f"{self.width}x{self.height}")
        
        self.canvas = Canvas(self.root, width=self.width, height=self.height, 
                           bg='black', highlightthickness=0)
        self.canvas.pack()
        
        # Radar settings
        self.info_panel_width = 200
        self.center_x = self.info_panel_width + (self.width - self.info_panel_width) // 2
        self.center_y = self.height // 2
        self.radar_radius = min(self.width - self.info_panel_width - 80, self.height - 80) // 2
        self.max_range = 12000  # 12 meters in mm
        
        # Colors
        self.GREEN = '#00ff00'
        self.DARK_GREEN = '#006400'
        self.BRIGHT_GREEN = '#00ff64'
        self.RED = '#ff4444'
        self.YELLOW = '#ffff00'
        self.CYAN = '#00ffff'
        self.WHITE = '#ffffff'
        self.GRAY = '#888888'
        
        # Serial
        try:
            self.serial = serial.Serial(port, 115200, timeout=0.1)
            print(f"Connected to {port}")
        except serial.SerialException as e:
            print(f"Error: {e}")
            print("Make sure Thonny is closed!")
            sys.exit(1)
        
        # Targets
        self.targets = {}
        self.target_lifetime = 2.0
        
        # State
        self.servo_angle = 90
        self.latest_dist = None
        self.latest_x = None
        self.latest_y = None
        
        # Controls
        self.root.bind('<Escape>', lambda e: self.root.quit())
        self.root.bind('<plus>', lambda e: self.adjust_range(1))
        self.root.bind('<equal>', lambda e: self.adjust_range(1))
        self.root.bind('<minus>', lambda e: self.adjust_range(-1))
        
        self.update()
        
    def adjust_range(self, delta):
        new_range = self.max_range / 1000 + delta
        self.max_range = max(1000, min(20000, new_range * 1000))
        
    def distance_to_pixels(self, dist_mm):
        return int((dist_mm / self.max_range) * self.radar_radius)
    
    def polar_to_xy(self, dist_mm, angle_deg):
        pixel_r = self.distance_to_pixels(dist_mm)
        rad = math.radians(angle_deg - 90)
        x = self.center_x + pixel_r * math.cos(rad)
        y = self.center_y + pixel_r * math.sin(rad)
        return x, y
    
    def draw_radar(self):
        self.canvas.delete("all")
        
        # Title
        self.canvas.create_text(self.center_x, 25, text="RD-03D RADAR - 360° VIEW",
                              fill=self.CYAN, font=('Consolas', 16, 'bold'))
        
        # Range circles
        max_range_m = int(self.max_range / 1000)
        for r in range(1, max_range_m + 1):
            radius = self.distance_to_pixels(r * 1000)
            x1 = self.center_x - radius
            y1 = self.center_y - radius
            x2 = self.center_x + radius
            y2 = self.center_y + radius
            self.canvas.create_oval(x1, y1, x2, y2, outline=self.DARK_GREEN, width=1)
            self.canvas.create_text(self.center_x + 5, self.center_y - radius - 5,
                                  text=f"{r}m", fill=self.DARK_GREEN, 
                                  font=('Consolas', 9), anchor='w')
        
        # Angle lines
        for angle in range(0, 360, 30):
            x, y = self.polar_to_xy(self.max_range, angle)
            color = self.GREEN if angle % 90 == 0 else self.DARK_GREEN
            width = 2 if angle % 90 == 0 else 1
            self.canvas.create_line(self.center_x, self.center_y, x, y,
                                  fill=color, width=width)
            lx, ly = self.polar_to_xy(self.max_range + 150, angle)
            labels = {0: "N", 90: "E", 180: "S", 270: "W"}
            label = labels.get(angle, f"{angle}°")
            self.canvas.create_text(lx, ly, text=label,
                                  fill=self.DARK_GREEN, font=('Consolas', 10))
        
        # Center boat
        self.canvas.create_oval(self.center_x - 6, self.center_y - 6,
                              self.center_x + 6, self.center_y + 6,
                              fill=self.GREEN, outline=self.WHITE, width=2)
        self.canvas.create_polygon(
            self.center_x, self.center_y - 12,
            self.center_x - 6, self.center_y - 2,
            self.center_x + 6, self.center_y - 2,
            fill=self.CYAN, outline=self.WHITE
        )
        
        # Sweep line
        sweep_x, sweep_y = self.polar_to_xy(self.max_range, self.servo_angle)
        self.canvas.create_line(self.center_x, self.center_y, sweep_x, sweep_y,
                              fill=self.BRIGHT_GREEN, width=3)
        
        # Sweep trail
        for i in range(1, 15):
            trail_angle = (self.servo_angle - i * 3) % 360
            tx, ty = self.polar_to_xy(self.max_range, trail_angle)
            intensity = max(0, 100 - i * 7)
            color = f'#{0:02x}{intensity:02x}{0:02x}'
            self.canvas.create_line(self.center_x, self.center_y, tx, ty,
                                  fill=color, width=max(1, 3 - i//4))
        
        # Draw targets
        current_time = time.time()
        to_remove = []
        for key, (dist, angle, timestamp) in self.targets.items():
            age = current_time - timestamp
            if age > self.target_lifetime:
                to_remove.append(key)
                continue
            
            fade = 1.0 - (age / self.target_lifetime)
            x, y = self.polar_to_xy(dist, angle)
            
            size = int(6 * fade) + 4
            r_color = int(255 * fade)
            g_color = int(80 * fade)
            color = f'#{r_color:02x}{g_color:02x}{g_color:02x}'
            self.canvas.create_oval(x - size, y - size, x + size, y + size,
                                  fill=color, outline=self.WHITE)
            
            if age < 0.5:
                dist_m = dist / 1000
                self.canvas.create_text(x + 15, y, text=f"{dist_m:.2f}m",
                                      fill=self.WHITE, font=('Consolas', 9), anchor='w')
        
        for key in to_remove:
            del self.targets[key]
        
        self.draw_info_panel()
    
    def draw_info_panel(self):
        px, py = 15, 50
        
        self.canvas.create_rectangle(10, 40, 195, 400,
                                   fill='#111111', outline=self.DARK_GREEN)
        
        self.canvas.create_text(px, py, text="RADAR INFO",
                              fill=self.WHITE, font=('Consolas', 12, 'bold'), anchor='w')
        py += 30
        
        self.canvas.create_text(px, py, text=f"Servo: {self.servo_angle}°",
                              fill=self.CYAN, font=('Consolas', 11), anchor='w')
        py += 22
        
        self.canvas.create_text(px, py, text=f"Range: {self.max_range/1000:.0f}m",
                              fill=self.GRAY, font=('Consolas', 11), anchor='w')
        py += 22
        
        self.canvas.create_text(px, py, text=f"Targets: {len(self.targets)}",
                              fill=self.YELLOW, font=('Consolas', 11), anchor='w')
        py += 35
        
        self.canvas.create_line(px, py, 175, py, fill=self.DARK_GREEN)
        py += 15
        
        if self.latest_dist:
            self.canvas.create_text(px, py, text="LATEST:",
                                  fill=self.BRIGHT_GREEN, font=('Consolas', 10), anchor='w')
            py += 20
            dist_m = self.latest_dist / 1000
            self.canvas.create_text(px + 10, py, text=f"Dist: {dist_m:.2f} m",
                                  fill=self.WHITE, font=('Consolas', 10), anchor='w')
            py += 18
            self.canvas.create_text(px + 10, py, text=f"Angle: {self.servo_angle}°",
                                  fill=self.WHITE, font=('Consolas', 10), anchor='w')
            py += 18
            if self.latest_x is not None:
                self.canvas.create_text(px + 10, py, text=f"X: {self.latest_x} mm",
                                      fill=self.WHITE, font=('Consolas', 10), anchor='w')
                py += 18
                self.canvas.create_text(px + 10, py, text=f"Y: {self.latest_y} mm",
                                      fill=self.WHITE, font=('Consolas', 10), anchor='w')
        else:
            self.canvas.create_text(px, py, text="No target",
                                  fill=self.GRAY, font=('Consolas', 10), anchor='w')
        
        self.canvas.create_text(px, 360, text="+/- Range | ESC Quit",
                              fill=self.GRAY, font=('Consolas', 9), anchor='w')
    
    def read_serial(self):
        try:
            while self.serial.in_waiting:
                line = self.serial.readline().decode().strip()
                if line:
                    try:
                        data = json.loads(line)
                        self.servo_angle = data.get("servo", 90)
                        dist = data.get("dist")
                        self.latest_x = data.get("x")
                        self.latest_y = data.get("y")
                        target_num = data.get("target", 0)
                        
                        if dist and dist <= self.max_range:
                            self.latest_dist = dist
                            key = f"{self.servo_angle}_{target_num}"
                            self.targets[key] = (dist, self.servo_angle, time.time())
                        elif target_num == 0:
                            self.latest_dist = None
                    except json.JSONDecodeError:
                        pass
        except:
            pass
    
    def update(self):
        self.read_serial()
        self.draw_radar()
        self.root.after(33, self.update)
    
    def run(self):
        self.root.mainloop()
        self.serial.close()


def main():
    if len(sys.argv) < 2:
        print("Usage: python radar_display.py COM5")
        sys.exit(1)
    
    port = sys.argv[1]
    print(f"Starting radar display on {port}...")
    print("Controls: +/- range, ESC quit")
    
    display = RadarDisplay(port)
    display.run()


if __name__ == "__main__":
    main()
