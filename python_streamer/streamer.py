#!/usr/bin/env python3
"""
Debug WebSocket Client for ESP32 Camera
Minimal client to test WebSocket connection and frame reception
"""

import websocket
import time
import sys

class DebugClient:
    def __init__(self, esp32_ip, port=80):
        self.esp32_ip = esp32_ip
        self.port = port
        self.ws_url = f"ws://{esp32_ip}:{port}/stream"
        self.frame_count = 0
        self.start_time = None
        self.last_frame_time = None

    def on_message(self, ws, message):
        """Handle incoming messages"""
        current_time = time.time()
        
        if self.start_time is None:
            self.start_time = current_time
            
        self.frame_count += 1
        frame_size = len(message)
        
        # Calculate time since last frame
        time_since_last = 0
        if self.last_frame_time:
            time_since_last = current_time - self.last_frame_time
            
        self.last_frame_time = current_time
        
        # Calculate total elapsed time and average FPS
        elapsed = current_time - self.start_time
        avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        print(f"Frame {self.frame_count}: {frame_size} bytes, "
              f"gap: {time_since_last:.3f}s, avg FPS: {avg_fps:.2f}")
        
        # Save first few frames for inspection
        if self.frame_count <= 3:
            filename = f"debug_frame_{self.frame_count}.jpg"
            with open(filename, 'wb') as f:
                f.write(message)
            print(f"  -> Saved as {filename}")

    def on_error(self, ws, error):
        """Handle errors"""
        print(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        """Handle close"""
        print(f"WebSocket closed: {close_status_code} - {close_msg}")
        if self.start_time:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            print(f"Final stats: {self.frame_count} frames in {elapsed:.1f}s (avg {avg_fps:.2f} FPS)")

    def on_open(self, ws):
        """Handle open"""
        print(f"Connected to {self.ws_url}")
        print("Waiting for frames... (Press Ctrl+C to stop)")

    def run(self):
        """Run the debug client"""
        print(f"Connecting to ESP32 at {self.ws_url}")
        
        # Enable debug logging
        websocket.enableTrace(True)
        
        ws = websocket.WebSocketApp(
            self.ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        try:
            ws.run_forever(
                ping_interval=30,
                ping_timeout=10,
                ping_payload="ping"
            )
        except KeyboardInterrupt:
            print("\nStopping...")
            ws.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: python debug_client.py <ESP32_IP> [PORT]")
        print("Example: python debug_client.py 192.168.1.100")
        sys.exit(1)
    
    esp32_ip = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 80
    
    client = DebugClient(esp32_ip, port)
    client.run()

if __name__ == "__main__":
    main()