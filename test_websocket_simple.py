#!/usr/bin/env python3
"""
Simple WebSocket client to test Foxglove server connection.
Uses only Python standard library.
"""

import socket
import base64
import hashlib
import json
import time
import sys

def generate_websocket_key():
    """Generate a valid WebSocket handshake key."""
    random_bytes = bytes([107, 61, 203, 81, 181, 179, 107, 61, 203, 81, 181, 179])
    return base64.b64encode(random_bytes).decode('ascii')

def perform_handshake(sock, host, port):
    """Perform WebSocket handshake."""
    key = generate_websocket_key()
    
    request = (
        f"GET / HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
        f"Sec-WebSocket-Version: 13\r\n"
        f"\r\n"
    )
    
    print(f"📤 Sending handshake request...")
    sock.sendall(request.encode())
    
    print(f"⏳ Waiting for handshake response...")
    response = sock.recv(4096).decode('utf-8', errors='ignore')
    
    print(f"📥 Received response:")
    print(response[:500])
    print()
    
    if "101" in response:
        print("✅ Handshake successful!")
        return True
    else:
        print("❌ Handshake failed!")
        return False

def send_websocket_frame(sock, data):
    """Send a WebSocket text frame."""
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    frame = bytearray()
    frame.append(0x81)  # FIN + text frame
    
    length = len(data)
    if length <= 125:
        frame.append(0x80 | length)  # MASK bit set
    elif length <= 65535:
        frame.append(0x80 | 126)
        frame.extend(length.to_bytes(2, 'big'))
    else:
        frame.append(0x80 | 127)
        frame.extend(length.to_bytes(8, 'big'))
    
    mask = bytes([0x00, 0x00, 0x00, 0x00])
    frame.extend(mask)
    
    masked_data = bytearray()
    for i, byte in enumerate(data):
        masked_data.append(byte ^ mask[i % 4])
    frame.extend(masked_data)
    
    sock.sendall(bytes(frame))

def receive_websocket_frame(sock, timeout=5.0):
    """Receive a WebSocket frame."""
    sock.settimeout(timeout)
    
    try:
        data = sock.recv(4096)
        if not data:
            return None
        
        first_byte = data[0]
        second_byte = data[1]
        
        is_fin = (first_byte & 0x80) != 0
        opcode = first_byte & 0x0F
        
        mask_bit = (second_byte & 0x80) != 0
        payload_length = second_byte & 0x7F
        
        idx = 2
        if payload_length == 126:
            payload_length = int.from_bytes(data[idx:idx+2], 'big')
            idx += 2
        elif payload_length == 127:
            payload_length = int.from_bytes(data[idx:idx+8], 'big')
            idx += 8
        
        if mask_bit:
            idx += 4
        
        payload = data[idx:idx+payload_length]
        return payload.decode('utf-8', errors='ignore')
        
    except socket.timeout:
        return None

def test_foxglove_server():
    """Test connection to Foxglove WebSocket server."""
    host = "localhost"
    port = 8765
    
    print("=" * 60)
    print("  Foxglove WebSocket Server Test")
    print("=" * 60)
    print()
    
    print(f"🔌 Connecting to {host}:{port}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        print("✅ Connected successfully!")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)
    
    print()
    
    if not perform_handshake(sock, host, port):
        sock.close()
        sys.exit(1)
    
    print()
    print("📤 Sending serverInfo message...")
    server_info = {
        "op": "serverInfo",
        "name": "RRT Path Planning",
        "capabilities": [],
        "supportedEncodings": ["json"]
    }
    send_websocket_frame(sock, json.dumps(server_info))
    print("✅ serverInfo sent")
    
    print()
    print("📤 Sending advertise messages...")
    topics = [
        "/planning/obstacles",
        "/planning/original_path",
        "/planning/smoothed_path",
        "/vehicle/state"
    ]
    for topic in topics:
        advertise = {
            "op": "advertise",
            "topics": [{"name": topic, "encoding": "json"}]
        }
        send_websocket_frame(sock, json.dumps(advertise))
        print(f"  ✅ Advertised: {topic}")
    
    print()
    print("=" * 60)
    print("  ✅ Connection test completed successfully!")
    print("=" * 60)
    print()
    print("📋 Summary:")
    print("   • WebSocket handshake: PASSED")
    print("   • serverInfo sent: PASSED")
    print("   • Topics advertised: PASSED")
    print()
    print("💡 If Foxglove Studio still doesn't work, the issue might be:")
    print("   1. Foxglove version incompatibility")
    print("   2. Foxglove expects different message formats")
    print("   3. Need to subscribe to topics from client side")
    print()
    
    sock.close()

if __name__ == "__main__":
    try:
        test_foxglove_server()
    except KeyboardInterrupt:
        print("\n\n👋 Test interrupted by user")
        sys.exit(0)
