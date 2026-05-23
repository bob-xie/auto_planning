#!/usr/bin/env python3

import asyncio
import websockets
import json
import sys

async def test_foxglove_connection():
    uri = "ws://localhost:8765"
    
    print(f"🔌 Connecting to {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Connected successfully!")
            
            # Send a test message (Foxglove doesn't require this, but let's see if server responds)
            await websocket.send("test")
            
            # Receive and print server messages
            print("\n📥 Waiting for messages...")
            print("(Press Ctrl+C to exit)\n")
            
            message_count = 0
            while True:
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    message_count += 1
                    print(f"📨 Message {message_count}:")
                    
                    try:
                        data = json.loads(message)
                        print(json.dumps(data, indent=2))
                    except:
                        print(message)
                    print()
                    
                except asyncio.TimeoutError:
                    print("⏱️  No message received for 5 seconds, still waiting...")
                    
    except websockets.exceptions.InvalidStatusCode as e:
        print(f"❌ Connection failed with status code: {e}")
        print("\n💡 This might be a protocol version mismatch.")
        print("   Foxglove Studio requires: foxglove.websocket.v1")
        sys.exit(1)
    except ConnectionRefusedError:
        print("❌ Connection refused!")
        print("   Make sure the test_rrt program is running.")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    print("=" * 60)
    print("  Foxglove WebSocket Test Client")
    print("=" * 60)
    print()
    
    try:
        asyncio.run(test_foxglove_connection())
    except KeyboardInterrupt:
        print("\n\n👋 Test interrupted by user")
