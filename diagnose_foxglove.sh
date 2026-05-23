#!/bin/bash

echo "========================================"
echo "  Foxglove Connection Diagnostic Tool"
echo "========================================"
echo ""

echo "Step 1: Checking if test_rrt is running..."
if pgrep -f test_rrt > /dev/null; then
    echo "✅ test_rrt is running"
else
    echo "❌ test_rrt is NOT running"
    echo "   Please run: ./test/test_rrt"
    exit 1
fi

echo ""
echo "Step 2: Checking if port 8765 is listening..."
if netstat -tuln 2>/dev/null | grep -q ":8765"; then
    echo "✅ Port 8765 is listening"
else
    echo "❌ Port 8765 is NOT listening"
    echo "   Check if test_rrt started successfully"
    exit 1
fi

echo ""
echo "Step 3: Checking Foxglove Studio version..."
echo "   Please check your Foxglove Studio version manually"
echo "   Supported versions: 1.0+ (WebSocket protocol v1)"

echo ""
echo "Step 4: Testing WebSocket connection..."
echo "   Trying to connect to ws://localhost:8765..."

# Use Python WebSocket client if available
python3 -c "
import asyncio
import websockets

async def test():
    try:
        async with websockets.connect('ws://localhost:8765', timeout=3) as ws:
            print('✅ WebSocket connection successful!')
            return True
    except Exception as e:
        print(f'❌ WebSocket connection failed: {e}')
        return False

asyncio.run(test())
" 2>/dev/null

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ All checks passed!"
    echo ""
    echo "💡 If Foxglove Studio still doesn't work:"
    echo "   1. Try restarting Foxglove Studio"
    echo "   2. Try a different Foxglove Studio version"
    echo "   3. Check Foxglove Studio's console for error messages"
else
    echo ""
    echo "❌ WebSocket connection test failed"
    echo ""
    echo "💡 Possible causes:"
    echo "   1. Firewall blocking the connection"
    echo "   2. Foxglove Studio version incompatibility"
    echo "   3. Network configuration issue"
fi
