#!/usr/bin/env python3
"""
MCAP file generator for Foxglove Studio with proper schemas
Install: pip install mcap
"""

import sys
import json
import time
import os

try:
    from mcap.writer import Writer
    from mcap.well_known import SchemaEncoding, MessageEncoding
except ImportError:
    print("⚠️  mcap library not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "mcap"])
    from mcap.writer import Writer
    from mcap.well_known import SchemaEncoding, MessageEncoding


OBSTACLE_SCHEMA = json.dumps({
    "title": "Obstacles",
    "description": "List of obstacles in the environment",
    "type": "object",
    "properties": {
        "obstacles": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "id": {"type": "integer"},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                    "radius": {"type": "number"}
                }
            }
        }
    }
}).encode()


PATH_SCHEMA = json.dumps({
    "title": "Path",
    "description": "A path consisting of waypoints",
    "type": "object",
    "properties": {
        "path": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"}
                }
            }
        }
    }
}).encode()


VEHICLE_STATE_SCHEMA = json.dumps({
    "title": "VehicleState",
    "description": "Current state of the vehicle",
    "type": "object",
    "properties": {
        "x": {"type": "number", "description": "X position"},
        "y": {"type": "number", "description": "Y position"},
        "z": {"type": "number", "description": "Z position"},
        "yaw": {"type": "number", "description": "Yaw angle in radians"},
        "speed": {"type": "number", "description": "Speed in m/s"}
    }
}).encode()


def create_sample_mcap(output_file):
    """Create a sample MCAP file with proper schemas"""
    print(f"📝 Creating MCAP file: {output_file}")
    
    with open(output_file, "wb") as f:
        writer = Writer(f)
        writer.start(profile="ros2", library="mcap-python")
        
        start_time = int(time.time_ns())
        
        print("  Registering schemas...")
        
        obstacle_schema_id = writer.register_schema(
            name="Obstacles",
            encoding=SchemaEncoding.JSONSchema,
            data=OBSTACLE_SCHEMA
        )
        
        path_schema_id = writer.register_schema(
            name="Path",
            encoding=SchemaEncoding.JSONSchema,
            data=PATH_SCHEMA
        )
        
        vehicle_schema_id = writer.register_schema(
            name="VehicleState",
            encoding=SchemaEncoding.JSONSchema,
            data=VEHICLE_STATE_SCHEMA
        )
        
        print("  Registering channels...")
        
        obstacle_channel_id = writer.register_channel(
            schema_id=obstacle_schema_id,
            topic="/planning/obstacles",
            message_encoding=MessageEncoding.JSON
        )
        
        original_path_channel_id = writer.register_channel(
            schema_id=path_schema_id,
            topic="/planning/original_path",
            message_encoding=MessageEncoding.JSON
        )
        
        smoothed_path_channel_id = writer.register_channel(
            schema_id=path_schema_id,
            topic="/planning/smoothed_path",
            message_encoding=MessageEncoding.JSON
        )
        
        vehicle_channel_id = writer.register_channel(
            schema_id=vehicle_schema_id,
            topic="/vehicle/state",
            message_encoding=MessageEncoding.JSON
        )
        
        print(f"✅ Registered 4 channels with proper schemas")
        
        obstacles = [
            {"id": 0, "x": 15.0, "y": 10.0, "z": 0.0, "radius": 3.0},
            {"id": 1, "x": 30.0, "y": 25.0, "z": 0.0, "radius": 2.5},
            {"id": 2, "x": 40.0, "y": 35.0, "z": 0.0, "radius": 2.0},
        ]
        
        print("  Generating path data...")
        
        original_path = []
        for i in range(100):
            t = i / 99.0
            x = 50 * t
            y = 50 * t
            original_path.append({"x": x, "y": y, "z": 0})
        
        smoothed_path = []
        for i in range(200):
            t = i / 199.0
            x = 50 * t
            y = 50 * t
            smoothed_path.append({"x": x, "y": y, "z": 0})
        
        print(f"  Writing messages...")
        
        num_timesteps = 200
        for step in range(num_timesteps):
            timestamp = start_time + step * 100000000
            
            t = step / (num_timesteps - 1)
            x = 50 * t
            y = 50 * t
            
            vehicle_state = {
                "x": x,
                "y": y,
                "z": 0,
                "yaw": 0.785,
                "speed": 5.0
            }
            
            writer.add_message(
                channel_id=obstacle_channel_id,
                log_time=timestamp,
                data=json.dumps({"obstacles": obstacles}).encode(),
                publish_time=timestamp,
            )
            
            writer.add_message(
                channel_id=original_path_channel_id,
                log_time=timestamp,
                data=json.dumps({"path": original_path}).encode(),
                publish_time=timestamp,
            )
            
            writer.add_message(
                channel_id=smoothed_path_channel_id,
                log_time=timestamp,
                data=json.dumps({"path": smoothed_path}).encode(),
                publish_time=timestamp,
            )
            
            writer.add_message(
                channel_id=vehicle_channel_id,
                log_time=timestamp,
                data=json.dumps(vehicle_state).encode(),
                publish_time=timestamp,
            )
            
            if step % 50 == 0:
                print(f"  📊 Progress: {step}/{num_timesteps}")
        
        writer.finish()
    
    file_size = os.path.getsize(output_file)
    print(f"\n✅ Success! MCAP file generated: {output_file}")
    print(f"📦 File size: {file_size / 1024:.1f} KB")
    print(f"📋 Contains {num_timesteps} timesteps")
    print(f"📝 Includes proper JSON Schemas for all message types")
    return True


def convert_jsonl_to_mcap(jsonl_file, output_file):
    """Convert JSONL file to MCAP with proper schemas"""
    print(f"📖 Reading JSONL file: {jsonl_file}")
    
    if not os.path.exists(jsonl_file):
        print(f"❌ File not found: {jsonl_file}")
        return False
    
    with open(jsonl_file, 'r') as f:
        lines = f.readlines()
    
    if not lines:
        print("❌ JSONL file is empty")
        return False
    
    print(f"  Found {len(lines)} lines")
    
    with open(output_file, "wb") as f:
        writer = Writer(f)
        writer.start(profile="ros2", library="mcap-python")
        
        start_time = int(time.time_ns())
        
        obstacle_schema_id = writer.register_schema(
            name="Obstacles",
            encoding=SchemaEncoding.JSONSchema,
            data=OBSTACLE_SCHEMA
        )
        
        path_schema_id = writer.register_schema(
            name="Path",
            encoding=SchemaEncoding.JSONSchema,
            data=PATH_SCHEMA
        )
        
        vehicle_schema_id = writer.register_schema(
            name="VehicleState",
            encoding=SchemaEncoding.JSONSchema,
            data=VEHICLE_STATE_SCHEMA
        )
        
        channel_ids = {
            "/planning/obstacles": writer.register_channel(
                schema_id=obstacle_schema_id,
                topic="/planning/obstacles",
                message_encoding=MessageEncoding.JSON
            ),
            "/planning/original_path": writer.register_channel(
                schema_id=path_schema_id,
                topic="/planning/original_path",
                message_encoding=MessageEncoding.JSON
            ),
            "/planning/smoothed_path": writer.register_channel(
                schema_id=path_schema_id,
                topic="/planning/smoothed_path",
                message_encoding=MessageEncoding.JSON
            ),
            "/vehicle/state": writer.register_channel(
                schema_id=vehicle_schema_id,
                topic="/vehicle/state",
                message_encoding=MessageEncoding.JSON
            )
        }
        
        print(f"✅ Registered {len(channel_ids)} channels with schemas")
        
        topic_counts = {}
        
        for i, line in enumerate(lines):
            line = line.strip()
            if not line:
                continue
            
            try:
                data = json.loads(line)
                topic = data.get("topic", "/unknown")
                
                if topic not in channel_ids:
                    continue
                
                timestamp = start_time + i * 100000000
                
                writer.add_message(
                    channel_id=channel_ids[topic],
                    log_time=timestamp,
                    data=json.dumps(data.get("data", {})).encode(),
                    publish_time=timestamp,
                )
                
                topic_counts[topic] = topic_counts.get(topic, 0) + 1
                
            except Exception as e:
                print(f"⚠️  Error parsing line {i}: {e}")
        
        writer.finish()
        
        print(f"\n📊 Messages per topic:")
        for topic, count in topic_counts.items():
            print(f"  {topic}: {count} messages")
    
    file_size = os.path.getsize(output_file)
    print(f"\n✅ Converted to MCAP: {output_file}")
    print(f"📦 File size: {file_size / 1024:.1f} KB")
    return True


if __name__ == "__main__":
    print("=" * 60)
    print("  MCAP File Generator for Foxglove Studio")
    print("  With Proper JSON Schemas")
    print("=" * 60)
    print()
    
    import argparse
    parser = argparse.ArgumentParser(description="Generate MCAP file with proper schemas")
    parser.add_argument("--jsonl", help="Input JSONL file")
    parser.add_argument("--output", default="rrt_simulation.mcap", help="Output MCAP file")
    parser.add_argument("--sample", action="store_true", help="Generate sample MCAP file")
    args = parser.parse_args()
    
    if args.sample or not args.jsonl:
        success = create_sample_mcap(args.output)
    else:
        success = convert_jsonl_to_mcap(args.jsonl, args.output)
    
    if success:
        print()
        print("📋 Next steps:")
        print(f"  1. Open Foxglove Studio")
        print(f"  2. Click 'File' -> 'Open File'")
        print(f"  3. Load {os.path.abspath(args.output)}")
        print(f"  4. Add panels to view the data:")
        print(f"     - Click '+' in top right")
        print(f"     - Select 'Raw Messages' to see JSON data")
        print(f"     - Select 'Plot' to visualize numeric values")
        print()
        sys.exit(0)
    else:
        sys.exit(1)
