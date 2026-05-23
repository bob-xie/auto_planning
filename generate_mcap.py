#!/usr/bin/env python3
"""
Generate valid MCAP file for Foxglove Studio
"""

import sys
import json
import time
import os
from datetime import datetime

try:
    import mcap
    from mcap.writer import Writer
    HAS_MCAP = True
except ImportError:
    HAS_MCAP = False
    print("⚠️  mcap library not found. Install with: pip install mcap")
    sys.exit(1)

def generate_mcap(output_file, simulation_data=None):
    """
    Generate MCAP file with simulation data
    """
    print(f"📝 Generating MCAP file: {output_file}")

    with open(output_file, "wb") as f:
        writer = Writer(f)

        writer.start()

        start_time = int(time.time_ns())

        obstacle_schema_id = writer.register_schema(
            name="Obstacles",
            encoding="jsonschema",
            data='{ "type": "object", "properties": { "obstacles": { "type": "array" } } }'.encode(),
        )

        obstacle_channel_id = writer.register_channel(
            topic="/planning/obstacles",
            message_encoding="json",
            schema_id=obstacle_schema_id,
        )

        path_schema_id = writer.register_schema(
            name="Path",
            encoding="jsonschema",
            data='{ "type": "object", "properties": { "path": { "type": "array" } } }'.encode(),
        )

        original_path_channel_id = writer.register_channel(
            topic="/planning/original_path",
            message_encoding="json",
            schema_id=path_schema_id,
        )

        smoothed_path_channel_id = writer.register_channel(
            topic="/planning/smoothed_path",
            message_encoding="json",
            schema_id=path_schema_id,
        )

        vehicle_schema_id = writer.register_schema(
            name="VehicleState",
            encoding="jsonschema",
            data='{ "type": "object" }'.encode(),
        )

        vehicle_channel_id = writer.register_channel(
            topic="/vehicle/state",
            message_encoding="json",
            schema_id=vehicle_schema_id,
        )

        print("✅ Channels registered")

        obstacles = [
            {"id": 0, "x": 15.0, "y": 10.0, "z": 0.0, "radius": 3.0},
            {"id": 1, "x": 30.0, "y": 25.0, "z": 0.0, "radius": 2.5},
            {"id": 2, "x": 40.0, "y": 35.0, "z": 0.0, "radius": 2.0},
        ]

        num_points = 100
        original_path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = 50 * t
            y = 50 * t
            z = 0
            original_path.append({"x": x, "y": y, "z": z})

        smoothed_path = []
        for i in range(num_points * 2):
            t = i / (num_points * 2 - 1)
            x = 50 * t
            y = 50 * t
            z = 0
            smoothed_path.append({"x": x, "y": y, "z": z})

        print(f"✅ Generated {len(original_path)} path points")

        num_timesteps = 200
        for i in range(num_timesteps):
            timestamp = start_time + i * 100000000

            t = i / (num_timesteps - 1)
            progress = t * 50

            vehicle_data = {
                "x": progress,
                "y": progress,
                "z": 0,
                "yaw": 0.785,
                "speed": 5.0
            }

            obstacle_msg = json.dumps({"obstacles": obstacles}).encode()
            writer.add_message(
                channel_id=obstacle_channel_id,
                log_time=timestamp,
                data=obstacle_msg,
                publish_time=timestamp,
            )

            original_path_msg = json.dumps({"path": original_path}).encode()
            writer.add_message(
                channel_id=original_path_channel_id,
                log_time=timestamp,
                data=original_path_msg,
                publish_time=timestamp,
            )

            smoothed_path_msg = json.dumps({"path": smoothed_path}).encode()
            writer.add_message(
                channel_id=smoothed_path_channel_id,
                log_time=timestamp,
                data=smoothed_path_msg,
                publish_time=timestamp,
            )

            vehicle_msg = json.dumps(vehicle_data).encode()
            writer.add_message(
                channel_id=vehicle_channel_id,
                log_time=timestamp,
                data=vehicle_msg,
                publish_time=timestamp,
            )

            if i % 50 == 0:
                print(f"📊 Written {i}/{num_timesteps} timesteps")

        writer.finish()

    print(f"✅ MCAP file generated successfully!")
    file_size = os.path.getsize(output_file)
    print(f"📦 File size: {file_size / 1024:.1f} KB")
    return True

if __name__ == "__main__":
    print("=" * 60)
    print("  MCAP File Generator for Foxglove Studio")
    print("=" * 60)
    print()

    output_file = "rrt_simulation.mcap"
    
    try:
        generate_mcap(output_file)
        
        print()
        print("📋 Next steps:")
        print(f"  1. Open Foxglove Studio")
        print(f"  2. Click 'Open Local File' or drag and drop")
        print(f"  3. Load {output_file}")
        print()

    except Exception as e:
        print(f"❌ Error generating MCAP file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
