#!/usr/bin/env python3
"""
Foxglove Scene Entity MCAP Writer - Fixed version
Receives data from C++ program via stdin and writes to MCAP file
Includes TF messages for coordinate frame support and SceneUpdate for 3D visualization
"""

import sys
import json
import os
import time
import math

try:
    from mcap.writer import Writer
    from mcap.well_known import SchemaEncoding, MessageEncoding
except ImportError:
    print("⚠️  mcap library not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "mcap"])
    from mcap.writer import Writer
    from mcap.well_known import SchemaEncoding, MessageEncoding


TF_MESSAGE_SCHEMA = json.dumps({
    "title": "tf2_msgs/TFMessage",
    "type": "object",
    "properties": {
        "transforms": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "header": {
                        "type": "object",
                        "properties": {
                            "stamp": {
                                "type": "object",
                                "properties": {
                                    "sec": {"type": "integer"},
                                    "nanosec": {"type": "integer"}
                                }
                            },
                            "frame_id": {"type": "string"}
                        }
                    },
                    "child_frame_id": {"type": "string"},
                    "transform": {
                        "type": "object",
                        "properties": {
                            "translation": {
                                "type": "object",
                                "properties": {
                                    "x": {"type": "number"},
                                    "y": {"type": "number"},
                                    "z": {"type": "number"}
                                }
                            },
                            "rotation": {
                                "type": "object",
                                "properties": {
                                    "x": {"type": "number"},
                                    "y": {"type": "number"},
                                    "z": {"type": "number"},
                                    "w": {"type": "number"}
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}).encode()


SCENE_UPDATE_SCHEMA = json.dumps({
    "title": "foxglove.SceneUpdate",
    "type": "object",
    "properties": {
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "required": ["id"],
                "properties": {
                    "id": {"type": "string"},
                    "frame_id": {"type": "string"},
                    "timestamp": {"type": "integer"},
                    "position": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"}
                        }
                    },
                    "orientation": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"},
                            "w": {"type": "number"}
                        }
                    },
                    "scale": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"}
                        }
                    },
                    "color": {
                        "type": "object",
                        "properties": {
                            "r": {"type": "number"},
                            "g": {"type": "number"},
                            "b": {"type": "number"},
                            "a": {"type": "number"}
                        }
                    },
                    "primitive": {
                        "type": "object",
                        "properties": {
                            "sphere": {
                                "type": "object",
                                "properties": {
                                    "radius": {"type": "number"}
                                }
                            },
                            "box": {
                                "type": "object",
                                "properties": {
                                    "dimensions": {
                                        "type": "array",
                                        "items": {"type": "number"},
                                        "minItems": 3,
                                        "maxItems": 3
                                    }
                                }
                            },
                            "arrow": {
                                "type": "object",
                                "properties": {
                                    "shaft_length": {"type": "number"},
                                    "head_length": {"type": "number"},
                                    "head_radius": {"type": "number"}
                                }
                            },
                            "line_strip": {
                                "type": "object",
                                "properties": {
                                    "points": {
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
                            }
                        }
                    }
                }
            }
        },
        "deleted_entity_ids": {"type": "array", "items": {"type": "string"}}
    }
}).encode()


OBSTACLE_SCHEMA = json.dumps({
    "title": "Obstacles",
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
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
        "yaw": {"type": "number"},
        "speed": {"type": "number"}
    }
}).encode()


def euler_to_quaternion(yaw):
    cos_yaw = math.cos(yaw / 2)
    sin_yaw = math.sin(yaw / 2)
    return {"x": 0.0, "y": 0.0, "z": sin_yaw, "w": cos_yaw}


def create_tf_message(timestamp, frame_id="map"):
    """Create a TF message defining the coordinate frame"""
    sec = timestamp // 1000000000
    nanosec = timestamp % 1000000000
    
    return {
        "transforms": [
            {
                "header": {
                    "stamp": {"sec": sec, "nanosec": nanosec},
                    "frame_id": "world"
                },
                "child_frame_id": frame_id,
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        ]
    }


def create_scene_update(obstacles, original_path, smoothed_path, vehicle_state):
    entities = []
    
    for obs in obstacles:
        entities.append({
            "id": f"obstacle_{obs.get('id', 0)}",
            "frame_id": "map",
            "position": {"x": obs.get('x', 0), "y": obs.get('y', 0), "z": obs.get('z', 0)},
            "color": {"r": 1.0, "g": 0.0, "b": 0.0, "a": 0.8},
            "primitive": {
                "sphere": {"radius": obs.get('radius', 1)}
            }
        })
    
    if original_path:
        points = [{"x": p.get('x', 0), "y": p.get('y', 0), "z": p.get('z', 0)} for p in original_path]
        entities.append({
            "id": "original_path",
            "frame_id": "map",
            "color": {"r": 1.0, "g": 0.647, "b": 0.0, "a": 1.0},
            "primitive": {
                "line_strip": {"points": points}
            }
        })
    
    if smoothed_path:
        points = [{"x": p.get('x', 0), "y": p.get('y', 0), "z": p.get('z', 0)} for p in smoothed_path]
        entities.append({
            "id": "smoothed_path",
            "frame_id": "map",
            "color": {"r": 0.0, "g": 0.8, "b": 1.0, "a": 1.0},
            "primitive": {
                "line_strip": {"points": points}
            }
        })
    
    if vehicle_state:
        quat = euler_to_quaternion(vehicle_state.get('yaw', 0))
        entities.append({
            "id": "vehicle_body",
            "frame_id": "map",
            "position": {"x": vehicle_state.get('x', 0), 
                       "y": vehicle_state.get('y', 0), 
                       "z": 0.75},
            "orientation": quat,
            "color": {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0},
            "primitive": {
                "box": {"dimensions": [4.0, 2.0, 1.5]}
            }
        })
        
        entities.append({
            "id": "vehicle_direction",
            "frame_id": "map",
            "position": {"x": vehicle_state.get('x', 0), 
                       "y": vehicle_state.get('y', 0), 
                       "z": 0.75},
            "orientation": quat,
            "color": {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0},
            "primitive": {
                "arrow": {"shaft_length": 3.0, "head_length": 1.0, "head_radius": 0.5}
            }
        })
    
    return {
        "entities": entities,
        "deleted_entity_ids": []
    }


def read_from_stdin_and_write_to_mcap(output_file):
    """Read data from stdin (from C++ program) and write to MCAP file"""
    print(f"📝 Creating Foxglove SceneUpdate MCAP with TF: {output_file}")
    
    f = open(output_file, "wb")
    writer = Writer(f)
    writer.start(profile="foxglove")
    
    tf_schema_id = writer.register_schema(
        name="tf2_msgs/TFMessage",
        encoding=SchemaEncoding.JSONSchema,
        data=TF_MESSAGE_SCHEMA
    )
    
    scene_schema_id = writer.register_schema(
        name="foxglove.SceneUpdate",
        encoding=SchemaEncoding.JSONSchema,
        data=SCENE_UPDATE_SCHEMA
    )
    
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
    
    tf_channel_id = writer.register_channel(
        schema_id=tf_schema_id,
        topic="/tf",
        message_encoding=MessageEncoding.JSON
    )
    
    scene_channel_id = writer.register_channel(
        schema_id=scene_schema_id,
        topic="/scene",
        message_encoding=MessageEncoding.JSON
    )
    
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
    
    print("✅ Registered all channels: /tf, /scene, /planning/obstacles, /planning/original_path, /planning/smoothed_path, /vehicle/state")
    
    start_time = int(time.time_ns())
    message_count = 0
    current_obstacles = []
    current_original_path = []
    current_smoothed_path = []
    current_vehicle_state = {}
    
    try:
        print("  Waiting for data from C++ program...")
        
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            
            try:
                data = json.loads(line)
                topic = data.get("topic")
                msg_data = data.get("data", {})
                
                if topic == "/planning/obstacles":
                    current_obstacles = msg_data.get("obstacles", [])
                elif topic == "/planning/original_path":
                    current_original_path = msg_data.get("path", [])
                elif topic == "/planning/smoothed_path":
                    current_smoothed_path = msg_data.get("path", [])
                elif topic == "/vehicle/state":
                    current_vehicle_state = msg_data
                
                timestamp = start_time + message_count * 100000000
                
                # Always write TF message to maintain coordinate frame
                tf_msg = create_tf_message(timestamp, "map")
                writer.add_message(
                    channel_id=tf_channel_id,
                    log_time=timestamp,
                    data=json.dumps(tf_msg).encode(),
                    publish_time=timestamp,
                )
                
                # Write SceneUpdate message for 3D visualization
                scene_msg = create_scene_update(
                    current_obstacles,
                    current_original_path,
                    current_smoothed_path,
                    current_vehicle_state
                )
                writer.add_message(
                    channel_id=scene_channel_id,
                    log_time=timestamp,
                    data=json.dumps(scene_msg).encode(),
                    publish_time=timestamp,
                )
                
                # Also write individual topic messages
                if current_obstacles:
                    writer.add_message(
                        channel_id=obstacle_channel_id,
                        log_time=timestamp,
                        data=json.dumps({"obstacles": current_obstacles}).encode(),
                        publish_time=timestamp,
                    )
                if current_original_path:
                    writer.add_message(
                        channel_id=original_path_channel_id,
                        log_time=timestamp,
                        data=json.dumps({"path": current_original_path}).encode(),
                        publish_time=timestamp,
                    )
                if current_smoothed_path:
                    writer.add_message(
                        channel_id=smoothed_path_channel_id,
                        log_time=timestamp,
                        data=json.dumps({"path": current_smoothed_path}).encode(),
                        publish_time=timestamp,
                    )
                if current_vehicle_state:
                    writer.add_message(
                        channel_id=vehicle_channel_id,
                        log_time=timestamp,
                        data=json.dumps(current_vehicle_state).encode(),
                        publish_time=timestamp,
                    )
                
                message_count += 1
                
                if message_count % 50 == 0:
                    print(f"  📊 Progress: {message_count} messages written")
                
            except Exception as e:
                print(f"⚠️  Error processing line: {e}", file=sys.stderr)
                import traceback
                traceback.print_exc()
        
        writer.finish()
        f.close()
        
        file_size = os.path.getsize(output_file)
        print(f"\n✅ Success! Scene MCAP file generated: {output_file}")
        print(f"📦 File size: {file_size / 1024:.1f} KB")
        print(f"📋 Contains {message_count} timesteps")
        
    except KeyboardInterrupt:
        writer.finish()
        f.close()
        print("\nReceived KeyboardInterrupt, saved partial file")
        file_size = os.path.getsize(output_file)
        print(f"📦 File size: {file_size / 1024:.1f} KB")


def create_sample_mcap(output_file):
    """Create a sample MCAP file for testing"""
    print(f"📝 Creating Foxglove SceneUpdate MCAP with TF: {output_file}")
    
    f = open(output_file, "wb")
    writer = Writer(f)
    writer.start(profile="foxglove")
    
    tf_schema_id = writer.register_schema(
        name="tf2_msgs/TFMessage",
        encoding=SchemaEncoding.JSONSchema,
        data=TF_MESSAGE_SCHEMA
    )
    
    scene_schema_id = writer.register_schema(
        name="foxglove.SceneUpdate",
        encoding=SchemaEncoding.JSONSchema,
        data=SCENE_UPDATE_SCHEMA
    )
    
    tf_channel_id = writer.register_channel(
        schema_id=tf_schema_id,
        topic="/tf",
        message_encoding=MessageEncoding.JSON
    )
    
    scene_channel_id = writer.register_channel(
        schema_id=scene_schema_id,
        topic="/scene",
        message_encoding=MessageEncoding.JSON
    )
    
    print("✅ Registered channels: /tf and /scene")
    
    obstacles = [
        {"id": 0, "x": 15.0, "y": 10.0, "z": 0.0, "radius": 3.0},
        {"id": 1, "x": 30.0, "y": 25.0, "z": 0.0, "radius": 2.5},
        {"id": 2, "x": 40.0, "y": 35.0, "z": 0.0, "radius": 2.0},
    ]
    
    original_path = [{"x": i * 0.5, "y": i * 0.5, "z": 0.0} for i in range(100)]
    smoothed_path = [{"x": i * 0.25, "y": i * 0.25 + 2 * math.sin(i * 0.1), "z": 0.0} for i in range(200)]
    
    start_time = int(time.time_ns())
    num_timesteps = 200
    
    print("  Writing messages...")
    
    for step in range(num_timesteps):
        timestamp = start_time + step * 100000000
        
        t = step / (num_timesteps - 1)
        vehicle_state = {
            "x": 50 * t,
            "y": 50 * t * (0.8 + 0.4 * t),
            "z": 0.75,
            "yaw": 0.785
        }
        
        tf_msg = create_tf_message(timestamp, "map")
        writer.add_message(
            channel_id=tf_channel_id,
            log_time=timestamp,
            data=json.dumps(tf_msg).encode(),
            publish_time=timestamp,
        )
        
        scene_msg = create_scene_update(
            obstacles,
            original_path,
            smoothed_path,
            vehicle_state
        )
        
        writer.add_message(
            channel_id=scene_channel_id,
            log_time=timestamp,
            data=json.dumps(scene_msg).encode(),
            publish_time=timestamp,
        )
        
        if step % 50 == 0:
            print(f"  📊 Progress: {step}/{num_timesteps}")
    
    writer.finish()
    f.close()
    
    file_size = os.path.getsize(output_file)
    print(f"\n✅ Success! Scene MCAP file generated: {output_file}")
    print(f"📦 File size: {file_size / 1024:.1f} KB")
    print(f"📋 Contains {num_timesteps} timesteps")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Write Foxglove SceneUpdate MCAP with TF support")
    parser.add_argument("output", help="Output MCAP file path")
    parser.add_argument("--sample", action="store_true", help="Generate sample data (no stdin input)")
    args = parser.parse_args()
    
    if args.sample:
        create_sample_mcap(args.output)
    else:
        read_from_stdin_and_write_to_mcap(args.output)
