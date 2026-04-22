#!/usr/bin/env python3
"""
rosbag_to_lerobot.py — Convert ROS2 rosbag to LeRobot v2.1 dataset format.

Output layout (LeRobot v3.0):
  <output_dir>/
    data/
      chunk-000/
        episode_000000.parquet
        ...
    videos/
      chunk-000/
        observation.images.camera_gripper_episode_000000.mp4
        observation.images.camera_global_episode_000000.mp4
        ...
    meta/
      info.json
      stats.json
      tasks.jsonl
      episodes.jsonl

Usage:
  python3 tools/rosbag_to_lerobot.py \
      --bag ~/rosbag_recordings/test_dual_camera \
      --output ~/lerobot_datasets/test_pick \
      --task "pick up the object" \
      [--episode-index 0] \
      [--slop 0.02] \
      [--img-size 224]
"""
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

# ROS2 env must be sourced — check before heavy imports
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    sys.exit(f"[ERROR] ROS2 not sourced or rosbag2_py not installed: {e}")

# ── Constants ────────────────────────────────────────────────────────────────

# Canonical joint order for observation.state (verified from /dsr01/joint_states)
JOINT_ORDER = ["joint_1", "joint_2", "joint_4", "joint_5", "joint_3", "joint_6"]

TOPIC_GRIPPER       = "/camera_gripper/camera_gripper/color/image_raw"
TOPIC_GLOBAL        = "/camera_global/camera_global/color/image_raw"
TOPIC_JOINTS        = "/dsr01/joint_states"
TOPIC_GRIPPER_STATE       = "/gripper_joint_states"  # OnRobot RG2: rg2_finger_joint angle
GRIPPER_OPEN_THRESHOLD_MM = 50.0                      # width >= threshold → open (1.0), else closed (0.0)

# RG2 kinematic constants for finger_joint → width conversion
import math
_RG2_L1, _RG2_L3                      = 0.108505, 0.055
_RG2_THETA1, _RG2_THETA3, _RG2_DY     = 1.41371, 0.76794, -0.0144


def _gripper_angle_to_width_mm(angle: float) -> float:
    """rg2_finger_joint angle (rad) → gripper opening width (mm)."""
    return ((math.cos(angle + _RG2_THETA3) * _RG2_L3
             + _RG2_DY + _RG2_L1 * math.cos(_RG2_THETA1)) * 2) * 1000.0

# ── Rosbag Reader ─────────────────────────────────────────────────────────────

def read_bag(bag_path: Path):
    """
    Read all messages from a rosbag.
    Returns dict: topic → list of (timestamp_ns, msg)
    """
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {}
    for meta in reader.get_all_topics_and_types():
        topic_types[meta.name] = meta.type

    # Filter to only the topics we care about
    wanted = {TOPIC_GRIPPER, TOPIC_GLOBAL, TOPIC_JOINTS, TOPIC_GRIPPER_STATE}
    msg_types = {}
    for topic, type_str in topic_types.items():
        if topic in wanted:
            msg_types[topic] = get_message(type_str)

    data: dict[str, list] = {t: [] for t in wanted}

    while reader.has_next():
        topic, raw, ts_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(raw, msg_types[topic])
        data[topic].append((ts_ns, msg))

    return data


# ── Timestamp Sync ────────────────────────────────────────────────────────────

def sync_frames(data: dict, slop_ns: int = 20_000_000) -> list[dict]:
    """
    Align frames using camera_gripper timestamps as reference.
    For each gripper frame, find nearest global frame, joint state, and (if present)
    gripper state within slop.
    Returns list of dicts with keys: ts_ns, img_gripper, img_global, joints,
    gripper_js (None if topic not in bag).
    """
    gripper_msgs      = data[TOPIC_GRIPPER]        # list of (ts_ns, img_msg)
    global_msgs       = data[TOPIC_GLOBAL]         # list of (ts_ns, img_msg)
    joint_msgs        = data[TOPIC_JOINTS]         # list of (ts_ns, js_msg)
    gripper_state_msgs = data.get(TOPIC_GRIPPER_STATE, [])

    global_ts  = np.array([t for t, _ in global_msgs],  dtype=np.int64)
    joint_ts   = np.array([t for t, _ in joint_msgs],   dtype=np.int64)
    gripper_state_ts = (
        np.array([t for t, _ in gripper_state_msgs], dtype=np.int64)
        if gripper_state_msgs else None
    )

    synced = []
    for ref_ts, gripper_img in gripper_msgs:
        gi = int(np.argmin(np.abs(global_ts - ref_ts)))
        if abs(global_ts[gi] - ref_ts) > slop_ns:
            continue

        ji = int(np.argmin(np.abs(joint_ts - ref_ts)))
        if abs(joint_ts[ji] - ref_ts) > slop_ns:
            continue

        # forward-fill: use most recent gripper state at or before ref_ts
        gripper_js = None
        if gripper_state_ts is not None:
            past = np.where(gripper_state_ts <= ref_ts)[0]
            if len(past) > 0:
                gripper_js = gripper_state_msgs[past[-1]][1]

        synced.append({
            "ts_ns":       ref_ts,
            "img_gripper": gripper_img,
            "img_global":  global_msgs[gi][1],
            "joints":      joint_msgs[ji][1],
            "gripper_js":  gripper_js,
        })

    return synced


# ── Image Conversion ──────────────────────────────────────────────────────────

def img_msg_to_bgr(msg, size: int) -> np.ndarray:
    """
    Convert sensor_msgs/Image to resized BGR numpy array.
    Supports rgb8 and bgr8 encodings.
    """
    enc = msg.encoding.lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if enc in ("rgb8", "bgr8", "mono8"):
        channels = 1 if enc == "mono8" else 3
        img = raw.reshape((msg.height, msg.width, channels))
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    else:
        # Fallback: try imdecode
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    return cv2.resize(img, (size, size), interpolation=cv2.INTER_LINEAR)


# ── Joint State Extraction ────────────────────────────────────────────────────

def extract_joints(js_msg) -> list[float]:
    """Extract arm joint positions in JOINT_ORDER. Returns 6 floats."""
    name_to_pos = dict(zip(js_msg.name, js_msg.position))
    return [name_to_pos.get(j, 0.0) for j in JOINT_ORDER]


def extract_gripper_binary(msg) -> float:
    """Convert rg2_finger_joint angle (JointState) to binary: 1.0=open, 0.0=closed."""
    name_to_pos = dict(zip(msg.name, msg.position))
    angle = name_to_pos.get("rg2_finger_joint")
    if angle is None:
        return 1.0
    width_mm = _gripper_angle_to_width_mm(angle)
    return 1.0 if width_mm >= GRIPPER_OPEN_THRESHOLD_MM else 0.0


# ── MP4 Writer ────────────────────────────────────────────────────────────────

def write_mp4(frames_bgr: list[np.ndarray], output_path: Path, fps: int = 30):
    """
    Write list of BGR frames to H.264 MP4.
    Uses imageio-ffmpeg (bundled binary) — no system ffmpeg required.
    Falls back to OpenCV mp4v if imageio is unavailable.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import imageio
        import imageio_ffmpeg  # noqa: F401 — ensures bundled binary is present

        writer = imageio.get_writer(
            str(output_path),
            fps=fps,
            codec="libx264",
            quality=None,
            output_params=["-crf", "18", "-preset", "fast"],
            pixelformat="yuv420p",
        )
        for frame_bgr in frames_bgr:
            writer.append_data(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        writer.close()

    except ImportError:
        # Fallback: OpenCV mp4v (may not play in all browsers/players)
        h, w = frames_bgr[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        cv_writer = cv2.VideoWriter(str(output_path), fourcc, fps, (w, h))
        for frame in frames_bgr:
            cv_writer.write(frame)
        cv_writer.release()


# ── Parquet Writer ────────────────────────────────────────────────────────────

def write_parquet(rows: list[dict], output_path: Path):
    """Write episode rows to Parquet (LeRobot v2.1 column schema)."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame(rows)
    table = pa.Table.from_pandas(df, preserve_index=False)
    pq.write_table(table, str(output_path))


def _next_global_index(output_dir: Path) -> int:
    """Return the next global frame index by scanning existing parquet files."""
    data_dir = output_dir / "data"
    if not data_dir.exists():
        return 0
    existing = sorted(data_dir.glob("chunk-*/*.parquet"))
    if not existing:
        return 0
    last_table = pq.read_table(str(existing[-1]), columns=["index"])
    indices = last_table["index"].to_pylist()
    if not indices:
        return 0
    return int(indices[-1]) + 1


# ── Meta Writers ─────────────────────────────────────────────────────────────

def write_meta(output_dir: Path, all_rows: list[dict], task_name: str, fps: int, img_size: int):
    meta_dir = output_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    n_episodes = int(all_rows[-1]["episode_index"]) + 1 if all_rows else 0
    n_frames   = len(all_rows)

    # ── info.json ──────────────────────────────────────────────────────────
    state_dim   = len(all_rows[0]["observation.state"]) if all_rows else 6
    has_gripper = state_dim == 7
    state_names = JOINT_ORDER + (["gripper_open"] if has_gripper else [])

    info = {
        "codebase_version": "v2.1",
        "robot_type":        "doosan_m0609",
        "fps":               fps,
        "splits":            {"train": f"0:{n_episodes}"},
        "data_path":         "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path":        "videos/chunk-{episode_chunk:03d}/{video_key}_episode_{episode_index:06d}.mp4",
        "features": {
            "observation.state": {
                "dtype":  "float32",
                "shape":  [state_dim],
                "names":  state_names,
            },
            "action": {
                "dtype": "float32",
                "shape": [state_dim],
                "names": state_names,
            },
            "observation.images.camera_gripper": {
                "dtype":   "video",
                "shape":   [img_size, img_size, 3],
                "names":   ["height", "width", "channel"],
                "info": {
                    "video.fps":            fps,
                    "video.codec":          "av1",
                    "video.pix_fmt":        "yuv420p",
                    "video.is_depth_map":   False,
                },
            },
            "observation.images.camera_global": {
                "dtype":   "video",
                "shape":   [img_size, img_size, 3],
                "names":   ["height", "width", "channel"],
                "info": {
                    "video.fps":            fps,
                    "video.codec":          "av1",
                    "video.pix_fmt":        "yuv420p",
                    "video.is_depth_map":   False,
                },
            },
            "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
            "index":         {"dtype": "int64",   "shape": [1], "names": None},
            "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
            "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
        },
        "total_episodes": n_episodes,
        "total_frames":   n_frames,
        "total_tasks":    1,
        "total_chunks":   1,
        "chunks_size":    1000,
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2))

    # ── tasks.jsonl ────────────────────────────────────────────────────────
    with open(meta_dir / "tasks.jsonl", "w") as f:
        f.write(json.dumps({"task_index": 0, "task": task_name}) + "\n")

    # ── episodes.jsonl ─────────────────────────────────────────────────────
    ep_rows = all_rows  # Already filtered per episode in caller; here we rebuild
    ep_map: dict[int, list] = {}
    for r in all_rows:
        ep_map.setdefault(int(r["episode_index"]), []).append(r)

    with open(meta_dir / "episodes.jsonl", "w") as f:
        for ep_idx, ep_rows_list in sorted(ep_map.items()):
            f.write(json.dumps({
                "episode_index": ep_idx,
                "tasks":         [task_name],
                "length":        len(ep_rows_list),
            }) + "\n")

    # ── stats.json ─────────────────────────────────────────────────────────
    states  = np.array([r["observation.state"] for r in all_rows], dtype=np.float32)
    actions = np.array([r["action"]             for r in all_rows], dtype=np.float32)
    stats = {
        "observation.state": {
            "mean": states.mean(axis=0).tolist(),
            "std":  states.std(axis=0).tolist(),
            "min":  states.min(axis=0).tolist(),
            "max":  states.max(axis=0).tolist(),
        },
        "action": {
            "mean": actions.mean(axis=0).tolist(),
            "std":  actions.std(axis=0).tolist(),
            "min":  actions.min(axis=0).tolist(),
            "max":  actions.max(axis=0).tolist(),
        },
    }
    (meta_dir / "stats.json").write_text(json.dumps(stats, indent=2))


# ── Main ──────────────────────────────────────────────────────────────────────

def convert(
    bag_path: Path,
    output_dir: Path,
    task_name: str,
    episode_index: int = 0,
    slop_s: float = 0.02,
    img_size: int = 224,
    fps: int = 30,
):
    print(f"[1/5] Reading rosbag: {bag_path}")
    data = read_bag(bag_path)

    n_gripper = len(data[TOPIC_GRIPPER])
    n_global  = len(data[TOPIC_GLOBAL])
    n_joints  = len(data[TOPIC_JOINTS])
    print(f"      gripper frames: {n_gripper}, global frames: {n_global}, joint states: {n_joints}")

    print(f"[2/5] Synchronising frames (slop={slop_s*1000:.0f}ms)")
    synced = sync_frames(data, slop_ns=int(slop_s * 1e9))
    n_synced = len(synced)
    print(f"      synced frames: {n_synced}")
    if n_synced == 0:
        sys.exit("[ERROR] No synced frames — check topic names or increase --slop")

    # Reference timestamp (start of episode)
    t0_ns = synced[0]["ts_ns"]

    print("[3/5] Encoding videos")
    frames_gripper = []
    frames_global  = []
    for sf in synced:
        frames_gripper.append(img_msg_to_bgr(sf["img_gripper"], img_size))
        frames_global.append(img_msg_to_bgr(sf["img_global"],   img_size))

    ep_str = f"episode_{episode_index:06d}"
    chunk  = "chunk-000"

    vid_dir = output_dir / "videos" / chunk
    write_mp4(frames_gripper, vid_dir / f"observation.images.camera_gripper_{ep_str}.mp4", fps)
    write_mp4(frames_global,  vid_dir / f"observation.images.camera_global_{ep_str}.mp4",  fps)
    print(f"      videos written to {vid_dir}")

    # Detect gripper state availability from first synced frame
    has_gripper = any(sf["gripper_js"] is not None for sf in synced)
    if has_gripper:
        print("      gripper state detected — observation.state will be 7D (6 joints + open/closed)")
    else:
        print("      no gripper state topic — observation.state is 6D")

    # Global frame index: continues from last existing parquet (multi-episode append)
    global_index_start = _next_global_index(output_dir)

    print("[4/5] Building Parquet rows")
    rows = []
    for fi, sf in enumerate(synced):
        arm_state = extract_joints(sf["joints"])

        if has_gripper:
            gs = extract_gripper_binary(sf["gripper_js"]) if sf["gripper_js"] else 1.0
            state = arm_state + [gs]
        else:
            state = arm_state

        # Action = next frame's state (teleoperation target); last frame repeats
        if fi + 1 < n_synced:
            next_sf = synced[fi + 1]
            next_arm = extract_joints(next_sf["joints"])
            if has_gripper:
                ngs = extract_gripper_binary(next_sf["gripper_js"]) if next_sf["gripper_js"] else 1.0
                action = next_arm + [ngs]
            else:
                action = next_arm
        else:
            action = state

        rows.append({
            "frame_index":       fi,
            "index":             global_index_start + fi,
            "timestamp":         float(sf["ts_ns"] - t0_ns) / 1e9,
            "episode_index":     episode_index,
            "task_index":        0,
            "observation.state": [float(x) for x in state],
            "action":            [float(x) for x in action],
        })

    parquet_path = output_dir / "data" / chunk / f"{ep_str}.parquet"
    write_parquet(rows, parquet_path)
    print(f"      parquet written: {parquet_path}")

    print("[5/5] Writing meta/")
    write_meta(output_dir, rows, task_name, fps, img_size)

    print(f"\n[DONE] Dataset: {output_dir}")
    print(f"       episode {episode_index}: {n_synced} frames, "
          f"{float(synced[-1]['ts_ns'] - t0_ns)/1e9:.1f}s")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Convert ROS2 rosbag → LeRobot v2.1 dataset")
    parser.add_argument("--bag",           required=True, type=Path, help="Path to rosbag directory")
    parser.add_argument("--output",        required=True, type=Path, help="Output dataset directory")
    parser.add_argument("--task",          default="pick up the object", help="Task description string")
    parser.add_argument("--episode-index", default=0, type=int, help="Episode index (for multi-episode append)")
    parser.add_argument("--slop",          default=0.02, type=float, help="Sync slop in seconds (default 0.02)")
    parser.add_argument("--img-size",      default=224, type=int, help="Output image size (square)")
    parser.add_argument("--fps",           default=30, type=int, help="Video FPS")
    args = parser.parse_args()

    convert(
        bag_path=args.bag.expanduser(),
        output_dir=args.output.expanduser(),
        task_name=args.task,
        episode_index=args.episode_index,
        slop_s=args.slop,
        img_size=args.img_size,
        fps=args.fps,
    )


if __name__ == "__main__":
    main()
