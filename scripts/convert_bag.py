#!/usr/bin/env python3
"""Convert a ROS1 Livox Avia .bag to a ROS2 (mcap) bag.

rosbags converts standard types automatically, but the Livox custom type must be
renamed: ROS1 `livox_ros_driver/msg/CustomMsg` -> ROS2 `livox_ros_driver2/msg/CustomMsg`.
Their wire formats are byte-identical (verified from the bag's embedded msgdef:
offset_time uint32, x/y/z float32, reflectivity/tag/line uint8, same order), so this
is a pure type-rename, not a field remap.

Usage: convert_bag.py <src.bag> <dst_dir>
"""
import sys
from pathlib import Path

import yaml

import rosbags.convert.converter as conv

# Inject the Livox type rename into the converter's static rename table.
conv.STATIC_MSGTYPE_RENAMES["livox_ros_driver/msg/CustomMsg"] = "livox_ros_driver2/msg/CustomMsg"
conv.STATIC_MSGTYPE_RENAMES["livox_ros_driver/msg/CustomPoint"] = "livox_ros_driver2/msg/CustomPoint"

# Default QoS profile string (Humble encodes offered_qos_profiles as one YAML-encoded
# string per profile). Sensor-style: KEEP_LAST(1) depth 10, BEST_EFFORT(2), VOLATILE(2).
DEFAULT_QOS = (
    "- history: 1\n"
    "  depth: 10\n"
    "  reliability: 2\n"
    "  durability: 2\n"
    "  deadline:\n"
    "    sec: 2147483647\n"
    "    nsec: 4294967295\n"
    "  lifespan:\n"
    "    sec: 2147483647\n"
    "    nsec: 4294967295\n"
    "  liveliness: 1\n"
    "  liveliness_lease_duration:\n"
    "    sec: 2147483647\n"
    "    nsec: 4294967295\n"
    "  avoid_ros_namespace_conventions: false"
)


def rewrite_metadata_humble(dst: Path) -> None:
    """Rewrite rosbags' v9 metadata.yaml into Humble's v5 schema (QoS as string)."""
    meta_path = dst / "metadata.yaml"
    with open(meta_path) as f:
        v9 = yaml.safe_load(f)["rosbag2_bagfile_information"]

    files = v9.get("files") or []
    if files:
        file0 = files[0]
        fname = file0.get("path") or v9["relative_file_paths"][0]
        file_start = file0["starting_time"]["nanoseconds_since_epoch"]
        file_dur = file0["duration"]["nanoseconds"]
        file_count = file0["message_count"]
    else:
        fname = v9["relative_file_paths"][0]
        file_start = v9["starting_time"]["nanoseconds_since_epoch"]
        file_dur = v9["duration"]["nanoseconds"]
        file_count = v9["message_count"]

    topics = []
    for t in v9["topics_with_message_count"]:
        tm = t["topic_metadata"]
        topics.append(
            {
                "topic_metadata": {
                    "name": tm["name"],
                    "type": tm["type"],
                    "serialization_format": "cdr",
                    "offered_qos_profiles": DEFAULT_QOS,
                },
                "message_count": t["message_count"],
            }
        )

    v5 = {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": v9["duration"]["nanoseconds"]},
            "starting_time": {"nanoseconds_since_epoch": v9["starting_time"]["nanoseconds_since_epoch"]},
            "message_count": v9["message_count"],
            "topics_with_message_count": topics,
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [fname],
            "files": [
                {
                    "path": fname,
                    "starting_time": {"nanoseconds_since_epoch": file_start},
                    "duration": {"nanoseconds": file_dur},
                    "message_count": file_count,
                }
            ],
        }
    }

    with open(meta_path, "w") as f:
        yaml.dump(v5, f, default_flow_style=False, sort_keys=False, width=10000)
    print(f"[convert_bag] rewrote {meta_path} to Humble v5 schema")


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    src = Path(sys.argv[1]).resolve()
    dst = Path(sys.argv[2]).resolve()
    dst.parent.mkdir(parents=True, exist_ok=True)
    print(f"[convert_bag] {src} -> {dst}")
    conv.convert(
        srcs=[src],
        dst=dst,
        dst_storage="mcap",
        dst_version=9,
        compress=None,
        compress_mode="file",
        default_typestore=None,
        typestore=None,
        exclude_topics=("/rosout", "/rosout_agg"),
        include_topics=(),
        exclude_msgtypes=(),
        include_msgtypes=(),
    )
    print(f"[convert_bag] done -> {dst}")
    rewrite_metadata_humble(dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
