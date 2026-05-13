#!/usr/bin/env python3
"""Extract training samples from pseudo-label JSONL for MLP fusion head.

Each sample = (9 features, target_distance) per detection with valid LiDAR fusion.

Features (order fixed for C++ inference):
  0  center_x_norm     = (left + right) / 2 / image_width
  1  center_y_norm     = (top + bottom) / 2 / image_height
  2  box_w_norm        = (right - left) / image_width
  3  box_h_norm        = (bottom - top) / image_height
  4  box_area_ratio    = (box_w * box_h) / (image_width * image_height)
  5  confidence        = detection confidence
  6  raw_distance_m    = LiDAR cluster median (from fusion diagnostics)
  7  candidate_points  = number of LiDAR points in detection sector
  8  cluster_score     = quality score of the selected cluster

Target:
  distance_m  = the final tracked distance (or raw_distance_m if no tracking)

Optional filters:
  --min-cluster-score   discard samples below this cluster_score (default 1.0)
  --min-candidates      discard samples below this candidate_points (default 2)
  --min-distance        discard samples closer than this (default 0.3m)
  --max-distance        discard samples farther than this (default 15.0m)

Usage:
  python3 tools/calib/extract_fusion_training_data.py \\
      --glob '/tmp/rk3588_pseudo_labels.jsonl*' \\
      --out /tmp/fusion_training_data.csv
"""

import argparse
import glob
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


FEATURE_NAMES = [
    "center_x_norm",
    "center_y_norm",
    "box_w_norm",
    "box_h_norm",
    "box_area_ratio",
    "confidence",
    "raw_distance_m",
    "candidate_points",
    "cluster_score",
]


def camera_angle_from_pixel(center_x: float, image_width: int,
                            camera_fov_deg: float = 55.0) -> float:
    """Convert pixel x-coordinate to camera angle (degrees, 0 = center)."""
    norm = (center_x / max(1, image_width)) - 0.5
    return norm * camera_fov_deg


def extract_sample(obj: Dict[str, Any],
                   image_width: int,
                   camera_fov_deg: float,
                   min_cluster_score: float,
                   min_candidates: int,
                   min_dist: float,
                   max_dist: float) -> Optional[Tuple[List[float], float, Dict[str, Any]]]:
    """Extract one sample from a detection object. Returns (features, target, meta) or None."""
    bbox = obj.get("bbox")
    if not isinstance(bbox, dict):
        return None

    left = bbox.get("left", 0)
    top = bbox.get("top", 0)
    right = bbox.get("right", 0)
    bottom = bbox.get("bottom", 0)
    box_w = max(0, right - left)
    box_h = max(0, bottom - top)
    if box_w <= 0 or box_h <= 0:
        return None

    confidence = obj.get("confidence", 0)
    if not isinstance(confidence, (int, float)) or confidence <= 0:
        return None

    fusion = obj.get("fusion")
    if not isinstance(fusion, dict):
        return None

    raw_dist = fusion.get("raw_distance_m", -1)
    cand_pts = fusion.get("candidate_points", 0)
    clust_pts = fusion.get("cluster_points", 0)
    clust_score = fusion.get("cluster_score", 0.0)
    rejected = fusion.get("rejected_by_sanity", False)
    fallback = fusion.get("used_fallback", False)

    if raw_dist is None or raw_dist < 0:
        return None
    raw_dist = float(raw_dist)

    if clust_score < min_cluster_score:
        return None
    if clust_pts < min_candidates:
        return None
    if raw_dist < min_dist or raw_dist > max_dist:
        return None
    if rejected or fallback:
        return None

    # target: use raw LiDAR cluster median as pseudo-ground-truth
    target = raw_dist

    center_x = 0.5 * (left + right)
    center_y = 0.5 * (top + bottom)
    iw = max(1, image_width)

    features = [
        center_x / iw,
        center_y / iw,             # normalized by width for aspect-ratio invariance
        box_w / iw,
        box_h / iw,
        (box_w * box_h) / (iw * iw),
        float(confidence),
        raw_dist,
        float(cand_pts),
        float(clust_score),
    ]

    meta = {
        "class_name": obj.get("class_name", "unknown"),
        "class_id": obj.get("class_id", -1),
        "track_id": obj.get("track_id", -1),
        "angle_deg": camera_angle_from_pixel(center_x, iw, camera_fov_deg),
        "cluster_points": clust_pts,
        "frame_id": obj.get("_frame_id", 0),
    }

    return features, target, meta


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract fusion MLP training data from pseudo-label JSONL"
    )
    parser.add_argument("--glob", default="",
                        help="Glob pattern for JSONL files")
    parser.add_argument("--paths", nargs="*", default=[],
                        help="Explicit file paths")
    parser.add_argument("--out", default="/tmp/fusion_training_data.csv",
                        help="Output CSV path")
    parser.add_argument("--min-cluster-score", type=float, default=1.0,
                        help="Minimum cluster_score to include sample")
    parser.add_argument("--min-candidates", type=int, default=2,
                        help="Minimum cluster_points to include sample")
    parser.add_argument("--min-distance", type=float, default=0.3,
                        help="Minimum distance in meters")
    parser.add_argument("--max-distance", type=float, default=15.0,
                        help="Maximum distance in meters")
    parser.add_argument("--image-width", type=int, default=640,
                        help="Camera image width in pixels")
    parser.add_argument("--camera-fov-deg", type=float, default=55.0,
                        help="Camera horizontal FOV in degrees")
    parser.add_argument("--max-samples", type=int, default=0,
                        help="Cap total samples (0 = unlimited)")
    args = parser.parse_args()

    # collect paths
    paths: List[Path] = []
    for p in args.paths:
        pp = Path(p)
        if pp.exists():
            paths.append(pp)
    for matched in sorted(glob.glob(args.glob)):
        mp = Path(matched)
        if mp.is_file():
            paths.append(mp)
    paths = sorted(set(paths))

    if not paths:
        print("error: no input files found", file=sys.stderr)
        return 2

    print(f"loading {len(paths)} file(s)...", file=sys.stderr)
    samples: List[Tuple[List[float], float, Dict[str, Any]]] = []

    for p in paths:
        with p.open("r", encoding="utf-8") as f:
            for raw in f:
                text = raw.strip()
                if not text:
                    continue
                try:
                    row = json.loads(text)
                except json.JSONDecodeError:
                    continue

                frame_id = row.get("frame_id", 0)
                lidar_matched = row.get("lidar_matched", False)
                lidar_delta_ms = row.get("lidar_delta_ms", 0)

                for obj in row.get("objects", []):
                    obj["_frame_id"] = frame_id
                    obj["_lidar_matched"] = lidar_matched
                    obj["_lidar_delta_ms"] = lidar_delta_ms

                    result = extract_sample(
                        obj,
                        image_width=args.image_width,
                        camera_fov_deg=args.camera_fov_deg,
                        min_cluster_score=args.min_cluster_score,
                        min_candidates=args.min_candidates,
                        min_dist=args.min_distance,
                        max_dist=args.max_distance,
                    )
                    if result is not None:
                        samples.append(result)
                        if args.max_samples > 0 and len(samples) >= args.max_samples:
                            break
                if args.max_samples > 0 and len(samples) >= args.max_samples:
                    break

    print(f"extracted {len(samples)} samples", file=sys.stderr)
    if not samples:
        print("error: no samples extracted (check filters)", file=sys.stderr)
        return 1

    # class distribution
    class_counts: Dict[str, int] = {}
    for _, _, meta in samples:
        cls = meta["class_name"]
        class_counts[cls] = class_counts.get(cls, 0) + 1
    print("class distribution:", file=sys.stderr)
    for cls, cnt in sorted(class_counts.items(), key=lambda x: -x[1]):
        print(f"  {cls}: {cnt}", file=sys.stderr)

    # distance distribution
    dists = sorted([t for _, t, _ in samples])
    print(f"distance range: [{dists[0]:.2f}, {dists[-1]:.2f}] m  "
          f"median={dists[len(dists)//2]:.2f}", file=sys.stderr)

    # write CSV
    with open(args.out, "w", encoding="utf-8") as out:
        header = ",".join(FEATURE_NAMES + ["target_distance_m",
                                            "class_name", "track_id",
                                            "angle_deg", "cluster_points",
                                            "frame_id"])
        out.write(header + "\n")
        for feats, target, meta in samples:
            row = ",".join(
                [f"{v:.6f}" for v in feats]
                + [f"{target:.4f}",
                   meta["class_name"],
                   str(meta["track_id"]),
                   f"{meta['angle_deg']:.2f}",
                   str(meta["cluster_points"]),
                   str(meta["frame_id"])]
            )
            out.write(row + "\n")

    print(f"wrote {args.out}", file=sys.stderr)

    # quick stats
    feats_array = [[s[0][i] for s in samples] for i in range(len(FEATURE_NAMES))]
    print("\nfeature statistics:", file=sys.stderr)
    for i, name in enumerate(FEATURE_NAMES):
        vals = feats_array[i]
        mean = sum(vals) / len(vals)
        std = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
        print(f"  {name:>20s}: mean={mean:8.4f}  std={std:8.4f}  "
              f"min={min(vals):8.4f}  max={max(vals):8.4f}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
