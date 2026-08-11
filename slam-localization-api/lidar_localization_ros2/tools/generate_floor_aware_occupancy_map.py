#!/usr/bin/env python3
"""
Floor-aware 3D PCD -> Nav2 2D OccupancyGrid converter.

Purpose:
- Reduce "mostly unknown/gray" maps caused by sparse per-cell observations.
- Treat locally continuous floor / ramp surfaces as traversable.
- Mark vertical structure and steep terrain as occupied.
- Allow per-floor Z slicing for multi-floor buildings.

This is a practical helper for GLIM-generated point clouds, not a replacement
for full 3D traversability estimation.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pcd", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--map-name", default="floor_aware_map")
    p.add_argument("--resolution", type=float, default=0.20)
    p.add_argument("--padding-m", type=float, default=2.0)

    # Multi-floor / slice controls.
    p.add_argument("--z-min", type=float, default=None,
                   help="Keep only points at or above this Z [m].")
    p.add_argument("--z-max", type=float, default=None,
                   help="Keep only points at or below this Z [m].")

    # Observation and floor model.
    p.add_argument("--min-points-per-cell", type=int, default=1)
    p.add_argument("--floor-percentile", type=float, default=10.0,
                   help="Low percentile used as local floor estimate.")
    p.add_argument("--floor-smooth-radius-cells", type=int, default=1,
                   help="Median smoothing radius for the local floor height grid.")

    # Obstacle / slope classification.
    p.add_argument("--obstacle-height-m", type=float, default=0.35,
                   help="Height above local floor treated as obstacle.")
    p.add_argument("--max-traversable-slope-deg", type=float, default=18.0,
                   help="Cells steeper than this are occupied.")
    p.add_argument("--inflate-radius-m", type=float, default=0.35)

    # Optional manual XY crop.
    p.add_argument("--x-min", type=float, default=None)
    p.add_argument("--x-max", type=float, default=None)
    p.add_argument("--y-min", type=float, default=None)
    p.add_argument("--y-max", type=float, default=None)
    return p.parse_args()


def load_points(path):
    cloud = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(cloud.points, dtype=np.float64)
    if pts.size == 0:
        raise RuntimeError(f"Point cloud is empty: {path}")
    return pts


def crop_points(points, args):
    mask = np.ones(points.shape[0], dtype=bool)
    if args.z_min is not None:
        mask &= points[:, 2] >= args.z_min
    if args.z_max is not None:
        mask &= points[:, 2] <= args.z_max
    if args.x_min is not None:
        mask &= points[:, 0] >= args.x_min
    if args.x_max is not None:
        mask &= points[:, 0] <= args.x_max
    if args.y_min is not None:
        mask &= points[:, 1] >= args.y_min
    if args.y_max is not None:
        mask &= points[:, 1] <= args.y_max
    out = points[mask]
    if out.size == 0:
        raise RuntimeError("No points remain after crop.")
    return out


def compute_bounds(points, args):
    min_x = args.x_min if args.x_min is not None else float(points[:, 0].min() - args.padding_m)
    max_x = args.x_max if args.x_max is not None else float(points[:, 0].max() + args.padding_m)
    min_y = args.y_min if args.y_min is not None else float(points[:, 1].min() - args.padding_m)
    max_y = args.y_max if args.y_max is not None else float(points[:, 1].max() + args.padding_m)
    return min_x, max_x, min_y, max_y


def make_cell_lists(points, resolution, min_x, max_x, min_y, max_y):
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    ix = np.clip(((points[:, 0] - min_x) / resolution).astype(np.int32), 0, width - 1)
    iy = np.clip(((points[:, 1] - min_y) / resolution).astype(np.int32), 0, height - 1)
    flat = iy * width + ix

    order = np.argsort(flat, kind="mergesort")
    flat_sorted = flat[order]
    z_sorted = points[order, 2]
    unique, starts, counts = np.unique(flat_sorted, return_index=True, return_counts=True)

    groups = {}
    for u, st, cnt in zip(unique, starts, counts):
        groups[int(u)] = z_sorted[st:st+cnt]

    return width, height, groups


def floor_grid_from_groups(width, height, groups, min_points, percentile):
    floor = np.full((height, width), np.nan, dtype=np.float64)
    count = np.zeros((height, width), dtype=np.int32)
    max_z = np.full((height, width), np.nan, dtype=np.float64)

    for flat, zs in groups.items():
        y = flat // width
        x = flat % width
        count[y, x] = len(zs)
        max_z[y, x] = float(np.max(zs))
        if len(zs) >= min_points:
            floor[y, x] = float(np.percentile(zs, percentile))
    return floor, max_z, count


def median_smooth_nan(grid, radius):
    if radius <= 0:
        return grid.copy()
    h, w = grid.shape
    out = grid.copy()
    for y in range(h):
        y0, y1 = max(0, y-radius), min(h, y+radius+1)
        for x in range(w):
            if not np.isfinite(grid[y, x]):
                continue
            x0, x1 = max(0, x-radius), min(w, x+radius+1)
            vals = grid[y0:y1, x0:x1]
            vals = vals[np.isfinite(vals)]
            if vals.size:
                out[y, x] = float(np.median(vals))
    return out


def slope_degrees(floor, resolution):
    h, w = floor.shape
    slope = np.full((h, w), np.nan, dtype=np.float64)

    for y in range(h):
        for x in range(w):
            if not np.isfinite(floor[y, x]):
                continue

            dzdx = None
            dzdy = None

            if x > 0 and x + 1 < w and np.isfinite(floor[y, x-1]) and np.isfinite(floor[y, x+1]):
                dzdx = (floor[y, x+1] - floor[y, x-1]) / (2.0 * resolution)
            elif x + 1 < w and np.isfinite(floor[y, x+1]):
                dzdx = (floor[y, x+1] - floor[y, x]) / resolution
            elif x > 0 and np.isfinite(floor[y, x-1]):
                dzdx = (floor[y, x] - floor[y, x-1]) / resolution

            if y > 0 and y + 1 < h and np.isfinite(floor[y-1, x]) and np.isfinite(floor[y+1, x]):
                dzdy = (floor[y+1, x] - floor[y-1, x]) / (2.0 * resolution)
            elif y + 1 < h and np.isfinite(floor[y+1, x]):
                dzdy = (floor[y+1, x] - floor[y, x]) / resolution
            elif y > 0 and np.isfinite(floor[y-1, x]):
                dzdy = (floor[y, x] - floor[y-1, x]) / resolution

            if dzdx is None and dzdy is None:
                slope[y, x] = 0.0
            else:
                gx = 0.0 if dzdx is None else dzdx
                gy = 0.0 if dzdy is None else dzdy
                slope[y, x] = math.degrees(math.atan(math.sqrt(gx*gx + gy*gy)))
    return slope


def inflate(mask, radius_cells):
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    out = mask.copy()
    ys, xs = np.nonzero(mask)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx*dx + dy*dy > radius_cells*radius_cells:
                continue
            yy = ys + dy
            xx = xs + dx
            valid = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
            out[yy[valid], xx[valid]] = True
    return out


def write_outputs(args, occupancy, slope, origin_x, origin_y, stats):
    outdir = Path(args.output_dir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    # Nav2 occupancy image.
    pgm = outdir / f"{args.map_name}.pgm"
    Image.fromarray(np.flipud(occupancy), mode="L").save(pgm)

    yaml_path = outdir / f"{args.map_name}.yaml"
    yaml_path.write_text(
        "\n".join([
            f'image: "{pgm.name}"',
            f"resolution: {args.resolution}",
            f"origin: [{origin_x}, {origin_y}, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            'mode: "trinary"',
        ]) + "\n",
        encoding="utf-8",
    )

    # Debug slope image: white=0deg, black=max slope or above.
    slope_img = np.full(slope.shape, 205, dtype=np.uint8)
    valid = np.isfinite(slope)
    clipped = np.clip(slope[valid], 0.0, 45.0)
    slope_img[valid] = (255.0 - clipped / 45.0 * 255.0).astype(np.uint8)
    Image.fromarray(np.flipud(slope_img), mode="L").save(outdir / f"{args.map_name}_slope_debug.pgm")

    stats_path = outdir / f"{args.map_name}_stats.json"
    stats_path.write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"pgm:   {pgm}")
    print(f"yaml:  {yaml_path}")
    print(f"slope: {outdir / (args.map_name + '_slope_debug.pgm')}")
    print(f"stats: {stats_path}")
    print(json.dumps(stats, indent=2, sort_keys=True))


def main():
    args = parse_args()
    pcd = Path(args.pcd).expanduser().resolve()
    points = crop_points(load_points(pcd), args)
    min_x, max_x, min_y, max_y = compute_bounds(points, args)

    width, height, groups = make_cell_lists(
        points, args.resolution, min_x, max_x, min_y, max_y
    )
    floor_raw, max_z, count = floor_grid_from_groups(
        width, height, groups,
        args.min_points_per_cell,
        args.floor_percentile,
    )
    floor = median_smooth_nan(floor_raw, args.floor_smooth_radius_cells)
    slope = slope_degrees(floor, args.resolution)

    observed = np.isfinite(floor)
    vertical_obstacle = observed & np.isfinite(max_z) & ((max_z - floor) >= args.obstacle_height_m)
    steep = observed & np.isfinite(slope) & (slope > args.max_traversable_slope_deg)

    occupied = vertical_obstacle | steep
    radius_cells = int(math.ceil(args.inflate_radius_m / args.resolution))
    occupied = inflate(occupied, radius_cells)
    free = observed & ~occupied

    # 205=unknown, 254=free, 0=occupied.
    occupancy = np.full((height, width), 205, dtype=np.uint8)
    occupancy[free] = 254
    occupancy[occupied] = 0

    total = occupancy.size
    stats = {
        "pcd": str(pcd),
        "point_count_after_crop": int(points.shape[0]),
        "resolution_m": args.resolution,
        "width": width,
        "height": height,
        "origin_x": min_x,
        "origin_y": min_y,
        "z_min": args.z_min,
        "z_max": args.z_max,
        "min_points_per_cell": args.min_points_per_cell,
        "floor_percentile": args.floor_percentile,
        "obstacle_height_m": args.obstacle_height_m,
        "max_traversable_slope_deg": args.max_traversable_slope_deg,
        "inflate_radius_m": args.inflate_radius_m,
        "free_cell_count": int(np.sum(occupancy == 254)),
        "occupied_cell_count": int(np.sum(occupancy == 0)),
        "unknown_cell_count": int(np.sum(occupancy == 205)),
        "unknown_ratio": float(np.sum(occupancy == 205) / total),
        "steep_cell_count_before_inflation": int(np.sum(steep)),
        "vertical_obstacle_cell_count_before_inflation": int(np.sum(vertical_obstacle)),
    }
    write_outputs(args, occupancy, slope, min_x, min_y, stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
