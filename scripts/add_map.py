#!/usr/bin/env python3
"""
add_map.py – scaffold a complete map directory under stack_master/maps/<NAME>

Usage:
    python3 add_map.py --name <NAME> --path <FILE.png> [--csv <FILE.csv>] [--workspace <DIR>]

Given a PNG + a matching YAML (same base name, same directory), this script:
  1. Creates stack_master/maps/<NAME>/
  2. Copies <NAME>_map.png  and creates a symlink <NAME>.png → <NAME>_map.png
  3. Writes <NAME>.yaml      (nav-stack / localization format)
  4. Writes <NAME>_map.yaml  (f1tenth gym simulator format)
  5. Writes global_waypoints.json  (populated from CSV if given, else skeleton)
  6. Writes ot_sectors.yaml        (2-sector config with real track length if CSV given)
  7. Writes speed_scaling.yaml     (2-sector config with real track length if CSV given)
  8. Writes <NAME>_centerline.csv  (converted from CSV input, or header-only stub)
  9. Writes <NAME>_raceline.csv    (header only – run global planner to populate)
 10. Writes <NAME>_DonkeySim_waypoints.txt  (from CSV x,y if given, else empty)

CSV auto-detection:
  - 2 cols  → x_m, y_m              (track widths defaulted to 1.1 m)
  - 3 cols  → x_m, y_m, <ignored>   (track widths defaulted to 1.1 m)
  - 4 cols  → x_m, y_m, w_right, w_left  (used as-is)
  Lines starting with '#' are treated as comments.
"""

import argparse
import csv
import json
import math
import os
import shutil
import sys
import textwrap
from pathlib import Path
from typing import Optional

try:
    import yaml
except ImportError:
    sys.exit("pyyaml is required: pip install pyyaml")


WORKSPACE_DEFAULT = "/home/ubuntu/ws/src/racing_playground"
DEFAULT_TRACK_WIDTH = 1.1  # metres, each side


# ── CSV helpers ──────────────────────────────────────────────────────────────

class Waypoint:
    __slots__ = ("x", "y", "w_right", "w_left", "s", "psi")

    def __init__(self, x: float, y: float, w_right: float, w_left: float):
        self.x = x
        self.y = y
        self.w_right = w_right
        self.w_left = w_left
        self.s = 0.0    # arc length from start (filled later)
        self.psi = 0.0  # heading in radians  (filled later)


def load_centerline_csv(path: Path) -> list[Waypoint]:
    """Parse a centerline CSV into Waypoint objects.

    Accepted column layouts (header comment lines starting with '#' are skipped):
      2 cols: x_m, y_m
      3 cols: x_m, y_m, <any – ignored>
      4 cols: x_m, y_m, w_tr_right_m, w_tr_left_m
    """
    waypoints: list[Waypoint] = []
    with open(path, newline="") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            # support both comma and semicolon delimiters
            parts = [p.strip() for p in line.replace(";", ",").split(",")]
            if len(parts) < 2:
                continue
            try:
                x, y = float(parts[0]), float(parts[1])
            except ValueError:
                continue  # skip header rows that slipped through
            if len(parts) >= 4:
                w_r, w_l = float(parts[2]), float(parts[3])
            else:
                w_r = w_l = DEFAULT_TRACK_WIDTH
            waypoints.append(Waypoint(x, y, w_r, w_l))

    if len(waypoints) < 2:
        sys.exit(f"Error: CSV {path} must contain at least 2 data rows.")

    _compute_arc_and_heading(waypoints)
    return waypoints


def _compute_arc_and_heading(wpts: list[Waypoint]) -> None:
    """Fill in cumulative arc-length and heading for each waypoint in-place."""
    n = len(wpts)
    s = 0.0
    for i in range(n):
        wpts[i].s = s
        nx = wpts[(i + 1) % n]
        dx = nx.x - wpts[i].x
        dy = nx.y - wpts[i].y
        wpts[i].psi = math.atan2(dy, dx)
        if i < n - 1:
            s += math.hypot(dx, dy)


# ── YAML helpers ────────────────────────────────────────────────────────────

def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def parse_origin(data: dict) -> list:
    """Return origin as a plain [x, y, z] list regardless of YAML format."""
    raw = data.get("origin", [0.0, 0.0, 0.0])
    if isinstance(raw, list):
        return [float(v) for v in raw]
    raw = str(raw).strip("[]")
    return [float(v.strip()) for v in raw.split(",")]


# ── file generators ──────────────────────────────────────────────────────────

def write_nav_yaml(dest: Path, name: str, src_data: dict) -> None:
    origin = parse_origin(src_data)
    content = {
        "image": f"{name}.png",
        "resolution": float(src_data.get("resolution", 0.05)),
        "origin": origin,
        "negate": int(src_data.get("negate", 0)),
        "occupied_thresh": float(src_data.get("occupied_thresh", 0.65)),
        "free_thresh": float(src_data.get("free_thresh", 0.196)),
    }
    with open(dest, "w") as f:
        yaml.dump(content, f, default_flow_style=False, sort_keys=False)


def write_map_yaml(dest: Path, name: str, src_data: dict) -> None:
    origin = parse_origin(src_data)
    origin_str = f"[{origin[0]},{origin[1]}, {origin[2]:.6f}]"
    res = float(src_data.get("resolution", 0.05))
    lines = [
        f"image: {name}_map.png",
        f"resolution: {res}",
        f"origin: {origin_str}",
        f"negate: {int(src_data.get('negate', 0))}",
        f"occupied_thresh: {float(src_data.get('occupied_thresh', 0.65))}",
        f"free_thresh: {float(src_data.get('free_thresh', 0.196))}",
    ]
    dest.write_text("\n".join(lines) + "\n")


def write_global_waypoints(
    dest: Path, map_name: str, wpts: Optional[list[Waypoint]] = None
) -> None:
    """global_waypoints.json – populated from real waypoints when available."""
    track_len = wpts[-1].s if wpts else 0.0
    wpnts_data = []
    if wpts:
        for i, w in enumerate(wpts):
            wpnts_data.append({
                "id": i,
                "s_m": round(w.s, 7),
                "d_m": 0,
                "x_m": round(w.x, 10),
                "y_m": round(w.y, 10),
                "d_right": round(w.w_right, 4),
                "d_left": round(w.w_left, 4),
                "psi_rad": round(w.psi, 7),
                "kappa_radpm": 0.0,
                "vx_mps": 0,
                "ax_mps2": 0,
            })

    data = {
        "map_info_str": {
            "data": (
                f"{map_name} track – {len(wpts)} centerline waypoints, "
                f"track length ≈ {track_len:.2f} m"
                if wpts else
                f"{map_name} track – skeleton, update after mapping"
            )
        },
        "est_lap_time": {"data": 0.0},
        "centerline_markers": {"markers": []},
        "global_waypoints": {"markers": []},
        "raceline_markers": {"markers": []},
        "centerline_waypoints": {
            "header": {
                "stamp": {"sec": 0, "nanosec": 0},
                "frame_id": "map",
            },
            "wpnts": wpnts_data,
        },
    }
    dest.write_text(json.dumps(data, indent=4) + "\n")


def write_ot_sectors(
    dest: Path, map_name: str, track_len: float = 200.0
) -> None:
    half = round(track_len / 2, 2)
    content = textwrap.dedent(f"""\
        ot_interpolator:
          ros__parameters:
            map_name: "{map_name}"
            n_sectors: 2
            yeet_factor: 1.25
            spline_len: 30
            ot_sector_begin: 0.5

            Overtaking_sector0:
              start: 0
              end: {half}
              ot_flag: false

            Overtaking_sector1:
              start: {half}
              end: {round(track_len, 2)}
              ot_flag: false
    """)
    dest.write_text(content)


def write_speed_scaling(
    dest: Path, map_name: str, track_len: float = 200.0
) -> None:
    half = round(track_len / 2, 2)
    content = textwrap.dedent(f"""\
        sector_tuner:
          ros__parameters:
            map_name: "{map_name}"
            global_limit: 1.0
            n_sectors: 2

            Sector0:
              start: 0.0
              end: {half}
              scaling: 1.0
              only_FTG: false
              no_FTG: false

            Sector1:
              start: {half}
              end: {round(track_len, 2)}
              scaling: 1.0
              only_FTG: false
              no_FTG: false
    """)
    dest.write_text(content)


def write_centerline_csv_from_waypoints(dest: Path, wpts: list[Waypoint]) -> None:
    lines = ["# x_m, y_m, w_tr_right_m, w_tr_left_m"]
    for w in wpts:
        lines.append(f"{w.x}, {w.y}, {w.w_right}, {w.w_left}")
    dest.write_text("\n".join(lines) + "\n")


def write_centerline_csv_stub(dest: Path) -> None:
    dest.write_text("# x_m, y_m, w_tr_right_m, w_tr_left_m\n")


def write_raceline_csv(dest: Path) -> None:
    dest.write_text("# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n")


def write_donkeysim_waypoints(
    dest: Path, wpts: Optional[list[Waypoint]] = None
) -> None:
    lines = ["# x, y, z"]
    if wpts:
        for w in wpts:
            lines.append(f"{w.x},{w.y},0.0")
    dest.write_text("\n".join(lines) + "\n")


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Scaffold a complete map directory under stack_master/maps/"
    )
    parser.add_argument("--name", required=True, help="Map name (e.g. GIU)")
    parser.add_argument(
        "--path",
        required=True,
        help="Path to the source PNG. "
             "The matching .yaml must live in the same directory with the same stem.",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="Path to a centerline CSV (x,y or x,y,w_r,w_l). "
             "Relative paths are resolved from the workspace root.",
    )
    parser.add_argument(
        "--workspace",
        default=os.environ.get("WORKSPACE", WORKSPACE_DEFAULT),
        help=f"Workspace root (default: {WORKSPACE_DEFAULT})",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing files (default: skip existing)",
    )
    args = parser.parse_args()

    workspace = Path(args.workspace).resolve()
    maps_dir = workspace / "stack_master" / "maps"

    # ── resolve PNG + YAML ──────────────────────────────────────────────────
    png_path = Path(args.path)
    if not png_path.is_absolute():
        png_path = workspace / png_path
    png_path = png_path.resolve()
    if not png_path.exists():
        sys.exit(f"Error: PNG not found: {png_path}")

    yaml_src = png_path.with_suffix(".yaml")
    if not yaml_src.exists():
        sys.exit(f"Error: Expected YAML not found: {yaml_src}")

    src_data = load_yaml(yaml_src)

    # ── resolve + parse CSV ─────────────────────────────────────────────────
    wpts: Optional[list[Waypoint]] = None
    if args.csv:
        csv_path = Path(args.csv)
        if not csv_path.is_absolute():
            csv_path = workspace / csv_path
        csv_path = csv_path.resolve()
        if not csv_path.exists():
            sys.exit(f"Error: CSV not found: {csv_path}")
        wpts = load_centerline_csv(csv_path)
        track_len = wpts[-1].s
        print(f"CSV loaded: {len(wpts)} waypoints, track length ≈ {track_len:.2f} m")

    track_len = wpts[-1].s if wpts else 200.0
    name = args.name
    map_dir = maps_dir / name
    map_dir.mkdir(parents=True, exist_ok=True)

    created: list[str] = []
    skipped: list[str] = []

    def put(dest: Path, write_fn, *fn_args) -> None:
        if dest.exists() and not args.overwrite:
            skipped.append(dest.name)
            return
        write_fn(dest, *fn_args)
        created.append(dest.name)

    # ── 1. PNG files ────────────────────────────────────────────────────────
    map_png = map_dir / f"{name}_map.png"
    if not map_png.exists() or args.overwrite:
        shutil.copy2(png_path, map_png)
        created.append(map_png.name)
    else:
        skipped.append(map_png.name)

    symlink = map_dir / f"{name}.png"
    if symlink.exists() or symlink.is_symlink():
        if args.overwrite:
            symlink.unlink()
        else:
            skipped.append(symlink.name)
    if not symlink.exists():
        symlink.symlink_to(f"{name}_map.png")
        created.append(symlink.name)

    # ── 2. YAML files ───────────────────────────────────────────────────────
    put(map_dir / f"{name}.yaml",     write_nav_yaml,  name, src_data)
    put(map_dir / f"{name}_map.yaml", write_map_yaml,  name, src_data)

    # ── 3. Config files (use real track length when available) ──────────────
    put(map_dir / "global_waypoints.json", write_global_waypoints, name, wpts)
    put(map_dir / "ot_sectors.yaml",       write_ot_sectors,       name, track_len)
    put(map_dir / "speed_scaling.yaml",    write_speed_scaling,    name, track_len)

    # ── 4. Centerline CSV ───────────────────────────────────────────────────
    cl_dest = map_dir / f"{name}_centerline.csv"
    if wpts:
        put(cl_dest, write_centerline_csv_from_waypoints, wpts)
    else:
        put(cl_dest, write_centerline_csv_stub)

    # ── 5. Remaining data files ─────────────────────────────────────────────
    put(map_dir / f"{name}_raceline.csv",           write_raceline_csv)
    put(map_dir / f"{name}_DonkeySim_waypoints.txt", write_donkeysim_waypoints, wpts)

    # ── summary ─────────────────────────────────────────────────────────────
    print(f"\nMap '{name}' scaffolded → {map_dir}")
    if created:
        print("  Created:")
        for f in created:
            print(f"    + {f}")
    if skipped:
        print("  Skipped (already exist – use --overwrite to replace):")
        for f in skipped:
            print(f"    ~ {f}")

    print()
    if wpts:
        print(f"  Track length : {track_len:.2f} m  ({len(wpts)} waypoints)")
        print("  Sector boundaries in ot_sectors.yaml and speed_scaling.yaml")
        print("  have been pre-set to track length — tune per-sector as needed.")
        print()
        print("Next steps:")
        print(f"  • Run the global planner to generate {name}_raceline.csv.")
        print(f"  • Fine-tune ot_sectors.yaml and speed_scaling.yaml sector ranges.")
    else:
        print("Next steps:")
        print(f"  • Run the mapping pipeline to generate real centerline/raceline CSVs.")
        print(f"  • Re-run with --csv <centerline.csv> to populate global_waypoints.json.")
        print(f"  • Update ot_sectors.yaml and speed_scaling.yaml once track length is known.")


if __name__ == "__main__":
    main()
