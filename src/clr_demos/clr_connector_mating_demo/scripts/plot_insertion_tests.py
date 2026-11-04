#!/usr/bin/env python3
#
# Copyright (c) 2026, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Plots how far the peg got and the peak force for each trial of the insertion tests.

This reads the rosbags and `trial.yaml` files in one or more test result folders, and saves a plot
and a table of the results in the first folder. A sweep over two values is plotted as a grid:

    python3 scripts/plot_insertion_tests.py test_results/<timestamp> [test_results/<timestamp> ...]

Depth comes from the transforms in the rosbag, relative to the true receptacle pose rather than the
detected one, so it does not include any AprilTag detection error.
Peak force is over the whole recording of the trial, which usually means the peg hitting the bore floor.
"""

import argparse
import csv
import re
from pathlib import Path

import matplotlib
import numpy as np
import rosbag2_py
import yaml
from geometry_msgs.msg import WrenchStamped
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

# In the receptacle frame, the bore axis points along -z, from the bore entry to the floor.
# See urdf/receptacle_macro.xacro: the tag is at z = 12.7 mm, and the bore entry is 109 mm in front of it.
BORE_ENTRY_Z = 0.0127 + 0.109
BORE_FLOOR_DEPTH = 0.109
# This matches the `required_depth` in trees/connector_mate_tests.xml.
SHORT_OF_FLOOR_TOLERANCE = 0.005

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
DEPTH_COLORS = ("#eef5fd", "#86b6ef", "#2a78d6", "#0d366b")
FORCE_COLORS = ("#fdf0ea", "#f5b190", "#eb6834", "#8a3211")
OUTCOME_STYLES = {
    "reached floor": {"marker": "o", "facecolor": "#2a78d6", "edgecolor": SURFACE},
    "stopped short": {"marker": "o", "facecolor": "none", "edgecolor": "#eb6834"},
    "aborted": {"marker": "X", "facecolor": "#eb6834", "edgecolor": "#eb6834"},
}


def read_trial(trial_dir: Path) -> dict | None:
    """Returns the results for a trial, or None if it never finished."""
    trial_file = trial_dir / "trial.yaml"
    bag_dir = trial_dir / "bag"
    if not trial_file.exists() or not (bag_dir / "metadata.yaml").exists():
        return None
    trial = yaml.safe_load(trial_file.read_text())

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    buffer = Buffer(cache_time=Duration(seconds=3600))
    stamps = set()
    peak_force = 0.0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in ("/tf", "/tf_static"):
            for transform in deserialize_message(data, TFMessage).transforms:
                if topic == "/tf_static":
                    buffer.set_transform_static(transform, "bag")
                else:
                    buffer.set_transform(transform, "bag")
                    stamps.add(Time.from_msg(transform.header.stamp).nanoseconds)
        elif topic == "/force_torque_sensor_broadcaster/wrench":
            force = deserialize_message(data, WrenchStamped).wrench.force
            peak_force = max(peak_force, float(np.linalg.norm([force.x, force.y, force.z])))

    depth = float("-inf")
    for stamp in sorted(stamps):
        try:
            tform = buffer.lookup_transform("receptacle", "peg_tip", Time(nanoseconds=stamp))
        except Exception:  # noqa: BLE001, S112 -- TF raises a variety of exceptions, and not every stamp has data
            continue
        depth = max(depth, BORE_ENTRY_Z - tform.transform.translation.z)

    # Older results only recorded whether the tree completed, in `succeeded`.
    completed = trial.get("tree_completed", trial["succeeded"])
    if not completed:
        outcome = "aborted"
    elif depth >= BORE_FLOOR_DEPTH - SHORT_OF_FLOOR_TOLERANCE:
        outcome = "reached floor"
    else:
        outcome = "stopped short"

    # Names look like <sweep>_<values>_run_<repeat>.
    match = re.fullmatch(r"(.*?)((?:_-?[\d.]+(?:e-?\d+)?)*)_run_(\d+)", trial["name"])
    return {
        "name": trial["name"],
        "sweep": match.group(1),
        "value": match.group(2).lstrip("_") or "none",
        "depth_mm": depth * 1000.0,
        "peak_force_n": peak_force,
        "outcome": outcome,
    }


def value_sort_key(value: str) -> tuple:
    """Sorts values numerically, including multiple values such as from combined sweeps."""
    if value == "none":
        return (0.0,)
    return tuple(float(part) for part in re.findall(r"-?[\d.]+(?:e-?\d+)?", value))


def value_label(sweep: str, value: str) -> str:
    """Shows lateral offsets in millimeters, which are saved in meters."""
    if value != "none" and "_" not in value and any(axis in sweep for axis in ("lateral", "translation", "miss")):
        return f"{float(value) * 1000.0:g}"
    return value


def plot(results: list[dict], output: Path, title: str) -> None:
    sweeps = sorted({r["sweep"] for r in results}, key=lambda s: (s != "baseline", s))
    values = {s: sorted({r["value"] for r in results if r["sweep"] == s}, key=value_sort_key) for s in sweeps}

    plt.rcParams.update({"font.size": 10, "text.color": TEXT_PRIMARY, "axes.labelcolor": TEXT_SECONDARY})
    fig, axes = plt.subplots(
        2,
        len(sweeps),
        sharey="row",
        squeeze=False,
        figsize=(max(9.0, 1.1 + 0.55 * sum(len(v) + 1 for v in values.values())), 6.5),
        gridspec_kw={"width_ratios": [len(values[s]) + 1 for s in sweeps], "wspace": 0.06, "hspace": 0.12},
        facecolor=SURFACE,
    )

    rows = (("depth_mm", "Deepest peg tip past bore entry (mm)"), ("peak_force_n", "Peak force in trial (N)"))
    for row, (key, ylabel) in enumerate(rows):
        for col, sweep in enumerate(sweeps):
            ax = axes[row][col]
            ax.set_facecolor(SURFACE)
            ax.grid(axis="y", color=GRID, linewidth=0.8)
            ax.set_axisbelow(True)
            ax.spines[["top", "right", "left"]].set_visible(False)
            ax.spines["bottom"].set_color(GRID)
            ax.tick_params(colors=TEXT_SECONDARY, length=0)
            ax.set_xlim(-0.75, len(values[sweep]) - 0.25)
            ax.set_xticks(range(len(values[sweep])))
            ax.set_xticklabels([value_label(sweep, v) for v in values[sweep]] if row == 1 else [])

            for outcome, style in OUTCOME_STYLES.items():
                points = [r for r in results if r["sweep"] == sweep and r["outcome"] == outcome]
                ax.scatter(
                    [values[sweep].index(r["value"]) for r in points],
                    [r[key] for r in points],
                    s=64,
                    linewidths=1.5,
                    zorder=3,
                    label=outcome,
                    **style,
                )

            if key == "depth_mm":
                ax.axhline(BORE_FLOOR_DEPTH * 1000.0, color=TEXT_SECONDARY, linewidth=1.0, zorder=2)
                ax.axhline(0.0, color=TEXT_SECONDARY, linewidth=1.0, linestyle=":", zorder=2)
                ax.set_title(sweep, color=TEXT_PRIMARY, fontsize=10)
            if col == 0:
                ax.set_ylabel(ylabel)
            if col == len(sweeps) - 1 and key == "depth_mm":
                for y, text in ((BORE_FLOOR_DEPTH * 1000.0, "floor"), (0.0, "bore entry")):
                    ax.annotate(
                        text,
                        (1.0, y),
                        xycoords=("axes fraction", "data"),
                        xytext=(4, 0),
                        textcoords="offset points",
                        va="center",
                        color=TEXT_SECONDARY,
                        fontsize=9,
                        annotation_clip=False,
                    )

    # The axes share these limits, which must come from all the data so that no points are clipped.
    axes[0][0].set_ylim(min(-10.0, min(r["depth_mm"] for r in results) - 5.0), BORE_FLOOR_DEPTH * 1000.0 + 8.0)
    axes[1][0].set_ylim(0.0, max(r["peak_force_n"] for r in results) * 1.08)

    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", ncols=3, frameon=False, bbox_to_anchor=(0.98, 1.0))
    fig.suptitle(title, x=0.02, ha="left", fontsize=12, color=TEXT_PRIMARY)
    fig.supxlabel("Commanded offset (mm for lateral sweeps, degrees for tilts)", color=TEXT_SECONDARY, fontsize=10)
    fig.savefig(output, dpi=150, bbox_inches="tight", facecolor=SURFACE)


def plot_grid(results: list[dict], output: Path, title: str) -> None:
    """Plots a sweep over a lateral offset and a tilt, with a heatmap each for depth and peak force."""
    points = {tuple(float(part) for part in r["value"].split("_")): r for r in results}
    offsets = sorted({offset for offset, _ in points})
    tilts = sorted({tilt for _, tilt in points})

    plt.rcParams.update({"font.size": 10, "text.color": TEXT_PRIMARY, "axes.labelcolor": TEXT_SECONDARY})
    fig, axes = plt.subplots(1, 2, figsize=(max(10.0, 1.5 * len(offsets)), 1.0 + 0.8 * len(tilts)), facecolor=SURFACE)
    panels = (
        (
            axes[0],
            "depth_mm",
            f"Deepest peg tip past bore entry (mm), with the floor at {BORE_FLOOR_DEPTH * 1000.0:.0f}",
            DEPTH_COLORS,
        ),
        (axes[1], "peak_force_n", "Peak force in trial (N)", FORCE_COLORS),
    )
    for ax, key, panel_title, colors in panels:
        cells = np.full((len(tilts), len(offsets)), np.nan)
        for (offset, tilt), result in points.items():
            cells[tilts.index(tilt), offsets.index(offset)] = result[key]

        colormap = LinearSegmentedColormap.from_list(key, colors)
        limits = (0.0, BORE_FLOOR_DEPTH * 1000.0) if key == "depth_mm" else (0.0, np.nanmax(cells))
        ax.imshow(cells, cmap=colormap, vmin=limits[0], vmax=limits[1], origin="lower", aspect="auto")
        ax.set_facecolor(SURFACE)
        ax.set_title(panel_title, color=TEXT_PRIMARY, fontsize=10, loc="left")
        ax.set_xticks(range(len(offsets)), [f"{offset * 1000.0:g}" for offset in offsets])
        ax.set_yticks(range(len(tilts)), [f"{tilt:g}" for tilt in tilts])
        ax.set_xlabel("Commanded x offset (mm)")
        ax.set_ylabel("Commanded tilt about z (degrees)")
        ax.tick_params(colors=TEXT_SECONDARY, length=0)
        ax.spines[:].set_visible(False)
        # Gaps between the cells, on the surface color.
        ax.set_xticks(np.arange(len(offsets) + 1) - 0.5, minor=True)
        ax.set_yticks(np.arange(len(tilts) + 1) - 0.5, minor=True)
        ax.grid(which="minor", color=SURFACE, linewidth=2)
        ax.tick_params(which="minor", length=0)

        for (offset, tilt), result in points.items():
            fraction = (result[key] - limits[0]) / max(limits[1] - limits[0], 1e-6)
            marker = {"reached floor": "", "stopped short": "short", "aborted": "aborted"}[result["outcome"]]
            ax.text(
                offsets.index(offset),
                tilts.index(tilt),
                f"{result[key]:.0f}" + (f"\n{marker}" if marker else ""),
                ha="center",
                va="center",
                color=SURFACE if fraction > 0.55 else TEXT_PRIMARY,
                fontsize=8.5,
            )

    fig.suptitle(title, x=0.02, ha="left", fontsize=12, color=TEXT_PRIMARY)
    fig.tight_layout()
    fig.savefig(output, dpi=150, bbox_inches="tight", facecolor=SURFACE)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("results", type=Path, nargs="+", help="Test result folders, each with one folder per trial.")
    args = parser.parse_args()

    results = []
    for folder in args.results:
        for trial_dir in sorted(path for path in folder.iterdir() if path.is_dir()):
            result = read_trial(trial_dir)
            if result is None:
                print(f"Skipping {trial_dir.name}, which did not finish.")
                continue
            results.append(result)
            print(
                f"{result['name']:32} {result['outcome']:14} "
                f"{result['depth_mm']:7.1f} mm {result['peak_force_n']:7.1f} N"
            )
    if not results:
        raise SystemExit("No finished trials found.")

    output_dir = args.results[0]
    with open(output_dir / "summary.csv", "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    title = (
        f"Insertion tests: {sum(r['outcome'] == 'reached floor' for r in results)} of {len(results)} reached the floor"
    )
    # A sweep over two values is a grid, and the rest go in columns by sweep.
    grid_results = [r for r in results if "_" in r["value"]]
    other_results = [r for r in results if "_" not in r["value"]]
    if grid_results:
        plot_grid(grid_results, output_dir / "summary_grid.png", title)
    if other_results:
        plot(other_results, output_dir / "summary.png", title)
    print(f"Saved the plots and {output_dir / 'summary.csv'} in {output_dir}.")


if __name__ == "__main__":
    main()
