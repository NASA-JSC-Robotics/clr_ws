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

"""Behaviors that load and step through insertion test trials."""

from datetime import datetime
from itertools import product
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from py_trees.common import Status
from py_trees.ports import BehaviourWithPorts, PortInformation
from scipy.spatial.transform import Rotation as R

TRANSLATION_AXES = ("translation_x", "translation_y", "translation_z")
ROTATION_AXES = ("rotation_x_deg", "rotation_y_deg", "rotation_z_deg")
SWEEP_AXES = (*TRANSLATION_AXES, *ROTATION_AXES, "insertion_speed")


def expand_trials(config: dict[str, Any]) -> list[dict[str, Any]]:
    """
    Expands a test file into a list of trials.

    Each sweep runs every combination of the values of its axes, on top of the defaults.
    An axis that a sweep leaves out takes the value from `defaults`, or else a zero offset.

    Args:
        config: The test file contents, with `defaults` and `sweeps` keys.

    Returns:
        The trials, each with a unique name and the inputs of the ConnectorMate tree.
    """
    defaults = config.get("defaults", {})
    trials = []
    for sweep in config["sweeps"]:
        settings = {**defaults, **sweep}
        unknown = set(settings) - {*SWEEP_AXES, "name", "repeats"}
        if unknown:
            raise ValueError(f"Unknown keys in sweep '{sweep.get('name')}': {sorted(unknown)}")

        # Lists are the axes being varied, and anything else stays fixed for the sweep.
        varied = [axis for axis in SWEEP_AXES if isinstance(settings.get(axis), list)]
        for values in product(*(settings[axis] for axis in varied)):
            point = {**settings, **dict(zip(varied, values))}
            label = "_".join(f"{value:g}" for value in values)
            for repeat in range(settings.get("repeats", 1)):
                trials.append(
                    {
                        "name": "_".join(part for part in (sweep["name"], label, f"run_{repeat}") if part),
                        "translation_xyz": [float(point.get(axis, 0.0)) for axis in TRANSLATION_AXES],
                        "orientation_xyzw": R.from_euler(
                            "xyz", [point.get(axis, 0.0) for axis in ROTATION_AXES], degrees=True
                        )
                        .as_quat()
                        .tolist(),
                        "insertion_speed": float(point["insertion_speed"]),
                    }
                )

    names = [trial["name"] for trial in trials]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"Duplicate trial names: {duplicates}")
    return trials


class LoadInsertionTrials(BehaviourWithPorts):
    """
    Loads a test file from this package's `config/insertion_tests` folder and expands it into trials.

    All the results of this run go in a new timestamped folder inside of `output_dir`.
    """

    INPUT_PORTS = {
        "test_file": PortInformation(data_type=str, required=True),
        "output_dir": PortInformation(data_type=str, required=False, default_value="test_results"),
    }
    OUTPUT_PORTS = {"trials": PortInformation(data_type=list, required=True)}

    def update(self) -> Status:
        """Load the test file."""
        path = Path(get_package_share_directory("clr_connector_mating_demo")) / "config" / "insertion_tests"
        with open(path / self.get_input("test_file")) as file:
            trials = expand_trials(yaml.safe_load(file))

        session_dir = Path(self.get_input("output_dir")).absolute() / datetime.now().strftime("%Y%m%d_%H%M%S")
        for trial in trials:
            trial["trial_dir"] = str(session_dir / trial["name"])

        self.logger.info(f"Loaded {len(trials)} trials, saving results to {session_dir}.")
        self._set_output("trials", trials)
        return Status.SUCCESS


class UnpackInsertionTrial(BehaviourWithPorts):
    """Writes out the inputs of the ConnectorMate tree for a trial."""

    INPUT_PORTS = {"trial": PortInformation(data_type=dict, required=True)}
    OUTPUT_PORTS = {
        "translation_xyz": PortInformation(data_type=list[float], required=True),
        "orientation_xyzw": PortInformation(data_type=list[float], required=True),
        "insertion_speed": PortInformation(data_type=float, required=True),
        "bag_path": PortInformation(data_type=str, required=True),
    }

    def update(self) -> Status:
        """Output the values for this trial."""
        trial = self.get_input("trial")
        self.logger.info(f"Starting trial {trial['name']}.")
        for key in ("translation_xyz", "orientation_xyzw", "insertion_speed"):
            self._set_output(key, trial[key])
        self._set_output("bag_path", str(Path(trial["trial_dir"]) / "bag"))
        return Status.SUCCESS


class WriteInsertionTrialResult(BehaviourWithPorts):
    """
    Saves the parameters of a finished trial and whether it succeeded to `trial.yaml` in the trial's folder.

    A trial folder without this file is from a trial that never finished.
    """

    INPUT_PORTS = {
        "trial": PortInformation(data_type=dict, required=True),
        "succeeded": PortInformation(data_type=bool, required=True),
    }
    OUTPUT_PORTS = {}

    def update(self) -> Status:
        """Write the file."""
        trial = self.get_input("trial")
        succeeded = self.get_input("succeeded")
        trial_dir = Path(trial["trial_dir"])
        trial_dir.mkdir(parents=True, exist_ok=True)
        with open(trial_dir / "trial.yaml", "w") as file:
            yaml.safe_dump({**trial, "succeeded": succeeded}, file)

        self.logger.info(f"Trial {trial['name']} {'succeeded' if succeeded else 'failed'}.")
        return Status.SUCCESS
