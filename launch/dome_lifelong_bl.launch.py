#!/usr/bin/env python3
# Launch slam_toolbox in lifelong mode: localize on existing map and extend it.

import yaml
import tempfile
from better_launch import BetterLaunch, launch_this


def yaml_override(base_file: str, override_file: str) -> str:
    def deep_merge(base: dict, override: dict) -> dict:
        result = base.copy()
        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = deep_merge(result[key], value)
            else:
                result[key] = value
        return result

    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}
    with open(override_file) as f:
        override_params = yaml.safe_load(f) or {}

    merged = deep_merge(base_params, override_params)

    temp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(merged, temp, default_flow_style=False, sort_keys=False)
    temp.close()
    return temp.name


@launch_this(ui=True)
def start_lifelong(rviz_arg: bool = False):
    bl = BetterLaunch()

    # Merge lifelong base params with dome overrides (mode: lifelong + map_file_name)
    slam_base = bl.find("slam_toolbox", "mapper_params_lifelong.yaml")
    slam_override = bl.find("dome", "slam_lifelong_patch.yaml")
    slam_config = yaml_override(slam_base, slam_override)

    print(f"******************** Merged lifelong SLAM parameters written to: {slam_config}")

    bl.include(
        "slam_toolbox",
        "online_async_launch.py",
        rviz=rviz_arg,
        sim=False,
        slam_params_file=slam_config,
    )

    bl.logger.info("***********")
