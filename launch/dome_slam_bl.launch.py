#!/usr/bin/env python3
# This is my version of launching the slam launch file from linorobot2

import yaml
import tempfile
from better_launch import BetterLaunch, launch_this

def yaml_override(base_file: str, override_file: str) -> str:
    """Merge two YAML files, with override_file taking precedence.

    Parameters
    ----------
    base_file : str
        Path to the base YAML file
    override_file : str
        Path to the override YAML file (values here take precedence)

    Returns
    -------
    str
        Path to the temporary merged YAML file
    """
    def deep_merge(base: dict, override: dict) -> dict:
        """Recursively merge two dictionaries, with override taking precedence."""
        result = base.copy()
        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                # Recursively merge nested dictionaries
                result[key] = deep_merge(result[key], value)
            else:
                # Override value (works for primitives, lists, or new keys)
                result[key] = value
        return result

    # Load base file
    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}

    # Load override file
    with open(override_file) as f:
        override_params = yaml.safe_load(f) or {}

    # Merge with override taking precedence
    merged = deep_merge(base_params, override_params)

    # Write to temporary file
    temp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(merged, temp, default_flow_style=False, sort_keys=False)
    temp.close()

    return temp.name


@launch_this(ui=True)
def start_nav(rviz_arg: bool = False):
    bl = BetterLaunch()

    # Merge slam_toolbox default params with your overrides
    slam_base = bl.find("slam_toolbox", "mapper_params_online_async.yaml")
    slam_override = bl.find("dome", "slam_param_patch.yaml")
    slam_config = yaml_override(slam_base, slam_override)

    # Merge nav2 default params with your overrides
    nav_base = bl.find("nav2_bringup", "nav2_params.yaml")
    nav_override = bl.find("dome", "nav2_param_patch.yaml")
    nav_config = yaml_override(nav_base, nav_override)

    bl.include(
        "nav2_bringup",
        "bringup_launch.py",
        rviz=rviz_arg,
        sim=False,
        params_file=nav_config
    )

    bl.include(
        "slam_toolbox",
        "online_async_launch.py",
        rviz=rviz_arg,
        sim=False,
        slam_params_file=slam_config,
    )

    bl.logger.info("***********")
