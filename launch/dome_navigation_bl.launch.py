#!/usr/bin/env python3
import os
import yaml
import tempfile

from better_launch import BetterLaunch, launch_this
os.environ["RCUTILS_LOGGING_MIN_SEVERITY"] = "WARN"  # or INFO, WARN, ERROR, FATAL

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

@launch_this(ui=False)
def start_nav():
    bl = BetterLaunch()

    base_params = bl.find("nav2_bringup", "nav2_params.yaml")
    override_params = bl.find("dome", "nav2_param_patch.yaml")
    merged_params = yaml_override(base_params, override_params)

# Configure Slam=True to use slam_toolbox with Navigsation

    bl.include(
        "nav2_bringup",
        "bringup_launch.py",
        map="/home/pitosalas/.control/maps/exp4.yaml",
        rviz=False,
        use_sim_time=False,
        slam=True, 
        use_localization=True,
        params_file=merged_params
    )
    bl.logger.info("*** LOG ***")

