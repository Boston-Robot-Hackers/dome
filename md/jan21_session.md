# Session Notes - Jan 21, 2026

## What We Did

- Switched active map from `upstairs` to `exp4`
- Updated map path in nav2_param_patch.yaml
- Updated map path in dome_navigation_bl.launch.py

## Key Learnings

- Map configuration needs to be updated in both the nav2 params and the launch file

## Open Issues

- None

## TODO

- Test navigation with new exp4 map

## Files Modified

- `config/nav2_param_patch.yaml` - changed map_file_name to exp4
- `launch/dome_navigation_bl.launch.py` - changed map to exp4.yaml
