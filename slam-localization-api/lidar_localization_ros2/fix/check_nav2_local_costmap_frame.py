"""Check the patched launch's YAML rewrite without importing ROS launch.
Usage: python3 check_nav2_local_costmap_frame.py /path/to/lidar_localization_ros2
"""
import ast
import copy
import os
from pathlib import Path
import sys
import tempfile
import yaml

root = Path(sys.argv[1])
source = (root / 'launch/nav2_navigation.launch.py').read_text()
tree = ast.parse(source)
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_rewrite_nav2_params')
namespace = dict(Path=Path, os=os, tempfile=tempfile, yaml=yaml)
exec(compile(ast.Module(body=[function], type_ignores=[]), '<rewrite>', 'exec'), namespace)
preset = root / 'param/nav2_humble_pointcloud.yaml'
before = yaml.safe_load(preset.read_text())
for requested, expected in [('', 'wheel_odom'), ('map', 'map'), ('site_map', 'site_map')]:
    args = [str(preset), '/test/cloud', '/test/odom', 'site_map', 'wheel_odom', 'body', None]
    output = Path(namespace['_rewrite_nav2_params'](*args, local_costmap_frame=requested))
    result = yaml.safe_load(output.read_text())
    output.unlink()
    local = result['local_costmap']['local_costmap']['ros__parameters']
    assert local['global_frame'] == expected
    assert local['robot_base_frame'] == 'body' and local['rolling_window'] is True
    assert result['behavior_server']['ros__parameters']['global_frame'] == expected
    assert result['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'site_map'
    layer = copy.deepcopy(local['voxel_layer'])
    layer['pointcloud']['topic'] = before['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['pointcloud']['topic']
    assert layer == before['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
assert yaml.safe_load(preset.read_text()) == before
print('PASS: default/custom/map frames, behavior frame, rolling window, sensor settings, source preservation')
