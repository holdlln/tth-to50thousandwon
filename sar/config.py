import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_config(path=None):
    config = json.loads(Path(path or ROOT / 'config/robot.json').read_text(encoding='utf-8'))
    for key in ('wheel_radius', 'axle_length', 'map_resolution', 'map_extent',
                'robot_radius', 'acceleration', 'mission_limit_seconds'):
        if config[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if config['expected_targets'] is not None and config['expected_targets'] < 1:
        raise ValueError('expected_targets must be positive or null')
    return config
