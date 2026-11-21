"""Optional scene evidence, separate from robot morphology and control policy."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SCENES = Path(__file__).with_name('scenes')


def load_scene(name, robot):
    if name == 'baseline':
        return None
    if name != 'recorded_bench':
        raise ValueError('Unknown camera setup')
    scene = json.loads((SCENES / 'recorded_bench.json').read_text())
    if scene['robot'] != robot:
        raise ValueError('This recording belongs to a different robot; retain its own camera setup')
    return scene


def reference_path(scene, camera):
    if scene is None or camera not in ('overhead', 'wrist'):
        return None
    path = (ROOT / scene['source'][camera]).resolve()
    if not path.is_relative_to(ROOT / 'logs'):
        raise ValueError('Recording must be inside the local logs directory')
    return path if path.is_file() else None


def table_point(camera, pixel, resolution, z=0.):
    """Intersect an estimated pinhole ray with the assumed table plane."""
    width, height = resolution
    focal = height / (2 * np.tan(np.deg2rad(camera['fovy_deg'] / 2)))
    ray = np.asarray(camera['rotation_matrix']) @ np.array([(pixel[0]-width/2)/focal, -(pixel[1]-height/2)/focal, -1.])
    origin = np.asarray(camera['position_m'])
    scale = (z-origin[2])/ray[2]
    if scale <= 0:
        raise ValueError('Recorded landmark is behind the camera')
    return origin + scale * ray


def available_scenes(robot):
    return ['baseline']+[p.stem for p in SCENES.glob('*.json') if json.loads(p.read_text()).get('robot')==robot]
