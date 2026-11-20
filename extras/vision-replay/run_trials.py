"""Build configurations outside upstream, then replay at the original sensor rate."""
import argparse
import json
from pathlib import Path
import subprocess
import time

ROOT = Path('/orb_work')
SOURCE = Path('/sensor_data')
DATA = {
    'mapping': SOURCE / 'mapping_20261117_074657',
    'restart_minus15': SOURCE / 'navigation_20261117_155645',
    'start_plus20': ROOT / 'data/start_plus20',
    'west_minus35': ROOT / 'data/west_minus35',
    'east_plus90': ROOT / 'data/east_plus90',
}
TEMPLATE = '''%YAML:1.0
File.version: "1.0"
Camera.type: "Rectified"
Camera1.fx: 467.74270306499267
Camera1.fy: 467.74270306499267
Camera1.cx: 320.0
Camera1.cy: 240.0
Camera.width: 640
Camera.height: 480
Camera.fps: {fps}
Camera.RGB: 0
Stereo.ThDepth: 60.0
Stereo.b: 0.16
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [0., 0., 1., 0., -1., 0., 0., 0., 0., -1., 0., 0., 0., 0., 0., 1.]
IMU.NoiseGyro: 1.414213562e-5
IMU.NoiseAcc: 1.414213562e-4
IMU.GyroWalk: 1.0e-5
IMU.AccWalk: 1.0e-4
IMU.Frequency: 200.0
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
Viewer.imageViewScale: 1.0
loopClosing: {loop}
'''

def trial(name, dataset, load_map=None, stride=1, loop=None, fast_init=True):
    restart = load_map is not None
    loop = restart if loop is None else loop
    results = ROOT / 'results' / name
    results.mkdir(parents=True, exist_ok=True)
    cfg = results/'settings.yaml'
    tag = str(Path('results') / (load_map if restart else name) / 'map')
    text = TEMPLATE.format(fps=round(62.5/stride),loop=int(loop))
    if fast_init:
        # The official legacy loader exposes fastInit, while File.version 1.0
        # does not read that flag. Both describe the same rectified camera.
        text = text.replace('File.version: "1.0"\n','').replace('"Rectified"','"PinHole"').replace('Camera1.','Camera.')
        text = text.replace('Stereo.ThDepth: 60.0\nStereo.b: 0.16','ThDepth: 60.0\nCamera.bf: 74.83883249039883\nCamera.k1: 0.0\nCamera.k2: 0.0\nCamera.p1: 0.0\nCamera.p2: 0.0')
        text = text.replace('IMU.T_b_c1:', 'Tbc:').replace('IMU.Frequency: 200.0','IMU.Frequency: 200\nIMU.fastInit: 1')
    cfg.write_text(text+
                   f'System.{"Load" if restart else "Save"}Atlas{"From" if restart else "To"}File: "{tag}"\n')
    args = [str(ROOT/'replay'),str(ROOT/'ORB_SLAM3/Vocabulary/ORBvoc.txt'),str(cfg),str(DATA[dataset]),
            str(results/'poses.csv'),'restart' if restart else 'mapping',str(stride)]
    print('START',name,flush=True)
    wall = time.monotonic()
    with (results/'console.log').open('w') as log:
        try:
            code = subprocess.run(args,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=160).returncode
        except subprocess.TimeoutExpired:
            code = -124
    meta = {'name':name,'data':dataset,'loaded_from':load_map,'stride':stride,'loop_closing':bool(loop),'fast_init':fast_init,'returncode':code,
            'wall_seconds_including_startup':time.monotonic()-wall,'command':args}
    (results/'run.json').write_text(json.dumps(meta,indent=2))
    print('END', json.dumps(meta), flush=True)
    if code or not (results/'poses.csv').exists() or (not restart and not (results/'map.osa').exists()):
        raise RuntimeError('Inspect '+str(results/'console.log'))

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('name')
    p.add_argument('dataset',choices=list(DATA))
    p.add_argument('--load-map')
    p.add_argument('--stride',type=int,default=1)
    p.add_argument('--loop',type=int,choices=[0,1])
    p.add_argument('--strict-init',action='store_true')
    a = p.parse_args()
    trial(a.name,a.dataset,a.load_map,a.stride,a.loop,not a.strict_init)
