"""Rendered MuJoCo acceptance trials; never connects to a physical robot."""
import argparse
import json
from pathlib import Path
from station_robotics.simulation import Simulation


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,default=Path('logs/robot-lab/acceptance'));a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    results=[]
    for robot in ['so101','elrobot']:
        s=Simulation(robot)
        try:
            for index,offset in enumerate([[.018,-.012],[.024,-.016],[.018,-.004]]):
                s.target_xy=s.center[:2]+offset;s.reset();s.observe();s.start()
                for _ in range(180):
                    s.step(.1);s.observe()
                    if not s.running:break
                row={'robot':robot,'trial':index,'stage':s.stage,'error_px':s.error_px,'moves':s.servo.moves,'reason':s.last_error,'history':s.history,'events':list(s.events),'simulation_only':True}
                results.append(row)
                (a.out/f'{robot}-{index}.json').write_text(json.dumps(s.state(),indent=2))
                for cam,jpg in s.frames.items():(a.out/f'{robot}-{index}-{cam}.jpg').write_bytes(jpg)
                print(json.dumps({k:v for k,v in row.items() if k not in ('history','events')}),flush=True)
            s.mode='open_loop';s.reset();s.observe();s.start()
            for _ in range(60):
                s.step(.1);s.observe()
                if not s.running:break
            results.append({'robot':robot,'trial':'single_move','stage':s.stage,'error_px':s.error_px,'moves':s.servo.moves,'simulation_only':True})
        finally:s.close()
    (a.out/'results.json').write_text(json.dumps(results,indent=2))
    assert all(r['stage']=='complete' and r['error_px']<=4 for r in results if r['trial']!='single_move'),results
    assert all(r['error_px']>4 for r in results if r['trial']=='single_move'),results
    print('Six rendered alignment trials and two single-move comparisons passed.')


if __name__=='__main__':main()
