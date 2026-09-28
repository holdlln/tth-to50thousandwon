"""Run installed Webots, keep all validation evidence, hide helper window."""
import argparse
import os
from pathlib import Path
import subprocess
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run-id',default='validation')
    parser.add_argument('--timeout',type=float,default=900)
    parser.add_argument('--world',default='worlds/rescue_medium.wbt')
    args=parser.parse_args()
    env=os.environ.copy()
    env.update(SAR_RUN_ID=args.run_id,SAR_BATCH='1',SAR_CAPTURE='1',SAR_PYTHON=sys.executable,
               PYTHONIOENCODING='utf-8')
    if os.name=='nt':
        home=Path(env.get('WEBOTS_HOME',r'C:\Program Files\Webots'))
        executable=home/'msys64/mingw64/bin/webots.exe'
    else:
        executable=(str(Path(env['WEBOTS_HOME'])/'webots') if env.get('WEBOTS_HOME') else shutil.which('webots'))
        if not executable: parser.error('Webots not found: set WEBOTS_HOME or add webots to PATH')
    out=ROOT/'output/runs'/args.run_id; out.mkdir(parents=True,exist_ok=True)
    command=[str(executable),'--batch','--minimize','--mode=fast','--stdout','--stderr',str(ROOT/args.world)]
    startup=None
    if os.name=='nt':
        startup=subprocess.STARTUPINFO(); startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow=0
    with (out/'webots.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,startupinfo=startup)
        print(f'Webots started: pid={process.pid}, run={args.run_id}',flush=True)
        try: code=process.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            # Terminate only this launched process tree, never unrelated sessions.
            if os.name=='nt':
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
            else: process.terminate()
            print('Webots validation timed out; logs preserved',flush=True); return 2
    print((out/'webots.log').read_text(encoding='utf-8',errors='replace')[-6000:],flush=True)
    return code


if __name__=='__main__': sys.exit(main())
