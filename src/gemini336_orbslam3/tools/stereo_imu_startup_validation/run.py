import json, os, signal, subprocess, time
import yaml
from pathlib import Path
root=Path('/workspaces/gemini336-orbslam3')
out=Path('/validation')
prefix=subprocess.check_output(['ros2','pkg','prefix','gemini336_orbslam3'],text=True).strip()
params=Path(prefix)/'share/gemini336_orbslam3/config/stereo_imu_slam.yaml'
exe=Path(prefix)/'lib/gemini336_orbslam3/slam_node'
assert params.is_file() and exe.is_file()
command=[str(exe),'--ros-args','--params-file',str(params)]
(out/'command.json').write_text(json.dumps({'command':command,'domain':os.environ.get('ROS_DOMAIN_ID'),'rmw':os.environ.get('RMW_IMPLEMENTATION'),'params':params.read_text()},indent=2))
result={}
def inspect(args,name):
    p=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=20)
    (out/name).write_text(p.stdout)
    if p.returncode: raise RuntimeError(f'{name}: exit {p.returncode}')
    return p.stdout
with (out/'node.log').open('w') as log:
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
    start=time.monotonic()
    try:
        deadline=start+60
        while 'Stereo SLAM initialized:' not in (out/'node.log').read_text():
            if process.poll() is not None: raise RuntimeError(f'Exited during startup: {process.returncode}')
            if time.monotonic()>deadline: raise RuntimeError('Startup exceeded 60 seconds')
            time.sleep(.2)
        result['startup_sec']=time.monotonic()-start
        ready=time.monotonic()
        inspect(['ros2','node','info','/slam_node','--no-daemon'],'node_info.txt')
        snapshot=inspect(['ros2','param','dump','/slam_node'],'parameters.yaml')
        actual=yaml.safe_load(snapshot)['/slam_node']['ros__parameters']
        expected=yaml.safe_load(params.read_text())['slam_node']['ros__parameters']
        def flatten(data, prefix=''):
            result={}
            for key,value in data.items():
                name=prefix+key
                if isinstance(value,dict): result.update(flatten(value,name+'.'))
                else: result[name]=value
            return result
        actual=flatten(actual)
        for key,value in expected.items():
            assert actual[key]==value and type(actual[key])==type(value),(key,actual[key],value)
        result['verified_parameter_count']=len(expected)
        for name,topic in [('left','/camera/left_ir/image_raw'),('right','/camera/right_ir/image_raw'),('imu','/camera/gyro_accel/sample')]:
            info=inspect(['ros2','topic','info',topic,'--verbose','--no-daemon'],name+'_topic.txt')
            assert 'Publisher count: 0' in info,(topic,info)
            assert 'Subscription count: 1' in info,(topic,info)
            assert 'BEST_EFFORT' in info,(topic,info)
        while time.monotonic()-ready<6.2:
            if process.poll() is not None: raise RuntimeError('Exited while waiting without inputs')
            time.sleep(.1)
        assert process.poll() is None
        result['alive_without_input_sec']=time.monotonic()-ready
        stop=time.monotonic()
        process.send_signal(signal.SIGINT)
        result['exit_code']=process.wait(timeout=20)
        result['shutdown_sec']=time.monotonic()-stop
        assert result['exit_code']==0,result
        text=(out/'node.log').read_text()
        for expected in ['Stereo-Inertial','Left camera to Imu Transform (Tbc)', 'Stereo SLAM shutdown returned','remaining_frames=0','processed=0']:
            assert expected in text,expected
        assert text.count('Stereo SLAM shutdown returned')==1
        assert text.count('scope=total')==1
        assert text.count('coordination: final=true')==1
        assert 'First stereo frame processed' not in text
        result['passed']=True
    except Exception as e:
        result['passed']=False
        result['error']=str(e)
        raise
    finally:
        if process.poll() is None:
            result['forced_termination']=True
            process.kill();process.wait()
        (out/'result.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
