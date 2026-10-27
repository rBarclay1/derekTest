#!/usr/bin/env python3
"""Run the supervised trial using Derek's live ROS environment and DDS peers."""
import fcntl
import json
from pathlib import Path
import subprocess
import tempfile


def run_ros_script(script, timeout=35):
    with open('/tmp/derek-approach-trial.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = None
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                arguments = path.read_bytes().split(b'\0')
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                continue
            if any(arg.endswith(b'/lib/ros_robot_controller/ros_robot_controller')
                   for arg in arguments):
                process = path.parent
                break
        if process is None:
            raise RuntimeError('Robot controller is not running')
        env = dict(item.decode().split('=', 1)
                   for item in (process/'environ').read_bytes().split(b'\0') if b'=' in item)
        addresses = json.loads(subprocess.check_output(
            ['ip', '-j', 'addr', 'show', 'dev', 'wlan0']))
        alias_present = any(address.get('local') == '192.168.149.1'
                            for device in addresses for address in device.get('addr_info', []))
        alias_added = False
        try:
            if not alias_present:
                subprocess.run(['sudo', '-n', 'ip', 'addr', 'add',
                                '192.168.149.1/32', 'dev', 'wlan0'], check=True)
                alias_added = True
            peers = ''.join(
                '<locator><udpv4><address>192.168.149.1</address>'
                f'<port>{port}</port></udpv4></locator>' for port in range(7410, 7512, 2))
            xml = ('<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
                   '<participant profile_name="supervised_trial" is_default_profile="true">'
                   '<rtps><builtin><initialPeersList>' + peers +
                   '</initialPeersList></builtin></rtps></participant></profiles>')
            with tempfile.NamedTemporaryFile(mode='w', suffix='.xml') as profile:
                profile.write(xml)
                profile.flush()
                env['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile.name
                env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
                subprocess.run(['/usr/bin/python3', str(script)],
                    env=env, timeout=timeout, check=True)
        finally:
            if alias_added:
                subprocess.run(['sudo', '-n', 'ip', 'addr', 'del',
                                '192.168.149.1/32', 'dev', 'wlan0'], check=True)


if __name__ == '__main__':
    run_ros_script(Path(__file__).resolve().with_name('approach_camera_trial.py'))
