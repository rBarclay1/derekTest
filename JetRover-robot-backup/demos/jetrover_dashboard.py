#!/usr/bin/env python3
"""Local-only JetRover camera, drive, and arm-control dashboard.

Run on the robot after sourcing ~/ros2_ws/.robotrc, then tunnel its port:
  ssh -L 8081:127.0.0.1:8081 ubuntu@192.168.149.1
  python3 ~/jetrover_dashboard.py
Open http://localhost:8081 on the computer.

Drive buttons use a short deadman timeout: releasing a button, changing tabs,
or losing the browser connection stops the base. Arm buttons adjust one servo
by a small pulse increment and never command more than 0..1000.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from servo_controller_msgs.msg import ServoPosition, ServoStateList, ServosPosition


JOINTS = {
    1: "Base rotation", 2: "Shoulder", 3: "Elbow", 4: "Wrist pitch",
    5: "Wrist rotate", 10: "Gripper",
}
DRIVES = {
    "forward": (0.08, 0.0, 0.0), "backward": (-0.08, 0.0, 0.0),
    "left": (0.0, 0.08, 0.0), "right": (0.0, -0.08, 0.0),
    "turn_left": (0.0, 0.0, 0.25), "turn_right": (0.0, 0.0, -0.25),
}

PAGE = '''<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>JetRover dashboard</title><style>
body{margin:0;background:#12151b;color:#edf1f7;font:16px system-ui,sans-serif;text-align:center}main{max-width:950px;margin:auto;padding:12px}
img{width:100%;max-height:58vh;object-fit:contain;background:#000;border-radius:10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;text-align:left;margin-top:12px}.card{background:#202631;padding:14px;border-radius:10px}button{font:inherit;padding:11px 14px;border:0;border-radius:7px;background:#3478d0;color:white;cursor:pointer;touch-action:none}button:active,button.active{background:#1e559c;box-shadow:0 0 0 3px #82b6ff}.drive{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;text-align:center}.drive button{min-height:48px}.blank{visibility:hidden}.joint{display:grid;grid-template-columns:1fr auto auto auto;align-items:center;gap:7px;padding:7px 0;border-bottom:1px solid #394251}.pulse{color:#9ab6d8;min-width:62px;text-align:right}.warn{color:#ffcc7a;font-size:.9rem} @media(max-width:650px){.grid{grid-template-columns:1fr}.joint{font-size:.9rem}}
</style></head><body><main><h2>JetRover control</h2><img src="/stream.mjpg" alt="Waiting for camera video"><div class="warn">Keep clear of the arm and wheels. Hold a drive button to move; release to stop. Hold Q to open or E to close the gripper.</div><div class="grid"><section class="card"><h3>Drive</h3><div class="drive"><button class="blank">.</button><button data-drive="forward">Forward</button><button class="blank">.</button><button data-drive="left">Strafe left</button><button onclick="stopDrive()">STOP</button><button data-drive="right">Strafe right</button><button class="blank">.</button><button data-drive="backward">Backward</button><button class="blank">.</button></div><p class="drive"><button data-drive="turn_left">Turn left</button><button class="blank">.</button><button data-drive="turn_right">Turn right</button></p></section><section class="card"><h3>Arm and gripper</h3><p><button data-gripper="-1">Open gripper (Q)</button> <button data-gripper="1">Close gripper (E)</button></p><p>Step: <button onclick="step=10;render()">10</button> <button onclick="step=25;render()">25</button> <button onclick="step=50;render()">50</button> pulses</p><div id="joints">Waiting for servo state…</div></section></div></main><script>
let state={positions:{},step:10};let step=10;const labels={1:'Base rotation',2:'Shoulder',3:'Elbow',4:'Wrist pitch',5:'Wrist rotate',10:'Gripper'};const heldKeys=new Set(),heldGripperKeys=new Set();let keyboardTimer=null,gripperTimer=null;
function api(path){return fetch(path).catch(()=>{});}function stopDrive(){api('/api/stop');}
function joint(id,d){api('/api/joint?id='+id+'&delta='+d+'&step='+step);setTimeout(refresh,120);}
function render(){let h='';for(const id of [1,2,3,4,5,10]){let p=state.positions[id];h+=`<div class="joint"><span>${labels[id]}</span><span class="pulse">${p===undefined?'…':p}</span><button onclick="joint(${id},-1)">−</button><button onclick="joint(${id},1)">+</button></div>`;}document.getElementById('joints').innerHTML=h;}
function refresh(){fetch('/api/state').then(r=>r.json()).then(x=>{state=x;render();}).catch(()=>{});}setInterval(refresh,600);refresh();
document.querySelectorAll('[data-drive]').forEach(b=>{let timer;const go=()=>{api('/api/drive?direction='+b.dataset.drive);};const start=e=>{e.preventDefault();b.classList.add('active');go();timer=setInterval(go,120);};const end=()=>{clearInterval(timer);b.classList.remove('active');stopDrive();};b.addEventListener('pointerdown',start);for(const e of ['pointerup','pointercancel','pointerleave'])b.addEventListener(e,end);});
document.querySelectorAll('[data-gripper]').forEach(b=>{let timer;const go=()=>api('/api/gripper_adjust?direction='+b.dataset.gripper);const start=e=>{e.preventDefault();b.classList.add('active');go();timer=setInterval(go,120);};const end=()=>{clearInterval(timer);b.classList.remove('active');};b.addEventListener('pointerdown',start);for(const e of ['pointerup','pointercancel','pointerleave'])b.addEventListener(e,end);});
function updateKeyboardDrive(){let x=(heldKeys.has('w')?0.08:0)+(heldKeys.has('s')?-0.08:0);let y=(heldKeys.has('a')?0.08:0)+(heldKeys.has('d')?-0.08:0);if(x===0&&y===0){stopDrive();return;}api('/api/drive_vector?x='+x+'&y='+y);}
function updateKeyboardGripper(){let d=(heldGripperKeys.has('q')?-1:0)+(heldGripperKeys.has('e')?1:0);if(d)api('/api/gripper_adjust?direction='+d);}
function clearKeys(){heldKeys.clear();heldGripperKeys.clear();document.querySelectorAll('[data-drive],[data-gripper]').forEach(b=>b.classList.remove('active'));clearInterval(keyboardTimer);clearInterval(gripperTimer);keyboardTimer=null;gripperTimer=null;stopDrive();}
document.addEventListener('keydown',e=>{let key=e.key.toLowerCase();if(!['w','a','s','d','q','e'].includes(key)||e.repeat)return;e.preventDefault();if(['q','e'].includes(key)){heldGripperKeys.add(key);document.querySelector('[data-gripper="'+(key==='q'?'-1':'1')+'"]').classList.add('active');updateKeyboardGripper();if(!gripperTimer)gripperTimer=setInterval(updateKeyboardGripper,120);return;}heldKeys.add(key);let direction={w:'forward',s:'backward',a:'left',d:'right'}[key];document.querySelector('[data-drive="'+direction+'"]').classList.add('active');updateKeyboardDrive();if(!keyboardTimer)keyboardTimer=setInterval(updateKeyboardDrive,120);});
document.addEventListener('keyup',e=>{let key=e.key.toLowerCase();if(['q','e'].includes(key)&&heldGripperKeys.has(key)){heldGripperKeys.delete(key);document.querySelector('[data-gripper="'+(key==='q'?'-1':'1')+'"]').classList.remove('active');if(heldGripperKeys.size===0){clearInterval(gripperTimer);gripperTimer=null;}return;}if(!heldKeys.has(key))return;heldKeys.delete(key);let direction={w:'forward',s:'backward',a:'left',d:'right'}[key];document.querySelector('[data-drive="'+direction+'"]').classList.remove('active');if(heldKeys.size===0){clearInterval(keyboardTimer);keyboardTimer=null;}updateKeyboardDrive();});document.addEventListener('visibilitychange',()=>{if(document.hidden)clearKeys();});window.addEventListener('blur',clearKeys);window.addEventListener('beforeunload',stopDrive);
</script></body></html>'''.encode('utf-8')


class Robot(Node):
    def __init__(self):
        super().__init__('jetrover_dashboard')
        self.camera_jpeg = None
        self.lock = threading.Lock()
        self.positions = {servo_id: 500 for servo_id in JOINTS}
        self.positions[10] = 700
        self.deadline = 0.0
        self.drive_message = Twist()
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.servo_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(ServoStateList, '/controller_manager/servo_states', self.on_servo_states, 10)
        self.create_timer(0.05, self.drive_tick)
        self.get_logger().info('Dashboard ready; waiting for camera and servo-state messages.')

    def on_image(self, msg):
        channels = 4 if msg.encoding.lower() in ('rgba8', 'bgra8') else 3
        data = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.width * channels)
        if data.size != msg.height * msg.width * channels:
            return
        image = data.reshape((msg.height, msg.width, channels))
        encoding = msg.encoding.lower()
        conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR}
        if encoding in conversions:
            image = cv2.cvtColor(image, conversions[encoding])
        elif encoding != 'bgr8':
            return
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.camera_jpeg = encoded.tobytes()

    def on_servo_states(self, msg):
        with self.lock:
            for state in msg.servo_state:
                if state.id in JOINTS:
                    self.positions[state.id] = int(state.position)

    def drive(self, direction):
        x, y, z = DRIVES[direction]
        self.drive_vector(x, y, z)

    def drive_vector(self, x, y, z=0.0):
        with self.lock:
            self.drive_message = Twist()
            self.drive_message.linear.x, self.drive_message.linear.y = x, y
            self.drive_message.angular.z = z
            self.deadline = time.monotonic() + 0.35

    def stop(self):
        with self.lock:
            self.deadline = 0.0
            self.drive_message = Twist()
        self.cmd_pub.publish(Twist())

    def drive_tick(self):
        with self.lock:
            active = time.monotonic() < self.deadline
            message = self.drive_message if active else Twist()
        self.cmd_pub.publish(message)

    def set_joint_target(self, servo_id, target):
        if servo_id not in JOINTS:
            raise ValueError('Unknown servo')
        target = max(0, min(1000, int(target)))
        with self.lock:
            self.positions[servo_id] = target
        message = ServosPosition()
        message.duration = 0.5
        message.position_unit = 'pulse'
        position = ServoPosition()
        position.id, position.position = servo_id, float(target)
        message.position = [position]
        self.servo_pub.publish(message)
        return target

    def adjust_joint(self, servo_id, direction, step):
        if servo_id not in JOINTS:
            raise ValueError('Unknown servo')
        step = max(1, min(step, 50))
        with self.lock:
            target = self.positions[servo_id] + direction * step
        return self.set_joint_target(servo_id, target)

    def adjust_gripper(self, direction):
        # Vendor examples use lower pulses to release and higher pulses to grip.
        # Five pulses per update, repeated every 0.12 s, gives deliberate motion.
        with self.lock:
            target = max(150, min(700, self.positions[10] + int(direction) * 5))
        return self.set_joint_target(10, target)


def handler_for(robot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def send_json(self, data):
            body = json.dumps(data).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            parsed, query = urlparse(self.path), parse_qs(urlparse(self.path).query)
            if parsed.path == '/':
                self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(PAGE))); self.end_headers(); self.wfile.write(PAGE); return
            if parsed.path == '/api/state':
                with robot.lock: self.send_json({'positions': robot.positions}); return
            if parsed.path == '/api/stop': robot.stop(); self.send_json({'ok': True}); return
            if parsed.path == '/api/drive':
                direction = query.get('direction', [''])[0]
                if direction not in DRIVES: self.send_error(400, 'Unknown direction'); return
                robot.drive(direction); self.send_json({'ok': True}); return
            if parsed.path == '/api/drive_vector':
                try:
                    x, y = float(query['x'][0]), float(query['y'][0])
                except (KeyError, ValueError): self.send_error(400, 'Bad drive vector'); return
                if abs(x) > 0.08 or abs(y) > 0.08: self.send_error(400, 'Drive vector out of range'); return
                robot.drive_vector(x, y); self.send_json({'ok': True}); return
            if parsed.path == '/api/joint':
                try:
                    target = robot.adjust_joint(int(query['id'][0]), int(query['delta'][0]), int(query.get('step', ['10'])[0]))
                    self.send_json({'ok': True, 'target': target})
                except (KeyError, ValueError): self.send_error(400, 'Bad joint request')
                return
            if parsed.path == '/api/gripper_adjust':
                try: direction = int(query['direction'][0])
                except (KeyError, ValueError): self.send_error(400, 'Bad gripper request'); return
                if direction not in (-1, 1): self.send_error(400, 'Bad gripper direction'); return
                self.send_json({'ok': True, 'target': robot.adjust_gripper(direction)})
                return
            if parsed.path != '/stream.mjpg': self.send_error(404); return
            self.send_response(200); self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame'); self.end_headers()
            try:
                while rclpy.ok():
                    with robot.lock: frame = robot.camera_jpeg
                    if frame:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n' + f'Content-Length: {len(frame)}\r\n\r\n'.encode() + frame + b'\r\n'); self.wfile.flush()
                    time.sleep(1 / 30)
            except (BrokenPipeError, ConnectionResetError): pass
    return Handler


def main():
    rclpy.init(); robot = Robot(); executor = SingleThreadedExecutor(); executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start(); server = ThreadingHTTPServer(('127.0.0.1', 8081), handler_for(robot))
    print('Open http://localhost:8081 through your SSH tunnel.')
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        robot.stop(); server.server_close(); executor.shutdown(); thread.join(timeout=2); robot.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
