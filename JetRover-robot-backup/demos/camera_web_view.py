#!/usr/bin/env python3
"""Expose JetRover's RGB camera as a local MJPEG web page.

Run this on the robot, then use an SSH tunnel from the computer:
  ssh -L 8080:localhost:8080 ubuntu@ROBOT_IP
Open http://localhost:8080 in the computer's browser.

No robot ports are exposed to the Wi-Fi network; the HTTP server listens only
on 127.0.0.1 and the SSH tunnel carries the video securely.
"""

import argparse
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image


HTML = b"""<!doctype html><html><head><title>JetRover camera</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:100vw;max-height:92vh;margin-top:1rem}</style></head>
<body><h3>JetRover RGB camera</h3><img src='/stream.mjpg'></body></html>"""


class CameraNode(Node):
    def __init__(self, topic):
        super().__init__("jetrover_camera_web_view")
        self._jpeg = None
        self._lock = threading.Lock()
        self.create_subscription(Image, topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(f"Waiting for camera frames on {topic}")

    def image_callback(self, message):
        channels = 4 if message.encoding.lower() in ("rgba8", "bgra8") else 3
        expected_size = message.height * message.width * channels
        if len(message.data) < expected_size:
            self.get_logger().warning("Ignoring an image with an unexpected data size")
            return

        image = np.frombuffer(message.data, dtype=np.uint8, count=expected_size)
        image = image.reshape((message.height, message.width, channels))
        encoding = message.encoding.lower()
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        elif encoding != "bgr8":
            self.get_logger().warning(f"Unsupported image encoding: {message.encoding}")
            return

        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._lock:
                self._jpeg = encoded.tobytes()

    def current_jpeg(self):
        with self._lock:
            return self._jpeg


def make_handler(camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(HTML)))
                self.end_headers()
                self.wfile.write(HTML)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while rclpy.ok():
                    jpeg = camera.current_jpeg()
                    if jpeg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg + b"\r\n")
                        self.wfile.flush()
                    threading.Event().wait(1 / 30)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description="JetRover camera viewer over an SSH tunnel")
    parser.add_argument("--topic", default="/depth_cam/rgb/image_raw")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    rclpy.init()
    camera = CameraNode(args.topic)
    executor = SingleThreadedExecutor()
    executor.add_node(camera)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    server = None
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(camera))
        print(f"Camera web view listening on http://127.0.0.1:{args.port}")
        server.serve_forever()
    except OSError as error:
        print(f"Could not listen on port {args.port}: {error}")
        print("Choose another port, for example: --port 8081")
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.server_close()
        executor.shutdown()
        ros_thread.join(timeout=2)
        camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
