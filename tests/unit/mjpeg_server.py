"""A local HTTP server serving MJPEG like a camera server, for the url: camera tests (no hardware)."""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest


class MjpegServer:
    """Serves /stream.mjpg (multipart MJPEG at ``fps``; each frame's gray level is its number mod 250),
    /nolength.mjpg (parts without Content-Length), /latest.jpg, and two misbehaving streams whose first part
    header never ends: /trickle.mjpg (a byte every 50 ms) and /longline.mjpg (100 kB, then a byte every 50 ms). ``drop_after`` closes each stream after
    that many frames."""

    def __init__(self, fps=100.0, drop_after=None, part_headers=b""):
        cv2 = pytest.importorskip("cv2")
        self.fps, self.drop_after, self.part_headers = fps, drop_after, part_headers
        self.sent = 0
        self.connections = 0

        def jpeg(n):
            ok, data = cv2.imencode(".jpg", np.full((48, 64, 3), n % 250, np.uint8))
            assert ok
            return data.tobytes()

        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                server.connections += 1
                if self.path == "/latest.jpg":
                    server.sent += 1
                    data = jpeg(server.sent)
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if self.path in ("/trickle.mjpg", "/longline.mjpg"):
                    # a misbehaving server: one part header that never ends, a byte at a time or all at once
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        self.wfile.write(b"--frame\r\nX-Pad: ")
                        if self.path == "/longline.mjpg":
                            self.wfile.write(b"a" * 100_000)
                        while True:
                            self.wfile.write(b"a")
                            self.wfile.flush()
                            time.sleep(0.05)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
                if self.path == "/text":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(b"<html></html>")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                n = 0
                try:
                    while server.drop_after is None or n < server.drop_after:
                        server.sent += 1
                        n += 1
                        data = jpeg(server.sent)
                        head = b"--frame\r\nContent-Type: image/jpeg\r\n" + server.part_headers
                        if self.path != "/nolength.mjpg":
                            head += b"Content-Length: " + str(len(data)).encode() + b"\r\n"
                        self.wfile.write(head + b"\r\n" + data + b"\r\n")
                        self.wfile.flush()
                        time.sleep(1.0 / server.fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.httpd.daemon_threads = True
        self.base = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()
