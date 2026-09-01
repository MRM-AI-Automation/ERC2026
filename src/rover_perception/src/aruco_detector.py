#!/usr/bin/env python3

import cv2
import logging
import socket
import threading
import time

import numpy as np
from flask import Flask, Response

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from msgs.msg import ArucoTag


app = Flask(__name__)

latest_jpeg = None
jpeg_lock = threading.Lock()
running = True


def get_jetson_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]

    except Exception:
        return "127.0.0.1"

    finally:
        sock.close()


@app.route("/")
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>ArUco Monitor</title>
        <style>
            body {
                background: #111;
                color: white;
                font-family: Arial;
                text-align: center;
                margin: 0;
                padding-top: 20px;
            }

            img {
                width: 640px;
                height: 360px;
            }
        </style>
    </head>

    <body>
        <h2>ArUco Detector Monitor</h2>
        <img src="/video_feed">
    </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def generate_frames():
    global latest_jpeg

    while running:
        with jpeg_lock:
            frame = latest_jpeg

        if frame is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: "
                + str(len(frame)).encode()
                + b"\r\n\r\n"
                + frame
                + b"\r\n"
            )

        time.sleep(0.03)


class ArucoDetector(Node):

    def __init__(self):
        super().__init__("aruco_detector")

        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_depth = None

        self.frame_lock = threading.Lock()
        self.depth_lock = threading.Lock()

        self.frame_event = threading.Event()

        self.last_detected_ids = set()

        self.last_detection_log = 0.0
        self.last_state_log = 0.0
        self.last_depth_warning = 0.0

        self.last_published_detected = False

        self.depth_info_logged = False

        self.aruco_pub = self.create_publisher(
            ArucoTag,
            "/aruco_detected",
            10,
        )

        self.image_sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/color/rect/image",
            self.image_callback,
            1,
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.depth_callback,
            1,
        )

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_5X5_100
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters,
        )

        self.detection_thread = threading.Thread(
            target=self.detection_worker,
            daemon=True,
        )

        self.detection_thread.start()

        self.flask_thread = threading.Thread(
            target=self.flask_worker,
            daemon=True,
        )

        self.flask_thread.start()

        self.get_logger().info("ArUco detector initialized")

        self.get_logger().info(
            "RGB: /zed/zed_node/rgb/color/rect/image"
        )

        self.get_logger().info(
            "Depth: /zed/zed_node/depth/depth_registered"
        )

        self.get_logger().info(
            "Output: /aruco_detected"
        )

        self.get_logger().info(
            "Dictionary: DICT_5X5_100"
        )

        self.get_logger().info(
            "No detection -> false, 0.0, 0.0, -1"
        )

        self.get_logger().info(
            "Horizontal offset -> normalized from -1.0 to +1.0"
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as e:
            self.get_logger().error(
                f"RGB conversion failed: {e}"
            )
            return

        with self.frame_lock:
            self.latest_frame = frame

        self.frame_event.set()

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )

        except Exception as e:
            self.get_logger().error(
                f"Depth conversion failed: {e}"
            )
            return

        if not self.depth_info_logged:
            self.get_logger().info(
                f"Depth format | "
                f"encoding={msg.encoding} | "
                f"dtype={depth.dtype} | "
                f"shape={depth.shape}"
            )

            self.depth_info_logged = True

        with self.depth_lock:
            self.latest_depth = depth

    def publish_no_detection(self):
        msg = ArucoTag()

        msg.is_detected = False
        msg.aruco_x = 0.0
        msg.aruco_y = 0.0
        msg.id = -1

        self.aruco_pub.publish(msg)

    def publish_detection(
        self,
        marker_id,
        distance,
        horizontal_offset,
    ):
        msg = ArucoTag()

        msg.is_detected = True
        msg.aruco_x = float(distance)
        msg.aruco_y = float(horizontal_offset)
        msg.id = int(marker_id)

        self.aruco_pub.publish(msg)

    def get_depth_median(
        self,
        depth,
        center_x,
        center_y,
    ):
        if depth is None:
            return None

        x = int(round(center_x))
        y = int(round(center_y))

        if (
            x < 0
            or y < 0
            or x >= depth.shape[1]
            or y >= depth.shape[0]
        ):
            return None

        radius = 4

        x1 = max(0, x - radius)
        x2 = min(depth.shape[1], x + radius + 1)

        y1 = max(0, y - radius)
        y2 = min(depth.shape[0], y + radius + 1)

        roi = depth[y1:y2, x1:x2]

        if roi.size == 0:
            return None

        if depth.dtype == np.uint16:
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        valid = roi_m[
            np.isfinite(roi_m)
            & (roi_m > 0.15)
            & (roi_m < 15.0)
        ]

        if valid.size < 5:
            return None

        median = float(np.median(valid))

        tolerance = max(
            0.20,
            median * 0.10,
        )

        filtered = valid[
            np.abs(valid - median) < tolerance
        ]

        if filtered.size < 5:
            return median

        return float(np.median(filtered))

    def detection_worker(self):
        while rclpy.ok():
            self.frame_event.wait()

            if not rclpy.ok():
                break

            with self.frame_lock:
                frame = self.latest_frame

            if frame is None:
                self.frame_event.clear()
                continue

            try:
                gray = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2GRAY,
                )

                corners, ids, _ = self.detector.detectMarkers(
                    gray
                )

            except Exception as e:
                self.get_logger().error(
                    f"ArUco detection failed: {e}"
                )

                self.publish_no_detection()

                self.frame_event.clear()
                continue

            valid_detection = False
            current_ids = set()

            if ids is not None:
                current_ids = set(
                    ids.flatten().tolist()
                )

                with self.depth_lock:
                    depth = self.latest_depth

                for i, marker_id in enumerate(ids.flatten()):
                    marker_corners = corners[i][0]

                    center_x = float(
                        np.mean(marker_corners[:, 0])
                    )

                    center_y = float(
                        np.mean(marker_corners[:, 1])
                    )

                    distance = self.get_depth_median(
                        depth,
                        center_x,
                        center_y,
                    )

                    if distance is None:
                        continue

                    # =============================================
                    # NORMALIZED HORIZONTAL OFFSET
                    #
                    # -1.0 = left edge
                    #  0.0 = image center
                    # +1.0 = right edge
                    # =============================================
                    frame_center_x = frame.shape[1] / 2.0

                    horizontal_offset = (
                        center_x - frame_center_x
                    ) / frame_center_x

                    horizontal_offset = float(
                        np.clip(
                            horizontal_offset,
                            -1.0,
                            1.0,
                        )
                    )

                    self.publish_detection(
                        marker_id,
                        distance,
                        horizontal_offset,
                    )

                    valid_detection = True

                    now = time.monotonic()

                    new_marker = (
                        int(marker_id)
                        not in self.last_detected_ids
                    )

                    if (
                        new_marker
                        or now - self.last_detection_log > 0.5
                    ):
                        self.get_logger().info(
                            f"Marker {int(marker_id)} | "
                            f"distance={distance:.3f} m | "
                            f"offset={horizontal_offset:.3f}"
                        )

                        self.last_detection_log = now

                    # Publish only the first valid marker found.
                    break

            if not valid_detection:
                self.publish_no_detection()

                now = time.monotonic()

                if (
                    self.last_published_detected
                    or now - self.last_state_log > 1.0
                ):
                    self.get_logger().info(
                        "ArUco state: NOT DETECTED"
                    )

                    self.last_state_log = now

            else:
                self.last_published_detected = True

            if not valid_detection:
                self.last_published_detected = False

            self.last_detected_ids = current_ids

            with self.frame_lock:
                if self.latest_frame is frame:
                    self.frame_event.clear()

    def flask_worker(self):
        global latest_jpeg

        while rclpy.ok():
            with self.frame_lock:
                frame = self.latest_frame

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                display = cv2.resize(
                    frame,
                    (320, 180),
                    interpolation=cv2.INTER_AREA,
                )

                success, encoded = cv2.imencode(
                    ".jpg",
                    display,
                    [
                        cv2.IMWRITE_JPEG_QUALITY,
                        20,
                    ],
                )

                if success:
                    with jpeg_lock:
                        latest_jpeg = encoded.tobytes()

            except Exception as e:
                self.get_logger().error(
                    f"Web stream encoding failed: {e}"
                )

            time.sleep(0.01)


def run_flask():
    logging.getLogger(
        "werkzeug"
    ).setLevel(
        logging.ERROR
    )

    ip = get_jetson_ip()

    print(
        f"ArUco monitor: http://{ip}:5000"
    )

    print(
        f"Video stream: "
        f"http://{ip}:5000/video_feed"
    )

    app.run(
        host="0.0.0.0",
        port=5000,
        threaded=True,
        debug=False,
        use_reloader=False,
    )


def main(args=None):
    global running

    rclpy.init(args=args)

    node = ArucoDetector()

    flask_thread = threading.Thread(
        target=run_flask,
        daemon=True,
    )

    flask_thread.start()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Shutdown requested"
        )

    finally:
        running = False

        node.frame_event.set()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
