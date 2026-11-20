#!/usr/bin/env python3
"""Read-only local MJPEG dashboard for an RTSP/HTTP camera or a test video."""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

from vision_cboard_link import CBoardReader


PAGE = Path(__file__).with_name("vision_live_web.html")


def find_lightbar_candidates(frame):
    """Find bright paired bars for display only; their geometry is not a calibrated target."""
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)[1]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bars = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if (height * .04 <= h <= height * .20 and
                3 <= w <= width * .025 and h / w >= 3 and
                cv2.contourArea(contour) >= w * h * .25):
            bars.append((x, y, w, h))
    pairs = []
    for i, bar_a in enumerate(bars):
        for bar_b in bars[i + 1:]:
            left, right = sorted((bar_a, bar_b), key=lambda bar: bar[0])
            lx, ly, lw, lh = left
            rx, ry, rw, rh = right
            average_height = (lh + rh) / 2
            gap_ratio = (rx + rw / 2 - lx - lw / 2) / average_height
            if (not 1.5 <= gap_ratio <= 4.5 or
                    abs((ly + lh / 2) - (ry + rh / 2)) > average_height * .3 or
                    max(lh, rh) / min(lh, rh) > 1.5):
                continue
            x, y = lx, min(ly, ry)
            w, h = rx + rw - x, max(ly + lh, ry + rh) - y
            # A bright wall gap is a common false pair; armor has a darker body.
            inner = gray[y + h // 4:y + 3 * h // 4, lx + lw + 3:rx - 3]
            if inner.size == 0 or inner.mean() > 150:
                continue
            quality = abs(gap_ratio - 2.5) + abs(lh - rh) / average_height
            pairs.append((quality, (x, y, w, h)))
    return [box for _, box in sorted(pairs, key=lambda pair: pair[0])]


def find_saturated_blue_pair(frame):
    """Locate two overexposed cores in blue halos; return display box and midpoint."""
    height, width = frame.shape[:2]
    white_core = cv2.inRange(frame, (220, 220, 220), (255, 255, 255))
    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(white_core, 8)
    cores = []
    for index in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[index])
        if (not width * height * .00015 <= area <= width * height * .10 or
                w < 8 or h < 8):
            continue
        margin = round(max(w, h) * .2)
        region = frame[max(0, y - margin):min(height, y + h + margin),
                       max(0, x - margin):min(width, x + w + margin)]
        # White room lights have no surrounding blue bloom. This is a display
        # heuristic, not a validated armor classifier.
        blue, green, red = cv2.mean(region)[:3]
        if blue - red < 35 or blue - green < 15:
            continue
        core_x, core_y = (float(value) for value in centroids[index])
        peak = cv2.inRange(frame[y:y + h, x:x + w], (252, 252, 252), (255, 255, 255))
        peak_count, _peak_labels, peak_stats, peak_centroids = cv2.connectedComponentsWithStats(peak, 8)
        if peak_count > 1:
            peak_index = max(range(1, peak_count), key=lambda item: peak_stats[item, cv2.CC_STAT_AREA])
            # Bloom shifts the broad centroid toward the halo. Use the brightest
            # connected core only when it covers enough of the coarse core.
            if peak_stats[peak_index, cv2.CC_STAT_AREA] >= area * .2:
                core_x = x + float(peak_centroids[peak_index][0])
                core_y = y + float(peak_centroids[peak_index][1])
        cores.append((x, y, w, h, area, core_x, core_y))
    pairs = []
    for i, core_a in enumerate(cores):
        for core_b in cores[i + 1:]:
            left, right = sorted((core_a, core_b), key=lambda core: core[5])
            lx, ly, lw, lh, la, lcx, lcy = left
            rx, ry, rw, rh, ra, rcx, rcy = right
            average_height = (lh + rh) / 2
            if (not 1.0 <= (rcx - lcx) / average_height <= 5.5 or
                    abs(rcy - lcy) > average_height * .5 or
                    max(la, ra) / min(la, ra) > 6):
                continue
            box = (lx, min(ly, ry), rx + rw - lx,
                   max(ly + lh, ry + rh) - min(ly, ry))
            center = (round((lcx + rcx) / 2), round((lcy + rcy) / 2))
            quality = abs((rcx - lcx) / average_height - 2.0) + abs(rcy - lcy) / average_height
            pairs.append((quality, box, center))
    return min(pairs, key=lambda pair: pair[0])[1:] if pairs else None


def choose_candidate(best, geometry, saturated, configured_model_floor):
    """Return display center only for a configured-score model or a geometry pair."""
    trusted_model = best is not None and best[0] >= configured_model_floor
    selected = best[2] if trusted_model else None
    source = "model" if trusted_model else "model diagnostic" if best else None
    if geometry:
        gx, gy, gw, gh = geometry[0]
        model_near = False
        if trusted_model:
            mx, my, mw, mh = best[2]
            cx, cy = mx + mw / 2, my + mh / 2
            model_near = (gx - gw * .15 <= cx <= gx + gw * 1.15 and
                          gy - gh * .3 <= cy <= gy + gh * 1.3)
        if not model_near:
            selected, source = geometry[0], "lightbar geometry"
    selected_center = None
    if saturated:
        selected, selected_center = saturated
        source = "saturated blue pair"
    return selected, selected_center, source


class Feed:
    def __init__(self, source, loop_file, detector=None, width=1280, height=720, fps=30,
                 board_reader=None):
        self.source = int(source) if source.isdecimal() else source
        self.loop_file = loop_file
        self.detector = detector
        self.width = width
        self.height = height
        self.requested_fps = fps
        self.board_reader = board_reader
        self.configured_model_floor = getattr(detector, "configured_floor", 1.0)
        source_type = ("local video device" if isinstance(self.source, int) else
                       "network stream" if source.startswith(("rtsp://", "rtsps://", "http://", "https://")) else
                       "video file")
        self.lock = threading.Condition()
        self.closed = False
        self.jpeg = None
        self.sequence = 0
        self.last_frame_s = 0.0
        self.meta = {"status": "connecting", "frames": 0, "fps": 0.0, "candidates": 0,
                     "model_candidates": 0, "geometry_candidates": 0,
                     "candidate_source": None, "best_confidence": None,
                     "offset_percent": None, "center_px": None, "box_px": None,
                     "inference_ms": None, "resolution": None, "detector": detector is not None,
                     "score_floor": getattr(detector, "score_floor", None),
                     "diagnostic_override": getattr(detector, "diagnostic_override", False),
                     "source_type": source_type, "scene_condition": "unknown",
                     "board_connected": False, "command_sent": False}

    def set_status(self, status):
        with self.lock:
            self.meta["status"] = status
            self.lock.notify_all()

    def metadata(self):
        with self.lock:
            result = dict(self.meta)
            age = time.monotonic() - self.last_frame_s if self.last_frame_s else None
        result["frame_age_ms"] = round(age * 1000) if age is not None else None
        if age is not None and age > 2 and result["status"] == "online":
            result["status"] = "stale"
        if self.board_reader:
            result.update(self.board_reader.snapshot())
        return result

    def run(self):
        while not self.closed:
            capture = (cv2.VideoCapture(self.source, cv2.CAP_V4L2)
                       if isinstance(self.source, int) else cv2.VideoCapture(self.source))
            if not capture.isOpened():
                self.set_status("source unavailable")
                capture.release()
                time.sleep(2)
                continue
            if isinstance(self.source, int):
                # UVC's raw 4K default can fall to a few FPS on a shared USB bus.
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                capture.set(cv2.CAP_PROP_FPS, self.requested_fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.set_status("waiting for frames")
            previous_s = None
            while not self.closed:
                ok, frame = capture.read()
                if not ok or frame is None:
                    if self.loop_file and isinstance(self.source, str) and Path(self.source).is_file():
                        capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    self.set_status("stream interrupted")
                    break
                now = time.monotonic()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                visible_pixels = cv2.countNonZero(cv2.inRange(gray, 30, 255))
                scene_condition = ("dark" if visible_pixels < gray.size * .001 else "visible")
                boxes = []
                inference_ms = None
                if self.detector and scene_condition == "visible":
                    start = time.monotonic()
                    try:
                        boxes = self.detector(frame)
                    except Exception:
                        self.set_status("detector error")
                        break
                    inference_ms = round((time.monotonic() - start) * 1000, 1)
                best = max(boxes, key=lambda box: box[0], default=None)
                geometry = find_lightbar_candidates(frame) if scene_condition == "visible" else []
                saturated = find_saturated_blue_pair(frame) if scene_condition == "visible" else None
                selected, selected_center, source = choose_candidate(
                    best, geometry, saturated, self.configured_model_floor)
                height, width = frame.shape[:2]
                for confidence, _class_id, (x, y, w, h) in boxes:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (98, 229, 218), 2)
                    cv2.putText(frame, f"MODEL? {confidence:.0%}", (x, max(18, y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (98, 229, 218), 2)
                if geometry:
                    x, y, w, h = geometry[0]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (46, 183, 255), 2)
                    cv2.putText(frame, "LIGHTBAR?", (x, max(18, y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (46, 183, 255), 2)
                if saturated:
                    x, y, w, h = saturated[0]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 170, 70), 2)
                    cv2.putText(frame, "BLUE PAIR?", (x, max(18, y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 170, 70), 2)
                center = None
                if selected:
                    x, y, w, h = selected
                    center = list(selected_center) if selected_center else [round(x + w / 2), round(y + h / 2)]
                    cv2.drawMarker(frame, tuple(center), (0, 255, 255),
                                   cv2.MARKER_CROSS, 22, 2)
                ok, data = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok:
                    continue
                offset = None
                if center:
                    offset = [round((center[0] / width - .5) * 100, 1),
                              round((center[1] / height - .5) * 100, 1)]
                with self.lock:
                    self.jpeg = data.tobytes()
                    self.sequence += 1
                    self.last_frame_s = now
                    self.meta.update(status="online", frames=self.sequence,
                                     fps=round(1 / (now - previous_s), 1) if previous_s and now > previous_s else 0,
                                     resolution=[width, height], scene_condition=scene_condition,
                                     candidates=len(boxes) + bool(geometry) + bool(saturated),
                                     model_candidates=len(boxes),
                                     geometry_candidates=int(bool(geometry)) + int(bool(saturated)),
                                     candidate_source=source,
                                     best_confidence=round(best[0], 3) if best else None,
                                     offset_percent=offset, center_px=center,
                                     box_px=list(selected) if selected else None,
                                     inference_ms=inference_ms)
                    self.lock.notify_all()
                previous_s = now
            capture.release()
            if not self.closed:
                time.sleep(1)

    def stop(self):
        with self.lock:
            self.closed = True
            self.lock.notify_all()


def load_detector(model, config_path, score_floor_override=None):
    try:
        import onnxruntime as ort
        import yaml
    except ImportError as error:
        raise ValueError(f"ONNX detection requires onnxruntime and PyYAML: {error}") from error
    from nyu_vision_export_predictions import detect

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config.get("yolo_name") != "yolov5":
        raise ValueError("Only yolo_name: yolov5 is supported")
    configured_floor = float(config.get("yolov5_score_threshold", .55 if config.get("backend") == "tensorrt" else .7))
    floor = configured_floor if score_floor_override is None else score_floor_override
    session = ort.InferenceSession(str(model.resolve()), providers=["CPUExecutionProvider"])

    def run(image):
        height, width = image.shape[:2]
        found = []
        if config.get("use_roi"):
            roi = config["roi"]
            x, y, w, h = (int(roi[key]) for key in ("x", "y", "width", "height"))
            if x >= 0 and y >= 0 and x + w <= width and y + h <= height:
                found = detect(session, image[y:y + h, x:x + w], floor, x, y)
        return found or detect(session, image, floor)

    run.score_floor = floor
    run.configured_floor = configured_floor
    run.diagnostic_override = score_floor_override is not None and score_floor_override != configured_floor
    return run


def make_handler(feed):
    page = PAGE.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                data, mime = page, "text/html; charset=utf-8"
            elif self.path == "/metadata":
                data = json.dumps(feed.metadata(), ensure_ascii=False).encode("utf-8")
                mime = "application/json; charset=utf-8"
            elif self.path == "/stream.mjpeg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = 0
                try:
                    while not feed.closed:
                        with feed.lock:
                            feed.lock.wait_for(lambda: feed.closed or feed.sequence > last, timeout=1)
                            if feed.closed:
                                return
                            if feed.sequence == last:
                                continue
                            last, jpeg = feed.sequence, feed.jpeg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="RTSP/HTTP URL, camera index, or local video path")
    parser.add_argument("--model", type=Path, help="optional nyu-vision YOLOv5 ONNX model")
    parser.add_argument("--score-floor", type=float,
                        help="diagnostic candidate threshold; does not change robot configuration")
    parser.add_argument("--config", type=Path, default=Path("nyu-vision/configs/odin.yaml"))
    parser.add_argument("--loop-file", action="store_true", help="loop a local test video")
    parser.add_argument("--width", type=int, default=1280, help="requested UVC frame width")
    parser.add_argument("--height", type=int, default=720, help="requested UVC frame height")
    parser.add_argument("--fps", type=int, default=30, help="requested UVC frame rate")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--cboard-port", help="USB CDC device for read-only Seasky telemetry")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not 1 <= args.width <= 4096 or not 1 <= args.height <= 4096 or not 1 <= args.fps <= 120:
        parser.error("invalid UVC width, height, or frame rate")
    if args.score_floor is not None and (not args.model or not 0 <= args.score_floor <= 1):
        parser.error("--score-floor requires --model and a value in [0,1]")
    try:
        detector = load_detector(args.model, args.config, args.score_floor) if args.model else None
    except (OSError, ValueError) as error:
        parser.error(str(error))
    board_reader = CBoardReader(args.cboard_port) if args.cboard_port else None
    feed = Feed(args.source, args.loop_file, detector, args.width, args.height, args.fps,
                board_reader=board_reader)
    worker = threading.Thread(target=feed.run, daemon=True)
    worker.start()
    if board_reader:
        board_reader.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(feed))
    print(f"Live camera dashboard: http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        feed.stop()
        if board_reader:
            board_reader.stop()
        server.server_close()
        worker.join(timeout=3)


if __name__ == "__main__":
    main()
