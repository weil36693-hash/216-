#!/usr/bin/env python3
"""Measure the camera's MJPEG stream without ROS, OpenCV or client throttling."""
import argparse
from datetime import datetime, timezone
import json
import time
import urllib.error
import urllib.parse
import urllib.request


class MultipartCounter:
    """A frame is complete when its following MIME boundary arrives."""
    def __init__(self, boundary):
        if not boundary or len(boundary) > 200:
            raise ValueError("Invalid multipart boundary")
        self.delimiter = b"\r\n--" + boundary + b"\r\n"
        self.buffer = bytearray(b"\r\n")
        self.started = False

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            pos = self.buffer.find(self.delimiter)
            if pos < 0:
                break
            part = bytes(self.buffer[:pos])
            del self.buffer[:pos + len(self.delimiter)]
            if self.started:
                headers, sep, jpeg = part.partition(b"\r\n\r\n")
                if not sep or b"image/jpeg" not in headers.lower():
                    raise ValueError("Stream part is not a JPEG")
                if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                    raise ValueError("Incomplete JPEG in stream")
                frames.append(len(jpeg))
            self.started = True
        if len(self.buffer) > 4 * 1024 * 1024:
            raise ValueError("No valid frame boundary within 4 MiB")
        return frames


def measure(url, seconds, expected_fps=None):
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        raise ValueError("Use a full HTTP URL, e.g. http://CAMERA_IP:81/")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request = urllib.request.Request(url, headers={"User-Agent": "Maturo-camera-5fps-check/1.0"})
    with opener.open(request, timeout=5) as response:
        if response.headers.get_content_type() != "multipart/x-mixed-replace":
            raise ValueError("This URL does not return an MJPEG stream; check port 81")
        boundary = response.headers.get_param("boundary")
        if boundary is None:
            raise ValueError("Stream has no multipart boundary")
        parser = MultipartCounter(boundary.encode("ascii"))
        frame_sizes, arrival_times = [], []
        total_bytes = 0
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            chunk = response.read1(65536)
            if not chunk:
                raise ValueError("Camera disconnected before the sample completed")
            now = time.monotonic()
            total_bytes += len(chunk)
            sizes = parser.feed(chunk)
            frame_sizes.extend(sizes)
            arrival_times.extend([now] * len(sizes))
        elapsed = time.monotonic() - started
        if len(arrival_times) < 2 or arrival_times[-1] <= arrival_times[0]:
            raise ValueError("Too few completed frames to measure FPS")
        fps = (len(arrival_times) - 1) / (arrival_times[-1] - arrival_times[0])
        header = response.headers.get("X-Camera-Stream-FPS")
        return {
            "observed_at": datetime.now(timezone.utc).isoformat(),
            "url": url,
            "source_fps_header": header,
            "expected_fps": expected_fps,
            "source_header_matches_expected": None if expected_fps is None else header == str(expected_fps),
            "firmware_version_header": response.headers.get("X-Camera-Firmware"),
            "sample_seconds": round(elapsed, 3),
            "complete_frames": len(frame_sizes),
            "received_fps": round(fps, 3),
            "response_body_mbps": round(total_bytes * 8 / elapsed / 1e6, 3),
            "mean_jpeg_bytes": round(sum(frame_sizes) / len(frame_sizes)),
            "limitations": [
                "Receiver-side average; no client-side FPS limiter.",
                "Frames are counted at the following multipart boundary; one frame may remain pending.",
                "Network buffering can change arrival timing. This is not exposure FPS or control latency.",
                "Header confirms advertised configuration, not firmware identity or a performance guarantee."
            ]
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True, help="http://CAMERA_IP:81/")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--expected-fps", type=int, help="Optional expected configured source limit (1..30)")
    args = parser.parse_args()
    if args.expected_fps is not None and not 1 <= args.expected_fps <= 30:
        parser.error("--expected-fps must be between 1 and 30")
    if not 5 <= args.seconds <= 600:
        parser.error("--seconds must be between 5 and 600")
    try:
        result = measure(args.url, args.seconds, args.expected_fps)
    except (OSError, ValueError, urllib.error.URLError) as exc:
        parser.exit(1, f"Stream check failed: {exc}\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 2 if result["source_header_matches_expected"] is False else 0


if __name__ == "__main__":
    raise SystemExit(main())
