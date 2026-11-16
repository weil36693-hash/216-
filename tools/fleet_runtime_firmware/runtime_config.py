#!/usr/bin/env python3
"""Read or update a camera/chassis HTTP API using Python's standard library."""
import argparse
import getpass
import ipaddress
import json
from pathlib import Path
import urllib.error
import urllib.request

RELEASE = "2026.11.16-runtime1"
PROJECTS = {"camera": "gc0308_http_stream", "chassis": "leap_ros_mcu_driver"}


def request(host, path, payload=None):
    ipaddress.IPv4Address(host)
    data = None if payload is None else json.dumps(payload, allow_nan=False).encode("utf-8")
    req = urllib.request.Request(f"http://{host}{path}", data=data,
                                 headers={"Content-Type": "application/json"})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=8) as response:
        raw = response.read(1024 * 1024 + 1)
    if len(raw) > 1024 * 1024:
        raise ValueError("HTTP response exceeds 1 MiB")
    result = json.loads(raw)
    if not isinstance(result, dict):
        raise ValueError("Expected one JSON object")
    return result


def verify_identity(status, device, expected_name):
    firmware = status.get("firmware", {})
    if not isinstance(firmware, dict):
        raise ValueError("Device did not report firmware information")
    project = firmware.get("project", firmware.get("project_name"))
    version = firmware.get("version", status.get("firmware_version"))
    observed = status.get("name" if device == "camera" else "device_name")
    if observed != expected_name:
        raise ValueError("Device name mismatch: recheck the car's current IP and module identity")
    if project != PROJECTS[device] or version != RELEASE:
        raise ValueError("Device type or firmware version differs from this package; no settings sent")


def redacted(value):
    if isinstance(value, dict):
        return {k: ("<redacted>" if any(word in k.lower() for word in ("password", "passwd", "secret"))
                    else redacted(v)) for k, v in value.items()}
    if isinstance(value, list):
        return [redacted(v) for v in value]
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=PROJECTS, required=True)
    parser.add_argument("--host", required=True, help="Current module IPv4 address; HTTP port 80")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status")
    commands.add_parser("get")
    update = commands.add_parser("set", help="Apply and save a partial JSON object")
    update.add_argument("--json-file", type=Path, required=True)
    update.add_argument("--expect-name", required=True)
    wifi = commands.add_parser("wifi", help="Save Wi-Fi settings and restart this module")
    wifi.add_argument("--ssid", required=True)
    wifi.add_argument("--expect-name", required=True)
    args = parser.parse_args(argv)
    try:
        path = "/api/settings" if args.device == "camera" else "/api/tuning"
        if args.command == "status":
            result = request(args.host, "/api/status")
        elif args.command == "get":
            result = request(args.host, path)
        else:
            status = request(args.host, "/api/status")
            verify_identity(status, args.device, args.expect_name)
            if args.command == "set":
                payload = json.loads(args.json_file.read_text(encoding="utf-8-sig"))
                if not isinstance(payload, dict) or not payload:
                    raise ValueError("Settings file must contain a nonempty JSON object")
            else:
                password = getpass.getpass("Wi-Fi password (empty for an open network): ")
                payload = {"ssid": args.ssid, "password": password}
                path = "/api/wifi"
            result = request(args.host, path, payload)
        print(json.dumps(redacted(result), ensure_ascii=False, indent=2))
        return 0
    except urllib.error.HTTPError as error:
        # Do not echo the submitted payload or arbitrary response pages.
        parser.exit(1, f"HTTP {error.code}: rejected or failed. Query status; check the device parameter ranges.\n")
    except (OSError, ValueError, EOFError) as error:
        parser.exit(1, f"Request failed: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
