#!/usr/bin/env python3
"""CLI and import bridge to credential-free session firmware snapshots."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/leap_fleet'))
from leap_fleet.firmware_session import configured_entries, record_firmware


def record(cars, path, *, kind='experiment', asynchronous=True):
    return record_firmware(cars, path, kind=kind, asynchronous=asynchronous)


def record_for_pose(pose_topic, path, *, kind):
    cars = configured_entries(ROOT / 'config/fleet.json')
    selected = [car for car in cars if car.get('pose_topic') == pose_topic]
    # Do not guess a car identity from historical car01/car_01 topic spelling.
    if not selected:
        selected = [{'name': 'unmapped_pose_topic'}]
    return record(selected, path, kind=kind)


def main():
    parser = argparse.ArgumentParser(description='只读采集底盘和相机固件版本，保存session JSON')
    parser.add_argument('--config', type=Path, default=ROOT / 'config/fleet.json')
    parser.add_argument('--cars', nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    entries = configured_entries(args.config, args.cars)
    if args.cars and set(args.cars) != {car['name'] for car in entries}:
        parser.error('有车号不在配置中')
    record(entries, args.output, asynchronous=False)
    print(args.output)
    result = json.loads(args.output.read_text(encoding='utf-8'))
    return 0 if all(car['chassis']['status'] == car['camera']['status'] == 'ok'
                    for car in result['cars']) else 2


if __name__ == '__main__':
    raise SystemExit(main())
