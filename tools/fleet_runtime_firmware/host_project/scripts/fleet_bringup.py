#!/usr/bin/env python3
"""One Agent and the existing chassis launch per vehicle, isolated by ROS domain."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from urllib.parse import urlencode
from urllib.request import build_opener, ProxyHandler, Request

from fleet_positions import DEFAULT_CONFIG, load_config
from fleet_processes import Processes

ROOT = Path(__file__).resolve().parents[1]


class StatusFile:
    """Optional, atomically replaced lifecycle metadata for a parent process."""

    def __init__(self, path=None):
        self.path = None if path is None else Path(path)
        self.phase = None
        self.log_dir = None

    def update(self, phase, message, *, log_dir=None, error=None):
        if self.phase == 'failed' and phase == 'stopped':
            return
        if log_dir is not None:
            self.log_dir = str(log_dir)
        payload = {'phase': phase, 'message': message, 'log_dir': self.log_dir,
                   'error': None if error is None else str(error), 'updated_at': time.time()}
        if self.path is not None:
            descriptor, temporary = tempfile.mkstemp(
                prefix='.' + self.path.name + '.', suffix='.tmp', dir=self.path.parent)
            try:
                with os.fdopen(descriptor, 'w', encoding='utf-8') as output:
                    json.dump(payload, output, ensure_ascii=False, allow_nan=False)
                    output.write('\n')
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
        self.phase = phase

    def finish(self, phase, message, *, error=None):
        # A reporting failure must not replace the actual startup/cleanup error.
        try:
            self.update(phase, message, error=error)
        except Exception as exc:
            print(f'状态文件写入失败：{exc}', file=sys.stderr)


def runtime_request(car, values=None):
    # Local vehicle requests must not be sent through the desktop HTTP proxy.
    data = urlencode(values).encode() if values is not None else None
    endpoint = '/api/status' if data is None else '/api/runtime-config'
    request = Request(f"http://{car['ip']}{endpoint}", data=data)
    with build_opener(ProxyHandler({})).open(request, timeout=5) as response:
        payload = response.read()
    if values is not None:
        return None
    config = json.loads(payload)['runtime']
    return {k: config[k] for k in ('comm_mode', 'microros_agent_ip', 'microros_agent_port')}


def host_ip(car):
    # UDP connect selects a route without sending a datagram.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((car['ip'], car['agent_port']))
        return sock.getsockname()[0]


def validate(cars):
    for key, low, high in [('domain_id', 1, 99), ('agent_port', 1024, 65535)]:
        values = [c[key] for c in cars]
        if len(values) != len(set(values)) or any(type(v) is not int or not low <= v <= high for v in values):
            raise ValueError(f'{key} 必须唯一且处于 {low}～{high}')
    for car in cars:
        ipaddress.IPv4Address(car['ip'])
        if not car['name'].isalnum():
            raise ValueError('车辆名称只能包含字母和数字')
        if any(7400 + 250 * c['domain_id'] <= car['agent_port'] < 7650 + 250 * c['domain_id']
               for c in cars):
            raise ValueError('Agent 端口与已选 ROS 域的 DDS 端口范围重叠')


def environment(car):
    return dict(os.environ, ROS_DOMAIN_ID=str(car['domain_id']), ROS_LOCALHOST_ONLY='0',
                LEAP_AGENT_DOMAIN_ID=str(car['domain_id']))


def main():
    parser = argparse.ArgumentParser(description='连接车辆并启动每车独立的 Agent、模型、适配器和 EKF')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--cars', nargs='+', help='只启动指定车辆，例如 car01 car23')
    parser.add_argument('--configure', action='store_true', help='将不匹配的车辆通信设置改为本机专用端口；车辆会重启')
    parser.add_argument('--agent-ip', help='MCU 连接的电脑 IPv4；默认逐车通过路由自动选择')
    parser.add_argument('--timeout', type=float, default=60, help='每阶段等待实时数据的秒数')
    parser.add_argument('--dry-run', action='store_true', help='只打印计划，不联网、不修改车辆、不启动服务')
    parser.add_argument('--status-file', type=Path, help='向父进程输出原子替换的 JSON 阶段状态')
    # Find the optional output before full parsing, so malformed CLI values can
    # still be reported when the parent supplied a valid status-file argument.
    status_parser = argparse.ArgumentParser(add_help=False)
    status_parser.add_argument('--status-file', type=Path)
    status_args, _ = status_parser.parse_known_args()
    status = StatusFile(status_args.status_file)
    try:
        status.update('checking', '检查车辆配置和运行环境')
        args = parser.parse_args()
        result = _run(args, parser, status)
    except KeyboardInterrupt:
        status.finish('stopped', '启动程序已停止，本次进程已清理')
        raise
    except SystemExit as exc:
        if exc.code in (None, 0):
            status.finish('stopped', '启动程序已结束')
        else:
            status.finish('failed', '启动参数校验失败', error=f'退出码：{exc.code}')
        raise
    except Exception as exc:
        status.finish('failed', '启动失败', error=exc)
        raise
    else:
        status.finish('stopped', '启动程序已结束')
        return result


def _run(args, parser, status):
    cars = load_config(args.config)
    validate(cars)
    if args.cars:
        missing = set(args.cars) - {c['name'] for c in cars}
        if missing:
            parser.error(f'未知车辆: {sorted(missing)}')
        cars = [c for c in cars if c['name'] in args.cars]
    if not 0 < args.timeout < float('inf'):
        parser.error('timeout 必须是有限正数')
    if args.agent_ip:
        ipaddress.IPv4Address(args.agent_ip)
    for c in cars:
        print(f"{c['name']} {c['ip']} -> 电脑 UDP {c['agent_port']} -> ROS_DOMAIN_ID={c['domain_id']} -> chuyun_bringup", flush=True)
    if args.dry_run:
        return 0

    agent = ROOT / 'install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent'
    library = ROOT / 'vendor/install/lib/libmicroxrcedds_agent.so'
    if not agent.is_file() or not library.is_file() or b'LEAP_AGENT_DOMAIN_ID' not in library.read_bytes():
        raise RuntimeError('需要带域映射的本地 Agent，请先运行 ./build.sh --packages-select micro_ros_agent')
    os.environ['LD_LIBRARY_PATH'] = str(library.parent) + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    for package in ('chuyun_bringup', 'chuyun_description', 'robot_state_publisher',
                    'joint_state_publisher', 'robot_localization'):
        subprocess.run(['ros2', 'pkg', 'prefix', package], check=True, stdout=subprocess.DEVNULL)

    (ROOT / 'log').mkdir(exist_ok=True)
    # Per-domain locks also protect invocations using different config files.
    locks = []
    for c in cars:
        lock = open(ROOT / 'log' / f"fleet_domain_{c['domain_id']}.lock", 'w')
        locks.append(lock)
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f"{c['name']} 的底盘启动脚本已在运行")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('0.0.0.0', c['agent_port']))
    run = Path(tempfile.mkdtemp(prefix=time.strftime('fleet_base_%Y%m%d_%H%M%S_'), dir=ROOT / 'log'))
    print(f'日志目录：{run}', flush=True)
    from firmware_log import record
    record(cars, run / 'firmware_versions.json', kind='fleet_bringup')
    status.update('checking', '读取并核对车辆通信设置', log_dir=run)
    desired = {c['name']: {'comm_mode': 'micro_ros',
                          'microros_agent_ip': args.agent_ip or host_ip(c),
                          'microros_agent_port': c['agent_port']} for c in cars}
    # Read every vehicle first, before changing any of them.
    with ThreadPoolExecutor(max_workers=8) as pool:
        futures = {c['name']: pool.submit(runtime_request, c) for c in cars}
        old, errors = {}, []
        for name, future in futures.items():
            try:
                old[name] = future.result()
            except Exception as exc:
                errors.append(f'{name}: {exc}')
    if errors:
        raise RuntimeError('车辆状态读取失败，尚未修改任何车辆：\n' + '\n'.join(errors))
    (run / 'network_before.json').write_text(json.dumps(old, indent=2) + '\n')
    (run / 'network_target.json').write_text(json.dumps(desired, indent=2) + '\n')
    changed = [c for c in cars if any(str(old[c['name']][k]) != str(v) for k, v in desired[c['name']].items())]
    if changed and not args.configure:
        raise RuntimeError('这些车辆需设置独立 Agent 端口：' + ', '.join(c['name'] for c in changed)
                           + '。首次使用请加 --configure；它会保存通信设置并重启这些车辆。')

    processes = Processes()
    persistent = []

    def check_children():
        for label, child in persistent:
            if child.poll() is not None:
                raise RuntimeError(f'{label} 提前退出，请检查 {run}')

    def wait_data(filtered=False):
        stage = 'filtered' if filtered else 'raw'
        status.update('waiting_filtered' if filtered else 'waiting_raw',
                      '等待融合里程计数据' if filtered else '等待 MCU 里程计和 IMU 数据')
        pending = {}
        for c in cars:
            command = [sys.executable, str(ROOT / 'scripts/fleet_probe.py'), '--timeout', str(args.timeout)]
            if filtered:
                command.append('--filtered')
            pending[c['name']] = processes.start(command, run / f"{c['name']}_{stage}.log", environment(c))
        deadline = time.monotonic() + args.timeout + 15
        while pending:
            check_children()
            for name, child in list(pending.items()):
                code = child.poll()
                if code is not None:
                    if code != 0:
                        raise RuntimeError(f'{name} 未通过 {stage} 数据检查，查看 {run}/{name}_{stage}.log')
                    print(f'{name}: {"底盘融合数据就绪" if filtered else "已收到 MCU 里程计和 IMU"}', flush=True)
                    del pending[name]
            if time.monotonic() > deadline:
                raise RuntimeError(f'数据检查超时: {list(pending)}')
            time.sleep(0.2)

    try:
        for c in cars:
            child = processes.start([str(agent), 'udp4', '-p', str(c['agent_port']), '-v', '4'],
                                    run / f"{c['name']}_agent.log", environment(c))
            persistent.append((c['name'] + ' Agent', child))
        time.sleep(1)
        check_children()
        for c in changed:
            status.update('configuring', f"配置并重启 {c['name']} 的通信设置")
            print(f"配置并重启 {c['name']}：{desired[c['name']]}", flush=True)
            runtime_request(c, desired[c['name']])
            with (run / 'network_applied.jsonl').open('a') as output:
                output.write(json.dumps({'car': c['name'], 'runtime': desired[c['name']]}) + '\n')
        wait_data()
        status.update('starting_base', '启动机器人模型、适配器和 EKF')
        for c in cars:
            child = processes.start(['ros2', 'launch', 'chuyun_bringup', 'chuyun_bringup.launch.py'],
                                    run / f"{c['name']}_base.log", environment(c))
            persistent.append((c['name'] + ' 底盘', child))
        wait_data(filtered=True)
        status.update('ready', f'{len(cars)} 台底盘原始数据和融合数据检查通过')
        print(f'{len(cars)} 台底盘已启动并收到融合数据。保持此终端运行，Ctrl+C 退出。', flush=True)
        while True:
            check_children()
            time.sleep(1)
    finally:
        processes.close()
        print('本次启动的 Agent 和底盘进程已结束；车辆通信设置保留。', flush=True)
    return 0


if __name__ == '__main__':
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f'启动失败：{exc}', file=sys.stderr)
        raise SystemExit(1)
