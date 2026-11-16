"""Read-only, credential-free firmware snapshots for experiment sessions.

No ROS imports. Camera addresses are discovered from the selected chassis, not
guessed from an earlier DHCP lease. Network collection runs outside control loops.
"""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
import time
import uuid
from urllib.request import build_opener, HTTPRedirectHandler, ProxyHandler, Request

MAX_STATUS_BYTES = 32768


def now():
    return datetime.now(timezone.utc).isoformat()


def text(value, limit=160):
    if not isinstance(value, str):
        return None
    return ''.join(char for char in value[:limit] if char.isprintable()) or None


def local_ipv4(value):
    address = ipaddress.IPv4Address(value)
    if (not address.is_private or address.is_loopback or address.is_unspecified
            or address.is_multicast):
        raise ValueError('Expected a private LAN IPv4 address')
    return str(address)


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError('Device status redirects are not accepted')


def fetch_status(ip, timeout=2.0):
    address = local_ipv4(ip)
    request = Request(f'http://{address}/api/status', headers={'Accept': 'application/json'})
    opener = build_opener(ProxyHandler({}), NoRedirect())
    with opener.open(request, timeout=timeout) as response:
        raw = response.read(MAX_STATUS_BYTES + 1)
    if len(raw) > MAX_STATUS_BYTES:
        raise ValueError('Device status exceeds size limit')
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError('Expected a JSON object')
    return value


def _device_snapshot(payload, ip, *, camera=False):
    """Never copy runtime/settings/wifi objects, or raw error/response bodies."""
    firmware = payload.get('firmware')
    firmware = firmware if isinstance(firmware, dict) else {}
    fields = ('version', 'project', 'project_name', 'build', 'build_date',
              'idf_version', 'elf_sha256', 'app_sha256')
    clean = {key: text(firmware[key]) for key in fields if text(firmware.get(key))}
    if not clean.get('version') and text(payload.get('firmware_version')):
        clean['version'] = text(payload['firmware_version'])
    revision = firmware.get('motion_safety_revision')
    if type(revision) is int and 0 <= revision < 1000000:
        clean['motion_safety_revision'] = revision
    identity = text(payload.get('name') if camera else payload.get('device_name'))
    result = {'status': 'ok' if clean.get('version') else 'version_unavailable',
              'ip': ip, 'device_name': identity, 'firmware': clean or None}
    if identity:
        match = re.fullmatch(r'Maturo_(?:CAM_)?([0-9A-Fa-f]{12})', identity)
        if match:
            raw = match[1].lower()
            result['mac'] = ':'.join(raw[n:n + 2] for n in range(0, 12, 2))
    sync = payload.get('time_sync')
    if isinstance(sync, dict):
        result['time_sync'] = {}
        if type(sync.get('synced')) is bool:
            result['time_sync']['synced'] = sync['synced']
        if text(sync.get('server')):
            result['time_sync']['server'] = text(sync['server'])
        value = sync.get('last_sync_unix_us')
        # cJSON serializes an integer-valued double using scientific notation
        # when shorter. Preserve valid microsecond epochs regardless of JSON's
        # lexical number form, within the exact IEEE-754 integer range.
        if (type(value) in (int, float) and math.isfinite(value)
                and 0 <= value < 2**53 and int(value) == value):
            result['time_sync']['last_sync_unix_us'] = int(value)
        for key in ('sync_interval_seconds', 'last_sync_age_seconds'):
            value = sync.get(key)
            if type(value) in (int, float) and 0 <= value < 10**12:
                result['time_sync'][key] = value
    wifi = payload.get('wifi')
    if isinstance(wifi, dict) and type(wifi.get('power_save')) in (bool, int):
        result['wifi_power_save'] = wifi['power_save']
    return result


def collect_car(car, *, fetcher=fetch_status, timeout=2.0):
    result = {'car': text(car.get('name')), 'checked_at': now(),
              'chassis': {'status': 'unavailable'},
              'camera': {'status': 'not_discovered'}}
    try:
        ip = local_ipv4(car.get('ip'))
        payload = fetcher(ip, timeout=timeout)
        if not isinstance(payload, dict):
            raise ValueError('Expected an object')
        result['chassis'] = _device_snapshot(payload, ip)
    except Exception as error:
        result['chassis']['error_type'] = type(error).__name__
        return result
    advertised = payload.get('camera')
    if not isinstance(advertised, dict):
        return result
    camera_name = text(advertised.get('name'))
    result['camera']['advertised_name'] = camera_name
    try:
        camera_ip = local_ipv4(advertised.get('ip'))
        observed = fetcher(camera_ip, timeout=timeout)
        if not isinstance(observed, dict):
            raise ValueError('Expected an object')
        camera = _device_snapshot(observed, camera_ip, camera=True)
        camera['advertised_name'] = camera_name
        if camera_name and camera.get('device_name') and camera_name != camera['device_name']:
            camera['status'] = 'identity_mismatch'
        elif camera['status'] == 'ok' and (not camera_name or not camera.get('device_name')):
            camera['status'] = 'identity_unverified'
        result['camera'] = camera
    except Exception as error:
        result['camera'].update(status='unavailable', error_type=type(error).__name__)
    return result


def _atomic_json(path, payload):
    descriptor, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def record_firmware(cars, path, *, session_id=None, kind='experiment', asynchronous=False,
                    fetcher=fetch_status, timeout=2.0):
    """Create a pending record immediately, then replace it with bounded results.

    A pending file surviving process termination explicitly means collection did
    not finish; it must not be treated as a verified firmware inventory.
    """
    cars = [dict(car) for car in cars]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {'schema': 'leap-firmware-session-v1', 'session_id': session_id or uuid.uuid4().hex,
               'kind': kind, 'started_at': now(), 'collection_status': 'pending',
               'cars': [{'car': text(car.get('name')), 'chassis': {'status': 'pending'},
                         'camera': {'status': 'pending'}} for car in cars]}
    # Replacing a previous session record is a caller error, never silently merge.
    with path.open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write('\n')

    def collect():
        try:
            with ThreadPoolExecutor(max_workers=4) as pool:
                futures = [pool.submit(collect_car, car, fetcher=fetcher, timeout=timeout) for car in cars]
                payload['cars'] = [future.result() for future in futures]
            payload.update(collection_status='complete', completed_at=now())
        except Exception as error:
            payload.update(collection_status='failed', error_type=type(error).__name__,
                           completed_at=now())
        _atomic_json(path, payload)

    if asynchronous:
        thread = threading.Thread(target=collect, name='firmware-session', daemon=True)
        thread.start()
        return path, thread
    collect()
    return path, None


def configured_entries(config, names=None):
    payload = json.loads(Path(config).read_text(encoding='utf-8'))
    entries = payload.get('cars') if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        raise ValueError('Configuration requires cars list')
    selected = set(names) if names is not None else None
    return [car for car in entries if isinstance(car, dict)
            and (selected is None or car.get('name') in selected)]


class SessionLogger:
    """One console process directory; each motion job gets its own snapshot."""
    def __init__(self, config, *, log_dir=None, kind='console'):
        self.entries = configured_entries(config)
        root = Path(log_dir) if log_dir else Path(config).resolve().parent.parent / 'log'
        root.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix=f'{kind}_{datetime.now():%Y%m%d_%H%M%S}_', dir=root))
        self.threads = []

    def record(self, names=None, *, session_id=None, kind='console'):
        token = session_id or uuid.uuid4().hex
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', token):
            raise ValueError('Invalid session ID')
        selected = set(names) if names is not None else None
        cars = [car for car in self.entries if selected is None or car.get('name') in selected]
        path, thread = record_firmware(cars, self.directory / f'{token}_firmware.json',
                                      session_id=token, kind=kind, asynchronous=True)
        self.threads = [entry for entry in self.threads if entry.is_alive()]
        self.threads.append(thread)
        return str(path)

    def close(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        for thread in self.threads:
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
