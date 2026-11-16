#!/usr/bin/env python3
"""Install/configure the main PC as the camera LAN's NTP server (Ubuntu/Debian).

Without --apply this only prints the intended configuration and reads local IPs.
Run with sudo and --apply to install chrony and configure the service. Existing
upstream clock sources remain intact; no fake 'local stratum' clock is introduced.
"""
import argparse
from datetime import datetime
import ipaddress
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

SERVER_IP = '10.1.1.249'
CLIENT_NET = '10.1.1.0/24'
BEGIN = '# BEGIN LEAP CAMERA NTP'
END = '# END LEAP CAMERA NTP'
MANAGED_BLOCK = f'{BEGIN}\nallow {CLIENT_NET}\nbindaddress {SERVER_IP}\n{END}\n'


def render_config(original):
    if original.count(BEGIN) != original.count(END) or original.count(BEGIN) > 1:
        raise ValueError('Invalid existing LEAP CAMERA NTP block; inspect chrony.conf')
    if BEGIN in original:
        before, rest = original.split(BEGIN, 1)
        _, after = rest.split(END, 1)
        original = before.rstrip() + '\n' + after.lstrip('\n')
    # A second IPv4 bind or disabled server port would make this configuration
    # ambiguous. Leave the operator's original configuration unchanged.
    retained = []
    for line in original.splitlines():
        parts = line.split('#', 1)[0].split()
        if parts and parts[0] == 'bindaddress' and len(parts) >= 2:
            try:
                is_ipv4 = isinstance(ipaddress.ip_address(parts[1]), ipaddress.IPv4Address)
            except ValueError:
                is_ipv4 = False
            if is_ipv4:
                if parts[1] != SERVER_IP:
                    raise ValueError('Existing IPv4 bindaddress differs; reconcile it before applying')
                continue
        if parts and parts[0] == 'port' and len(parts) >= 2 and parts[1] != '123':
            raise ValueError('Existing NTP server port differs from 123')
        retained.append(line)
    return '\n'.join(retained).rstrip() + '\n\n' + MANAGED_BLOCK


def addresses():
    result = subprocess.run(['ip', '-j', '-4', 'addr', 'show'], check=True,
                            capture_output=True, text=True)
    payload = json.loads(result.stdout)
    return {entry['local'] for interface in payload for entry in interface.get('addr_info', [])
            if entry.get('family') == 'inet'}


def run(command, **kwargs):
    return subprocess.run(command, check=True, **kwargs)


def apply():
    if os.geteuid() != 0:
        raise PermissionError('Use sudo python3 configure_camera_chrony.py --apply')
    if SERVER_IP not in addresses():
        raise ValueError(f'This computer does not own {SERVER_IP}; no changes made')
    config = Path('/etc/chrony/chrony.conf')
    backup = Path('/var/backups/leap-camera-chrony') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    backup.mkdir(parents=True, mode=0o700)
    if config.exists():
        shutil.copy2(config, backup / 'chrony.conf.before-install')
    installed = subprocess.run(['dpkg-query', '-W', '-f=${Status}', 'chrony'],
                               capture_output=True, text=True, check=False)
    if installed.returncode or installed.stdout.strip() != 'install ok installed':
        run(['apt-get', 'update'])
        run(['apt-get', 'install', '-y', 'chrony'])
    original = config.read_text(encoding='utf-8')
    desired = render_config(original)
    shutil.copy2(config, backup / 'chrony.conf.before-configure')
    descriptor, candidate = tempfile.mkstemp(prefix='.leap-camera-', dir=config.parent)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(desired)
            stream.flush()
            os.fsync(stream.fileno())
        # Parse the entire include tree without launching chronyd or touching the clock.
        checked = subprocess.run(['chronyd', '-p', '-f', candidate], capture_output=True, text=True)
        if checked.returncode:
            (backup / 'validation_error.txt').write_text(checked.stderr, encoding='utf-8')
            raise RuntimeError('chronyd rejected the proposed configuration; original preserved')
        parsed = checked.stdout
        binds = [line.split()[1] for line in parsed.splitlines()
                 if line.split() and line.split()[0] == 'bindaddress' and len(line.split()) > 1]
        if sum(1 for value in binds if ':' not in value) != 1:
            raise RuntimeError('Included files contain conflicting IPv4 bindaddress settings')
        if desired != original:
            os.chmod(candidate, 0o644)
            os.replace(candidate, config)
        try:
            run(['systemctl', 'enable', '--now', 'chrony'])
            if desired != original:
                run(['systemctl', 'restart', 'chrony'])
        except subprocess.CalledProcessError:
            # Restore the prior file on service startup failure. The installation
            # itself remains available, and the private backup records the change.
            if desired != original:
                shutil.copy2(backup / 'chrony.conf.before-configure', config)
                subprocess.run(['systemctl', 'restart', 'chrony'], check=False)
            raise
        # Add one narrow rule only if UFW is already enabled; never enable it.
        if shutil.which('ufw'):
            status = run(['ufw', 'status'], capture_output=True, text=True,
                         env={**os.environ, 'LC_ALL': 'C'}).stdout
            if 'Status: active' in status:
                run(['ufw', 'allow', 'from', CLIENT_NET, 'to', SERVER_IP,
                     'port', '123', 'proto', 'udp', 'comment', 'LEAP camera NTP'])
        run(['systemctl', 'is-active', '--quiet', 'chrony'])
        print(f'Chrony is active; config backups: {backup}')
        print('NTP access from a representative camera:')
        run(['chronyc', 'accheck', '10.1.1.233'])
        print('Clock state (Leap status should become Normal after upstream synchronization):')
        run(['chronyc', 'tracking'])
        run(['chronyc', 'sources', '-v'])
        synchronized = subprocess.run(['chronyc', 'waitsync', '1'], capture_output=True, text=True)
        if synchronized.returncode:
            print('Configuration applied, but host synchronization is NOT yet confirmed. '
                  'Check upstream reachability before relying on camera timestamps.', file=sys.stderr)
            return 2
        print('Host chrony reports synchronized. Verify each camera time_sync.synced separately.')
        return 0
    finally:
        if os.path.exists(candidate):
            os.unlink(candidate)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Install and apply system configuration; requires sudo')
    args = parser.parse_args()
    try:
        if args.apply:
            return apply()
        print(MANAGED_BLOCK)
        print(f'{SERVER_IP} assigned locally: {SERVER_IP in addresses()}')
        print('Preview only. Apply with: sudo python3 scripts/configure_camera_chrony.py --apply')
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'Chrony setup failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
