"""Vehicle routing; extra fields in the existing fleet.json are left untouched."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re


@dataclass(frozen=True)
class CarConfig:
    name: str
    domain_id: int
    cmd_vel_topic: str = '/cmd_vel'


def default_config():
    override = os.environ.get('LEAP_FLEET_CONFIG')
    if override:
        return Path(override).expanduser()
    workspace_config = Path(__file__).resolve().parents[3] / 'config' / 'fleet.json'
    if workspace_config.is_file():
        return workspace_config
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('leap_fleet')) / 'config' / 'fleet.json'


def load_cars(config=None, cars=None):
    """Return selected CarConfig entries without importing or initializing ROS.

    ``config`` is a JSON path, defaulting to the workspace fleet.json (or its
    installed copy). ``LEAP_FLEET_CONFIG`` overrides that default. IP and mocap
    fields are deliberately not required by this velocity-only interface.
    """
    path = default_config() if config is None else Path(config).expanduser()
    payload = json.loads(path.read_text(encoding='utf-8'))
    entries = payload.get('cars') if isinstance(payload, dict) else None
    if not isinstance(entries, list) or not entries:
        raise ValueError('config must contain a nonempty cars list')
    result = []
    names, domains = set(), set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError('each car must be an object')
        name, domain = entry.get('name'), entry.get('domain_id')
        topic = entry.get('cmd_vel_topic', '/cmd_vel')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('car name must start with a letter and contain letters/digits/_')
        if type(domain) is not int or not 0 <= domain <= 99:
            raise ValueError(f'{name}: domain_id must be an integer in 0..99')
        if name in names or domain in domains:
            raise ValueError('car names and domain_id values must be unique')
        if (not isinstance(topic, str) or
                not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', topic)):
            raise ValueError(f'{name}: cmd_vel_topic must be an absolute ROS topic')
        names.add(name)
        domains.add(domain)
        result.append(CarConfig(name, domain, topic))
    if cars is None:
        return tuple(result)
    requested = [cars] if isinstance(cars, str) else list(cars)
    if not requested or any(not isinstance(name, str) for name in requested):
        raise ValueError('cars must select at least one vehicle by name')
    if len(requested) != len(set(requested)):
        raise ValueError('cars selection contains duplicates')
    missing = set(requested) - names
    if missing:
        raise KeyError(f'unknown cars: {sorted(missing)}')
    return tuple(car for car in result if car.name in requested)
