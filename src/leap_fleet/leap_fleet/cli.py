"""Command line access to the per-vehicle velocity interface."""

import argparse
import json
import math
from pathlib import Path
import sys

from .config import load_cars


_COMMANDS = frozenset(('list', 'velocity', 'stop', 'batch'))


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError('must be a finite number') from error
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('must be a finite number')
    return number


def _positive(value):
    number = _finite(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return number


def _nonnegative(value):
    number = _finite(value)
    if number < 0:
        raise argparse.ArgumentTypeError('must be zero or greater')
    return number


def _command_first(argv):
    """Allow common options on either side of the subcommand.

    In particular, argparse would otherwise consume the subcommand as another
    vehicle in ``--cars car01 car02 velocity``. A config path is an option value,
    even when its filename happens to be a subcommand name.
    """
    skip_value = False
    for index, token in enumerate(argv):
        if skip_value:
            skip_value = False
            continue
        if token == '--config':
            skip_value = True
        elif token in _COMMANDS:
            # Keep batch's positional JSON path ahead of a moved --cars list.
            return [token, *argv[index + 1:], *argv[:index]]
    return argv


def _parser():
    parser = argparse.ArgumentParser(
        prog='drive',
        description='Select vehicles and publish bounded-duration velocity commands.')
    subparsers = parser.add_subparsers(dest='command', required=True)
    for command, description in (
            ('list', 'List configured vehicles without connecting to ROS.'),
            ('velocity', 'Apply the same velocity to the selected vehicles.'),
            ('stop', 'Stop the selected vehicles.'),
            ('batch', 'Apply velocities from a JSON file.')):
        child = subparsers.add_parser(command, help=description)
        child.add_argument('--config', type=Path, help='Fleet JSON configuration path.')
        child.add_argument(
            '--cars', nargs='+',
            help=('Allowed vehicle IDs; defaults to the JSON targets.' if command == 'batch'
                  else 'Vehicle IDs; defaults to all configured vehicles.'))
        if command == 'list':
            continue
        child.add_argument('--wait', type=_nonnegative, default=5.0,
                           help='Discovery timeout in seconds; 0 skips waiting (default: 5).')
        if command == 'stop':
            continue
        child.add_argument('--duration', type=_positive, required=True,
                           help='Command duration in seconds; must be finite and positive.')
        child.add_argument('--rate', type=_positive, default=20.0,
                           help='Command publication rate in Hz (default: 20).')
        if command == 'velocity':
            child.add_argument('--v', type=_finite, default=0.0,
                               help='Forward velocity in m/s (default: 0).')
            child.add_argument('--w', type=_finite, default=0.0,
                               help='Yaw velocity in rad/s (default: 0).')
        else:
            child.add_argument('commands_json', type=Path,
                               help='JSON object mapping IDs to {"v": number, "w": number}.')
    return parser


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _batch_commands(path):
    with path.open(encoding='utf-8') as source:
        contents = json.load(source, object_pairs_hook=_unique_object)
    if not isinstance(contents, dict) or not contents:
        raise ValueError('batch commands must be a nonempty JSON object')
    commands = {}
    for name, command in contents.items():
        if not isinstance(name, str) or not name:
            raise ValueError('batch vehicle IDs must be nonempty strings')
        if not isinstance(command, dict) or set(command) != {'v', 'w'}:
            raise ValueError(f'{name}: expected exactly the fields v and w')
        values = tuple(command[key] for key in ('v', 'w'))
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            raise ValueError(f'{name}: v and w must be finite numbers')
        commands[name] = values
    return commands


def main(argv=None):
    """Return an exit status; all argument validation precedes ROS startup."""
    parser = _parser()
    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        args = parser.parse_args(_command_first(arguments))
    except SystemExit as error:
        return int(error.code)

    try:
        commands = _batch_commands(args.commands_json) if args.command == 'batch' else None
        selected_names = args.cars
        if commands is not None and selected_names is None:
            selected_names = list(commands)
        configs = load_cars(config=args.config, cars=selected_names)
        names = tuple(car.name for car in configs)
        if args.command == 'list':
            print('name\tdomain_id\tcmd_vel_topic')
            for car in configs:
                print(f'{car.name}\t{car.domain_id}\t{car.cmd_vel_topic}')
            return 0
        if commands is not None:
            unselected = set(commands) - set(names)
            if unselected:
                raise ValueError(
                    'batch targets are outside --cars: ' + ', '.join(sorted(unselected)))
            # An explicit selection is an allowlist. Opening unrelated cars
            # would also stop them when the Fleet context closes.
            names = tuple(name for name in names if name in commands)

        # Import the SDK only once the complete request has passed validation.
        from .control import Fleet, Velocity

        if args.command == 'velocity':
            commands = {name: Velocity(args.v, args.w) for name in names}
        elif commands is not None:
            commands = {name: Velocity(*value) for name, value in commands.items()}

        # Start the snapshot off the command path; unavailable cameras must never
        # delay an urgent stop or the command workers' heartbeat.
        from .firmware_session import SessionLogger
        from .config import default_config
        firmware_logger = None
        try:
            firmware_logger = SessionLogger(args.config or default_config(), kind='drive')
            firmware_logger.record(names, kind='drive_' + args.command)
            print(f'会话固件日志：{firmware_logger.directory}')
        except (OSError, ValueError) as error:
            print(f'固件日志创建失败：{type(error).__name__}', file=sys.stderr)
        try:
            with Fleet(config=args.config, cars=names) as fleet:
                if args.wait > 0 and not fleet.wait_for_subscribers(timeout=args.wait):
                    raise RuntimeError('subscriber discovery timed out; no motion command was sent')
                if args.command == 'stop':
                    fleet.stop()
                else:
                    fleet.run_for(commands, duration=args.duration, rate=args.rate)
        finally:
            # All motion/stop work has completed before waiting for metadata.
            if firmware_logger is not None:
                firmware_logger.close()
        return 0
    except KeyboardInterrupt:
        print('drive: interrupted', file=sys.stderr)
        return 130
    except Exception as error:
        print(f'drive: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
