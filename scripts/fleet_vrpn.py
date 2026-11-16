"""Coordinate this host's VRPN starters without taking ownership of existing clients."""
import fcntl
from pathlib import Path
import time


class VRPNStartLock:
    def __init__(self, directory, domain_id):
        self.path = Path(directory) / f'fleet_vrpn_domain_{int(domain_id)}.lock'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = None

    def acquire(self):
        if self._file is not None:
            raise RuntimeError('VRPN start lock already acquired')
        stream = self.path.open('a')
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.close()
            return False
        except BaseException:
            stream.close()
            raise
        self._file = stream
        return True

    def fileno(self):
        return self._file.fileno()

    def close(self):
        if self._file is not None:
            # The launch process inherits this open file description. Never
            # LOCK_UN or unlink: it must retain the lock if the reader exits.
            self._file.close()
            self._file = None


def discover_vrpn(node, topics, spin_once, *, timeout=2.0, clock=time.monotonic):
    """Bounded graph discovery; graph errors abort startup rather than imply absence.

    The lock serializes cooperating starters. Discovery also catches older or
    external clients, but cannot prevent external launches after this window.
    """
    deadline = clock() + timeout
    publishers = {}
    client_found = False
    while True:
        for topic in topics:
            count = node.count_publishers(topic)
            if count:
                publishers[topic] = max(count, publishers.get(topic, 0))
        client_found |= any(
            name == 'vrpn_mocap_client_node' and namespace == '/vrpn_mocap'
            for name, namespace in node.get_node_names_and_namespaces())
        remaining = deadline - clock()
        if remaining <= 0:
            return publishers, client_found
        spin_once(node, timeout_sec=min(0.1, remaining))
