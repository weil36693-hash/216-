"""Own subprocess groups and stop only the processes started by this invocation."""
import os
import signal
import subprocess
import time


class Processes:
    def __init__(self):
        self.children = []

    def start(self, command, log, env=None, *, pass_fds=()):
        with open(log, 'w', encoding='utf-8') as output:
            child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                     env=env, start_new_session=True, pass_fds=pass_fds)
        self.children.append(child)
        return child

    def close(self):
        # A launch parent can exit before its children; address the whole group.
        for sig, timeout in [(signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 1)]:
            for child in self.children:
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                alive = False
                for child in self.children:
                    child.poll()
                    try:
                        os.killpg(child.pid, 0)
                        alive = True
                    except ProcessLookupError:
                        pass
                if not alive:
                    break
                time.sleep(0.1)
        for child in self.children:
            child.wait(timeout=1)
