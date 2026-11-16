"""A subprocess replacement which models flash in memory, never opening a port."""

from pathlib import Path
import subprocess

from synthetic_flash import CHIP_OUTPUT, FLASH_ID_OUTPUT, SECURITY_OUTPUT, flash_image


class FakeEsptool:
    def __init__(self, image=None):
        self.flash = bytearray(flash_image() if image is None else image)
        self.commands = []
        self.security = SECURITY_OUTPUT
        self.flash_id = FLASH_ID_OUTPUT
        self.read_error = False
        self.short_read = False
        self.write_error = False
        self.corrupt_protected_after_write = False
        self.wrong_mac_after_security = False
        self.verify_error = False

    @property
    def writes(self):
        return [command for command in self.commands if "write_flash" in command]

    @property
    def resets(self):
        return [command for command in self.commands
                if "run" in command or "hard_reset" in command]

    def __call__(self, command, **kwargs):
        """Emulate subprocess.run; unknown commands fail loudly instead of escaping."""
        if not isinstance(command, list):
            raise AssertionError("esptool must be called with an argument list, not shell text")
        if kwargs.get("shell"):
            raise AssertionError("The flasher must not execute a shell command")
        self.commands.append(list(command))
        stdout = CHIP_OUTPUT
        code = 0
        if "get_security_info" in command:
            stdout = self.security
        elif "flash_id" in command:
            stdout = self.flash_id
        elif "read_flash" in command:
            index = command.index("read_flash")
            offset, size = [int(value, 0) for value in command[index + 1:index + 3]]
            target = Path(command[index + 3])
            if self.read_error:
                target.write_bytes(bytes(self.flash[offset:offset + min(16, size)]))
                code = 2
                stdout += "Synthetic read failure\n"
            else:
                data = bytes(self.flash[offset:offset + size])
                if self.short_read:
                    data = data[:-1]
                if self.corrupt_protected_after_write and self.writes and offset == 0:
                    data = bytes((data[0] ^ 1,)) + data[1:]
                target.write_bytes(data)
                stdout += "Read complete\n"
        elif "write_flash" in command:
            offset, source = int(command[-2], 0), Path(command[-1])
            data = source.read_bytes()
            if offset < 0 or offset + len(data) > len(self.flash):
                raise AssertionError("Attempt to write beyond synthetic flash")
            if self.write_error:
                self.flash[offset:offset + 16] = data[:16]
                code = 2
                stdout += "Synthetic interrupted write\n"
            else:
                self.flash[offset:offset + len(data)] = data
                stdout += "Hash of data verified.\n"
        elif "verify_flash" in command:
            offset, source = int(command[-2], 0), Path(command[-1])
            data = source.read_bytes()
            if self.verify_error or bytes(self.flash[offset:offset + len(data)]) != data:
                code = 2
                stdout += "Synthetic verification failure\n"
            else:
                stdout += "verify OK\n"
        elif "run" in command or "read_mac" in command:
            stdout += "Hard resetting via RTS pin...\n"
        elif "version" in command or "--version" in command:
            stdout = "esptool.py v4.13.0\n"
        else:
            raise AssertionError(f"Unrecognized fake subprocess call: {command!r}")
        if self.wrong_mac_after_security and "get_security_info" not in command:
            stdout = stdout.replace("aa:bb:cc:11:22:33", "aa:bb:cc:11:22:44")
        if code and kwargs.get("check"):
            raise subprocess.CalledProcessError(code, command, output=stdout, stderr="")
        return subprocess.CompletedProcess(command, code, stdout=stdout, stderr="")
