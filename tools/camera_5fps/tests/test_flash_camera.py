"""End-to-end CLI safety tests with synthetic flash and a fake esptool process.

No test here opens a serial port. subprocess.run, dependency discovery and the
serial module are replaced before calling the application entry point.
"""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import flash_camera as flasher
from fake_esptool import FakeEsptool
from synthetic_flash import FLASH_SIZE, OTA_DATA_OFFSET, OTA_OFFSETS, OTA_SIZE, TEST_MAC, app_image, flash_image


class FlashCLITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="camera tests 中文 ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.device = FakeEsptool()
        self.original = bytes(self.device.flash)
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(flasher, "check_dependencies"))
        self.stack.enter_context(patch.object(flasher.subprocess, "run", self.device))
        # Even an unexpected import inside a reset helper can only get this mock.
        self.serial = types.SimpleNamespace(Serial=Mock(), SerialException=OSError)
        self.stack.enter_context(patch.dict(sys.modules, {"serial": self.serial}))
        self.output = io.StringIO()
        self.stack.enter_context(redirect_stdout(self.output))
        self.stack.enter_context(redirect_stderr(self.output))

    def invoke_flash(self, *, answer="FLASH", mac=TEST_MAC):
        with patch("builtins.input", return_value=answer) as prompt:
            code = flasher.main([
                "flash", "--port", "FAKE-PORT-DO-NOT-OPEN", "--car", "car20",
                "--expect-mac", mac, "--backup-root", str(self.root / "backups"),
            ])
        return code, prompt

    def assert_failed_without_write(self, code):
        self.assertNotEqual(code, 0, self.output.getvalue())
        self.assertEqual(self.device.writes, [])
        self.assertEqual(bytes(self.device.flash), self.original)

    def result_files(self):
        return list(self.root.rglob("result.json"))

    def prepare_backup(self):
        self.invoke_flash(answer="no")
        manifest = list(self.root.rglob("backup.json"))
        self.assertEqual(len(manifest), 1)
        self.device.commands.clear()
        return manifest[0].parent

    def invoke_restore(self, directory, *, answer="RESTORE"):
        with patch("builtins.input", return_value=answer) as prompt:
            code = flasher.main([
                "restore", "--port", "FAKE-PORT-DO-NOT-OPEN", "--backup", str(directory),
            ])
        return code, prompt

    def test_flash_success_uses_only_selected_ota_zero(self):
        code, prompt = self.invoke_flash()
        self.assertEqual(code, 0, self.output.getvalue())
        prompt.assert_called_once()
        self.assertEqual(len(self.device.writes), 1)
        write = self.device.writes[0]
        self.assertEqual(int(write[-2], 0), OTA_OFFSETS[0])
        self.assertEqual(bytes(self.device.flash[:OTA_OFFSETS[0]]),
                         self.original[:OTA_OFFSETS[0]])
        self.assertTrue(any("verify_flash" in command for command in self.device.commands))
        self.assertTrue(self.device.resets)
        self.assertTrue(self.result_files())
        result = json.loads(self.result_files()[0].read_text(encoding="utf-8"))
        self.assertTrue(result["application_verified"])
        self.assertTrue(result["protected_regions_verified"])
        self.assertTrue(result["reset_lines_released"])
        for name in ("--flash_mode", "--flash_freq", "--flash_size"):
            self.assertEqual(write[write.index(name) + 1], "keep")

    def test_flash_success_uses_selected_ota_one(self):
        self.device.flash = bytearray(flash_image(first=(1, 2), second=(2, 2)))
        before = bytes(self.device.flash)
        code, _ = self.invoke_flash()
        self.assertEqual(code, 0, self.output.getvalue())
        self.assertEqual(int(self.device.writes[0][-2], 0), OTA_OFFSETS[1])
        self.assertEqual(bytes(self.device.flash[:OTA_OFFSETS[1]]), before[:OTA_OFFSETS[1]])

    def test_mac_mismatch_does_not_write_or_ask_to_flash(self):
        code, prompt = self.invoke_flash(mac="AA:BB:CC:11:22:44")
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_changed_mac_in_next_command_does_not_write(self):
        self.device.wrong_mac_after_security = True
        code, _ = self.invoke_flash()
        self.assert_failed_without_write(code)

    def test_declined_confirmation_never_writes(self):
        code, prompt = self.invoke_flash(answer="no")
        self.assert_failed_without_write(code)
        prompt.assert_called_once()

    def test_partial_backup_failure_never_writes(self):
        self.device.read_error = True
        code, prompt = self.invoke_flash()
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_short_backup_with_success_exit_status_never_writes(self):
        self.device.short_read = True
        code, prompt = self.invoke_flash()
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_complete_backup_fsync_failure_never_writes_or_asks_for_confirmation(self):
        real_fsync = os.fsync
        failed_descriptors = []

        def fail_complete_backup(descriptor):
            if os.fstat(descriptor).st_size == FLASH_SIZE:
                failed_descriptors.append(descriptor)
                raise OSError("Synthetic complete-backup fsync failure")
            return real_fsync(descriptor)

        with patch.object(flasher.os, "fsync", side_effect=fail_complete_backup):
            code, prompt = self.invoke_flash()
        self.assertEqual(len(failed_descriptors), 1, "Must sync the fully written backup")
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_pending_ota_selection_never_writes(self):
        self.device.flash = bytearray(flash_image(second=(2, 1)))
        self.original = bytes(self.device.flash)
        code, prompt = self.invoke_flash()
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_chassis_project_never_writes(self):
        self.device.flash = bytearray(flash_image(project="leap_ros_mcu_driver"))
        self.original = bytes(self.device.flash)
        code, prompt = self.invoke_flash()
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_protected_region_corruption_is_reported_and_never_resets(self):
        self.device.corrupt_protected_after_write = True
        code, _ = self.invoke_flash()
        self.assertNotEqual(code, 0, self.output.getvalue())
        self.assertEqual(len(self.device.writes), 1)
        self.assertEqual(self.device.resets, [])
        self.assertTrue(self.result_files(), "Failure should retain backup and a report")

    def test_verify_failure_never_resets(self):
        self.device.verify_error = True
        code, _ = self.invoke_flash()
        self.assertNotEqual(code, 0, self.output.getvalue())
        self.assertEqual(len(self.device.writes), 1)
        self.assertEqual(self.device.resets, [])

    def test_interrupted_write_never_resets_or_attempts_another_write(self):
        self.device.write_error = True
        code, _ = self.invoke_flash()
        self.assertNotEqual(code, 0, self.output.getvalue())
        self.assertEqual(len(self.device.writes), 1)
        self.assertEqual(self.device.resets, [])
        result = json.loads(self.result_files()[0].read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["write_started"])
        self.assertFalse(result["application_verified"])

    def test_tampered_bundle_never_contacts_a_device(self):
        data = bytearray(flasher.FIRMWARE_PATH.read_bytes())
        data[-1] ^= 1
        path = self.root / "tampered app.bin"
        path.write_bytes(data)
        with patch.object(flasher, "FIRMWARE_PATH", path):
            code, prompt = self.invoke_flash()
        self.assert_failed_without_write(code)
        prompt.assert_not_called()
        self.assertEqual(self.device.commands, [])

    def test_oversized_valid_app_never_writes(self):
        oversized = app_image(payload=b"\0" * OTA_SIZE)
        path = self.root / "oversized app.bin"
        path.write_bytes(oversized)
        # Allow a syntactically valid future bundle through its fixed identity
        # gate, to exercise the independent partition-size guard.
        with patch.object(flasher, "FIRMWARE_PATH", path), \
             patch.object(flasher, "FIRMWARE_SIZE", len(oversized)), \
             patch.object(flasher, "FIRMWARE_SHA256", hashlib.sha256(oversized).hexdigest()):
            code, prompt = self.invoke_flash()
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_restore_preserves_current_nvs_and_restores_only_original_active_slot(self):
        backup = self.prepare_backup()
        # A later Wi-Fi/NVS change is kept; restore must not replay old settings.
        self.device.flash[0x9000] = 0x42
        current = bytes(self.device.flash)
        self.device.flash[OTA_OFFSETS[0]:OTA_OFFSETS[0] + 100] = b"\0" * 100
        code, prompt = self.invoke_restore(backup)
        self.assertEqual(code, 0, self.output.getvalue())
        prompt.assert_called_once()
        self.assertEqual(len(self.device.writes), 1)
        self.assertEqual(int(self.device.writes[0][-2], 0), OTA_OFFSETS[0])
        self.assertEqual(bytes(self.device.flash[:OTA_OFFSETS[0]]), current[:OTA_OFFSETS[0]])
        self.assertEqual(bytes(self.device.flash[OTA_OFFSETS[0]:]), self.original[OTA_OFFSETS[0]:])

    def test_restore_requires_exact_confirmation(self):
        backup = self.prepare_backup()
        code, prompt = self.invoke_restore(backup, answer="FLASH")
        self.assert_failed_without_write(code)
        prompt.assert_called_once()

    def test_restore_verification_failure_points_to_original_backup_and_never_resets(self):
        backup = self.prepare_backup()
        self.output.seek(0)
        self.output.truncate(0)
        self.device.verify_error = True
        code, _ = self.invoke_restore(backup)
        self.assertNotEqual(code, 0, self.output.getvalue())
        self.assertEqual(len(self.device.writes), 1)
        self.assertEqual(self.device.resets, [])
        recovery_lines = [line for line in self.output.getvalue().splitlines()
                          if "For app-only recovery" in line]
        self.assertEqual(len(recovery_lines), 1)
        self.assertTrue(recovery_lines[0].endswith(str(backup)), recovery_lines[0])
        self.assertNotIn("restore_runs", recovery_lines[0])

    def test_restore_rejects_different_mac(self):
        backup = self.prepare_backup()
        self.device.security = self.device.security.replace("aa:bb:cc:11:22:33", "aa:bb:cc:11:22:44")
        code, prompt = self.invoke_restore(backup)
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_restore_rejects_corrupt_backup_before_contacting_a_device(self):
        backup = self.prepare_backup()
        path = backup / "original_flash_4mb.bin"
        data = bytearray(path.read_bytes())
        data[0] ^= 1
        path.write_bytes(data)
        code, prompt = self.invoke_restore(backup)
        self.assert_failed_without_write(code)
        prompt.assert_not_called()
        self.assertEqual(self.device.commands, [])

    def test_restore_rejects_changed_ota_metadata(self):
        backup = self.prepare_backup()
        changed = flash_image(first=(3, 2))
        self.device.flash[OTA_DATA_OFFSET:OTA_DATA_OFFSET + 0x2000] = \
            changed[OTA_DATA_OFFSET:OTA_DATA_OFFSET + 0x2000]
        self.original = bytes(self.device.flash)
        code, prompt = self.invoke_restore(backup)
        self.assert_failed_without_write(code)
        prompt.assert_not_called()

    def test_second_attempt_creates_distinct_backup(self):
        # Declining leaves flash unchanged but must preserve the complete backup.
        self.invoke_flash(answer="no")
        first = set(self.root.rglob("*.bin"))
        self.assertTrue(first)
        digests = {path: path.read_bytes() for path in first}
        self.invoke_flash(answer="no")
        second = set(self.root.rglob("*.bin"))
        self.assertGreater(len(second), len(first))
        for path, contents in digests.items():
            self.assertEqual(path.read_bytes(), contents)


if __name__ == "__main__":
    unittest.main()
