"""Chassis factory-app installation and recovery with an in-memory device."""
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_layout as layout
import flash_firmware as flasher
from fake_esptool import FakeEsptool
from synthetic_flash import app_image, partition_table, TEST_MAC, flash_image


def chassis_image(project="leap_ros_mcu_driver"):
    entries = tuple((p["type"], p["subtype"], p["offset"], p["size"], p["label"], p["flags"])
                    for p in layout.CHASSIS_PARTITIONS)
    result = bytearray(b"\xff" * 0x400000)
    result[0x8000:0x9000] = partition_table(entries)
    application = app_image(project=project)
    result[0x10000:0x10000 + len(application)] = application
    result[0x210000:0x21000c] = b"storage-data"
    return bytes(result)


class ChassisFlashTests(unittest.TestCase):
    def setUp(self):
        flasher.configure_device("chassis")
        self.addCleanup(flasher.configure_device, "camera")
        temporary = tempfile.TemporaryDirectory(prefix="chassis 中文 ")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.fake = FakeEsptool(chassis_image())
        # The chassis does not use external PSRAM.
        self.fake.security = self.fake.security.replace(", Embedded PSRAM 8MB (AP_3v3)", "")
        self.before = bytes(self.fake.flash)
        self.output = io.StringIO()
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(flasher, "check_dependencies"))
        stack.enter_context(patch.object(flasher.subprocess, "run", self.fake))
        stack.enter_context(patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=Mock())}))
        stack.enter_context(redirect_stdout(self.output))
        stack.enter_context(redirect_stderr(self.output))

    def invoke(self, answer="FLASH", mac=TEST_MAC):
        with patch("builtins.input", return_value=answer):
            return flasher.main(["--device", "chassis", "flash", "--car", "car20",
                                 "--port", "FAKE-PORT", "--expect-mac", mac,
                                 "--backup-root", str(self.directory)])

    def backup(self):
        return next(self.directory.rglob("backup.json")).parent

    def test_factory_app_written_without_touching_nvs_or_storage(self):
        self.assertEqual(self.invoke(), 0, self.output.getvalue())
        self.assertEqual(len(self.fake.writes), 1)
        self.assertEqual(int(self.fake.writes[0][-2], 0), 0x10000)
        self.assertEqual(bytes(self.fake.flash[:0x10000]), self.before[:0x10000])
        self.assertEqual(bytes(self.fake.flash[0x210000:]), self.before[0x210000:])
        saved = json.loads((self.backup() / "backup.json").read_text())
        self.assertEqual(saved["device"], "chassis")
        self.assertEqual(saved["active_partition"]["label"], "factory")

    def test_camera_layout_cannot_be_written_as_chassis(self):
        self.fake.flash = bytearray(flash_image())
        self.assertNotEqual(self.invoke(), 0)
        self.assertEqual(self.fake.writes, [])

    def test_wrong_project_on_factory_layout_rejected(self):
        self.fake.flash = bytearray(chassis_image("gc0308_http_stream"))
        self.assertNotEqual(self.invoke(), 0)
        self.assertEqual(self.fake.writes, [])

    def test_wrong_mac_rejected(self):
        self.assertNotEqual(self.invoke(mac="AA:BB:CC:11:22:44"), 0)
        self.assertEqual(self.fake.writes, [])

    def test_restore_after_failed_app_preserves_current_nvs(self):
        self.assertNotEqual(self.invoke(answer="no"), 0)
        backup = self.backup()
        self.fake.flash[0x10000:0x10010] = b"\0" * 16
        self.fake.flash[0x9000:0x9004] = b"NVS2"
        with patch("builtins.input", return_value="RESTORE"):
            code = flasher.main(["--device", "chassis", "restore", "--port", "FAKE-PORT",
                                 "--backup", str(backup)])
        self.assertEqual(code, 0, self.output.getvalue())
        self.assertEqual(bytes(self.fake.flash[0x10000:0x210000]), self.before[0x10000:0x210000])
        self.assertEqual(bytes(self.fake.flash[0x9000:0x9004]), b"NVS2")
        self.assertEqual(bytes(self.fake.flash[0x210000:]), self.before[0x210000:])

    def test_backup_cannot_be_restored_as_other_device_type(self):
        self.invoke(answer="no")
        with patch("builtins.input", return_value="RESTORE"):
            code = flasher.main(["--device", "camera", "restore", "--port", "FAKE-PORT",
                                 "--backup", str(self.backup())])
        self.assertNotEqual(code, 0)
        self.assertEqual(self.fake.writes, [])

    def test_profile_switch_does_not_retain_chassis_offsets(self):
        self.assertEqual(layout.PROTECTED_SIZE, 0x10000)
        flasher.configure_device("camera")
        self.assertEqual(layout.PROTECTED_SIZE, 0x20000)
        self.assertEqual(layout.PROJECT_NAME, "gc0308_http_stream")


if __name__ == "__main__":
    unittest.main()
