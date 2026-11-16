"""Offline validation of partition, OTA selection and ESP app corruption checks."""

import hashlib
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import camera_layout as layout
from synthetic_flash import (
    CAMERA_PROJECT, FLASH_ID_OUTPUT, OTA_DATA_OFFSET, OTA_OFFSETS, OTA_SIZE, PARTITIONS,
    SECURITY_OUTPUT, TEST_MAC,
    TABLE_OFFSET, TABLE_SIZE, app_image, flash_image, flip_byte, ota_entry,
    partition_table, replace_bytes,
)


class PartitionTableTests(unittest.TestCase):
    def test_valid_table_contains_known_camera_offsets(self):
        entries = layout.parse_partition_table(partition_table())
        self.assertEqual([entry["label"] for entry in entries],
                         ["nvs", "otadata", "phy_init", "ota_0", "ota_1"])
        self.assertEqual(entries[3]["offset"], 0x20000)
        self.assertEqual(entries[4]["offset"], 0x1A0000)
        self.assertEqual(entries[4]["size"], 0x180000)

    def test_corrupt_partition_digest_is_rejected(self):
        table = flip_byte(partition_table(), len(PARTITIONS) * 32 + 16)
        with self.assertRaises(layout.LayoutError):
            layout.parse_partition_table(table)

    def test_entry_changed_without_new_digest_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.parse_partition_table(flip_byte(partition_table(), 4))

    def test_missing_partition_digest_is_rejected(self):
        table = replace_bytes(partition_table(), len(PARTITIONS) * 32, b"\xff" * 32)
        with self.assertRaises(layout.LayoutError):
            layout.parse_partition_table(table)

    def test_different_layout_with_valid_digest_is_rejected_by_inspection(self):
        entries = list(PARTITIONS)
        entries[3] = (0, 0x10, 0x30000, OTA_SIZE, "ota_0", 0)
        with self.assertRaises(layout.LayoutError):
            layout.inspect_flash(flash_image(entries=entries))

    def test_incomplete_flash_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.inspect_flash(flash_image()[:-1])


class OTASelectionTests(unittest.TestCase):
    def select(self, *, first=(1, 2), second=None, data=None):
        if data is None:
            data = flash_image(first=first, second=second)
        return layout.select_active_partition(
            data[OTA_DATA_OFFSET:OTA_DATA_OFFSET + 0x2000],
            layout.parse_partition_table(data[TABLE_OFFSET:TABLE_OFFSET + TABLE_SIZE]),
        )

    def test_first_valid_copy_selects_ota_zero(self):
        result = self.select()
        self.assertEqual(result["partition"]["label"], "ota_0")
        self.assertEqual(result["sequence"], 1)

    def test_newest_valid_copy_selects_ota_one(self):
        result = self.select(first=(1, 2), second=(2, 2))
        self.assertEqual(result["partition"]["label"], "ota_1")
        self.assertEqual(result["sequence"], 2)

    def test_rollover_to_ota_zero_by_sequence(self):
        result = self.select(first=(3, 2), second=(2, 2))
        self.assertEqual(result["partition"]["label"], "ota_0")
        self.assertEqual(result["sequence"], 3)

    def test_undefined_state_is_supported_for_no_rollback_firmware(self):
        self.assertEqual(self.select(first=(1, 0xFFFFFFFF))["partition"]["label"], "ota_0")

    def test_bad_crc_in_only_populated_entry_is_rejected(self):
        data = flip_byte(flash_image(), OTA_DATA_OFFSET + 28)
        with self.assertRaises(layout.LayoutError):
            self.select(data=data)

    def test_bad_crc_in_newer_entry_never_selects_it(self):
        data = flip_byte(flash_image(second=(2, 2)), OTA_DATA_OFFSET + 0x1000 + 28)
        try:
            selected = self.select(data=data)
        except layout.LayoutError:
            return  # Conservative rejection is also safe for a flashing utility.
        self.assertEqual(selected["partition"]["label"], "ota_0")

    def test_pending_app_is_rejected_even_when_older_valid_copy_exists(self):
        with self.assertRaises(layout.LayoutError):
            self.select(first=(1, 2), second=(2, 1))

    def test_new_unconfirmed_app_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            self.select(first=(1, 2), second=(2, 0))

    def test_erased_ota_data_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            self.select(first=None, second=None)

    def test_equal_sequences_are_rejected_as_ambiguous(self):
        with self.assertRaises(layout.LayoutError):
            self.select(first=(1, 2), second=(1, 2))

    def test_invalid_and_aborted_entries_cannot_select_an_app(self):
        for state in (3, 4):
            with self.subTest(state=state), self.assertRaises(layout.LayoutError):
                self.select(first=(1, state))


class AppImageTests(unittest.TestCase):
    def test_valid_app_descriptor_and_size(self):
        data = app_image()
        parsed = layout.parse_app_image(data)
        self.assertEqual(parsed["project_name"], CAMERA_PROJECT)
        self.assertEqual(parsed["chip_id"], 9)
        self.assertEqual(parsed["image_size"], len(data))
        self.assertEqual(parsed["sha256"], hashlib.sha256(data).hexdigest())

    def test_erased_partition_padding_is_accepted(self):
        data = app_image()
        parsed = layout.parse_app_image(data.ljust(OTA_SIZE, b"\xff"))
        self.assertEqual(parsed["image_size"], len(data))

    def test_corrupt_payload_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.parse_app_image(flip_byte(app_image(), 32 + 160))

    def test_corrupt_xor_is_rejected_even_if_sha_is_recomputed(self):
        data = app_image()
        body = flip_byte(data[:-32], len(data) - 33)
        corrupted = body + hashlib.sha256(body).digest()
        with self.assertRaises(layout.LayoutError):
            layout.parse_app_image(corrupted)

    def test_corrupt_sha_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.parse_app_image(flip_byte(app_image(), -1))

    def test_truncated_app_is_rejected(self):
        for length in (0, 23, 30, 100, len(app_image()) - 1):
            with self.subTest(length=length), self.assertRaises(layout.LayoutError):
                layout.parse_app_image(app_image()[:length])

    def test_segment_length_cannot_escape_input(self):
        data = replace_bytes(app_image(), 28, struct.pack("<I", 0xFFFFFFFC))
        with self.assertRaises(layout.LayoutError):
            layout.parse_app_image(data)

    def test_non_erased_app_trailer_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.parse_app_image(app_image() + b"unexpected trailer")

    def test_chassis_firmware_is_rejected_when_inspecting_flash(self):
        with self.assertRaises(layout.LayoutError):
            layout.inspect_flash(flash_image(project="leap_ros_mcu_driver"))

    def test_non_s3_image_is_rejected_when_inspecting_flash(self):
        data = replace_bytes(flash_image(), OTA_OFFSETS[0], app_image(chip=0))
        with self.assertRaises(layout.LayoutError):
            layout.inspect_flash(data)

    def test_inspection_reads_selected_ota_slot(self):
        data = flash_image(first=(1, 2), second=(2, 2))
        result = layout.inspect_flash(data)
        self.assertEqual(result["ota"]["partition"]["offset"], OTA_OFFSETS[1])
        self.assertEqual(result["app"]["version"], "synthetic-slot-1")
        self.assertEqual(result["flash_sha256"], hashlib.sha256(data).hexdigest())


class MacAddressTests(unittest.TestCase):
    def test_normalizes_address_case(self):
        self.assertEqual(layout.normalize_mac("aa:bb:cc:11:22:33").lower(),
                         "aa:bb:cc:11:22:33")

    def test_rejects_invalid_addresses(self):
        for value in ("", "aa:bb:cc:11:22", "gg:bb:cc:11:22:33", "aa:bb:cc:11:22:333"):
            with self.subTest(value=value), self.assertRaises(layout.LayoutError):
                layout.normalize_mac(value)


class DeviceIdentityTests(unittest.TestCase):
    def test_known_supported_hardware(self):
        result = layout.validate_security_info(SECURITY_OUTPUT, TEST_MAC)
        self.assertEqual(result["chip_id"], 9)
        self.assertEqual(result["psram_mb"], 8)
        self.assertEqual(result["mac"].lower(), TEST_MAC.lower())
        self.assertEqual(layout.validate_flash_id(FLASH_ID_OUTPUT, TEST_MAC), 4 * 1024 * 1024)

    def test_wrong_mac_fails_both_identity_commands(self):
        for function, output in (
            (layout.validate_security_info, SECURITY_OUTPUT),
            (layout.validate_flash_id, FLASH_ID_OUTPUT),
        ):
            with self.subTest(function=function.__name__), self.assertRaises(layout.LayoutError):
                function(output, "AA:BB:CC:11:22:44")

    def test_missing_mac_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.validate_security_info(
                SECURITY_OUTPUT.replace("MAC: aa:bb:cc:11:22:33\n", ""), TEST_MAC)

    def test_security_or_hardware_mismatch_is_rejected(self):
        replacements = (
            ("Secure Boot: Disabled", "Secure Boot: Enabled"),
            ("Flash Encryption: Disabled", "Flash Encryption: Enabled"),
            ("Chip ID: 9", "Chip ID: 0"),
            ("Chip is ESP32-S3", "Chip is ESP32"),
            ("Embedded PSRAM 8MB", "Embedded PSRAM 2MB"),
            ("Secure Boot: Disabled", ""),
        )
        for old, new in replacements:
            with self.subTest(replacement=new), self.assertRaises(layout.LayoutError):
                layout.validate_security_info(SECURITY_OUTPUT.replace(old, new), TEST_MAC)

    def test_flash_size_mismatch_is_rejected(self):
        with self.assertRaises(layout.LayoutError):
            layout.validate_flash_id(FLASH_ID_OUTPUT.replace("4MB", "8MB"), TEST_MAC)


if __name__ == "__main__":
    unittest.main()
