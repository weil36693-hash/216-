"""Small, artificial ESP images for offline tests; no device data is included."""

import binascii
import hashlib
import struct


FLASH_SIZE = 4 * 1024 * 1024
TABLE_OFFSET = 0x8000
TABLE_SIZE = 0x1000
OTA_DATA_OFFSET = 0xF000
OTA_OFFSETS = (0x20000, 0x1A0000)
OTA_SIZE = 0x180000
CAMERA_PROJECT = "gc0308_http_stream"
TEST_MAC = "AA:BB:CC:11:22:33"
CHIP_OUTPUT = """esptool.py v4.13.0
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
USB mode: USB-Serial/JTAG
MAC: aa:bb:cc:11:22:33
"""
SECURITY_OUTPUT = CHIP_OUTPUT + """
Security Information:
=====================
Flags: 0x00000000 (0b0)
Chip ID: 9
API Version: 0
Secure Boot: Disabled
Flash Encryption: Disabled
SPI Boot Crypt Count (SPI_BOOT_CRYPT_CNT): 0x0
Staying in bootloader.
"""
FLASH_ID_OUTPUT = CHIP_OUTPUT + """Manufacturer: 46
Device: 4016
Detected flash size: 4MB
Flash type set in eFuse: quad (4 data lines)
Flash voltage set by eFuse to 3.3V
Staying in bootloader.
"""

# ESP-IDF partition entries: type, subtype, offset, size, label, flags.
PARTITIONS = (
    (1, 2, 0x9000, 0x6000, "nvs", 0),
    (1, 0, 0xF000, 0x2000, "otadata", 0),
    (1, 1, 0x11000, 0x1000, "phy_init", 0),
    (0, 0x10, 0x20000, 0x180000, "ota_0", 0),
    (0, 0x11, 0x1A0000, 0x180000, "ota_1", 0),
)


def partition_table(entries=PARTITIONS):
    """Build a complete partition-table sector with an independent MD5 entry."""
    data = b"".join(
        struct.pack("<HBBII16sI", 0x50AA, kind, subtype, offset, size,
                    label.encode("ascii"), flags)
        for kind, subtype, offset, size, label, flags in entries
    )
    data += b"\xeb\xeb" + b"\xff" * 14 + hashlib.md5(data).digest()
    return data.ljust(TABLE_SIZE, b"\xff")


def ota_entry(sequence, state=2):
    """ESP-IDF esp_ota_select_entry_t, including its sequence-only CRC32."""
    crc = binascii.crc32(struct.pack("<I", sequence), 0xFFFFFFFF) & 0xFFFFFFFF
    return struct.pack("<I20sII", sequence, b"\xff" * 20, state, crc)


def app_image(project=CAMERA_PROJECT, *, chip=9, version="offline-test", payload=b""):
    """Build one-segment ESP32-S3 image with app descriptor, XOR and SHA256."""
    descriptor = bytearray(256)
    struct.pack_into("<I", descriptor, 0, 0xABCD5432)
    descriptor[16:48] = version.encode("ascii")[:31].ljust(32, b"\0")
    descriptor[48:80] = project.encode("ascii")[:31].ljust(32, b"\0")
    descriptor[112:144] = b"v5.5.3-test".ljust(32, b"\0")
    segment = bytes(descriptor) + payload
    segment += b"\0" * (-len(segment) % 4)
    header = bytearray(24)
    header[:4] = bytes((0xE9, 1, 2, 0x2F))
    struct.pack_into("<I", header, 4, 0x40374000)
    struct.pack_into("<H", header, 12, chip)
    header[23] = 1
    data = bytes(header) + struct.pack("<II", 0x3C000020, len(segment)) + segment
    checksum = 0xEF
    for value in segment:
        checksum ^= value
    data += b"\0" * ((15 - len(data)) % 16) + bytes((checksum,))
    return data + hashlib.sha256(data).digest()


def flash_image(*, first=(1, 2), second=None, project=CAMERA_PROJECT,
                entries=PARTITIONS):
    """Create an erased artificial 4 MiB flash and install both fake OTA apps."""
    data = bytearray(b"\xff" * FLASH_SIZE)
    data[TABLE_OFFSET:TABLE_OFFSET + TABLE_SIZE] = partition_table(entries)
    for index, record in enumerate((first, second)):
        if record is not None:
            offset = OTA_DATA_OFFSET + index * 0x1000
            data[offset:offset + 32] = ota_entry(*record)
    for slot, offset in enumerate(OTA_OFFSETS):
        app = app_image(project, version=f"synthetic-slot-{slot}")
        data[offset:offset + len(app)] = app
    return bytes(data)


def replace_bytes(data, offset, replacement):
    if offset < 0:
        offset += len(data)
    return data[:offset] + replacement + data[offset + len(replacement):]


def flip_byte(data, offset):
    return replace_bytes(data, offset, bytes((data[offset] ^ 0x01,)))
