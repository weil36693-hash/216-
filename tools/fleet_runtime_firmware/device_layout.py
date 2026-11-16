"""Offline checks for the fleet's gc0308_http_stream ESP32-S3 firmware.

These checks deliberately reject unfamiliar firmware layouts instead of guessing.
No function in this module opens a serial port or modifies a file.
The historical project name does not identify the image sensor model.
"""

from __future__ import annotations

import binascii
import hashlib
import re
import struct


class LayoutError(ValueError):
    """The target cannot be positively identified as a supported camera."""


FLASH_SIZE = 0x400000
PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_SIZE = 0x1000
PROTECTED_SIZE = 0x20000
OTADATA_OFFSET = 0xF000
OTADATA_SIZE = 0x2000
PROJECT_NAME = "gc0308_http_stream"
CHIP_ID = 9
EXPECTED_PARTITIONS = (
    {"label": "nvs", "type": 1, "subtype": 2, "offset": 0x9000, "size": 0x6000, "flags": 0},
    {"label": "otadata", "type": 1, "subtype": 0, "offset": 0xF000, "size": 0x2000, "flags": 0},
    {"label": "phy_init", "type": 1, "subtype": 1, "offset": 0x11000, "size": 0x1000, "flags": 0},
    {"label": "ota_0", "type": 0, "subtype": 0x10, "offset": 0x20000, "size": 0x180000, "flags": 0},
    {"label": "ota_1", "type": 0, "subtype": 0x11, "offset": 0x1A0000, "size": 0x180000, "flags": 0},
)


CAMERA_PARTITIONS = EXPECTED_PARTITIONS
CHASSIS_PARTITIONS = (
    {"label": "nvs", "type": 1, "subtype": 2, "offset": 0x9000, "size": 0x4000, "flags": 0},
    {"label": "otadata", "type": 1, "subtype": 0, "offset": 0xD000, "size": 0x2000, "flags": 0},
    {"label": "phy_init", "type": 1, "subtype": 1, "offset": 0xF000, "size": 0x1000, "flags": 0},
    {"label": "factory", "type": 0, "subtype": 0, "offset": 0x10000, "size": 0x200000, "flags": 0},
    {"label": "storage", "type": 1, "subtype": 0x82, "offset": 0x210000, "size": 0x100000, "flags": 0},
)
DEVICE = "camera"
REQUIRE_PSRAM_8MB = True


def configure_device(device: str) -> None:
    """CLI processes handle one explicit device type at a time."""
    global DEVICE, EXPECTED_PARTITIONS, PROJECT_NAME, PROTECTED_SIZE, OTADATA_OFFSET, REQUIRE_PSRAM_8MB
    if device not in ("camera", "chassis"):
        raise LayoutError("Device must be camera or chassis.")
    DEVICE = device
    EXPECTED_PARTITIONS = CAMERA_PARTITIONS if device == "camera" else CHASSIS_PARTITIONS
    PROJECT_NAME = "gc0308_http_stream" if device == "camera" else "leap_ros_mcu_driver"
    PROTECTED_SIZE = 0x20000 if device == "camera" else 0x10000
    OTADATA_OFFSET = 0xF000 if device == "camera" else 0xD000
    REQUIRE_PSRAM_8MB = device == "camera"


def normalize_mac(value: str) -> str:
    """Accept a colon/dash separated MAC, or the 12 hex digits on the car label."""
    value = value.strip()
    if value.startswith("Maturo_CAM_"):
        value = value[len("Maturo_CAM_") :]
    elif value.startswith("Maturo_"):
        value = value[len("Maturo_") :]
    if not (re.fullmatch(r"[0-9A-Fa-f]{12}", value) or
            re.fullmatch(r"(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}", value) or
            re.fullmatch(r"(?:[0-9A-Fa-f]{2}-){5}[0-9A-Fa-f]{2}", value)):
        raise LayoutError("MAC must contain exactly 12 hexadecimal digits.")
    compact = value.replace(":", "").replace("-", "").lower()
    if compact in ("000000000000", "ffffffffffff") or int(compact[:2], 16) & 1:
        raise LayoutError("Expected a unicast device MAC, not an empty/broadcast address.")
    return ":".join(compact[n : n + 2] for n in range(0, 12, 2))


def validate_observed_mac(output: str, expected_mac: str) -> str:
    observed = re.findall(r"^MAC:\s*([0-9a-fA-F:]+)\s*$", output, re.MULTILINE)
    if not observed:
        raise LayoutError("esptool did not report a MAC; target identity is unverified.")
    expected = normalize_mac(expected_mac)
    if any(normalize_mac(value) != expected for value in observed):
        raise LayoutError("The connected chip MAC does not match --expect-mac / backup.")
    return expected


def validate_security_info(output: str, expected_mac: str) -> dict:
    mac = validate_observed_mac(output, expected_mac)
    if not re.search(r"^Chip is ESP32-S3(?:\s|\()", output, re.MULTILINE):
        raise LayoutError("Only an ESP32-S3 device is supported.")
    if REQUIRE_PSRAM_8MB and not re.search(r"^Features:.*\bEmbedded PSRAM 8MB\b", output, re.MULTILINE):
        raise LayoutError("Expected ESP32-S3 with embedded 8MB PSRAM.")
    required = {
        "Chip ID": "9",
        "Secure Boot": "Disabled",
        "Flash Encryption": "Disabled",
    }
    for key, expected in required.items():
        values = re.findall(r"^" + re.escape(key) + r":\s*([^\r\n]+)$", output, re.MULTILINE)
        if not values or any(value.strip() != expected for value in values):
            raise LayoutError(f"Unsupported or unverified security setting: {key}.")
    flags = re.findall(r"^Flags:\s*(0x[0-9a-fA-F]+)\b", output, re.MULTILINE)
    if not flags or any(int(value, 16) != 0 for value in flags):
        raise LayoutError("Unexpected security flags; refusing to write flash.")
    return {"mac": mac, "chip_id": CHIP_ID, "psram_mb_required": 8 if REQUIRE_PSRAM_8MB else None,
            "secure_boot": False, "flash_encryption": False}


def validate_flash_id(output: str, expected_mac: str) -> int:
    validate_observed_mac(output, expected_mac)
    sizes = re.findall(r"^Detected flash size:\s*([^\r\n]+)$", output, re.MULTILINE)
    if not sizes or any(value.strip() != "4MB" for value in sizes):
        raise LayoutError("Expected exactly 4MB of flash.")
    return FLASH_SIZE


def parse_partition_table(table: bytes) -> list[dict]:
    if len(table) not in (0xC00, PARTITION_TABLE_SIZE):
        raise LayoutError("Partition table must contain exactly 3072 or 4096 bytes.")
    partitions = []
    md5_found = False
    for pos in range(0, len(table), 32):
        entry = table[pos : pos + 32]
        magic = struct.unpack_from("<H", entry)[0]
        if magic == 0xEBEB:
            if entry[:16] != b"\xeb\xeb" + b"\xff" * 14:
                raise LayoutError("Malformed partition MD5 record.")
            if entry[16:] != hashlib.md5(table[:pos]).digest():
                raise LayoutError("Partition table MD5 mismatch.")
            if table[pos + 32 :] != b"\xff" * (len(table) - pos - 32):
                raise LayoutError("Unexpected data after partition table MD5.")
            md5_found = True
            break
        if magic != 0x50AA:
            raise LayoutError("Missing partition MD5 or malformed partition entry.")
        _, ptype, subtype, offset, size, label_bytes, flags = struct.unpack("<HBBII16sI", entry)
        try:
            label = label_bytes.split(b"\0", 1)[0].decode("ascii")
        except UnicodeDecodeError as exc:
            raise LayoutError("Partition label is not ASCII.") from exc
        if label_bytes != label.encode("ascii").ljust(16, b"\0"):
            raise LayoutError("Unexpected padding in partition label.")
        partitions.append({"label": label, "type": ptype, "subtype": subtype,
                           "offset": offset, "size": size, "flags": flags})
    if not md5_found:
        raise LayoutError("Partition table has no MD5 record.")
    if partitions != list(EXPECTED_PARTITIONS):
        raise LayoutError("Partition layout differs from the verified camera firmware layout.")
    return partitions


def select_active_partition(otadata: bytes, partitions: list[dict]) -> dict:
    """Select only a stable, CRC-verified OTA state; never fall back by guessing."""
    if len(otadata) != OTADATA_SIZE or partitions != list(EXPECTED_PARTITIONS):
        raise LayoutError("Unexpected OTA data size or partition layout.")
    if DEVICE == "chassis":
        # Exact layout has only one factory app and no OTA slots.
        return {"partition": next(p for p in partitions if p["label"] == "factory"),
                "entries": [], "sequence": None}
    entries = []
    candidates = []
    for index in range(2):
        raw = otadata[index * 0x1000 : index * 0x1000 + 32]
        seq, state, crc = struct.unpack_from("<I20xII", raw)
        empty = raw == b"\xff" * 32
        expected_crc = binascii.crc32(struct.pack("<I", seq), 0xFFFFFFFF) & 0xFFFFFFFF
        entry = {"index": index, "sequence": seq, "state": state, "crc": crc,
                 "expected_crc": expected_crc, "empty": empty, "valid": False}
        entries.append(entry)
        if empty:
            continue
        # Corrupt or interrupted metadata must be inspected by a person. Even an
        # older NEW/PENDING entry is rejected instead of emulating rollback.
        if seq in (0, 0xFFFFFFFF) or crc != expected_crc:
            raise LayoutError("Nonempty OTA entry has an invalid sequence or CRC.")
        if state not in (2, 0xFFFFFFFF):
            raise LayoutError("OTA state is not stable VALID/UNDEFINED; refusing to write.")
        entry["valid"] = True
        candidates.append(entry)
    if not candidates:
        raise LayoutError("No stable OTA selection; active application is ambiguous.")
    if len(candidates) == 2 and candidates[0]["sequence"] == candidates[1]["sequence"]:
        raise LayoutError("Equal OTA sequences are ambiguous; refusing to choose a slot.")
    selected = max(candidates, key=lambda entry: entry["sequence"])
    slot = (selected["sequence"] - 1) % 2
    partition = next(p for p in partitions if p["label"] == f"ota_{slot}")
    return {"partition": partition, "entries": entries, "sequence": selected["sequence"]}


def _cstring(value: bytes, name: str) -> str:
    if b"\0" not in value:
        raise LayoutError(f"Unterminated application descriptor {name}.")
    try:
        return value.split(b"\0", 1)[0].decode("utf-8")
    except UnicodeDecodeError as exc:
        raise LayoutError(f"Invalid application descriptor {name}.") from exc


def parse_app_image(data: bytes) -> dict:
    if len(data) < 24 or data[0] != 0xE9:
        raise LayoutError("Active application has no valid ESP image header.")
    segment_count = data[1]
    chip_id = struct.unpack_from("<H", data, 12)[0]
    if chip_id != CHIP_ID:
        raise LayoutError("Application image is not for ESP32-S3 (chip ID 9).")
    if not 1 <= segment_count <= 16 or data[23] != 1:
        raise LayoutError("Invalid segment count or missing appended SHA256.")
    cursor = 24
    checksum = 0xEF
    first_segment = b""
    segments = []
    for index in range(segment_count):
        if cursor + 8 > len(data):
            raise LayoutError("Truncated ESP segment header.")
        address, size = struct.unpack_from("<II", data, cursor)
        cursor += 8
        if size > len(data) - cursor or size % 4:
            raise LayoutError("Truncated or unaligned ESP segment.")
        segment = data[cursor : cursor + size]
        if index == 0:
            first_segment = segment
        for value in segment:
            checksum ^= value
        segments.append({"address": address, "size": size})
        cursor += size
    checksum_offset = cursor + (15 - cursor % 16)
    image_size = checksum_offset + 1 + 32
    if image_size > len(data):
        raise LayoutError("Truncated ESP checksum or SHA256.")
    if data[cursor:checksum_offset] != b"\0" * (checksum_offset - cursor):
        raise LayoutError("Unexpected ESP checksum padding.")
    if data[checksum_offset] != checksum:
        raise LayoutError("ESP image XOR checksum mismatch.")
    digest = hashlib.sha256(data[: checksum_offset + 1]).digest()
    if digest != data[checksum_offset + 1 : image_size]:
        raise LayoutError("ESP image appended SHA256 mismatch.")
    if data[image_size:] != b"\xff" * (len(data) - image_size):
        raise LayoutError("Unexpected non-erased data after application image.")
    if len(first_segment) < 256 or struct.unpack_from("<I", first_segment)[0] != 0xABCD5432:
        raise LayoutError("Application descriptor is missing from the first segment.")
    project_name = _cstring(first_segment[48:80], "project name")
    if project_name != PROJECT_NAME:
        raise LayoutError(f"Application project must be {PROJECT_NAME}; wrong device type or unknown firmware.")
    return {"image_size": image_size, "project_name": project_name, "chip_id": chip_id,
            "version": _cstring(first_segment[16:48], "version"),
            "idf_version": _cstring(first_segment[112:144], "IDF version"),
            "sha256": hashlib.sha256(data[:image_size]).hexdigest(),
            "appended_sha256": digest.hex(), "segment_count": segment_count,
            "segments": segments}


def inspect_flash(data: bytes) -> dict:
    if len(data) != FLASH_SIZE:
        raise LayoutError("Expected a complete 4MB flash backup.")
    partitions = parse_partition_table(data[PARTITION_TABLE_OFFSET : PARTITION_TABLE_OFFSET + PARTITION_TABLE_SIZE])
    ota = select_active_partition(data[OTADATA_OFFSET : OTADATA_OFFSET + OTADATA_SIZE], partitions)
    selected = ota["partition"]
    app = parse_app_image(data[selected["offset"] : selected["offset"] + selected["size"]])
    return {"partitions": partitions, "ota": ota, "app": app,
            "flash_sha256": hashlib.sha256(data).hexdigest()}
