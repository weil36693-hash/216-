#!/usr/bin/env python3
"""Portable, fail-closed app-only installer for the fleet's runtime firmware."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import uuid

import device_layout as formats
from device_layout import LayoutError, normalize_mac, validate_observed_mac
from firmware_identity import FIRMWARES


ESPTOOL_VERSION = "4.13.0"
PACKAGE_ROOT = Path(__file__).resolve().parent
DEVICE = "camera"
FIRMWARE_SHA256 = FIRMWARES[DEVICE]["sha256"]
FIRMWARE_SIZE = FIRMWARES[DEVICE]["size"]
FIRMWARE_PATH = PACKAGE_ROOT / "firmware" / FIRMWARES[DEVICE]["file"]
BACKUP_FORMAT = "fleet-runtime-backup-v1"


def configure_device(device: str) -> None:
    global DEVICE, FIRMWARE_SHA256, FIRMWARE_SIZE, FIRMWARE_PATH
    formats.configure_device(device)
    DEVICE = device
    FIRMWARE_SHA256 = FIRMWARES[device]["sha256"]
    FIRMWARE_SIZE = FIRMWARES[device]["size"]
    FIRMWARE_PATH = PACKAGE_ROOT / "firmware" / FIRMWARES[device]["file"]



class FlashError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def private_mode(path: Path, mode: int) -> None:
    try:
        path.chmod(mode)
    except OSError:
        if os.name != "nt":
            raise


def sync_directory(directory: Path) -> None:
    # Windows does not expose POSIX directory fsync; files are still fsynced.
    if os.name == "nt":
        return
    descriptor = os.open(str(directory), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def private_write(path: Path, content: bytes) -> None:
    """Create a new private file; never overwrite an existing backup."""
    descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    private_mode(path, 0o600)
    sync_directory(path.parent)


def save_json(path: Path, value: dict, *, replace: bool = False) -> None:
    content = (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    if not replace:
        private_write(path, content)
        return
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    private_write(temporary, content)
    os.replace(temporary, path)
    sync_directory(path.parent)


def make_run_directory(root: Path, car: str, mac: str, prefix: str = "") -> Path:
    root = root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    sync_directory(root.parent)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    name = f"{prefix}{car}_{mac.replace(':', '')}_{stamp}_{uuid.uuid4().hex[:8]}"
    result = root / name
    result.mkdir(mode=0o700, exist_ok=False)
    private_mode(result, 0o700)
    sync_directory(root)
    return result


def check_dependencies() -> None:
    if sys.version_info < (3, 10):
        raise FlashError("Python 3.10 or newer is required.")
    try:
        installed = metadata.version("esptool")
    except metadata.PackageNotFoundError as exc:
        raise FlashError("Install dependencies first: python -m pip install -r requirements.txt") from exc
    if installed != ESPTOOL_VERSION:
        raise FlashError(f"Expected esptool=={ESPTOOL_VERSION}; installed version is {installed}.")


class EsptoolRunner:
    def __init__(self, port: str, log_dir: Path, expected_mac: str):
        self.port = port
        self.log_dir = log_dir
        self.expected_mac = normalize_mac(expected_mac)
        self.counter = 0

    def run(self, args: list[str], before: str = "no_reset", after: str = "no_reset") -> str:
        self.counter += 1
        operation = args[0]
        command = [sys.executable, "-m", "esptool", "--chip", "esp32s3",
                   "--port", self.port, "--baud", "460800", "--before", before,
                   "--after", after, *args]
        log_path = self.log_dir / f"{self.counter:02d}_{operation}.log"
        print(f"[{self.counter:02d}] {operation} ...", flush=True)
        environment = os.environ.copy()
        environment["PYTHONIOENCODING"] = "utf-8"
        try:
            completed = subprocess.run(command, capture_output=True, text=True,
                                       encoding="utf-8", errors="replace", timeout=600,
                                       env=environment, check=False)
        except subprocess.TimeoutExpired as exc:
            output = exc.stdout or b""
            error = exc.stderr or b""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            if isinstance(error, bytes):
                error = error.decode("utf-8", errors="replace")
            private_write(log_path, (output + "\n" + error + "\nTIMEOUT\n").encode("utf-8"))
            raise FlashError(f"{operation} timed out; see {log_path}") from exc
        except (OSError, KeyboardInterrupt) as exc:
            private_write(log_path, f"{type(exc).__name__}: operation interrupted or could not start.\n".encode())
            raise
        output = completed.stdout + "\n" + completed.stderr
        private_write(log_path, output.encode("utf-8"))
        if completed.returncode != 0:
            raise FlashError(f"esptool {operation} failed (exit {completed.returncode}); see {log_path}")
        validate_observed_mac(output, self.expected_mac)
        return output


def read_region(runner: EsptoolRunner, offset: int, size: int, destination: Path) -> bytes:
    # Create mode 0600 before esptool opens the path, including on a permissive umask.
    private_write(destination, b"")
    runner.run(["read_flash", hex(offset), hex(size), str(destination)])
    private_mode(destination, 0o600)
    # esptool closed the file, but its data may remain only in the OS write cache.
    # Flush the completed backup before permitting any flash write.
    with destination.open("r+b") as stream:
        os.fsync(stream.fileno())
    data = destination.read_bytes()
    if len(data) != size:
        raise FlashError(f"Incomplete flash read: expected {size} bytes; retained {destination}.")
    return data


def probe_target(runner: EsptoolRunner, *, first: bool) -> dict:
    security = formats.validate_security_info(
        runner.run(["get_security_info"], before="default_reset" if first else "no_reset"),
        runner.expected_mac,
    )
    formats.validate_flash_id(runner.run(["flash_id"]), runner.expected_mac)
    return security


def release_reset_lines(port: str) -> None:
    """Native USB Serial/JTAG can stay reset until both modem signals are released."""
    import serial

    connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    try:
        connection.dtr = False
        connection.rts = False
        connection.port = port
        connection.open()
        connection.dtr = False
        connection.rts = False
    finally:
        connection.close()


def reboot(runner: EsptoolRunner, result: dict) -> None:
    result["reset_requested"] = False
    result["reset_lines_released"] = False
    try:
        runner.run(["read_mac"], after="hard_reset")
        result["reset_requested"] = True
        release_reset_lines(runner.port)
        result["reset_lines_released"] = True
    except (Exception, KeyboardInterrupt) as exc:
        result["reset_warning"] = f"{type(exc).__name__}: {exc}"
        print("Firmware was verified, but automatic restart was not confirmed. Unplug and reconnect USB/power.")
    result["video_verified"] = False


def require_confirmation(word: str) -> None:
    print("Keep this same device connected. Close serial monitors and other flashing programs.")
    try:
        answer = input(f"Type {word} exactly to write this device, or press Enter to cancel: ")
    except EOFError as exc:
        raise FlashError("Interactive confirmation is required; no flash was written.") from exc
    if answer.strip() != word:
        raise FlashError("Cancelled; no flash was written.")


def show_target(car: str, mac: str, port: str, layout: dict, run_dir: Path, action: str) -> None:
    partition = layout["ota"]["partition"]
    print(f"\nAction: {action}\nCar: {car}\nPort: {port}\nExpected / verified MAC: {mac}")
    print(f"Target: {DEVICE}, ESP32-S3, 4MB flash, security disabled")
    print(f"Project: {layout['app']['project_name']}\nOriginal version: {layout['app']['version']}")
    print(f"Write range: {partition['label']} 0x{partition['offset']:06x}.."
          f"0x{partition['offset'] + partition['size'] - 1:06x} ({partition['size']} bytes)")
    print(f"Backup / logs: {run_dir}")
    print("Preserved: bootloader, partition table, NVS/Wi-Fi credentials, OTA metadata, PHY, inactive app.\n")


def validate_car(value: str) -> str:
    if not re.fullmatch(r"car(?:0[1-9]|1[0-9]|2[0-5])", value):
        raise FlashError("Use a car identifier from car01 through car25.")
    return value


def verify_bundle() -> bytes:
    data = FIRMWARE_PATH.read_bytes()
    if len(data) != FIRMWARE_SIZE or sha256(data) != FIRMWARE_SHA256:
        raise FlashError("Bundled application image has the wrong size or SHA256; no device was touched.")
    info = formats.parse_app_image(data)
    if info["image_size"] != len(data):
        raise FlashError("Unexpected padding in bundled firmware.")
    return data


def snapshot_metadata(car: str, mac: str, data: bytes, layout: dict) -> dict:
    partition = layout["ota"]["partition"]
    app = data[partition["offset"] : partition["offset"] + partition["size"]]
    return {
        "format": BACKUP_FORMAT, "device": DEVICE, "created_utc": utc_now(), "car": car, "mac": mac,
        "flash_file": "original_flash_4mb.bin", "flash_size": formats.FLASH_SIZE,
        "flash_sha256": sha256(data), "active_partition": partition,
        "original_app": layout["app"], "original_app_file": "original_active_app.bin",
        "original_app_sha256": sha256(app),
        "protected_sha256": sha256(data[:formats.PROTECTED_SIZE]),
        "partition_table_sha256": sha256(data[formats.PARTITION_TABLE_OFFSET:formats.PARTITION_TABLE_OFFSET + formats.PARTITION_TABLE_SIZE]),
        "otadata_sha256": sha256(data[formats.OTADATA_OFFSET:formats.OTADATA_OFFSET + formats.OTADATA_SIZE]),
    }


def write_and_verify(runner: EsptoolRunner, image: Path, partition: dict,
                     protected_before: bytes, run_dir: Path, result: dict) -> None:
    offset = partition["offset"]
    if len(image.read_bytes()) != partition["size"]:
        raise FlashError("Write image must occupy exactly the verified active app partition.")
    result["write_started"] = True
    result["status"] = "writing"
    save_json(run_dir / "result.json", result, replace=True)
    runner.run(["write_flash", "--flash_mode", "keep", "--flash_freq", "keep",
                "--flash_size", "keep", hex(offset), str(image)])
    result["status"] = "verifying"
    save_json(run_dir / "result.json", result, replace=True)
    runner.run(["verify_flash", hex(offset), str(image)])
    result["application_verified"] = True
    protected_after = read_region(runner, 0, formats.PROTECTED_SIZE, run_dir / "protected_after.bin")
    if protected_after != protected_before:
        raise FlashError("Protected flash regions changed. Do not reboot; keep backup and inspect logs.")
    result["protected_regions_verified"] = True
    result["protected_after_sha256"] = sha256(protected_after)


def finish_error(result: dict, run_dir: Path, error: BaseException) -> None:
    result["status"] = "failed"
    result["finished_utc"] = utc_now()
    result["error"] = f"{type(error).__name__}: {error}"
    result["automatic_restart_attempted_after_failure"] = False
    try:
        save_json(run_dir / "result.json", result, replace=True)
    except OSError:
        pass
    print(f"Backup / logs retained: {run_dir}", file=sys.stderr)
    if result.get("write_started"):
        recovery_dir = result.get("backup_directory", str(run_dir))
        print("Write or verification failed. No automatic restart was attempted.", file=sys.stderr)
        print(f"For app-only recovery, use restore --backup with this original backup directory: {recovery_dir}", file=sys.stderr)
    else:
        print("No flash write was started. Device may remain in the bootloader; reconnect USB/power to restart.", file=sys.stderr)


def initial_result(action: str, car: str, mac: str, port: str) -> dict:
    return {"format": "fleet-runtime-result-v1", "device": DEVICE, "action": action, "car": car, "mac": mac,
            "port": port, "started_utc": utc_now(), "status": "inspecting",
            "write_started": False, "application_verified": False,
            "protected_regions_verified": False, "video_verified": False}


def flash(args: argparse.Namespace) -> int:
    car = validate_car(args.car)
    mac = normalize_mac(args.expect_mac)
    firmware = verify_bundle()
    check_dependencies()
    run_dir = make_run_directory(Path(args.backup_root), car, mac)
    result = initial_result("flash", car, mac, args.port)
    result["firmware_sha256"] = FIRMWARE_SHA256
    result["firmware_size"] = FIRMWARE_SIZE
    save_json(run_dir / "result.json", result)
    runner = EsptoolRunner(args.port, run_dir, mac)
    try:
        result["security"] = probe_target(runner, first=True)
        print("Reading the complete 4MB flash backup; this may take several minutes.", flush=True)
        original = read_region(runner, 0, formats.FLASH_SIZE, run_dir / "original_flash_4mb.bin")
        result["backup_sha256"] = sha256(original)
        save_json(run_dir / "result.json", result, replace=True)
        layout = formats.inspect_flash(original)
        partition = layout["ota"]["partition"]
        if len(firmware) > partition["size"]:
            raise FlashError("Bundled application exceeds the active OTA partition.")
        manifest = snapshot_metadata(car, mac, original, layout)
        private_write(run_dir / "original_active_app.bin",
                      original[partition["offset"] : partition["offset"] + partition["size"]])
        save_json(run_dir / "backup.json", manifest)
        padded = firmware + b"\xff" * (partition["size"] - len(firmware))
        image = run_dir / "padded_active_app.bin"
        private_write(image, padded)
        result.update({"backup_sha256": manifest["flash_sha256"], "active_partition": partition,
                       "app_offset": partition["offset"], "write_size": len(padded),
                       "write_sha256": sha256(padded), "status": "awaiting_confirmation"})
        save_json(run_dir / "result.json", result, replace=True)
        show_target(car, mac, args.port, layout, run_dir, "Install runtime-configurable application")
        require_confirmation("FLASH")
        # Recheck identity/security and the protected data after the human pause.
        probe_target(runner, first=False)
        prewrite = read_region(runner, 0, formats.PROTECTED_SIZE, run_dir / "protected_prewrite.bin")
        if prewrite != original[:formats.PROTECTED_SIZE]:
            raise FlashError("Protected metadata changed after backup; refusing to write.")
        if image.read_bytes() != padded:
            raise FlashError("Prepared application changed before writing.")
        write_and_verify(runner, image, partition, original[:formats.PROTECTED_SIZE], run_dir, result)
        result["status"] = "verified"
        save_json(run_dir / "result.json", result, replace=True)
        reboot(runner, result)
        result["status"] = "success"
        result["finished_utc"] = utc_now()
        save_json(run_dir / "result.json", result, replace=True)
        print(f"\nVerified installation: {car}, MAC {mac}, app offset 0x{partition['offset']:x}.")
        print(f"Protected regions match the backup. Result: {run_dir / 'result.json'}")
        print("Hardware behavior is not tested by this tool. Disconnect BOOT jumpers, then check /api/status before operation.")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        finish_error(result, run_dir, exc)
        raise


def load_backup(directory: Path) -> tuple[dict, bytes, dict]:
    manifest = json.loads((directory / "backup.json").read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("format") != BACKUP_FORMAT:
        raise FlashError("Unsupported backup format.")
    if manifest.get("flash_file") != "original_flash_4mb.bin" or manifest.get("flash_size") != formats.FLASH_SIZE:
        raise FlashError("Unexpected backup file or flash size.")
    if manifest.get("device") != DEVICE:
        raise FlashError("Backup device type differs from --device.")
    data = (directory / "original_flash_4mb.bin").read_bytes()
    if sha256(data) != manifest.get("flash_sha256"):
        raise FlashError("Full backup SHA256 mismatch.")
    layout = formats.inspect_flash(data)
    mac = normalize_mac(manifest["mac"])
    car = validate_car(manifest["car"])
    expected = snapshot_metadata(car, mac, data, layout)
    for key in ("active_partition", "original_app", "original_app_file", "original_app_sha256",
                "protected_sha256", "partition_table_sha256", "otadata_sha256"):
        if manifest.get(key) != expected[key]:
            raise FlashError(f"Backup metadata mismatch: {key}.")
    return manifest, data, layout


def restore(args: argparse.Namespace) -> int:
    backup_dir = Path(args.backup).expanduser().resolve()
    manifest, original, layout = load_backup(backup_dir)
    mac, car = normalize_mac(manifest["mac"]), validate_car(manifest["car"])
    check_dependencies()
    run_dir = make_run_directory(backup_dir / "restore_runs", car, mac, "restore_")
    result = initial_result("restore", car, mac, args.port)
    result["backup_directory"] = str(backup_dir)
    result["backup_sha256"] = manifest["flash_sha256"]
    save_json(run_dir / "result.json", result)
    runner = EsptoolRunner(args.port, run_dir, mac)
    try:
        result["security"] = probe_target(runner, first=True)
        current = read_region(runner, 0, formats.PROTECTED_SIZE, run_dir / "protected_before.bin")
        partitions = formats.parse_partition_table(current[formats.PARTITION_TABLE_OFFSET:formats.PARTITION_TABLE_OFFSET + formats.PARTITION_TABLE_SIZE])
        ota = formats.select_active_partition(current[formats.OTADATA_OFFSET:formats.OTADATA_OFFSET + formats.OTADATA_SIZE], partitions)
        for offset, size in ((formats.PARTITION_TABLE_OFFSET, formats.PARTITION_TABLE_SIZE), (formats.OTADATA_OFFSET, formats.OTADATA_SIZE)):
            if current[offset:offset + size] != original[offset:offset + size]:
                raise FlashError("Current partition table / OTA metadata differs from the backup.")
        if ota["partition"] != layout["ota"]["partition"]:
            raise FlashError("Current active application differs from backup; refusing restore.")
        partition = ota["partition"]
        original_app = original[partition["offset"] : partition["offset"] + partition["size"]]
        image = run_dir / "restore_active_app.bin"
        private_write(image, original_app)
        result.update({"active_partition": partition, "app_offset": partition["offset"],
                       "write_size": len(original_app), "write_sha256": sha256(original_app),
                       "status": "awaiting_confirmation"})
        save_json(run_dir / "result.json", result, replace=True)
        show_target(car, mac, args.port, layout, backup_dir, "Restore original active application")
        require_confirmation("RESTORE")
        probe_target(runner, first=False)
        prewrite = read_region(runner, 0, formats.PROTECTED_SIZE, run_dir / "protected_prewrite.bin")
        if prewrite != current:
            raise FlashError("Protected flash changed during restore confirmation; refusing to write.")
        if image.read_bytes() != original_app:
            raise FlashError("Prepared restore application changed before writing.")
        write_and_verify(runner, image, partition, current, run_dir, result)
        result["status"] = "verified"
        save_json(run_dir / "result.json", result, replace=True)
        reboot(runner, result)
        result["status"] = "success"
        result["finished_utc"] = utc_now()
        save_json(run_dir / "result.json", result, replace=True)
        print(f"\nOriginal application restored and verified: {car}, MAC {mac}, offset 0x{partition['offset']:x}.")
        print(f"Current protected regions preserved. Result: {run_dir / 'result.json'}")
        print("Video has not been tested by this tool.")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        finish_error(result, run_dir, exc)
        raise


def ports(_args: argparse.Namespace) -> int:
    check_dependencies()
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        print("No serial ports found. Connect the selected module using a USB data cable.")
    for port in sorted(found, key=lambda entry: entry.device):
        vidpid = f"{port.vid:04x}:{port.pid:04x}" if port.vid is not None and port.pid is not None else "unknown"
        print(f"{port.device} | {port.description} | USB {vidpid} | serial={port.serial_number or 'unknown'}")
    print("Port listing does not identify the module. Match --expect-mac to its own Maturo_CAM_ or Maturo_ name.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Safely install/restore the runtime-configurable Maturo ESP32-S3 firmware.")
    parser.add_argument("--device", choices=("camera", "chassis"), default="camera", help="Hardware type; must match original app and layout")
    commands = parser.add_subparsers(dest="command", required=True)
    ports_parser = commands.add_parser("ports", help="List serial ports without opening them")
    ports_parser.set_defaults(handler=ports)
    flash_parser = commands.add_parser("flash", help="Back up, validate, confirm, then install the selected runtime firmware")
    flash_parser.add_argument("--port", required=True)
    flash_parser.add_argument("--car", required=True, help="Car identifier, car01 through car25")
    flash_parser.add_argument("--expect-mac", required=True, help="Expected module MAC from its Maturo_CAM_ (camera) or Maturo_ (chassis) name")
    flash_parser.add_argument("--backup-root", default=str(PACKAGE_ROOT / "backups"), help="Private backup directory")
    flash_parser.set_defaults(handler=flash)
    restore_parser = commands.add_parser("restore", help="Restore only the original active app from a verified backup")
    restore_parser.add_argument("--port", required=True)
    restore_parser.add_argument("--backup", required=True, help="One backup directory containing backup.json")
    restore_parser.set_defaults(handler=restore)
    args = parser.parse_args(argv)
    try:
        configure_device(args.device)
        return args.handler(args)
    except KeyboardInterrupt:
        print("Interrupted. Keep the backup and review result.json before restarting the device.", file=sys.stderr)
        return 130
    except (FlashError, LayoutError, OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
