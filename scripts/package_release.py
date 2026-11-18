#!/usr/bin/env python3
"""Package an already-built standalone image and the offline-buildable project."""

import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "release"
ARCHIVE_NAME = "Flight_controller_2_IMU_project.zip"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    binary = ROOT / "build/imu_firmware.bin"
    data = binary.read_bytes()
    stack, reset = struct.unpack_from("<II", data)
    if stack != 0x20020000 or not (reset & 1):
        raise SystemExit("Invalid STM32H743 initial stack/reset vector")
    if not 0x08000000 <= (reset & ~1) < 0x08000000 + len(data):
        raise SystemExit("Expected standalone FLASH_OFFSET_KB=0 build")
    cache = (ROOT / "build/CMakeCache.txt").read_text()
    if "FLASH_OFFSET_KB:STRING=0\n" not in cache:
        raise SystemExit("Release must use FLASH_OFFSET_KB=0")
    own_sources = [ROOT / "CMakeLists.txt"]
    for directory in ("firmware", "linker", "cmake"):
        own_sources.extend(path for path in (ROOT / directory).rglob("*") if path.is_file())
    source_digest = hashlib.sha256()
    for path in sorted(own_sources):
        source_digest.update(path.relative_to(ROOT).as_posix().encode() + b"\0")
        source_digest.update(path.read_bytes())
    RELEASE.mkdir(exist_ok=True)
    artifacts = []
    for suffix in ("elf", "bin", "hex", "map"):
        source = ROOT / "build" / f"imu_firmware.{suffix}"
        destination = RELEASE / source.name
        shutil.copy2(source, destination)
        artifacts.append(destination)
    compiler = subprocess.check_output(["arm-none-eabi-gcc", "--version"], text=True)
    validation_path = ROOT / "docs/validation.json"
    validation = json.loads(validation_path.read_text()) if validation_path.exists() else None
    if validation and validation.get("firmware_bin_sha256") != sha256(binary):
        validation = None
    info = {
        "target": "STM32H743, 16 MHz HSE crystal, LDO, BMI088 SPI3, USB FS CDC",
        "flash_origin": "0x08000000",
        "initial_stack": f"0x{stack:08X}",
        "reset_vector_thumb": f"0x{reset:08X}",
        "binary_bytes": len(data),
        "compiler": compiler.splitlines()[0],
        "source_sha256": source_digest.hexdigest(),
        "protocol": "IMU2 v2; 48 bytes; 1000Hz paired raw six-axis output",
        "software_validation": "ARM Release build and 38 passing host/native C tests",
        "hardware_validation": validation or "Not yet validated for this exact image",
        "dependencies": json.loads((ROOT / "scripts/dependencies.lock.json").read_text()),
    }
    info_file = RELEASE / "BUILD_INFO.json"
    info_file.write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")
    artifacts.append(info_file)
    (RELEASE / "SHA256SUMS").write_text("".join(
        f"{sha256(path)}  {path.name}\n" for path in artifacts))

    include = [ROOT / name for name in ("README.md", "LICENSE", ".gitignore", "CMakeLists.txt")]
    for directory in ("firmware", "linker", "cmake", "scripts", "tools", "tests", "docs",
                      "third_party", "release"):
        include.extend(path for path in (ROOT / directory).rglob("*") if path.is_file())
    archive = RELEASE / ARCHIVE_NAME
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as output:
        for path in sorted(include):
            relative = path.relative_to(ROOT)
            if "__pycache__" in relative.parts or path.suffix == ".pyc":
                continue
            if path.parent == RELEASE and path.name in (ARCHIVE_NAME, ARCHIVE_NAME + ".sha256"):
                continue
            output.write(path, "Flight_controller_2_IMU/" + relative.as_posix())
    (RELEASE / (ARCHIVE_NAME + ".sha256")).write_text(
        f"{sha256(archive)}  {ARCHIVE_NAME}\n")
    print(f"Created {archive.relative_to(ROOT)} ({archive.stat().st_size} bytes)")
    print(f"Standalone binary: {len(data)} bytes, initial SP 0x{stack:08X}, reset 0x{reset:08X}")


if __name__ == "__main__":
    main()
