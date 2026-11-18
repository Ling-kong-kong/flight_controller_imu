#!/usr/bin/env python3
"""Restore the minimal vendor source trees from SHA-256 pinned archives."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
HAL_SOURCES = {
    "stm32h7xx_hal.c", "stm32h7xx_hal_cortex.c", "stm32h7xx_hal_dma.c",
    "stm32h7xx_hal_dma_ex.c", "stm32h7xx_hal_gpio.c", "stm32h7xx_hal_rcc.c",
    "stm32h7xx_hal_rcc_ex.c", "stm32h7xx_hal_pwr.c", "stm32h7xx_hal_pwr_ex.c",
    "stm32h7xx_hal_flash.c", "stm32h7xx_hal_flash_ex.c", "stm32h7xx_hal_spi.c",
    "stm32h7xx_hal_spi_ex.c", "stm32h7xx_hal_exti.c",
}


def wanted(name: str, path: str) -> bool:
    if path in {"LICENSE", "LICENSE.md", "LICENSE.txt", "README.md", "README.rst"}:
        return True
    if name == "stm32h7xx_hal_driver":
        return path.startswith("Inc/") or (
            path.startswith("Src/") and PurePosixPath(path).name in HAL_SOURCES)
    if name == "cmsis_device_h7":
        return path in {
            "Include/stm32h7xx.h", "Include/stm32h743xx.h", "Include/system_stm32h7xx.h",
            "Source/Templates/system_stm32h7xx.c",
            "Source/Templates/gcc/startup_stm32h743xx.s",
        }
    if name == "cmsis_5":
        return path.startswith("CMSIS/Core/Include/")
    if name == "tinyusb":
        return path.startswith("src/")
    raise ValueError(f"Unknown dependency: {name}")


def install(dep: dict, cache: Path, force: bool) -> None:
    target = ROOT / "third_party" / dep["name"]
    marker = target / ".source-sha256"
    if not force and marker.exists() and marker.read_text().strip() == dep["sha256"]:
        print(f"{dep['name']} {dep['version']}: already present")
        return
    archive = cache / f"{dep['name']}-{dep['version']}.tar.gz"
    if not archive.exists():
        print(f"Downloading {dep['url']}")
        with urllib.request.urlopen(dep["url"], timeout=180) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != dep["sha256"]:
            raise RuntimeError(f"SHA-256 mismatch: {dep['name']}")
        archive.write_bytes(data)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != dep["sha256"]:
        raise RuntimeError(f"SHA-256 mismatch: {archive}; delete it and retry")
    with tempfile.TemporaryDirectory(prefix="imu-vendor-", dir=ROOT / "third_party") as tmp:
        stage = Path(tmp)
        with tarfile.open(archive, "r:gz") as tar:
            for member in tar:
                parts = PurePosixPath(member.name).parts
                relative = "/".join(parts[1:])
                if not relative or not wanted(dep["name"], relative):
                    continue
                if ".." in parts or member.issym() or member.islnk():
                    raise RuntimeError(f"Unsafe archive member: {member.name}")
                if not member.isfile():
                    continue
                output = stage.joinpath(*parts[1:])
                output.parent.mkdir(parents=True, exist_ok=True)
                source = tar.extractfile(member)
                if source is None:
                    raise RuntimeError(f"Missing archive member: {member.name}")
                with source, output.open("wb") as destination:
                    shutil.copyfileobj(source, destination)
        (stage / ".source-sha256").write_text(dep["sha256"] + "\n")
        if target.exists():
            shutil.rmtree(target)
        shutil.copytree(stage, target)
    print(f"{dep['name']} {dep['version']}: installed")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", type=Path, default=ROOT / ".cache" / "dependencies")
    parser.add_argument("--force", action="store_true", help="Re-extract pinned sources")
    args = parser.parse_args()
    args.cache_dir.mkdir(parents=True, exist_ok=True)
    (ROOT / "third_party").mkdir(exist_ok=True)
    lock = json.loads((ROOT / "scripts" / "dependencies.lock.json").read_text())
    for dep in lock["dependencies"]:
        install(dep, args.cache_dir, args.force)


if __name__ == "__main__":
    main()
