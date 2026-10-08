import shutil
import subprocess
from pathlib import Path

import pytest


def test_firmware_latch_on_host(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ unavailable for portable firmware state-machine test")
    base = Path(__file__).resolve().parents[1]
    binary = tmp_path / "firmware-latch"
    subprocess.run(
        [
            compiler,
            "-std=c++11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(base / "firmware/esp32_estop/include"),
            str(base / "tests/firmware_latch.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        timeout=20,
    )
    subprocess.run([str(binary)], check=True, capture_output=True, timeout=5)
