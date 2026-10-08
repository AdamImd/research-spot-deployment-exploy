"""Short offline acceptance run from a clean commit. No robot/configuration access."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

from spot_deploy.contracts import ContractError
from spot_deploy.records import RunRecord, root, source_identity, verify_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = source_identity()
    if source["dirty"]:
        raise ContractError("commit the package before acceptance validation")
    record = RunRecord(args.output, "offline-package-acceptance")
    cases = []
    fixture = root() / "fixtures" / "standing"
    commands = [
        [sys.executable, "-m", "pytest", "-q"],
        [str(Path(sys.executable).parent / "ruff"), "check", "src", "tests", "tools"],
    ]
    for mode in ("inspect-policy", "replay", "readiness"):
        cmd = [sys.executable, "-m", "spot_deploy.cli", mode, "--output", str(args.output / mode)]
        if mode != "readiness":
            cmd += ["--manifest", str(fixture / "manifest.json")]
        if mode == "replay":
            cmd += [
                "--states",
                str(fixture / "states.jsonl"),
                "--envelope",
                str(fixture / "envelope.json"),
            ]
        commands.append(cmd)
    try:
        for index, command in enumerate(commands):
            completed = subprocess.run(
                command, cwd=root(), capture_output=True, text=True, timeout=30
            )
            log = record.directory / f"check-{index:02d}.log"
            log.write_text(completed.stdout + completed.stderr)
            cases.append({"command": command, "returncode": completed.returncode, "log": log.name})
            if completed.returncode:
                raise ContractError(f"acceptance step {index} failed")
        for mode in ("inspect-policy", "replay", "readiness"):
            if not verify_run(args.output / mode):
                raise ContractError("nested completion record failed verification")
        result = {
            "passed": True,
            "checks": cases,
            "hardware_access": False,
            "hardware_readiness": "unverified",
            "live_rl": "blocked",
            "source_commit": source["commit"],
        }
        record.finish("passed", result)
        print(json.dumps(result))
        return 0
    except BaseException as exc:
        record.finish(
            "failed", {"passed": False, "checks": cases, "error_type": type(exc).__name__}
        )
        raise


if __name__ == "__main__":
    raise SystemExit(main())
