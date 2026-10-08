import json
from pathlib import Path

import pytest

from spot_deploy.contracts import Envelope, Manifest, State, load

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "standing"


@pytest.fixture
def fixture_path():
    return FIXTURE


@pytest.fixture
def manifest():
    return load(FIXTURE / "manifest.json", Manifest)


@pytest.fixture
def upstream_manifest(manifest):
    reference = json.loads((FIXTURE.parent / "upstream-reference.json").read_text())
    return Manifest.model_validate(manifest.model_dump() | reference["manifest_patch"])


@pytest.fixture
def envelope():
    return load(FIXTURE / "envelope.json", Envelope)


@pytest.fixture
def state():
    return State.model_validate(json.loads((FIXTURE / "states.jsonl").read_text().splitlines()[0]))
