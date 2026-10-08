#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
if [[ $# != 1 ]]; then
  echo "Usage: $0 /absolute/path/to/isaac-compatible/python" >&2
  exit 2
fi
"$1" -c 'import sys, torch; assert sys.version_info[:2] == (3, 11); assert torch.__version__ == "2.7.0+cu128", torch.__version__'
uv venv --python "$1" --system-site-packages "$root/.venv-exploy-export"
uv pip install --python "$root/.venv-exploy-export/bin/python" --no-deps -r "$root/requirements-exploy-export.txt"
"$root/.venv-exploy-export/bin/python" -c 'import torch, onnx, onnxruntime, onnxscript, pydantic; assert torch.__version__ == "2.7.0+cu128"'
