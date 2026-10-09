#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
status=0

echo "[check] shell syntax"
while IFS= read -r -d '' f; do bash -n "$f" || status=1; done \
  < <(find launch scripts tuning -type f -name '*.sh' -print0)

echo "[check] Python syntax"
python3 - <<'PY' || status=1
import ast
from pathlib import Path
for root in (Path('launch'), Path('scripts'), Path('tuning')):
    for path in root.rglob('*.py'):
        if '__pycache__' in path.parts:
            continue
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
PY

echo "[check] forbidden host paths"
if rg -n '/home/yf|/root/codes/ros1_to_pingslam_bag' \
    --glob '!scripts/check_public_release.sh' \
    --glob '!build/**' --glob '!install/**' --glob '!log/**' \
    --glob '!data/**' --glob '!tuning_runs/**' --glob '!recordresults/**' \
    --glob '!paper_video/**' .; then status=1; fi

echo "[check] required release documents"
for f in README.md REPRODUCE.md dependencies/README.md; do
  [[ -s "$f" ]] || { echo "missing $f" >&2; status=1; }
done

if [[ $status -ne 0 ]]; then echo '[check] FAILED' >&2; exit "$status"; fi
echo '[check] OK'
