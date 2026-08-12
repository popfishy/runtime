#!/usr/bin/env bash
set -eo pipefail

RUNTIME_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PYTHONPATH="${RUNTIME_ROOT}/src:${PYTHONPATH:-}"

exec python3 "${RUNTIME_ROOT}/main.py" --mode tcp "$@"
