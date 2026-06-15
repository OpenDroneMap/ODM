#!/bin/bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "${ROOT}/scripts/pixi-shell-hook" ] \
   && [ -f "${ROOT}/.pixi/envs/prod/bin/python" ]; then
    . "${ROOT}/scripts/pixi-shell-hook"
fi

# The pixi env is now activated, so whatever we exec below (and any process it
# spawns) inherits it. This lets downstream images (e.g. NodeODM) inherit this
# entrypoint and run their own command activated, while standalone ODM usage
# keeps working: if no command is given, or the first argument isn't an
# executable on PATH (i.e. it's an ODM option or a positional project name),
# dispatch to run.py. A real command (node, bash, ...) is exec'd as-is.
if [ "$#" -eq 0 ] || ! command -v -- "$1" >/dev/null 2>&1; then
    set -- python3 "${ROOT}/run.py" "$@"
fi

exec "$@"
