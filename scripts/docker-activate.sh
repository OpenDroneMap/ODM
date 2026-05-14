# Docker production image only: source pixi shell-hook when prod env is present.
# Not used during builder/tests (default pixi env via `pixi run`).

_docker_activate_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "${_docker_activate_root}/scripts/pixi-shell-hook" ] \
   && [ -f "${_docker_activate_root}/.pixi/envs/prod/bin/python" ]; then
    # shellcheck disable=SC1091
    . "${_docker_activate_root}/scripts/pixi-shell-hook"
fi
