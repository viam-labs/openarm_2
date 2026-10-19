#!/bin/sh
# Entrypoint for viam-server. The socket path is passed through to the module.
set -eu
cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  ./setup.sh
fi

export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
exec .venv/bin/python -m openarm_viam "$@"
