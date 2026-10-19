#!/bin/sh
# Create the virtualenv viam-server uses to run this module.
set -eu
cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  if ! python3 -m venv .venv; then
    echo "openarm setup: python3 -m venv failed; trying to install python3-venv" >&2
    if ! command -v apt-get >/dev/null 2>&1; then
      echo "python3 venv is required. On Debian/Ubuntu: sudo apt install python3-venv" >&2
      exit 1
    fi
    if [ "$(id -u)" -eq 0 ]; then
      apt-get update -qq
      apt-get install -y python3-venv
    elif command -v sudo >/dev/null 2>&1; then
      sudo apt-get update -qq
      sudo apt-get install -y python3-venv
    else
      echo "python3 venv is required. On Debian/Ubuntu: sudo apt install python3-venv" >&2
      exit 1
    fi
    python3 -m venv .venv
  fi
fi

.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
