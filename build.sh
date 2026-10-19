#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p dist
tar -czf dist/archive.tar.gz \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  meta.json \
  run.sh \
  requirements.txt \
  setup.sh \
  src/openarm_viam
