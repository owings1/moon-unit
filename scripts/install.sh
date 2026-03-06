#!/bin/bash
set -e
if [[ "$#" -ne 1 ]]; then
  echo "Usage: $0 <dest>" >&2
  exit 1
fi
dest="$(realpath "$1")"
cd "$(dirname "$0")/../src/cmdr"
cp -X -v -r \
  code.py \
  defaults.py \
  utils.py \
  components \
  "$dest"
cp -X -v -n \
  settings.py \
  "$dest" || true
cp -X -v -n -r \
  lib/* \
  "$dest/lib/" || true