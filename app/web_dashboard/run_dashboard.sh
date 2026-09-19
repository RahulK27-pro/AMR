#!/usr/bin/env bash
set -e

# Prioritize native Linux Node & npm installed in home directory
if [ -d "$HOME/node-v20.18.0-linux-x64/bin" ]; then
  export PATH="$HOME/node-v20.18.0-linux-x64/bin:$PATH"
fi

cd "$(dirname "$0")"

echo "Using Node: $(which node) ($(node -v))"
echo "Using npm:  $(which npm)  ($(npm -v))"
echo "Starting AMR Control Dashboard on http://localhost:5173 ..."

npm run dev -- --host 0.0.0.0
