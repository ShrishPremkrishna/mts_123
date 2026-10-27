#!/bin/bash
# Remove API keys from a Pupper. Run before rebasing any test Pupper.
#   ./wipe_api_keys.sh <ssh-host> [--dry-run] [--include-demo-key]
# Blanks every *KEY*/*SECRET*/*TOKEN*/*PASSWORD* value in real env files under
# /home/pi (templates like .env.example are left alone), then scans the home
# folder for key-shaped strings that remain and reports them masked.
# --include-demo-key also deletes the Gemini demo key (~/gemini_eval/.gemini_key)
# and the kiosk browser profile that stores it.
set -euo pipefail
HOST=${1:?ssh host}; shift
ssh "$HOST" "bash -s -- $*" < "$(dirname "$0")/lib/wipe_keys_remote.sh"
