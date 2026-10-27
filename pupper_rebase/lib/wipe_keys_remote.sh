#!/bin/bash
# Runs on the Pupper. Arguments: [--dry-run] [--include-demo-key]
set -uo pipefail
DRY=0; DEMO=0
for a in "$@"; do case $a in --dry-run) DRY=1;; --include-demo-key) DEMO=1;; esac; done
ROOT=${WIPE_ROOT:-$HOME}
SECRET_RE='^[[:space:]]*(export[[:space:]]+)?[A-Za-z0-9_]*(KEY|SECRET|TOKEN|PASSWORD)[A-Za-z0-9_]*[[:space:]]*='
mask() { sed -E 's/((AIza|sk-|sk_|ghp_|APIy)[A-Za-z0-9_-]{2})[A-Za-z0-9_-]{6,}([A-Za-z0-9_-]{2})/\1...\3/g'; }
act() { if [ $DRY = 1 ]; then echo "  would: $*"; else eval "$@"; fi; }

echo "== env files under $ROOT"
found=0
while IFS= read -r -d '' f; do
  case "$f" in *.example|*.sample|*.template|*.dist) continue;; esac
  n=$(grep -cE "${SECRET_RE}[[:space:]]*[\"']?[^\"'[:space:]]" "$f" || true)
  [ "$n" -gt 0 ] || continue
  found=1
  echo "$f: $n secret value(s)"
  grep -E "${SECRET_RE}" "$f" | sed -E 's/=.*/=<value>/' | sed 's/^/    /'
  act "sed -i -E \"s/(${SECRET_RE})[[:space:]]*.*/\\\\1\\\"\\\"/\" '$f'"
done < <(find "$ROOT" \( -name node_modules -o -name .git -o -name .cache \) -prune -o -type f \( -name ".env" -o -name ".env.*" -o -name "*.env" \) -print0 2>/dev/null)
[ $found = 1 ] || echo "  none with values"

if [ $DEMO = 1 ]; then
  echo "== Gemini demo key"
  for p in "$ROOT/gemini_eval/.gemini_key" "$ROOT/gemini_eval/gemini-kiosk-profile"; do
    [ -e "$p" ] && { echo "  removing $p"; act "rm -rf '$p'"; }
  done
  true
fi

echo "== key-shaped strings still in $ROOT (masked)"
grep -rIoE --exclude-dir=.git --exclude-dir=node_modules --exclude-dir=build --exclude-dir=install \
  --exclude-dir=log --exclude-dir=target --exclude-dir=venv --exclude-dir=.cache \
  'AIza[0-9A-Za-z_-]{35}|sk-[A-Za-z0-9_-]{20,}|sk_[A-Za-z0-9]{20,}|ghp_[A-Za-z0-9]{30,}' "$ROOT" 2>/dev/null \
  | mask | sort | uniq -c || true
echo "(done$( [ $DRY = 1 ] && echo ', dry run: nothing changed'))"
