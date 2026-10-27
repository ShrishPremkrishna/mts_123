#!/bin/bash
# Build the artifacts rebase.sh ships to each Pupper (run on this PC):
#   artifacts/monorepo-cs123-build.bundle  the cs123-build commit on top of the class image's commit
#   artifacts/pupper-rs                    the screen app, cross-compiled for the Pupper (aarch64, glibc 2.36)
# Needs ~/projects/pupperv3-monorepo-upstream (clone of Nate711/pupperv3-monorepo with the
# cs123-build branch), Rust with the aarch64 target, cargo-zigbuild and zig (~/.local/zigenv).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
UP=${UPSTREAM:-$HOME/projects/pupperv3-monorepo-upstream}
BASE=b5b8c89ec5c71b18fe8bb3d3fa27d492441644b3
TARGET=0e50afbdc11ae066606b0290c23969f46a265626
export PATH=$HOME/.cargo/bin:$HOME/.local/zigenv/bin:$PATH
[ "$(git -C "$UP" rev-parse cs123-build)" = "$TARGET" ] || { echo "cs123-build in $UP is not $TARGET"; exit 1; }
mkdir -p "$HERE/artifacts" "$HERE/build"
git -C "$UP" bundle create "$HERE/artifacts/monorepo-cs123-build.bundle" cs123-build "^$BASE"
rm -rf "$HERE/build/pupper-rs" && git -C "$UP" archive cs123-build pupper-rs | tar -x -m -C "$HERE/build"
(cd "$HERE/build/pupper-rs" && cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.36)
cp "$HERE/build/pupper-rs/target/aarch64-unknown-linux-gnu/release/pupper-rs" "$HERE/artifacts/pupper-rs"
(cd "$HERE/artifacts" && sha256sum monorepo-cs123-build.bundle pupper-rs > SHA256SUMS && cat SHA256SUMS)
