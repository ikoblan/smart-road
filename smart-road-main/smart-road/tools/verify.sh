#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")/.."
command -v cargo >/dev/null 2>&1 || { echo 'Install a stable Rust toolchain with Cargo first.' >&2; exit 1; }
cargo test --no-default-features
cargo build --release
cargo test
cargo run --release --no-default-features -- --headless 60 42
printf '\nCommand-line checks finished. Run cargo run --release for manual GUI checks.\n'
