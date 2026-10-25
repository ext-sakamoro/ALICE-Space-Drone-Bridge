#!/usr/bin/env bash
# Every CI gate, locally. Matches .github/workflows/ci.yml step for step: a step added to CI is added here.
#
#   scripts/preflight.sh            everything
#   scripts/preflight.sh --quick    the same without the test run
set -euo pipefail
cd "$(dirname "$0")/.."

quick=0
[[ "${1:-}" == "--quick" ]] && quick=1

step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

step "siblings: each ../<repo> of .github/alice-siblings is checked out at its pinned commit"
n=0
while read -r repo sha; do
  case "$repo" in ''|'#'*) continue ;; esac
  got=$(git -C "../$repo" rev-parse HEAD)
  if [ "$got" != "$sha" ]; then echo "../$repo is at $got, the pin is $sha" >&2; exit 1; fi
  n=$((n + 1))
done < .github/alice-siblings
[ "$n" -gt 0 ] || { echo 'no sibling pinned' >&2; exit 1; }

step "fmt"
cargo fmt --all -- --check

step "clippy (all targets, -D warnings)"
cargo clippy --workspace --all-targets -- -D warnings

if [ "$quick" -eq 0 ]; then
  step "test"
  cargo test --workspace
fi

printf '\n\033[1;32mpreflight green\033[0m\n'
