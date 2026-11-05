#!/usr/bin/env bash
# One-time setup of a RunPod pod for GodView Phase C (V2V4Real replay).
# Safe to re-run, and re-run it after a pod restart: anything outside
# /workspace (apt packages, Claude Code, its login) lives on the container
# disk and is wiped when the pod restarts. Rust, the repo and the data are
# kept under /workspace, which persists.
#
#   curl -fsSL https://raw.githubusercontent.com/Galanafai/godview/feat/boxes-ids-real-data/scripts/runpod/setup.sh | bash
#   source /workspace/godview.env
set -euo pipefail

WS=${WS:-/workspace}
BRANCH=${BRANCH:-feat/boxes-ids-real-data}
REPO_URL=${REPO_URL:-https://github.com/Galanafai/godview.git}
REPO_DIR=$WS/godview
DATA_DIR=$WS/data/v2v4real

SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO=sudo; fi

echo "== system packages"
export DEBIAN_FRONTEND=noninteractive
$SUDO apt-get update -y -qq
$SUDO apt-get install -y -qq --no-install-recommends \
  build-essential pkg-config git curl ca-certificates tmux unzip \
  python3 python3-venv python3-pip >/dev/null

echo "== environment file ($WS/godview.env)"
cat >"$WS/godview.env" <<EOF
export RUSTUP_HOME=$WS/.rustup
export CARGO_HOME=$WS/.cargo
export PATH=$WS/.cargo/bin:\$HOME/.local/bin:\$PATH
export GODVIEW_REPO=$REPO_DIR
export V2V4REAL_DIR=$DATA_DIR
EOF
grep -q "godview.env" "$HOME/.bashrc" 2>/dev/null || echo "source $WS/godview.env" >>"$HOME/.bashrc"
# shellcheck disable=SC1091
source "$WS/godview.env"

echo "== Rust (kept in $WS)"
if [ ! -x "$CARGO_HOME/bin/cargo" ]; then
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs |
    sh -s -- -y --profile minimal --no-modify-path
fi
rustup default stable >/dev/null

echo "== Claude Code"
if ! command -v claude >/dev/null; then
  curl -fsSL https://claude.ai/install.sh | bash
fi

echo "== repository ($BRANCH)"
if [ -d "$REPO_DIR/.git" ]; then
  git -C "$REPO_DIR" fetch -q origin
  git -C "$REPO_DIR" checkout -q "$BRANCH"
  git -C "$REPO_DIR" pull -q --ff-only origin "$BRANCH"
else
  git clone -q -b "$BRANCH" "$REPO_URL" "$REPO_DIR"
fi

echo "== Python environment for the converter ($WS/venv)"
if [ ! -x "$WS/venv/bin/python" ]; then python3 -m venv "$WS/venv"; fi
"$WS/venv/bin/pip" install -q --upgrade pip numpy pyyaml

echo "== data directory ($DATA_DIR)"
mkdir -p "$DATA_DIR"

echo "== release build (a few minutes the first time)"
(cd "$REPO_DIR" && cargo build -q --release -p godview_sim -p godview_node)

cat <<EOF

Setup done. Next:
  source $WS/godview.env
  cd $REPO_DIR && claude          # first time only: /login, then /exit
  tmux new -s godview
  claude remote-control --name "GodView Phase C"
  # detach with Ctrl-b then d; open the session at claude.ai/code
EOF
