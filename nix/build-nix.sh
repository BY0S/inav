#!/usr/bin/env bash
set -euo pipefail

# Find script directory and repository root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
BUILD_DIR="$REPO_ROOT/build-nix"
SHELL_FILE="$REPO_ROOT/shell.nix"

# Sourcing Nix daemon profile if needed
if ! command -v nix-shell &> /dev/null; then
    if [ -e '/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh' ]; then
        . '/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh'
    else
        echo "❌ Nix is not installed or not running."
        exit 1
    fi
fi

if [ ! -d "$BUILD_DIR" ] || [ ! -f "$BUILD_DIR/build.ninja" ]; then
    echo "⚠️ Build directory not configured yet. Running configuration script..."
    "$SCRIPT_DIR/configure-nix.sh"
fi

if [[ $# == 0 ]]; then
    echo "Usage: ./nix/build-nix.sh <TARGET>"
    echo "Available targets list:"
    nix-shell "$SHELL_FILE" --run "ninja -C \"$BUILD_DIR\" -t targets | grep -E '^[^:]+' | cut -d: -f1 | sort | uniq" || true
    exit 1
fi

TARGET="$1"

echo "=========================================================="
echo "🚀 Building target [$TARGET] via Nix..."
echo "=========================================================="

nix-shell "$SHELL_FILE" --run "ninja -C \"$BUILD_DIR\" \"$TARGET\""

echo "----------------------------------------------------------"
echo "✅ Build completed!"
if [ -d "$BUILD_DIR" ]; then
    echo "Output files can be found in the '$BUILD_DIR' directory."
fi
echo "=========================================================="
