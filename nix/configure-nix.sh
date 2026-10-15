#!/usr/bin/env bash
set -euo pipefail

# Find script directory and repository root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
SHELL_FILE="$REPO_ROOT/shell.nix"

SITL_MODE="OFF"
if [[ "${1:-}" == "--sitl" ]] || [[ "${1:-}" == "SITL" ]]; then
    SITL_MODE="ON"
    BUILD_DIR="$REPO_ROOT/build-nix-sitl"
else
    BUILD_DIR="$REPO_ROOT/build-nix"
fi

# Sourcing the Nix profile if Nix is not already in the PATH
if ! command -v nix-shell &> /dev/null; then
    for profile in \
        "$HOME/.nix-profile/etc/profile.d/nix.sh" \
        "/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh" \
        "/etc/profile.d/nix.sh"; do
        if [ -e "$profile" ]; then
            . "$profile"
            break
        fi
    done
fi

if ! command -v nix-shell &> /dev/null; then
    echo "❌ Nix is not installed or nix-shell is not in your PATH. Please check your installation."
    exit 1
fi

echo "=========================================================="
echo "🔧 Configuring INAV build environment via Nix (SITL=$SITL_MODE)..."
echo "=========================================================="

# Run CMake configuration inside the nix-shell environment.
# Using Ninja as the build generator.
# Setting COMPILER_VERSION_CHECK=OFF allows using Nix's gcc-arm-embedded (v13.3)
# or host gcc for SITL instead of triggering CMake to download v13.2.1.
nix-shell "$SHELL_FILE" --run "
  mkdir -p \"$BUILD_DIR\" && \
  cd \"$BUILD_DIR\" && \
  cmake -GNinja -DSITL=$SITL_MODE -DCOMPILER_VERSION_CHECK=OFF \"$REPO_ROOT\"
"

echo "----------------------------------------------------------"
echo "✅ Configuration successful! (Build dir: $BUILD_DIR)"
echo "👉 You can now compile the project by running: ./nix/build-nix.sh <TARGET>"
echo "👉 Examples of targets: SPEEDYBEEF405V5, MATEKF722SE, SITL"
echo "=========================================================="
