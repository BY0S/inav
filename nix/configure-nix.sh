#!/usr/bin/env bash
set -euo pipefail

# Find script directory and repository root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
BUILD_DIR="$REPO_ROOT/build-nix"
SHELL_FILE="$REPO_ROOT/shell.nix"

# Sourcing the Nix daemon profile if Nix is not already in the PATH
if ! command -v nix-shell &> /dev/null; then
    if [ -e '/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh' ]; then
        . '/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh'
    else
        echo "❌ Nix is not installed or not running. Please restart your shell or check your Nix installation."
        exit 1
    fi
fi

echo "=========================================================="
echo "🔧 Configuring INAV build environment via Nix..."
echo "=========================================================="

# Run CMake configuration inside the nix-shell environment.
# Using Ninja as the build generator.
# Setting COMPILER_VERSION_CHECK=OFF allows using Nix's gcc-arm-embedded (v13.3)
# instead of triggering CMake to download an exact match of v13.2.1.
nix-shell "$SHELL_FILE" --run "
  mkdir -p \"$BUILD_DIR\" && \
  cd \"$BUILD_DIR\" && \
  cmake -GNinja -DCOMPILER_VERSION_CHECK=OFF \"$REPO_ROOT\"
"

echo "----------------------------------------------------------"
echo "✅ Configuration successful!"
echo "👉 You can now compile the project by running: ./nix/build-nix.sh <TARGET>"
echo "👉 Examples of targets: MATEKF722SE, SITL"
echo "=========================================================="
