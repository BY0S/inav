{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  name = "inav-dev-shell";

  # Development tools and compiler dependencies
  buildInputs = with pkgs; [
    git
    cmake
    ninja
    ruby
    python3
    python3Packages.pyyaml
    gcc-arm-embedded-13
  ];

  shellHook = ''
    echo "==========================================="
    echo "🚀 INAV Nix Development Shell Activated!"
    echo "  - Compiler: $(which arm-none-eabi-gcc)"
    echo "  - CMake:    $(cmake --version | head -n 1)"
    echo "  - Ruby:     $(ruby --version)"
    echo "==========================================="
  '';
}
