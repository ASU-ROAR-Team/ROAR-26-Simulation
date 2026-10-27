#!/bin/bash
set -e

# Establish local dependencies installation path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVSTACK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$NAVSTACK_DIR"
DEPS_DIR="$WORKSPACE/deps"
INSTALL_PREFIX="$WORKSPACE/local_deps"

mkdir -p "$DEPS_DIR"
mkdir -p "$INSTALL_PREFIX"

echo "=== Installing XTL (0.7.7) ==="
if [ ! -d "$DEPS_DIR/xtl" ]; then
  git clone --depth 1 --branch 0.7.7 https://github.com/xtensor-stack/xtl.git "$DEPS_DIR/xtl"
fi
mkdir -p "$DEPS_DIR/xtl/build"
cd "$DEPS_DIR/xtl/build"
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
make install

echo "=== Installing XSimd (11.1.0) ==="
if [ ! -d "$DEPS_DIR/xsimd" ]; then
  git clone --depth 1 --branch 11.1.0 https://github.com/xtensor-stack/xsimd.git "$DEPS_DIR/xsimd"
fi
mkdir -p "$DEPS_DIR/xsimd/build"
cd "$DEPS_DIR/xsimd/build"
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
make install

echo "=== Installing XTensor (0.24.7) ==="
if [ ! -d "$DEPS_DIR/xtensor" ]; then
  git clone --depth 1 --branch 0.24.7 https://github.com/xtensor-stack/xtensor.git "$DEPS_DIR/xtensor"
fi
mkdir -p "$DEPS_DIR/xtensor/build"
cd "$DEPS_DIR/xtensor/build"
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" -DCMAKE_PREFIX_PATH="$INSTALL_PREFIX" ..
make install

echo "=== Dependencies installed successfully under $INSTALL_PREFIX ==="
