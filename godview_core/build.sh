#!/bin/bash

# GodView Core v3 - Build and Test Script

set -e

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW CORE V3 - BUILD SCRIPT           ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo "❌ Rust/Cargo not found!"
    echo ""
    echo "Install Rust:"
    echo "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    echo ""
    echo "Or use the install script from the parent directory:"
    echo "  cd .. && ./install_dependencies.sh"
    exit 1
fi

echo "✅ Rust found: $(rustc --version)"
echo ""

# Build the library
echo "--- [1/4] BUILDING LIBRARY (RELEASE MODE) ---"
cargo build --release
echo ""

# Run tests
echo "--- [2/4] RUNNING TESTS ---"
cargo test
echo ""

# Check formatting
echo "--- [3/4] CHECKING CODE FORMATTING ---"
cargo fmt -- --check || {
    echo "⚠️  Code formatting issues found. Run 'cargo fmt' to fix."
}
echo ""

# Run clippy
echo "--- [4/4] RUNNING CLIPPY (LINTER) ---"
cargo clippy -- -D warnings || {
    echo "⚠️  Clippy warnings found. Review and fix."
}
echo ""

echo "╔════════════════════════════════════════════╗"
echo "║          BUILD COMPLETE                    ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "Library built at: target/release/libgodview_core.rlib"
echo ""
echo "Next steps:"
echo "  1. Run 'cargo doc --open' to view documentation"
echo "  2. Run 'cargo test -- --nocapture' for verbose test output"
echo "  3. Integrate into your project with:"
echo "     [dependencies]"
echo "     godview_core = { path = \"../godview_core\" }"
echo ""
