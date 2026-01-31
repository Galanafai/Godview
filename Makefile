# GodView Makefile
#
# Quick commands for development and testing.

.PHONY: build test dst dst-quick dst-stress dst-all clean

# Default target
all: build test

# Build all crates
build:
	cargo build --release

# Run all unit tests
test:
	cargo test -p godview_env
	cargo test -p godview_core
	cargo test -p godview_sim

# DST: Quick single-seed run
dst:
	cargo run --release -p godview_sim -- --seed 42 --scenario all --duration 10

# DST: Quick 10-seed run
dst-quick:
	cargo run --release -p godview_sim -- --seeds 10 --scenario all --duration 10

# DST: Stress test (100 seeds)
dst-stress:
	cargo run --release -p godview_sim -- --seeds 100 --scenario all --duration 30

# DST: All scenarios individually
dst-all:
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "Running all DST scenarios..."
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	cargo run --release -p godview_sim -- --seed 42 --scenario time_warp
	cargo run --release -p godview_sim -- --seed 42 --scenario split_brain
	cargo run --release -p godview_sim -- --seed 42 --scenario byzantine
	cargo run --release -p godview_sim -- --seed 42 --scenario flash_mob
	cargo run --release -p godview_sim -- --seed 42 --scenario slow_loris
	cargo run --release -p godview_sim -- --seed 42 --scenario swarm
	cargo run --release -p godview_sim -- --seed 42 --scenario adaptive_swarm
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "✅ All scenarios passed!"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# DST: Specific scenario (usage: make dst-scenario SCENARIO=swarm)
dst-scenario:
	cargo run --release -p godview_sim -- --seed 42 --scenario $(SCENARIO) --duration 30

# DST: Adaptive swarm (learning agents)
dst-adaptive:
	cargo run --release -p godview_sim -- --seed 42 --scenario adaptive_swarm --duration 30

# Clean build artifacts
clean:
	cargo clean

# Help
help:
	@echo "GodView Makefile Commands:"
	@echo ""
	@echo "  make build       - Build all crates (release)"
	@echo "  make test        - Run all unit tests"
	@echo ""
	@echo "  make dst         - Quick single-seed DST run"
	@echo "  make dst-quick   - Quick 10-seed DST run"
	@echo "  make dst-stress  - Stress test (100 seeds)"
	@echo "  make dst-all     - Run all scenarios individually"
	@echo "  make dst-adaptive - Run adaptive swarm scenario"
	@echo ""
	@echo "  SCENARIO=<name> make dst-scenario - Run specific scenario"
	@echo ""
	@echo "  make clean       - Clean build artifacts"
