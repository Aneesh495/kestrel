.PHONY: all clean test sim vm serve-web help

BUILD_DIR ?= build

all:
	@mkdir -p $(BUILD_DIR)
	@cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=Release
	@cmake --build $(BUILD_DIR) -j

test: all
	@echo "==================================================================="
	@echo " RUNNING FULL KESTREL AUTOMATED TEST SUITE (10 SUITES)"
	@echo "==================================================================="
	@ctest --test-dir $(BUILD_DIR) --output-on-failure

sim: all
	@./$(BUILD_DIR)/kestrel-sim --steps 22500

vm: all
	@./$(BUILD_DIR)/kestrel-vm Content/Blueprints

serve-web:
	@echo "Starting Kestrel Ground Control Station Web Telemetry at http://localhost:8000"
	@python3 -m http.server 8000 --directory web

clean:
	@rm -rf $(BUILD_DIR) telemetry.csv telemetry.json
	@echo "Cleaned build artifacts."

help:
	@echo "Kestrel Aerial Robotics Build Automation"
	@echo "Targets:"
	@echo "  make all       : Build core libraries, VM, simulator, and test binaries"
	@echo "  make test      : Execute all 10 automated test suites with CTest"
	@echo "  make sim       : Run the 500 Hz SITL autonomous flight simulator"
	@echo "  make vm        : Parse and validate all 33k+ lines of Blueprint graphs"
	@echo "  make serve-web : Launch the local telemetry HUD web dashboard"
	@echo "  make clean     : Remove compiled binaries and build cache"
