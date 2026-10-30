SKETCH     := code/GnimuCAN
FQBN       := esp32:esp32:XIAO_ESP32C6
CORE       := esp32:esp32@3.3.12
BUILD_DIR  := build
BAUD       := 115200
PORT       ?= $(shell arduino-cli board list | awk '/usbmodem/ {print $$1; exit}')

.PHONY: build flash monitor clean setup check-port

build:
	arduino-cli compile --fqbn $(FQBN) --warnings all --build-path $(BUILD_DIR) $(SKETCH)

flash: build check-port
	arduino-cli upload --fqbn $(FQBN) --input-dir $(BUILD_DIR) --port $(PORT) $(SKETCH)

monitor: check-port
	arduino-cli monitor --port $(PORT) --config baudrate=$(BAUD)

clean:
	rm -rf $(BUILD_DIR)

# One-time toolchain install (pinned core version).
setup:
	arduino-cli core update-index
	arduino-cli core install $(CORE)

check-port:
	@test -n "$(PORT)" || (echo "No XIAO found; plug it in or pass PORT=/dev/cu.usbmodemXXXX" && exit 1)
