SHELL := /bin/bash

PIO ?= $(shell command -v pio 2>/dev/null || command -v platformio 2>/dev/null || { test -x "$$HOME/.platformio/penv/bin/pio" && echo "$$HOME/.platformio/penv/bin/pio"; })
ENV ?= blackpill_f401cc
MONITOR_BAUD ?= 115200

.PHONY: help build upload monitor clean cleanall check-tools check-python size

help:
	@echo "FloatSat embedded firmware"
	@echo ""
	@echo "Targets:"
	@echo "  make build        Build firmware with PlatformIO"
	@echo "  make upload       Build and upload over DFU"
	@echo "  make monitor      Open serial monitor at $(MONITOR_BAUD) baud"
	@echo "  make clean        Remove PlatformIO build output"
	@echo "  make cleanall     Remove build output and downloaded project libraries"
	@echo "  make check-python Validate Python helper scripts"
	@echo "  make size         Print firmware memory usage"
	@echo ""
	@echo "Variables:"
	@echo "  ENV=$(ENV)"
	@echo "  MONITOR_BAUD=$(MONITOR_BAUD)"
	@echo "  PIO=$(PIO)"

check-tools:
	@if [ -z "$(PIO)" ]; then \
		echo "PlatformIO was not found. Install it or set PIO=/path/to/pio"; \
		exit 1; \
	fi

build: check-tools
	$(PIO) run -e $(ENV)

upload: check-tools
	$(PIO) run -e $(ENV) -t upload

monitor: check-tools
	$(PIO) device monitor -b $(MONITOR_BAUD)

clean: check-tools
	$(PIO) run -e $(ENV) -t clean

cleanall:
	rm -rf .pio

check-python:
	python3 -m py_compile tools/attitude_viewer.py tools/pid_gain_calculator.py

size: check-tools
	$(PIO) run -e $(ENV) -t size
