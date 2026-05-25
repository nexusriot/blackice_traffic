VERSION := 0.3.0
HOST_ARCH := $(shell dpkg --print-architecture 2>/dev/null || uname -m)

.PHONY: all bin deb deb-amd64 deb-arm64 clean help

help:
	@echo "blackice_traffic build targets:"
	@echo "  make bin         - build standalone binary via PyInstaller (host arch: $(HOST_ARCH))"
	@echo "  make deb         - build .deb for host arch ($(HOST_ARCH))"
	@echo "  make deb-amd64   - build amd64 .deb (must run on amd64)"
	@echo "  make deb-arm64   - build arm64 .deb (must run on arm64)"
	@echo "  make clean       - remove build artifacts"

bin: blackice_traffic

blackice_traffic: blackice_traffic.py requirements.txt
	./build_linux_bin.sh
	cp dist/blackice_traffic ./blackice_traffic

deb: deb-$(HOST_ARCH)

deb-amd64: bin
	@if [ "$(HOST_ARCH)" != "amd64" ]; then \
		echo "ERROR: deb-amd64 must be built on an amd64 host (current: $(HOST_ARCH))"; exit 1; \
	fi
	./build_deb.sh

deb-arm64: bin
	@if [ "$(HOST_ARCH)" != "arm64" ]; then \
		echo "ERROR: deb-arm64 must be built on an arm64 host (current: $(HOST_ARCH))"; exit 1; \
	fi
	./build_deb_arm64.sh

clean:
	rm -rf build/ dist/ __pycache__/
	rm -f blackice-traffic_*.deb
