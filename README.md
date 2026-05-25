# BLACK ICE - Traffic Visualizer

Minimal hacker‑style network traffic visualizer for Linux, built with **Python + PyQt6**.

Green‑phosphor UI, scanlines, matrix rain, live bandwidth waveform, and a world map
showing your remote connections — with optional **"my location"** marker and rays.




WARNING: this __not__ a hack tool, just a educational project made just 4fun

![Traffic](resources/traffic.png)
![Map](resources/map.png)

---
## Features

- Old‑school **BLACK ICE / cyberpunk** interface  :) 
- Live RX/TX bandwidth (per interface or total)  
- Event console (connections discovered in real time)  
- **CONTACTS tab** — sortable/filterable table of remote endpoints (with CSV export)
- World map (Leaflet via QtWebEngine)  
- Optional **My Location** marker
- **Rays** from you to remote contacts  
- **Snapshot button** on every tab — save the current view as PNG
- Offline GeoIP support (MaxMind GeoLite2)  
- No packet sniffing by default (psutil‑based, safe mode)
---

## Requirements

- Linux/Windows*
- Python 3.9+
- PyQt6
- psutil
- PyQt6-WebEngine (for MAP tab)

Optional:
- `geoip2` + `GeoLite2-City.mmdb` for offline geolocation

---

## Install

```bash
python3 -m venv .venv
source .venv/bin/activate

pip install PyQt6 psutil PyQt6-WebEngine
# optional offline GeoIP
pip install geoip2
```

---

## Run

```bash
python3 blackice_traffic.py
```

With offline GeoIP:

```bash
GEOIP_DB=/path/to/GeoLite2-City.mmdb python3 blackice_traffic.py
```

Building deb package
------------- 

Install required packages:
```
sudo apt-get install git devscripts build-essential lintian upx-ucl
```
Run build:
```
./build_deb.sh
```

Building linux binary (PyInstaller) 
------------- 
Run build:
```
./build_linux_bin.sh
```

Building Windows binary (PyInstaller) 
------------- 
Run build:
```
./build_win.cmd
```

---

## Map notes

- Private / loopback / Docker IPs are labeled as **LOCAL**
- Connections without coordinates are not placed on (0,0)
- "My location" is optional and can be toggled in the MAP tab

---

## Disclaimer

This tool is **visualization only**.
It does not block traffic, inject packets, or perform intrusion (not yet).
Use responsibly ;).

---

> BLACK ICE DEFENDER — observe the net.
