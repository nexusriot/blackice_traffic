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
- **⌖ MY LOCATION** button — geolocates you in the background and flies the map to your position
- **Rays** from you to remote contacts  
- **Snapshot button** on every tab — save the current view as PNG
- **▣ SAVE CONFIG** toggle — optional, off by default: remembers window geometry, open tab and layout between runs
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

## Configuration (optional)

Saving is **off by default** — out of the box the app writes nothing and always
starts from its defaults.

Press **▣ SAVE CONFIG** in the top bar of the BLACK ICE tab to turn it on. From
then on the following is restored on the next launch:

- window geometry and maximized state (re-centered if the saved monitor is gone)
- the selected tab
- the interface picked on the BLACK ICE and STATS tabs, and the STATS time window
- the CONTACTS column widths, sort column/direction and filter text
- the MAP splitter position and the *Show my location + rays* checkbox

State lives in a plain, hand-editable INI file:

```
~/.config/blackice/blackice_traffic.ini
```

Turning the toggle off deletes every stored key (the flag itself stays, so the
app knows not to restore). Deleting the file by hand has the same effect.

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
