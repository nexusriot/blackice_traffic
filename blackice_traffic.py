#!/usr/bin/env python3
import os
import sys
import math
import time
import json
import random
import socket
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import urllib.request
import urllib.error
import psutil
import ipaddress
from PyQt5 import QtCore, QtGui, QtWidgets

APP_NAME = "BLACK ICE"
APP_VERSION = "0.3.0"
APP_BUILD = "alpha"


# Map tab requires QtWebEngine (separate pkg)
try:
    from PyQt5 import QtWebEngineWidgets
    HAVE_WEBENGINE = True
except Exception:
    HAVE_WEBENGINE = False

# Optional offline GeoIP (MaxMind mmdb)
HAVE_GEOIP = False
_geoip_reader = None
try:
    import geoip2.database
    HAVE_GEOIP = True
except Exception:
    HAVE_GEOIP = False


PHOSPHOR = QtGui.QColor("#00ff66")
PHOSPHOR_DIM = QtGui.QColor("#00aa44")
BG = QtGui.QColor("#050a06")
GRID = QtGui.QColor("#0b2a12")
RED = QtGui.QColor("#ff3355")
AMBER = QtGui.QColor("#ffcc33")


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def human_bps(bps: float) -> str:
    units = ["b/s", "Kb/s", "Mb/s", "Gb/s", "Tb/s"]
    v = float(bps)
    for u in units:
        if v < 1000.0:
            return f"{v:,.1f} {u}"
        v /= 1000.0
    return f"{v:,.1f} Pb/s"


def human_bytes(n: float) -> str:
    units = ["B", "KB", "MB", "GB", "TB", "PB"]
    v = float(n)
    for u in units:
        if v < 1024.0:
            return f"{v:,.1f} {u}"
        v /= 1024.0
    return f"{v:,.1f} EB"


@dataclass
class ConnPoint:
    ip: str
    port: int
    proto: str
    lat: float
    lon: float
    label: str
    ts: float


class HackerFont:
    @staticmethod
    def mono(size=11, bold=False):
        f = QtGui.QFont("DejaVu Sans Mono", size)
        f.setStyleHint(QtGui.QFont.Monospace)
        f.setBold(bold)
        return f


class ScanlinesOverlay(QtWidgets.QWidget):
    """Transparent scanlines + subtle flicker overlay."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self._phase = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

    def _tick(self):
        self._phase += 0.10
        self.update()

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, False)

        w, h = self.width(), self.height()
        # scanlines
        p.setOpacity(0.18)
        pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 255))
        for y in range(0, h, 3):
            alpha = 35 + int(20 * (0.5 + 0.5 * math.sin((y * 0.08) + self._phase)))
            c = QtGui.QColor(0, 0, 0, alpha)
            pen.setColor(c)
            p.setPen(pen)
            p.drawLine(0, y, w, y)

        # vignette
        p.setOpacity(0.25)
        grad = QtGui.QRadialGradient(w * 0.5, h * 0.5, max(w, h) * 0.75)
        grad.setColorAt(0.0, QtGui.QColor(0, 0, 0, 0))
        grad.setColorAt(1.0, QtGui.QColor(0, 0, 0, 220))
        p.fillRect(self.rect(), grad)

        # flicker
        p.setOpacity(0.04 + random.random() * 0.03)
        p.fillRect(self.rect(), QtGui.QColor(255, 255, 255, 255))


class MatrixRain(QtWidgets.QWidget):
    """Simple matrix-rain background."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)

        self.cols = []
        self.char_set = list("01abcdef#$%&*+<>/\\|[]{}()~")
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def resizeEvent(self, e):
        self._init_cols()

    def _init_cols(self):
        w = max(1, self.width())
        col_w = 12
        n = max(8, w // col_w)
        self.cols = []
        for i in range(n):
            self.cols.append({
                "x": i * col_w + random.randint(0, 3),
                "y": random.randint(-self.height(), 0),
                "speed": random.randint(10, 28),
                "len": random.randint(10, 26),
            })

    def _tick(self):
        if not self.cols:
            self._init_cols()
        for c in self.cols:
            c["y"] += c["speed"]
            if c["y"] - (c["len"] * 14) > self.height():
                c["y"] = random.randint(-self.height(), 0)
                c["speed"] = random.randint(10, 28)
                c["len"] = random.randint(10, 26)
        self.update()

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, False)
        p.setFont(HackerFont.mono(10))

        # very subtle
        p.setOpacity(0.20)
        for c in self.cols:
            x = c["x"]
            y = c["y"]
            for i in range(c["len"]):
                ch = random.choice(self.char_set)
                yy = y - i * 14
                if 0 <= yy <= self.height():
                    # head brighter
                    if i == 0:
                        p.setPen(QtGui.QPen(PHOSPHOR))
                        p.setOpacity(0.30)
                    else:
                        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
                        p.setOpacity(0.16)
                    p.drawText(x, yy, ch)


class Oscilloscope(QtWidgets.QWidget):
    """Hacker-style bandwidth waveform (RX/TX)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(180)
        self._rx = [0.0] * 240
        self._tx = [0.0] * 240
        self._max = 1.0

    def push(self, rx_bps: float, tx_bps: float):
        self._rx.pop(0); self._rx.append(rx_bps)
        self._tx.pop(0); self._tx.append(tx_bps)
        self._max = max(1.0, max(max(self._rx), max(self._tx)) * 1.10)
        self.update()

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), BG)

        w, h = self.width(), self.height()

        # grid
        p.setPen(QtGui.QPen(GRID, 1))
        for x in range(0, w, 40):
            p.drawLine(x, 0, x, h)
        for y in range(0, h, 20):
            p.drawLine(0, y, w, y)

        # axes labels
        p.setFont(HackerFont.mono(9))
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        p.drawText(10, 16, "BANDWIDTH WAVEFORM (RX/TX)")

        def poly(values):
            pts = []
            n = len(values)
            for i, v in enumerate(values):
                x = (i / (n - 1)) * (w - 1)
                y = h - 1 - (v / self._max) * (h - 26)
                pts.append(QtCore.QPointF(x, y))
            return QtGui.QPolygonF(pts)

        # RX line
        p.setPen(QtGui.QPen(PHOSPHOR, 2))
        p.setOpacity(0.85)
        p.drawPolyline(poly(self._rx))

        # TX line (amber)
        p.setPen(QtGui.QPen(AMBER, 2))
        p.setOpacity(0.75)
        p.drawPolyline(poly(self._tx))

        # max scale
        p.setOpacity(1.0)
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        p.drawText(w - 190, 16, f"scale max: {human_bps(self._max)}")


class TrafficPoller(QtCore.QThread):
    traffic = QtCore.pyqtSignal(dict)

    def __init__(self, interval=1.0, parent=None):
        super().__init__(parent)
        self.interval = interval
        self._stop = False
        self._prev = None

    def stop(self):
        self._stop = True

    def run(self):
        self._prev = psutil.net_io_counters(pernic=True)
        prev_t = time.time()
        while not self._stop:
            time.sleep(self.interval)
            now = psutil.net_io_counters(pernic=True)
            now_t = time.time()
            dt = max(0.2, now_t - prev_t)

            snap = {}
            total_rx_bps = 0.0
            total_tx_bps = 0.0

            for nic, cnt in now.items():
                if nic not in self._prev:
                    continue
                p = self._prev[nic]
                rx_bps = (cnt.bytes_recv - p.bytes_recv) * 8.0 / dt
                tx_bps = (cnt.bytes_sent - p.bytes_sent) * 8.0 / dt
                snap[nic] = {
                    "rx_bps": max(0.0, rx_bps),
                    "tx_bps": max(0.0, tx_bps),
                    "rx_total": cnt.bytes_recv,
                    "tx_total": cnt.bytes_sent,
                    "pkts_in": cnt.packets_recv,
                    "pkts_out": cnt.packets_sent,
                }
                total_rx_bps += max(0.0, rx_bps)
                total_tx_bps += max(0.0, tx_bps)

            snap["_totals"] = {"rx_bps": total_rx_bps, "tx_bps": total_tx_bps}
            self._prev = now
            prev_t = now_t
            self.traffic.emit(snap)


class ConnScanner(QtCore.QThread):
    points = QtCore.pyqtSignal(list)
    event = QtCore.pyqtSignal(str)

    def __init__(self, interval=3.0, parent=None):
        super().__init__(parent)
        self.interval = interval
        self._stop = False
        self._seen: Dict[str, float] = {}

    def stop(self):
        self._stop = True

    def _normalize_ip(self, ip: str) -> str:
        # Normalize IPv4-mapped IPv6 like ::ffff:127.0.0.1 → 127.0.0.1
        if ip.startswith("::ffff:"):
            v4 = ip.split("::ffff:", 1)[1]
            # Sometimes it can be hex-ish, but psutil normally gives dotted quad.
            return v4
        return ip

    def _is_privateish(self, ip: str) -> bool:
        try:
            addr = ipaddress.ip_address(ip)
            return (
                    addr.is_private
                    or addr.is_loopback
                    or addr.is_link_local
                    or addr.is_multicast
                    or addr.is_reserved
            )
        except Exception:
            return True  # if we can't parse, treat as non-public

    def _geo_lookup(self, ip: str) -> Tuple[Optional[float], Optional[float], str]:
        ip = self._normalize_ip(ip)

        # Classify local/private early
        if self._is_privateish(ip):
            # Better label than "GeoIP unavailable"
            try:
                addr = ipaddress.ip_address(ip)
                if addr.is_loopback:
                    return None, None, "LOCAL LOOPBACK"
                if addr.is_private:
                    return None, None, "LOCAL RFC1918 / PRIVATE"
                if addr.is_link_local:
                    return None, None, "LOCAL LINK-LOCAL"
            except Exception:
                pass
            return None, None, "LOCAL / NON-PUBLIC"

        # Offline GeoIP if available
        global _geoip_reader
        if HAVE_GEOIP and _geoip_reader is not None:
            try:
                r = _geoip_reader.city(ip)

                country = (r.country.iso_code or "").strip()
                city = (r.city.name or "").strip()
                region = ""
                if r.subdivisions and len(r.subdivisions) > 0:
                    region = (r.subdivisions.most_specific.name or "").strip()

                lat = r.location.latitude
                lon = r.location.longitude

                # Build best-effort label
                parts = [p for p in [city, region, country] if p]
                label = " ".join(parts) if parts else (country or "Unknown")

                # Some records have no coords
                if lat is None or lon is None:
                    return None, None, label

                return float(lat), float(lon), label
            except Exception:
                pass

        return None, None, "GeoIP unavailable"

    def _scan_psutil(self) -> List[ConnPoint]:
        out: List[ConnPoint] = []
        try:
            conns = psutil.net_connections(kind="inet")
        except Exception as e:
            self.event.emit(f"[!] net_connections failed: {e}")
            return out

        now = time.time()
        for c in conns:
            if not c.raddr:
                continue
            ip = c.raddr.ip
            port = int(c.raddr.port)
            proto = "tcp" if c.type == socket.SOCK_STREAM else "udp"
            key = f"{proto}:{ip}:{port}"
            # only public-ish (still allow private if you want; skip localhost)
            if ip.startswith("127.") or ip == "::1":
                continue
            # rate-limit duplicates
            if key in self._seen and (now - self._seen[key]) < 20:
                continue
            self._seen[key] = now

            norm_ip = self._normalize_ip(ip)
            lat, lon, where = self._geo_lookup(norm_ip)
            label = f"{norm_ip}:{port} ({proto}) — {where}"
            out.append(ConnPoint(ip=norm_ip, port=port, proto=proto,
                                 lat=lat or 0.0, lon=lon or 0.0, label=label, ts=now))

        return out

    def run(self):
        while not self._stop:
            pts = self._scan_psutil()
            if pts:
                self.points.emit([p.__dict__ for p in pts])
                self.event.emit(f"[+] contacts detected: {len(pts)}")
            time.sleep(self.interval)


class ConsoleLog(QtWidgets.QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(5000)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setFont(HackerFont.mono(10))
        self.setStyleSheet(
            "QPlainTextEdit { background: #050a06; color: #00ff66; selection-background-color: #0b2a12; }"
        )

    def push(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.appendPlainText(f"{ts} {msg}")


class BlackIceDashboard(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.Window, BG)
        self.setPalette(pal)

        self.matrix = MatrixRain(self)
        self.scan = ScanlinesOverlay(self)

        self.title = QtWidgets.QLabel("BLACK ICE  — TRAFFIC SENTINEL")
        self.version = QtWidgets.QLabel(f"v{APP_VERSION} · {APP_BUILD}")
        self.version.setFont(HackerFont.mono(9))
        self.version.setStyleSheet("color:#00aa44;")
        self.title.setFont(HackerFont.mono(16, bold=True))
        self.title.setStyleSheet("color: #00ff66; letter-spacing: 1px;")

        self.sub = QtWidgets.QLabel("STATUS: ARMED  |  MODE: PASSIVE SNIFF  |  SHIELD: SYN-FILTER (SIM)")
        self.sub.setFont(HackerFont.mono(10))
        self.sub.setStyleSheet("color: #00aa44;")

        self.scope = Oscilloscope(self)

        self.iface = QtWidgets.QComboBox()
        self.iface.setFont(HackerFont.mono(10))
        self.iface.setStyleSheet(
            "QComboBox { background: #07100a; color:#00ff66; border: 1px solid #0b2a12; padding: 4px; }"
            "QAbstractItemView { background: #07100a; color:#00ff66; selection-background-color:#0b2a12; }"
        )

        self.rx_lbl = QtWidgets.QLabel("RX: 0 b/s")
        self.tx_lbl = QtWidgets.QLabel("TX: 0 b/s")
        self.rx_lbl.setFont(HackerFont.mono(12, bold=True))
        self.tx_lbl.setFont(HackerFont.mono(12, bold=True))
        self.rx_lbl.setStyleSheet("color: #00ff66;")
        self.tx_lbl.setStyleSheet("color: #ffcc33;")

        self.totals_lbl = QtWidgets.QLabel("TOTALS: RX 0 B  |  TX 0 B")
        self.totals_lbl.setFont(HackerFont.mono(10))
        self.totals_lbl.setStyleSheet("color: #00aa44;")

        self.log = ConsoleLog()

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.title)
        top.addSpacing(12)
        top.addWidget(self.version)
        top.addStretch(1)
        top.addWidget(QtWidgets.QLabel("INTERFACE:"))
        lbl = top.itemAt(top.count()-1).widget()
        lbl.setFont(HackerFont.mono(10))
        lbl.setStyleSheet("color:#00aa44;")
        top.addWidget(self.iface)

        meters = QtWidgets.QHBoxLayout()
        meters.addWidget(self.rx_lbl)
        meters.addSpacing(20)
        meters.addWidget(self.tx_lbl)
        meters.addStretch(1)
        meters.addWidget(self.totals_lbl)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addWidget(self.sub)
        layout.addSpacing(6)
        layout.addLayout(meters)
        layout.addWidget(self.scope, 1)
        layout.addWidget(QtWidgets.QLabel("EVENT LOG:"))
        el = layout.itemAt(layout.count()-1).widget()
        el.setFont(HackerFont.mono(10))
        el.setStyleSheet("color:#00aa44;")
        layout.addWidget(self.log, 1)

        self._last_snap = {}
        self._populate_ifaces()

    def _populate_ifaces(self):
        self.iface.clear()
        nics = list(psutil.net_io_counters(pernic=True).keys())
        self.iface.addItem("ALL")
        for n in nics:
            self.iface.addItem(n)

    def resizeEvent(self, e):
        self.matrix.setGeometry(self.rect())
        self.scan.setGeometry(self.rect())
        super().resizeEvent(e)

    def set_event(self, msg: str):
        self.log.push(msg)

    def update_traffic(self, snap: dict):
        self._last_snap = snap
        sel = self.iface.currentText()
        if sel == "ALL":
            rx = snap.get("_totals", {}).get("rx_bps", 0.0)
            tx = snap.get("_totals", {}).get("tx_bps", 0.0)
            self.scope.push(rx, tx)
            self.rx_lbl.setText(f"RX: {human_bps(rx)}")
            self.tx_lbl.setText(f"TX: {human_bps(tx)}")
            # totals (sum totals from all)
            rx_total = sum(v.get("rx_total", 0) for k, v in snap.items() if k != "_totals")
            tx_total = sum(v.get("tx_total", 0) for k, v in snap.items() if k != "_totals")
            self.totals_lbl.setText(f"TOTALS: RX {human_bytes(rx_total)}  |  TX {human_bytes(tx_total)}")
        else:
            v = snap.get(sel)
            if not v:
                self._populate_ifaces()
                return
            rx = v["rx_bps"]; tx = v["tx_bps"]
            self.scope.push(rx, tx)
            self.rx_lbl.setText(f"RX: {human_bps(rx)}")
            self.tx_lbl.setText(f"TX: {human_bps(tx)}")
            self.totals_lbl.setText(
                f"{sel}: RX {human_bytes(v['rx_total'])}  |  TX {human_bytes(v['tx_total'])}  |  PKTS {v['pkts_in']}/{v['pkts_out']}"
            )


LEAFLET_HTML = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>BLACK ICE MAP</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <style>
    html, body { height:100%; margin:0; background:#050a06; }
    #map { height:100%; }
    .leaflet-popup-content-wrapper, .leaflet-popup-tip {
      background:#07100a; color:#00ff66; border:1px solid #0b2a12;
      font-family: "DejaVu Sans Mono", monospace;
    }
    .leaflet-control-attribution { display:none; }
  </style>
</head>
<body>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
  const map = L.map('map', { zoomControl: true }).setView([20, 0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

  const markers = new Map();

  // "Me" marker + rays
  let me = null;            // {lat, lon, label}
  let meMarker = null;
  let rayLayer = L.layerGroup().addTo(map);

  function setMyLocation(obj) {
    // obj: {lat, lon, label}
    if (!obj || obj.lat == null || obj.lon == null) return;
    me = { lat: obj.lat, lon: obj.lon, label: obj.label || "ME" };

    if (!meMarker) {
      meMarker = L.circleMarker([me.lat, me.lon], {
        radius: 8,
        weight: 2,
        color: "#ff9900",
        fillColor: "#ff9900",
        fillOpacity: 0.35
      }).addTo(map);
      meMarker.bindPopup(me.label);
    } else {
      meMarker.setLatLng([me.lat, me.lon]);
      meMarker.setPopupContent(me.label);
    }
  }

  function clearRays() {
    rayLayer.clearLayers();
  }

  function redrawRays() {
    clearRays();
    if (!me) return;

    // draw rays to every marker we have
    for (const [key, m] of markers.entries()) {
      const ll = m.getLatLng();
      // ignore unknowns (0,0) if they slip in
      if (Math.abs(ll.lat) < 1e-6 && Math.abs(ll.lng) < 1e-6) continue;

      const line = L.polyline([[me.lat, me.lon], [ll.lat, ll.lng]], {
        color: "#ff9900",
        weight: 1.6,
        opacity: 0.55
      });
      line.addTo(rayLayer);
    }
  }

  function upsertPoints(points) {
    for (const p of points) {
      const key = `${p.proto}:${p.ip}:${p.port}`;
      const lat = p.lat || 0;
      const lon = p.lon || 0;
      const label = p.label || key;

      if (markers.has(key)) {
        const m = markers.get(key);
        m.setLatLng([lat, lon]);
        m.setPopupContent(label);
      } else {
        const m = L.circleMarker([lat, lon], {
          radius: 6,
          weight: 2,
          color: "#00ff66",
          fillColor: "#00ff66",
          fillOpacity: 0.25
        }).addTo(map);
        m.bindPopup(label);
        markers.set(key, m);
      }
    }
    redrawRays();
  }

  window.BLACKICE = { upsertPoints, setMyLocation, redrawRays, clearRays };
</script>
</body>
</html>
"""


class MapTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.Window, BG)
        self.setPalette(pal)

        self.title = QtWidgets.QLabel("NET TRACE MAP — REMOTE CONTACTS")
        self.title.setFont(HackerFont.mono(14, bold=True))
        self.title.setStyleSheet("color:#00ff66;")

        self.hint = QtWidgets.QLabel(
            "NOTE: markers need GeoIP to be meaningful. Set GEOIP_DB=GeoLite2-City.mmdb for offline coordinates."
        )
        self.hint.setFont(HackerFont.mono(9))
        self.hint.setStyleSheet("color:#00aa44;")

        if HAVE_WEBENGINE:
            self.web = QtWebEngineWidgets.QWebEngineView()
            self.web.setHtml(LEAFLET_HTML)
        else:
            self.web = QtWidgets.QLabel(
                "QtWebEngine not installed.\n\nInstall PyQtWebEngine to enable the MAP tab."
            )
            self.web.setAlignment(QtCore.Qt.AlignCenter)
            self.web.setFont(HackerFont.mono(11))
            self.web.setStyleSheet("color:#ffcc33; background:#07100a; border:1px solid #0b2a12; padding:20px;")

        self.list = QtWidgets.QPlainTextEdit()
        self.list.setReadOnly(True)
        self.list.setFont(HackerFont.mono(10))
        self.list.setStyleSheet(
            "QPlainTextEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; }"
        )

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.addWidget(self.web)
        split.addWidget(self.list)
        split.setSizes([700, 300])

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(self.title)
        layout.addWidget(self.hint)
        layout.addWidget(split, 1)

        self.version = QtWidgets.QLabel(f"{APP_NAME} v{APP_VERSION}")
        self.version.setFont(HackerFont.mono(8))
        self.version.setStyleSheet("color:#0b2a12;")

        footer = QtWidgets.QHBoxLayout()
        footer.addStretch(1)
        footer.addWidget(self.version)

        layout.addLayout(footer)

        self.me_enable = QtWidgets.QCheckBox("Show my location + rays")
        self.me_enable.setFont(HackerFont.mono(10))
        self.me_enable.setStyleSheet("color:#ffcc33;")  # readable on dark

        self.me_refresh = QtWidgets.QPushButton("Locate me now")
        self.me_refresh.setFont(HackerFont.mono(10))
        self.me_refresh.setStyleSheet(
            "QPushButton { background:#07100a; color:#ff9900; border:1px solid #0b2a12; padding:6px 10px; }"
            "QPushButton:hover { border:1px solid #ff9900; }"
        )

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.me_enable)
        row.addWidget(self.me_refresh)
        row.addStretch(1)

        # Insert into layout, e.g. after hint:
        layout.addLayout(row)
        self.me_enable.toggled.connect(self._on_me_toggled)
        self.me_refresh.clicked.connect(self.locate_me)
        self._me_obj = None  # cache {lat, lon, label}

    def _js(self, code: str):
        if HAVE_WEBENGINE and isinstance(self.web, QtWebEngineWidgets.QWebEngineView):
            self.web.page().runJavaScript(code)

    def _get_public_ip(self) -> Optional[str]:
        # Try a couple providers (plain text)
        urls = [
            "https://ifconfig.co/ip",
            "https://api.ipify.org",
        ]
        headers = {"User-Agent": "blackice-defender/1.0"}
        for u in urls:
            try:
                req = urllib.request.Request(u, headers=headers)
                with urllib.request.urlopen(req, timeout=5) as r:
                    ip = r.read().decode("utf-8", "replace").strip()
                    if ip:
                        return ip
            except Exception:
                continue
        return None

    def _geo_online_ipapi(self) -> Tuple[Optional[float], Optional[float], str]:
        # Online fallback: ipapi.co/json
        try:
            req = urllib.request.Request(
                "https://ipapi.co/json/",
                headers={"User-Agent": "blackice-defender/1.0"},
            )
            with urllib.request.urlopen(req, timeout=6) as r:
                data = json.loads(r.read().decode("utf-8", "replace"))
                lat = data.get("latitude")
                lon = data.get("longitude")
                city = (data.get("city") or "").strip()
                country = (data.get("country_code") or "").strip()
                label = " ".join([p for p in [city, country] if p]) or "ME"
                if lat is None or lon is None:
                    return None, None, label
                return float(lat), float(lon), label
        except Exception:
            return None, None, "ME"

    def locate_me(self):
        """
        Resolve "my location".
        Priority:
          - Existing GeoLite2-City mmdb via geoip2 (offline) using public IP
          - Online ipapi.co/json fallback
        """
        lat = lon = None
        label = "ME"

        # If offline GeoIP is available, use public IP + mmdb
        if HAVE_GEOIP and _geoip_reader is not None:
            ip = self._get_public_ip()
            if ip:
                try:
                    r = _geoip_reader.city(ip)
                    lat = r.location.latitude
                    lon = r.location.longitude
                    city = (r.city.name or "").strip()
                    cc = (r.country.iso_code or "").strip()
                    label = " ".join([p for p in [city, cc] if p]) or (cc or "ME")
                    if lat is not None and lon is not None:
                        lat = float(lat);
                        lon = float(lon)
                except Exception:
                    lat = lon = None

        # Online fallback if still unknown
        if lat is None or lon is None:
            lat, lon, label2 = self._geo_online_ipapi()
            label = label2 or label

        if lat is None or lon is None:
            self.list.appendPlainText(f"{time.strftime('%H:%M:%S')}  [!] failed to locate ME")
            self._me_obj = None
            return

        self._me_obj = {"lat": lat, "lon": lon, "label": f"ME — {label}"}
        self.list.appendPlainText(f"{time.strftime('%H:%M:%S')}  [*] ME located: {label} @ ({lat:.3f},{lon:.3f})")

        # Push into map immediately if enabled
        if self.me_enable.isChecked():
            payload = json.dumps(self._me_obj)
            self._js(f"window.BLACKICE && window.BLACKICE.setMyLocation({payload}); window.BLACKICE.redrawRays();")

    def _on_me_toggled(self, enabled: bool):
        if not enabled:
            # just clear rays; keep markers
            self._js("window.BLACKICE && window.BLACKICE.clearRays();")
            return
        # enabled
        if self._me_obj is None:
            self.locate_me()
        else:
            payload = json.dumps(self._me_obj)
            self._js(f"window.BLACKICE && window.BLACKICE.setMyLocation({payload}); window.BLACKICE.redrawRays();")

    def push_points(self, points: List[dict]):
        # right-side list
        lines = []
        for p in points:
            lines.append(
                f"{time.strftime('%H:%M:%S')}  {p.get('label', '')}  @ ({p.get('lat', 0):.3f},{p.get('lon', 0):.3f})"
            )
        if lines:
            self.list.appendPlainText("\n".join(lines))

        if HAVE_WEBENGINE and isinstance(self.web, QtWebEngineWidgets.QWebEngineView):
            map_points = []
            for p in points:
                lat = float(p.get("lat", 0.0) or 0.0)
                lon = float(p.get("lon", 0.0) or 0.0)
                if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                    continue
                map_points.append(p)

            if map_points:
                payload = json.dumps(map_points)
                js = f"window.BLACKICE && window.BLACKICE.upsertPoints({payload});"
                self.web.page().runJavaScript(js)

            # ensure rays follow updates (only if ME enabled & set)
            if self.me_enable.isChecked() and self._me_obj is not None:
                self._js("window.BLACKICE && window.BLACKICE.redrawRays();")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} v{APP_VERSION} ({APP_BUILD}) — Traffic Visualizer")
        self.resize(1200, 780)

        # global stylesheet
        self.setStyleSheet("""
            QMainWindow { background: #050a06; }
            QTabWidget::pane { border: 1px solid #0b2a12; }
            QTabBar::tab {
                background: #07100a; color: #00aa44; padding: 10px 14px;
                border: 1px solid #0b2a12; border-bottom: none;
                font-family: "DejaVu Sans Mono";
            }
            QTabBar::tab:selected { color: #00ff66; }
        """)

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        self.dash = BlackIceDashboard()
        self.map = MapTab()

        self.tabs.addTab(self.dash, "BLACK ICE")
        self.tabs.addTab(self.map, "MAP")

        # pollers
        self.poller = TrafficPoller(interval=1.0)
        self.poller.traffic.connect(self._on_traffic)
        self.poller.start()

        self.scanner = ConnScanner(interval=3.0)
        self.scanner.points.connect(self._on_points)
        self.scanner.event.connect(self.dash.set_event)
        self.scanner.start()

        self.dash.set_event("[*] boot sequence complete")
        self.dash.set_event("[*] traffic sensors online")
        self.dash.set_event("[*] net trace scanner armed")
        self.dash.set_event(f"[*] {APP_NAME} v{APP_VERSION} ({APP_BUILD}) initialized")

        # small UI timer to keep interface list fresh
        self._ui_timer = QtCore.QTimer(self)
        self._ui_timer.timeout.connect(self._ui_tick)
        self._ui_timer.start(5000)

    def _ui_tick(self):
        # refresh interfaces (hotplug, etc.)
        try:
            current = self.dash.iface.currentText()
            self.dash._populate_ifaces()
            idx = self.dash.iface.findText(current)
            if idx >= 0:
                self.dash.iface.setCurrentIndex(idx)
        except Exception:
            pass

    def _on_traffic(self, snap: dict):
        self.dash.update_traffic(snap)

    def _on_points(self, points: list):
        self.map.push_points(points)

    def closeEvent(self, e):
        try:
            self.poller.stop()
            self.poller.wait(1500)
        except Exception:
            pass
        try:
            self.scanner.stop()
            self.scanner.wait(1500)
        except Exception:
            pass
        super().closeEvent(e)


def init_geoip():
    global _geoip_reader
    if not HAVE_GEOIP:
        return
    db = os.environ.get("GEOIP_DB", "").strip()
    if not db:
        db = "./GeoLite2-City.mmdb"
    if not os.path.exists(db):
        db = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "./GeoLite2-City.mmdb")
    if not os.path.exists(db):
        return
    try:
        _geoip_reader = geoip2.database.Reader(db)
    except Exception:
        _geoip_reader = None


def main():
    init_geoip()

    app = QtWidgets.QApplication(sys.argv)
    app.setFont(HackerFont.mono(10))
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
