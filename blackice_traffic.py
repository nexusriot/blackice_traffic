#!/usr/bin/env python3
import collections
import csv
import os
import sys
import math
import threading
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
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

try:
    from PyQt6 import QtWebEngineWidgets
    from PyQt6.QtWebEngineCore import QWebEnginePage  # noqa: F401
    HAVE_WEBENGINE = True
except Exception:
    HAVE_WEBENGINE = False

try:
    import geoip2.database
    HAVE_GEOIP = True
except Exception:
    HAVE_GEOIP = False

APP_NAME = "BLACK ICE"
APP_VERSION = "0.8.0"
APP_BUILD = "alpha"

_geoip_reader = None
HAVE_ASN = False
_asn_reader = None


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


_PORT_SERVICES: Dict[int, str] = {
    20: "FTP-DATA", 21: "FTP", 22: "SSH", 23: "TELNET",
    25: "SMTP", 53: "DNS", 67: "DHCP", 68: "DHCP",
    80: "HTTP", 110: "POP3", 119: "NNTP", 123: "NTP",
    143: "IMAP", 161: "SNMP", 194: "IRC", 443: "HTTPS",
    445: "SMB", 465: "SMTPS", 587: "SMTP-SUB", 636: "LDAPS",
    993: "IMAPS", 995: "POP3S", 1194: "OPENVPN", 1433: "MSSQL",
    1723: "PPTP", 3306: "MYSQL", 3389: "RDP", 5432: "PGSQL",
    5900: "VNC", 6379: "REDIS", 6667: "IRC", 6881: "TORRENT",
    8080: "HTTP-ALT", 8443: "HTTPS-ALT", 9200: "ELASTIC",
    27017: "MONGODB", 51820: "WIREGUARD",
}


def port_service(port: int) -> str:
    return _PORT_SERVICES.get(port, "")


def normalize_ip(ip: str) -> str:
    if ip.startswith("::ffff:"):
        return ip.split("::ffff:", 1)[1]
    return ip


def is_privateish(ip: str) -> bool:
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
        return True


_LOOPBACK_NICS = {"lo", "lo0", "lo1", "loopback"}


def is_loopback_nic(name: str) -> bool:
    """True for loopback interfaces. Their traffic is counted on both RX and
    TX of the same NIC, so including them in the aggregate double-counts
    localhost bytes as if they were real network throughput."""
    n = (name or "").strip().lower()
    return n in _LOOPBACK_NICS or "loopback" in n


def is_loopback_ip(ip: str) -> bool:
    try:
        return ipaddress.ip_address(ip).is_loopback
    except ValueError:
        return False


def ip_sort_key(ip: str) -> Tuple[int, int]:
    """Sortable key for an IP column: IPv4 before IPv6, then numeric order.
    Unparseable strings sort last."""
    try:
        addr = ipaddress.ip_address(ip)
        return (addr.version, int(addr))
    except Exception:
        return (99, 0)


def get_public_ip() -> Optional[str]:
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
                ipaddress.ip_address(ip)  # reject error pages / garbage
                return ip
        except Exception:
            continue
    return None


def geo_online_ipapi() -> Tuple[Optional[float], Optional[float], str]:
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


def resolve_my_location() -> Tuple[Optional[float], Optional[float], str]:
    """Geolocate this host: offline GeoIP on the public IP first, then the
    online ipapi fallback. Blocking (network) — run off the GUI thread."""
    lat = lon = None
    label = "ME"

    if HAVE_GEOIP and _geoip_reader is not None:
        ip = get_public_ip()
        if ip:
            try:
                r = _geoip_reader.city(ip)
                lat = r.location.latitude
                lon = r.location.longitude
                city = (r.city.name or "").strip()
                cc = (r.country.iso_code or "").strip()
                label = " ".join([p for p in [city, cc] if p]) or (cc or "ME")
                if lat is not None and lon is not None:
                    lat = float(lat)
                    lon = float(lon)
            except Exception:
                lat = lon = None

    if lat is None or lon is None:
        lat, lon, label2 = geo_online_ipapi()
        label = label2 or label

    return lat, lon, label


@dataclass
class ConnPoint:
    ip: str
    port: int
    proto: str
    lat: float
    lon: float
    label: str
    ts: float
    process: str = ""
    asn: str = ""


CONFIG_ORG = "blackice"
CONFIG_APP = "blackice_traffic"


def app_settings() -> QtCore.QSettings:
    """A predictable, hand-editable INI store (~/.config/blackice/blackice_traffic.ini)
    that behaves the same for the source tree, the PyInstaller binary and the .deb."""
    return QtCore.QSettings(
        QtCore.QSettings.Format.IniFormat,
        QtCore.QSettings.Scope.UserScope,
        CONFIG_ORG,
        CONFIG_APP,
    )


class AppConfig:
    """Opt-in persistence of window geometry, layout and view state.

    Only the enabled flag is always written; every other key is read and
    written exclusively while saving is on, so switching it off leaves the app
    starting from defaults again. Reads never raise: the INI is meant to be
    hand-editable, and a mangled value must not stop the app from starting."""

    KEY_ENABLED = "config/save_enabled"

    def __init__(self, settings: Optional[QtCore.QSettings] = None):
        self._s = settings if settings is not None else app_settings()
        self._enabled = self._read(self.KEY_ENABLED, False, bool)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def path(self) -> str:
        return self._s.fileName()

    def _read(self, key: str, default, cast=None):
        try:
            if cast is not None:
                return self._s.value(key, default, type=cast)
            return self._s.value(key, default)
        except (TypeError, ValueError):
            return default

    def set_enabled(self, on: bool):
        self._enabled = bool(on)
        self._s.setValue(self.KEY_ENABLED, self._enabled)
        self._s.sync()

    def get(self, key: str, default=None, cast=None):
        if not self._enabled:
            return default
        return self._read(key, default, cast)

    def set(self, key: str, value):
        if not self._enabled:
            return
        self._s.setValue(key, value)

    def clear_state(self):
        """Forget every saved layout key, keeping the enabled flag itself."""
        for key in self._s.allKeys():
            if key != self.KEY_ENABLED:
                self._s.remove(key)
        self._s.sync()

    def sync(self):
        self._s.sync()


class HackerFont:
    @staticmethod
    def mono(size=11, bold=False):
        f = QtGui.QFont("DejaVu Sans Mono", size)
        f.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        f.setBold(bold)
        return f


class AnimatedOverlay(QtWidgets.QWidget):
    """Transparent overlay whose repaint timer only ticks while visible."""
    interval_ms = 33

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)

    def _tick(self):
        raise NotImplementedError

    def set_running(self, run: bool):
        if run:
            if self.isVisible() and not self._timer.isActive():
                self._timer.start(self.interval_ms)
        else:
            self._timer.stop()

    def showEvent(self, e):
        super().showEvent(e)
        self.set_running(True)

    def hideEvent(self, e):
        super().hideEvent(e)
        self.set_running(False)


class ScanlinesOverlay(AnimatedOverlay):
    """Transparent scanlines + subtle flicker overlay."""
    interval_ms = 33

    def __init__(self, parent=None):
        super().__init__(parent)
        self._phase = 0.0

    def _tick(self):
        self._phase += 0.10
        self.update()

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, False)

        w, h = self.width(), self.height()
        p.setOpacity(0.18)
        pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 255))
        for y in range(0, h, 3):
            alpha = 35 + int(20 * (0.5 + 0.5 * math.sin((y * 0.08) + self._phase)))
            c = QtGui.QColor(0, 0, 0, alpha)
            pen.setColor(c)
            p.setPen(pen)
            p.drawLine(0, y, w, y)

        p.setOpacity(0.25)
        grad = QtGui.QRadialGradient(w * 0.5, h * 0.5, max(w, h) * 0.75)
        grad.setColorAt(0.0, QtGui.QColor(0, 0, 0, 0))
        grad.setColorAt(1.0, QtGui.QColor(0, 0, 0, 220))
        p.fillRect(self.rect(), grad)

        p.setOpacity(0.04 + random.random() * 0.03)
        p.fillRect(self.rect(), QtGui.QColor(255, 255, 255, 255))


class MatrixRain(AnimatedOverlay):
    """Simple matrix-rain background."""
    interval_ms = 50

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cols = []
        self.char_set = list("01abcdef#$%&*+<>/\\|[]{}()~")

    def resizeEvent(self, e):
        super().resizeEvent(e)
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
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, False)
        p.setFont(HackerFont.mono(10))

        p.setOpacity(0.20)
        for c in self.cols:
            x = c["x"]
            y = c["y"]
            for i in range(c["len"]):
                ch = random.choice(self.char_set)
                yy = y - i * 14
                if 0 <= yy <= self.height():
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

    def reset(self):
        """Drop the waveform history (e.g. after an interface switch, where
        the retained samples belong to a different NIC)."""
        self._rx = [0.0] * len(self._rx)
        self._tx = [0.0] * len(self._tx)
        self._max = 1.0
        self.update()

    def push(self, rx_bps: float, tx_bps: float):
        self._rx.pop(0); self._rx.append(rx_bps)
        self._tx.pop(0); self._tx.append(tx_bps)
        self._max = max(1.0, max(max(self._rx), max(self._tx)) * 1.10)
        self.update()

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), BG)

        w, h = self.width(), self.height()

        p.setPen(QtGui.QPen(GRID, 1))
        for x in range(0, w, 40):
            p.drawLine(x, 0, x, h)
        for y in range(0, h, 20):
            p.drawLine(0, y, w, y)

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

        p.setPen(QtGui.QPen(PHOSPHOR, 2))
        p.setOpacity(0.85)
        p.drawPolyline(poly(self._rx))

        p.setPen(QtGui.QPen(AMBER, 2))
        p.setOpacity(0.75)
        p.drawPolyline(poly(self._tx))

        p.setOpacity(1.0)
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        p.drawText(w - 190, 16, f"scale max: {human_bps(self._max)}")


class HistoryGraph(QtWidgets.QWidget):
    """Rolling time-series graph for bandwidth history."""

    WINDOWS = {"5 MIN": 300, "15 MIN": 900, "30 MIN": 1800}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220)
        self._window = 300
        self._data: Dict[str, collections.deque] = {}
        self._selected_iface = "ALL"

    def set_window(self, seconds: int):
        self._window = seconds
        self.update()

    def set_iface(self, iface: str):
        self._selected_iface = iface
        self.update()

    def push(self, ts: float, snap: dict):
        for iface, v in snap.items():
            if not isinstance(v, dict) or "rx_bps" not in v:
                continue
            if iface not in self._data:
                self._data[iface] = collections.deque(maxlen=1800)
            self._data[iface].append((ts, v["rx_bps"], v["tx_bps"]))

        # Drop NICs that stopped reporting (transient docker/veth names) so
        # _data does not keep a deque per name ever seen.
        stale_before = ts - max(self.WINDOWS.values())
        for gone in [k for k, d in self._data.items() if not d or d[-1][0] < stale_before]:
            del self._data[gone]

        self.update()

    def _get_series(self) -> List[Tuple[float, float, float]]:
        src_key = "_totals" if self._selected_iface == "ALL" else self._selected_iface
        src = self._data.get(src_key, collections.deque())
        cutoff = time.time() - self._window
        return [(ts, rx, tx) for ts, rx, tx in src if ts >= cutoff]

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), BG)

        w, h = self.width(), self.height()
        ML, MR, MT, MB = 90, 20, 24, 30
        gw = w - ML - MR
        gh = h - MT - MB

        p.setPen(QtGui.QPen(GRID, 1))
        p.setOpacity(1.0)
        for i in range(5):
            y = MT + int(i * gh / 4)
            p.drawLine(ML, y, ML + gw, y)
        for i in range(7):
            x = ML + int(i * gw / 6)
            p.drawLine(x, MT, x, MT + gh)

        series = self._get_series()

        p.setFont(HackerFont.mono(9))
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        win_label = next((k for k, v in self.WINDOWS.items() if v == self._window), f"{self._window}s")
        p.drawText(ML + 4, 16, f"BANDWIDTH HISTORY  /  WINDOW: {win_label}  /  IFACE: {self._selected_iface}")
        p.setPen(QtGui.QPen(PHOSPHOR))
        p.drawText(w - 120, 16, "▬ RX")
        p.setPen(QtGui.QPen(AMBER))
        p.drawText(w - 70, 16, "▬ TX")

        if not series:
            p.setPen(QtGui.QPen(PHOSPHOR_DIM))
            p.setFont(HackerFont.mono(11))
            p.drawText(ML + gw // 2 - 70, MT + gh // 2 + 6, "ACCUMULATING DATA...")
            return

        max_val = max(max(rx, tx) for _, rx, tx in series) * 1.1
        max_val = max(1.0, max_val)

        p.setFont(HackerFont.mono(8))
        for i in range(5):
            val = max_val * (4 - i) / 4
            y = MT + int(i * gh / 4)
            p.setPen(QtGui.QPen(PHOSPHOR_DIM))
            p.drawText(2, y - 5, ML - 6, 14,
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       human_bps(val))

        now_ts = time.time()
        start_ts = now_ts - self._window
        for i in range(7):
            frac = i / 6.0
            ts_lbl = start_ts + frac * self._window
            x = ML + int(frac * gw)
            lbl = time.strftime("%H:%M", time.localtime(ts_lbl))
            p.setPen(QtGui.QPen(PHOSPHOR_DIM))
            p.drawText(x - 20, MT + gh + 4, 40, MB - 4,
                       Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, lbl)

        def to_pt(ts, val):
            xf = (ts - start_ts) / self._window if self._window else 0.5
            yf = 1.0 - clamp(val / max_val, 0.0, 1.0)
            return QtCore.QPointF(ML + xf * gw, MT + yf * gh)

        if len(series) > 1:
            rx_poly = QtGui.QPolygonF([to_pt(ts, rx) for ts, rx, _ in series])
            p.setPen(QtGui.QPen(PHOSPHOR, 2))
            p.setOpacity(0.85)
            p.drawPolyline(rx_poly)

            tx_poly = QtGui.QPolygonF([to_pt(ts, tx) for ts, _, tx in series])
            p.setPen(QtGui.QPen(AMBER, 2))
            p.setOpacity(0.75)
            p.drawPolyline(tx_poly)

        p.setOpacity(1.0)


def build_snapshot(prev: dict, now: dict, dt: float) -> dict:
    """Per-NIC rates plus the aggregate under "_totals". Loopback NICs are
    reported individually but kept out of the aggregate."""
    snap: Dict[str, dict] = {}
    total_rx_bps = 0.0
    total_tx_bps = 0.0

    for nic, cnt in now.items():
        p = prev.get(nic)
        if p is None:
            continue
        rx_bps = max(0.0, (cnt.bytes_recv - p.bytes_recv) * 8.0 / dt)
        tx_bps = max(0.0, (cnt.bytes_sent - p.bytes_sent) * 8.0 / dt)
        snap[nic] = {
            "rx_bps": rx_bps,
            "tx_bps": tx_bps,
            "rx_total": cnt.bytes_recv,
            "tx_total": cnt.bytes_sent,
            "pkts_in": cnt.packets_recv,
            "pkts_out": cnt.packets_sent,
        }
        if not is_loopback_nic(nic):
            total_rx_bps += rx_bps
            total_tx_bps += tx_bps

    snap["_totals"] = {"rx_bps": total_rx_bps, "tx_bps": total_tx_bps}
    return snap


class TrafficPoller(QtCore.QThread):
    traffic = QtCore.pyqtSignal(dict)

    def __init__(self, interval=1.0, parent=None):
        super().__init__(parent)
        self.interval = interval
        self._stop_evt = QtCore.QWaitCondition()
        self._mtx = QtCore.QMutex()
        self._stop = False
        self._prev = None

    def stop(self):
        self._mtx.lock()
        self._stop = True
        self._stop_evt.wakeAll()
        self._mtx.unlock()

    def _sleep(self, secs: float) -> bool:
        self._mtx.lock()
        try:
            if self._stop:
                return True
            self._stop_evt.wait(self._mtx, int(secs * 1000))
            return self._stop
        finally:
            self._mtx.unlock()

    def run(self):
        self._prev = psutil.net_io_counters(pernic=True)
        prev_t = time.time()
        while not self._stop:
            if self._sleep(self.interval):
                break
            now = psutil.net_io_counters(pernic=True)
            now_t = time.time()
            dt = max(0.2, now_t - prev_t)

            snap = build_snapshot(self._prev, now, dt)
            self._prev = now
            prev_t = now_t
            self.traffic.emit(snap)


class ConnScanner(QtCore.QThread):
    points = QtCore.pyqtSignal(list)
    event = QtCore.pyqtSignal(str)

    def __init__(self, interval=3.0, parent=None):
        super().__init__(parent)
        self.interval = interval
        self._stop_evt = QtCore.QWaitCondition()
        self._mtx = QtCore.QMutex()
        self._stop = False
        self._seen: Dict[str, float] = {}

    def stop(self):
        self._mtx.lock()
        self._stop = True
        self._stop_evt.wakeAll()
        self._mtx.unlock()

    def _sleep(self, secs: float) -> bool:
        self._mtx.lock()
        try:
            if self._stop:
                return True
            self._stop_evt.wait(self._mtx, int(secs * 1000))
            return self._stop
        finally:
            self._mtx.unlock()

    def _normalize_ip(self, ip: str) -> str:
        return normalize_ip(ip)

    def _is_privateish(self, ip: str) -> bool:
        return is_privateish(ip)

    def _geo_lookup(self, ip: str) -> Tuple[Optional[float], Optional[float], str]:
        ip = self._normalize_ip(ip)

        if self._is_privateish(ip):
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

                parts = [p for p in [city, region, country] if p]
                label = " ".join(parts) if parts else (country or "Unknown")

                if lat is None or lon is None:
                    return None, None, label

                return float(lat), float(lon), label
            except Exception:
                pass

        return None, None, "GeoIP unavailable"

    def _asn_lookup(self, ip: str) -> str:
        global _asn_reader
        if not HAVE_ASN or _asn_reader is None:
            return ""
        try:
            r = _asn_reader.asn(ip)
            org = (r.autonomous_system_organization or "").strip()
            asn_num = r.autonomous_system_number
            if org and asn_num:
                return f"AS{asn_num} {org}"
            return org or (f"AS{asn_num}" if asn_num else "")
        except Exception:
            return ""

    def _scan_psutil(self) -> List[ConnPoint]:
        out: List[ConnPoint] = []
        try:
            conns = psutil.net_connections(kind="inet")
        except Exception as e:
            self.event.emit(f"[!] net_connections failed: {e}")
            return out

        now = time.time()
        cutoff = now - 600
        self._seen = {k: t for k, t in self._seen.items() if t >= cutoff}
        for c in conns:
            if not c.raddr:
                continue
            # Normalize first: an IPv4-mapped v6 socket and a plain v4
            # socket to the same endpoint must share one dedupe key, and
            # ::ffff:127.0.0.1 has to be filtered as loopback.
            ip = self._normalize_ip(c.raddr.ip)
            port = int(c.raddr.port)
            proto = "tcp" if c.type == socket.SOCK_STREAM else "udp"
            key = f"{proto}:{ip}:{port}"
            if is_loopback_ip(ip):
                continue
            if key in self._seen and (now - self._seen[key]) < 20:
                continue
            self._seen[key] = now

            proc_name = ""
            if c.pid:
                try:
                    proc_name = psutil.Process(c.pid).name()
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass

            lat, lon, where = self._geo_lookup(ip)
            asn = self._asn_lookup(ip) if not self._is_privateish(ip) else ""
            label = f"{ip}:{port} ({proto}) — {where}"
            out.append(ConnPoint(ip=ip, port=port, proto=proto,
                                 lat=lat or 0.0, lon=lon or 0.0, label=label, ts=now,
                                 process=proc_name, asn=asn))

        return out

    def run(self):
        while not self._stop:
            pts = self._scan_psutil()
            if pts:
                self.points.emit([p.__dict__ for p in pts])
                self.event.emit(f"[+] contacts detected: {len(pts)}")
            if self._sleep(self.interval):
                break


class ConsoleLog(QtWidgets.QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(5000)
        self.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.setFont(HackerFont.mono(10))
        self.setStyleSheet(
            "QPlainTextEdit { background: #050a06; color: #00ff66; selection-background-color: #0b2a12; }"
        )

    def push(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.appendPlainText(f"{ts} {msg}")


def _hacker_button(text: str, checkable: bool = False) -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    b.setFont(HackerFont.mono(10))
    b.setCheckable(checkable)
    b.setStyleSheet(
        "QPushButton { background:#07100a; color:#00ff66; border:1px solid #0b2a12; padding:6px 10px; }"
        "QPushButton:hover { border:1px solid #00ff66; }"
        "QPushButton:checked { background:#0b2a12; border:1px solid #00ff66; }"
    )
    return b


class BlackIceDashboard(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()
    configToggled = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
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

        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)

        self.config_btn = _hacker_button("▣ SAVE CONFIG", checkable=True)
        self.config_btn.setToolTip(
            "Remember window geometry, layout, selected tab and view state between runs"
        )
        self.config_btn.toggled.connect(self.configToggled.emit)

        self.log = ConsoleLog()

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.title)
        top.addSpacing(12)
        top.addWidget(self.version)
        top.addStretch(1)
        iface_lbl = QtWidgets.QLabel("INTERFACE:")
        iface_lbl.setFont(HackerFont.mono(10))
        iface_lbl.setStyleSheet("color:#00aa44;")
        top.addWidget(iface_lbl)
        top.addWidget(self.iface)
        top.addSpacing(12)
        top.addWidget(self.config_btn)
        top.addWidget(self.snapshot_btn)

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
        ev_lbl = QtWidgets.QLabel("EVENT LOG:")
        ev_lbl.setFont(HackerFont.mono(10))
        ev_lbl.setStyleSheet("color:#00aa44;")
        layout.addWidget(ev_lbl)
        layout.addWidget(self.log, 1)

        self._last_snap = {}
        self._populate_ifaces()
        # A switch changes which NIC the waveform describes, so the retained
        # samples (and the derived scale) must not carry over.
        self.iface.currentTextChanged.connect(self._on_iface_changed)

    def _populate_ifaces(self):
        self.iface.clear()
        nics = list(psutil.net_io_counters(pernic=True).keys())
        self.iface.addItem("ALL")
        for n in nics:
            self.iface.addItem(n)

    def set_config_enabled(self, on: bool):
        """Reflect the stored flag without re-emitting configToggled."""
        self.config_btn.blockSignals(True)
        self.config_btn.setChecked(bool(on))
        self.config_btn.blockSignals(False)

    def _on_iface_changed(self, _name: str):
        self.scope.reset()
        self.rx_lbl.setText("RX: 0 b/s")
        self.tx_lbl.setText("TX: 0 b/s")

    def resizeEvent(self, e):
        self.matrix.setGeometry(self.rect())
        self.scan.setGeometry(self.rect())
        super().resizeEvent(e)

    def set_fx_running(self, run: bool):
        self.matrix.set_running(run)
        self.scan.set_running(run)

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
            nics = [(k, v) for k, v in snap.items()
                    if k != "_totals" and not is_loopback_nic(k)]
            rx_total = sum(v.get("rx_total", 0) for _, v in nics)
            tx_total = sum(v.get("tx_total", 0) for _, v in nics)
            self.totals_lbl.setText(f"TOTALS: RX {human_bytes(rx_total)}  |  TX {human_bytes(tx_total)}")
        else:
            v = snap.get(sel)
            if not v:
                # No counters for the selected NIC this tick (it just appeared
                # or went away). Keep the user's selection — _ui_tick rebuilds
                # the combo when the NIC set actually changes.
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

  let me = null;
  let meMarker = null;
  let rayLayer = L.layerGroup().addTo(map);

  function setMyLocation(obj) {
    if (!obj || obj.lat == null || obj.lon == null) return;
    me = { lat: obj.lat, lon: obj.lon, label: obj.label || "ME" };

    if (!meMarker) {
      meMarker = L.circleMarker([me.lat, me.lon], {
        radius: 8, weight: 2, color: "#ff9900",
        fillColor: "#ff9900", fillOpacity: 0.35
      }).addTo(map);
      meMarker.bindPopup(me.label);
    } else {
      meMarker.setLatLng([me.lat, me.lon]);
      meMarker.setPopupContent(me.label);
    }
  }

  function clearRays() { rayLayer.clearLayers(); }

  function clearMe() {
    if (meMarker) { map.removeLayer(meMarker); meMarker = null; }
    me = null;
    clearRays();
  }

  function focusMe(zoom) {
    if (me) map.flyTo([me.lat, me.lon], zoom || 8);
  }

  function redrawRays() {
    clearRays();
    if (!me) return;
    for (const [key, m] of markers.entries()) {
      const ll = m.getLatLng();
      if (Math.abs(ll.lat) < 1e-6 && Math.abs(ll.lng) < 1e-6) continue;
      const line = L.polyline([[me.lat, me.lon], [ll.lat, ll.lng]], {
        color: "#ff9900", weight: 1.6, opacity: 0.55
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
          radius: 6, weight: 2, color: "#00ff66",
          fillColor: "#00ff66", fillOpacity: 0.25
        }).addTo(map);
        m.bindPopup(label);
        markers.set(key, m);
      }
    }
    redrawRays();
  }

  window.BLACKICE = { upsertPoints, setMyLocation, redrawRays, clearRays, clearMe, focusMe };
</script>
</body>
</html>
"""


class LocateBridge(QtCore.QObject):
    """Marshals locate-worker results back onto the GUI thread."""
    located = QtCore.pyqtSignal(float, float, str)
    failed = QtCore.pyqtSignal(str)


class MapTab(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()

    MAX_PENDING_JS = 64

    def __init__(self):
        super().__init__()

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
        self.setPalette(pal)

        self.title = QtWidgets.QLabel("NET TRACE MAP — REMOTE CONTACTS")
        self.title.setFont(HackerFont.mono(14, bold=True))
        self.title.setStyleSheet("color:#00ff66;")

        self.hint = QtWidgets.QLabel(
            "NOTE: markers need GeoIP to be meaningful. Set GEOIP_DB=GeoLite2-City.mmdb for offline coordinates."
        )
        self.hint.setFont(HackerFont.mono(9))
        self.hint.setStyleSheet("color:#00aa44;")

        self._page_ready = False
        self._pending_js: List[str] = []

        if HAVE_WEBENGINE:
            self.web = QtWebEngineWidgets.QWebEngineView()
            self.web.loadFinished.connect(self._on_load_finished)
            self.web.setHtml(LEAFLET_HTML)
        else:
            self.web = QtWidgets.QLabel(
                "QtWebEngine not installed.\n\nInstall PyQt6-WebEngine to enable the MAP tab."
            )
            self.web.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.web.setFont(HackerFont.mono(11))
            self.web.setStyleSheet("color:#ffcc33; background:#07100a; border:1px solid #0b2a12; padding:20px;")
            self._page_ready = True  # nothing to wait for; _exec_js no-ops

        self.list = QtWidgets.QPlainTextEdit()
        self.list.setReadOnly(True)
        self.list.setMaximumBlockCount(5000)
        self.list.setFont(HackerFont.mono(10))
        self.list.setStyleSheet(
            "QPlainTextEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; }"
        )

        self.split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)
        self.split.addWidget(self.web)
        self.split.addWidget(self.list)
        self.split.setSizes([700, 300])

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(self.title)
        layout.addWidget(self.hint)
        layout.addWidget(self.split, 1)

        self.version = QtWidgets.QLabel(f"{APP_NAME} v{APP_VERSION}")
        self.version.setFont(HackerFont.mono(8))
        self.version.setStyleSheet("color:#0b2a12;")

        footer = QtWidgets.QHBoxLayout()
        footer.addStretch(1)
        footer.addWidget(self.version)

        layout.addLayout(footer)

        self.me_enable = QtWidgets.QCheckBox("Show my location + rays")
        self.me_enable.setFont(HackerFont.mono(10))
        self.me_enable.setStyleSheet("color:#ffcc33;")

        self.me_refresh = QtWidgets.QPushButton("Locate me now")
        self.focus_btn = QtWidgets.QPushButton("⌖ MY LOCATION")
        for b in (self.me_refresh, self.focus_btn):
            b.setFont(HackerFont.mono(10))
            b.setStyleSheet(
                "QPushButton { background:#07100a; color:#ff9900; border:1px solid #0b2a12; padding:6px 10px; }"
                "QPushButton:hover { border:1px solid #ff9900; }"
                "QPushButton:disabled { color:#664400; }"
            )

        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.me_enable)
        row.addWidget(self.me_refresh)
        row.addWidget(self.focus_btn)
        row.addStretch(1)
        row.addWidget(self.snapshot_btn)

        layout.addLayout(row)
        self.me_enable.toggled.connect(self._on_me_toggled)
        self.me_refresh.clicked.connect(lambda: self.locate_me(focus=False))
        self.focus_btn.clicked.connect(lambda: self.locate_me(focus=True))
        self._me_obj = None
        self._locating = False
        self._focus_pending = False
        self._bridge = LocateBridge(self)
        self._bridge.located.connect(self._on_located)
        self._bridge.failed.connect(self._on_locate_failed)

    def _exec_js(self, code: str):
        if HAVE_WEBENGINE and isinstance(self.web, QtWebEngineWidgets.QWebEngineView):
            self.web.page().runJavaScript(code)

    def _js(self, code: str):
        # setHtml() loads asynchronously and Leaflet itself is fetched over the
        # network, so window.BLACKICE does not exist for the first seconds.
        # Queue instead of letting the guard silently swallow the call.
        if not self._page_ready:
            self._pending_js.append(code)
            del self._pending_js[:-self.MAX_PENDING_JS]
            return
        self._exec_js(code)

    def _on_load_finished(self, ok: bool):
        if not ok:
            self._pending_js.clear()
            self._log("[!] map page failed to load")
            return
        self._page_ready = True
        pending, self._pending_js = self._pending_js, []
        for code in pending:
            self._exec_js(code)

    def _log(self, msg: str):
        self.list.appendPlainText(f"{time.strftime('%H:%M:%S')}  {msg}")

    def _push_me_marker(self):
        payload = json.dumps(self._me_obj)
        self._js(
            "if (window.BLACKICE) { window.BLACKICE.setMyLocation(%s); window.BLACKICE.redrawRays(); }"
            % payload
        )

    def locate_me(self, focus: bool = False):
        """Kick off geolocation in the background; never blocks the GUI."""
        if self._locating:
            self._focus_pending = self._focus_pending or focus
            return
        self._locating = True
        self._focus_pending = focus
        self.me_refresh.setEnabled(False)
        self.focus_btn.setEnabled(False)
        self._log("[*] locating ME ...")
        threading.Thread(target=self._locate_worker, daemon=True).start()

    def _locate_worker(self):
        try:
            lat, lon, label = resolve_my_location()
            if lat is None or lon is None:
                self._bridge.failed.emit(label or "no result")
            else:
                self._bridge.located.emit(lat, lon, label)
        except RuntimeError:
            pass  # widget torn down while the lookup was in flight

    def _on_located(self, lat: float, lon: float, label: str):
        self._locating = False
        self.me_refresh.setEnabled(True)
        self.focus_btn.setEnabled(True)
        focus = self._focus_pending
        self._focus_pending = False

        self._me_obj = {"lat": lat, "lon": lon, "label": f"ME — {label}"}
        self._log(f"[*] ME located: {label} @ ({lat:.3f},{lon:.3f})")

        if focus and not self.me_enable.isChecked():
            self.me_enable.setChecked(True)  # pushes the marker via _on_me_toggled
        elif self.me_enable.isChecked():
            self._push_me_marker()

        if focus:
            self._js("if (window.BLACKICE) { window.BLACKICE.focusMe(8); }")
            self._log("[*] map focused on ME")

    def _on_locate_failed(self, msg: str):
        self._locating = False
        self._focus_pending = False
        self.me_refresh.setEnabled(True)
        self.focus_btn.setEnabled(True)
        self._log(f"[!] failed to locate ME ({msg})")

    def _on_me_toggled(self, enabled: bool):
        if not enabled:
            self._js("if (window.BLACKICE) { window.BLACKICE.clearMe(); }")
            return
        if self._me_obj is None:
            self.locate_me()
        else:
            self._push_me_marker()

    def push_points(self, points: List[dict]):
        lines = []
        for p in points:
            lines.append(
                f"{time.strftime('%H:%M:%S')}  {p.get('label', '')}  @ ({p.get('lat', 0):.3f},{p.get('lon', 0):.3f})"
            )
        if lines:
            self.list.appendPlainText("\n".join(lines))

        map_points = []
        for p in points:
            lat = float(p.get("lat", 0.0) or 0.0)
            lon = float(p.get("lon", 0.0) or 0.0)
            if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                continue
            map_points.append(p)

        if map_points:
            payload = json.dumps(map_points)
            self._js(f"window.BLACKICE && window.BLACKICE.upsertPoints({payload});")

        if self.me_enable.isChecked() and self._me_obj is not None:
            self._js("window.BLACKICE && window.BLACKICE.redrawRays();")


class ContactSortProxy(QtCore.QSortFilterProxyModel):
    """Sorts by the UserRole key (timestamps, ports, hit counts, packed IPs)
    when present, falling back to the display text."""

    def lessThan(self, left, right):
        lv = left.data(Qt.ItemDataRole.UserRole)
        rv = right.data(Qt.ItemDataRole.UserRole)
        if lv is not None and rv is not None:
            try:
                return lv < rv
            except TypeError:
                pass
        return super().lessThan(left, right)


class ConnectionsTab(QtWidgets.QWidget):
    """Sortable, filterable live table of remote contacts."""

    COLS = ["First Seen", "Last Seen", "Proto", "IP", "Port/Svc", "Location", "Process", "ASN/Org", "Hits"]

    snapshotRequested = QtCore.pyqtSignal()
    csvExportRequested = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
        self.setPalette(pal)

        self.title = QtWidgets.QLabel("CONTACT TABLE — AGGREGATED REMOTE ENDPOINTS")
        self.title.setFont(HackerFont.mono(14, bold=True))
        self.title.setStyleSheet("color:#00ff66;")

        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setPlaceholderText("filter: ip / port / proto / location ...")
        self.filter_edit.setFont(HackerFont.mono(10))
        self.filter_edit.setStyleSheet(
            "QLineEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; padding:4px; }"
        )

        self.count_lbl = QtWidgets.QLabel("0 contacts")
        self.count_lbl.setFont(HackerFont.mono(10))
        self.count_lbl.setStyleSheet("color:#00aa44;")

        self.clear_btn = _hacker_button("⌫ CLEAR")
        self.export_btn = _hacker_button("⤓ EXPORT CSV")
        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")
        self.clear_btn.clicked.connect(self._clear)
        self.export_btn.clicked.connect(self.csvExportRequested.emit)
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)

        self.model = QtGui.QStandardItemModel(0, len(self.COLS), self)
        self.model.setHorizontalHeaderLabels(self.COLS)

        self.proxy = ContactSortProxy(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.proxy.setFilterKeyColumn(-1)
        self.filter_edit.textChanged.connect(self.proxy.setFilterFixedString)
        self.filter_edit.textChanged.connect(lambda _t: self._update_count())

        self.view = QtWidgets.QTableView()
        self.view.setModel(self.proxy)
        self.view.setSortingEnabled(True)
        self.view.setAlternatingRowColors(True)
        self.view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.view.verticalHeader().setVisible(False)
        self.view.horizontalHeader().setStretchLastSection(True)
        self.view.setFont(HackerFont.mono(10))
        self.view.setStyleSheet(
            "QTableView { background:#050a06; alternate-background-color:#07100a;"
            " color:#00ff66; gridline-color:#0b2a12; selection-background-color:#0b2a12;"
            " selection-color:#00ff66; border:1px solid #0b2a12; }"
            "QHeaderView::section { background:#07100a; color:#00aa44; border:1px solid #0b2a12; padding:4px; }"
        )
        self.view.sortByColumn(1, Qt.SortOrder.DescendingOrder)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(self.filter_edit, 1)
        controls.addWidget(self.count_lbl)
        controls.addSpacing(8)
        controls.addWidget(self.clear_btn)
        controls.addWidget(self.export_btn)
        controls.addWidget(self.snapshot_btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(self.title)
        layout.addLayout(controls)
        layout.addWidget(self.view, 1)

        self._rows: Dict[str, int] = {}

    def _clear(self):
        self.model.removeRows(0, self.model.rowCount())
        self._rows.clear()
        self._update_count()

    def _update_count(self):
        total = self.model.rowCount()
        shown = self.proxy.rowCount()
        self.count_lbl.setText(
            f"{total} contacts" if shown == total else f"{shown} of {total} contacts"
        )

    def _make_item(self, text: str, sort_value=None) -> QtGui.QStandardItem:
        it = QtGui.QStandardItem(text)
        it.setEditable(False)
        if sort_value is not None:
            it.setData(sort_value, Qt.ItemDataRole.UserRole)
        return it

    def add_points(self, points: List[dict]):
        for p in points:
            proto = p.get("proto", "?")
            ip = p.get("ip", "?")
            port = int(p.get("port", 0))
            label = p.get("label", "")
            location = label.split("—", 1)[1].strip() if "—" in label else label
            ts = float(p.get("ts", time.time()))
            ts_str = time.strftime("%H:%M:%S", time.localtime(ts))
            key = f"{proto}:{ip}:{port}"
            process = p.get("process", "")
            asn = p.get("asn", "")
            svc = port_service(port)
            port_svc = f"{port} · {svc}" if svc else str(port)

            if key in self._rows:
                src_row = self._rows[key]
                self.model.item(src_row, 1).setText(ts_str)
                self.model.item(src_row, 1).setData(ts, Qt.ItemDataRole.UserRole)
                hits_item = self.model.item(src_row, 8)
                try:
                    hits = int(hits_item.text()) + 1
                except Exception:
                    hits = 2
                hits_item.setText(str(hits))
                hits_item.setData(hits, Qt.ItemDataRole.UserRole)
                self.model.item(src_row, 5).setText(location)
                if process:
                    self.model.item(src_row, 6).setText(process)
                if asn:
                    self.model.item(src_row, 7).setText(asn)
                continue

            row = [
                self._make_item(ts_str, ts),
                self._make_item(ts_str, ts),
                self._make_item(proto.upper()),
                self._make_item(ip, ip_sort_key(ip)),
                self._make_item(port_svc, port),
                self._make_item(location),
                self._make_item(process),
                self._make_item(asn),
                self._make_item("1", 1),
            ]
            color = QtGui.QBrush(PHOSPHOR)
            if location.startswith("LOCAL"):
                color = QtGui.QBrush(PHOSPHOR_DIM)
            elif "unavailable" in location.lower():
                color = QtGui.QBrush(AMBER)
            for it in row:
                it.setForeground(color)
            self.model.appendRow(row)
            self._rows[key] = self.model.rowCount() - 1

        self._update_count()

    def export_csv(self, path: str):
        """Export what the table shows: the active filter and the current sort
        order, so the file matches the view the user exported from."""
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            for r in range(self.proxy.rowCount()):
                w.writerow([
                    self.proxy.index(r, c).data() or ""
                    for c in range(self.proxy.columnCount())
                ])


class StatsTab(QtWidgets.QWidget):
    """Bandwidth history analytics with configurable time window."""

    snapshotRequested = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
        self.setPalette(pal)

        self.title = QtWidgets.QLabel("BANDWIDTH HISTORY — TIME-SERIES ANALYTICS")
        self.title.setFont(HackerFont.mono(14, bold=True))
        self.title.setStyleSheet("color:#00ff66;")

        self.graph = HistoryGraph()

        self.iface_combo = QtWidgets.QComboBox()
        self.iface_combo.setFont(HackerFont.mono(10))
        self.iface_combo.setStyleSheet(
            "QComboBox { background: #07100a; color:#00ff66; border: 1px solid #0b2a12; padding: 4px; }"
            "QAbstractItemView { background: #07100a; color:#00ff66; selection-background-color:#0b2a12; }"
        )
        self.iface_combo.addItem("ALL")
        for n in psutil.net_io_counters(pernic=True):
            self.iface_combo.addItem(n)
        self.iface_combo.currentTextChanged.connect(self.graph.set_iface)

        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)

        self._win_group = QtWidgets.QButtonGroup(self)
        self.win_buttons: Dict[int, QtWidgets.QRadioButton] = {}
        win_row = QtWidgets.QHBoxLayout()
        for lbl_text, secs in HistoryGraph.WINDOWS.items():
            rb = QtWidgets.QRadioButton(lbl_text)
            rb.setFont(HackerFont.mono(10))
            rb.setStyleSheet(
                "QRadioButton { color:#00aa44; }"
                "QRadioButton::checked { color:#00ff66; }"
            )
            if lbl_text == "5 MIN":
                rb.setChecked(True)
            rb.toggled.connect(lambda checked, s=secs: checked and self.graph.set_window(s))
            self._win_group.addButton(rb)
            self.win_buttons[secs] = rb
            win_row.addWidget(rb)
        win_row.addStretch(1)

        self.rx_peak_lbl = QtWidgets.QLabel("RX PEAK: —")
        self.rx_avg_lbl  = QtWidgets.QLabel("RX AVG:  —")
        self.tx_peak_lbl = QtWidgets.QLabel("TX PEAK: —")
        self.tx_avg_lbl  = QtWidgets.QLabel("TX AVG:  —")
        for lbl in (self.rx_peak_lbl, self.rx_avg_lbl):
            lbl.setFont(HackerFont.mono(11))
            lbl.setStyleSheet("color:#00ff66;")
        for lbl in (self.tx_peak_lbl, self.tx_avg_lbl):
            lbl.setFont(HackerFont.mono(11))
            lbl.setStyleSheet("color:#ffcc33;")

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.title)
        top.addStretch(1)
        iface_lbl = QtWidgets.QLabel("INTERFACE:")
        iface_lbl.setFont(HackerFont.mono(10))
        iface_lbl.setStyleSheet("color:#00aa44;")
        top.addWidget(iface_lbl)
        top.addWidget(self.iface_combo)
        top.addSpacing(12)
        top.addWidget(self.snapshot_btn)

        stats_row = QtWidgets.QHBoxLayout()
        stats_row.addWidget(self.rx_peak_lbl)
        stats_row.addSpacing(20)
        stats_row.addWidget(self.rx_avg_lbl)
        stats_row.addSpacing(30)
        stats_row.addWidget(self.tx_peak_lbl)
        stats_row.addSpacing(20)
        stats_row.addWidget(self.tx_avg_lbl)
        stats_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addLayout(win_row)
        layout.addWidget(self.graph, 1)
        layout.addLayout(stats_row)

    def push_traffic(self, snap: dict):
        ts = time.time()
        self.graph.push(ts, snap)
        series = self.graph._get_series()
        if series:
            rx_vals = [rx for _, rx, _ in series]
            tx_vals = [tx for _, _, tx in series]
            n = len(rx_vals)
            self.rx_peak_lbl.setText(f"RX PEAK: {human_bps(max(rx_vals))}")
            self.tx_peak_lbl.setText(f"TX PEAK: {human_bps(max(tx_vals))}")
            self.rx_avg_lbl.setText(f"RX AVG:  {human_bps(sum(rx_vals) / n)}")
            self.tx_avg_lbl.setText(f"TX AVG:  {human_bps(sum(tx_vals) / n)}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, config: Optional[AppConfig] = None):
        super().__init__()
        self.cfg = config if config is not None else AppConfig()
        self.setWindowTitle(f"{APP_NAME} v{APP_VERSION} ({APP_BUILD}) — Traffic Visualizer")
        self.resize(1200, 780)

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
        self.conns = ConnectionsTab()
        self.map = MapTab()
        self.stats = StatsTab()

        self.tabs.addTab(self.dash, "BLACK ICE")
        self.tabs.addTab(self.conns, "CONTACTS")
        self.tabs.addTab(self.map, "MAP")
        self.tabs.addTab(self.stats, "STATS")

        self.dash.set_config_enabled(self.cfg.enabled)
        self.dash.configToggled.connect(self._on_config_toggled)

        self.dash.snapshotRequested.connect(lambda: self._snapshot(self.dash, "blackice_dashboard"))
        self.conns.snapshotRequested.connect(lambda: self._snapshot(self.conns, "blackice_contacts"))
        self.map.snapshotRequested.connect(lambda: self._snapshot(self.map, "blackice_map"))
        self.stats.snapshotRequested.connect(lambda: self._snapshot(self.stats, "blackice_stats"))
        self.conns.csvExportRequested.connect(self._export_csv)

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
        self.dash.set_event(f"[*] {APP_NAME} v{APP_VERSION} ({APP_BUILD}) initialized (PyQt6)")

        self._ui_timer = QtCore.QTimer(self)
        self._ui_timer.timeout.connect(self._ui_tick)
        self._ui_timer.start(5000)

        if self.cfg.enabled:
            self.restore_state()
            self.dash.set_event(f"[*] config restored ← {self.cfg.path}")

    # ---- optional configuration persistence -----------------------------

    def _on_config_toggled(self, on: bool):
        self.cfg.set_enabled(on)
        if on:
            # Persist immediately so the choice sticks even if the app is killed.
            self.save_state()
            self.dash.set_event(f"[*] config saving ENABLED → {self.cfg.path}")
        else:
            self.cfg.clear_state()
            self.dash.set_event("[*] config saving DISABLED (stored layout cleared)")

    def save_state(self):
        """Write window, layout and view state. A no-op while saving is off."""
        if not self.cfg.enabled:
            return
        header = self.conns.view.horizontalHeader()
        self.cfg.set("window/geometry", self.saveGeometry())
        self.cfg.set("window/tab", self.tabs.currentIndex())
        self.cfg.set("dash/iface", self.dash.iface.currentText())
        self.cfg.set("stats/iface", self.stats.iface_combo.currentText())
        self.cfg.set("stats/window", int(self.stats.graph._window))
        self.cfg.set("contacts/header", header.saveState())
        self.cfg.set("contacts/sort_col", header.sortIndicatorSection())
        self.cfg.set("contacts/sort_order", header.sortIndicatorOrder().value)
        self.cfg.set("contacts/filter", self.conns.filter_edit.text())
        self.cfg.set("map/splitter", self.map.split.saveState())
        self.cfg.set("map/show_me", self.map.me_enable.isChecked())
        self.cfg.sync()

    def restore_state(self):
        """Apply saved state. A no-op while saving is off."""
        if not self.cfg.enabled:
            return

        geo = self.cfg.get("window/geometry", None, QtCore.QByteArray)
        if geo:
            self.restoreGeometry(geo)
            self._ensure_on_screen()

        tab = self.cfg.get("window/tab", 0, int)
        if 0 <= tab < self.tabs.count():
            self.tabs.setCurrentIndex(tab)

        self._restore_combo(self.dash.iface, self.cfg.get("dash/iface", "", str))
        self._restore_combo(self.stats.iface_combo, self.cfg.get("stats/iface", "", str))

        rb = self.stats.win_buttons.get(self.cfg.get("stats/window", 0, int))
        if rb is not None:
            rb.setChecked(True)

        hdr_state = self.cfg.get("contacts/header", None, QtCore.QByteArray)
        header = self.conns.view.horizontalHeader()
        if hdr_state:
            header.restoreState(hdr_state)
        sort_col = self.cfg.get("contacts/sort_col", -1, int)
        if 0 <= sort_col < self.conns.model.columnCount():
            order = (Qt.SortOrder.DescendingOrder
                     if self.cfg.get("contacts/sort_order", 0, int) == 1
                     else Qt.SortOrder.AscendingOrder)
            self.conns.view.sortByColumn(sort_col, order)
        self.conns.filter_edit.setText(self.cfg.get("contacts/filter", "", str))

        split_state = self.cfg.get("map/splitter", None, QtCore.QByteArray)
        if split_state:
            self.map.split.restoreState(split_state)
        # Checking this kicks off a background locate, exactly as a click would.
        self.map.me_enable.setChecked(self.cfg.get("map/show_me", False, bool))

    @staticmethod
    def _restore_combo(combo: QtWidgets.QComboBox, name: str):
        idx = combo.findText(name) if name else -1
        if idx >= 0:
            combo.setCurrentIndex(idx)

    def _ensure_on_screen(self):
        """A saved geometry can point at a monitor that is no longer attached."""
        frame = self.frameGeometry()
        if any(s.availableGeometry().intersects(frame)
               for s in QtGui.QGuiApplication.screens()):
            return
        screen = QtGui.QGuiApplication.primaryScreen()
        if screen is not None:
            self.resize(1200, 780)
            self.move(screen.availableGeometry().center() - self.rect().center())

    def _ui_tick(self):
        try:
            nics = ["ALL"] + list(psutil.net_io_counters(pernic=True).keys())
            for combo in (self.dash.iface, self.stats.iface_combo):
                self._refresh_combo(combo, nics)
        except Exception:
            pass

    @staticmethod
    def _refresh_combo(combo: QtWidgets.QComboBox, items: List[str]):
        # Rebuild only on actual NIC changes so an open dropdown isn't closed
        # under the user every tick.
        current_items = [combo.itemText(i) for i in range(combo.count())]
        if current_items == items:
            return
        sel = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItems(items)
        idx = combo.findText(sel)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)
        if combo.currentText() != sel:
            # selection was lost (NIC vanished) — let listeners react
            combo.currentTextChanged.emit(combo.currentText())

    def _on_traffic(self, snap: dict):
        self.dash.update_traffic(snap)
        self.stats.push_traffic(snap)

    def _on_points(self, points: list):
        self.map.push_points(points)
        self.conns.add_points(points)

    def _snapshot(self, widget: QtWidgets.QWidget, stem: str):
        ts = time.strftime("%Y%m%d_%H%M%S")
        default = os.path.join(
            os.path.expanduser("~"), f"{stem}_{ts}.png"
        )
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Snapshot", default, "PNG Image (*.png)"
        )
        if not path:
            return
        pix = self._grab_widget(widget)
        ok = pix.save(path, "PNG")
        if ok:
            self.dash.set_event(f"[*] snapshot saved → {path}")
        else:
            self.dash.set_event(f"[!] snapshot FAILED → {path}")

    def _grab_widget(self, widget: QtWidgets.QWidget) -> QtGui.QPixmap:
        # QWebEngineView renders in a separate GPU process, so QWidget.grab()
        # captures it as a black rectangle. Grab the on-screen pixels instead
        # when the widget hosts a web view; fall back to grab() elsewhere
        # (e.g. Wayland, where screen capture is denied).
        has_web = HAVE_WEBENGINE and widget.findChild(QtWebEngineWidgets.QWebEngineView) is not None
        if has_web and widget.isVisible():
            handle = widget.window().windowHandle()
            screen = handle.screen() if handle else None
            if screen is not None:
                tl = widget.mapToGlobal(QtCore.QPoint(0, 0))
                pix = screen.grabWindow(0, tl.x(), tl.y(), widget.width(), widget.height())
                if not pix.isNull():
                    return pix
        return widget.grab()

    def _export_csv(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        default = os.path.join(os.path.expanduser("~"), f"blackice_contacts_{ts}.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Contacts CSV", default, "CSV (*.csv)"
        )
        if not path:
            return
        try:
            self.conns.export_csv(path)
            self.dash.set_event(f"[*] contacts CSV exported → {path}")
        except Exception as e:
            self.dash.set_event(f"[!] CSV export failed: {e}")

    def changeEvent(self, e):
        if e.type() == QtCore.QEvent.Type.WindowStateChange:
            minimized = bool(self.windowState() & Qt.WindowState.WindowMinimized)
            self.dash.set_fx_running(not minimized)
        super().changeEvent(e)

    def closeEvent(self, e):
        try:
            self.save_state()
        except Exception as exc:  # never block shutdown on a config write
            print(f"[!] could not save config: {exc}", file=sys.stderr)
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


def resource_path(*names: str) -> Optional[str]:
    """Locate a bundled resource across dev, PyInstaller and installed layouts."""
    bases = [
        getattr(sys, "_MEIPASS", ""),
        os.path.dirname(os.path.abspath(__file__)),
        os.path.dirname(os.path.abspath(sys.executable)),
        "/usr/lib/blackice_traffic",
        "/usr/share/blackice_traffic",
        os.path.expanduser("~/.local/share/blackice_traffic"),
        os.getcwd(),
    ]
    for base in bases:
        if not base:
            continue
        for name in names:
            for cand in (os.path.join(base, name), os.path.join(base, "resources", name)):
                if os.path.exists(cand):
                    return cand
    return None


def init_geoip():
    global _geoip_reader
    if not HAVE_GEOIP:
        return
    db = os.environ.get("GEOIP_DB", "").strip()
    candidates = []
    if db:
        candidates.append(db)
    candidates += [
        "./GeoLite2-City.mmdb",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "GeoLite2-City.mmdb"),
        os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "GeoLite2-City.mmdb"),
        "/usr/lib/blackice_traffic/GeoLite2-City.mmdb",
        "/usr/share/blackice_traffic/GeoLite2-City.mmdb",
        os.path.expanduser("~/.local/share/blackice_traffic/GeoLite2-City.mmdb"),
    ]
    db = next((p for p in candidates if p and os.path.exists(p)), None)
    if not db:
        return
    try:
        _geoip_reader = geoip2.database.Reader(db)
    except Exception:
        _geoip_reader = None


def init_asn():
    global _asn_reader, HAVE_ASN
    if not HAVE_GEOIP:
        return
    db = os.environ.get("ASN_DB", "").strip()
    candidates = []
    if db:
        candidates.append(db)
    candidates += [
        "./GeoLite2-ASN.mmdb",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "GeoLite2-ASN.mmdb"),
        os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "GeoLite2-ASN.mmdb"),
        "/usr/lib/blackice_traffic/GeoLite2-ASN.mmdb",
        "/usr/share/blackice_traffic/GeoLite2-ASN.mmdb",
        os.path.expanduser("~/.local/share/blackice_traffic/GeoLite2-ASN.mmdb"),
    ]
    db = next((p for p in candidates if p and os.path.exists(p)), None)
    if not db:
        return
    try:
        _asn_reader = geoip2.database.Reader(db)
        HAVE_ASN = True
    except Exception:
        _asn_reader = None


def main():
    init_geoip()
    init_asn()

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationDisplayName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    app.setDesktopFileName("blackice_traffic")
    app.setFont(HackerFont.mono(10))

    icon_path = resource_path("icon.png", "icon.ico")
    if icon_path:
        app.setWindowIcon(QtGui.QIcon(icon_path))

    w = MainWindow()
    if icon_path:
        w.setWindowIcon(QtGui.QIcon(icon_path))
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
