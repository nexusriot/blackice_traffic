#!/usr/bin/env python3
import collections
import csv
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
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

APP_NAME = "BLACK ICE"
APP_VERSION = "0.6.1"
APP_BUILD = "alpha"


try:
    from PyQt6 import QtWebEngineWidgets
    from PyQt6.QtWebEngineCore import QWebEnginePage  # noqa: F401
    HAVE_WEBENGINE = True
except Exception:
    HAVE_WEBENGINE = False

HAVE_GEOIP = False
_geoip_reader = None
try:
    import geoip2.database
    HAVE_GEOIP = True
except Exception:
    HAVE_GEOIP = False

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


class HackerFont:
    @staticmethod
    def mono(size=11, bold=False):
        f = QtGui.QFont("DejaVu Sans Mono", size)
        f.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        f.setBold(bold)
        return f


class ScanlinesOverlay(QtWidgets.QWidget):
    """Transparent scanlines + subtle flicker overlay."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self._phase = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

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


class MatrixRain(QtWidgets.QWidget):
    """Simple matrix-rain background."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

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
        if ip.startswith("::ffff:"):
            v4 = ip.split("::ffff:", 1)[1]
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
            return True

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
        for c in conns:
            if not c.raddr:
                continue
            ip = c.raddr.ip
            port = int(c.raddr.port)
            proto = "tcp" if c.type == socket.SOCK_STREAM else "udp"
            key = f"{proto}:{ip}:{port}"
            if ip.startswith("127.") or ip == "::1":
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

            norm_ip = self._normalize_ip(ip)
            lat, lon, where = self._geo_lookup(norm_ip)
            asn = self._asn_lookup(norm_ip) if not self._is_privateish(norm_ip) else ""
            label = f"{norm_ip}:{port} ({proto}) — {where}"
            out.append(ConnPoint(ip=norm_ip, port=port, proto=proto,
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


def _hacker_button(text: str) -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    b.setFont(HackerFont.mono(10))
    b.setStyleSheet(
        "QPushButton { background:#07100a; color:#00ff66; border:1px solid #0b2a12; padding:6px 10px; }"
        "QPushButton:hover { border:1px solid #00ff66; }"
    )
    return b


class BlackIceDashboard(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()

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

  window.BLACKICE = { upsertPoints, setMyLocation, redrawRays, clearRays };
</script>
</body>
</html>
"""


class MapTab(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()

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

        if HAVE_WEBENGINE:
            self.web = QtWebEngineWidgets.QWebEngineView()
            self.web.setHtml(LEAFLET_HTML)
        else:
            self.web = QtWidgets.QLabel(
                "QtWebEngine not installed.\n\nInstall PyQt6-WebEngine to enable the MAP tab."
            )
            self.web.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.web.setFont(HackerFont.mono(11))
            self.web.setStyleSheet("color:#ffcc33; background:#07100a; border:1px solid #0b2a12; padding:20px;")

        self.list = QtWidgets.QPlainTextEdit()
        self.list.setReadOnly(True)
        self.list.setFont(HackerFont.mono(10))
        self.list.setStyleSheet(
            "QPlainTextEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; }"
        )

        split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)
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
        self.me_enable.setStyleSheet("color:#ffcc33;")

        self.me_refresh = QtWidgets.QPushButton("Locate me now")
        self.me_refresh.setFont(HackerFont.mono(10))
        self.me_refresh.setStyleSheet(
            "QPushButton { background:#07100a; color:#ff9900; border:1px solid #0b2a12; padding:6px 10px; }"
            "QPushButton:hover { border:1px solid #ff9900; }"
        )

        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.me_enable)
        row.addWidget(self.me_refresh)
        row.addStretch(1)
        row.addWidget(self.snapshot_btn)

        layout.addLayout(row)
        self.me_enable.toggled.connect(self._on_me_toggled)
        self.me_refresh.clicked.connect(self.locate_me)
        self._me_obj = None

    def _js(self, code: str):
        if HAVE_WEBENGINE and isinstance(self.web, QtWebEngineWidgets.QWebEngineView):
            self.web.page().runJavaScript(code)

    def _get_public_ip(self) -> Optional[str]:
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
        lat = lon = None
        label = "ME"

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
                        lat = float(lat)
                        lon = float(lon)
                except Exception:
                    lat = lon = None

        if lat is None or lon is None:
            lat, lon, label2 = self._geo_online_ipapi()
            label = label2 or label

        if lat is None or lon is None:
            self.list.appendPlainText(f"{time.strftime('%H:%M:%S')}  [!] failed to locate ME")
            self._me_obj = None
            return

        self._me_obj = {"lat": lat, "lon": lon, "label": f"ME — {label}"}
        self.list.appendPlainText(f"{time.strftime('%H:%M:%S')}  [*] ME located: {label} @ ({lat:.3f},{lon:.3f})")

        if self.me_enable.isChecked():
            payload = json.dumps(self._me_obj)
            self._js(f"window.BLACKICE && window.BLACKICE.setMyLocation({payload}); window.BLACKICE.redrawRays();")

    def _on_me_toggled(self, enabled: bool):
        if not enabled:
            self._js("window.BLACKICE && window.BLACKICE.clearRays();")
            return
        if self._me_obj is None:
            self.locate_me()
        else:
            payload = json.dumps(self._me_obj)
            self._js(f"window.BLACKICE && window.BLACKICE.setMyLocation({payload}); window.BLACKICE.redrawRays();")

    def push_points(self, points: List[dict]):
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

            if self.me_enable.isChecked() and self._me_obj is not None:
                self._js("window.BLACKICE && window.BLACKICE.redrawRays();")


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

        self.proxy = QtCore.QSortFilterProxyModel(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.proxy.setFilterKeyColumn(-1)
        self.filter_edit.textChanged.connect(self.proxy.setFilterFixedString)

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
        self.count_lbl.setText("0 contacts")

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
                self._make_item(ip),
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

        self.count_lbl.setText(f"{self.model.rowCount()} contacts")

    def export_csv(self, path: str):
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            for r in range(self.model.rowCount()):
                w.writerow([self.model.item(r, c).text() for c in range(self.model.columnCount())])


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
    def __init__(self):
        super().__init__()
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

    def _ui_tick(self):
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
        pix = widget.grab()
        ok = pix.save(path, "PNG")
        if ok:
            self.dash.set_event(f"[*] snapshot saved → {path}")
        else:
            self.dash.set_event(f"[!] snapshot FAILED → {path}")

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
    app.setFont(HackerFont.mono(10))
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
