#!/usr/bin/env python3
import argparse
import collections
import csv
import os
import shutil
import sys
import math
import threading
import tempfile
import time
import json
import random
import socket
import sqlite3
import struct
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
APP_VERSION = "0.11.0"
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


_VIRTUAL_NIC_PREFIXES = (
    "docker", "veth", "br-", "virbr", "vmnet", "vboxnet", "tun", "tap",
    "wg", "cni", "flannel", "cali", "kube", "podman", "lxc", "lxd", "zt",
)


def is_virtual_nic(name: str) -> bool:
    """True for container/VM/tunnel interfaces whose bytes are usually also
    counted on the physical NIC they ride over. Including them in an aggregate
    double-counts the same packets, so the dashboard can hide them — the
    heuristic is name-based and therefore the user's call, not the app's."""
    n = (name or "").strip().lower()
    if not n or is_loopback_nic(n):
        return False
    return n.startswith(_VIRTUAL_NIC_PREFIXES)


EXPOSURE_WORLD = "WORLD"
EXPOSURE_LAN = "LAN"
EXPOSURE_LOCAL = "LOCAL"
EXPOSURE_UNKNOWN = "?"


def exposure_class(ip: str) -> str:
    """How far a listening socket can be reached from.

    A wildcard bind (0.0.0.0 / ::) accepts from every interface the host has,
    so it is the one worth shouting about; a loopback bind cannot be reached
    off the machine at all."""
    addr = (ip or "").strip()
    if addr in ("", "*"):
        return EXPOSURE_WORLD
    addr = normalize_ip(addr)
    try:
        parsed = ipaddress.ip_address(addr)
    except ValueError:
        return EXPOSURE_UNKNOWN
    if parsed.is_unspecified:
        return EXPOSURE_WORLD
    if parsed.is_loopback:
        return EXPOSURE_LOCAL
    if parsed.is_link_local or parsed.is_private:
        return EXPOSURE_LAN
    return EXPOSURE_WORLD


# Ports that are a problem specifically when they answer the whole world.
_SENSITIVE_PORTS = {
    22: "remote shell", 23: "cleartext login", 445: "file sharing",
    3389: "remote desktop", 5900: "remote desktop", 3306: "database",
    5432: "database", 6379: "database (often unauthenticated)",
    9200: "search cluster", 27017: "database (often unauthenticated)",
    2375: "docker daemon (root-equivalent)", 2376: "docker daemon",
    11211: "memcached (amplification)",
}


def listener_risk(port: int, exposure: str) -> str:
    """A short note for a listening socket, or "" when there is nothing to say.
    Only wildcard binds are flagged: the same port on 127.0.0.1 is normal."""
    if exposure != EXPOSURE_WORLD:
        return ""
    note = _SENSITIVE_PORTS.get(int(port), "")
    if note:
        return note.upper()
    return ""


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
    state: str = ""
    country: str = ""
    host: str = ""
    location: str = ""


@dataclass
class ListenPoint:
    """A socket this host is accepting on. Unlike a contact this is a state,
    not an event: every scan replaces the whole set."""
    ip: str
    port: int
    proto: str
    process: str = ""
    pid: int = 0
    exposure: str = EXPOSURE_UNKNOWN
    note: str = ""

    @property
    def key(self) -> str:
        return f"{self.proto}:{self.ip}:{self.port}"


RULE_KINDS = ("cidr", "port", "asn", "country", "process", "host")


@dataclass
class WatchRule:
    kind: str
    value: str
    label: str = ""

    def describe(self) -> str:
        return self.label or f"{self.kind.upper()} {self.value}"


class RuleSet:
    """Watchlist rules loaded from a plain, hand-editable text file.

    Deliberately not a threat feed: a bundled list of "bad" addresses is stale
    the day it ships and cannot be checked offline. What the user writes down
    is theirs, is auditable, and stays true."""

    def __init__(self, rules: Optional[List[WatchRule]] = None):
        self.rules: List[WatchRule] = list(rules or [])
        self.errors: List[str] = []
        self._nets: List[Tuple[ipaddress._BaseNetwork, WatchRule]] = []
        self._reindex()

    def _reindex(self):
        self._nets = []
        for r in self.rules:
            if r.kind != "cidr":
                continue
            try:
                self._nets.append((ipaddress.ip_network(r.value, strict=False), r))
            except ValueError:
                continue

    def __len__(self) -> int:
        return len(self.rules)

    @classmethod
    def parse(cls, text: str) -> "RuleSet":
        """`kind value [label...]`, one per line; # starts a comment.
        A malformed line is reported and skipped — never fatal, because this
        file is edited by hand while the app is running."""
        rules: List[WatchRule] = []
        errors: List[str] = []
        for lineno, raw in enumerate((text or "").splitlines(), start=1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            kind = parts[0].lower()
            if kind not in RULE_KINDS:
                errors.append(f"line {lineno}: unknown rule kind {parts[0]!r}")
                continue
            if len(parts) < 2:
                errors.append(f"line {lineno}: {kind} rule has no value")
                continue
            value = parts[1]
            if kind == "cidr":
                try:
                    ipaddress.ip_network(value, strict=False)
                except ValueError:
                    errors.append(f"line {lineno}: {value!r} is not a network")
                    continue
            if kind == "port":
                try:
                    int(value)
                except ValueError:
                    errors.append(f"line {lineno}: {value!r} is not a port")
                    continue
            rules.append(WatchRule(kind, value, " ".join(parts[2:]).strip()))
        rs = cls(rules)
        rs.errors = errors
        return rs

    @classmethod
    def load(cls, path: str) -> "RuleSet":
        try:
            with open(path, encoding="utf-8") as f:
                return cls.parse(f.read())
        except FileNotFoundError:
            return cls()
        except OSError as e:
            rs = cls()
            rs.errors = [f"{path}: {e}"]
            return rs

    def match(self, point: dict) -> List[str]:
        """Labels of every rule this contact trips, in file order."""
        hits: List[str] = []
        ip = str(point.get("ip", ""))
        port = str(point.get("port", ""))
        asn = str(point.get("asn", "")).upper()
        country = str(point.get("country", "")).upper()
        process = str(point.get("process", "")).lower()
        host = str(point.get("host", "")).lower()

        parsed = None
        try:
            parsed = ipaddress.ip_address(ip)
        except ValueError:
            pass

        for rule in self.rules:
            val = rule.value
            hit = False
            if rule.kind == "cidr":
                if parsed is not None:
                    for net, r in self._nets:
                        if r is rule and parsed.version == net.version and parsed in net:
                            hit = True
                            break
            elif rule.kind == "port":
                hit = port == str(int(val)) if val.isdigit() else False
            elif rule.kind == "asn":
                hit = val.upper() in asn and bool(asn)
            elif rule.kind == "country":
                hit = bool(country) and country == val.upper()
            elif rule.kind == "process":
                hit = bool(process) and val.lower() in process
            elif rule.kind == "host":
                hit = bool(host) and val.lower() in host
            if hit:
                hits.append(rule.describe())
        return hits


def default_watchlist_path() -> str:
    return os.path.join(
        os.path.dirname(app_settings().fileName()), "watchlist.txt"
    )


WATCHLIST_TEMPLATE = """# BLACK ICE watchlist — one rule per line, # starts a comment.
#
#   kind      value              label (optional, shown in the FLAGS column)
#   ------------------------------------------------------------------------
#   cidr      185.220.100.0/22   TOR EXIT RANGE
#   port      3389               RDP
#   asn       AS13335            CLOUDFLARE
#   country   RU
#   process   nc
#   host      .example.com
#
# Nothing here is shipped as a threat feed: these are your rules, and the app
# never fetches or updates them.
"""


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


class RDnsCache:
    """Bounded reverse-DNS cache with negative entries.

    A miss is cached as "" on purpose: most addresses have no PTR record, and
    without that the same dead lookup would be re-queued on every scan."""

    def __init__(self, capacity: int = 4096, ttl: float = 1800.0):
        self.capacity = capacity
        self.ttl = ttl
        self._items: "collections.OrderedDict[str, Tuple[str, float]]" = collections.OrderedDict()
        # Three threads touch this: the resolver writes, the scanner and the
        # GUI read. Without the lock an eviction between a lookup and the
        # following move_to_end raises KeyError.
        self._lock = threading.Lock()

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)

    def get(self, ip: str) -> Optional[str]:
        """The hostname, "" for a known-nameless address, None if unknown."""
        with self._lock:
            item = self._items.get(ip)
            if item is None:
                return None
            host, ts = item
            if time.time() - ts > self.ttl:
                del self._items[ip]
                return None
            self._items.move_to_end(ip)
            return host

    def put(self, ip: str, host: str):
        with self._lock:
            self._items[ip] = (host or "", time.time())
            self._items.move_to_end(ip)
            while len(self._items) > self.capacity:
                self._items.popitem(last=False)


class RDnsResolver(QtCore.QThread):
    """Resolves PTR records off the GUI thread.

    gethostbyaddr can block for seconds against an unreachable resolver, so it
    can run neither on the GUI thread nor on the scanner thread, whose 3-second
    cadence is what keeps the contact table live."""

    resolved = QtCore.pyqtSignal(str, str)

    MAX_QUEUE = 512

    def __init__(self, cache: Optional[RDnsCache] = None, parent=None):
        super().__init__(parent)
        self.cache = cache if cache is not None else RDnsCache()
        self._queue: "collections.deque[str]" = collections.deque()
        self._queued: set = set()
        self._stop_evt = QtCore.QWaitCondition()
        self._mtx = QtCore.QMutex()
        self._stop = False

    def request(self, ips: List[str]):
        """Queue addresses that are neither cached nor already pending."""
        self._mtx.lock()
        try:
            for ip in ips:
                if not ip or ip in self._queued or self.cache.get(ip) is not None:
                    continue
                if len(self._queue) >= self.MAX_QUEUE:
                    break
                self._queue.append(ip)
                self._queued.add(ip)
            self._stop_evt.wakeAll()
        finally:
            self._mtx.unlock()

    def stop(self):
        self._mtx.lock()
        self._stop = True
        self._stop_evt.wakeAll()
        self._mtx.unlock()

    @staticmethod
    def _lookup(ip: str) -> str:
        try:
            return socket.gethostbyaddr(ip)[0]
        except (OSError, socket.herror, socket.gaierror):
            return ""

    def _next(self) -> Optional[str]:
        self._mtx.lock()
        try:
            while not self._stop and not self._queue:
                self._stop_evt.wait(self._mtx, 500)
            if self._stop:
                return None
            return self._queue.popleft()
        finally:
            self._mtx.unlock()

    def run(self):
        while not self._stop:
            ip = self._next()
            if ip is None:
                break
            host = self._lookup(ip)
            self._mtx.lock()
            self._queued.discard(ip)
            self._mtx.unlock()
            self.cache.put(ip, host)
            if host:
                self.resolved.emit(ip, host)


def default_history_path() -> str:
    return os.path.join(
        os.path.expanduser("~"), ".local", "share", "blackice", "history.db"
    )


class SessionStore:
    """Optional on-disk history of contacts and bandwidth.

    Off by default and the file is not even created until the user turns it
    on: recording who this machine talks to is exactly the kind of thing that
    should be a deliberate act, matching how config saving already works."""

    SCHEMA = (
        "CREATE TABLE IF NOT EXISTS contacts ("
        " ip TEXT NOT NULL, port INTEGER NOT NULL, proto TEXT NOT NULL,"
        " first_seen REAL NOT NULL, last_seen REAL NOT NULL,"
        " hits INTEGER NOT NULL DEFAULT 1, location TEXT DEFAULT '',"
        " country TEXT DEFAULT '', asn TEXT DEFAULT '', process TEXT DEFAULT '',"
        " host TEXT DEFAULT '', PRIMARY KEY (ip, port, proto))",
        "CREATE INDEX IF NOT EXISTS contacts_last_seen ON contacts(last_seen)",
        "CREATE TABLE IF NOT EXISTS samples ("
        " ts REAL PRIMARY KEY, rx_bps REAL NOT NULL, tx_bps REAL NOT NULL)",
        "CREATE TABLE IF NOT EXISTS sessions ("
        " started REAL PRIMARY KEY, version TEXT)",
    )

    RETENTION_DAYS = 30

    def __init__(self, path: Optional[str] = None):
        self.path = path or default_history_path()
        self._db: Optional[sqlite3.Connection] = None
        self.baseline: set = set()
        self.error = ""

    @property
    def enabled(self) -> bool:
        return self._db is not None

    def open(self) -> bool:
        """Create/attach the database and snapshot the baseline. Returns False
        (and records .error) rather than raising: history is a nice-to-have and
        must never stop the app from starting."""
        if self._db is not None:
            return True
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            db = sqlite3.connect(self.path)
            db.execute("PRAGMA journal_mode=WAL")
            for stmt in self.SCHEMA:
                db.execute(stmt)
            db.commit()
        except (OSError, sqlite3.Error) as e:
            self.error = str(e)
            return False
        self._db = db
        # The baseline has to be read before this session writes anything,
        # otherwise every contact would immediately look already-known.
        self.baseline = self._read_baseline()
        try:
            db.execute("INSERT OR REPLACE INTO sessions(started, version) VALUES (?,?)",
                       (time.time(), APP_VERSION))
            db.commit()
            self.purge()
        except sqlite3.Error as e:
            self.error = str(e)
        return True

    def close(self):
        if self._db is None:
            return
        try:
            self._db.commit()
            self._db.close()
        except sqlite3.Error:
            pass
        self._db = None

    def _read_baseline(self) -> set:
        try:
            return {row[0] for row in self._db.execute("SELECT DISTINCT ip FROM contacts")}
        except sqlite3.Error:
            return set()

    def is_new(self, ip: str) -> bool:
        """True for an address this store has never seen in an earlier run."""
        return bool(ip) and ip not in self.baseline

    def record_contacts(self, points: List[dict]):
        if self._db is None:
            return
        try:
            for p in points:
                ts = float(p.get("ts", time.time()))
                self._db.execute(
                    "INSERT INTO contacts(ip, port, proto, first_seen, last_seen, hits,"
                    " location, country, asn, process, host) VALUES (?,?,?,?,?,1,?,?,?,?,?)"
                    " ON CONFLICT(ip, port, proto) DO UPDATE SET"
                    " last_seen=excluded.last_seen, hits=hits+1,"
                    " location=CASE WHEN excluded.location != '' THEN excluded.location ELSE location END,"
                    " country=CASE WHEN excluded.country != '' THEN excluded.country ELSE country END,"
                    " asn=CASE WHEN excluded.asn != '' THEN excluded.asn ELSE asn END,"
                    " process=CASE WHEN excluded.process != '' THEN excluded.process ELSE process END,"
                    " host=CASE WHEN excluded.host != '' THEN excluded.host ELSE host END",
                    (str(p.get("ip", "")), int(p.get("port", 0)), str(p.get("proto", "")),
                     ts, ts, str(p.get("location", "") or ""), str(p.get("country", "") or ""),
                     str(p.get("asn", "") or ""), str(p.get("process", "") or ""),
                     str(p.get("host", "") or "")),
                )
            self._db.commit()
        except sqlite3.Error as e:
            self.error = str(e)

    def record_sample(self, ts: float, rx_bps: float, tx_bps: float):
        if self._db is None:
            return
        try:
            self._db.execute(
                "INSERT OR REPLACE INTO samples(ts, rx_bps, tx_bps) VALUES (?,?,?)",
                (float(ts), float(rx_bps), float(tx_bps)))
            self._db.commit()
        except sqlite3.Error as e:
            self.error = str(e)

    def set_host(self, ip: str, host: str):
        if self._db is None:
            return
        try:
            self._db.execute("UPDATE contacts SET host=? WHERE ip=?", (host, ip))
            self._db.commit()
        except sqlite3.Error:
            pass

    def purge(self, retention_days: Optional[int] = None):
        days = self.RETENTION_DAYS if retention_days is None else retention_days
        if self._db is None:
            return
        cutoff = time.time() - days * 86400
        try:
            self._db.execute("DELETE FROM contacts WHERE last_seen < ?", (cutoff,))
            self._db.execute("DELETE FROM samples WHERE ts < ?", (cutoff,))
            self._db.commit()
        except sqlite3.Error:
            pass

    def counts(self) -> Tuple[int, int]:
        """(contacts, bandwidth samples) currently stored."""
        if self._db is None:
            return (0, 0)
        try:
            c = self._db.execute("SELECT COUNT(*) FROM contacts").fetchone()[0]
            s = self._db.execute("SELECT COUNT(*) FROM samples").fetchone()[0]
            return (int(c), int(s))
        except sqlite3.Error:
            return (0, 0)

    def recent(self, limit: int = 500) -> List[dict]:
        if self._db is None:
            return []
        try:
            cur = self._db.execute(
                "SELECT ip, port, proto, first_seen, last_seen, hits, location,"
                " country, asn, process, host FROM contacts"
                " ORDER BY last_seen DESC LIMIT ?", (int(limit),))
            cols = [d[0] for d in cur.description]
            return [dict(zip(cols, row)) for row in cur.fetchall()]
        except sqlite3.Error:
            return []


class HackerFont:
    @staticmethod
    def mono(size=11, bold=False):
        f = QtGui.QFont("DejaVu Sans Mono", size)
        f.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        f.setBold(bold)
        return f


FX_LEVELS = ("OFF", "SUBTLE", "FULL")
_FX_GAIN = {"OFF": 0.0, "SUBTLE": 0.45, "FULL": 1.0}


class AnimatedOverlay(QtWidgets.QWidget):
    """Transparent overlay whose repaint timer only ticks while visible."""
    interval_ms = 33

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.gain = 1.0
        self.level = "FULL"
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)

    def set_intensity(self, level: str):
        """OFF hides the overlay outright, which also stops its timer through
        hideEvent — an invisible effect should not be burning a repaint every
        33 ms."""
        self.level = level if level in FX_LEVELS else "FULL"
        self.gain = _FX_GAIN[self.level]
        self.setVisible(self.gain > 0.0)
        self.update()

    def _tick(self):
        raise NotImplementedError

    def set_running(self, run: bool):
        if self.gain <= 0.0:
            run = False
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
        p.setOpacity(0.18 * self.gain)
        pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 255))
        for y in range(0, h, 3):
            alpha = 35 + int(20 * (0.5 + 0.5 * math.sin((y * 0.08) + self._phase)))
            c = QtGui.QColor(0, 0, 0, alpha)
            pen.setColor(c)
            p.setPen(pen)
            p.drawLine(0, y, w, y)

        p.setOpacity(0.25 * self.gain)
        grad = QtGui.QRadialGradient(w * 0.5, h * 0.5, max(w, h) * 0.75)
        grad.setColorAt(0.0, QtGui.QColor(0, 0, 0, 0))
        grad.setColorAt(1.0, QtGui.QColor(0, 0, 0, 220))
        p.fillRect(self.rect(), grad)

        p.setOpacity((0.04 + random.random() * 0.03) * self.gain)
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

        p.setOpacity(0.20 * self.gain)
        for c in self.cols:
            x = c["x"]
            y = c["y"]
            for i in range(c["len"]):
                ch = random.choice(self.char_set)
                yy = y - i * 14
                if 0 <= yy <= self.height():
                    if i == 0:
                        p.setPen(QtGui.QPen(PHOSPHOR))
                        p.setOpacity(0.30 * self.gain)
                    else:
                        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
                        p.setOpacity(0.16 * self.gain)
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


def rollup_counts(rows: List[dict], field: str, limit: int = 8) -> List[Tuple[str, int]]:
    """Top `limit` values of `field` by row count, ties broken alphabetically
    so the panel does not reshuffle between equal entries every refresh."""
    counter: Dict[str, int] = {}
    for row in rows:
        val = str(row.get(field, "") or "").strip()
        if not val:
            continue
        counter[val] = counter.get(val, 0) + 1
    ordered = sorted(counter.items(), key=lambda kv: (-kv[1], kv[0]))
    return ordered[:limit]


def build_snapshot(prev: dict, now: dict, dt: float, hide_virtual: bool = False) -> dict:
    """Per-NIC rates plus the aggregate under "_totals". Loopback NICs are
    reported individually but kept out of the aggregate; container/VM
    interfaces optionally too, since their bytes are usually also counted on
    the physical NIC underneath."""
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
        snap[nic]["virtual"] = is_virtual_nic(nic)
        if is_loopback_nic(nic):
            continue
        if hide_virtual and is_virtual_nic(nic):
            continue
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
        self.hide_virtual = False

    def stop(self):
        self._mtx.lock()
        self._stop = True
        self._stop_evt.wakeAll()
        self._mtx.unlock()

    def set_hide_virtual(self, on: bool):
        self.hide_virtual = bool(on)

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

            snap = build_snapshot(self._prev, now, dt, hide_virtual=self.hide_virtual)
            self._prev = now
            prev_t = now_t
            self.traffic.emit(snap)


class ConnScanner(QtCore.QThread):
    points = QtCore.pyqtSignal(list)
    listeners = QtCore.pyqtSignal(list)
    states = QtCore.pyqtSignal(dict)
    event = QtCore.pyqtSignal(str)

    def __init__(self, interval=3.0, parent=None, rdns: Optional[RDnsCache] = None):
        super().__init__(parent)
        self.interval = interval
        self._stop_evt = QtCore.QWaitCondition()
        self._mtx = QtCore.QMutex()
        self._stop = False
        self._seen: Dict[str, float] = {}
        # Read-only here: the resolver thread fills it, the scanner only
        # stamps whatever name is already known onto a fresh contact.
        self.rdns = rdns if rdns is not None else RDnsCache()

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

    def _geo_lookup(self, ip: str) -> Tuple[Optional[float], Optional[float], str, str]:
        """(lat, lon, label, country). The country code is returned separately
        because rollups and `country` watchlist rules must not have to parse it
        back out of a human-readable label."""
        ip = self._normalize_ip(ip)

        if self._is_privateish(ip):
            try:
                addr = ipaddress.ip_address(ip)
                if addr.is_loopback:
                    return None, None, "LOCAL LOOPBACK", ""
                if addr.is_private:
                    return None, None, "LOCAL RFC1918 / PRIVATE", ""
                if addr.is_link_local:
                    return None, None, "LOCAL LINK-LOCAL", ""
            except Exception:
                pass
            return None, None, "LOCAL / NON-PUBLIC", ""

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
                    return None, None, label, country

                return float(lat), float(lon), label, country
            except Exception:
                pass

        return None, None, "GeoIP unavailable", ""

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

    @staticmethod
    def _proc_name(pid: Optional[int]) -> str:
        if not pid:
            return ""
        try:
            return psutil.Process(pid).name()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            return ""

    def _scan_psutil(self) -> Tuple[List[ConnPoint], List[ListenPoint], Dict[str, int]]:
        """One pass over the socket table: new remote contacts, the current
        listening set, and a histogram of connection states. All three come
        from the same syscall — the listeners and the states used to be
        discarded, which is most of what psutil actually hands us."""
        out: List[ConnPoint] = []
        listening: List[ListenPoint] = []
        state_counts: Dict[str, int] = {}
        try:
            conns = psutil.net_connections(kind="inet")
        except Exception as e:
            self.event.emit(f"[!] net_connections failed: {e}")
            return out, listening, state_counts

        now = time.time()
        cutoff = now - 600
        self._seen = {k: t for k, t in self._seen.items() if t >= cutoff}
        for c in conns:
            status = (c.status or "").upper()
            if status and status != "NONE":
                state_counts[status] = state_counts.get(status, 0) + 1

            if not c.raddr:
                if status == psutil.CONN_LISTEN and c.laddr:
                    lip = self._normalize_ip(c.laddr.ip)
                    lport = int(c.laddr.port)
                    exposure = exposure_class(c.laddr.ip)
                    listening.append(ListenPoint(
                        ip=lip, port=lport,
                        proto="tcp" if c.type == socket.SOCK_STREAM else "udp",
                        process=self._proc_name(c.pid), pid=int(c.pid or 0),
                        exposure=exposure, note=listener_risk(lport, exposure),
                    ))
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

            proc_name = self._proc_name(c.pid)

            lat, lon, where, country = self._geo_lookup(ip)
            asn = self._asn_lookup(ip) if not self._is_privateish(ip) else ""
            label = f"{ip}:{port} ({proto}) — {where}"
            out.append(ConnPoint(ip=ip, port=port, proto=proto,
                                 lat=lat or 0.0, lon=lon or 0.0, label=label, ts=now,
                                 process=proc_name, asn=asn, state=status,
                                 country=country, host=self.rdns.get(ip) or "",
                                 location=where))

        return out, listening, state_counts

    def run(self):
        while not self._stop:
            pts, listening, states = self._scan_psutil()
            self.listeners.emit([lp.__dict__ for lp in listening])
            self.states.emit(states)
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


_STATE_COLORS = {
    "ESTABLISHED": PHOSPHOR,
    "LISTEN": PHOSPHOR_DIM,
    "TIME_WAIT": PHOSPHOR_DIM,
    "SYN_SENT": AMBER,
    "SYN_RECV": AMBER,
    "FIN_WAIT1": AMBER,
    "FIN_WAIT2": AMBER,
    "LAST_ACK": AMBER,
    "CLOSING": AMBER,
    "CLOSE_WAIT": RED,
}


def state_color(state: str) -> QtGui.QColor:
    """CLOSE_WAIT is red because a pile of them is a socket leak in some
    process; SYN_SENT is amber because a pile of those is something failing to
    connect. Everything ordinary stays phosphor."""
    return _STATE_COLORS.get((state or "").upper(), PHOSPHOR_DIM)


class StateBar(QtWidgets.QWidget):
    """Stacked histogram of TCP connection states.

    psutil returns a status for every socket and the app used to throw it
    away, yet it is the one field that says whether the network is healthy:
    SYN_SENT means connects are not completing, CLOSE_WAIT means somebody is
    not closing their sockets."""

    ORDER = ["ESTABLISHED", "SYN_SENT", "SYN_RECV", "FIN_WAIT1", "FIN_WAIT2",
             "TIME_WAIT", "CLOSE_WAIT", "LAST_ACK", "CLOSING", "LISTEN"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(46)
        self._counts: Dict[str, int] = {}

    def set_counts(self, counts: Dict[str, int]):
        self._counts = dict(counts or {})
        self.update()

    def ordered(self) -> List[Tuple[str, int]]:
        """Known states in protocol order first, then anything unexpected."""
        known = [(s, self._counts[s]) for s in self.ORDER if self._counts.get(s)]
        extra = sorted((s, n) for s, n in self._counts.items()
                       if n and s not in self.ORDER)
        return known + extra

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), BG)
        p.setFont(HackerFont.mono(9))

        rows = self.ordered()
        total = sum(n for _, n in rows)
        w, h = self.width(), self.height()
        bar_h = 12

        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        if not total:
            p.drawText(2, 12, "SOCKET STATES: —")
            return
        p.drawText(2, 12, f"SOCKET STATES ({total} sockets)")

        x = 0.0
        for state, n in rows:
            seg = (n / total) * w
            colour = state_color(state)
            p.fillRect(QtCore.QRectF(x, 18, max(1.0, seg - 1), bar_h), colour)
            x += seg

        legend = "  ".join(f"{s} {n}" for s, n in rows)
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        p.drawText(QtCore.QRectF(2, 32, w - 4, 14),
                   int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                   legend)


class RollupPanel(QtWidgets.QWidget):
    """Top-N bar list — the cheapest analytics available, since every value it
    draws is already in the contact table."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self._rows: List[Tuple[str, int]] = []
        self.setMinimumHeight(140)

    def set_rows(self, rows: List[Tuple[str, int]]):
        self._rows = list(rows or [])
        self.update()

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), BG)
        p.setPen(QtGui.QPen(GRID))
        p.drawRect(self.rect().adjusted(0, 0, -1, -1))

        p.setFont(HackerFont.mono(9, bold=True))
        p.setPen(QtGui.QPen(PHOSPHOR))
        p.drawText(8, 16, self.title)

        p.setFont(HackerFont.mono(9))
        if not self._rows:
            p.setPen(QtGui.QPen(PHOSPHOR_DIM))
            p.drawText(8, 34, "no data yet")
            return

        top = max(n for _, n in self._rows)
        y = 26
        w = self.width()
        for name, n in self._rows:
            if y + 14 > self.height():
                break
            frac = n / top if top else 0.0
            p.fillRect(QtCore.QRectF(8, y, max(1.0, frac * (w - 16)), 12),
                       QtGui.QColor(0, 170, 68, 90))
            p.setPen(QtGui.QPen(PHOSPHOR))
            p.drawText(QtCore.QRectF(12, y, w - 70, 12),
                       int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                       name)
            p.setPen(QtGui.QPen(AMBER))
            p.drawText(QtCore.QRectF(w - 58, y, 50, 12),
                       int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter),
                       str(n))
            y += 15


class ListenersTab(QtWidgets.QWidget):
    """What this host is accepting on — the sockets the scanner used to skip.

    A remote contact says who the machine reached out to; this says who can
    reach in, which for anything calling itself a defender is the more
    important half."""

    COLS = ["Proto", "Bind", "Port/Svc", "Exposure", "Process", "PID", "Note"]
    C_PROTO, C_BIND, C_PORT, C_EXPOSURE, C_PROCESS, C_PID, C_NOTE = range(7)

    snapshotRequested = QtCore.pyqtSignal()
    csvExportRequested = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
        self.setPalette(pal)

        self.title = QtWidgets.QLabel("ATTACK SURFACE — LISTENING SOCKETS")
        self.title.setFont(HackerFont.mono(14, bold=True))
        self.title.setStyleSheet("color:#00ff66;")

        self.summary = QtWidgets.QLabel("—")
        self.summary.setFont(HackerFont.mono(10))
        self.summary.setStyleSheet("color:#00aa44;")

        self.world_only = QtWidgets.QCheckBox("WORLD-REACHABLE ONLY")
        self.world_only.setFont(HackerFont.mono(10))
        self.world_only.setStyleSheet("QCheckBox { color:#ffcc33; }")

        self.export_btn = _hacker_button("⤓ EXPORT CSV")
        self.export_btn.clicked.connect(self.csvExportRequested.emit)

        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)

        self.model = QtGui.QStandardItemModel(0, len(self.COLS), self)
        self.model.setHorizontalHeaderLabels(self.COLS)
        self.proxy = ContactSortProxy(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.proxy.setFilterKeyColumn(self.C_EXPOSURE)

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
        self.view.sortByColumn(self.C_PORT, Qt.SortOrder.AscendingOrder)
        for col, width in ((self.C_PROTO, 60), (self.C_BIND, 150), (self.C_PORT, 110),
                           (self.C_EXPOSURE, 90), (self.C_PROCESS, 130), (self.C_PID, 64)):
            self.view.setColumnWidth(col, width)

        self.world_only.toggled.connect(self._on_world_only)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.title)
        top.addStretch(1)
        top.addWidget(self.world_only)
        top.addSpacing(12)
        top.addWidget(self.export_btn)
        top.addWidget(self.snapshot_btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addWidget(self.summary)
        layout.addWidget(self.view, 1)

        self._keys: List[str] = []

    def _on_world_only(self, on: bool):
        self.proxy.setFilterFixedString(EXPOSURE_WORLD if on else "")

    def set_listeners(self, listeners: List[dict]):
        """Listening sockets are a state, not a stream: rebuild whenever the
        set changes so a service that stopped disappears from the table."""
        keys = sorted(f"{l.get('proto')}:{l.get('ip')}:{l.get('port')}" for l in listeners)
        world = sum(1 for l in listeners if l.get("exposure") == EXPOSURE_WORLD)
        flagged = [l for l in listeners if l.get("note")]
        self.summary.setText(
            f"{len(listeners)} listening · {world} reachable from any interface"
            + (f" · {len(flagged)} sensitive" if flagged else "")
        )
        if keys == self._keys:
            return
        self._keys = keys

        header = self.view.horizontalHeader()
        sort_col, sort_order = header.sortIndicatorSection(), header.sortIndicatorOrder()
        self.model.removeRows(0, self.model.rowCount())
        for l in sorted(listeners, key=lambda x: (int(x.get("port", 0)), str(x.get("ip", "")))):
            port = int(l.get("port", 0))
            svc = port_service(port)
            exposure = str(l.get("exposure", EXPOSURE_UNKNOWN))
            note = str(l.get("note", ""))
            items = [
                self._item(str(l.get("proto", "")).upper()),
                self._item(str(l.get("ip", "")), ip_sort_key(str(l.get("ip", "")))),
                self._item(f"{port} · {svc}" if svc else str(port), port),
                self._item(exposure),
                self._item(str(l.get("process", "")) or "—"),
                self._item(str(l.get("pid", 0) or ""), int(l.get("pid", 0) or 0)),
                self._item(note),
            ]
            colour = PHOSPHOR_DIM
            if exposure == EXPOSURE_WORLD:
                colour = RED if note else AMBER
            elif exposure == EXPOSURE_LAN:
                colour = PHOSPHOR
            brush = QtGui.QBrush(colour)
            for it in items:
                it.setForeground(brush)
            self.model.appendRow(items)
        self.view.sortByColumn(sort_col, sort_order)

    @staticmethod
    def _item(text: str, sort_value=None) -> QtGui.QStandardItem:
        it = QtGui.QStandardItem(text)
        it.setEditable(False)
        if sort_value is not None:
            it.setData(sort_value, Qt.ItemDataRole.UserRole)
        return it

    def export_csv(self, path: str):
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            for r in range(self.proxy.rowCount()):
                w.writerow([self.proxy.index(r, c).data() or ""
                            for c in range(self.proxy.columnCount())])


class BlackIceDashboard(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()
    configToggled = QtCore.pyqtSignal(bool)
    historyToggled = QtCore.pyqtSignal(bool)
    hideVirtualToggled = QtCore.pyqtSignal(bool)
    fxChanged = QtCore.pyqtSignal(str)

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
        # Reserve the widest reading up front. Without it the label is sized by
        # whatever string it last held, so growing "0 b/s" into "100.0 Kb/s"
        # clips the final glyph until the next layout pass — and the TX meter
        # jumps sideways every time RX changes magnitude.
        _meter_w = QtGui.QFontMetrics(self.rx_lbl.font()).horizontalAdvance(
            "RX: 000.0 Mb/s") + 12
        self.rx_lbl.setMinimumWidth(_meter_w)
        self.tx_lbl.setMinimumWidth(_meter_w)

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

        self.history_btn = _hacker_button("▤ HISTORY", checkable=True)
        self.history_btn.setToolTip(
            "Record contacts and bandwidth to a local database so they survive a restart"
        )
        self.history_btn.toggled.connect(self.historyToggled.emit)

        self.hide_virtual = QtWidgets.QCheckBox("HIDE VIRTUAL NICS")
        self.hide_virtual.setFont(HackerFont.mono(9))
        self.hide_virtual.setToolTip(
            "Exclude docker/veth/bridge/tunnel interfaces, whose bytes are usually\n"
            "also counted on the physical NIC they ride over"
        )
        self.hide_virtual.setStyleSheet("QCheckBox { color:#00aa44; }")
        self.hide_virtual.toggled.connect(self.hideVirtualToggled.emit)

        self.fx_combo = QtWidgets.QComboBox()
        self.fx_combo.setFont(HackerFont.mono(9))
        self.fx_combo.addItems(FX_LEVELS)
        self.fx_combo.setCurrentText("FULL")
        self.fx_combo.setToolTip("Matrix rain and scanline intensity")
        self.fx_combo.setStyleSheet(
            "QComboBox { background: #07100a; color:#00aa44; border: 1px solid #0b2a12; padding: 2px; }"
            "QAbstractItemView { background: #07100a; color:#00ff66; selection-background-color:#0b2a12; }"
        )
        self.fx_combo.currentTextChanged.connect(self._on_fx_changed)

        self.alert_lbl = QtWidgets.QLabel("")
        self.alert_lbl.setFont(HackerFont.mono(11, bold=True))
        self.alert_lbl.setStyleSheet(
            "QLabel { color:#ff3355; background:#1a0509; border:1px solid #ff3355; padding:4px; }"
        )
        self.alert_lbl.setVisible(False)

        self.states = StateBar(self)

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
        top.addWidget(self.hide_virtual)
        top.addSpacing(8)
        fx_lbl = QtWidgets.QLabel("FX:")
        fx_lbl.setFont(HackerFont.mono(9))
        fx_lbl.setStyleSheet("color:#00aa44;")
        top.addWidget(fx_lbl)
        top.addWidget(self.fx_combo)
        top.addSpacing(12)
        top.addWidget(self.config_btn)
        top.addWidget(self.history_btn)
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
        layout.addWidget(self.alert_lbl)
        layout.addSpacing(6)
        layout.addLayout(meters)
        layout.addWidget(self.scope, 1)
        layout.addWidget(self.states)
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

    def set_history_enabled(self, on: bool):
        self.history_btn.blockSignals(True)
        self.history_btn.setChecked(bool(on))
        self.history_btn.blockSignals(False)

    def _on_fx_changed(self, level: str):
        self.set_fx_level(level)
        self.fxChanged.emit(level)

    def set_fx_level(self, level: str):
        """Apply an intensity without re-emitting fxChanged (restore path)."""
        if self.fx_combo.currentText() != level:
            self.fx_combo.blockSignals(True)
            self.fx_combo.setCurrentText(level)
            self.fx_combo.blockSignals(False)
        self.matrix.set_intensity(level)
        self.scan.set_intensity(level)
        if self.matrix.isVisible():
            self._raise_fx()

    def _raise_fx(self):
        # The overlays are constructed before the layout widgets, so without
        # this they end up *behind* the content and the CRT effect is only
        # visible in the margins.
        self.matrix.raise_()
        self.scan.raise_()

    def set_states(self, counts: Dict[str, int]):
        self.states.set_counts(counts)

    def set_alert(self, text: str):
        self.alert_lbl.setText(text)
        self.alert_lbl.setVisible(bool(text))

    def _on_iface_changed(self, _name: str):
        self.scope.reset()
        self.rx_lbl.setText("RX: 0 b/s")
        self.tx_lbl.setText("TX: 0 b/s")

    def resizeEvent(self, e):
        self.matrix.setGeometry(self.rect())
        self.scan.setGeometry(self.rect())
        self._raise_fx()
        super().resizeEvent(e)

    def showEvent(self, e):
        super().showEvent(e)
        self._raise_fx()

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
            skip_virtual = self.hide_virtual.isChecked()
            nics = [(k, v) for k, v in snap.items()
                    if k != "_totals" and not is_loopback_nic(k)
                    and not (skip_virtual and is_virtual_nic(k))]
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


class WorldOutline:
    """The bundled world outline: Natural Earth 1:110m land, public domain,
    Douglas-Peucker simplified and stored as signed 16-bit degrees times 100.

    119 rings, 2505 points, ten kilobytes — which is the whole point: at world
    scale a coastline is all a dot map needs, so the MAP tab can draw itself
    with no tile server, no web engine and no network."""

    MAGIC = b"BIWM"
    SCALE = 100.0
    ASSET = "world.bin"

    def __init__(self, rings: Optional[List[List[Tuple[float, float]]]] = None):
        self.rings: List[List[Tuple[float, float]]] = rings or []

    def __len__(self) -> int:
        return len(self.rings)

    @property
    def point_count(self) -> int:
        return sum(len(r) for r in self.rings)

    @classmethod
    def parse(cls, data: bytes) -> "WorldOutline":
        if len(data) < 6 or data[:4] != cls.MAGIC:
            raise ValueError("not a world outline")
        off = 4
        (ring_count,) = struct.unpack_from(">H", data, off)
        off += 2
        rings: List[List[Tuple[float, float]]] = []
        for _ in range(ring_count):
            (n,) = struct.unpack_from(">H", data, off)
            off += 2
            coords = struct.unpack_from(">%dh" % (n * 2), data, off)
            off += n * 4
            rings.append([(coords[i * 2] / cls.SCALE, coords[i * 2 + 1] / cls.SCALE)
                          for i in range(n)])
        return cls(rings)

    @classmethod
    def load(cls, path: Optional[str] = None) -> "WorldOutline":
        """Empty rather than raising: a missing outline should degrade the map
        to a grid with dots on it, not stop the tab from opening."""
        path = path or resource_path(cls.ASSET)
        if not path:
            return cls()
        try:
            with open(path, "rb") as f:
                return cls.parse(f.read())
        except (OSError, ValueError, struct.error):
            return cls()


class WorldMapWidget(QtWidgets.QWidget):
    """Offline world map drawn with QPainter.

    Deliberately not a web view: the Leaflet map needs unpkg and a tile server
    to draw anything, renders in a separate process so snapshots come out
    black, and cannot be told anything until its page has finished loading.
    None of that is true here."""

    MIN_ZOOM = 1.0
    MAX_ZOOM = 12.0

    def __init__(self, outline: Optional[WorldOutline] = None, parent=None):
        super().__init__(parent)
        self.outline = outline if outline is not None else WorldOutline()
        self.setMinimumSize(200, 150)
        self.setMouseTracking(True)
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
        self.setPalette(pal)
        self._points: Dict[str, dict] = {}
        self._me: Optional[dict] = None
        self.show_rays = False
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._drag: Optional[QtCore.QPoint] = None

    # ---- data -----------------------------------------------------------

    def set_points(self, points: List[dict]):
        for p in points:
            lat = float(p.get("lat", 0.0) or 0.0)
            lon = float(p.get("lon", 0.0) or 0.0)
            if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                continue
            self._points[f"{p.get('proto')}:{p.get('ip')}:{p.get('port')}"] = p
        self.update()

    def clear_points(self):
        self._points.clear()
        self.update()

    def set_me(self, me: Optional[dict]):
        self._me = me
        self.update()

    def set_rays(self, on: bool):
        self.show_rays = bool(on)
        self.update()

    def point_count(self) -> int:
        return len(self._points)

    # ---- projection -----------------------------------------------------

    def project(self, lon: float, lat: float) -> Tuple[float, float]:
        w, h = self.width(), self.height()
        x = (lon + 180.0) / 360.0 * w * self.zoom + self.pan_x
        y = (90.0 - lat) / 180.0 * h * self.zoom + self.pan_y
        return x, y

    def clamp_pan(self):
        """Keep the projected world covering the viewport so panning cannot
        strand the user in empty space beside the map."""
        w, h = self.width(), self.height()
        min_x = w - w * self.zoom
        min_y = h - h * self.zoom
        self.pan_x = clamp(self.pan_x, min(min_x, 0.0), 0.0)
        self.pan_y = clamp(self.pan_y, min(min_y, 0.0), 0.0)

    def set_zoom(self, zoom: float, anchor: Optional[QtCore.QPointF] = None):
        old = self.zoom
        self.zoom = clamp(zoom, self.MIN_ZOOM, self.MAX_ZOOM)
        if anchor is not None and old:
            scale = self.zoom / old
            self.pan_x = anchor.x() - (anchor.x() - self.pan_x) * scale
            self.pan_y = anchor.y() - (anchor.y() - self.pan_y) * scale
        self.clamp_pan()
        self.update()

    def focus_on(self, lat: float, lon: float, zoom: float = 4.0):
        self.zoom = clamp(zoom, self.MIN_ZOOM, self.MAX_ZOOM)
        w, h = self.width(), self.height()
        self.pan_x = w / 2.0 - (lon + 180.0) / 360.0 * w * self.zoom
        self.pan_y = h / 2.0 - (90.0 - lat) / 180.0 * h * self.zoom
        self.clamp_pan()
        self.update()

    def reset_view(self):
        self.zoom = 1.0
        self.pan_x = self.pan_y = 0.0
        self.update()

    # ---- interaction ----------------------------------------------------

    def wheelEvent(self, e):
        steps = e.angleDelta().y() / 120.0
        if steps:
            self.set_zoom(self.zoom * (1.25 ** steps), e.position())
        e.accept()

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drag = e.pos()

    def mouseMoveEvent(self, e):
        if self._drag is not None:
            delta = e.pos() - self._drag
            self._drag = e.pos()
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self.clamp_pan()
            self.update()
            return
        hit = self.point_at(e.pos())
        self.setToolTip(hit.get("label", "") if hit else "")

    def mouseReleaseEvent(self, e):
        self._drag = None

    def mouseDoubleClickEvent(self, e):
        self.reset_view()

    def point_at(self, pos: QtCore.QPoint, radius: float = 8.0) -> Optional[dict]:
        """The nearest contact within `radius` pixels, for hover tooltips."""
        best = None
        best_d = radius * radius
        for p in self._points.values():
            x, y = self.project(float(p.get("lon", 0.0)), float(p.get("lat", 0.0)))
            d = (x - pos.x()) ** 2 + (y - pos.y()) ** 2
            if d <= best_d:
                best_d = d
                best = p
        return best

    # ---- painting -------------------------------------------------------

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), BG)
        w, h = self.width(), self.height()

        p.setPen(QtGui.QPen(GRID, 1))
        for lon in range(-180, 181, 30):
            x, _ = self.project(lon, 0)
            p.drawLine(QtCore.QPointF(x, 0), QtCore.QPointF(x, h))
        for lat in range(-90, 91, 30):
            _, y = self.project(0, lat)
            p.drawLine(QtCore.QPointF(0, y), QtCore.QPointF(w, y))

        p.setPen(QtGui.QPen(PHOSPHOR_DIM, 1))
        for ring in self.outline.rings:
            poly = QtGui.QPolygonF([QtCore.QPointF(*self.project(lon, lat))
                                    for lon, lat in ring])
            p.drawPolyline(poly)

        me_pt = None
        if self._me:
            me_pt = self.project(float(self._me.get("lon", 0.0)),
                                 float(self._me.get("lat", 0.0)))

        if me_pt and self.show_rays:
            p.setPen(QtGui.QPen(QtGui.QColor(255, 153, 0, 140), 1))
            for point in self._points.values():
                x, y = self.project(float(point.get("lon", 0.0)),
                                    float(point.get("lat", 0.0)))
                p.drawLine(QtCore.QPointF(*me_pt), QtCore.QPointF(x, y))

        p.setPen(QtGui.QPen(PHOSPHOR, 1))
        p.setBrush(QtGui.QBrush(QtGui.QColor(0, 255, 102, 70)))
        for point in self._points.values():
            x, y = self.project(float(point.get("lon", 0.0)),
                                float(point.get("lat", 0.0)))
            p.drawEllipse(QtCore.QPointF(x, y), 4.0, 4.0)

        if me_pt:
            p.setPen(QtGui.QPen(QtGui.QColor("#ff9900"), 2))
            p.setBrush(QtGui.QBrush(QtGui.QColor(255, 153, 0, 90)))
            p.drawEllipse(QtCore.QPointF(*me_pt), 6.0, 6.0)

        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setFont(HackerFont.mono(9))
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        note = "OFFLINE OUTLINE" if len(self.outline) else "OUTLINE MISSING"
        p.drawText(8, h - 8,
                   f"{note} · {len(self._points)} contacts · zoom {self.zoom:.1f}x"
                   " · wheel zoom, drag pan, double-click reset")


class LocateBridge(QtCore.QObject):
    """Marshals locate-worker results back onto the GUI thread."""
    located = QtCore.pyqtSignal(float, float, str)
    failed = QtCore.pyqtSignal(str)


class MapTab(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()

    MAX_PENDING_JS = 64
    MODES = ("OFFLINE", "ONLINE")

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
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.setFont(HackerFont.mono(9))
        self.mode_combo.addItems(self.MODES)
        self.mode_combo.setToolTip(
            "OFFLINE draws a bundled world outline and needs no network.\n"
            "ONLINE loads Leaflet and OpenStreetMap tiles from the internet."
        )
        self.mode_combo.setStyleSheet(
            "QComboBox { background: #07100a; color:#00aa44; border: 1px solid #0b2a12; padding: 2px; }"
            "QAbstractItemView { background: #07100a; color:#00ff66; selection-background-color:#0b2a12; }"
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

        self.world = WorldMapWidget(WorldOutline.load())

        # Both maps are kept fed so switching is instant; only one is shown.
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.world)
        self.stack.addWidget(self.web)
        self.mode_combo.currentTextChanged.connect(self.set_mode)

        self.list = QtWidgets.QPlainTextEdit()
        self.list.setReadOnly(True)
        self.list.setMaximumBlockCount(5000)
        self.list.setFont(HackerFont.mono(10))
        self.list.setStyleSheet(
            "QPlainTextEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; }"
        )

        self.split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)
        self.split.addWidget(self.stack)
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
        mode_lbl = QtWidgets.QLabel("MAP:")
        mode_lbl.setFont(HackerFont.mono(9))
        mode_lbl.setStyleSheet("color:#00aa44;")
        row.addWidget(mode_lbl)
        row.addWidget(self.mode_combo)
        row.addSpacing(12)
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

    def mode(self) -> str:
        return self.mode_combo.currentText()

    def set_mode(self, mode: str):
        """OFFLINE is the default: it needs no network, it captures correctly
        in a snapshot, and it cannot swallow updates that arrive before a page
        has loaded."""
        if mode not in self.MODES:
            return
        if self.mode_combo.currentText() != mode:
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentText(mode)
            self.mode_combo.blockSignals(False)
        self.stack.setCurrentWidget(self.world if mode == "OFFLINE" else self.web)

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
        self.world.set_me(self._me_obj)
        self.world.set_rays(self.me_enable.isChecked())
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
            self.world.focus_on(lat, lon, 4.0)
            self._js("if (window.BLACKICE) { window.BLACKICE.focusMe(8); }")
            self._log("[*] map focused on ME")

    def _on_locate_failed(self, msg: str):
        self._locating = False
        self._focus_pending = False
        self.me_refresh.setEnabled(True)
        self.focus_btn.setEnabled(True)
        self._log(f"[!] failed to locate ME ({msg})")

    def _on_me_toggled(self, enabled: bool):
        self.world.set_rays(enabled)
        if not enabled:
            self.world.set_me(None)
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
            self.world.set_points(map_points)
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

    COLS = ["First Seen", "Last Seen", "Proto", "State", "IP", "Host",
            "Port/Svc", "Location", "Process", "ASN/Org", "Hits", "Flags"]
    (C_FIRST, C_LAST, C_PROTO, C_STATE, C_IP, C_HOST,
     C_PORT, C_LOCATION, C_PROCESS, C_ASN, C_HITS, C_FLAGS) = range(12)

    FLAG_NEW = "NEW"
    KEY_ROLE = Qt.ItemDataRole.UserRole + 1

    snapshotRequested = QtCore.pyqtSignal()
    csvExportRequested = QtCore.pyqtSignal()
    resolveRequested = QtCore.pyqtSignal(list)
    alertRaised = QtCore.pyqtSignal(str, str)
    watchlistAppendRequested = QtCore.pyqtSignal(str, str, str)
    whoisRequested = QtCore.pyqtSignal(str)

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
        self.filter_edit.setPlaceholderText("filter: ip / host / port / proto / location ...")
        self.filter_edit.setFont(HackerFont.mono(10))
        self.filter_edit.setStyleSheet(
            "QLineEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; padding:4px; }"
        )

        self.count_lbl = QtWidgets.QLabel("0 contacts")
        self.count_lbl.setFont(HackerFont.mono(10))
        self.count_lbl.setStyleSheet("color:#00aa44;")

        self.clear_btn = _hacker_button("⌫ CLEAR")
        self.export_btn = _hacker_button("⤓ EXPORT CSV")
        self.rules_btn = _hacker_button("⚑ RULES")
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
        self.view.sortByColumn(self.C_LAST, Qt.SortOrder.DescendingOrder)
        for col, width in ((self.C_FIRST, 78), (self.C_LAST, 78), (self.C_PROTO, 52),
                           (self.C_STATE, 96), (self.C_IP, 130), (self.C_HOST, 190),
                           (self.C_PORT, 104), (self.C_LOCATION, 150),
                           (self.C_PROCESS, 110), (self.C_ASN, 150), (self.C_HITS, 48)):
            self.view.setColumnWidth(col, width)
        self.view.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.view.customContextMenuRequested.connect(self._show_menu)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(self.filter_edit, 1)
        controls.addWidget(self.count_lbl)
        controls.addSpacing(8)
        controls.addWidget(self.clear_btn)
        controls.addWidget(self.export_btn)
        controls.addWidget(self.rules_btn)
        controls.addWidget(self.snapshot_btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(self.title)
        layout.addLayout(controls)
        layout.addWidget(self.view, 1)

        self._rows: Dict[str, int] = {}
        self._points: Dict[str, dict] = {}
        self.rules = RuleSet()
        self.baseline: Optional[set] = None
        self.resolve_hosts = True

    # ---- inputs ---------------------------------------------------------

    def set_rules(self, rules: RuleSet):
        """Swap the watchlist and re-flag every row already on screen, so a
        reload is retroactive instead of only affecting future contacts."""
        self.rules = rules
        for key, point in self._points.items():
            row = self._rows.get(key)
            if row is None:
                continue
            point["watch"] = self.rules.match(point)
            self._apply_flags(row, point)

    def set_baseline(self, baseline: Optional[set]):
        """Addresses seen in earlier sessions. Anything outside it is NEW.

        Rows collected before history was switched on are re-judged here:
        otherwise turning the toggle on mid-session leaves whatever is already
        on screen permanently unflagged."""
        self.baseline = baseline
        for key, point in self._points.items():
            row = self._rows.get(key)
            if row is None:
                continue
            point["new"] = bool(baseline is not None
                                and point.get("ip") not in baseline
                                and not is_privateish(str(point.get("ip", ""))))
            self._apply_flags(row, point)

    def set_host(self, ip: str, host: str):
        """Fill the Host column for every row sharing this address."""
        for key, point in self._points.items():
            if point.get("ip") != ip:
                continue
            point["host"] = host
            row = self._rows.get(key)
            if row is not None and self.model.item(row, self.C_HOST) is not None:
                self.model.item(row, self.C_HOST).setText(host)
                before = point.get("watch") or []
                point["watch"] = self.rules.match(point)
                self._apply_flags(row, point)
                if point["watch"] and not before:
                    self.alertRaised.emit(f"{point.get('ip')}:{point.get('port')}",
                                          ", ".join(point["watch"]))

    # ---- table ----------------------------------------------------------

    def _clear(self):
        self.model.removeRows(0, self.model.rowCount())
        self._rows.clear()
        self._points.clear()
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

    def _flags_text(self, point: dict) -> str:
        parts = []
        if point.get("new"):
            parts.append(self.FLAG_NEW)
        parts.extend(f"⚑{lbl}" for lbl in point.get("watch") or [])
        return " ".join(parts)

    def _row_colour(self, point: dict) -> QtGui.QBrush:
        if point.get("watch"):
            return QtGui.QBrush(RED)
        location = str(point.get("location", ""))
        if location.startswith("LOCAL"):
            return QtGui.QBrush(PHOSPHOR_DIM)
        if "unavailable" in location.lower():
            return QtGui.QBrush(AMBER)
        return QtGui.QBrush(PHOSPHOR)

    def _apply_flags(self, row: int, point: dict):
        item = self.model.item(row, self.C_FLAGS)
        if item is None:
            return
        item.setText(self._flags_text(point))
        brush = self._row_colour(point)
        for col in range(self.model.columnCount()):
            cell = self.model.item(row, col)
            if cell is not None:
                cell.setForeground(brush)
        state_cell = self.model.item(row, self.C_STATE)
        if state_cell is not None and not point.get("watch"):
            state_cell.setForeground(QtGui.QBrush(state_color(point.get("state", ""))))

    def add_points(self, points: List[dict]):
        unresolved: List[str] = []
        for p in points:
            proto = p.get("proto", "?")
            ip = p.get("ip", "?")
            port = int(p.get("port", 0))
            label = p.get("label", "")
            location = p.get("location") or (
                label.split("—", 1)[1].strip() if "—" in label else label)
            ts = float(p.get("ts", time.time()))
            ts_str = time.strftime("%H:%M:%S", time.localtime(ts))
            key = f"{proto}:{ip}:{port}"
            process = p.get("process", "")
            asn = p.get("asn", "")
            state = str(p.get("state", "") or "")
            host = p.get("host", "") or ""
            svc = port_service(port)
            port_svc = f"{port} · {svc}" if svc else str(port)

            point = dict(p)
            point["location"] = location
            point["host"] = host
            if self.resolve_hosts and not host:
                unresolved.append(ip)

            existing = self._points.get(key)
            if existing is not None:
                point["new"] = existing.get("new", False)
                point["host"] = point["host"] or existing.get("host", "")
            else:
                point["new"] = bool(self.baseline is not None
                                    and ip not in self.baseline
                                    and not is_privateish(ip))
            point["watch"] = self.rules.match(point)
            self._points[key] = point

            if key in self._rows:
                src_row = self._rows[key]
                self.model.item(src_row, self.C_LAST).setText(ts_str)
                self.model.item(src_row, self.C_LAST).setData(ts, Qt.ItemDataRole.UserRole)
                hits_item = self.model.item(src_row, self.C_HITS)
                try:
                    hits = int(hits_item.text()) + 1
                except ValueError:
                    hits = 2
                hits_item.setText(str(hits))
                hits_item.setData(hits, Qt.ItemDataRole.UserRole)
                self.model.item(src_row, self.C_LOCATION).setText(location)
                if state:
                    self.model.item(src_row, self.C_STATE).setText(state)
                if point["host"]:
                    self.model.item(src_row, self.C_HOST).setText(point["host"])
                if process:
                    self.model.item(src_row, self.C_PROCESS).setText(process)
                if asn:
                    self.model.item(src_row, self.C_ASN).setText(asn)
                self._apply_flags(src_row, point)
                continue

            row = [
                self._make_item(ts_str, ts),
                self._make_item(ts_str, ts),
                self._make_item(proto.upper()),
                self._make_item(state),
                self._make_item(ip, ip_sort_key(ip)),
                self._make_item(point["host"]),
                self._make_item(port_svc, port),
                self._make_item(location),
                self._make_item(process),
                self._make_item(asn),
                self._make_item("1", 1),
                self._make_item(self._flags_text(point)),
            ]
            row[self.C_FIRST].setData(key, self.KEY_ROLE)
            self.model.appendRow(row)
            self._rows[key] = self.model.rowCount() - 1
            self._apply_flags(self.model.rowCount() - 1, point)

            if point["watch"]:
                self.alertRaised.emit(f"{ip}:{port}", ", ".join(point["watch"]))

        if unresolved:
            self.resolveRequested.emit(sorted(set(unresolved)))
        self._update_count()

    # ---- rollups and export ---------------------------------------------

    def points(self) -> List[dict]:
        return list(self._points.values())

    def rollup(self, field: str, limit: int = 8) -> List[Tuple[str, int]]:
        return rollup_counts(self.points(), field, limit)

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

    def visible_points(self) -> List[dict]:
        """The rows the filter and sort currently show, in view order."""
        out = []
        for r in range(self.proxy.rowCount()):
            key = self.proxy.index(r, self.C_FIRST).data(self.KEY_ROLE)
            point = self._points.get(key)
            if point is not None:
                out.append(point)
        return out

    def export_json(self, path: str):
        """The same visible rows as the CSV export, but with the raw values —
        coordinates and country code included — instead of display strings."""
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.visible_points(), f, indent=2, sort_keys=True)

    # ---- context menu ---------------------------------------------------

    def row_at(self, pos: QtCore.QPoint) -> Optional[dict]:
        index = self.view.indexAt(pos)
        if not index.isValid():
            return None
        key = self.proxy.index(index.row(), self.C_FIRST).data(self.KEY_ROLE)
        return self._points.get(key)

    def _show_menu(self, pos: QtCore.QPoint):
        point = self.row_at(pos)
        if point is None:
            return
        menu = self.build_menu(point)
        menu.exec(self.view.viewport().mapToGlobal(pos))

    def build_menu(self, point: dict) -> QtWidgets.QMenu:
        ip = str(point.get("ip", ""))
        asn = str(point.get("asn", ""))
        country = str(point.get("country", ""))
        clip = QtWidgets.QApplication.clipboard()

        menu = QtWidgets.QMenu(self)
        menu.setFont(HackerFont.mono(10))
        menu.addAction(f"Copy {ip}", lambda: clip and clip.setText(ip))
        menu.addAction("Copy row as CSV",
                       lambda: clip and clip.setText(self._row_csv(point)))
        menu.addSeparator()
        menu.addAction(f"Filter to {ip}", lambda: self.filter_edit.setText(ip))
        if asn:
            menu.addAction(f"Filter to {asn}", lambda: self.filter_edit.setText(asn))
        if country:
            menu.addAction(f"Filter to {country}",
                           lambda: self.filter_edit.setText(country))
        menu.addSeparator()
        menu.addAction(
            f"Add {ip} to watchlist",
            lambda: self.watchlistAppendRequested.emit("cidr", self._host_cidr(ip), ""))
        if asn:
            menu.addAction(
                f"Add {asn.split(' ')[0]} to watchlist",
                lambda: self.watchlistAppendRequested.emit("asn", asn.split(" ")[0], ""))
        menu.addSeparator()
        menu.addAction("Look up WHOIS in browser (external)",
                       lambda: self.whoisRequested.emit(ip))
        return menu

    @staticmethod
    def _host_cidr(ip: str) -> str:
        """A single address as a network, so it can be written as a cidr rule."""
        try:
            addr = ipaddress.ip_address(ip)
        except ValueError:
            return ip
        return f"{ip}/{32 if addr.version == 4 else 128}"

    def _row_csv(self, point: dict) -> str:
        key = f"{point.get('proto')}:{point.get('ip')}:{point.get('port')}"
        row = self._rows.get(key)
        if row is None:
            return ""
        return ",".join(str(self.model.index(row, c).data() or "")
                        for c in range(self.model.columnCount()))


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

        # Endpoint counts, not bytes: psutil exposes no per-process byte
        # counters on Linux, and inventing one would be worse than saying so.
        self.geo_panel = RollupPanel("TOP COUNTRIES — CONTACTS")
        self.asn_panel = RollupPanel("TOP NETWORKS — CONTACTS PER ASN")
        self.proc_panel = RollupPanel("TOP TALKERS — ENDPOINTS PER PROCESS")

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

        rollups = QtWidgets.QHBoxLayout()
        rollups.addWidget(self.geo_panel, 1)
        rollups.addWidget(self.asn_panel, 1)
        rollups.addWidget(self.proc_panel, 1)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addLayout(win_row)
        layout.addWidget(self.graph, 2)
        layout.addLayout(stats_row)
        layout.addLayout(rollups, 1)

    def set_rollups(self, points: List[dict]):
        self.geo_panel.set_rows(rollup_counts(points, "country"))
        self.asn_panel.set_rows(rollup_counts(points, "asn"))
        self.proc_panel.set_rows(rollup_counts(points, "process"))

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

    SAMPLE_PERIOD = 60.0

    def __init__(self, config: Optional[AppConfig] = None,
                 store: Optional[SessionStore] = None):
        super().__init__()
        self.cfg = config if config is not None else AppConfig()
        self.store = store if store is not None else SessionStore()
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
        self.listen = ListenersTab()
        self.map = MapTab()
        self.stats = StatsTab()

        self.tabs.addTab(self.dash, "BLACK ICE")
        self.tabs.addTab(self.conns, "CONTACTS")
        self.tabs.addTab(self.listen, "LISTEN")
        self.tabs.addTab(self.map, "MAP")
        self.tabs.addTab(self.stats, "STATS")

        self.dash.set_config_enabled(self.cfg.enabled)
        self.dash.configToggled.connect(self._on_config_toggled)
        self.dash.historyToggled.connect(self._on_history_toggled)
        self.dash.hideVirtualToggled.connect(self._on_hide_virtual)
        self.dash.fxChanged.connect(lambda level: self.cfg.set("dash/fx", level))

        self.dash.snapshotRequested.connect(lambda: self._snapshot(self.dash, "blackice_dashboard"))
        self.conns.snapshotRequested.connect(lambda: self._snapshot(self.conns, "blackice_contacts"))
        self.listen.snapshotRequested.connect(lambda: self._snapshot(self.listen, "blackice_listeners"))
        self.map.snapshotRequested.connect(lambda: self._snapshot(self.map, "blackice_map"))
        self.stats.snapshotRequested.connect(lambda: self._snapshot(self.stats, "blackice_stats"))
        self.conns.csvExportRequested.connect(self._export_csv)
        self.listen.csvExportRequested.connect(self._export_listeners)
        self.conns.alertRaised.connect(self._on_alert)
        self.conns.watchlistAppendRequested.connect(self.append_watchlist)
        self.conns.whoisRequested.connect(self._open_whois)
        self.conns.rules_btn.clicked.connect(lambda: self.reload_rules(announce=True))

        self.watchlist_path = default_watchlist_path()
        self.reload_rules()

        self.rdns = RDnsResolver()
        self.rdns.resolved.connect(self._on_host_resolved)
        self.rdns.start()
        self.conns.resolveRequested.connect(self.rdns.request)

        self.poller = TrafficPoller(interval=1.0)
        self.poller.traffic.connect(self._on_traffic)
        self.poller.start()

        self.scanner = ConnScanner(interval=3.0, rdns=self.rdns.cache)
        self.scanner.points.connect(self._on_points)
        self.scanner.listeners.connect(self.listen.set_listeners)
        self.scanner.states.connect(self.dash.set_states)
        self.scanner.event.connect(self.dash.set_event)
        self.scanner.start()

        self._alerts = 0
        self._sample_rx = 0.0
        self._sample_tx = 0.0
        self._sample_n = 0
        self._sample_flushed = time.time()

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

    def reload_rules(self, announce: bool = False):
        """(Re)read the watchlist. Applied retroactively to rows already on
        screen, so editing the file while the app runs actually means
        something."""
        rules = RuleSet.load(self.watchlist_path)
        self.conns.set_rules(rules)
        for err in rules.errors:
            self.dash.set_event(f"[!] watchlist: {err}")
        if announce or len(rules):
            self.dash.set_event(
                f"[*] watchlist: {len(rules)} rule(s) ← {self.watchlist_path}")
        return rules

    def append_watchlist(self, kind: str, value: str, label: str = ""):
        """Write one rule to the watchlist file and pick it up immediately."""
        line = " ".join(p for p in (kind, value, label) if p)
        try:
            os.makedirs(os.path.dirname(self.watchlist_path), exist_ok=True)
            fresh = not os.path.exists(self.watchlist_path)
            with open(self.watchlist_path, "a", encoding="utf-8") as f:
                if fresh:
                    f.write(WATCHLIST_TEMPLATE)
                f.write(line + "\n")
        except OSError as e:
            self.dash.set_event(f"[!] could not write watchlist: {e}")
            return
        self.dash.set_event(f"[*] watchlist += {line}")
        self.reload_rules()

    def _open_whois(self, ip: str):
        """Explicit, user-initiated, and named as external in the menu: this
        hands the address to a third-party lookup service."""
        if not ip:
            return
        url = QtCore.QUrl(f"https://www.whois.com/whois/{ip}")
        QtGui.QDesktopServices.openUrl(url)
        self.dash.set_event(f"[*] WHOIS lookup opened in browser for {ip}")

    def _on_alert(self, endpoint: str, labels: str):
        self._alerts += 1
        self.dash.set_event(f"[!] ALERT {endpoint} — {labels}")
        self.dash.set_alert(f"⚑ WATCHLIST HIT ({self._alerts}): {endpoint} — {labels}")

    def _on_host_resolved(self, ip: str, host: str):
        self.conns.set_host(ip, host)
        self.store.set_host(ip, host)

    def _on_hide_virtual(self, on: bool):
        self.poller.set_hide_virtual(on)
        self.cfg.set("dash/hide_virtual", bool(on))
        self.dash.set_event(
            f"[*] virtual interfaces {'excluded from' if on else 'included in'} totals")
        self._ui_tick()

    def _on_history_toggled(self, on: bool):
        self.cfg.set("history/enabled", bool(on))
        if not on:
            self.store.close()
            self.conns.set_baseline(None)
            self.dash.set_event("[*] history recording DISABLED")
            return
        if not self.store.open():
            self.dash.set_event(f"[!] history unavailable: {self.store.error}")
            self.dash.set_history_enabled(False)
            return
        self.conns.set_baseline(self.store.baseline)
        contacts, samples = self.store.counts()
        self.dash.set_event(
            f"[*] history ENABLED → {self.store.path} "
            f"({contacts} contacts, {samples} samples, {len(self.store.baseline)} known hosts)")

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
        self.cfg.set("map/mode", self.map.mode())
        self.cfg.set("dash/fx", self.dash.fx_combo.currentText())
        self.cfg.set("dash/hide_virtual", self.dash.hide_virtual.isChecked())
        self.cfg.set("history/enabled", self.store.enabled)
        self.cfg.set("listen/world_only", self.listen.world_only.isChecked())
        self.cfg.set("listen/header", self.listen.view.horizontalHeader().saveState())
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

        self.dash.set_fx_level(self.cfg.get("dash/fx", "FULL", str))
        self.dash.hide_virtual.setChecked(self.cfg.get("dash/hide_virtual", False, bool))
        self.listen.world_only.setChecked(self.cfg.get("listen/world_only", False, bool))
        listen_hdr = self.cfg.get("listen/header", None, QtCore.QByteArray)
        if listen_hdr:
            self.listen.view.horizontalHeader().restoreState(listen_hdr)
        self.map.set_mode(self.cfg.get("map/mode", "OFFLINE", str))
        # Recording is a deliberate act, so it is restored the same way the
        # button turns it on — including the "could not open" path.
        self.dash.set_history_enabled(self.cfg.get("history/enabled", False, bool))
        if self.dash.history_btn.isChecked():
            self._on_history_toggled(True)

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
            names = list(psutil.net_io_counters(pernic=True).keys())
            if self.dash.hide_virtual.isChecked():
                names = [n for n in names if not is_virtual_nic(n)]
            nics = ["ALL"] + names
            for combo in (self.dash.iface, self.stats.iface_combo):
                self._refresh_combo(combo, nics)
        except Exception:
            pass
        self.stats.set_rollups(self.conns.points())

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
        self._accumulate_sample(snap)

    def _accumulate_sample(self, snap: dict):
        """One row a minute, averaged. A row per second would put 86k rows a
        day into the database to say the same thing."""
        totals = snap.get("_totals", {})
        self._sample_rx += float(totals.get("rx_bps", 0.0))
        self._sample_tx += float(totals.get("tx_bps", 0.0))
        self._sample_n += 1
        now = time.time()
        if now - self._sample_flushed < self.SAMPLE_PERIOD or not self._sample_n:
            return
        if self.store.enabled:
            self.store.record_sample(now, self._sample_rx / self._sample_n,
                                     self._sample_tx / self._sample_n)
        self._sample_rx = self._sample_tx = 0.0
        self._sample_n = 0
        self._sample_flushed = now

    def _on_points(self, points: list):
        self.map.push_points(points)
        self.conns.add_points(points)
        if self.store.enabled:
            self.store.record_contacts(points)

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
        web = widget.findChild(QtWebEngineWidgets.QWebEngineView) if HAVE_WEBENGINE else None
        has_web = web is not None and web.isVisible()
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
        path, selected = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Contacts", default, "CSV (*.csv);;JSON (*.json)"
        )
        if not path:
            return
        as_json = path.lower().endswith(".json") or "json" in (selected or "").lower()
        try:
            if as_json:
                if not path.lower().endswith(".json"):
                    path += ".json"
                self.conns.export_json(path)
            else:
                self.conns.export_csv(path)
            self.dash.set_event(
                f"[*] contacts {'JSON' if as_json else 'CSV'} exported → {path}")
        except OSError as e:
            self.dash.set_event(f"[!] export failed: {e}")

    def _export_listeners(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        default = os.path.join(os.path.expanduser("~"), f"blackice_listeners_{ts}.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Listening Sockets", default, "CSV (*.csv)"
        )
        if not path:
            return
        try:
            self.listen.export_csv(path)
            self.dash.set_event(f"[*] listeners CSV exported → {path}")
        except OSError as e:
            self.dash.set_event(f"[!] listeners export failed: {e}")

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
        try:
            self.rdns.stop()
            self.rdns.wait(1500)
        except Exception:
            pass
        self.store.close()
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


CONTACT_FIELDS = ["first_seen", "last_seen", "proto", "state", "ip", "host",
                  "port", "service", "location", "country", "process", "asn",
                  "hits", "flags"]


class ContactAggregator:
    """The contact table without a table: same dedupe key, same hit counting
    and the same watchlist flags, so the headless export and the GUI agree."""

    def __init__(self, rules: Optional[RuleSet] = None):
        self.rules = rules if rules is not None else RuleSet()
        self._rows: Dict[str, dict] = {}

    def __len__(self) -> int:
        return len(self._rows)

    def add(self, points: List[dict]):
        for p in points:
            ip = str(p.get("ip", ""))
            port = int(p.get("port", 0))
            proto = str(p.get("proto", ""))
            key = f"{proto}:{ip}:{port}"
            ts = float(p.get("ts", time.time()))
            row = self._rows.get(key)
            if row is None:
                row = {
                    "first_seen": ts, "last_seen": ts, "proto": proto,
                    "state": p.get("state", ""), "ip": ip, "host": p.get("host", ""),
                    "port": port, "service": port_service(port),
                    "location": p.get("location", ""), "country": p.get("country", ""),
                    "process": p.get("process", ""), "asn": p.get("asn", ""),
                    "hits": 1,
                }
                self._rows[key] = row
            else:
                row["last_seen"] = ts
                row["hits"] += 1
                for field in ("state", "host", "location", "country", "process", "asn"):
                    if p.get(field):
                        row[field] = p[field]
            row["flags"] = " ".join(self.rules.match(row))

    def rows(self) -> List[dict]:
        return sorted(self._rows.values(), key=lambda r: (-r["hits"], r["ip"]))


def write_contacts_csv(path: str, rows: List[dict]):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CONTACT_FIELDS)
        for row in rows:
            w.writerow([row.get(k, "") for k in CONTACT_FIELDS])


def write_contacts_json(path: str, rows: List[dict]):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2, sort_keys=True)


def run_headless(args) -> int:
    """Scan without a GUI, then write what was seen.

    Same scanner the window uses, so this can run over ssh or in a cron job
    and produce exactly the contact set the CONTACTS tab would have shown."""
    # QThread objects want an application object even when never started.
    app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([sys.argv[0]])
    del app

    rules = RuleSet.load(args.watchlist or default_watchlist_path())
    for err in rules.errors:
        print(f"[!] watchlist: {err}", file=sys.stderr)

    scanner = ConnScanner(interval=args.interval)
    agg = ContactAggregator(rules)
    listeners: List[dict] = []
    states: Dict[str, int] = {}

    deadline = time.time() + max(0.0, args.duration)
    passes = 0
    while True:
        points, listen, st = scanner._scan_psutil()
        if args.resolve:
            for p in points:
                if not p.host:
                    p.host = RDnsResolver._lookup(p.ip)
        agg.add([p.__dict__ for p in points])
        listeners = [l.__dict__ for l in listen]
        states = st
        passes += 1
        if time.time() >= deadline:
            break
        time.sleep(min(args.interval, max(0.0, deadline - time.time())))

    rows = agg.rows()
    world = [l for l in listeners if l.get("exposure") == EXPOSURE_WORLD]
    print(f"{APP_NAME} v{APP_VERSION} headless — {passes} pass(es) over {args.duration:.0f}s")
    print(f"  contacts  : {len(rows)}")
    print(f"  listeners : {len(listeners)} ({len(world)} reachable from any interface)")
    print("  states    : " + (", ".join(f"{k} {v}" for k, v in sorted(states.items())) or "—"))
    flagged = [r for r in rows if r.get("flags")]
    for row in flagged:
        print(f"  [!] {row['ip']}:{row['port']} — {row['flags']}")

    if args.export:
        path = args.export
        try:
            if path.lower().endswith(".json"):
                write_contacts_json(path, rows)
            else:
                write_contacts_csv(path, rows)
        except OSError as e:
            print(f"[!] export failed: {e}", file=sys.stderr)
            return 2
        print(f"  exported  : {path}")
    return 0


def run_selftest() -> int:
    """Check the pieces that have no visible failure mode until they are
    needed — a missing world outline, an unwritable history database, a
    watchlist the parser silently drops."""
    results: List[Tuple[str, bool, str]] = []

    def check(name: str, fn):
        try:
            ok, detail = fn()
        except Exception as e:  # a self-test must report, never traceback
            ok, detail = False, f"{type(e).__name__}: {e}"
        results.append((name, bool(ok), detail))

    def _units():
        return (human_bps(1500) == "1.5 Kb/s" and human_bytes(2048) == "2.0 KB",
                f"{human_bps(1500)} / {human_bytes(2048)}")

    def _addresses():
        ok = (normalize_ip("::ffff:8.8.8.8") == "8.8.8.8"
              and is_loopback_ip("127.0.0.1")
              and exposure_class("0.0.0.0") == EXPOSURE_WORLD
              and exposure_class("127.0.0.1") == EXPOSURE_LOCAL
              and is_virtual_nic("docker0") and not is_virtual_nic("eth0"))
        return ok, "normalize/loopback/exposure/virtual-nic"

    def _outline():
        outline = WorldOutline.load()
        return len(outline) > 0, f"{len(outline)} rings, {outline.point_count} points"

    def _rules():
        rs = RuleSet.parse(WATCHLIST_TEMPLATE + "\ncidr 203.0.113.0/24 TEST\nport 22 SSH\n")
        hit = rs.match({"ip": "203.0.113.9", "port": 443})
        return (len(rs) == 2 and not rs.errors and hit == ["TEST"],
                f"{len(rs)} rules, {len(rs.errors)} errors")

    def _store():
        tmp = tempfile.mkdtemp(prefix="blackice-selftest-")
        try:
            store = SessionStore(os.path.join(tmp, "history.db"))
            if not store.open():
                return False, store.error
            store.record_contacts([{"ip": "203.0.113.5", "port": 443, "proto": "tcp",
                                    "ts": time.time(), "location": "Somewhere XX"}])
            store.record_sample(time.time(), 100.0, 50.0)
            contacts, samples = store.counts()
            store.close()
            return contacts == 1 and samples == 1, f"{contacts} contacts, {samples} samples"
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def _snapshot():
        counters = collections.namedtuple(
            "c", "bytes_recv bytes_sent packets_recv packets_sent")
        prev = {"eth0": counters(0, 0, 0, 0), "docker0": counters(0, 0, 0, 0)}
        now = {"eth0": counters(1000, 500, 4, 2), "docker0": counters(1000, 500, 4, 2)}
        full = build_snapshot(prev, now, 1.0)["_totals"]["rx_bps"]
        trimmed = build_snapshot(prev, now, 1.0, hide_virtual=True)["_totals"]["rx_bps"]
        return full == 16000.0 and trimmed == 8000.0, f"{full:.0f} vs {trimmed:.0f} b/s"

    def _scan():
        scanner = ConnScanner()
        points, listeners, states = scanner._scan_psutil()
        return True, (f"{len(points)} contacts, {len(listeners)} listeners, "
                      f"{sum(states.values())} sockets")

    def _geoip():
        init_geoip()
        init_asn()
        return (_geoip_reader is not None,
                "city db loaded" + (", asn db loaded" if HAVE_ASN else ", no asn db"))

    check("units", _units)
    check("addresses", _addresses)
    check("world outline", _outline)
    check("watchlist rules", _rules)
    check("history store", _store)
    check("snapshot math", _snapshot)
    check("socket scan", _scan)
    check("geoip", _geoip)

    print(f"{APP_NAME} v{APP_VERSION} self-test")
    for name, ok, detail in results:
        print(f"  [{'OK  ' if ok else 'FAIL'}] {name:<16} {detail}")
    failed = [n for n, ok, _ in results if not ok]
    print(f"  {len(results) - len(failed)}/{len(results)} checks passed")
    return 1 if failed else 0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="blackice_traffic",
        description=f"{APP_NAME} — traffic visualizer (visualization only).")
    p.add_argument("--version", action="version",
                   version=f"{APP_NAME} {APP_VERSION} ({APP_BUILD})")
    p.add_argument("--headless", action="store_true",
                   help="scan without a GUI and print/export the result")
    p.add_argument("--duration", type=float, default=10.0, metavar="SECONDS",
                   help="how long --headless keeps scanning (default: 10)")
    p.add_argument("--interval", type=float, default=3.0, metavar="SECONDS",
                   help="seconds between scans (default: 3)")
    p.add_argument("--export", metavar="PATH",
                   help="write contacts to PATH (.json for JSON, else CSV)")
    p.add_argument("--resolve", action="store_true",
                   help="reverse-DNS every contact (slower)")
    p.add_argument("--watchlist", metavar="PATH",
                   help=f"watchlist rules file (default: {default_watchlist_path()})")
    p.add_argument("--selftest", action="store_true",
                   help="run internal checks and exit")
    return p


def main(argv: Optional[List[str]] = None):
    args = build_arg_parser().parse_args(argv if argv is not None else sys.argv[1:])

    if args.selftest:
        sys.exit(run_selftest())

    if args.headless:
        init_geoip()
        init_asn()
        sys.exit(run_headless(args))

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
