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
import subprocess
import re
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
APP_VERSION = "0.15.1"
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


def human_duration(seconds: float) -> str:
    """A span of watching, in the largest unit that keeps it readable."""
    s = max(0.0, float(seconds or 0.0))
    if s < 60:
        return f"{s:.0f}s"
    if s < 3600:
        return f"{s / 60:.0f}m"
    if s < 86400:
        return f"{s / 3600:.1f}h"
    return f"{s / 86400:.1f}d"


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
        # Keyed by position in self.rules: match() needs *this* rule's network,
        # and scanning a flat list for it made every contact cost
        # cidr-rules squared.
        self._nets: Dict[int, "ipaddress._BaseNetwork"] = {}
        self._reindex()

    def _reindex(self):
        self._nets = {}
        for idx, r in enumerate(self.rules):
            if r.kind != "cidr":
                continue
            try:
                self._nets[idx] = ipaddress.ip_network(r.value, strict=False)
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

        for idx, rule in enumerate(self.rules):
            val = rule.value
            hit = False
            if rule.kind == "cidr":
                net = self._nets.get(idx)
                hit = (parsed is not None and net is not None
                       and parsed.version == net.version and parsed in net)
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


FIREWALL_SYNTAXES = ("nftables", "iptables", "ufw")


def firewall_rule(syntax: str, ip: str, port: int = 0, proto: str = "tcp") -> str:
    """A drop rule for one endpoint, as text.

    Generated, never applied — this tool reports and does not touch the
    network, and the gap between "here is the rule" and "I ran it for you"
    is the whole disclaimer. What it saves is getting the address family and
    the flag order right at the moment you have already decided.

    Returns "" for an address that will not parse, rather than emitting a
    rule built around a typo."""
    try:
        addr = ipaddress.ip_address(normalize_ip(ip))
    except ValueError:
        return ""
    v6 = addr.version == 6
    transport = "udp" if str(proto).lower() == "udp" else "tcp"
    port = int(port or 0)
    if syntax == "nftables":
        family = "ip6" if v6 else "ip"
        dport = f" {transport} dport {port}" if port else ""
        return (f"nft add rule inet filter output {family} daddr {addr}"
                f"{dport} counter drop")
    if syntax == "iptables":
        binary = "ip6tables" if v6 else "iptables"
        dport = f" --dport {port}" if port else ""
        return (f"{binary} -A OUTPUT -p {transport} -d {addr}{dport} -j DROP")
    if syntax == "ufw":
        if port:
            return f"ufw deny out to {addr} port {port} proto {transport}"
        return f"ufw deny out to {addr}"
    return ""


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
    #: (ip, the PTR name resolves back to this address)
    fcrdns = QtCore.pyqtSignal(str, bool)

    MAX_QUEUE = 512
    #: forward-confirm every PTR we get back
    VERIFY = True

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

    @staticmethod
    def _confirm(ip: str, host: str) -> bool:
        """Does the PTR name resolve back to the address it came from?

        A PTR record is set by whoever controls the address and can say
        anything at all — "trusted.google.com" costs its owner nothing. Only
        the forward lookup is published by the name's owner, so agreement
        between the two is the part that carries information.

        The whole answer set has to be checked, not the first address:
        anycast names like one.one.one.one resolve to several addresses, and
        comparing against one of them would flag Cloudflare as a forgery."""
        if not ip or not host:
            return False
        try:
            infos = socket.getaddrinfo(host, None)
        except (OSError, socket.gaierror, UnicodeError):
            return False
        return normalize_ip(ip) in {normalize_ip(str(i[4][0])) for i in infos}

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
                if self.VERIFY:
                    self.fcrdns.emit(ip, self._confirm(ip, host))


_SS_BYTES_RECV = re.compile(r"\bbytes_received:(\d+)")
_SS_BYTES_SENT = re.compile(r"\bbytes_sent:(\d+)")
_SS_BYTES_ACKED = re.compile(r"\bbytes_acked:(\d+)")
_SS_USER = re.compile(r'users:\(\("([^"]+)",pid=(\d+)')

UNATTRIBUTED = "(other users)"


def is_elevated() -> bool:
    """True when this process can see every socket's owning pid."""
    try:
        return os.geteuid() == 0
    except AttributeError:  # not POSIX
        return False


def attribution_summary(stats: Dict[str, int]) -> str:
    """How much of the socket table this process is actually allowed to read.

    Linux hands /proc/net/tcp to anybody, but mapping a socket to the process
    that owns it means walking /proc/<pid>/fd, which is readable only for
    your own processes. Unprivileged, roughly half the Process column comes
    back blank — and a blank cell reads as "no process", not as "not
    permitted to know". Saying which it is costs one status field."""
    if stats.get("elevated"):
        return "all processes (root)"
    remote = int(stats.get("remote", 0) or 0)
    blank = int(stats.get("unattributed", 0) or 0)
    if not remote or not blank:
        return "own processes"
    return f"own processes ({blank * 100 // remote}% unattributed)"


def split_endpoint(text: str) -> Tuple[str, int]:
    """`ss`'s address notation -> (ip, port).

    Three shapes have to survive this: "1.2.3.4:443", "[2001:db8::1]:443"
    and "[::ffff:1.2.3.4]:443" — the last one is how the kernel renders a v4
    connection on a v6 socket, and it has to come back as plain 1.2.3.4 or it
    will not join to the contact the scanner recorded."""
    raw = (text or "").strip()
    if raw.startswith("["):
        host, _, rest = raw.partition("]")
        ip, port = host[1:], rest.lstrip(":")
    else:
        ip, _, port = raw.rpartition(":")
        if not ip:
            return "", 0
    try:
        return normalize_ip(ip), int(port)
    except ValueError:
        return normalize_ip(ip), 0


class ProcTrafficSampler:
    """Per-process network bytes, taken from the kernel's own TCP counters.

    The STATS tab used to rank processes by *endpoint count* with a comment
    admitting that psutil exposes no per-process byte counter on Linux. That
    is true of psutil, but not of the kernel: `ss -tinp` prints the tcp_info
    block for every socket, which carries bytes_received and bytes_sent, next
    to the pid that owns it. Summing per-socket deltas per process gives real
    bytes without a packet capture, a kernel module or root.

    What it does not give:
      * UDP — tcp_info is TCP only, so a QUIC-heavy process reads low;
      * sockets belonging to other users, whose owner the kernel will not
        name without root — those bytes are counted under UNATTRIBUTED
        rather than dropped, so the totals stay honest;
      * a socket that opened and closed entirely between two samples.
    """

    COMMAND = ("ss", "-tinpH")
    TIMEOUT = 4.0
    MAX_FAILURES = 3
    MAX_PROCESSES = 512
    MAX_PEERS = 4000

    def __init__(self, command: Optional[Tuple[str, ...]] = None):
        self.command = tuple(command or self.COMMAND)
        self.totals: Dict[str, Tuple[int, int]] = {}
        # Same deltas, bucketed by the endpoint instead of the process. The
        # per-socket numbers were already being parsed and then collapsed
        # away; keeping this second bucket is what lets the contact table say
        # how much an endpoint actually moved.
        self.peers: Dict[str, Tuple[int, int]] = {}
        self.error = ""
        self.enabled = self.available(self.command[0])
        if not self.enabled:
            self.error = f"{self.command[0]} not found"
        self._last: Dict[str, Tuple[int, int]] = {}
        self._peer_keys: Dict[str, str] = {}
        self._failures = 0
        # Sockets already open at startup carry a counter that accrued before
        # anyone was watching. Counting it would attribute a morning's
        # download to the first three seconds of the session, so the first
        # sample only records where every socket started.
        self._primed = False

    @staticmethod
    def available(binary: str = "ss") -> bool:
        return bool(shutil.which(binary))

    def _run(self) -> Optional[str]:
        try:
            proc = subprocess.run(self.command, capture_output=True, text=True,
                                  timeout=self.TIMEOUT, check=False)
        except (OSError, subprocess.SubprocessError) as e:
            self._failures += 1
            self.error = str(e)
            if self._failures >= self.MAX_FAILURES:
                self.enabled = False
            return None
        if proc.returncode != 0:
            self._failures += 1
            self.error = (proc.stderr or "").strip() or f"exit {proc.returncode}"
            if self._failures >= self.MAX_FAILURES:
                self.enabled = False
            return None
        self._failures = 0
        return proc.stdout

    @staticmethod
    def parse(text: str) -> Dict[str, Tuple[str, int, int]]:
        """`ss` output -> {socket key: (process, bytes_received, bytes_sent)}.

        One socket spans two lines: the address line, then an indented
        tcp_info line. A socket without an info line (a non-established one)
        simply has no counters to read."""
        out: Dict[str, Tuple[str, int, int]] = {}
        key: Optional[str] = None
        proc = UNATTRIBUTED
        for raw in (text or "").splitlines():
            if not raw.strip():
                continue
            if raw[0].isspace():
                if key is None:
                    continue
                rx = _SS_BYTES_RECV.search(raw)
                # bytes_sent is newer than bytes_acked; older kernels only
                # report what the peer acknowledged.
                tx = _SS_BYTES_SENT.search(raw) or _SS_BYTES_ACKED.search(raw)
                if rx or tx:
                    out[key] = (proc,
                                int(rx.group(1)) if rx else 0,
                                int(tx.group(1)) if tx else 0)
                key = None
                continue
            parts = raw.split()
            if len(parts) < 5:
                key = None
                continue
            key = f"{parts[3]}|{parts[4]}"
            owner = _SS_USER.search(raw)
            proc = owner.group(1) if owner else UNATTRIBUTED
        return out

    @staticmethod
    def peer_key(socket_key: str) -> str:
        """Contact key ("tcp:ip:port") for the remote end of a socket key.

        `ss -t` only ever reports TCP, so the proto is not in doubt."""
        _local, _, peer = socket_key.partition("|")
        ip, port = split_endpoint(peer)
        return f"tcp:{ip}:{port}" if ip else ""

    def _accumulate(self, seen: Dict[str, Tuple[str, int, int]]):
        for key, (proc, rx, tx) in seen.items():
            prev = self._last.get(key)
            if prev is None:
                # Unknown socket: brand new if we have sampled before,
                # pre-existing if this is the first pass.
                d_rx, d_tx = (rx, tx) if self._primed else (0, 0)
            elif rx < prev[0] or tx < prev[1]:
                # Counters only climb, so a drop means the address pair was
                # reused by a different socket. Treat it as a new one.
                d_rx, d_tx = rx, tx
            else:
                d_rx, d_tx = rx - prev[0], tx - prev[1]
            if not (d_rx or d_tx):
                continue
            total_rx, total_tx = self.totals.get(proc, (0, 0))
            self.totals[proc] = (total_rx + d_rx, total_tx + d_tx)
            peer = self._peer_keys.get(key)
            if peer is None:
                peer = self._peer_keys[key] = self.peer_key(key)
            if peer:
                # Several sockets to one endpoint simply add up, which is
                # what somebody asking "how much did this host pull" means.
                peer_rx, peer_tx = self.peers.get(peer, (0, 0))
                self.peers[peer] = (peer_rx + d_rx, peer_tx + d_tx)
        # Closed sockets leave _last; their bytes stay in the totals.
        self._last = {k: (rx, tx) for k, (_p, rx, tx) in seen.items()}
        self._peer_keys = {k: v for k, v in self._peer_keys.items() if k in self._last}
        self._primed = True
        if len(self.totals) > self.MAX_PROCESSES:
            keep = sorted(self.totals.items(), key=lambda kv: -sum(kv[1]))
            self.totals = dict(keep[:self.MAX_PROCESSES])
        if len(self.peers) > self.MAX_PEERS:
            keep = sorted(self.peers.items(), key=lambda kv: -sum(kv[1]))
            self.peers = dict(keep[:self.MAX_PEERS])

    def peer_totals(self) -> Dict[str, Tuple[int, int]]:
        """{contact key: (rx_bytes, tx_bytes)} since startup."""
        return dict(self.peers)

    def sample(self) -> Dict[str, Tuple[int, int]]:
        """Refresh and return {process: (rx_bytes, tx_bytes)} since startup."""
        if not self.enabled:
            return dict(self.totals)
        text = self._run()
        if text is None:
            return dict(self.totals)
        self._accumulate(self.parse(text))
        return dict(self.totals)

    def top(self, limit: int = 8) -> List[Tuple[str, int]]:
        """Processes by total bytes moved, busiest first."""
        rows = [(proc, rx + tx) for proc, (rx, tx) in self.totals.items() if rx + tx]
        return sorted(rows, key=lambda kv: (-kv[1], kv[0]))[:limit]


class BeaconDetector:
    """Spot endpoints that are contacted on a metronome.

    Malware that phones home does it on a timer, and regularity is the thing
    about it that is hard to hide. This measures the interval between sightings
    of an endpoint and reports its period and jitter — a measurement, not a
    verdict, because a software updater polling every ten minutes looks exactly
    the same and only the user knows which is which.

    One honest limitation, and the reason for MIN_PERIOD: a connection that
    simply stays open is re-reported by the scanner every ~20 seconds by its
    own dedupe window, which is a perfect rhythm that means nothing. Periods
    near or below that floor cannot be told apart from a connection that was
    never gone, so they are not reported at all."""

    MIN_EVENTS = 7
    MAX_EVENTS = 64
    MIN_PERIOD = 45.0
    MAX_JITTER_RATIO = 0.15
    MAX_TRACKED = 4000

    def __init__(self):
        self._events: "collections.OrderedDict[str, collections.deque]" = \
            collections.OrderedDict()

    def observe(self, key: str, ts: float):
        series = self._events.get(key)
        if series is None:
            if len(self._events) >= self.MAX_TRACKED:
                self._events.popitem(last=False)
            series = self._events[key] = collections.deque(maxlen=self.MAX_EVENTS)
        elif series and ts - series[-1] <= 0:
            return
        series.append(float(ts))
        self._events.move_to_end(key)

    def verdict(self, key: str) -> Optional[dict]:
        """{period, jitter, samples} when this endpoint keeps a rhythm."""
        series = self._events.get(key)
        if series is None or len(series) < self.MIN_EVENTS:
            return None
        gaps = [b - a for a, b in zip(series, list(series)[1:])]
        if not gaps:
            return None
        mean = sum(gaps) / len(gaps)
        if mean < self.MIN_PERIOD:
            return None
        variance = sum((g - mean) ** 2 for g in gaps) / len(gaps)
        jitter = math.sqrt(variance)
        if jitter > mean * self.MAX_JITTER_RATIO:
            return None
        return {"period": mean, "jitter": jitter, "samples": len(series)}

    @staticmethod
    def describe(verdict: dict) -> str:
        return (f"BEACON ~{verdict['period']:.0f}s ±{verdict['jitter']:.0f}s"
                f" ×{verdict['samples']}")

    def beacons(self) -> List[Tuple[str, dict]]:
        found = ((key, self.verdict(key)) for key in self._events)
        return [(k, v) for k, v in found if v]


class ProcessBaseline:
    """What each program on this machine normally talks to.

    The NEW flag asks "has this host ever contacted this address", which goes
    quiet within a day or two of ordinary use. This asks a sharper question:
    has *this program* ever talked to this network before. A browser reaching
    a new CDN is unremarkable; sshd reaching an ASN it has never used is not,
    and the global flag cannot tell those apart.

    Built from the history database alone — the user's own machine, their own
    record, checkable offline. Exactly the ground the watchlist stands on, and
    the reason this is possible without shipping a threat feed that would be
    stale the day it left."""

    #: below this many recorded contacts a process has no habits to deviate from
    MIN_OBSERVATIONS = 20
    MAX_VALUES = 256

    def __init__(self, profiles: Optional[Dict[str, dict]] = None):
        self.profiles: Dict[str, dict] = profiles or {}

    def __len__(self) -> int:
        return len(self.profiles)

    @staticmethod
    def asn_number(asn: str) -> str:
        """"AS13335 CLOUDFLARE" -> "AS13335". The organisation string is
        editorial and changes; the number is the identity."""
        token = str(asn or "").strip().split(" ", 1)[0].upper()
        return token if token.startswith("AS") else ""

    def knows(self, process: str) -> bool:
        profile = self.profiles.get(str(process or ""))
        return bool(profile) and profile.get("hits", 0) >= self.MIN_OBSERVATIONS

    def judge(self, point: dict) -> List[str]:
        """Labels for whatever this contact does that its process never has."""
        process = str(point.get("process", "") or "")
        if not self.knows(process):
            return []
        ip = str(point.get("ip", ""))
        if is_privateish(ip):
            return []  # the LAN is not a behavioural signal
        profile = self.profiles[process]
        hits: List[str] = []
        asn = self.asn_number(point.get("asn", ""))
        if asn and asn not in profile["asn"]:
            hits.append(f"{asn} NEW FOR {process}")
        country = str(point.get("country", "") or "").upper()
        if country and country not in profile["country"]:
            hits.append(f"{country} NEW FOR {process}")
        return hits


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
        " started REAL PRIMARY KEY, version TEXT, ended REAL)",
    )

    # Databases written by 0.12 and earlier have no `ended` column. Adding it
    # is the whole migration; everything else about the schema is unchanged.
    MIGRATIONS = (
        "ALTER TABLE sessions ADD COLUMN ended REAL",
    )

    RETENTION_DAYS = 30

    def __init__(self, path: Optional[str] = None):
        self.path = path or default_history_path()
        self._db: Optional[sqlite3.Connection] = None
        self.baseline: set = set()
        self.error = ""
        self._session_started: Optional[float] = None

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
            for stmt in self.MIGRATIONS:
                try:
                    db.execute(stmt)
                except sqlite3.OperationalError:
                    pass  # already applied
            db.commit()
        except (OSError, sqlite3.Error) as e:
            self.error = str(e)
            return False
        self._db = db
        # The baseline has to be read before this session writes anything,
        # otherwise every contact would immediately look already-known.
        self.baseline = self._read_baseline()
        try:
            self._session_started = time.time()
            db.execute(
                "INSERT OR REPLACE INTO sessions(started, version, ended) VALUES (?,?,?)",
                (self._session_started, APP_VERSION, self._session_started))
            db.commit()
            self.purge()
        except sqlite3.Error as e:
            self.error = str(e)
        return True

    def touch_session(self, now: Optional[float] = None):
        """Move this session's end marker forward.

        Written as we go rather than only on exit: the interesting case for
        "how long has this machine been watched" is the run that was killed,
        and that run never gets to write anything on the way out."""
        if self._db is None or self._session_started is None:
            return
        try:
            self._db.execute("UPDATE sessions SET ended=? WHERE started=?",
                             (time.time() if now is None else now,
                              self._session_started))
            self._db.commit()
        except sqlite3.Error:
            pass

    def process_profiles(self, limit: int = 256) -> Dict[str, dict]:
        """{process: {asn: set, country: set, hits: n}} from recorded history."""
        if self._db is None:
            return {}
        try:
            rows = self._db.execute(
                "SELECT process, asn, country, SUM(hits) FROM contacts"
                " WHERE process != '' GROUP BY process, asn, country").fetchall()
        except sqlite3.Error:
            return {}
        profiles: Dict[str, dict] = {}
        for process, asn, country, hits in rows:
            profile = profiles.get(process)
            if profile is None:
                if len(profiles) >= limit:
                    continue
                profile = profiles[process] = {"asn": set(), "country": set(), "hits": 0}
            profile["hits"] += int(hits or 0)
            number = ProcessBaseline.asn_number(asn)
            if number and len(profile["asn"]) < ProcessBaseline.MAX_VALUES:
                profile["asn"].add(number)
            cc = str(country or "").upper()
            if cc and len(profile["country"]) < ProcessBaseline.MAX_VALUES:
                profile["country"].add(cc)
        return profiles

    def sessions_summary(self) -> dict:
        """{sessions, first, watched} — runs recorded, when watching began,
        and the total time covered."""
        empty = {"sessions": 0, "first": 0.0, "watched": 0.0}
        if self._db is None:
            return empty
        try:
            row = self._db.execute(
                "SELECT COUNT(*), MIN(started),"
                " TOTAL(MAX(COALESCE(ended, started) - started, 0)) FROM sessions"
            ).fetchone()
        except sqlite3.Error:
            return empty
        if not row or not row[0]:
            return empty
        return {"sessions": int(row[0]), "first": float(row[1] or 0.0),
                "watched": float(row[2] or 0.0)}

    def close(self):
        if self._db is None:
            return
        self.touch_session()
        try:
            self._db.commit()
            self._db.close()
        except sqlite3.Error:
            pass
        self._db = None
        self._session_started = None

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

    def clear(self) -> bool:
        """Drop every recorded contact and sample, keeping the file."""
        if self._db is None:
            return False
        try:
            self._db.execute("DELETE FROM contacts")
            self._db.execute("DELETE FROM samples")
            self._db.commit()
        except sqlite3.Error as e:
            self.error = str(e)
            return False
        self.baseline = set()
        return True

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


_CSV_INJECTION_PREFIXES = ("=", "+", "-", "@", "\t", "\r")


def csv_safe(value) -> str:
    """Defuse a spreadsheet formula before it reaches a CSV cell.

    Almost everything exported here is attacker-influenced: a PTR record, a
    GeoIP organisation string and a process name all arrive from outside and
    all land in a file somebody opens in Excel or LibreOffice, where a leading
    '=' is a formula, not text. Prefixing an apostrophe is the standard
    neutralisation and leaves the value readable."""
    text = "" if value is None else str(value)
    if text.startswith(_CSV_INJECTION_PREFIXES):
        return "'" + text
    return text


_PORT_RANGE_PATH = "/proc/sys/net/ipv4/ip_local_port_range"
_DEFAULT_PORT_RANGE = (32768, 60999)
_port_range_cache: Optional[Tuple[int, int]] = None


def local_port_range() -> Tuple[int, int]:
    """The kernel's ephemeral port range, as the kernel reports it."""
    global _port_range_cache
    if _port_range_cache is None:
        try:
            with open(_PORT_RANGE_PATH, encoding="utf-8") as f:
                lo, hi = (int(x) for x in f.read().split()[:2])
            _port_range_cache = (lo, hi) if lo <= hi else _DEFAULT_PORT_RANGE
        except (OSError, ValueError):
            _port_range_cache = _DEFAULT_PORT_RANGE
    return _port_range_cache


def is_ephemeral_port(port) -> bool:
    lo, hi = local_port_range()
    try:
        return lo <= int(port) <= hi
    except (TypeError, ValueError):
        return False


def is_notable_listener(listener: dict) -> bool:
    """Is a socket appearing or vanishing worth telling the user about?

    A client making outbound connections constantly opens and closes sockets
    on ephemeral ports — syncthing alone churns several a minute. Those are
    not changes in attack surface, and an alert channel that cries wolf every
    ten seconds is one nobody reads. They stay in the LISTEN inventory, which
    is meant to be the complete `ss -ltun` picture; they just do not ring a
    bell. A port carrying a known service name or a sensitive note is always
    notable, so a WireGuard endpoint high in the ephemeral range still
    announces itself."""
    port = int(listener.get("port", 0) or 0)
    if listener.get("note") or port_service(port):
        return True
    return not is_ephemeral_port(port)


def listener_key(listener: dict) -> str:
    return f"{listener.get('proto')}:{listener.get('ip')}:{listener.get('port')}"


def listener_diff(previous: Optional[Dict[str, dict]],
                  current: Dict[str, dict]) -> Tuple[List[dict], List[dict]]:
    """(opened, closed) between two listening-socket inventories.

    `previous` of None means there is no earlier photograph to compare
    against, so nothing opened — the sockets already up when the app started
    are the baseline, not ninety-eight ports springing open at once."""
    if previous is None:
        return [], []
    opened = [current[k] for k in sorted(set(current) - set(previous))]
    closed = [previous[k] for k in sorted(set(previous) - set(current))]
    return opened, closed


def _stamp(ts: float) -> str:
    """Absolute local time — history spans days, so H:M:S alone would lie."""
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(float(ts)))
    except (TypeError, ValueError, OSError):
        return ""


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

    def set_interval(self, secs: float):
        self._mtx.lock()
        self.interval = max(0.2, float(secs))
        self._stop_evt.wakeAll()  # cut the current sleep short
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

            snap = build_snapshot(self._prev, now, dt, hide_virtual=self.hide_virtual)
            self._prev = now
            prev_t = now_t
            self.traffic.emit(snap)


class ConnScanner(QtCore.QThread):
    points = QtCore.pyqtSignal(list)
    listeners = QtCore.pyqtSignal(list)
    states = QtCore.pyqtSignal(dict)
    event = QtCore.pyqtSignal(str)
    #: {process: (rx_bytes, tx_bytes)} accumulated since startup
    procIo = QtCore.pyqtSignal(dict)
    #: {contact key: (rx_bytes, tx_bytes)} accumulated since startup
    peerIo = QtCore.pyqtSignal(dict)
    #: how much of the last scan this process was allowed to see
    scanStats = QtCore.pyqtSignal(dict)

    def __init__(self, interval=3.0, parent=None, rdns: Optional[RDnsCache] = None,
                 proc_io: Optional[ProcTrafficSampler] = None):
        super().__init__(parent)
        self.interval = interval
        self._stop_evt = QtCore.QWaitCondition()
        self._mtx = QtCore.QMutex()
        self._stop = False
        self._seen: Dict[str, float] = {}
        # Read-only here: the resolver thread fills it, the scanner only
        # stamps whatever name is already known onto a fresh contact.
        self.rdns = rdns if rdns is not None else RDnsCache()
        # Cleared at the start of every scan: a pid can be reused.
        self._pid_names: Dict[int, str] = {}
        self.proc_io = proc_io if proc_io is not None else ProcTrafficSampler()
        self.scan_stats: Dict[str, int] = {}

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

    def set_interval(self, secs: float):
        """Change the cadence and apply it now rather than after the sleep
        already in progress — a user dropping 30s to 1s should not have to
        wait half a minute to find out whether it worked."""
        self._mtx.lock()
        self.interval = max(0.2, float(secs))
        self._stop_evt.wakeAll()
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

    def _proc_name(self, pid: Optional[int]) -> str:
        """Process name for a pid, memoised for the duration of one scan.

        A busy host has thousands of sockets but only dozens of owning
        processes, and every miss here is a /proc walk. Without the cache the
        scanner re-reads the same process name once per socket, every three
        seconds."""
        if not pid:
            return ""
        cached = self._pid_names.get(pid)
        if cached is not None:
            return cached
        try:
            name = psutil.Process(pid).name()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            name = ""
        self._pid_names[pid] = name
        return name

    def _scan_psutil(self) -> Tuple[List[ConnPoint], List[ListenPoint], Dict[str, int]]:
        """One pass over the socket table: new remote contacts, the current
        listening set, and a histogram of connection states. All three come
        from the same syscall — the listeners and the states used to be
        discarded, which is most of what psutil actually hands us."""
        out: List[ConnPoint] = []
        listening: List[ListenPoint] = []
        state_counts: Dict[str, int] = {}
        self._pid_names = {}
        remote = unattributed = 0
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

            udp = c.type == socket.SOCK_DGRAM
            if not c.raddr:
                # A UDP socket has no LISTEN state — psutil reports NONE for
                # every one of them — but a bound UDP socket still receives
                # from whatever its bind address lets in. Keying the listening
                # set on CONN_LISTEN alone silently dropped every one of them,
                # so DNS, SSDP, WireGuard and friends never showed up on the
                # tab that calls itself the attack surface. This is the set
                # `ss -ltun` reports.
                if c.laddr and (status == psutil.CONN_LISTEN or udp):
                    lip = self._normalize_ip(c.laddr.ip)
                    lport = int(c.laddr.port)
                    exposure = exposure_class(c.laddr.ip)
                    listening.append(ListenPoint(
                        ip=lip, port=lport,
                        proto="udp" if udp else "tcp",
                        process=self._proc_name(c.pid), pid=int(c.pid or 0),
                        exposure=exposure, note=listener_risk(lport, exposure),
                    ))
                continue
            # Normalize first: an IPv4-mapped v6 socket and a plain v4
            # socket to the same endpoint must share one dedupe key, and
            # ::ffff:127.0.0.1 has to be filtered as loopback.
            ip = self._normalize_ip(c.raddr.ip)
            port = int(c.raddr.port)
            proto = "udp" if udp else "tcp"
            key = f"{proto}:{ip}:{port}"
            if is_loopback_ip(ip):
                continue

            # Counted before the dedupe, over the whole socket table: the
            # share of sockets whose owner the kernel will not name is a fact
            # about this process's privileges, not about the handful of
            # endpoints that happened to be new this scan. Reading c.pid is
            # free — _proc_name, which walks /proc, stays behind the dedupe.
            remote += 1
            if not c.pid:
                unattributed += 1

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

        self.scan_stats = {
            "remote": remote,
            "unattributed": unattributed,
            "listeners": len(listening),
            "elevated": int(is_elevated()),
        }
        return out, listening, state_counts

    def run(self):
        while not self._stop:
            pts, listening, states = self._scan_psutil()
            self.listeners.emit([lp.__dict__ for lp in listening])
            self.states.emit(states)
            self.scanStats.emit(dict(self.scan_stats))
            if pts:
                self.points.emit([p.__dict__ for p in pts])
                self.event.emit(f"[+] contacts detected: {len(pts)}")
            was_enabled = self.proc_io.enabled
            self.procIo.emit(self.proc_io.sample())
            self.peerIo.emit(self.proc_io.peer_totals())
            if was_enabled and not self.proc_io.enabled:
                self.event.emit(
                    f"[!] per-process byte accounting disabled: {self.proc_io.error}")
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


def tray_icon(alert: bool = False) -> QtGui.QIcon:
    """The tray image, with a red pip when something is flagged.

    Falls back to a drawn disc when the packaged icon is not on disk, so a
    source checkout gets a tray icon rather than an invisible one."""
    path = resource_path("icon.png", "icon.ico")
    pix = QtGui.QPixmap(path) if path else QtGui.QPixmap()
    if pix.isNull():
        pix = QtGui.QPixmap(32, 32)
        pix.fill(Qt.GlobalColor.transparent)
        p = QtGui.QPainter(pix)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        p.setPen(QtGui.QPen(PHOSPHOR, 2))
        p.setBrush(QtGui.QBrush(BG))
        p.drawEllipse(3, 3, 26, 26)
        p.end()
    else:
        pix = pix.scaled(32, 32, Qt.AspectRatioMode.KeepAspectRatio,
                         Qt.TransformationMode.SmoothTransformation)
    if alert:
        p = QtGui.QPainter(pix)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QBrush(RED))
        p.drawEllipse(pix.width() - 13, pix.height() - 13, 12, 12)
        p.end()
    return QtGui.QIcon(pix)


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


class SocketHealthMonitor:
    """Make the socket-state histogram say something.

    state_color() already explains why these two states matter — a pile of
    CLOSE_WAIT is a process leaking sockets, a pile of SYN_SENT is something
    failing to connect — and then the app drew the bar and said nothing. A
    level alone is not news (a busy server legitimately holds thousands of
    TIME_WAIT), so a report needs both a threshold and a climb."""

    WINDOW = 10
    MIN_SAMPLES = 4
    REPEAT_SECONDS = 300.0
    THRESHOLDS = {"CLOSE_WAIT": 32, "SYN_SENT": 16}
    EXPLANATIONS = {
        "CLOSE_WAIT": "a process is not closing its sockets",
        "SYN_SENT": "connections are not completing — unreachable host, "
                    "blocked port or packet loss",
    }

    def __init__(self):
        self._series: Dict[str, collections.deque] = {}
        self._reported: Dict[str, float] = {}

    def feed(self, counts: Dict[str, int], now: Optional[float] = None) -> List[str]:
        """Record one histogram; return the lines worth logging."""
        now = time.time() if now is None else now
        messages: List[str] = []
        for state, threshold in self.THRESHOLDS.items():
            n = int((counts or {}).get(state, 0) or 0)
            series = self._series.setdefault(state, collections.deque(maxlen=self.WINDOW))
            series.append(n)
            if len(series) < self.MIN_SAMPLES or n < threshold:
                continue
            if n <= series[0]:
                continue  # at or below where this window started: not climbing
            last = self._reported.get(state)
            if last is not None and now - last < self.REPEAT_SECONDS:
                continue
            self._reported[state] = now
            messages.append(
                f"{n} sockets in {state} and climbing (from {series[0]}) — "
                f"{self.EXPLANATIONS[state]}")
        return messages


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

    #: the label of a bar the user clicked
    rowActivated = QtCore.pyqtSignal(str)

    # Painting and hit-testing have to agree about where the rows are, so
    # both read these rather than each carrying its own copy of 26/15/12.
    ROW_TOP = 26
    ROW_STEP = 15
    ROW_H = 12

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self._rows: List[Tuple[str, int]] = []
        self._fmt = str
        self.setMinimumHeight(140)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Click a row to filter the contact table by it")

    def set_title(self, title: str):
        self.title = title
        self.update()

    def set_rows(self, rows: List[Tuple[str, int]], value_fmt=None):
        """`value_fmt` renders the number beside each bar — counts by default,
        but the same panel also has to say "12.3 MB"."""
        self._rows = list(rows or [])
        self._fmt = value_fmt or str
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
        y = self.ROW_TOP
        w = self.width()
        for name, n in self._rows:
            if y + self.ROW_H + 2 > self.height():
                break
            frac = n / top if top else 0.0
            p.fillRect(QtCore.QRectF(8, y, max(1.0, frac * (w - 16)), 12),
                       QtGui.QColor(0, 170, 68, 90))
            p.setPen(QtGui.QPen(PHOSPHOR))
            p.drawText(QtCore.QRectF(12, y, w - 100, 12),
                       int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                       name)
            p.setPen(QtGui.QPen(AMBER))
            p.drawText(QtCore.QRectF(w - 88, y, 80, self.ROW_H),
                       int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter),
                       self._fmt(n))
            y += self.ROW_STEP

    def row_at(self, pos: QtCore.QPoint) -> Optional[str]:
        """The label of the bar under a point, or None."""
        index = (pos.y() - self.ROW_TOP) // self.ROW_STEP
        if index < 0 or index >= len(self._rows):
            return None
        if (pos.y() - self.ROW_TOP) % self.ROW_STEP > self.ROW_H:
            return None  # the gap between bars is not a bar
        return self._rows[int(index)][0]

    def mousePressEvent(self, e):
        name = self.row_at(e.pos())
        if name:
            self.rowActivated.emit(name)
        super().mousePressEvent(e)


class ListenersTab(QtWidgets.QWidget):
    """What this host is accepting on — the sockets the scanner used to skip.

    A remote contact says who the machine reached out to; this says who can
    reach in, which for anything calling itself a defender is the more
    important half."""

    COLS = ["Proto", "Bind", "Port/Svc", "Exposure", "Process", "PID", "Note"]
    C_PROTO, C_BIND, C_PORT, C_EXPOSURE, C_PROCESS, C_PID, C_NOTE = range(7)

    snapshotRequested = QtCore.pyqtSignal()
    csvExportRequested = QtCore.pyqtSignal()
    #: sockets that appeared / disappeared since the previous scan
    listenersOpened = QtCore.pyqtSignal(list)
    listenersClosed = QtCore.pyqtSignal(list)

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
        # None until the first scan lands: the sockets already open when the
        # app started are the baseline, not ninety-eight ports "opening".
        self._prev: Optional[Dict[str, dict]] = None

    def _on_world_only(self, on: bool):
        self.proxy.setFilterFixedString(EXPOSURE_WORLD if on else "")

    def set_listeners(self, listeners: List[dict]):
        """Listening sockets are a state, not a stream: rebuild whenever the
        set changes so a service that stopped disappears from the table."""
        current = {listener_key(l): l for l in listeners}
        keys = sorted(current)
        self._emit_diff(current)
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

    def _emit_diff(self, current: Dict[str, dict]):
        """Report what changed since the last scan.

        A listening-socket inventory is a photograph; the difference between
        two of them is the thing worth waking somebody for. The set was
        already being computed to decide whether to rebuild the table — it
        was just thrown away."""
        previous, self._prev = self._prev, dict(current)
        opened, closed = listener_diff(previous, current)
        if opened:
            self.listenersOpened.emit(opened)
        if closed:
            self.listenersClosed.emit(closed)

    @staticmethod
    def describe(listener: dict) -> str:
        """One line for a socket, for the event log and the alert banner."""
        proto = str(listener.get("proto", "")).upper()
        port = int(listener.get("port", 0))
        svc = port_service(port)
        where = f"{listener.get('ip', '')}:{port}"
        process = str(listener.get("process", "") or "")
        pid = int(listener.get("pid", 0) or 0)
        bits = [f"{proto} {where}"]
        if svc:
            bits.append(f"({svc})")
        if process:
            bits.append(f"[{process}{f'/{pid}' if pid else ''}]")
        bits.append(str(listener.get("exposure", EXPOSURE_UNKNOWN)))
        note = str(listener.get("note", "") or "")
        if note:
            bits.append(f"· {note}")
        return " ".join(bits)

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
                w.writerow([csv_safe(self.proxy.index(r, c).data())
                            for c in range(self.proxy.columnCount())])


class BlackIceDashboard(QtWidgets.QWidget):
    STATUS_BASE = "MODE: SOCKET TABLE (psutil) — NO PACKET CAPTURE, NO FILTERING"

    #: seconds between socket scans, offered in the SCAN combo
    SCAN_INTERVALS = (1.0, 3.0, 5.0, 10.0, 30.0)

    snapshotRequested = QtCore.pyqtSignal()
    configToggled = QtCore.pyqtSignal(bool)
    historyToggled = QtCore.pyqtSignal(bool)
    hideVirtualToggled = QtCore.pyqtSignal(bool)
    fxChanged = QtCore.pyqtSignal(str)
    freezeToggled = QtCore.pyqtSignal(bool)
    scanIntervalChanged = QtCore.pyqtSignal(float)
    trayToggled = QtCore.pyqtSignal(bool)

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

        # What this line used to claim — "PASSIVE SNIFF", "SHIELD: SYN-FILTER
        # (SIM)" — the app does not do: it reads the kernel socket table and
        # filters nothing. Flavour text that overstates a security tool's
        # reach is the one kind this tool should not ship, so the line now
        # reports the capabilities actually loaded.
        self.sub = QtWidgets.QLabel(self.STATUS_BASE)
        self.sub.setFont(HackerFont.mono(10))
        self.sub.setStyleSheet("color: #00aa44;")
        self._status: "collections.OrderedDict[str, str]" = collections.OrderedDict()

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

        self.tray_btn = _hacker_button("▭ TRAY", checkable=True)
        self.tray_btn.setToolTip(
            "Keep watching when the window is closed.\n"
            "A sentinel that stops when you close it is not a sentinel.")
        self.tray_btn.toggled.connect(self.trayToggled.emit)

        self.freeze_btn = _hacker_button("⏸ FREEZE", checkable=True)
        self.freeze_btn.setToolTip(
            "Hold the tables, graphs and map still so you can read them.\n"
            "Scanning, recording and the event log carry on underneath.")
        self.freeze_btn.toggled.connect(self.freezeToggled.emit)

        self.scan_combo = QtWidgets.QComboBox()
        self.scan_combo.setFont(HackerFont.mono(9))
        for secs in self.SCAN_INTERVALS:
            self.scan_combo.addItem(f"{secs:g}s", secs)
        self.scan_combo.setCurrentIndex(self.SCAN_INTERVALS.index(3.0))
        self.scan_combo.setToolTip(
            "How often the socket table is read. Every scan walks /proc, so a\n"
            "longer interval is cheaper on a busy host or a battery.")
        self.scan_combo.setStyleSheet(
            "QComboBox { background: #07100a; color:#00aa44; border: 1px solid #0b2a12; padding: 2px; }"
            "QAbstractItemView { background: #07100a; color:#00ff66; selection-background-color:#0b2a12; }")
        self.scan_combo.currentIndexChanged.connect(
            lambda _i: self.scanIntervalChanged.emit(self.scan_interval()))

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
        top.addSpacing(8)
        scan_lbl = QtWidgets.QLabel("SCAN:")
        scan_lbl.setFont(HackerFont.mono(9))
        scan_lbl.setStyleSheet("color:#00aa44;")
        top.addWidget(scan_lbl)
        top.addWidget(self.scan_combo)
        top.addSpacing(12)
        top.addWidget(self.freeze_btn)
        top.addWidget(self.tray_btn)
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

    def set_tray_enabled(self, on: bool):
        self.tray_btn.blockSignals(True)
        self.tray_btn.setChecked(bool(on))
        self.tray_btn.blockSignals(False)

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

    def set_status(self, key: str, value: str):
        """Append or update one capability reading on the status line."""
        self._status[key] = value
        parts = [self.STATUS_BASE] + [f"{k}: {v}" for k, v in self._status.items()]
        self.sub.setText("  |  ".join(parts))

    def scan_interval(self) -> float:
        value = self.scan_combo.currentData()
        return float(value) if value else 3.0

    def set_scan_interval(self, secs: float):
        """Apply a stored interval without re-emitting scanIntervalChanged."""
        index = self.scan_combo.findData(float(secs))
        if index < 0:
            return
        self.scan_combo.blockSignals(True)
        self.scan_combo.setCurrentIndex(index)
        self.scan_combo.blockSignals(False)

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
  __LEAFLET_CSS__
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
__LEAFLET_JS__
<script>
  const map = L.map('map', { zoomControl: true }).setView([20, 0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

  const markers = new Map();

  // Every string reaching a popup comes from outside: a GeoIP city or org
  // name, or the label ipapi.co returned. bindPopup() parses a string as
  // HTML, so it is handed a text node instead.
  function asText(s) {
    const node = document.createElement('span');
    node.textContent = s == null ? '' : String(s);
    return node;
  }

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
      meMarker.bindPopup(asText(me.label));
    } else {
      meMarker.setLatLng([me.lat, me.lon]);
      meMarker.setPopupContent(asText(me.label));
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
        m.setPopupContent(asText(label));
      } else {
        const m = L.circleMarker([lat, lon], {
          radius: 6, weight: 2, color: "#00ff66",
          fillColor: "#00ff66", fillOpacity: 0.25
        }).addTo(map);
        m.bindPopup(asText(label));
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


LEAFLET_CDN = "https://unpkg.com/leaflet@1.9.4/dist/"
LEAFLET_ASSETS = ("leaflet.css", "leaflet.js")


def _leaflet_asset(name: str) -> Optional[str]:
    path = resource_path(os.path.join("leaflet", name), name)
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return f.read()
    except OSError:
        return None


def leaflet_html() -> Tuple[str, bool]:
    """(page, vendored). Inlines the bundled Leaflet when it is there.

    ONLINE mode used to pull its JavaScript from a CDN every time the tab
    opened — a third party able to change the code running inside a security
    tool, on every launch. Leaflet is BSD-2-Clause and 160 KB; it ships in
    resources/leaflet with its licence. Tiles still come from OpenStreetMap,
    which is what ONLINE mode is for, but the code no longer does.

    Inlined rather than linked because QWebEngineView.setHtml() has no base
    URL to resolve a relative path against. Falls back to the CDN if the
    vendored copy is missing, and says which it used."""
    css = _leaflet_asset("leaflet.css")
    js = _leaflet_asset("leaflet.js")
    if css is not None and js is not None:
        return (LEAFLET_HTML
                .replace("__LEAFLET_CSS__", f"<style>{css}</style>")
                .replace("__LEAFLET_JS__", f"<script>{js}</script>"), True)
    return (LEAFLET_HTML
            .replace("__LEAFLET_CSS__",
                     f'<link rel="stylesheet" href="{LEAFLET_CDN}leaflet.css"/>')
            .replace("__LEAFLET_JS__",
                     f'<script src="{LEAFLET_CDN}leaflet.js"></script>'), False)


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

    Deliberately not a web view: the Leaflet map needs a tile server to draw
    anything, renders in a separate process so snapshots come out black, and
    cannot be told anything until its page has finished loading. None of that
    is true here."""

    MIN_ZOOM = 1.0
    MAX_ZOOM = 12.0
    # Every dot is redrawn on every repaint, and ray mode draws a line per dot
    # as well, so an uncapped set turns a long session into a slideshow.
    MAX_POINTS = 2000
    # Below this zoom a hundred dots on one city are a single blob, so they
    # are rolled up by the network that owns them instead.
    CLUSTER_ZOOM = 3.0
    CLUSTER_GRID = 5.0

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
        self.cluster = True
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
            key = f"{p.get('proto')}:{p.get('ip')}:{p.get('port')}"
            self._points.pop(key, None)  # re-insert so dicts keep arrival order
            self._points[key] = p
        while len(self._points) > self.MAX_POINTS:
            self._points.pop(next(iter(self._points)))
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

    def set_cluster(self, on: bool):
        self.cluster = bool(on)
        self.update()

    def clustering(self) -> bool:
        """Rolled up only while zoomed out — zoom in and the individual
        endpoints come back, which is the whole reason to zoom in."""
        return self.cluster and self.zoom < self.CLUSTER_ZOOM

    def clusters(self) -> List[dict]:
        """Contacts grouped by owning network, positioned at the mean of
        their members.

        ASN is the right grouping because it is what the dots actually have
        in common: forty markers on Ashburn are one Amazon, and saying
        "AS16509 · 40" is the fact the map was being asked for. Contacts with
        no ASN — no ASN database loaded, or a network that has none — fall
        back to a coarse geographic cell so they are still rolled up rather
        than drawn forty times on one pixel."""
        groups: "collections.OrderedDict[str, dict]" = collections.OrderedDict()
        for point in self._points.values():
            lat = float(point.get("lat", 0.0) or 0.0)
            lon = float(point.get("lon", 0.0) or 0.0)
            asn = str(point.get("asn", "") or "").strip()
            if asn:
                key = asn.split(" ", 1)[0].upper()
                name = asn
            else:
                cell_lat = math.floor(lat / self.CLUSTER_GRID) * self.CLUSTER_GRID
                cell_lon = math.floor(lon / self.CLUSTER_GRID) * self.CLUSTER_GRID
                key = f"@{cell_lat:.0f},{cell_lon:.0f}"
                name = ""
            group = groups.get(key)
            if group is None:
                group = {"key": key, "name": name, "count": 0,
                         "lat_sum": 0.0, "lon_sum": 0.0, "members": []}
                groups[key] = group
            group["count"] += 1
            group["lat_sum"] += lat
            group["lon_sum"] += lon
            if len(group["members"]) < 12:
                group["members"].append(point)

        out: List[dict] = []
        for group in groups.values():
            n = group["count"]
            lat = group["lat_sum"] / n
            lon = group["lon_sum"] / n
            if group["name"]:
                label = f"{group['name']} — {n} contact(s)"
            else:
                label = f"{n} contact(s) near ({lat:.1f}, {lon:.1f})"
            sample = ", ".join(f"{m.get('ip')}:{m.get('port')}"
                               for m in group["members"][:6])
            if sample:
                label += f"\n{sample}" + (" …" if n > 6 else "")
            out.append({"key": group["key"], "label": label, "count": n,
                        "lat": lat, "lon": lon})
        return sorted(out, key=lambda c: -c["count"])

    @staticmethod
    def cluster_radius(count: int) -> float:
        return min(16.0, 4.0 + 2.2 * math.sqrt(max(1, count) - 1))

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
        """The nearest contact (or cluster) within `radius` pixels, for hover
        tooltips. Hit-testing has to follow whatever is actually drawn, or the
        tooltip names an endpoint the user cannot see."""
        best = None
        best_d = radius * radius
        candidates = self.clusters() if self.clustering() else self._points.values()
        for p in candidates:
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

        clustered = self.clustering()
        drawn = self.clusters() if clustered else list(self._points.values())

        if me_pt and self.show_rays:
            p.setPen(QtGui.QPen(QtGui.QColor(255, 153, 0, 140), 1))
            for point in drawn:
                x, y = self.project(float(point.get("lon", 0.0)),
                                    float(point.get("lat", 0.0)))
                p.drawLine(QtCore.QPointF(*me_pt), QtCore.QPointF(x, y))

        p.setPen(QtGui.QPen(PHOSPHOR, 1))
        p.setBrush(QtGui.QBrush(QtGui.QColor(0, 255, 102, 70)))
        for point in drawn:
            x, y = self.project(float(point.get("lon", 0.0)),
                                float(point.get("lat", 0.0)))
            r = self.cluster_radius(point["count"]) if clustered else 4.0
            p.drawEllipse(QtCore.QPointF(x, y), r, r)

        if clustered:
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setFont(HackerFont.mono(8, bold=True))
            p.setPen(QtGui.QPen(AMBER))
            for point in drawn:
                if point["count"] < 2:
                    continue
                x, y = self.project(float(point.get("lon", 0.0)),
                                    float(point.get("lat", 0.0)))
                r = self.cluster_radius(point["count"])
                p.drawText(QtCore.QPointF(x + r + 2, y + 3), str(point["count"]))

        if me_pt:
            p.setPen(QtGui.QPen(QtGui.QColor("#ff9900"), 2))
            p.setBrush(QtGui.QBrush(QtGui.QColor(255, 153, 0, 90)))
            p.drawEllipse(QtCore.QPointF(*me_pt), 6.0, 6.0)

        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setFont(HackerFont.mono(9))
        p.setPen(QtGui.QPen(PHOSPHOR_DIM))
        note = "OFFLINE OUTLINE" if len(self.outline) else "OUTLINE MISSING"
        rollup = (f" · {len(drawn)} clusters (zoom past {self.CLUSTER_ZOOM:.0f}x to split)"
                  if clustered else "")
        p.drawText(8, h - 8,
                   f"{note} · {len(self._points)} contacts{rollup}"
                   f" · zoom {self.zoom:.1f}x"
                   " · wheel zoom, drag pan, double-click reset")


class LocateBridge(QtCore.QObject):
    """Marshals locate-worker results back onto the GUI thread."""
    located = QtCore.pyqtSignal(float, float, str)
    failed = QtCore.pyqtSignal(str)


class MapTab(QtWidgets.QWidget):
    snapshotRequested = QtCore.pyqtSignal()
    #: raised whenever something here is about to reach a third party
    externalLookup = QtCore.pyqtSignal(str)

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
            "ONLINE adds OpenStreetMap tiles, which does reach the internet."
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
            page, self.leaflet_vendored = leaflet_html()
            self.web.setHtml(page)
        else:
            self.web = QtWidgets.QLabel(
                "QtWebEngine not installed.\n\nInstall PyQt6-WebEngine to enable the MAP tab."
            )
            self.web.setWordWrap(True)
            self.web.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.web.setFont(HackerFont.mono(11))
            self.web.setStyleSheet("color:#ffcc33; background:#07100a; border:1px solid #0b2a12; padding:20px;")
            self._page_ready = True  # nothing to wait for; _exec_js no-ops
            self.leaflet_vendored = False

        self.world = WorldMapWidget(WorldOutline.load())

        # Both maps are kept fed so switching is instant; only one is shown.
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.world)
        self.stack.addWidget(self.web)
        # A QStackedWidget's implicit minimum is the widest of all its pages,
        # shown or not, so the web view (or the long "not installed" notice)
        # set a floor the map pane could not be dragged below — which also
        # meant a saved splitter position narrower than that floor was
        # silently ignored on restore.
        self.stack.setMinimumWidth(200)
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

        self.cluster_enable = QtWidgets.QCheckBox("Cluster by ASN")
        self.cluster_enable.setFont(HackerFont.mono(10))
        self.cluster_enable.setChecked(True)
        self.cluster_enable.setToolTip(
            "While zoomed out, roll contacts up by the network that owns them.\n"
            "Zoom past 3x to see individual endpoints again.")
        self.cluster_enable.setStyleSheet("QCheckBox { color:#00aa44; }")
        self.cluster_enable.toggled.connect(self.world.set_cluster)

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
        row.addWidget(self.cluster_enable)
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
        # Geolocating this host means handing its public address to somebody
        # else. That belongs in the main event log, not only in this tab's
        # own scroller, however the lookup was triggered.
        self.externalLookup.emit(
            "geolocating this host — sending your public IP to ifconfig.co / ipapi.co")
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


class WatchlistDialog(QtWidgets.QDialog):
    """Edit the watchlist without leaving the app.

    A table of rules would be tidier, but the file is documented as plain and
    hand-editable and it carries the user's own comments — a structured
    editor would quietly eat them on the first save. So this edits the text,
    and earns its keep by parsing on every keystroke: the rule count and the
    exact line number of anything malformed are visible before saving, which
    the text editor the user would otherwise reach for cannot do."""

    def __init__(self, path: str, parent=None):
        super().__init__(parent)
        self.path = path
        self.setWindowTitle(f"{APP_NAME} — watchlist rules")
        self.resize(760, 560)
        self.setStyleSheet("QDialog { background:#050a06; }")

        self.header = QtWidgets.QLabel(f"⚑ {path}")
        self.header.setFont(HackerFont.mono(10))
        self.header.setStyleSheet("color:#00aa44;")

        self.syntax = QtWidgets.QLabel(
            "kind value [label]   ·   kinds: "
            + ", ".join(RULE_KINDS)
            + "   ·   # starts a comment")
        self.syntax.setFont(HackerFont.mono(9))
        self.syntax.setStyleSheet("color:#00aa44;")

        self.edit = QtWidgets.QPlainTextEdit()
        self.edit.setFont(HackerFont.mono(10))
        self.edit.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.edit.setStyleSheet(
            "QPlainTextEdit { background:#07100a; color:#00ff66;"
            " border:1px solid #0b2a12; selection-background-color:#0b2a12; }")
        self.edit.setTabChangesFocus(True)

        self.status = QtWidgets.QLabel("")
        self.status.setFont(HackerFont.mono(9))
        self.status.setWordWrap(True)

        self.save_btn = _hacker_button("▣ SAVE")
        self.reload_btn = _hacker_button("↻ RELOAD FROM DISK")
        self.cancel_btn = _hacker_button("✕ CANCEL")

        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.reload_btn)
        buttons.addStretch(1)
        buttons.addWidget(self.cancel_btn)
        buttons.addWidget(self.save_btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(self.header)
        layout.addWidget(self.syntax)
        layout.addWidget(self.edit, 1)
        layout.addWidget(self.status)
        layout.addLayout(buttons)

        self.edit.textChanged.connect(self._validate)
        self.save_btn.clicked.connect(self._save)
        self.reload_btn.clicked.connect(self.reload)
        self.cancel_btn.clicked.connect(self.reject)

        self.saved = False
        self.error = ""
        self.reload()

    def text(self) -> str:
        return self.edit.toPlainText()

    def reload(self):
        """Re-read the file, falling back to the commented template so a
        first-time user is not handed an empty box."""
        try:
            with open(self.path, encoding="utf-8") as f:
                body = f.read()
        except FileNotFoundError:
            body = WATCHLIST_TEMPLATE
        except OSError as e:
            body = WATCHLIST_TEMPLATE
            self.status.setText(f"could not read {self.path}: {e}")
        self.edit.setPlainText(body)
        self._validate()

    def _validate(self) -> "RuleSet":
        rules = RuleSet.parse(self.text())
        if rules.errors:
            self.status.setStyleSheet("color:#ff3355;")
            shown = "   ".join(rules.errors[:4])
            more = f"   (+{len(rules.errors) - 4} more)" if len(rules.errors) > 4 else ""
            self.status.setText(
                f"{len(rules)} rule(s) · {len(rules.errors)} problem(s): {shown}{more}")
        else:
            self.status.setStyleSheet("color:#00aa44;")
            self.status.setText(f"{len(rules)} rule(s) · no problems")
        # Malformed lines are skipped, not fatal, so saving stays allowed —
        # the point is that the user can see what will be ignored.
        return rules

    def _save(self):
        body = self.text()
        if body and not body.endswith("\n"):
            body += "\n"
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as f:
                f.write(body)
        except OSError as e:
            self.error = str(e)
            self.status.setStyleSheet("color:#ff3355;")
            self.status.setText(f"could not write {self.path}: {e}")
            return
        self.saved = True
        self.accept()


class ConnectionsTab(QtWidgets.QWidget):
    """Sortable, filterable live table of remote contacts."""

    COLS = ["First Seen", "Last Seen", "Proto", "State", "IP", "Host",
            "Port/Svc", "Location", "Process", "ASN/Org", "Hits",
            "RX", "TX", "Flags"]
    (C_FIRST, C_LAST, C_PROTO, C_STATE, C_IP, C_HOST,
     C_PORT, C_LOCATION, C_PROCESS, C_ASN, C_HITS,
     C_RX, C_TX, C_FLAGS) = range(14)

    FLAG_NEW = "NEW"
    FLAG_UNCONFIRMED = "?PTR"
    KEY_ROLE = Qt.ItemDataRole.UserRole + 1

    # A host doing short-lived connections (a crawler, a torrent client, a CI
    # runner) produces a new endpoint every few seconds forever. Without a cap
    # the model, the point dict and the five-second rollup pass over all of it
    # grow for as long as the app is open. Oldest-contacted rows go first.
    MAX_ROWS = 5000
    EVICT_CHUNK = 500

    snapshotRequested = QtCore.pyqtSignal()
    csvExportRequested = QtCore.pyqtSignal()
    resolveRequested = QtCore.pyqtSignal(list)
    alertRaised = QtCore.pyqtSignal(str, str)
    watchlistAppendRequested = QtCore.pyqtSignal(str, str, str)
    whoisRequested = QtCore.pyqtSignal(str)
    rulesEditRequested = QtCore.pyqtSignal()
    #: (endpoint, what is unusual about it) — a measurement, not a verdict,
    #: so it goes to the event log rather than the watchlist banner
    anomalyNoticed = QtCore.pyqtSignal(str, str)

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
        self.rules_btn.setToolTip("Re-read the watchlist file from disk")
        self.edit_rules_btn = _hacker_button("✎ EDIT RULES")
        self.edit_rules_btn.setToolTip("Edit the watchlist file, with live validation")
        self.edit_rules_btn.clicked.connect(self.rulesEditRequested.emit)
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
        self.filter_edit.textChanged.connect(self._on_filter_typed)

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
                           (self.C_PROCESS, 110), (self.C_ASN, 150), (self.C_HITS, 48),
                           (self.C_RX, 86), (self.C_TX, 86)):
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
        controls.addWidget(self.edit_rules_btn)
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
        self.process_baseline: Optional[ProcessBaseline] = None
        self.beacons = BeaconDetector()
        self._fcrdns: Dict[str, bool] = {}
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

    def observe_only(self, points: List[dict]):
        """Feed the rhythm detector without touching the table.

        Used while the view is frozen. A beacon is measured from the gaps
        between sightings, so a gap that exists only because somebody held
        the view still would wreck the measurement."""
        for p in points:
            key = f"{p.get('proto', '?')}:{p.get('ip', '?')}:{int(p.get('port', 0))}"
            self.beacons.observe(key, float(p.get("ts", time.time())))

    def _on_filter_typed(self, text: str):
        """Typing in the box always means a plain search over every column;
        only a drill-down narrows it to one."""
        self.proxy.setFilterKeyColumn(-1)
        self.proxy.setFilterFixedString(text)
        self._update_count()

    def filter_to(self, text: str, column: int = -1, whole_word: bool = False):
        """Show only the rows matching `text`, optionally in one column.

        A drill-down from a rollup knows which column it came from, so it can
        be exact where the search box has to guess: filtering the whole table
        for the country code "DE" otherwise matches Delhi and Denver."""
        self.filter_edit.blockSignals(True)
        self.filter_edit.setText(text)
        self.filter_edit.blockSignals(False)
        self.proxy.setFilterKeyColumn(column)
        if whole_word and text:
            self.proxy.setFilterRegularExpression(
                QtCore.QRegularExpression(
                    r"\b" + QtCore.QRegularExpression.escape(text) + r"\b",
                    QtCore.QRegularExpression.PatternOption.CaseInsensitiveOption))
        else:
            self.proxy.setFilterFixedString(text)
        self._update_count()

    def set_process_baseline(self, baseline: Optional["ProcessBaseline"]):
        """Swap the learned per-process profile and re-judge what is on
        screen, so enabling history mid-session is retroactive — the same
        contract set_baseline() already keeps for the NEW flag."""
        self.process_baseline = baseline
        for key, point in self._points.items():
            row = self._rows.get(key)
            if row is None:
                continue
            point["anomaly"] = baseline.judge(point) if baseline else []
            self._apply_flags(row, point)

    def set_fcrdns(self, ip: str, confirmed: bool):
        """Record whether an address's PTR name resolves back to it."""
        self._fcrdns[ip] = bool(confirmed)
        for key, point in self._points.items():
            if point.get("ip") != ip:
                continue
            point["fcrdns"] = bool(confirmed)
            row = self._rows.get(key)
            if row is not None:
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

    @staticmethod
    def _bytes_cell(value) -> Tuple[str, int]:
        """A byte count that sorts numerically and reads as a size.

        Zero shows as an em dash: a column of "0.0 B" is a column of noise,
        and a contact with no byte figure is usually one `ss` cannot see
        (a UDP endpoint, or another user's socket) rather than a silent one."""
        n = int(value or 0)
        return (human_bytes(n) if n else "—"), n

    def set_peer_io(self, totals: Dict[str, Tuple[int, int]]):
        """Fill the RX/TX columns from the kernel's per-socket counters."""
        for key, (rx, tx) in (totals or {}).items():
            point = self._points.get(key)
            row = self._rows.get(key)
            if point is None or row is None:
                continue
            point["rx_bytes"], point["tx_bytes"] = int(rx), int(tx)
            for col, value in ((self.C_RX, rx), (self.C_TX, tx)):
                cell = self.model.item(row, col)
                if cell is None:
                    continue
                text, sort_value = self._bytes_cell(value)
                cell.setText(text)
                cell.setData(sort_value, Qt.ItemDataRole.UserRole)

    def _flags_text(self, point: dict) -> str:
        parts = []
        if point.get("new"):
            parts.append(self.FLAG_NEW)
        parts.extend(f"⚑{lbl}" for lbl in point.get("watch") or [])
        parts.extend(f"⚠{lbl}" for lbl in point.get("anomaly") or [])
        beacon = point.get("beacon")
        if beacon:
            parts.append(f"◷{BeaconDetector.describe(beacon)}")
        if point.get("host") and point.get("fcrdns") is False:
            parts.append(self.FLAG_UNCONFIRMED)
        return " ".join(parts)

    def _row_colour(self, point: dict) -> QtGui.QBrush:
        if point.get("watch"):
            return QtGui.QBrush(RED)
        if point.get("anomaly") or point.get("beacon"):
            return QtGui.QBrush(AMBER)
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
            if ip in self._fcrdns:
                point["fcrdns"] = self._fcrdns[ip]
            if self.resolve_hosts and not host:
                unresolved.append(ip)
            # Every sighting feeds the rhythm detector, repeats included —
            # the gaps between them are the whole signal.
            self.beacons.observe(key, ts)
            point["beacon"] = self.beacons.verdict(key)
            point["anomaly"] = (self.process_baseline.judge(point)
                                if self.process_baseline else [])

            existing = self._points.get(key)
            noticed = list(existing.get("noticed", [])) if existing else []
            if existing is not None:
                point["new"] = existing.get("new", False)
                point["host"] = point["host"] or existing.get("host", "")
                if point.get("fcrdns") is None:
                    point["fcrdns"] = existing.get("fcrdns")
                for field in ("rx_bytes", "tx_bytes"):
                    point[field] = existing.get(field, 0)
            else:
                point["new"] = bool(self.baseline is not None
                                    and ip not in self.baseline
                                    and not is_privateish(ip))
            point["watch"] = self.rules.match(point)

            # Say each thing once: a persistent contact is re-seen every
            # twenty seconds and must not re-announce itself every time.
            fresh = []
            observations = list(point["anomaly"])
            if point["beacon"]:
                observations.append(BeaconDetector.describe(point["beacon"]))
            for label in observations:
                if label not in noticed:
                    noticed.append(label)
                    fresh.append(label)
            point["noticed"] = noticed
            self._points[key] = point
            if fresh:
                self.anomalyNoticed.emit(f"{ip}:{port}", ", ".join(fresh))

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
                self._make_item(*self._bytes_cell(point.get("rx_bytes", 0))),
                self._make_item(*self._bytes_cell(point.get("tx_bytes", 0))),
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
        self._enforce_cap()
        self._update_count()

    def _enforce_cap(self):
        """Drop the least recently contacted rows once the table is over
        MAX_ROWS, in chunks so this does not run on every single scan."""
        overflow = self.model.rowCount() - self.MAX_ROWS
        if overflow < self.EVICT_CHUNK:
            return
        last_seen: Dict[str, float] = {}
        for key, row in self._rows.items():
            item = self.model.item(row, self.C_LAST)
            value = item.data(Qt.ItemDataRole.UserRole) if item is not None else None
            last_seen[key] = float(value or 0.0)
        doomed = sorted(last_seen, key=lambda k: last_seen[k])[:overflow]
        # Descending row order: removing a row shifts every row after it.
        for row in sorted((self._rows[k] for k in doomed if k in self._rows),
                          reverse=True):
            self.model.removeRow(row)
        for key in doomed:
            self._points.pop(key, None)
        # Row numbers below a removed row moved, so the index is rebuilt from
        # the model rather than patched.
        self._rows = {}
        for row in range(self.model.rowCount()):
            item = self.model.item(row, self.C_FIRST)
            if item is not None:
                self._rows[str(item.data(self.KEY_ROLE))] = row

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
                    csv_safe(self.proxy.index(r, c).data())
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
        rules = menu.addMenu("Copy firewall rule")
        rules.setFont(HackerFont.mono(10))
        rules.setToolTip("Puts the rule on the clipboard. Nothing is applied.")
        port = int(point.get("port", 0) or 0)
        proto = str(point.get("proto", "tcp"))
        for syntax in FIREWALL_SYNTAXES:
            text = firewall_rule(syntax, ip, port, proto)
            if not text:
                continue
            rules.addAction(f"{syntax}: {text}",
                            lambda t=text: clip and clip.setText(t))
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
        return ",".join(csv_safe(self.model.index(row, c).data())
                        for c in range(self.model.columnCount()))


class StatsTab(QtWidgets.QWidget):
    """Bandwidth history analytics with configurable time window."""

    TALKERS_BY_COUNT = "TOP TALKERS — ENDPOINTS PER PROCESS"
    TALKERS_BY_BYTES = "TOP TALKERS — TCP BYTES PER PROCESS (RX+TX)"

    snapshotRequested = QtCore.pyqtSignal()
    #: (what kind of value, the value) — a rollup row the user clicked
    drillDown = QtCore.pyqtSignal(str, str)

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

        self.geo_panel = RollupPanel("TOP COUNTRIES — CONTACTS")
        self.asn_panel = RollupPanel("TOP NETWORKS — CONTACTS PER ASN")
        # Real bytes when the kernel will tell us (see ProcTrafficSampler),
        # endpoint counts when it will not. The title always says which.
        self.proc_panel = RollupPanel(self.TALKERS_BY_COUNT)
        self._proc_bytes: List[Tuple[str, int]] = []

        # Reading a name off a chart and retyping it into a filter box is
        # work the chart can do itself.
        for field, panel in (("country", self.geo_panel), ("asn", self.asn_panel),
                             ("process", self.proc_panel)):
            panel.rowActivated.connect(
                lambda value, f=field: self.drillDown.emit(f, value))

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
        if self._proc_bytes:
            self.proc_panel.set_title(self.TALKERS_BY_BYTES)
            self.proc_panel.set_rows(self._proc_bytes, value_fmt=human_bytes)
        else:
            self.proc_panel.set_title(self.TALKERS_BY_COUNT)
            self.proc_panel.set_rows(rollup_counts(points, "process"))

    def set_proc_io(self, totals: Dict[str, Tuple[int, int]], limit: int = 8):
        """Per-process byte totals from the kernel, busiest first."""
        rows = [(proc, rx + tx) for proc, (rx, tx) in (totals or {}).items()
                if rx + tx]
        self._proc_bytes = sorted(rows, key=lambda kv: (-kv[1], kv[0]))[:limit]
        if self._proc_bytes:
            self.proc_panel.set_title(self.TALKERS_BY_BYTES)
            self.proc_panel.set_rows(self._proc_bytes, value_fmt=human_bytes)

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


class HistoryTab(QtWidgets.QWidget):
    """What the history database already knows.

    SessionStore has recorded contacts since the toggle existed and exposed
    recent() to read them back, but nothing ever asked. Everything here is
    already on disk — this tab is the missing window onto it."""

    COLS = ["First Seen", "Last Seen", "Hits", "Proto", "IP", "Host",
            "Port/Svc", "Location", "Country", "ASN/Org", "Process"]
    (C_FIRST, C_LAST, C_HITS, C_PROTO, C_IP, C_HOST,
     C_PORT, C_LOCATION, C_COUNTRY, C_ASN, C_PROCESS) = range(11)

    LIMITS = (200, 500, 2000, 10000)

    snapshotRequested = QtCore.pyqtSignal()
    csvExportRequested = QtCore.pyqtSignal()

    def __init__(self, store: Optional["SessionStore"] = None):
        super().__init__()
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, BG)
        self.setPalette(pal)
        self.store = store

        self.title = QtWidgets.QLabel("RECORDED HISTORY — CONTACTS THAT SURVIVED A RESTART")
        self.title.setFont(HackerFont.mono(14, bold=True))
        self.title.setStyleSheet("color:#00ff66;")

        self.summary = QtWidgets.QLabel("—")
        self.summary.setFont(HackerFont.mono(10))
        self.summary.setStyleSheet("color:#00aa44;")

        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setPlaceholderText("filter: ip / host / country / process ...")
        self.filter_edit.setFont(HackerFont.mono(10))
        self.filter_edit.setStyleSheet(
            "QLineEdit { background:#07100a; color:#00ff66; border:1px solid #0b2a12; padding:4px; }"
        )

        self.limit_combo = QtWidgets.QComboBox()
        self.limit_combo.setFont(HackerFont.mono(9))
        self.limit_combo.addItems([f"LAST {n}" for n in self.LIMITS])
        self.limit_combo.setStyleSheet(
            "QComboBox { background: #07100a; color:#00aa44; border: 1px solid #0b2a12; padding: 2px; }"
            "QAbstractItemView { background: #07100a; color:#00ff66; selection-background-color:#0b2a12; }"
        )

        self.refresh_btn = _hacker_button("↻ REFRESH")
        self.export_btn = _hacker_button("⤓ EXPORT CSV")
        self.forget_btn = _hacker_button("⌫ FORGET ALL")
        self.forget_btn.setToolTip("Delete every recorded contact and bandwidth sample")
        self.snapshot_btn = _hacker_button("◉ SNAPSHOT")

        self.model = QtGui.QStandardItemModel(0, len(self.COLS), self)
        self.model.setHorizontalHeaderLabels(self.COLS)
        self.proxy = ContactSortProxy(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.proxy.setFilterKeyColumn(-1)

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
        for col, width in ((self.C_FIRST, 130), (self.C_LAST, 130), (self.C_HITS, 54),
                           (self.C_PROTO, 52), (self.C_IP, 130), (self.C_HOST, 180),
                           (self.C_PORT, 104), (self.C_LOCATION, 150),
                           (self.C_COUNTRY, 64), (self.C_ASN, 150)):
            self.view.setColumnWidth(col, width)

        self.filter_edit.textChanged.connect(self.proxy.setFilterFixedString)
        self.refresh_btn.clicked.connect(self.refresh)
        self.limit_combo.currentTextChanged.connect(lambda _t: self.refresh())
        self.export_btn.clicked.connect(self.csvExportRequested.emit)
        self.snapshot_btn.clicked.connect(self.snapshotRequested.emit)
        self.forget_btn.clicked.connect(self._confirm_forget)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.title)
        top.addStretch(1)
        top.addWidget(self.limit_combo)
        top.addSpacing(8)
        top.addWidget(self.refresh_btn)
        top.addWidget(self.export_btn)
        top.addWidget(self.forget_btn)
        top.addWidget(self.snapshot_btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addWidget(self.summary)
        layout.addWidget(self.filter_edit)
        layout.addWidget(self.view, 1)

        self.set_store(store)

    def limit(self) -> int:
        idx = self.limit_combo.currentIndex()
        return self.LIMITS[idx] if 0 <= idx < len(self.LIMITS) else self.LIMITS[0]

    def set_store(self, store: Optional["SessionStore"]):
        self.store = store
        self.refresh()

    def _enabled(self) -> bool:
        return self.store is not None and self.store.enabled

    def refresh(self):
        self.model.removeRows(0, self.model.rowCount())
        if not self._enabled():
            self.summary.setText(
                "History recording is OFF — nothing is read and nothing is written. "
                "Turn on ▤ HISTORY on the BLACK ICE tab to start recording.")
            for btn in (self.export_btn, self.forget_btn):
                btn.setEnabled(False)
            return
        for btn in (self.export_btn, self.forget_btn):
            btn.setEnabled(True)
        rows = self.store.recent(self.limit())
        for row in rows:
            self.model.appendRow(self._row_items(row))
        contacts, samples = self.store.counts()
        runs = self.store.sessions_summary()
        watched = ""
        if runs["sessions"]:
            since = time.strftime("%Y-%m-%d", time.localtime(runs["first"]))
            watched = (f" · watched {human_duration(runs['watched'])} across "
                       f"{runs['sessions']} session(s) since {since}")
        self.summary.setText(
            f"{len(rows)} of {contacts} recorded contacts · {samples} bandwidth samples"
            f"{watched} · retention {SessionStore.RETENTION_DAYS} days · {self.store.path}")

    def _row_items(self, row: dict) -> List[QtGui.QStandardItem]:
        port = int(row.get("port", 0) or 0)
        svc = port_service(port)
        first = float(row.get("first_seen", 0.0) or 0.0)
        last = float(row.get("last_seen", 0.0) or 0.0)
        hits = int(row.get("hits", 0) or 0)
        ip = str(row.get("ip", "") or "")
        items = [
            self._item(_stamp(first), first),
            self._item(_stamp(last), last),
            self._item(str(hits), hits),
            self._item(str(row.get("proto", "") or "").upper()),
            self._item(ip, ip_sort_key(ip)),
            self._item(str(row.get("host", "") or "")),
            self._item(f"{port} · {svc}" if svc else str(port), port),
            self._item(str(row.get("location", "") or "")),
            self._item(str(row.get("country", "") or "")),
            self._item(str(row.get("asn", "") or "")),
            self._item(str(row.get("process", "") or "")),
        ]
        brush = QtGui.QBrush(PHOSPHOR_DIM if is_privateish(ip) else PHOSPHOR)
        for it in items:
            it.setForeground(brush)
        return items

    @staticmethod
    def _item(text: str, sort_value=None) -> QtGui.QStandardItem:
        it = QtGui.QStandardItem(text)
        it.setEditable(False)
        if sort_value is not None:
            it.setData(sort_value, Qt.ItemDataRole.UserRole)
        return it

    def _confirm_forget(self):
        """Deleting the user's own record of their own machine, but deleting
        it all the same — so it is asked for out loud."""
        if not self._enabled():
            return
        contacts, samples = self.store.counts()
        answer = QtWidgets.QMessageBox.question(
            self, "Forget recorded history",
            f"Permanently delete {contacts} recorded contact(s) and "
            f"{samples} bandwidth sample(s) from\n{self.store.path}?\n\n"
            "This cannot be undone.",
            QtWidgets.QMessageBox.StandardButton.Yes
            | QtWidgets.QMessageBox.StandardButton.Cancel,
            QtWidgets.QMessageBox.StandardButton.Cancel)
        if answer != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.store.clear()
        self.refresh()

    def export_csv(self, path: str):
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            for r in range(self.proxy.rowCount()):
                w.writerow([csv_safe(self.proxy.index(r, c).data())
                            for c in range(self.proxy.columnCount())])


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
        self.history = HistoryTab(self.store)

        self.tabs.addTab(self.dash, "BLACK ICE")
        self.tabs.addTab(self.conns, "CONTACTS")
        self.tabs.addTab(self.listen, "LISTEN")
        self.tabs.addTab(self.map, "MAP")
        self.tabs.addTab(self.stats, "STATS")
        self.tabs.addTab(self.history, "HISTORY")
        # The database keeps filling while another tab is in front, so the
        # counts this tab last drew go stale the moment you look away.
        self.tabs.currentChanged.connect(self._on_tab_changed)

        self.dash.set_config_enabled(self.cfg.enabled)
        self.dash.configToggled.connect(self._on_config_toggled)
        self.dash.historyToggled.connect(self._on_history_toggled)
        self.dash.hideVirtualToggled.connect(self._on_hide_virtual)
        self.dash.freezeToggled.connect(self._on_freeze)
        self.dash.trayToggled.connect(self._on_tray_toggled)
        self.dash.scanIntervalChanged.connect(self._on_scan_interval)
        self.dash.fxChanged.connect(lambda level: self.cfg.set("dash/fx", level))

        self.dash.snapshotRequested.connect(lambda: self._snapshot(self.dash, "blackice_dashboard"))
        self.conns.snapshotRequested.connect(lambda: self._snapshot(self.conns, "blackice_contacts"))
        self.listen.snapshotRequested.connect(lambda: self._snapshot(self.listen, "blackice_listeners"))
        self.map.snapshotRequested.connect(lambda: self._snapshot(self.map, "blackice_map"))
        self.map.externalLookup.connect(lambda msg: self.dash.set_event(f"[~] {msg}"))
        self.stats.snapshotRequested.connect(lambda: self._snapshot(self.stats, "blackice_stats"))
        self.stats.drillDown.connect(self._on_drill_down)
        self.history.snapshotRequested.connect(lambda: self._snapshot(self.history, "blackice_history"))
        self.history.csvExportRequested.connect(self._export_history)
        self.conns.csvExportRequested.connect(self._export_csv)
        self.listen.csvExportRequested.connect(self._export_listeners)
        self.listen.listenersOpened.connect(self._on_listeners_opened)
        self.listen.listenersClosed.connect(self._on_listeners_closed)
        self.conns.alertRaised.connect(self._on_alert)
        self.conns.watchlistAppendRequested.connect(self.append_watchlist)
        self.conns.whoisRequested.connect(self._open_whois)
        self.conns.rules_btn.clicked.connect(lambda: self.reload_rules(announce=True))
        self.conns.rulesEditRequested.connect(self.edit_watchlist)
        self.conns.anomalyNoticed.connect(self._on_anomaly)

        self.watchlist_path = default_watchlist_path()
        self.reload_rules()

        self.rdns = RDnsResolver()
        self.rdns.resolved.connect(self._on_host_resolved)
        self.rdns.fcrdns.connect(self.conns.set_fcrdns)
        self.rdns.start()
        self.conns.resolveRequested.connect(self.rdns.request)

        self.poller = TrafficPoller(interval=1.0)
        self.poller.traffic.connect(self._on_traffic)
        self.poller.start()

        self.scanner = ConnScanner(interval=3.0, rdns=self.rdns.cache)
        self.scanner.points.connect(self._on_points)
        self.scanner.listeners.connect(self._on_listeners)
        self.scanner.states.connect(self._on_states)
        self.scanner.scanStats.connect(self._on_scan_stats)
        self.scanner.event.connect(self.dash.set_event)
        self.scanner.procIo.connect(self.stats.set_proc_io)
        self.scanner.peerIo.connect(self.conns.set_peer_io)
        self.scanner.start()

        self.dash.set_status("GEOIP", (
            "CITY+ASN" if (_geoip_reader is not None and HAVE_ASN)
            else "CITY" if _geoip_reader is not None else "OFF"))
        if self.scanner.proc_io.enabled:
            self.dash.set_status("PROC BYTES", "ss (TCP)")
            self.dash.set_event(
                "[*] per-process byte accounting online (ss, TCP only)")
        else:
            self.dash.set_status("PROC BYTES", "OFF")
            self.dash.set_event(
                f"[*] per-process bytes unavailable ({self.scanner.proc_io.error})"
                " — top talkers fall back to endpoint counts")
        self.dash.set_status("HISTORY", "ON" if self.store.enabled else "OFF")

        self.health = SocketHealthMonitor()
        self._frozen = False
        self.tray: Optional[QtWidgets.QSystemTrayIcon] = None
        self._quitting = False
        self._tray_hint_shown = False
        self._scope = ""
        self._scope_noted = False
        self._alerts = 0
        self._alerts_muted = 0
        self._alert_seen: Dict[Tuple[str, str], float] = {}
        self._alert_endpoints: set = set()
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

    def edit_watchlist(self) -> bool:
        """Open the rule editor; a save is applied retroactively at once."""
        dialog = WatchlistDialog(self.watchlist_path, self)
        if dialog.exec() != QtWidgets.QDialog.DialogCode.Accepted or not dialog.saved:
            if dialog.error:
                self.dash.set_event(f"[!] could not write watchlist: {dialog.error}")
            return False
        self.dash.set_event(f"[*] watchlist saved → {self.watchlist_path}")
        self.reload_rules(announce=True)
        return True

    def _open_whois(self, ip: str):
        """Explicit, user-initiated, and named as external in the menu: this
        hands the address to a third-party lookup service."""
        if not ip:
            return
        url = QtCore.QUrl(f"https://www.whois.com/whois/{ip}")
        QtGui.QDesktopServices.openUrl(url)
        self.dash.set_event(f"[*] WHOIS lookup opened in browser for {ip}")

    ALERT_REPEAT_SECONDS = 60.0

    #: which contact column each rollup field is actually stored in
    DRILL_COLUMNS = {
        "process": (ConnectionsTab.C_PROCESS, False),
        "asn": (ConnectionsTab.C_ASN, False),
        # There is no Country column; the code is the tail of Location, so it
        # has to match as a whole word or "DE" also finds Delhi.
        "country": (ConnectionsTab.C_LOCATION, True),
    }

    def _on_drill_down(self, field: str, value: str):
        column, whole_word = self.DRILL_COLUMNS.get(field, (-1, False))
        self.conns.filter_to(value, column, whole_word)
        self.tabs.setCurrentWidget(self.conns)
        self.dash.set_event(f"[*] contacts filtered to {field} = {value}")

    def _on_tray_toggled(self, on: bool):
        """Put the app in the tray so closing the window stops the window,
        not the watching."""
        if not on:
            self._teardown_tray()
            self.dash.set_event("[*] tray disabled — closing the window will quit")
            self.cfg.set("window/tray", False)
            return
        if not QtWidgets.QSystemTrayIcon.isSystemTrayAvailable():
            self.dash.set_event("[!] no system tray on this desktop")
            self.dash.set_tray_enabled(False)
            return
        self.tray = QtWidgets.QSystemTrayIcon(tray_icon(), self)
        menu = QtWidgets.QMenu()
        menu.addAction("Show BLACK ICE", self._tray_show)
        freeze = menu.addAction("Freeze view")
        freeze.setCheckable(True)
        freeze.setChecked(self._frozen)
        freeze.toggled.connect(self.dash.freeze_btn.setChecked)
        menu.addSeparator()
        menu.addAction("Quit", self._tray_quit)
        self.tray.setContextMenu(menu)
        self.tray.activated.connect(self._on_tray_activated)
        self.tray.setToolTip(f"{APP_NAME} v{APP_VERSION}")
        self.tray.show()
        # Otherwise hiding the last window ends the process, which is the one
        # thing this toggle exists to prevent.
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.setQuitOnLastWindowClosed(False)
        self.cfg.set("window/tray", True)
        self.dash.set_event("[*] tray enabled — closing the window keeps watching")

    def _teardown_tray(self):
        if self.tray is not None:
            self.tray.hide()
            self.tray.deleteLater()
            self.tray = None
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.setQuitOnLastWindowClosed(True)

    def _tray_show(self):
        self.showNormal()
        self.raise_()
        self.activateWindow()

    def _tray_quit(self):
        self._quitting = True
        self.close()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.quit()

    def _on_tray_activated(self, reason):
        if reason == QtWidgets.QSystemTrayIcon.ActivationReason.Trigger:
            self._tray_show() if self.isHidden() else self.hide()

    def _refresh_tray(self):
        if self.tray is None:
            return
        alert = bool(self._alert_endpoints)
        self.tray.setIcon(tray_icon(alert))
        self.tray.setToolTip(
            f"{APP_NAME} — {self.conns.model.rowCount()} contacts, "
            f"{len(self._alert_endpoints)} flagged")

    def _on_freeze(self, on: bool):
        """Hold the view still.

        What stops is everything that moves under the reader: the tables,
        the waveform, the graphs, the socket bar, the map. What carries on
        is everything that would be corrupted or lost by a gap — recording,
        the rhythm and socket-health measurements — plus the event log and
        the alert banner, because those are the channel telling you why you
        wanted it still in the first place."""
        self._frozen = bool(on)
        self.dash.set_status("VIEW", "FROZEN" if on else "LIVE")
        self.dash.set_event(
            "[*] view FROZEN — scanning and recording continue" if on
            else "[*] view live again")

    def _on_scan_interval(self, secs: float):
        self.scanner.set_interval(secs)
        self.cfg.set("dash/scan_interval", float(secs))
        self.dash.set_event(f"[*] socket scan interval set to {secs:g}s")

    def _on_listeners(self, listeners: list):
        if self._frozen:
            # Not just skipped: the tab keeps the previous inventory as its
            # diff baseline, so whatever opened during the freeze is reported
            # the moment the view comes back.
            return
        self.listen.set_listeners(listeners)

    def _on_tab_changed(self, index: int):
        if self.tabs.widget(index) is self.history:
            self.history.refresh()

    def _on_anomaly(self, endpoint: str, labels: str):
        """Behavioural observations are reported, not alarmed about: a
        software updater on a ten-minute timer is indistinguishable from a
        beacon, and only the person reading this knows which it is."""
        self.dash.set_event(f"[~] {endpoint} — {labels}")

    def _on_states(self, counts: Dict[str, int]):
        # Fed even while frozen: "climbing" is measured across samples, and a
        # hole in the series would hide exactly the trend it looks for.
        for message in self.health.feed(counts):
            self.dash.set_event(f"[!] {message}")
        if not self._frozen:
            self.dash.set_states(counts)

    def _on_scan_stats(self, stats: Dict[str, int]):
        """Keep the status line honest about how much of the socket table
        this process is allowed to attribute."""
        scope = attribution_summary(stats)
        if scope != self._scope:
            self._scope = scope
            self.dash.set_status("SCOPE", scope)
        blank = int(stats.get("unattributed", 0) or 0)
        if blank and not self._scope_noted and not stats.get("elevated"):
            self._scope_noted = True
            self.dash.set_event(
                f"[*] {blank} of {stats.get('remote', 0)} remote sockets have no "
                "owning process — mapping a socket to a pid needs root for other "
                "users' processes, so those Process cells stay blank")

    def _on_listeners_opened(self, opened: List[dict]):
        """A port that was not open a moment ago is the closest thing this
        tool has to an intrusion signal, so it is louder than a contact."""
        for listener in filter(is_notable_listener, opened):
            line = ListenersTab.describe(listener)
            world = listener.get("exposure") == EXPOSURE_WORLD
            self.dash.set_event(f"[{'!' if world else '+'}] PORT OPENED  {line}")
        sensitive = [l for l in opened
                     if l.get("exposure") == EXPOSURE_WORLD and l.get("note")]
        if sensitive:
            self._on_alert(ListenersTab.describe(sensitive[0]),
                           "SENSITIVE PORT NOW REACHABLE FROM ANY INTERFACE")

    def _on_listeners_closed(self, closed: List[dict]):
        for listener in filter(is_notable_listener, closed):
            self.dash.set_event(f"[-] port closed  {ListenersTab.describe(listener)}")

    def _on_alert(self, endpoint: str, labels: str):
        """One watchlist rule matching a chatty CIDR used to repaint the
        banner and write a log line on every single scan, with a counter
        climbing forever. The same endpoint tripping the same rules is the
        same news, so it is only re-reported once a minute; what the banner
        shows is how many *distinct* endpoints have been hit."""
        now = time.time()
        signature = (endpoint, labels)
        last = self._alert_seen.get(signature)
        self._alert_seen[signature] = now
        if last is not None and now - last < self.ALERT_REPEAT_SECONDS:
            self._alerts_muted += 1
            return

        self._alert_endpoints.add(endpoint)
        self._alerts += 1
        self.dash.set_event(f"[!] ALERT {endpoint} — {labels}")
        suffix = f" (+{self._alerts_muted} repeats muted)" if self._alerts_muted else ""
        self.dash.set_alert(
            f"⚑ WATCHLIST: {len(self._alert_endpoints)} endpoint(s) flagged — "
            f"latest {endpoint} — {labels}{suffix}")

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
            self.conns.set_process_baseline(None)
            self.dash.set_status("HISTORY", "OFF")
            self.history.set_store(self.store)
            self.dash.set_event("[*] history recording DISABLED")
            return
        if not self.store.open():
            self.dash.set_event(f"[!] history unavailable: {self.store.error}")
            self.dash.set_history_enabled(False)
            return
        self.dash.set_status("HISTORY", "ON")
        self.history.set_store(self.store)
        self.conns.set_baseline(self.store.baseline)
        profiles = ProcessBaseline(self.store.process_profiles())
        self.conns.set_process_baseline(profiles)
        contacts, samples = self.store.counts()
        learned = sum(1 for name in profiles.profiles if profiles.knows(name))
        self.dash.set_event(
            f"[*] history ENABLED → {self.store.path} "
            f"({contacts} contacts, {samples} samples, {len(self.store.baseline)} known hosts)")
        if learned:
            self.dash.set_event(
                f"[*] behavioural baseline: {learned} process(es) with enough "
                "history to judge — contacts outside their habits are flagged")

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
        self.cfg.set("map/cluster", self.map.cluster_enable.isChecked())
        self.cfg.set("map/mode", self.map.mode())
        self.cfg.set("dash/fx", self.dash.fx_combo.currentText())
        self.cfg.set("dash/hide_virtual", self.dash.hide_virtual.isChecked())
        self.cfg.set("dash/scan_interval", self.dash.scan_interval())
        self.cfg.set("window/tray", self.tray is not None)
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
        # Freeze is deliberately not restored: starting up already frozen
        # would look exactly like a tool that is not working.
        interval = self.cfg.get("dash/scan_interval", 3.0, float)
        self.dash.set_scan_interval(interval)
        self.scanner.set_interval(interval)
        if self.cfg.get("window/tray", False, bool):
            self.dash.tray_btn.setChecked(True)
        self.listen.world_only.setChecked(self.cfg.get("listen/world_only", False, bool))
        listen_hdr = self.cfg.get("listen/header", None, QtCore.QByteArray)
        if listen_hdr:
            self.listen.view.horizontalHeader().restoreState(listen_hdr)
        self.map.set_mode(self.cfg.get("map/mode", "OFFLINE", str))
        self.map.cluster_enable.setChecked(self.cfg.get("map/cluster", True, bool))
        # Recording is a deliberate act, so it is restored the same way the
        # button turns it on — including the "could not open" path.
        self.dash.set_history_enabled(self.cfg.get("history/enabled", False, bool))
        if self.dash.history_btn.isChecked():
            self._on_history_toggled(True)

        split_state = self.cfg.get("map/splitter", None, QtCore.QByteArray)
        if split_state:
            self.map.split.restoreState(split_state)
        # Checking this kicks off a background locate, exactly as a click
        # would — which means a restored session reaches the network before
        # the user has touched anything. Saying so is the least it can do.
        if self.cfg.get("map/show_me", False, bool):
            self.dash.set_event("[*] 'show my location' was saved on — locating now")
            self.map.me_enable.setChecked(True)

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
        if not self._frozen:
            self.stats.set_rollups(self.conns.points())
        self._refresh_tray()

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
        self._accumulate_sample(snap)
        if self._frozen:
            return
        self.dash.update_traffic(snap)
        self.stats.push_traffic(snap)

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
            self.store.touch_session(now)
        self._sample_rx = self._sample_tx = 0.0
        self._sample_n = 0
        self._sample_flushed = now

    def _on_points(self, points: list):
        if self.store.enabled:
            self.store.record_contacts(points)
        if self._frozen:
            self.conns.observe_only(points)
            return
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

    def _export_history(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        default = os.path.join(os.path.expanduser("~"), f"blackice_history_{ts}.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Recorded History", default, "CSV (*.csv)")
        if not path:
            return
        try:
            self.history.export_csv(path)
            self.dash.set_event(f"[*] history CSV exported → {path}")
        except OSError as e:
            self.dash.set_event(f"[!] history export failed: {e}")

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

    @staticmethod
    def _shutdown_thread(thread: QtCore.QThread, stop):
        """Ask a worker to stop, then make sure it is actually gone.

        gethostbyaddr blocks until the resolver answers or gives up, which can
        outlast any wait() worth making the user sit through. A QThread still
        running when Python drops its last reference takes the whole process
        down with it, so an unresponsive worker is terminated rather than left
        to abort the exit."""
        try:
            stop()
            if thread.wait(1500):
                return
            thread.terminate()
            thread.wait(1500)
        except Exception:
            pass

    def closeEvent(self, e):
        if self.tray is not None and not self._quitting:
            # Closing the window is not closing the app while the tray is on.
            e.ignore()
            self.hide()
            if not self._tray_hint_shown:
                self._tray_hint_shown = True
                self.tray.showMessage(
                    APP_NAME, "Still watching. Right-click the tray icon to quit.",
                    QtWidgets.QSystemTrayIcon.MessageIcon.Information, 4000)
            return
        self._teardown_tray()
        try:
            self.save_state()
        except Exception as exc:  # never block shutdown on a config write
            print(f"[!] could not save config: {exc}", file=sys.stderr)
        for thread, stop in ((self.poller, self.poller.stop),
                             (self.scanner, self.scanner.stop),
                             (self.rdns, self.rdns.stop)):
            self._shutdown_thread(thread, stop)
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
            w.writerow([csv_safe(row.get(k, "")) for k in CONTACT_FIELDS])


def write_contacts_json(path: str, rows: List[dict]):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2, sort_keys=True)


#: --watch exit codes. 0/2 keep their usual meanings; the rest say what was
#: seen, so a cron job or a systemd unit can branch on it without parsing
#: anything. An exposure change outranks a watchlist hit: a port that just
#: opened to the world is news about this machine, not about a list.
EXIT_OK = 0
EXIT_ERROR = 2
EXIT_WATCHLIST = 3
EXIT_EXPOSURE = 4


def notify(title: str, body: str) -> bool:
    """Best-effort desktop notification. False when there is nobody to tell."""
    binary = shutil.which("notify-send")
    if not binary:
        return False
    try:
        subprocess.run([binary, "-a", APP_NAME, title, body],
                       timeout=5, check=False, capture_output=True)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def emit_scan_json(pass_no: int, contacts: List[dict], listeners: List[dict],
                   opened: List[dict], closed: List[dict],
                   states: Dict[str, int],
                   proc_bytes: Dict[str, Tuple[int, int]],
                   rules: "RuleSet", stream=None) -> dict:
    """One newline-delimited JSON object per scan.

    NDJSON rather than one big document at the end: the point of a stream is
    that `jq` or a log shipper sees each pass as it happens, and a run that is
    killed at hour six still leaves five hours of valid records behind."""
    now = time.time()
    record = {
        "ts": now,
        "iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)),
        "pass": pass_no,
        "version": APP_VERSION,
        "contacts": contacts,
        "listeners": listeners,
        "listeners_opened": opened,
        "listeners_closed": closed,
        "states": states,
        "proc_bytes": {name: {"rx": rx, "tx": tx}
                       for name, (rx, tx) in sorted(proc_bytes.items())},
        "flagged": [
            {"ip": c.get("ip"), "port": c.get("port"), "proto": c.get("proto"),
             "flags": hits}
            for c in contacts for hits in [rules.match(c)] if hits
        ],
    }
    target = stream if stream is not None else sys.stdout
    target.write(json.dumps(record, default=str) + "\n")
    target.flush()
    return record


def _watch_report(contacts: List[dict], opened: List[dict], rules: "RuleSet",
                  want_notify: bool, out) -> int:
    """Turn one scan into a --watch verdict, announcing anything found."""
    verdict = EXIT_OK
    exposed = [l for l in opened
               if l.get("exposure") == EXPOSURE_WORLD and l.get("note")]
    for listener in exposed:
        line = ListenersTab.describe(listener)
        print(f"  [!] EXPOSURE  {line}", file=out)
        if want_notify:
            notify(f"{APP_NAME}: port exposed", line)
        verdict = max(verdict, EXIT_EXPOSURE)
    for contact in contacts:
        hits = rules.match(contact)
        if not hits:
            continue
        line = f"{contact.get('ip')}:{contact.get('port')} — {', '.join(hits)}"
        print(f"  [!] WATCHLIST  {line}", file=out)
        if want_notify:
            notify(f"{APP_NAME}: watchlist hit", line)
        verdict = max(verdict, EXIT_WATCHLIST)
    return verdict


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
    previous: Optional[Dict[str, dict]] = None

    # In --json mode stdout carries nothing but NDJSON, so the human summary
    # moves to stderr and the stream stays safe to pipe into jq.
    out = sys.stderr if args.json else sys.stdout

    # --watch with no positive duration is the systemd/cron shape: run until
    # something stops it.
    forever = bool(args.watch) and args.duration <= 0
    deadline = None if forever else time.time() + max(0.0, args.duration)
    verdict = EXIT_OK
    passes = 0
    try:
        while True:
            points, listen, st = scanner._scan_psutil()
            if args.resolve:
                for p in points:
                    if not p.host:
                        p.host = RDnsResolver._lookup(p.ip)
            contacts = [p.__dict__ for p in points]
            agg.add(contacts)
            listeners = [l.__dict__ for l in listen]
            states = st
            passes += 1

            current = {listener_key(l): l for l in listeners}
            opened, closed = listener_diff(previous, current)
            previous = current
            proc_bytes = scanner.proc_io.sample()

            if args.json:
                emit_scan_json(passes, contacts, listeners, opened, closed,
                               states, proc_bytes, rules)
            else:
                for listener in filter(is_notable_listener, opened):
                    print(f"  [+] PORT OPENED  {ListenersTab.describe(listener)}", file=out)
                for listener in filter(is_notable_listener, closed):
                    print(f"  [-] port closed  {ListenersTab.describe(listener)}", file=out)

            if args.watch:
                verdict = max(verdict, _watch_report(contacts, opened, rules,
                                                     args.notify, out))

            if deadline is not None and time.time() >= deadline:
                break
            time.sleep(args.interval if forever
                       else min(args.interval, max(0.0, deadline - time.time())))
    except KeyboardInterrupt:
        print("  interrupted", file=out)

    rows = agg.rows()
    world = [l for l in listeners if l.get("exposure") == EXPOSURE_WORLD]
    print(f"{APP_NAME} v{APP_VERSION} headless — {passes} pass(es) over {args.duration:.0f}s", file=out)
    print(f"  contacts  : {len(rows)}", file=out)
    print(f"  listeners : {len(listeners)} ({len(world)} reachable from any interface)", file=out)
    print("  states    : " + (", ".join(f"{k} {v}" for k, v in sorted(states.items())) or "—"), file=out)
    talkers = scanner.proc_io.top(5)
    if talkers:
        print("  talkers   : " + ", ".join(f"{n} {human_bytes(v)}" for n, v in talkers), file=out)
    flagged = [r for r in rows if r.get("flags")]
    for row in flagged:
        print(f"  [!] {row['ip']}:{row['port']} — {row['flags']}", file=out)
    if args.watch:
        print(f"  verdict   : exit {verdict} ("
              + {EXIT_OK: "nothing flagged",
                 EXIT_WATCHLIST: "watchlist hit",
                 EXIT_EXPOSURE: "port exposed to the world"}[verdict] + ")",
              file=out)

    if args.export:
        path = args.export
        try:
            if path.lower().endswith(".json"):
                write_contacts_json(path, rows)
            else:
                write_contacts_csv(path, rows)
        except OSError as e:
            print(f"[!] export failed: {e}", file=sys.stderr)
            return EXIT_ERROR
        print(f"  exported  : {path}", file=out)
    return verdict


def run_selftest() -> int:
    """Check the pieces that have no visible failure mode until they are
    needed — a missing world outline, an unwritable history database, a
    watchlist the parser silently drops."""
    # ok is True/False, or None for "skipped": an optional component that is
    # simply not installed is not a failure, and a self-test that goes red on
    # a clean checkout is a self-test nobody runs.
    results: List[Tuple[str, Optional[bool], str]] = []

    def check(name: str, fn):
        try:
            ok, detail = fn()
        except Exception as e:  # a self-test must report, never traceback
            ok, detail = False, f"{type(e).__name__}: {e}"
        results.append((name, ok if ok is None else bool(ok), detail))

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
        if not HAVE_GEOIP:
            return None, "geoip2 not installed — locations stay empty"
        if _geoip_reader is None:
            return None, "no GeoLite2-City.mmdb found — locations stay empty"
        return True, "city db loaded" + (", asn db loaded" if HAVE_ASN else ", no asn db")

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
        mark = "SKIP" if ok is None else ("OK  " if ok else "FAIL")
        print(f"  [{mark}] {name:<16} {detail}")
    failed = [n for n, ok, _ in results if ok is False]
    skipped = [n for n, ok, _ in results if ok is None]
    ran = len(results) - len(skipped)
    print(f"  {ran - len(failed)}/{ran} checks passed"
          + (f", {len(skipped)} skipped" if skipped else ""))
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
    p.add_argument("--json", action="store_true",
                   help="stream one NDJSON record per scan to stdout "
                        "(implies --headless; the summary goes to stderr)")
    p.add_argument("--watch", action="store_true",
                   help=f"implies --headless; exit {EXIT_WATCHLIST} on a watchlist "
                        f"hit, {EXIT_EXPOSURE} on a newly world-reachable sensitive "
                        "port. With no positive --duration, runs until interrupted")
    p.add_argument("--notify", action="store_true",
                   help="send a desktop notification (notify-send) for anything "
                        "--watch flags")
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

    if args.headless or args.json or args.watch:
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
