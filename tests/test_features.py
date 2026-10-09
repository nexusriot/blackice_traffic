"""Tests for the 0.10.0 feature round: listeners, socket states, reverse DNS,
history, watchlist rules, rollups, the offline map, the CLI and the FX switch.

Run from the project root with the runtime deps installed (PyQt6, psutil):

    QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests
"""
import collections
import csv
import io
import json
import os
import re
import shutil
import socket
import sqlite3
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6 import QtCore, QtWidgets
from PyQt6.QtCore import Qt

import blackice_traffic as bit

_app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])

_counters = collections.namedtuple(
    "counters", "bytes_recv bytes_sent packets_recv packets_sent")


def _listener(ip="0.0.0.0", port=22, proto="tcp", process="sshd", pid=1):
    exposure = bit.exposure_class(ip)
    return bit.ListenPoint(ip=ip, port=port, proto=proto, process=process,
                           pid=pid, exposure=exposure,
                           note=bit.listener_risk(port, exposure)).__dict__


def _contact(ip="203.0.113.9", port=443, proto="tcp", **kw):
    point = {
        "ip": ip, "port": port, "proto": proto, "lat": 1.0, "lon": 2.0,
        "label": f"{ip}:{port} ({proto}) — Somewhere XX", "ts": time.time(),
        "process": "curl", "asn": "AS64500 Example", "state": "ESTABLISHED",
        "country": "XX", "host": "", "location": "Somewhere XX",
    }
    point.update(kw)
    return point


# --------------------------------------------------------------------------
# interfaces, exposure and listener risk
# --------------------------------------------------------------------------

class TestVirtualNic(unittest.TestCase):
    def test_container_and_tunnel_names_are_virtual(self):
        for name in ("docker0", "veth1a2b", "br-abc123", "virbr0", "tun0",
                     "wg0", "vboxnet0", "lxcbr0"):
            self.assertTrue(bit.is_virtual_nic(name), name)

    def test_physical_names_are_not(self):
        for name in ("eth0", "wlan0", "enp3s0", "wlp2s0", "eno1", ""):
            self.assertFalse(bit.is_virtual_nic(name), name)

    def test_loopback_is_not_reported_as_virtual(self):
        # It is already excluded by is_loopback_nic; double-classifying it
        # would make the two filters disagree.
        self.assertFalse(bit.is_virtual_nic("lo"))


class TestExposureClass(unittest.TestCase):
    def test_wildcard_binds_are_world_reachable(self):
        for ip in ("0.0.0.0", "::", "", "*"):
            self.assertEqual(bit.exposure_class(ip), bit.EXPOSURE_WORLD, ip)

    def test_loopback_binds_are_local(self):
        for ip in ("127.0.0.1", "127.0.0.53", "::1", "::ffff:127.0.0.1"):
            self.assertEqual(bit.exposure_class(ip), bit.EXPOSURE_LOCAL, ip)

    def test_private_and_link_local_binds_are_lan(self):
        for ip in ("192.168.1.5", "10.0.0.2", "169.254.1.1", "fe80::1"):
            self.assertEqual(bit.exposure_class(ip), bit.EXPOSURE_LAN, ip)

    def test_public_bind_is_world(self):
        self.assertEqual(bit.exposure_class("8.8.8.8"), bit.EXPOSURE_WORLD)

    def test_garbage_is_unknown(self):
        self.assertEqual(bit.exposure_class("not-an-ip"), bit.EXPOSURE_UNKNOWN)


class TestListenerRisk(unittest.TestCase):
    def test_sensitive_port_on_wildcard_is_flagged(self):
        self.assertTrue(bit.listener_risk(22, bit.EXPOSURE_WORLD))
        self.assertIn("DOCKER", bit.listener_risk(2375, bit.EXPOSURE_WORLD))

    def test_same_port_on_loopback_is_not_flagged(self):
        self.assertEqual(bit.listener_risk(22, bit.EXPOSURE_LOCAL), "")
        self.assertEqual(bit.listener_risk(6379, bit.EXPOSURE_LAN), "")

    def test_ordinary_port_has_no_note(self):
        self.assertEqual(bit.listener_risk(80, bit.EXPOSURE_WORLD), "")


class TestSnapshotHidesVirtual(unittest.TestCase):
    def setUp(self):
        self.prev = {"eth0": _counters(0, 0, 0, 0), "docker0": _counters(0, 0, 0, 0),
                     "lo": _counters(0, 0, 0, 0)}
        self.now = {"eth0": _counters(1000, 500, 4, 2),
                    "docker0": _counters(1000, 500, 4, 2),
                    "lo": _counters(9999, 9999, 9, 9)}

    def test_virtual_counted_by_default(self):
        snap = bit.build_snapshot(self.prev, self.now, 1.0)
        self.assertEqual(snap["_totals"]["rx_bps"], 16000.0)

    def test_virtual_excluded_on_request(self):
        snap = bit.build_snapshot(self.prev, self.now, 1.0, hide_virtual=True)
        self.assertEqual(snap["_totals"]["rx_bps"], 8000.0)
        self.assertEqual(snap["_totals"]["tx_bps"], 4000.0)

    def test_per_nic_rows_survive_and_are_tagged(self):
        snap = bit.build_snapshot(self.prev, self.now, 1.0, hide_virtual=True)
        self.assertIn("docker0", snap)
        self.assertTrue(snap["docker0"]["virtual"])
        self.assertFalse(snap["eth0"]["virtual"])


class TestRollupCounts(unittest.TestCase):
    ROWS = [{"country": "DE"}, {"country": "DE"}, {"country": "US"},
            {"country": ""}, {"country": "AT"}, {"country": "US"},
            {"country": "DE"}]

    def test_orders_by_count_then_name(self):
        self.assertEqual(bit.rollup_counts(self.ROWS, "country"),
                         [("DE", 3), ("US", 2), ("AT", 1)])

    def test_blank_values_are_skipped(self):
        self.assertNotIn("", [k for k, _ in bit.rollup_counts(self.ROWS, "country")])

    def test_limit_applies(self):
        self.assertEqual(len(bit.rollup_counts(self.ROWS, "country", limit=2)), 2)

    def test_missing_field_is_empty(self):
        self.assertEqual(bit.rollup_counts(self.ROWS, "asn"), [])


# --------------------------------------------------------------------------
# watchlist rules
# --------------------------------------------------------------------------

class TestRuleSet(unittest.TestCase):
    TEXT = """
# a comment
cidr 203.0.113.0/24 TEST RANGE
port 3389 RDP
asn AS13335
country RU
process nc
host .example.com
"""

    def setUp(self):
        self.rules = bit.RuleSet.parse(self.TEXT)

    def test_parses_every_kind(self):
        self.assertEqual(len(self.rules), 6)
        self.assertEqual(self.rules.errors, [])

    def test_label_falls_back_to_the_rule_itself(self):
        self.assertEqual(self.rules.rules[0].describe(), "TEST RANGE")
        self.assertEqual(self.rules.rules[2].describe(), "ASN AS13335")

    def test_cidr_match(self):
        self.assertEqual(self.rules.match({"ip": "203.0.113.9"}), ["TEST RANGE"])
        self.assertEqual(self.rules.match({"ip": "198.51.100.1"}), [])

    def test_cidr_does_not_match_across_ip_versions(self):
        rules = bit.RuleSet.parse("cidr ::/0 EVERYTHING-V6")
        self.assertEqual(rules.match({"ip": "2001:db8::1"}), ["EVERYTHING-V6"])
        self.assertEqual(rules.match({"ip": "8.8.8.8"}), [])

    def test_port_match_is_exact(self):
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "port": 3389}), ["RDP"])
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "port": 33890}), [])

    def test_asn_match_is_substring(self):
        self.assertEqual(
            self.rules.match({"ip": "8.8.8.8", "asn": "AS13335 Cloudflare"}),
            ["ASN AS13335"])

    def test_country_match_is_exact_and_case_insensitive(self):
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "country": "ru"}),
                         ["COUNTRY RU"])
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "country": "RUS"}), [])

    def test_process_and_host_match_substrings(self):
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "process": "ncat"}),
                         ["PROCESS nc"])
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "host": "a.example.com"}),
                         ["HOST .example.com"])

    def test_empty_field_never_matches(self):
        self.assertEqual(self.rules.match({"ip": "8.8.8.8", "asn": "", "country": "",
                                           "process": "", "host": ""}), [])

    def test_multiple_hits_are_reported_in_file_order(self):
        hits = self.rules.match({"ip": "203.0.113.9", "port": 3389})
        self.assertEqual(hits, ["TEST RANGE", "RDP"])

    def test_bad_lines_are_reported_and_skipped(self):
        rules = bit.RuleSet.parse("nonsense 1\ncidr not-a-net\nport http\ncidr 10.0.0.0/8 OK")
        self.assertEqual(len(rules), 1)
        self.assertEqual(len(rules.errors), 3)

    def test_shipped_template_is_all_comments(self):
        rules = bit.RuleSet.parse(bit.WATCHLIST_TEMPLATE)
        self.assertEqual(len(rules), 0)
        self.assertEqual(rules.errors, [])

    def test_load_missing_file_is_empty_not_an_error(self):
        rules = bit.RuleSet.load("/nonexistent/blackice/watchlist.txt")
        self.assertEqual(len(rules), 0)
        self.assertEqual(rules.errors, [])

    def test_load_reads_a_file(self):
        tmp = tempfile.mkdtemp()
        try:
            path = os.path.join(tmp, "w.txt")
            with open(path, "w", encoding="utf-8") as f:
                f.write("port 22 SSH\n")
            self.assertEqual(len(bit.RuleSet.load(path)), 1)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------------------------------------------
# reverse DNS
# --------------------------------------------------------------------------

class TestRDnsCache(unittest.TestCase):
    def test_unknown_is_none_and_miss_is_empty_string(self):
        cache = bit.RDnsCache()
        self.assertIsNone(cache.get("8.8.8.8"))
        cache.put("8.8.8.8", "")
        self.assertEqual(cache.get("8.8.8.8"), "")

    def test_hit_round_trip(self):
        cache = bit.RDnsCache()
        cache.put("8.8.8.8", "dns.google")
        self.assertEqual(cache.get("8.8.8.8"), "dns.google")

    def test_entries_expire(self):
        cache = bit.RDnsCache(ttl=0.0)
        cache.put("8.8.8.8", "dns.google")
        self.assertIsNone(cache.get("8.8.8.8"))

    def test_stays_consistent_under_concurrent_use(self):
        # The scanner thread and the GUI read this while the resolver writes
        # it. The GIL makes the unlocked failure window too narrow to
        # reproduce on demand, so this asserts the invariants rather than
        # claiming to catch the race: no exception escapes, and the capacity
        # bound still holds after concurrent eviction.
        cache = bit.RDnsCache(capacity=16)
        errors = []

        def hammer(write):
            try:
                for i in range(4000):
                    ip = f"203.0.113.{i % 64}"
                    if write:
                        cache.put(ip, f"h{i}")
                    else:
                        cache.get(ip)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=hammer, args=(n < 2,)) for n in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(errors, [])
        self.assertLessEqual(len(cache), 16)

    def test_capacity_evicts_least_recently_used(self):
        cache = bit.RDnsCache(capacity=2)
        cache.put("1.1.1.1", "a")
        cache.put("2.2.2.2", "b")
        cache.get("1.1.1.1")          # refresh
        cache.put("3.3.3.3", "c")
        self.assertIsNone(cache.get("2.2.2.2"))
        self.assertEqual(cache.get("1.1.1.1"), "a")


class TestRDnsResolver(unittest.TestCase):
    def setUp(self):
        self.r = bit.RDnsResolver()

    def test_queues_only_unknown_addresses(self):
        self.r.cache.put("1.1.1.1", "one.one.one.one")
        self.r.request(["1.1.1.1", "2.2.2.2"])
        self.assertEqual(list(self.r._queue), ["2.2.2.2"])

    def test_repeat_requests_are_not_queued_twice(self):
        self.r.request(["2.2.2.2"])
        self.r.request(["2.2.2.2"])
        self.assertEqual(list(self.r._queue), ["2.2.2.2"])

    def test_queue_is_bounded(self):
        self.r.request([f"203.0.113.{i}" for i in range(1, 200)]
                       + [f"198.51.100.{i}" for i in range(1, 200)]
                       + [f"192.0.2.{i}" for i in range(1, 200)])
        self.assertLessEqual(len(self.r._queue), self.r.MAX_QUEUE)

    def test_lookup_failure_is_an_empty_name(self):
        with mock.patch.object(bit.socket, "gethostbyaddr",
                               side_effect=OSError("no PTR")):
            self.assertEqual(bit.RDnsResolver._lookup("203.0.113.9"), "")


# --------------------------------------------------------------------------
# history store
# --------------------------------------------------------------------------

class TestSessionStore(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, "sub", "history.db")
        self.store = bit.SessionStore(self.path)

    def tearDown(self):
        self.store.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_nothing_on_disk_until_opened(self):
        self.assertFalse(self.store.enabled)
        self.assertFalse(os.path.exists(self.path))

    def test_open_creates_the_database(self):
        self.assertTrue(self.store.open())
        self.assertTrue(os.path.exists(self.path))
        self.assertTrue(self.store.enabled)

    def test_contacts_upsert_and_count_hits(self):
        self.store.open()
        point = {"ip": "203.0.113.5", "port": 443, "proto": "tcp",
                 "ts": 1000.0, "location": "Somewhere XX", "country": "XX"}
        self.store.record_contacts([point])
        self.store.record_contacts([dict(point, ts=2000.0)])
        rows = self.store.recent()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["hits"], 2)
        self.assertEqual(rows[0]["last_seen"], 2000.0)
        self.assertEqual(rows[0]["first_seen"], 1000.0)

    def test_blank_fields_do_not_overwrite_known_ones(self):
        self.store.open()
        base = {"ip": "203.0.113.5", "port": 443, "proto": "tcp", "ts": 1.0}
        self.store.record_contacts([dict(base, asn="AS64500 Example")])
        self.store.record_contacts([dict(base, asn="")])
        self.assertEqual(self.store.recent()[0]["asn"], "AS64500 Example")

    def test_baseline_is_taken_before_this_session_writes(self):
        self.store.open()
        self.store.record_contacts([{"ip": "203.0.113.5", "port": 443,
                                     "proto": "tcp", "ts": 1.0}])
        self.assertTrue(self.store.is_new("203.0.113.5"))
        self.store.close()

        second = bit.SessionStore(self.path)
        second.open()
        try:
            self.assertFalse(second.is_new("203.0.113.5"))
            self.assertTrue(second.is_new("198.51.100.1"))
        finally:
            second.close()

    def test_samples_and_counts(self):
        self.store.open()
        self.store.record_sample(1.0, 100.0, 50.0)
        self.store.record_sample(2.0, 200.0, 60.0)
        self.assertEqual(self.store.counts(), (0, 2))

    def test_purge_drops_old_rows(self):
        self.store.open()
        old = time.time() - 40 * 86400
        self.store.record_contacts([{"ip": "203.0.113.5", "port": 1, "proto": "tcp",
                                     "ts": old}])
        self.store.record_sample(old, 1.0, 1.0)
        self.store.purge()
        self.assertEqual(self.store.counts(), (0, 0))

    def test_set_host_updates_every_row_for_an_address(self):
        self.store.open()
        for port in (80, 443):
            self.store.record_contacts([{"ip": "203.0.113.5", "port": port,
                                         "proto": "tcp", "ts": 1.0}])
        self.store.set_host("203.0.113.5", "example.test")
        self.assertEqual({r["host"] for r in self.store.recent()}, {"example.test"})

    def test_writes_are_no_ops_while_closed(self):
        self.store.record_contacts([{"ip": "203.0.113.5", "port": 1, "proto": "tcp"}])
        self.store.record_sample(1.0, 1.0, 1.0)
        self.assertEqual(self.store.counts(), (0, 0))
        self.assertFalse(os.path.exists(self.path))

    def test_unopenable_path_reports_instead_of_raising(self):
        store = bit.SessionStore(os.path.join(self.tmp, "file.txt", "history.db"))
        with open(os.path.join(self.tmp, "file.txt"), "w", encoding="utf-8") as f:
            f.write("not a directory")
        self.assertFalse(store.open())
        self.assertTrue(store.error)


# --------------------------------------------------------------------------
# world outline
# --------------------------------------------------------------------------

class TestWorldOutline(unittest.TestCase):
    def test_bundled_asset_parses(self):
        outline = bit.WorldOutline.load()
        self.assertEqual(len(outline), 119)
        self.assertEqual(outline.point_count, 2505)

    def test_coordinates_are_in_range(self):
        outline = bit.WorldOutline.load()
        for lon, lat in outline.rings[0]:
            self.assertGreaterEqual(lon, -180.0)
            self.assertLessEqual(lon, 180.0)
            self.assertGreaterEqual(lat, -90.0)
            self.assertLessEqual(lat, 90.0)

    def test_round_trip_of_a_hand_built_outline(self):
        data = b"BIWM" + struct.pack(">H", 1) + struct.pack(">H", 2) \
            + struct.pack(">hhhh", -1000, 4550, 1000, -4550)
        outline = bit.WorldOutline.parse(data)
        self.assertEqual(outline.rings, [[(-10.0, 45.5), (10.0, -45.5)]])

    def test_wrong_magic_is_rejected(self):
        with self.assertRaises(ValueError):
            bit.WorldOutline.parse(b"NOPE" + struct.pack(">H", 0))

    def test_missing_file_yields_an_empty_outline(self):
        outline = bit.WorldOutline.load("/nonexistent/world.bin")
        self.assertEqual(len(outline), 0)


class TestWorldMapWidget(unittest.TestCase):
    def setUp(self):
        self.w = bit.WorldMapWidget(bit.WorldOutline.load())
        self.w.resize(360, 180)

    def test_projection_places_the_prime_meridian_and_equator(self):
        x, y = self.w.project(0.0, 0.0)
        self.assertAlmostEqual(x, 180.0)
        self.assertAlmostEqual(y, 90.0)

    def test_projection_corners(self):
        self.assertAlmostEqual(self.w.project(-180.0, 90.0)[0], 0.0)
        self.assertAlmostEqual(self.w.project(-180.0, 90.0)[1], 0.0)
        self.assertAlmostEqual(self.w.project(180.0, -90.0)[0], 360.0)

    def test_zoom_is_clamped(self):
        self.w.set_zoom(9999.0)
        self.assertEqual(self.w.zoom, self.w.MAX_ZOOM)
        self.w.set_zoom(0.01)
        self.assertEqual(self.w.zoom, self.w.MIN_ZOOM)

    def test_pan_cannot_strand_the_viewport_off_the_map(self):
        self.w.set_zoom(2.0)
        self.w.pan_x = 5000.0
        self.w.pan_y = -5000.0
        self.w.clamp_pan()
        self.assertLessEqual(self.w.pan_x, 0.0)
        self.assertGreaterEqual(self.w.pan_y, self.w.height() - self.w.height() * 2.0)

    def test_focus_centres_the_requested_position(self):
        self.w.set_zoom(1.0)
        self.w.focus_on(0.0, 0.0, 1.0)
        x, y = self.w.project(0.0, 0.0)
        self.assertAlmostEqual(x, self.w.width() / 2.0)
        self.assertAlmostEqual(y, self.w.height() / 2.0)

    def test_points_are_deduped_and_null_island_is_dropped(self):
        self.w.set_points([_contact(lat=1.0, lon=2.0), _contact(lat=3.0, lon=4.0)])
        self.assertEqual(self.w.point_count(), 1)
        self.w.set_points([_contact(ip="198.51.100.1", lat=0.0, lon=0.0)])
        self.assertEqual(self.w.point_count(), 1)

    def test_hover_finds_the_nearest_contact(self):
        self.w.set_points([_contact(lat=0.0, lon=0.0, ip="203.0.113.1")])
        # lat/lon 0,0 is dropped, so use a real position
        self.w.clear_points()
        self.w.set_points([_contact(lat=0.0, lon=90.0)])
        x, y = self.w.project(90.0, 0.0)
        self.assertIsNotNone(self.w.point_at(QtCore.QPoint(int(x), int(y))))
        self.assertIsNone(self.w.point_at(QtCore.QPoint(int(x) + 60, int(y))))

    def test_reset_view(self):
        self.w.set_zoom(4.0)
        self.w.pan_x = -100.0
        self.w.reset_view()
        self.assertEqual((self.w.zoom, self.w.pan_x, self.w.pan_y), (1.0, 0.0, 0.0))

    def test_paints_without_an_outline(self):
        widget = bit.WorldMapWidget(bit.WorldOutline())
        widget.resize(200, 100)
        widget.set_points([_contact()])
        widget.grab()  # must not raise


# --------------------------------------------------------------------------
# headless mode
# --------------------------------------------------------------------------

class TestContactAggregator(unittest.TestCase):
    def test_dedupes_and_counts_hits(self):
        agg = bit.ContactAggregator()
        agg.add([_contact(ts=1.0), _contact(ts=2.0)])
        rows = agg.rows()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["hits"], 2)
        self.assertEqual(rows[0]["first_seen"], 1.0)
        self.assertEqual(rows[0]["last_seen"], 2.0)

    def test_service_name_is_resolved(self):
        agg = bit.ContactAggregator()
        agg.add([_contact(port=22)])
        self.assertEqual(agg.rows()[0]["service"], "SSH")

    def test_later_scans_fill_in_blanks(self):
        agg = bit.ContactAggregator()
        agg.add([_contact(process="")])
        agg.add([_contact(process="firefox")])
        self.assertEqual(agg.rows()[0]["process"], "firefox")

    def test_rules_flag_rows(self):
        agg = bit.ContactAggregator(bit.RuleSet.parse("cidr 203.0.113.0/24 TEST"))
        agg.add([_contact()])
        self.assertEqual(agg.rows()[0]["flags"], "TEST")

    def test_rows_sort_by_hits(self):
        agg = bit.ContactAggregator()
        agg.add([_contact(ip="203.0.113.1"), _contact(ip="203.0.113.2")])
        agg.add([_contact(ip="203.0.113.2")])
        self.assertEqual(agg.rows()[0]["ip"], "203.0.113.2")


class TestHeadlessWriters(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        agg = bit.ContactAggregator()
        agg.add([_contact(ip="203.0.113.1"), _contact(ip="203.0.113.2", port=22)])
        self.rows = agg.rows()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_csv_has_the_declared_header(self):
        path = os.path.join(self.tmp, "out.csv")
        bit.write_contacts_csv(path, self.rows)
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], bit.CONTACT_FIELDS)
        self.assertEqual(len(rows), 3)

    def test_json_round_trips(self):
        path = os.path.join(self.tmp, "out.json")
        bit.write_contacts_json(path, self.rows)
        with open(path, encoding="utf-8") as f:
            loaded = json.load(f)
        self.assertEqual(len(loaded), 2)
        self.assertEqual({r["ip"] for r in loaded},
                         {"203.0.113.1", "203.0.113.2"})


class TestCli(unittest.TestCase):
    def test_defaults(self):
        args = bit.build_arg_parser().parse_args([])
        self.assertFalse(args.headless)
        self.assertFalse(args.selftest)
        self.assertEqual(args.interval, 3.0)

    def test_headless_flags(self):
        args = bit.build_arg_parser().parse_args(
            ["--headless", "--duration", "1", "--export", "x.json", "--resolve"])
        self.assertTrue(args.headless)
        self.assertEqual(args.duration, 1.0)
        self.assertEqual(args.export, "x.json")
        self.assertTrue(args.resolve)

    def test_version_exits(self):
        with mock.patch("sys.stdout"), self.assertRaises(SystemExit):
            bit.build_arg_parser().parse_args(["--version"])

    def test_selftest_passes(self):
        with mock.patch("sys.stdout"):
            self.assertEqual(bit.run_selftest(), 0)

    def test_headless_run_exports(self):
        tmp = tempfile.mkdtemp()
        try:
            path = os.path.join(tmp, "out.csv")
            args = bit.build_arg_parser().parse_args(
                ["--headless", "--duration", "0", "--export", path,
                 "--watchlist", os.path.join(tmp, "none.txt")])
            with mock.patch("sys.stdout"):
                self.assertEqual(bit.run_headless(args), 0)
            self.assertTrue(os.path.exists(path))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------------------------------------------
# scanner
# --------------------------------------------------------------------------

class TestScannerListenersAndStates(unittest.TestCase):
    Addr = collections.namedtuple("Addr", "ip port")
    Conn = collections.namedtuple("Conn", "fd family type laddr raddr status pid")

    def _scan(self, conns):
        scanner = bit.ConnScanner()
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns):
            return scanner._scan_psutil()

    def test_listening_sockets_are_reported_not_dropped(self):
        conns = [self.Conn(1, 2, 1, self.Addr("0.0.0.0", 22), (), "LISTEN", None)]
        _contacts, listening, states = self._scan(conns)
        self.assertEqual(len(listening), 1)
        self.assertEqual(listening[0].port, 22)
        self.assertEqual(listening[0].exposure, bit.EXPOSURE_WORLD)
        self.assertTrue(listening[0].note)
        self.assertEqual(states, {"LISTEN": 1})

    def test_loopback_listener_is_reported_as_local(self):
        conns = [self.Conn(1, 2, 1, self.Addr("127.0.0.1", 6379), (), "LISTEN", None)]
        _c, listening, _s = self._scan(conns)
        self.assertEqual(listening[0].exposure, bit.EXPOSURE_LOCAL)
        self.assertEqual(listening[0].note, "")

    def test_connection_state_reaches_the_contact(self):
        conns = [self.Conn(1, 2, 1, self.Addr("0.0.0.0", 0),
                           self.Addr("203.0.113.9", 443), "ESTABLISHED", None)]
        contacts, _l, states = self._scan(conns)
        self.assertEqual(contacts[0].state, "ESTABLISHED")
        self.assertEqual(states, {"ESTABLISHED": 1})

    def test_udp_none_state_is_not_counted(self):
        conns = [self.Conn(1, 2, 2, self.Addr("0.0.0.0", 0),
                           self.Addr("203.0.113.9", 53), "NONE", None)]
        _c, _l, states = self._scan(conns)
        self.assertEqual(states, {})

    def test_cached_hostname_is_stamped_on_a_contact(self):
        cache = bit.RDnsCache()
        cache.put("203.0.113.9", "host.example")
        scanner = bit.ConnScanner(rdns=cache)
        conns = [self.Conn(1, 2, 1, self.Addr("0.0.0.0", 0),
                           self.Addr("203.0.113.9", 443), "ESTABLISHED", None)]
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns):
            contacts, _l, _s = scanner._scan_psutil()
        self.assertEqual(contacts[0].host, "host.example")

    def test_geo_lookup_returns_a_country_code(self):
        reader = mock.Mock()
        city = reader.city.return_value
        city.location.latitude = 52.5
        city.location.longitude = 13.4
        city.city.name = "Berlin"
        city.country.iso_code = "DE"
        city.subdivisions = []
        with mock.patch.object(bit, "HAVE_GEOIP", True), \
             mock.patch.object(bit, "_geoip_reader", reader):
            lat, lon, label, country = bit.ConnScanner()._geo_lookup("8.8.8.8")
        self.assertEqual((lat, lon, country), (52.5, 13.4, "DE"))
        self.assertIn("Berlin", label)

    def test_private_addresses_have_no_country(self):
        self.assertEqual(bit.ConnScanner()._geo_lookup("192.168.1.5")[3], "")


# --------------------------------------------------------------------------
# widgets
# --------------------------------------------------------------------------

class TestStateBar(unittest.TestCase):
    def test_known_states_come_in_protocol_order(self):
        bar = bit.StateBar()
        bar.set_counts({"TIME_WAIT": 3, "ESTABLISHED": 9, "SYN_SENT": 1})
        self.assertEqual([s for s, _ in bar.ordered()],
                         ["ESTABLISHED", "SYN_SENT", "TIME_WAIT"])

    def test_unknown_states_are_appended_not_dropped(self):
        bar = bit.StateBar()
        bar.set_counts({"ESTABLISHED": 1, "BOUND": 2})
        self.assertEqual([s for s, _ in bar.ordered()], ["ESTABLISHED", "BOUND"])

    def test_zero_counts_are_hidden(self):
        bar = bit.StateBar()
        bar.set_counts({"ESTABLISHED": 0, "LISTEN": 4})
        self.assertEqual(bar.ordered(), [("LISTEN", 4)])

    def test_paints_when_empty(self):
        bar = bit.StateBar()
        bar.resize(200, 46)
        bar.grab()

    def test_close_wait_is_red_and_established_is_phosphor(self):
        self.assertEqual(bit.state_color("CLOSE_WAIT"), bit.RED)
        self.assertEqual(bit.state_color("ESTABLISHED"), bit.PHOSPHOR)


class TestRollupPanel(unittest.TestCase):
    def test_paints_with_and_without_rows(self):
        panel = bit.RollupPanel("TOP")
        panel.resize(240, 140)
        panel.grab()
        panel.set_rows([("DE", 4), ("US", 2)])
        panel.grab()


class TestListenersTab(unittest.TestCase):
    def setUp(self):
        self.tab = bit.ListenersTab()

    def test_rows_are_populated(self):
        self.tab.set_listeners([_listener(port=22), _listener("127.0.0.1", 6379)])
        self.assertEqual(self.tab.model.rowCount(), 2)

    def test_summary_counts_world_reachable(self):
        self.tab.set_listeners([_listener(port=22), _listener("127.0.0.1", 6379)])
        self.assertIn("2 listening", self.tab.summary.text())
        self.assertIn("1 reachable", self.tab.summary.text())
        self.assertIn("1 sensitive", self.tab.summary.text())

    def test_table_is_rebuilt_when_the_set_changes(self):
        self.tab.set_listeners([_listener(port=22)])
        self.tab.set_listeners([_listener(port=80, process="nginx")])
        self.assertEqual(self.tab.model.rowCount(), 1)
        self.assertIn("80", self.tab.model.item(0, self.tab.C_PORT).text())

    def test_unchanged_set_does_not_rebuild(self):
        self.tab.set_listeners([_listener(port=22)])
        first = self.tab.model.item(0, self.tab.C_PROCESS)
        self.tab.set_listeners([_listener(port=22)])
        self.assertIs(self.tab.model.item(0, self.tab.C_PROCESS), first)

    def test_world_only_filter(self):
        self.tab.set_listeners([_listener(port=22), _listener("127.0.0.1", 6379)])
        self.assertEqual(self.tab.proxy.rowCount(), 2)
        self.tab.world_only.setChecked(True)
        self.assertEqual(self.tab.proxy.rowCount(), 1)
        self.assertEqual(self.tab.proxy.index(0, self.tab.C_EXPOSURE).data(),
                         bit.EXPOSURE_WORLD)

    def test_export_csv(self):
        tmp = tempfile.mkdtemp()
        try:
            self.tab.set_listeners([_listener(port=22)])
            path = os.path.join(tmp, "l.csv")
            self.tab.export_csv(path)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], bit.ListenersTab.COLS)
            self.assertEqual(len(rows), 2)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_export_button_is_wired(self):
        seen = []
        self.tab.csvExportRequested.connect(lambda: seen.append(1))
        self.tab.export_btn.click()
        self.assertEqual(seen, [1])

    def test_export_honors_the_world_only_filter(self):
        tmp = tempfile.mkdtemp()
        try:
            self.tab.set_listeners([_listener(port=22),
                                    _listener("127.0.0.1", 6379)])
            self.tab.world_only.setChecked(True)
            path = os.path.join(tmp, "l.csv")
            self.tab.export_csv(path)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(len(rows), 2)  # header + the one visible socket
            self.assertEqual(rows[1][bit.ListenersTab.C_EXPOSURE],
                             bit.EXPOSURE_WORLD)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_ports_sort_numerically(self):
        self.tab.set_listeners([_listener(port=8080), _listener(port=80),
                                _listener(port=443)])
        self.tab.proxy.sort(self.tab.C_PORT, Qt.SortOrder.AscendingOrder)
        shown = [self.tab.proxy.index(r, self.tab.C_PORT).data()
                 for r in range(self.tab.proxy.rowCount())]
        self.assertEqual(shown[0].split(" ")[0], "80")
        self.assertEqual(shown[-1].split(" ")[0], "8080")


class TestContactsNewColumns(unittest.TestCase):
    def setUp(self):
        self.tab = bit.ConnectionsTab()

    def _cell(self, col, row=0):
        return self.tab.model.item(row, col).text()

    def test_state_and_host_columns(self):
        self.tab.add_points([_contact(state="SYN_SENT", host="a.example")])
        self.assertEqual(self._cell(self.tab.C_STATE), "SYN_SENT")
        self.assertEqual(self._cell(self.tab.C_HOST), "a.example")

    def test_host_arrives_later_and_fills_every_row_for_that_ip(self):
        self.tab.add_points([_contact(port=80), _contact(port=443)])
        self.tab.set_host("203.0.113.9", "late.example")
        self.assertEqual(self._cell(self.tab.C_HOST, 0), "late.example")
        self.assertEqual(self._cell(self.tab.C_HOST, 1), "late.example")

    def test_unresolved_addresses_are_requested_once_per_batch(self):
        seen = []
        self.tab.resolveRequested.connect(seen.append)
        self.tab.add_points([_contact(port=80), _contact(port=443)])
        self.assertEqual(seen, [["203.0.113.9"]])

    def test_known_host_is_not_requested(self):
        seen = []
        self.tab.resolveRequested.connect(seen.append)
        self.tab.add_points([_contact(host="known.example")])
        self.assertEqual(seen, [])

    def test_new_flag_needs_a_baseline(self):
        self.tab.add_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")
        self.tab._clear()
        self.tab.set_baseline({"198.51.100.1"})
        self.tab.add_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self._cell(self.tab.C_FLAGS), "NEW")

    def test_baseline_arriving_late_re_judges_existing_rows(self):
        self.tab.add_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")
        self.tab.set_baseline({"1.1.1.1"})
        self.assertEqual(self._cell(self.tab.C_FLAGS), "NEW")
        self.tab.set_baseline(None)
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")

    def test_known_address_is_not_new(self):
        self.tab.set_baseline({"8.8.8.8"})
        self.tab.add_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")

    def test_private_addresses_are_never_new(self):
        self.tab.set_baseline(set())
        self.tab.add_points([_contact(ip="192.168.1.5")])
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")

    def test_watchlist_hit_flags_the_row_and_raises_an_alert(self):
        alerts = []
        self.tab.alertRaised.connect(lambda ep, labels: alerts.append((ep, labels)))
        self.tab.set_rules(bit.RuleSet.parse("cidr 203.0.113.0/24 TEST"))
        self.tab.add_points([_contact()])
        self.assertIn("⚑TEST", self._cell(self.tab.C_FLAGS))
        self.assertEqual(alerts, [("203.0.113.9:443", "TEST")])

    def test_late_hostname_can_raise_an_alert(self):
        alerts = []
        self.tab.alertRaised.connect(lambda ep, labels: alerts.append(ep))
        self.tab.set_rules(bit.RuleSet.parse("host .evil.example WATCHED HOST"))
        self.tab.add_points([_contact()])
        self.assertEqual(alerts, [])
        self.tab.set_host("203.0.113.9", "node.evil.example")
        self.assertEqual(alerts, ["203.0.113.9:443"])
        self.assertIn("⚑WATCHED HOST", self._cell(self.tab.C_FLAGS))

    def test_late_hostname_does_not_re_alert(self):
        alerts = []
        self.tab.set_rules(bit.RuleSet.parse("cidr 203.0.113.0/24 TEST"))
        self.tab.add_points([_contact()])
        self.tab.alertRaised.connect(lambda ep, labels: alerts.append(ep))
        self.tab.set_host("203.0.113.9", "host.example")
        self.assertEqual(alerts, [])

    def test_reloading_rules_reflags_existing_rows(self):
        self.tab.add_points([_contact()])
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")
        self.tab.set_rules(bit.RuleSet.parse("port 443 HTTPS-WATCH"))
        self.assertIn("HTTPS-WATCH", self._cell(self.tab.C_FLAGS))
        self.tab.set_rules(bit.RuleSet())
        self.assertEqual(self._cell(self.tab.C_FLAGS), "")

    def test_flagged_rows_are_red(self):
        self.tab.set_rules(bit.RuleSet.parse("port 443 X"))
        self.tab.add_points([_contact()])
        self.assertEqual(self.tab.model.item(0, self.tab.C_IP).foreground().color(),
                         bit.RED)

    def test_rollup_over_the_live_table(self):
        self.tab.add_points([_contact(ip="203.0.113.1", country="DE"),
                             _contact(ip="203.0.113.2", country="DE"),
                             _contact(ip="203.0.113.3", country="US")])
        self.assertEqual(self.tab.rollup("country"), [("DE", 2), ("US", 1)])

    def test_visible_points_follow_the_filter(self):
        self.tab.add_points([_contact(ip="203.0.113.1"), _contact(ip="198.51.100.2")])
        self.assertEqual(len(self.tab.visible_points()), 2)
        self.tab.filter_edit.setText("198.51.100.2")
        self.assertEqual([p["ip"] for p in self.tab.visible_points()],
                         ["198.51.100.2"])

    def test_json_export_carries_raw_values(self):
        tmp = tempfile.mkdtemp()
        try:
            self.tab.add_points([_contact()])
            path = os.path.join(tmp, "c.json")
            self.tab.export_json(path)
            with open(path, encoding="utf-8") as f:
                rows = json.load(f)
            self.assertEqual(rows[0]["country"], "XX")
            self.assertEqual(rows[0]["lat"], 1.0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_csv_export_header_matches_columns(self):
        tmp = tempfile.mkdtemp()
        try:
            self.tab.add_points([_contact()])
            path = os.path.join(tmp, "c.csv")
            self.tab.export_csv(path)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], bit.ConnectionsTab.COLS)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestContactsContextMenu(unittest.TestCase):
    def setUp(self):
        self.tab = bit.ConnectionsTab()
        self.tab.add_points([_contact()])
        self.point = self.tab.points()[0]

    def _labels(self):
        return [a.text() for a in self.tab.build_menu(self.point).actions()]

    def test_menu_offers_copy_filter_and_watchlist(self):
        labels = " | ".join(self._labels())
        self.assertIn("Copy 203.0.113.9", labels)
        self.assertIn("Filter to 203.0.113.9", labels)
        self.assertIn("watchlist", labels)
        self.assertIn("WHOIS", labels)

    def test_filter_action_sets_the_filter(self):
        menu = self.tab.build_menu(self.point)
        next(a for a in menu.actions()
             if a.text() == "Filter to 203.0.113.9").trigger()
        self.assertEqual(self.tab.filter_edit.text(), "203.0.113.9")

    def test_watchlist_action_emits_a_host_route(self):
        seen = []
        self.tab.watchlistAppendRequested.connect(
            lambda k, v, l: seen.append((k, v, l)))
        menu = self.tab.build_menu(self.point)
        next(a for a in menu.actions() if "to watchlist" in a.text()).trigger()
        self.assertEqual(seen, [("cidr", "203.0.113.9/32", "")])

    def test_whois_action_emits_the_address(self):
        seen = []
        self.tab.whoisRequested.connect(seen.append)
        menu = self.tab.build_menu(self.point)
        next(a for a in menu.actions() if "WHOIS" in a.text()).trigger()
        self.assertEqual(seen, ["203.0.113.9"])

    def test_host_cidr_covers_both_families(self):
        self.assertEqual(bit.ConnectionsTab._host_cidr("8.8.8.8"), "8.8.8.8/32")
        self.assertEqual(bit.ConnectionsTab._host_cidr("2001:db8::1"),
                         "2001:db8::1/128")

    def test_row_at_an_invalid_position_is_none(self):
        self.assertIsNone(self.tab.row_at(QtCore.QPoint(9999, 9999)))


class TestDashboardExtras(unittest.TestCase):
    def setUp(self):
        self.d = bit.BlackIceDashboard()

    def test_fx_off_hides_the_overlays_and_stops_them(self):
        self.d.set_fx_level("OFF")
        self.assertEqual(self.d.matrix.gain, 0.0)
        self.assertFalse(self.d.matrix.isVisible())
        self.d.matrix.set_running(True)
        self.assertFalse(self.d.matrix._timer.isActive())

    def test_fx_subtle_dims_without_hiding(self):
        self.d.set_fx_level("SUBTLE")
        self.assertGreater(self.d.matrix.gain, 0.0)
        self.assertLess(self.d.matrix.gain, 1.0)

    def test_unknown_fx_level_falls_back_to_full(self):
        self.d.matrix.set_intensity("NONSENSE")
        self.assertEqual(self.d.matrix.gain, 1.0)

    def test_setting_the_level_does_not_re_emit(self):
        seen = []
        self.d.fxChanged.connect(seen.append)
        self.d.set_fx_level("SUBTLE")
        self.assertEqual(seen, [])
        self.d.fx_combo.setCurrentText("OFF")
        self.assertEqual(seen, ["OFF"])

    def test_meter_labels_reserve_room_for_the_widest_reading(self):
        # Growing the text must never clip the unit off the end.
        fm = bit.QtGui.QFontMetrics(self.d.rx_lbl.font())
        for text in ("RX: 0 b/s", "RX: 100.0 Kb/s", "TX: 999.9 Mb/s"):
            self.assertLessEqual(fm.horizontalAdvance(text),
                                 self.d.rx_lbl.minimumWidth(), text)

    def test_meters_do_not_move_when_the_reading_grows(self):
        self.d.resize(900, 600)
        self.d.show()
        try:
            snap = {"_totals": {"rx_bps": 1.0, "tx_bps": 1.0}}
            self.d.iface.setCurrentText("ALL")
            self.d.update_traffic(snap)
            narrow = self.d.tx_lbl.pos().x()
            self.d.update_traffic({"_totals": {"rx_bps": 9.9e6, "tx_bps": 1.0}})
            self.assertEqual(self.d.tx_lbl.pos().x(), narrow)
        finally:
            self.d.hide()

    def test_alert_banner_shows_and_clears(self):
        self.assertFalse(self.d.alert_lbl.isVisible())
        self.d.set_alert("⚑ WATCHLIST HIT")
        self.assertEqual(self.d.alert_lbl.text(), "⚑ WATCHLIST HIT")
        self.d.set_alert("")
        self.assertFalse(self.d.alert_lbl.isVisible())

    def test_state_histogram_is_forwarded(self):
        self.d.set_states({"ESTABLISHED": 3})
        self.assertEqual(self.d.states.ordered(), [("ESTABLISHED", 3)])

    def test_hide_virtual_excludes_docker_bytes_from_totals(self):
        snap = {
            "eth0": {"rx_bps": 1.0, "tx_bps": 1.0, "rx_total": 100, "tx_total": 100,
                     "pkts_in": 1, "pkts_out": 1},
            "docker0": {"rx_bps": 1.0, "tx_bps": 1.0, "rx_total": 900, "tx_total": 900,
                        "pkts_in": 1, "pkts_out": 1},
            "_totals": {"rx_bps": 2.0, "tx_bps": 2.0},
        }
        self.d.iface.setCurrentText("ALL")
        self.d.update_traffic(snap)
        self.assertIn("1,000.0 B", self.d.totals_lbl.text())
        self.d.hide_virtual.setChecked(True)
        self.d.update_traffic(snap)
        self.assertIn("100.0 B", self.d.totals_lbl.text())

    def test_overlays_are_raised_above_the_content(self):
        # Constructed before the layout widgets, so without an explicit raise
        # the CRT effect is only visible in the margins.
        self.assertLess(self.d.children().index(self.d.matrix),
                        self.d.children().index(self.d.log))
        self.d.show()
        try:
            children = self.d.children()
            self.assertGreater(children.index(self.d.matrix),
                               children.index(self.d.log))
            self.assertGreater(children.index(self.d.scan),
                               children.index(self.d.log))
        finally:
            self.d.hide()


class TestStatsRollups(unittest.TestCase):
    def test_panels_are_filled_from_contacts(self):
        stats = bit.StatsTab()
        stats.set_rollups([_contact(country="DE", asn="AS1 A", process="curl"),
                           _contact(country="DE", asn="AS2 B", process="curl"),
                           _contact(country="US", asn="AS1 A", process="ssh")])
        self.assertEqual(stats.geo_panel._rows, [("DE", 2), ("US", 1)])
        self.assertEqual(stats.asn_panel._rows[0], ("AS1 A", 2))
        self.assertEqual(stats.proc_panel._rows[0], ("curl", 2))


class TestMapModes(unittest.TestCase):
    def setUp(self):
        self.tab = bit.MapTab()

    def tearDown(self):
        self.tab.deleteLater()
        _app.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)

    def test_offline_is_the_default(self):
        self.assertEqual(self.tab.mode(), "OFFLINE")
        self.assertIs(self.tab.stack.currentWidget(), self.tab.world)

    def test_switching_modes(self):
        self.tab.set_mode("ONLINE")
        self.assertIs(self.tab.stack.currentWidget(), self.tab.web)
        self.tab.set_mode("OFFLINE")
        self.assertIs(self.tab.stack.currentWidget(), self.tab.world)

    def test_unknown_mode_is_ignored(self):
        self.tab.set_mode("SATELLITE")
        self.assertEqual(self.tab.mode(), "OFFLINE")

    def test_points_reach_the_offline_map(self):
        self.tab.push_points([_contact(lat=48.2, lon=16.4)])
        self.assertEqual(self.tab.world.point_count(), 1)

    def test_me_marker_and_rays_reach_the_offline_map(self):
        self.tab._me_obj = {"lat": 1.0, "lon": 2.0, "label": "ME"}
        self.tab.me_enable.setChecked(True)
        self.assertIsNotNone(self.tab.world._me)
        self.assertTrue(self.tab.world.show_rays)
        self.tab.me_enable.setChecked(False)
        self.assertIsNone(self.tab.world._me)
        self.assertFalse(self.tab.world.show_rays)


# --------------------------------------------------------------------------
# window integration
# --------------------------------------------------------------------------

class TestMainWindowFeatures(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.ini = os.path.join(self.tmp, "blackice.ini")
        self.windows = []

    def tearDown(self):
        for w in self.windows:
            w.close()
            w.deleteLater()
        _app.processEvents()
        _app.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _cfg(self):
        return bit.AppConfig(QtCore.QSettings(self.ini, QtCore.QSettings.Format.IniFormat))

    def _window(self, **kw):
        w = bit.MainWindow(config=self._cfg(),
                           store=bit.SessionStore(os.path.join(self.tmp, "h.db")),
                           **kw)
        w.watchlist_path = os.path.join(self.tmp, "watchlist.txt")
        self.windows.append(w)
        return w

    def test_listener_export_writes_a_file(self):
        w = self._window()
        w.listen.set_listeners([_listener()])
        path = os.path.join(self.tmp, "listeners.csv")
        with mock.patch.object(bit.QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=(path, "")):
            w._export_listeners()
        self.assertTrue(os.path.exists(path))
        self.assertIn("listeners CSV exported", w.dash.log.toPlainText())

    def test_listen_tab_exists_and_receives_listeners(self):
        w = self._window()
        self.assertEqual([w.tabs.tabText(i) for i in range(w.tabs.count())],
                         ["BLACK ICE", "CONTACTS", "LISTEN", "MAP", "STATS", "HISTORY"])
        w.listen.set_listeners([_listener()])
        self.assertEqual(w.listen.model.rowCount(), 1)

    def test_nothing_is_recorded_while_history_is_off(self):
        w = self._window()
        self.assertFalse(w.store.enabled)
        w._on_points([_contact()])
        self.assertFalse(os.path.exists(os.path.join(self.tmp, "h.db")))

    def test_history_toggle_records_and_reports(self):
        w = self._window()
        w.dash.history_btn.setChecked(True)
        self.assertTrue(w.store.enabled)
        w._on_points([_contact()])
        self.assertEqual(w.store.counts()[0], 1)
        self.assertIn("history ENABLED", w.dash.log.toPlainText())

    def test_only_the_new_batch_is_recorded(self):
        # Handing the store the whole table each scan would add one hit per
        # row per scan instead of one hit per sighting.
        w = self._window()
        w.dash.history_btn.setChecked(True)
        w._on_points([_contact(ip="203.0.113.1"), _contact(ip="203.0.113.2")])
        w._on_points([_contact(ip="203.0.113.1")])
        rows = {r["ip"]: r["hits"] for r in w.store.recent()}
        self.assertEqual(rows, {"203.0.113.1": 2, "203.0.113.2": 1})

    def test_history_toggle_off_closes_the_store(self):
        w = self._window()
        w.dash.history_btn.setChecked(True)
        w.dash.history_btn.setChecked(False)
        self.assertFalse(w.store.enabled)
        self.assertIsNone(w.conns.baseline)

    def test_history_failure_untoggles_the_button(self):
        w = self._window()
        w.store.path = os.path.join(self.tmp, "h.db", "nested", "x.db")
        with open(os.path.join(self.tmp, "h.db"), "w", encoding="utf-8") as f:
            f.write("blocker")
        w.dash.history_btn.setChecked(True)
        self.assertFalse(w.dash.history_btn.isChecked())
        self.assertIn("history unavailable", w.dash.log.toPlainText())

    def test_baseline_marks_unseen_addresses_as_new(self):
        w = self._window()
        w.dash.history_btn.setChecked(True)
        w._on_points([_contact(ip="8.8.8.8")])
        self.assertIn("NEW",
                      w.conns.model.item(0, w.conns.C_FLAGS).text())

    def test_watchlist_append_writes_and_reloads(self):
        w = self._window()
        w.append_watchlist("cidr", "203.0.113.9/32", "MANUAL")
        with open(w.watchlist_path, encoding="utf-8") as f:
            body = f.read()
        self.assertIn("cidr 203.0.113.9/32 MANUAL", body)
        self.assertIn("BLACK ICE watchlist", body)  # template header on creation
        self.assertEqual(len(w.conns.rules), 1)

    def test_alert_reaches_the_banner_and_the_log(self):
        w = self._window()
        w.append_watchlist("cidr", "203.0.113.0/24", "TESTNET")
        w._on_points([_contact()])
        self.assertIn("WATCHLIST", w.dash.alert_lbl.text())
        self.assertIn("203.0.113.9:443", w.dash.alert_lbl.text())
        self.assertIn("[!] ALERT", w.dash.log.toPlainText())

    def test_resolved_host_reaches_the_table(self):
        w = self._window()
        w._on_points([_contact()])
        w._on_host_resolved("203.0.113.9", "resolved.example")
        self.assertEqual(w.conns.model.item(0, w.conns.C_HOST).text(),
                         "resolved.example")

    def test_hide_virtual_reaches_the_poller_and_the_combo(self):
        w = self._window()
        w.dash.hide_virtual.setChecked(True)
        self.assertTrue(w.poller.hide_virtual)
        names = [w.dash.iface.itemText(i) for i in range(w.dash.iface.count())]
        self.assertFalse([n for n in names if bit.is_virtual_nic(n)])

    def test_samples_are_flushed_once_a_minute(self):
        w = self._window()
        w.dash.history_btn.setChecked(True)
        snap = {"_totals": {"rx_bps": 100.0, "tx_bps": 50.0}}
        w._accumulate_sample(snap)
        self.assertEqual(w.store.counts()[1], 0)
        w._sample_flushed -= w.SAMPLE_PERIOD + 1
        w._accumulate_sample(snap)
        self.assertEqual(w.store.counts()[1], 1)

    def test_rollups_refresh_on_the_ui_tick(self):
        w = self._window()
        w._on_points([_contact(country="DE"), _contact(ip="198.51.100.2",
                                                       country="DE")])
        w._ui_tick()
        self.assertEqual(w.stats.geo_panel._rows, [("DE", 2)])

    def test_new_view_state_round_trips(self):
        w = self._window()
        w.dash.config_btn.setChecked(True)
        w.dash.fx_combo.setCurrentText("SUBTLE")
        w.dash.hide_virtual.setChecked(True)
        w.listen.world_only.setChecked(True)
        w.map.set_mode("ONLINE")
        w.save_state()

        w2 = self._window()
        self.assertEqual(w2.dash.fx_combo.currentText(), "SUBTLE")
        self.assertEqual(w2.dash.matrix.gain, bit._FX_GAIN["SUBTLE"])
        self.assertTrue(w2.dash.hide_virtual.isChecked())
        self.assertTrue(w2.listen.world_only.isChecked())
        self.assertEqual(w2.map.mode(), "ONLINE")

    def test_history_flag_is_restored(self):
        w = self._window()
        w.dash.config_btn.setChecked(True)
        w.dash.history_btn.setChecked(True)
        w.save_state()

        w2 = self._window()
        self.assertTrue(w2.dash.history_btn.isChecked())
        self.assertTrue(w2.store.enabled)

    def test_whois_opens_a_browser_for_the_address(self):
        w = self._window()
        with mock.patch.object(bit.QtGui.QDesktopServices, "openUrl") as opened:
            w._open_whois("203.0.113.9")
        opened.assert_called_once()
        self.assertIn("203.0.113.9", opened.call_args[0][0].toString())


class TestReadmeMatchesCode(unittest.TestCase):
    """The README drifted seven versions behind once already (its screenshots
    still showed v0.3.0's two tabs). These pin the claims a test can check."""

    @classmethod
    def setUpClass(cls):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.root = root
        with open(os.path.join(root, "README.md"), encoding="utf-8") as f:
            cls.readme = f.read()

    def test_every_cli_flag_is_documented(self):
        parser = bit.build_arg_parser()
        flags = {s for a in parser._actions for s in a.option_strings
                 if s.startswith("--")} - {"--help"}
        documented = set(re.findall(r"\|\s*`(--[a-z-]+)[^`]*`", self.readme))
        self.assertEqual(flags, documented)

    def test_documented_defaults_are_the_real_defaults(self):
        parser = bit.build_arg_parser()
        self.assertIn(f"default {int(parser.get_default('duration'))}s", self.readme)
        self.assertIn(f"default {int(parser.get_default('interval'))}", self.readme)

    def test_every_watchlist_rule_kind_has_an_example(self):
        documented = set(re.findall(r"^(cidr|port|asn|country|process|host)\s",
                                    self.readme, re.M))
        self.assertEqual(set(bit.RULE_KINDS), documented)

    def test_paths_are_the_ones_the_code_uses(self):
        self.assertTrue(bit.app_settings().fileName().endswith(
            "blackice/blackice_traffic.ini"))
        self.assertIn("~/.config/blackice/blackice_traffic.ini", self.readme)
        self.assertTrue(bit.default_watchlist_path().endswith("blackice/watchlist.txt"))
        self.assertIn("~/.config/blackice/watchlist.txt", self.readme)
        self.assertTrue(bit.default_history_path().endswith(
            ".local/share/blackice/history.db"))
        self.assertIn("~/.local/share/blackice/history.db", self.readme)

    def test_quoted_numbers_are_current(self):
        outline = bit.WorldOutline.load()
        self.assertIn(f"{len(outline)} rings, {outline.point_count} points", self.readme)
        self.assertIn(f"{bit.SessionStore.RETENTION_DAYS} days", self.readme)
        self.assertEqual(bit.MainWindow.SAMPLE_PERIOD, 60.0)
        self.assertIn("once\u2011a\u2011minute", self.readme)

    def test_fx_levels_and_exposure_classes(self):
        self.assertEqual(bit.FX_LEVELS, ("OFF", "SUBTLE", "FULL"))
        self.assertIn("OFF / SUBTLE / FULL", self.readme)
        self.assertIn("(WORLD / LAN / LOCAL)", self.readme)

    def test_referenced_screenshots_exist(self):
        images = re.findall(r"\]\((resources/[^)]+\.png)\)", self.readme)
        self.assertTrue(images)
        for rel in images:
            self.assertTrue(os.path.exists(os.path.join(self.root, rel)), rel)

    def test_world_outline_asset_ships_in_every_build(self):
        for script in ("build_linux_bin.sh", "build_win.cmd", "build_deb.sh"):
            with open(os.path.join(self.root, script), encoding="utf-8") as f:
                self.assertIn("world.bin", f.read(), script)

    def test_linux_build_does_not_need_an_activated_venv(self):
        with open(os.path.join(self.root, "build_linux_bin.sh"), encoding="utf-8") as f:
            build = f.read()
        # A bare `pyinstaller` resolves only inside an activated venv, so `make`
        # from a plain shell died with "command not found".
        self.assertNotRegex(build, r"(?m)^\s*pyinstaller\s")
        self.assertIn(".venv/bin/python", build)
        self.assertIn("-m PyInstaller", build)

    def test_control_file_has_no_hand_maintained_size(self):
        with open(os.path.join(self.root, "DEBIAN", "control"), encoding="utf-8") as f:
            control = f.read()
        self.assertIn("Installed-Size: _size_", control)
        self.assertIn("Version: _version_", control)
        with open(os.path.join(self.root, "build_deb.sh"), encoding="utf-8") as f:
            build = f.read()
        self.assertIn("s/_size_/", build)


class TestMainWindowDocClaims(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.w = bit.MainWindow(
            config=bit.AppConfig(QtCore.QSettings(
                os.path.join(self.tmp, "b.ini"), QtCore.QSettings.Format.IniFormat)),
            store=bit.SessionStore(os.path.join(self.tmp, "h.db")))

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        _app.processEvents()
        _app.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_snapshot_button_on_every_tab(self):
        for i in range(self.w.tabs.count()):
            tab = self.w.tabs.widget(i)
            self.assertTrue(hasattr(tab, "snapshot_btn"), self.w.tabs.tabText(i))

    def test_save_state_writes_exactly_the_documented_keys(self):
        written = set()
        self.w.cfg._enabled = True
        self.w.cfg.set = lambda k, v: written.add(k)
        self.w.save_state()
        self.assertEqual(written, {
            "window/geometry", "window/tab", "dash/iface", "dash/fx",
            "dash/hide_virtual", "dash/scan_interval",
            "stats/iface", "stats/window", "contacts/header",
            "contacts/sort_col", "contacts/sort_order", "contacts/filter",
            "map/splitter", "map/show_me", "map/mode", "map/cluster",
            "window/tray", "history/enabled",
            "listen/world_only", "listen/header"})


# --------------------------------------------------------------------------
# regressions for the 0.12.1 fix round


class TestUdpListenersAreReported(unittest.TestCase):
    """A UDP socket never reports LISTEN, so keying the listening set on that
    status alone hid every bound UDP port from the attack-surface tab."""

    Addr = collections.namedtuple("Addr", "ip port")
    Conn = collections.namedtuple("Conn", "fd family type laddr raddr status pid")

    def _scan(self, conns):
        scanner = bit.ConnScanner()
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns):
            return scanner._scan_psutil()

    def test_bound_udp_socket_is_listed(self):
        conns = [self.Conn(1, 2, 2, self.Addr("0.0.0.0", 53), (), "NONE", None)]
        _c, listening, _s = self._scan(conns)
        self.assertEqual(len(listening), 1)
        self.assertEqual(listening[0].proto, "udp")
        self.assertEqual(listening[0].port, 53)
        self.assertEqual(listening[0].exposure, bit.EXPOSURE_WORLD)

    def test_sensitive_udp_port_still_gets_its_note(self):
        conns = [self.Conn(1, 2, 2, self.Addr("::", 5900), (), "NONE", None)]
        _c, listening, _s = self._scan(conns)
        self.assertTrue(listening[0].note)

    def test_loopback_udp_bind_is_local(self):
        conns = [self.Conn(1, 2, 2, self.Addr("127.0.0.1", 323), (), "NONE", None)]
        _c, listening, _s = self._scan(conns)
        self.assertEqual(listening[0].exposure, bit.EXPOSURE_LOCAL)

    def test_tcp_listener_is_unaffected(self):
        conns = [self.Conn(1, 2, 1, self.Addr("0.0.0.0", 22), (), "LISTEN", None)]
        _c, listening, _s = self._scan(conns)
        self.assertEqual(listening[0].proto, "tcp")

    def test_udp_state_histogram_is_still_not_polluted(self):
        conns = [self.Conn(1, 2, 2, self.Addr("0.0.0.0", 53), (), "NONE", None)]
        _c, _l, states = self._scan(conns)
        self.assertEqual(states, {})

    def test_process_name_is_looked_up_once_per_pid(self):
        conns = [self.Conn(i, 2, 1, self.Addr("0.0.0.0", 0),
                           self.Addr("203.0.113.%d" % i, 443), "ESTABLISHED", 77)
                 for i in range(1, 6)]
        proc = mock.Mock()
        proc.name.return_value = "curl"
        scanner = bit.ConnScanner()
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns), \
             mock.patch.object(bit.psutil, "Process", return_value=proc) as factory:
            contacts, _l, _s = scanner._scan_psutil()
        self.assertEqual(len(contacts), 5)
        self.assertEqual(factory.call_count, 1)
        self.assertTrue(all(c.process == "curl" for c in contacts))


class TestCsvInjection(unittest.TestCase):
    """Hostnames come from PTR records and org strings from GeoIP: both are
    chosen by somebody else and both end up in a spreadsheet."""

    def test_formula_prefixes_are_defused(self):
        for raw in ("=1+1", "+1", "-1", "@SUM(A1)", "\tx", "\rx"):
            self.assertTrue(bit.csv_safe(raw).startswith("'"), raw)

    def test_ordinary_values_are_untouched(self):
        for raw in ("203.0.113.9", "host.example", "AS13335 CLOUDFLARE", ""):
            self.assertEqual(bit.csv_safe(raw), raw)

    def test_none_becomes_empty(self):
        self.assertEqual(bit.csv_safe(None), "")

    def test_headless_export_defuses_a_hostile_ptr(self):
        tmp = tempfile.mkdtemp(prefix="blackice-csv-")
        try:
            path = os.path.join(tmp, "contacts.csv")
            bit.write_contacts_csv(path, [{"ip": "203.0.113.9", "port": 443,
                                           "host": '=cmd|\'/c calc\'!A1'}])
            with open(path, encoding="utf-8") as f:
                body = f.read()
            self.assertIn("'=cmd", body)
            self.assertNotIn(",=cmd", body)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_contacts_tab_export_defuses_a_hostile_ptr(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                         "ts": time.time(), "host": "=HYPERLINK(\"x\")"}])
        tmp = tempfile.mkdtemp(prefix="blackice-csv-")
        try:
            path = os.path.join(tmp, "contacts.csv")
            tab.export_csv(path)
            with open(path, encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertIn("'=HYPERLINK(\"x\")", rows[1])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
            tab.deleteLater()


class TestContactTableCap(unittest.TestCase):
    """A host making short-lived connections produces endpoints forever."""

    def setUp(self):
        self.tab = bit.ConnectionsTab()
        self.tab.resolve_hosts = False
        self.tab.MAX_ROWS = 40
        self.tab.EVICT_CHUNK = 10

    def tearDown(self):
        self.tab.deleteLater()
        _app.processEvents()

    def _flood(self, n, start=0, ts=None):
        now = ts if ts is not None else time.time()
        self.tab.add_points([
            {"ip": "203.0.113.%d" % (i % 254 + 1), "port": 1000 + i, "proto": "tcp",
             "ts": now + i, "location": "XX"} for i in range(start, start + n)])

    def test_table_stops_growing(self):
        for chunk in range(12):
            self._flood(20, start=chunk * 20)
        self.assertLessEqual(self.tab.model.rowCount(), self.tab.MAX_ROWS + self.tab.EVICT_CHUNK)

    def test_row_index_survives_eviction(self):
        self._flood(120)
        # Every surviving key must still point at its own row.
        for key, row in self.tab._rows.items():
            item = self.tab.model.item(row, self.tab.C_FIRST)
            self.assertIsNotNone(item)
            self.assertEqual(item.data(self.tab.KEY_ROLE), key)
        self.assertEqual(set(self.tab._rows), set(self.tab._points))

    def test_the_oldest_contacts_are_the_ones_dropped(self):
        base = time.time()
        self._flood(120, ts=base)
        survivors = {self.tab._points[k]["port"] for k in self.tab._rows}
        self.assertNotIn(1000, survivors)       # first contact, oldest
        self.assertIn(1000 + 119, survivors)    # last contact, newest

    def test_an_evicted_endpoint_can_come_back(self):
        self._flood(120)
        self.tab.add_points([{"ip": "203.0.113.1", "port": 1000, "proto": "tcp",
                              "ts": time.time(), "location": "XX"}])
        self.assertIn("tcp:203.0.113.1:1000", self.tab._rows)


class TestWorldMapPointCap(unittest.TestCase):
    def test_points_stop_growing(self):
        w = bit.WorldMapWidget()
        w.MAX_POINTS = 50
        for chunk in range(5):
            w.set_points([{"proto": "tcp", "ip": "203.0.113.%d" % (i % 254),
                           "port": 1000 + i, "lat": 10.0, "lon": 20.0}
                          for i in range(chunk * 30, chunk * 30 + 30)])
        self.assertLessEqual(w.point_count(), 50)
        w.deleteLater()

    def test_the_newest_point_survives(self):
        w = bit.WorldMapWidget()
        w.MAX_POINTS = 5
        w.set_points([{"proto": "tcp", "ip": "203.0.113.%d" % i, "port": 1,
                       "lat": 1.0, "lon": 2.0} for i in range(20)])
        self.assertIn("tcp:203.0.113.19:1", w._points)
        w.deleteLater()


class TestMapSplitterCanBeNarrowed(unittest.TestCase):
    """The stacked widget's implicit minimum used to pin the map pane wide,
    which silently discarded any saved splitter position below it."""

    def test_map_pane_can_be_made_smaller_than_the_list(self):
        tab = bit.MapTab()
        tab.split.resize(800, 400)
        tab.split.setSizes([250, 550])
        map_w, list_w = tab.split.sizes()
        self.assertLess(map_w, list_w)
        tab.deleteLater()

    def test_saved_position_round_trips(self):
        a = bit.MapTab()
        a.split.resize(800, 400)
        a.split.setSizes([250, 550])
        state = a.split.saveState()
        b = bit.MapTab()
        b.split.resize(800, 400)
        self.assertTrue(b.split.restoreState(state))
        map_w, list_w = b.split.sizes()
        self.assertLess(map_w, list_w)
        a.deleteLater()
        b.deleteLater()


class TestRuleSetCidrIndexing(unittest.TestCase):
    """Each cidr rule has to match against its own network, not whichever
    network happens to appear first in the index."""

    def test_each_rule_matches_only_its_own_network(self):
        rs = bit.RuleSet.parse(
            "cidr 203.0.113.0/24 DOCS\n"
            "cidr 198.51.100.0/24 TEST2\n"
            "cidr 192.0.2.0/24 TEST1\n")
        self.assertEqual(rs.match({"ip": "198.51.100.5"}), ["TEST2"])
        self.assertEqual(rs.match({"ip": "192.0.2.5"}), ["TEST1"])
        self.assertEqual(rs.match({"ip": "203.0.113.5"}), ["DOCS"])
        self.assertEqual(rs.match({"ip": "8.8.8.8"}), [])

    def test_overlapping_rules_all_fire_in_file_order(self):
        rs = bit.RuleSet.parse("cidr 203.0.113.0/24 WIDE\ncidr 203.0.113.5/32 EXACT\n")
        self.assertEqual(rs.match({"ip": "203.0.113.5"}), ["WIDE", "EXACT"])

    def test_a_malformed_cidr_rule_does_not_shift_the_others(self):
        rs = bit.RuleSet([bit.WatchRule("cidr", "not-a-network", "BAD"),
                          bit.WatchRule("cidr", "203.0.113.0/24", "GOOD")])
        self.assertEqual(rs.match({"ip": "203.0.113.5"}), ["GOOD"])

    def test_v4_rule_does_not_match_a_v6_address(self):
        rs = bit.RuleSet.parse("cidr 0.0.0.0/0 ANY4\n")
        self.assertEqual(rs.match({"ip": "2001:db8::1"}), [])


class TestSelftestSkipsOptionalGeoip(unittest.TestCase):
    def test_absent_geoip_database_is_a_skip_not_a_failure(self):
        with mock.patch.object(bit, "HAVE_GEOIP", False), \
             mock.patch.object(bit, "_geoip_reader", None), \
             mock.patch.object(bit, "init_geoip"), \
             mock.patch.object(bit, "init_asn"):
            with mock.patch("sys.stdout", new=io.StringIO()) as out:
                rc = bit.run_selftest()
        self.assertEqual(rc, 0)
        self.assertIn("[SKIP] geoip", out.getvalue())

    def test_a_real_failure_still_exits_non_zero(self):
        with mock.patch.object(bit, "human_bps", return_value="wrong"):
            with mock.patch("sys.stdout", new=io.StringIO()) as out:
                rc = bit.run_selftest()
        self.assertEqual(rc, 1)
        self.assertIn("[FAIL] units", out.getvalue())


# --------------------------------------------------------------------------
# 0.13.0: honest status line, listener diffing, per-process bytes, history
# tab, rule editor, NDJSON stream, map clustering, alert rate limiting


class TestStatusLineIsHonest(unittest.TestCase):
    def test_it_no_longer_claims_to_sniff_or_filter(self):
        d = bit.BlackIceDashboard()
        self.assertNotIn("PASSIVE SNIFF", d.sub.text())
        self.assertNotIn("SYN-FILTER", d.sub.text())
        self.assertIn("NO PACKET CAPTURE", d.sub.text())
        d.deleteLater()

    def test_capabilities_are_appended_and_updated(self):
        d = bit.BlackIceDashboard()
        d.set_status("GEOIP", "OFF")
        d.set_status("HISTORY", "OFF")
        self.assertIn("GEOIP: OFF", d.sub.text())
        self.assertIn("HISTORY: OFF", d.sub.text())
        d.set_status("HISTORY", "ON")
        self.assertIn("HISTORY: ON", d.sub.text())
        self.assertNotIn("HISTORY: OFF", d.sub.text())
        d.deleteLater()


class TestListenerDiff(unittest.TestCase):
    def test_the_first_inventory_is_a_baseline_not_a_hundred_openings(self):
        self.assertEqual(bit.listener_diff(None, {"a": {}}), ([], []))

    def test_opened_and_closed_are_both_reported(self):
        prev = {"tcp:0.0.0.0:22": {"port": 22}}
        cur = {"tcp:0.0.0.0:80": {"port": 80}}
        opened, closed = bit.listener_diff(prev, cur)
        self.assertEqual([l["port"] for l in opened], [80])
        self.assertEqual([l["port"] for l in closed], [22])

    def test_an_unchanged_inventory_is_silent(self):
        same = {"tcp:0.0.0.0:22": {"port": 22}}
        self.assertEqual(bit.listener_diff(dict(same), same), ([], []))

    def test_describe_names_port_process_and_exposure(self):
        line = bit.ListenersTab.describe(_listener(port=22, process="sshd", pid=9))
        self.assertIn("TCP 0.0.0.0:22", line)
        self.assertIn("SSH", line)
        self.assertIn("sshd/9", line)
        self.assertIn(bit.EXPOSURE_WORLD, line)

    def test_tab_emits_nothing_on_the_first_scan(self):
        tab = bit.ListenersTab()
        seen = []
        tab.listenersOpened.connect(seen.append)
        tab.set_listeners([_listener(port=22), _listener(port=80)])
        self.assertEqual(seen, [])
        tab.deleteLater()

    def test_tab_emits_the_second_time_round(self):
        tab = bit.ListenersTab()
        opened, closed = [], []
        tab.listenersOpened.connect(opened.extend)
        tab.listenersClosed.connect(closed.extend)
        tab.set_listeners([_listener(port=22)])
        tab.set_listeners([_listener(port=22), _listener(port=6379)])
        self.assertEqual([l["port"] for l in opened], [6379])
        tab.set_listeners([_listener(port=22)])
        self.assertEqual([l["port"] for l in closed], [6379])
        tab.deleteLater()


class TestEphemeralPortNoise(unittest.TestCase):
    """A client churning outbound sockets must not drown the sentinel."""

    def test_the_kernel_range_is_read_when_available(self):
        bit._port_range_cache = None
        with mock.patch("builtins.open", mock.mock_open(read_data="10000\t20000\n")):
            self.assertEqual(bit.local_port_range(), (10000, 20000))
        bit._port_range_cache = None

    def test_an_unreadable_range_falls_back(self):
        bit._port_range_cache = None
        with mock.patch("builtins.open", side_effect=OSError):
            self.assertEqual(bit.local_port_range(), bit._DEFAULT_PORT_RANGE)
        bit._port_range_cache = None

    def test_a_churned_ephemeral_port_is_not_notable(self):
        lo, _hi = bit.local_port_range()
        self.assertFalse(bit.is_notable_listener(
            {"port": lo + 1, "proto": "udp", "note": ""}))

    def test_a_service_port_is_always_notable(self):
        self.assertTrue(bit.is_notable_listener({"port": 80, "note": ""}))

    def test_a_known_service_high_in_the_range_is_still_notable(self):
        self.assertTrue(bit.is_notable_listener({"port": 51820, "note": ""}))

    def test_a_sensitive_note_overrides_everything(self):
        lo, _hi = bit.local_port_range()
        self.assertTrue(bit.is_notable_listener({"port": lo + 2, "note": "DATABASE"}))

    def test_the_noisy_socket_still_appears_in_the_inventory(self):
        tab = bit.ListenersTab()
        lo, _hi = bit.local_port_range()
        tab.set_listeners([_listener(port=lo + 3, proto="udp", process="syncthing")])
        self.assertEqual(tab.model.rowCount(), 1)
        tab.deleteLater()

    def test_it_is_not_logged_as_a_port_opening(self):
        tmp = tempfile.mkdtemp(prefix="blackice-noise-")
        try:
            w = bit.MainWindow(config=bit.AppConfig(QtCore.QSettings(
                os.path.join(tmp, "cfg.ini"), QtCore.QSettings.Format.IniFormat)))
            lo, _hi = bit.local_port_range()
            w._on_listeners_opened([
                _listener(port=lo + 4, proto="udp", process="syncthing")])
            self.assertNotIn("PORT OPENED", w.dash.log.toPlainText())
            w._on_listeners_opened([_listener(port=22, process="sshd")])
            self.assertIn("PORT OPENED", w.dash.log.toPlainText())
            w.close()
            w.deleteLater()
            _app.processEvents()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestProcTrafficSampler(unittest.TestCase):
    SS = (
        'ESTAB 0 0 192.168.1.5:40994 199.232.197.91:443 '
        'users:(("firefox",pid=1283969,fd=229))\n'
        '\t cubic rtt:29.6 bytes_sent:4720 bytes_acked:4721 bytes_received:6099 '
        'segs_out:20\n'
        'ESTAB 0 0 192.168.1.5:37008 88.99.166.23:12476\n'
        '\t cubic rtt:57.6 bytes_sent:1509 bytes_acked:1510 bytes_received:92991\n'
    )

    def _sampler(self):
        s = bit.ProcTrafficSampler()
        s.enabled = True
        return s

    def test_parse_pulls_bytes_and_owner(self):
        parsed = bit.ProcTrafficSampler.parse(self.SS)
        self.assertEqual(len(parsed), 2)
        proc, rx, tx = parsed["192.168.1.5:40994|199.232.197.91:443"]
        self.assertEqual((proc, rx, tx), ("firefox", 6099, 4720))

    def test_a_socket_without_a_named_owner_is_not_dropped(self):
        parsed = bit.ProcTrafficSampler.parse(self.SS)
        proc, rx, _tx = parsed["192.168.1.5:37008|88.99.166.23:12476"]
        self.assertEqual(proc, bit.UNATTRIBUTED)
        self.assertEqual(rx, 92991)

    def test_older_kernels_without_bytes_sent_fall_back_to_bytes_acked(self):
        text = ('ESTAB 0 0 a:1 b:2 users:(("x",pid=1,fd=2))\n'
                '\t cubic bytes_acked:500 bytes_received:100\n')
        parsed = bit.ProcTrafficSampler.parse(text)
        self.assertEqual(parsed["a:1|b:2"], ("x", 100, 500))

    def test_a_socket_with_no_info_line_is_skipped(self):
        parsed = bit.ProcTrafficSampler.parse("LISTEN 0 128 0.0.0.0:22 0.0.0.0:*\n")
        self.assertEqual(parsed, {})

    def test_the_first_pass_only_establishes_a_baseline(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=self.SS):
            self.assertEqual(s.sample(), {})

    def test_the_second_pass_counts_the_delta_only(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=self.SS):
            s.sample()
        grown = self.SS.replace("bytes_received:6099", "bytes_received:9099")
        with mock.patch.object(s, "_run", return_value=grown):
            totals = s.sample()
        self.assertEqual(totals["firefox"], (3000, 0))

    def test_a_socket_appearing_after_the_baseline_counts_in_full(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=""):
            s.sample()
        with mock.patch.object(s, "_run", return_value=self.SS):
            totals = s.sample()
        self.assertEqual(totals["firefox"], (6099, 4720))

    def test_a_reused_address_pair_does_not_produce_a_negative_delta(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=self.SS):
            s.sample()
        reset = self.SS.replace("bytes_received:6099", "bytes_received:12")
        with mock.patch.object(s, "_run", return_value=reset):
            totals = s.sample()
        self.assertEqual(totals["firefox"][0], 12)

    def test_a_closed_socket_keeps_the_bytes_it_moved(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=""):
            s.sample()
        with mock.patch.object(s, "_run", return_value=self.SS):
            s.sample()
        with mock.patch.object(s, "_run", return_value=""):
            totals = s.sample()
        self.assertEqual(totals["firefox"], (6099, 4720))

    def test_top_ranks_by_total_bytes(self):
        s = self._sampler()
        s.totals = {"a": (10, 10), "b": (500, 0), "c": (0, 0)}
        self.assertEqual(s.top(), [("b", 500), ("a", 20)])

    def test_it_disables_itself_after_repeated_failures(self):
        s = self._sampler()
        with mock.patch.object(bit.subprocess, "run", side_effect=OSError("nope")):
            for _ in range(bit.ProcTrafficSampler.MAX_FAILURES):
                s.sample()
        self.assertFalse(s.enabled)
        self.assertIn("nope", s.error)

    def test_a_non_zero_exit_is_reported_not_raised(self):
        s = self._sampler()
        done = subprocess.CompletedProcess(["ss"], 2, "", "operation not permitted")
        with mock.patch.object(bit.subprocess, "run", return_value=done):
            self.assertEqual(s.sample(), {})
        self.assertIn("not permitted", s.error)

    def test_a_hung_ss_is_given_up_on(self):
        s = self._sampler()
        with mock.patch.object(bit.subprocess, "run",
                               side_effect=subprocess.TimeoutExpired("ss", 4.0)):
            self.assertEqual(s.sample(), {})
        self.assertFalse(s.enabled is None)

    def test_a_missing_binary_is_reported_not_raised(self):
        with mock.patch.object(bit.shutil, "which", return_value=None):
            s = bit.ProcTrafficSampler()
        self.assertFalse(s.enabled)
        self.assertEqual(s.sample(), {})


class TestStatsTalkerPanel(unittest.TestCase):
    def setUp(self):
        self.tab = bit.StatsTab()

    def tearDown(self):
        self.tab.deleteLater()
        _app.processEvents()

    def test_it_counts_endpoints_until_real_bytes_arrive(self):
        self.tab.set_rollups([{"process": "curl"}, {"process": "curl"}])
        self.assertEqual(self.tab.proc_panel.title, bit.StatsTab.TALKERS_BY_COUNT)
        self.assertEqual(self.tab.proc_panel._rows, [("curl", 2)])

    def test_real_bytes_take_over_the_panel(self):
        self.tab.set_proc_io({"firefox": (2_000_000, 1_000_000), "sshd": (10, 10)})
        self.assertEqual(self.tab.proc_panel.title, bit.StatsTab.TALKERS_BY_BYTES)
        self.assertEqual(self.tab.proc_panel._rows[0], ("firefox", 3_000_000))
        self.assertEqual(self.tab.proc_panel._fmt, bit.human_bytes)

    def test_bytes_survive_a_rollup_refresh(self):
        self.tab.set_proc_io({"firefox": (2_000_000, 1_000_000)})
        self.tab.set_rollups([{"process": "curl"}])
        self.assertEqual(self.tab.proc_panel.title, bit.StatsTab.TALKERS_BY_BYTES)

    def test_a_process_that_moved_nothing_is_not_listed(self):
        self.tab.set_proc_io({"idle": (0, 0)})
        self.assertEqual(self.tab.proc_panel._rows, [])


class TestHistoryTab(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="blackice-hist-")
        self.store = bit.SessionStore(os.path.join(self.tmp, "history.db"))
        self.tab = bit.HistoryTab()

    def tearDown(self):
        self.store.close()
        self.tab.deleteLater()
        _app.processEvents()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _record(self, n=3):
        self.store.open()
        now = time.time()
        self.store.record_contacts([
            {"ip": "203.0.113.%d" % i, "port": 440 + i, "proto": "tcp", "ts": now + i,
             "location": "Berlin DE", "country": "DE", "asn": "AS3320 DTAG",
             "process": "firefox", "host": "h%d.example" % i}
            for i in range(n)])

    def test_it_says_so_when_recording_is_off(self):
        self.tab.set_store(self.store)
        self.assertEqual(self.tab.model.rowCount(), 0)
        self.assertIn("OFF", self.tab.summary.text())
        self.assertFalse(self.tab.export_btn.isEnabled())

    def test_recorded_contacts_reach_the_table(self):
        self._record(3)
        self.tab.set_store(self.store)
        self.assertEqual(self.tab.model.rowCount(), 3)
        self.assertIn("3 recorded contacts", self.tab.summary.text())
        self.assertTrue(self.tab.export_btn.isEnabled())

    def test_columns_carry_what_was_stored(self):
        self._record(1)
        self.tab.set_store(self.store)
        texts = [self.tab.model.item(0, c).text()
                 for c in range(self.tab.model.columnCount())]
        self.assertIn("203.0.113.0", texts)
        self.assertIn("h0.example", texts)
        self.assertIn("AS3320 DTAG", texts)

    def test_timestamps_are_absolute_because_history_spans_days(self):
        self._record(1)
        self.tab.set_store(self.store)
        self.assertRegex(self.tab.model.item(0, self.tab.C_LAST).text(),
                         r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$")

    def test_the_filter_narrows_the_view(self):
        self._record(3)
        self.tab.set_store(self.store)
        self.tab.filter_edit.setText("203.0.113.1")
        self.assertEqual(self.tab.proxy.rowCount(), 1)

    def test_forget_all_empties_the_database(self):
        self._record(3)
        self.tab.set_store(self.store)
        with mock.patch.object(QtWidgets.QMessageBox, "question",
                               return_value=QtWidgets.QMessageBox.StandardButton.Yes):
            self.tab._confirm_forget()
        self.assertEqual(self.tab.model.rowCount(), 0)
        self.assertEqual(self.store.counts()[0], 0)

    def test_cancelling_forget_keeps_everything(self):
        self._record(3)
        self.tab.set_store(self.store)
        with mock.patch.object(QtWidgets.QMessageBox, "question",
                               return_value=QtWidgets.QMessageBox.StandardButton.Cancel):
            self.tab._confirm_forget()
        self.assertEqual(self.store.counts()[0], 3)

    def test_export_writes_the_visible_rows(self):
        self._record(2)
        self.tab.set_store(self.store)
        path = os.path.join(self.tmp, "history.csv")
        self.tab.export_csv(path)
        with open(path, encoding="utf-8") as f:
            rows = list(csv.reader(f))
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0], self.tab.COLS)


class TestWatchlistDialog(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="blackice-rules-")
        self.path = os.path.join(self.tmp, "watchlist.txt")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_a_missing_file_opens_on_the_template(self):
        d = bit.WatchlistDialog(self.path)
        self.assertIn("BLACK ICE watchlist", d.text())
        self.assertIn("no problems", d.status.text())
        d.deleteLater()

    def test_an_existing_file_is_loaded_verbatim(self):
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("# mine\ncidr 203.0.113.0/24 DOCS\n")
        d = bit.WatchlistDialog(self.path)
        self.assertIn("# mine", d.text())
        self.assertIn("1 rule(s)", d.status.text())
        d.deleteLater()

    def test_a_bad_line_is_named_before_saving(self):
        d = bit.WatchlistDialog(self.path)
        d.edit.setPlainText("cidr not-a-network OOPS\nport 22 SSH\n")
        self.assertIn("line 1", d.status.text())
        self.assertIn("1 problem", d.status.text())
        d.deleteLater()

    def test_saving_writes_the_file_and_keeps_comments(self):
        d = bit.WatchlistDialog(self.path)
        d.edit.setPlainText("# keep me\nport 3389 RDP")
        d._save()
        self.assertTrue(d.saved)
        with open(self.path, encoding="utf-8") as f:
            body = f.read()
        self.assertIn("# keep me", body)
        self.assertTrue(body.endswith("\n"))
        self.assertEqual(len(bit.RuleSet.load(self.path)), 1)
        d.deleteLater()

    def test_an_unwritable_path_reports_instead_of_raising(self):
        d = bit.WatchlistDialog(os.path.join(self.tmp, "nope", "watchlist.txt"))
        with mock.patch.object(bit.os, "makedirs", side_effect=OSError("denied")):
            d._save()
        self.assertFalse(d.saved)
        self.assertIn("denied", d.status.text())
        d.deleteLater()

    def test_reload_discards_unsaved_edits(self):
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("port 22 SSH\n")
        d = bit.WatchlistDialog(self.path)
        d.edit.setPlainText("garbage")
        d.reload()
        self.assertIn("port 22 SSH", d.text())
        d.deleteLater()


class TestNdjsonStream(unittest.TestCase):
    Addr = collections.namedtuple("Addr", "ip port")
    Conn = collections.namedtuple("Conn", "fd family type laddr raddr status pid")

    def _args(self, *extra):
        return bit.build_arg_parser().parse_args(
            ["--json", "--duration", "0", "--interval", "0"] + list(extra))

    def _conns(self):
        return [
            self.Conn(1, 2, 1, self.Addr("0.0.0.0", 0),
                      self.Addr("203.0.113.9", 443), "ESTABLISHED", None),
            self.Conn(2, 2, 1, self.Addr("0.0.0.0", 22), (), "LISTEN", None),
        ]

    def _run(self, *extra):
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(bit.psutil, "net_connections", return_value=self._conns()), \
             mock.patch.object(bit.ProcTrafficSampler, "available", return_value=False), \
             mock.patch("sys.stdout", new=out), mock.patch("sys.stderr", new=err):
            rc = bit.run_headless(self._args(*extra))
        return rc, out.getvalue(), err.getvalue()

    def test_stdout_is_nothing_but_ndjson(self):
        rc, out, _err = self._run()
        self.assertEqual(rc, 0)
        lines = [l for l in out.splitlines() if l.strip()]
        self.assertTrue(lines)
        for line in lines:
            json.loads(line)

    def test_the_human_summary_goes_to_stderr(self):
        _rc, out, err = self._run()
        self.assertIn("headless", err)
        self.assertNotIn("headless", out)

    def test_a_record_carries_contacts_listeners_and_states(self):
        _rc, out, _err = self._run()
        record = json.loads(out.splitlines()[0])
        self.assertEqual(record["pass"], 1)
        self.assertEqual(record["version"], bit.APP_VERSION)
        self.assertEqual([c["ip"] for c in record["contacts"]], ["203.0.113.9"])
        self.assertEqual([l["port"] for l in record["listeners"]], [22])
        self.assertEqual(record["states"], {"ESTABLISHED": 1, "LISTEN": 1})

    def test_the_first_record_reports_no_port_changes(self):
        _rc, out, _err = self._run()
        record = json.loads(out.splitlines()[0])
        self.assertEqual(record["listeners_opened"], [])
        self.assertEqual(record["listeners_closed"], [])

    def test_watchlist_hits_are_called_out_in_the_record(self):
        rules = bit.RuleSet.parse("cidr 203.0.113.0/24 DOCS\n")
        out = io.StringIO()
        bit.emit_scan_json(1, [{"ip": "203.0.113.9", "port": 443, "proto": "tcp"}],
                           [], [], [], {}, {}, rules, stream=out)
        record = json.loads(out.getvalue())
        self.assertEqual(record["flagged"],
                         [{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                           "flags": ["DOCS"]}])

    def test_proc_bytes_are_split_into_rx_and_tx(self):
        out = io.StringIO()
        bit.emit_scan_json(1, [], [], [], [], {}, {"firefox": (10, 20)},
                           bit.RuleSet(), stream=out)
        record = json.loads(out.getvalue())
        self.assertEqual(record["proc_bytes"], {"firefox": {"rx": 10, "tx": 20}})

    def test_json_implies_headless(self):
        args = bit.build_arg_parser().parse_args(["--json"])
        self.assertFalse(args.headless)
        with mock.patch.object(bit, "run_headless", return_value=0) as run, \
             mock.patch.object(bit, "init_geoip"), mock.patch.object(bit, "init_asn"):
            with self.assertRaises(SystemExit):
                bit.main(["--json"])
        self.assertTrue(run.called)


class TestMapClustering(unittest.TestCase):
    def setUp(self):
        self.w = bit.WorldMapWidget()

    def tearDown(self):
        self.w.deleteLater()
        _app.processEvents()

    def _points(self):
        return [
            {"proto": "tcp", "ip": "203.0.113.1", "port": 443, "lat": 39.0,
             "lon": -77.0, "asn": "AS16509 AMAZON"},
            {"proto": "tcp", "ip": "203.0.113.2", "port": 443, "lat": 39.2,
             "lon": -77.4, "asn": "AS16509 AMAZON"},
            {"proto": "tcp", "ip": "203.0.113.3", "port": 443, "lat": 52.5,
             "lon": 13.4, "asn": "AS3320 DTAG"},
        ]

    def test_contacts_group_by_owning_network(self):
        self.w.set_points(self._points())
        clusters = self.w.clusters()
        self.assertEqual([(c["key"], c["count"]) for c in clusters],
                         [("AS16509", 2), ("AS3320", 1)])

    def test_a_cluster_sits_at_the_mean_of_its_members(self):
        self.w.set_points(self._points())
        amazon = self.w.clusters()[0]
        self.assertAlmostEqual(amazon["lat"], 39.1, places=5)
        self.assertAlmostEqual(amazon["lon"], -77.2, places=5)

    def test_contacts_without_an_asn_fall_back_to_a_geographic_cell(self):
        self.w.set_points([
            {"proto": "tcp", "ip": "198.51.100.%d" % i, "port": 80,
             "lat": 48.1 + i * 0.1, "lon": 16.3, "asn": ""} for i in range(3)])
        clusters = self.w.clusters()
        self.assertEqual(len(clusters), 1)
        self.assertTrue(clusters[0]["key"].startswith("@"))
        self.assertIn("3 contact(s)", clusters[0]["label"])

    def test_the_label_names_the_network_and_samples_members(self):
        self.w.set_points(self._points())
        label = self.w.clusters()[0]["label"]
        self.assertIn("AS16509 AMAZON", label)
        self.assertIn("2 contact(s)", label)
        self.assertIn("203.0.113.1:443", label)

    def test_clustering_switches_off_once_zoomed_in(self):
        self.assertTrue(self.w.clustering())
        self.w.zoom = bit.WorldMapWidget.CLUSTER_ZOOM + 0.1
        self.assertFalse(self.w.clustering())

    def test_the_toggle_wins_over_the_zoom(self):
        self.w.set_cluster(False)
        self.assertFalse(self.w.clustering())

    def test_a_bigger_cluster_draws_a_bigger_circle(self):
        self.assertLess(bit.WorldMapWidget.cluster_radius(2),
                        bit.WorldMapWidget.cluster_radius(50))

    def test_the_radius_is_capped(self):
        self.assertLessEqual(bit.WorldMapWidget.cluster_radius(100000), 16.0)

    def test_hover_names_the_cluster_while_clustered(self):
        self.w.resize(800, 400)
        self.w.set_points(self._points())
        cluster = self.w.clusters()[0]
        x, y = self.w.project(cluster["lon"], cluster["lat"])
        hit = self.w.point_at(QtCore.QPoint(int(x), int(y)))
        self.assertIsNotNone(hit)
        self.assertIn("AS16509", hit["label"])

    def test_painting_a_clustered_map_does_not_raise(self):
        self.w.resize(400, 300)
        self.w.set_points(self._points())
        self.w.set_me({"lat": 50.0, "lon": 8.0})
        self.w.set_rays(True)
        self.w.grab()

    def test_the_map_tab_toggle_drives_the_widget(self):
        tab = bit.MapTab()
        tab.cluster_enable.setChecked(False)
        self.assertFalse(tab.world.cluster)
        tab.cluster_enable.setChecked(True)
        self.assertTrue(tab.world.cluster)
        tab.deleteLater()


class TestAlertRateLimit(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="blackice-alert-")
        self.w = bit.MainWindow(config=bit.AppConfig(QtCore.QSettings(
            os.path.join(self.tmp, "cfg.ini"), QtCore.QSettings.Format.IniFormat)))

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        _app.processEvents()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_the_same_hit_is_reported_once(self):
        for _ in range(20):
            self.w._on_alert("203.0.113.9:443", "DOCS")
        self.assertEqual(self.w._alerts, 1)
        self.assertEqual(self.w._alerts_muted, 19)

    def test_the_banner_counts_distinct_endpoints(self):
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.w._on_alert("203.0.113.10:443", "DOCS")
        self.assertIn("2 endpoint(s)", self.w.dash.alert_lbl.text())

    def test_muted_repeats_are_disclosed_not_hidden(self):
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.w._on_alert("203.0.113.10:443", "DOCS")
        self.assertIn("repeats muted", self.w.dash.alert_lbl.text())

    def test_a_hit_repeats_once_the_window_has_passed(self):
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.w._alert_seen[("203.0.113.9:443", "DOCS")] -= (
            self.w.ALERT_REPEAT_SECONDS + 1)
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.assertEqual(self.w._alerts, 2)

    def test_a_new_rule_on_the_same_endpoint_is_new_news(self):
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.w._on_alert("203.0.113.9:443", "TOR EXIT")
        self.assertEqual(self.w._alerts, 2)

    def test_a_newly_world_reachable_sensitive_port_raises_the_banner(self):
        self.w._on_listeners_opened([_listener(port=6379, process="redis-server")])
        self.assertIn("SENSITIVE PORT", self.w.dash.alert_lbl.text())
        self.assertIn("PORT OPENED", self.w.dash.log.toPlainText())

    def test_a_loopback_port_opening_is_logged_but_not_alarming(self):
        self.w._on_listeners_opened([_listener("127.0.0.1", 6379, process="redis-server")])
        self.assertIn("PORT OPENED", self.w.dash.log.toPlainText())
        self.assertEqual(self.w._alerts, 0)

    def test_a_closed_port_is_logged(self):
        self.w._on_listeners_closed([_listener(port=8080, process="python3")])
        self.assertIn("port closed", self.w.dash.log.toPlainText())


# --------------------------------------------------------------------------
# 0.14.0: per-contact bytes, attribution honesty, socket health, session
# accounting, per-process baseline, beacon detection, forward-confirmed DNS


class TestSplitEndpoint(unittest.TestCase):
    def test_plain_v4(self):
        self.assertEqual(bit.split_endpoint("1.2.3.4:443"), ("1.2.3.4", 443))

    def test_bracketed_v6(self):
        self.assertEqual(bit.split_endpoint("[2001:db8::1]:443"), ("2001:db8::1", 443))

    def test_v4_mapped_onto_a_v6_socket_comes_back_as_v4(self):
        self.assertEqual(bit.split_endpoint("[::ffff:94.19.200.5]:33008"),
                         ("94.19.200.5", 33008))

    def test_garbage_does_not_raise(self):
        for raw in ("", "nonsense", ":", "[unterminated"):
            self.assertIsInstance(bit.split_endpoint(raw), tuple)


class TestPerContactBytes(unittest.TestCase):
    SS = (
        'ESTAB 0 0 192.168.1.5:40994 [::ffff:203.0.113.9]:443 '
        'users:(("firefox",pid=1,fd=9))\n'
        '\t cubic bytes_sent:100 bytes_received:900\n'
        'ESTAB 0 0 192.168.1.5:40995 203.0.113.9:443 '
        'users:(("firefox",pid=1,fd=10))\n'
        '\t cubic bytes_sent:10 bytes_received:90\n'
    )

    def _sampler(self):
        s = bit.ProcTrafficSampler()
        s.enabled = True
        with mock.patch.object(s, "_run", return_value=""):
            s.sample()  # prime, so the next pass counts in full
        return s

    def test_the_peer_key_matches_a_contact_key(self):
        self.assertEqual(
            bit.ProcTrafficSampler.peer_key("192.168.1.5:40994|[::ffff:203.0.113.9]:443"),
            "tcp:203.0.113.9:443")

    def test_bytes_are_bucketed_by_endpoint(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=self.SS):
            s.sample()
        # Two sockets to the same endpoint, one of them v4-mapped: one bucket.
        self.assertEqual(s.peer_totals(), {"tcp:203.0.113.9:443": (990, 110)})

    def test_process_and_peer_totals_agree(self):
        s = self._sampler()
        with mock.patch.object(s, "_run", return_value=self.SS):
            totals = s.sample()
        self.assertEqual(totals["firefox"], (990, 110))

    def test_peer_totals_are_capped(self):
        s = self._sampler()
        s.MAX_PEERS = 3
        rows = "".join(
            f'ESTAB 0 0 10.0.0.1:{1000 + i} 203.0.113.{i}:443\n'
            f'\t cubic bytes_sent:{i + 1} bytes_received:{i + 1}\n'
            for i in range(10))
        with mock.patch.object(s, "_run", return_value=rows):
            s.sample()
        self.assertLessEqual(len(s.peer_totals()), 3)

    def test_the_table_shows_them(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                         "ts": time.time()}])
        self.assertEqual(tab.model.item(0, tab.C_RX).text(), "—")
        tab.set_peer_io({"tcp:203.0.113.9:443": (2048, 1024)})
        self.assertEqual(tab.model.item(0, tab.C_RX).text(), "2.0 KB")
        self.assertEqual(tab.model.item(0, tab.C_TX).text(), "1.0 KB")
        tab.deleteLater()

    def test_byte_cells_sort_numerically_not_alphabetically(self):
        self.assertLess(bit.ConnectionsTab._bytes_cell(900)[1],
                        bit.ConnectionsTab._bytes_cell(2048)[1])
        self.assertEqual(bit.ConnectionsTab._bytes_cell(900)[0], "900.0 B")

    def test_a_figure_survives_the_endpoint_being_seen_again(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        point = {"ip": "203.0.113.9", "port": 443, "proto": "tcp", "ts": time.time()}
        tab.add_points([point])
        tab.set_peer_io({"tcp:203.0.113.9:443": (2048, 1024)})
        tab.add_points([dict(point, ts=time.time() + 30)])
        self.assertEqual(tab.model.item(0, tab.C_RX).text(), "2.0 KB")
        tab.deleteLater()

    def test_an_unknown_endpoint_is_ignored(self):
        tab = bit.ConnectionsTab()
        tab.set_peer_io({"tcp:198.51.100.1:443": (1, 1)})  # must not raise
        self.assertEqual(tab.model.rowCount(), 0)
        tab.deleteLater()

    def test_the_figures_reach_the_json_export(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                         "ts": time.time()}])
        tab.set_peer_io({"tcp:203.0.113.9:443": (2048, 1024)})
        tmp = tempfile.mkdtemp(prefix="blackice-bytes-")
        try:
            path = os.path.join(tmp, "c.json")
            tab.export_json(path)
            with open(path, encoding="utf-8") as f:
                rows = json.load(f)
            self.assertEqual(rows[0]["rx_bytes"], 2048)
            self.assertEqual(rows[0]["tx_bytes"], 1024)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
            tab.deleteLater()


class TestAttributionHonesty(unittest.TestCase):
    Addr = collections.namedtuple("Addr", "ip port")
    Conn = collections.namedtuple("Conn", "fd family type laddr raddr status pid")

    def test_unprivileged_scope_names_the_blind_spot(self):
        self.assertEqual(
            bit.attribution_summary({"remote": 100, "unattributed": 45}),
            "own processes (45% unattributed)")

    def test_root_sees_everything(self):
        self.assertEqual(
            bit.attribution_summary({"remote": 100, "unattributed": 45,
                                     "elevated": 1}),
            "all processes (root)")

    def test_nothing_hidden_means_no_parenthetical(self):
        self.assertEqual(
            bit.attribution_summary({"remote": 10, "unattributed": 0}),
            "own processes")

    def test_an_empty_scan_does_not_divide_by_zero(self):
        self.assertEqual(bit.attribution_summary({}), "own processes")

    def test_the_ratio_covers_the_whole_table_not_just_new_contacts(self):
        """Counted after the dedupe, the figure jumped around with whatever
        handful of endpoints happened to be new that scan."""
        conns = [
            self.Conn(i, 2, 1, self.Addr("0.0.0.0", 0),
                      self.Addr("203.0.113.%d" % i, 443), "ESTABLISHED",
                      None if i % 2 else 7)
            for i in range(1, 11)]
        proc = mock.Mock()
        proc.name.return_value = "curl"
        scanner = bit.ConnScanner()
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns), \
             mock.patch.object(bit.psutil, "Process", return_value=proc):
            scanner._scan_psutil()
            first = dict(scanner.scan_stats)
            scanner._scan_psutil()   # everything deduped away this time
            second = dict(scanner.scan_stats)
        self.assertEqual(first["remote"], 10)
        self.assertEqual(first["unattributed"], 5)
        self.assertEqual(second, first)

    def test_the_scanner_counts_what_it_could_not_attribute(self):
        conns = [
            self.Conn(1, 2, 1, self.Addr("0.0.0.0", 0),
                      self.Addr("203.0.113.1", 443), "ESTABLISHED", None),
            self.Conn(2, 2, 1, self.Addr("0.0.0.0", 0),
                      self.Addr("203.0.113.2", 443), "ESTABLISHED", 7),
        ]
        proc = mock.Mock()
        proc.name.return_value = "curl"
        scanner = bit.ConnScanner()
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns), \
             mock.patch.object(bit.psutil, "Process", return_value=proc):
            scanner._scan_psutil()
        self.assertEqual(scanner.scan_stats["remote"], 2)
        self.assertEqual(scanner.scan_stats["unattributed"], 1)

    def test_the_window_reports_it_once(self):
        tmp = tempfile.mkdtemp(prefix="blackice-scope-")
        try:
            w = bit.MainWindow(config=bit.AppConfig(QtCore.QSettings(
                os.path.join(tmp, "c.ini"), QtCore.QSettings.Format.IniFormat)))
            w._on_scan_stats({"remote": 100, "unattributed": 45})
            w._on_scan_stats({"remote": 100, "unattributed": 45})
            self.assertIn("SCOPE: own processes (45% unattributed)", w.dash.sub.text())
            log = w.dash.log.toPlainText()
            self.assertEqual(log.count("have no owning process"), 1)
            self.assertIn("needs root", log)
            w.close()
            w.deleteLater()
            _app.processEvents()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestSocketHealthMonitor(unittest.TestCase):
    def setUp(self):
        self.m = bit.SocketHealthMonitor()

    def _feed(self, state, values, now=0.0):
        """Collect every report across the series — the monitor speaks up as
        soon as it has grounds to, not only on the last sample."""
        out = []
        for v in values:
            out.extend(self.m.feed({state: v}, now=now))
        return out

    def test_a_climbing_pile_of_close_wait_is_reported(self):
        out = self._feed("CLOSE_WAIT", [10, 20, 30, 40, 60])
        self.assertTrue(out)
        self.assertIn("CLOSE_WAIT", out[0])
        self.assertIn("not closing its sockets", out[0])

    def test_a_high_but_flat_level_is_not_news(self):
        self.assertEqual(self._feed("CLOSE_WAIT", [200] * 6), [])

    def test_a_climb_below_the_threshold_is_not_news(self):
        self.assertEqual(self._feed("CLOSE_WAIT", [1, 2, 3, 4, 5]), [])

    def test_too_few_samples_to_judge(self):
        self.assertEqual(self._feed("CLOSE_WAIT", [10, 200]), [])

    def test_syn_sent_explains_itself_differently(self):
        out = self._feed("SYN_SENT", [1, 4, 9, 14, 30])
        self.assertIn("not completing", out[0])

    def test_it_does_not_repeat_within_the_window(self):
        self._feed("CLOSE_WAIT", [10, 20, 30, 40, 60])
        self.assertEqual(self.m.feed({"CLOSE_WAIT": 80}, now=10.0), [])

    def test_it_speaks_again_once_the_window_passes(self):
        self._feed("CLOSE_WAIT", [10, 20, 30, 40, 60])
        later = bit.SocketHealthMonitor.REPEAT_SECONDS + 1
        self.assertTrue(self.m.feed({"CLOSE_WAIT": 90}, now=later))

    def test_ordinary_states_are_never_reported(self):
        self.assertEqual(self._feed("TIME_WAIT", [10, 500, 2000, 9000, 20000]), [])


class TestSessionAccounting(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="blackice-sessions-")
        self.path = os.path.join(self.tmp, "history.db")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_a_run_is_recorded_and_read_back(self):
        store = bit.SessionStore(self.path)
        store.open()
        store.close()
        store.open()
        summary = store.sessions_summary()
        self.assertEqual(summary["sessions"], 2)
        self.assertGreater(summary["first"], 0)
        store.close()

    def test_watched_time_accumulates(self):
        store = bit.SessionStore(self.path)
        store.open()
        store.touch_session(store._session_started + 3600)
        self.assertAlmostEqual(store.sessions_summary()["watched"], 3600, places=0)
        store.close()

    def test_a_killed_run_still_counts_what_it_watched(self):
        store = bit.SessionStore(self.path)
        store.open()
        store.touch_session(store._session_started + 120)
        store._db.close()          # killed: no clean close
        store._db = None
        reopened = bit.SessionStore(self.path)
        reopened.open()
        self.assertGreaterEqual(reopened.sessions_summary()["watched"], 120)
        reopened.close()

    def test_a_pre_migration_database_is_upgraded_in_place(self):
        db = sqlite3.connect(self.path)
        db.execute("CREATE TABLE sessions (started REAL PRIMARY KEY, version TEXT)")
        db.execute("INSERT INTO sessions VALUES (1000.0, '0.12.0')")
        db.commit()
        db.close()
        store = bit.SessionStore(self.path)
        self.assertTrue(store.open())
        self.assertEqual(store.sessions_summary()["sessions"], 2)
        store.close()

    def test_an_empty_database_reports_zeroes(self):
        store = bit.SessionStore(self.path)
        self.assertEqual(store.sessions_summary(),
                         {"sessions": 0, "first": 0.0, "watched": 0.0})

    def test_durations_read_in_a_sensible_unit(self):
        self.assertEqual(bit.human_duration(42), "42s")
        self.assertEqual(bit.human_duration(300), "5m")
        self.assertEqual(bit.human_duration(5400), "1.5h")
        self.assertEqual(bit.human_duration(172800), "2.0d")

    def test_the_history_tab_shows_it(self):
        store = bit.SessionStore(self.path)
        store.open()
        store.touch_session(store._session_started + 7200)
        tab = bit.HistoryTab(store)
        self.assertIn("watched 2.0h across 1 session(s)", tab.summary.text())
        tab.deleteLater()
        store.close()


class TestProcessBaseline(unittest.TestCase):
    def _baseline(self, hits=100):
        return bit.ProcessBaseline({
            "firefox": {"asn": {"AS13335", "AS16509"}, "country": {"US", "DE"},
                        "hits": hits}})

    def test_a_familiar_network_is_unremarkable(self):
        b = self._baseline()
        self.assertEqual(b.judge({"ip": "8.8.8.8", "process": "firefox",
                                  "asn": "AS13335 CLOUDFLARE", "country": "US"}), [])

    def test_an_unfamiliar_asn_for_that_process_is_flagged(self):
        b = self._baseline()
        hits = b.judge({"ip": "8.8.8.8", "process": "firefox",
                        "asn": "AS12345 SOMETHING", "country": "US"})
        self.assertEqual(hits, ["AS12345 NEW FOR firefox"])

    def test_an_unfamiliar_country_is_flagged_too(self):
        b = self._baseline()
        hits = b.judge({"ip": "8.8.8.8", "process": "firefox",
                        "asn": "AS13335 CLOUDFLARE", "country": "BY"})
        self.assertEqual(hits, ["BY NEW FOR firefox"])

    def test_a_process_with_too_little_history_is_not_judged(self):
        b = self._baseline(hits=3)
        self.assertEqual(b.judge({"ip": "8.8.8.8", "process": "firefox",
                                  "asn": "AS12345 X", "country": "BY"}), [])

    def test_an_unknown_process_is_not_judged(self):
        b = self._baseline()
        self.assertEqual(b.judge({"ip": "8.8.8.8", "process": "curl",
                                  "asn": "AS12345 X"}), [])

    def test_the_lan_is_not_a_behavioural_signal(self):
        b = self._baseline()
        self.assertEqual(b.judge({"ip": "192.168.1.5", "process": "firefox",
                                  "asn": "AS12345 X", "country": "BY"}), [])

    def test_the_org_string_is_not_the_identity(self):
        self.assertEqual(bit.ProcessBaseline.asn_number("AS13335 CLOUDFLARE, INC."),
                         "AS13335")
        self.assertEqual(bit.ProcessBaseline.asn_number("no number here"), "")

    def test_profiles_are_learned_from_the_database(self):
        tmp = tempfile.mkdtemp(prefix="blackice-profile-")
        try:
            store = bit.SessionStore(os.path.join(tmp, "h.db"))
            store.open()
            now = time.time()
            store.record_contacts([
                {"ip": "8.8.8.%d" % i, "port": 443, "proto": "tcp", "ts": now,
                 "process": "firefox", "asn": "AS13335 CLOUDFLARE", "country": "US"}
                for i in range(30)])
            profiles = store.process_profiles()
            self.assertEqual(profiles["firefox"]["asn"], {"AS13335"})
            self.assertEqual(profiles["firefox"]["country"], {"US"})
            self.assertGreaterEqual(profiles["firefox"]["hits"], 30)
            store.close()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_the_table_flags_and_colours_an_anomaly(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.set_process_baseline(self._baseline())
        tab.add_points([{"ip": "8.8.8.8", "port": 443, "proto": "tcp",
                         "ts": time.time(), "process": "firefox",
                         "asn": "AS12345 X", "country": "BY"}])
        self.assertIn("⚠AS12345 NEW FOR firefox",
                      tab.model.item(0, tab.C_FLAGS).text())
        self.assertEqual(tab.model.item(0, tab.C_IP).foreground().color().name(),
                         bit.AMBER.name())
        tab.deleteLater()

    def test_applying_a_baseline_is_retroactive(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "8.8.8.8", "port": 443, "proto": "tcp",
                         "ts": time.time(), "process": "firefox",
                         "asn": "AS12345 X", "country": "BY"}])
        self.assertEqual(tab.model.item(0, tab.C_FLAGS).text(), "")
        tab.set_process_baseline(self._baseline())
        self.assertIn("NEW FOR firefox", tab.model.item(0, tab.C_FLAGS).text())
        tab.deleteLater()

    def test_an_anomaly_is_announced_once_not_every_scan(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.set_process_baseline(self._baseline())
        seen = []
        tab.anomalyNoticed.connect(lambda e, l: seen.append((e, l)))
        point = {"ip": "8.8.8.8", "port": 443, "proto": "tcp",
                 "ts": time.time(), "process": "firefox", "asn": "AS12345 X"}
        for i in range(5):
            tab.add_points([dict(point, ts=time.time() + i * 20)])
        self.assertEqual(len(seen), 1)
        self.assertIn("AS12345 NEW FOR firefox", seen[0][1])
        tab.deleteLater()

    def test_a_watchlist_hit_still_outranks_an_anomaly_in_colour(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.set_process_baseline(self._baseline())
        tab.set_rules(bit.RuleSet.parse("cidr 8.8.8.0/24 GOOGLE DNS\n"))
        tab.add_points([{"ip": "8.8.8.8", "port": 443, "proto": "tcp",
                         "ts": time.time(), "process": "firefox",
                         "asn": "AS12345 X"}])
        self.assertEqual(tab.model.item(0, tab.C_IP).foreground().color().name(),
                         bit.RED.name())
        tab.deleteLater()


class TestBeaconDetector(unittest.TestCase):
    def _series(self, detector, gaps, start=1000.0):
        t = start
        detector.observe("k", t)
        for g in gaps:
            t += g
            detector.observe("k", t)

    def test_a_steady_rhythm_is_measured(self):
        d = bit.BeaconDetector()
        self._series(d, [300.0] * 9)
        verdict = d.verdict("k")
        self.assertAlmostEqual(verdict["period"], 300.0, places=3)
        self.assertEqual(verdict["samples"], 10)
        self.assertIn("BEACON ~300s", bit.BeaconDetector.describe(verdict))

    def test_irregular_traffic_is_not_a_beacon(self):
        d = bit.BeaconDetector()
        self._series(d, [120, 480, 150, 420, 200, 390, 130, 470, 160])
        self.assertIsNone(d.verdict("k"))

    def test_too_few_sightings_to_say(self):
        d = bit.BeaconDetector()
        self._series(d, [300.0] * 3)
        self.assertIsNone(d.verdict("k"))

    def test_a_persistently_open_connection_is_not_reported(self):
        # The scanner re-emits an unbroken connection every ~20s, which is a
        # perfect rhythm that means nothing at all.
        d = bit.BeaconDetector()
        self._series(d, [20.0] * 12)
        self.assertIsNone(d.verdict("k"))

    def test_a_slow_beacon_survives_a_little_jitter(self):
        d = bit.BeaconDetector()
        self._series(d, [300, 302, 298, 301, 299, 300, 303, 297, 300])
        self.assertIsNotNone(d.verdict("k"))

    def test_duplicate_timestamps_are_ignored(self):
        d = bit.BeaconDetector()
        for _ in range(10):
            d.observe("k", 1000.0)
        self.assertIsNone(d.verdict("k"))

    def test_an_unseen_key_has_no_verdict(self):
        self.assertIsNone(bit.BeaconDetector().verdict("nope"))

    def test_tracking_is_bounded(self):
        d = bit.BeaconDetector()
        d.MAX_TRACKED = 5
        for i in range(50):
            d.observe("k%d" % i, 1000.0 + i)
        self.assertLessEqual(len(d._events), 5)

    def test_beacons_lists_only_the_rhythmic_ones(self):
        d = bit.BeaconDetector()
        self._series(d, [300.0] * 9)
        t = 1000.0
        for g in [10, 900, 20, 700]:
            t += g
            d.observe("noisy", t)
        self.assertEqual([k for k, _v in d.beacons()], ["k"])

    def test_the_table_flags_a_beacon(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        base = time.time()
        for i in range(8):
            tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                             "ts": base + i * 300}])
        self.assertIn("◷BEACON ~300s", tab.model.item(0, tab.C_FLAGS).text())
        tab.deleteLater()


class TestForwardConfirmedDns(unittest.TestCase):
    def test_a_name_that_resolves_back_is_confirmed(self):
        infos = [(2, 1, 6, "", ("203.0.113.9", 0))]
        with mock.patch.object(bit.socket, "getaddrinfo", return_value=infos):
            self.assertTrue(bit.RDnsResolver._confirm("203.0.113.9", "a.example"))

    def test_an_anycast_name_with_several_addresses_still_confirms(self):
        infos = [(2, 1, 6, "", ("1.0.0.1", 0)), (2, 1, 6, "", ("1.1.1.1", 0))]
        with mock.patch.object(bit.socket, "getaddrinfo", return_value=infos):
            self.assertTrue(bit.RDnsResolver._confirm("1.1.1.1", "one.one.one.one"))

    def test_a_name_that_points_elsewhere_is_not_confirmed(self):
        infos = [(2, 1, 6, "", ("198.51.100.1", 0))]
        with mock.patch.object(bit.socket, "getaddrinfo", return_value=infos):
            self.assertFalse(bit.RDnsResolver._confirm("203.0.113.9", "liar.example"))

    def test_a_v4_mapped_answer_still_matches(self):
        infos = [(10, 1, 6, "", ("::ffff:203.0.113.9", 0, 0, 0))]
        with mock.patch.object(bit.socket, "getaddrinfo", return_value=infos):
            self.assertTrue(bit.RDnsResolver._confirm("203.0.113.9", "a.example"))

    def test_a_failed_forward_lookup_is_not_a_confirmation(self):
        with mock.patch.object(bit.socket, "getaddrinfo",
                               side_effect=socket.gaierror):
            self.assertFalse(bit.RDnsResolver._confirm("203.0.113.9", "a.example"))

    def test_nothing_to_check_is_not_a_confirmation(self):
        self.assertFalse(bit.RDnsResolver._confirm("203.0.113.9", ""))

    def test_the_table_flags_an_unconfirmed_name(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                         "ts": time.time(), "host": "liar.example"}])
        self.assertEqual(tab.model.item(0, tab.C_FLAGS).text(), "")
        tab.set_fcrdns("203.0.113.9", False)
        self.assertIn(bit.ConnectionsTab.FLAG_UNCONFIRMED,
                      tab.model.item(0, tab.C_FLAGS).text())
        tab.deleteLater()

    def test_a_confirmed_name_carries_no_flag(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                         "ts": time.time(), "host": "honest.example"}])
        tab.set_fcrdns("203.0.113.9", True)
        self.assertEqual(tab.model.item(0, tab.C_FLAGS).text(), "")
        tab.deleteLater()

    def test_an_address_with_no_name_is_never_flagged(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([{"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                         "ts": time.time()}])
        tab.set_fcrdns("203.0.113.9", False)
        self.assertEqual(tab.model.item(0, tab.C_FLAGS).text(), "")
        tab.deleteLater()

    def test_the_verdict_sticks_when_the_endpoint_is_seen_again(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        point = {"ip": "203.0.113.9", "port": 443, "proto": "tcp",
                 "ts": time.time(), "host": "liar.example"}
        tab.add_points([point])
        tab.set_fcrdns("203.0.113.9", False)
        tab.add_points([dict(point, ts=time.time() + 30)])
        self.assertIn(bit.ConnectionsTab.FLAG_UNCONFIRMED,
                      tab.model.item(0, tab.C_FLAGS).text())
        tab.deleteLater()


# --------------------------------------------------------------------------
# 0.15.0: tray, --watch, freeze, scan interval, drill-down, firewall rules


class TestFirewallRules(unittest.TestCase):
    def test_nftables_v4(self):
        self.assertEqual(
            bit.firewall_rule("nftables", "203.0.113.9", 443, "tcp"),
            "nft add rule inet filter output ip daddr 203.0.113.9 "
            "tcp dport 443 counter drop")

    def test_nftables_switches_family_for_v6(self):
        self.assertIn("ip6 daddr 2001:db8::1",
                      bit.firewall_rule("nftables", "2001:db8::1", 443))

    def test_iptables_v4(self):
        self.assertEqual(
            bit.firewall_rule("iptables", "203.0.113.9", 443, "tcp"),
            "iptables -A OUTPUT -p tcp -d 203.0.113.9 --dport 443 -j DROP")

    def test_iptables_switches_binary_for_v6(self):
        self.assertTrue(
            bit.firewall_rule("iptables", "2001:db8::1", 53, "udp")
            .startswith("ip6tables "))

    def test_ufw_v4(self):
        self.assertEqual(bit.firewall_rule("ufw", "203.0.113.9", 443, "tcp"),
                         "ufw deny out to 203.0.113.9 port 443 proto tcp")

    def test_no_port_means_the_whole_host(self):
        self.assertEqual(bit.firewall_rule("ufw", "203.0.113.9"),
                         "ufw deny out to 203.0.113.9")
        self.assertNotIn("dport", bit.firewall_rule("nftables", "203.0.113.9"))

    def test_udp_is_carried_through(self):
        self.assertIn("proto udp", bit.firewall_rule("ufw", "203.0.113.9", 53, "udp"))

    def test_a_v4_mapped_address_is_normalised(self):
        self.assertIn("203.0.113.9",
                      bit.firewall_rule("ufw", "::ffff:203.0.113.9", 443))

    def test_an_unparseable_address_yields_no_rule(self):
        for syntax in bit.FIREWALL_SYNTAXES:
            self.assertEqual(bit.firewall_rule(syntax, "not-an-ip", 443), "")

    def test_an_unknown_syntax_yields_no_rule(self):
        self.assertEqual(bit.firewall_rule("pf", "203.0.113.9", 443), "")

    def test_the_context_menu_offers_every_syntax(self):
        tab = bit.ConnectionsTab()
        menu = tab.build_menu(_contact())
        labels = []
        for action in menu.actions():
            if action.menu():
                labels = [a.text() for a in action.menu().actions()]
        self.assertEqual(len(labels), len(bit.FIREWALL_SYNTAXES))
        self.assertTrue(any(l.startswith("nftables: nft add rule") for l in labels))
        tab.deleteLater()


class TestFreezeView(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="blackice-freeze-")
        self.w = bit.MainWindow(
            config=bit.AppConfig(QtCore.QSettings(
                os.path.join(self.tmp, "c.ini"), QtCore.QSettings.Format.IniFormat)),
            store=bit.SessionStore(os.path.join(self.tmp, "h.db")))

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        _app.processEvents()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_the_table_stops_moving(self):
        self.w.dash.freeze_btn.setChecked(True)
        self.w._on_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self.w.conns.model.rowCount(), 0)

    def test_and_moves_again_on_release(self):
        self.w.dash.freeze_btn.setChecked(True)
        self.w._on_points([_contact(ip="8.8.8.8")])
        self.w.dash.freeze_btn.setChecked(False)
        self.w._on_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self.w.conns.model.rowCount(), 1)

    def test_recording_carries_on_underneath(self):
        self.w.dash.history_btn.setChecked(True)
        self.w.dash.freeze_btn.setChecked(True)
        self.w._on_points([_contact(ip="8.8.8.8")])
        self.assertEqual(self.w.store.counts()[0], 1)

    def test_the_rhythm_detector_is_still_fed(self):
        # A gap caused by the user holding the view still would otherwise
        # destroy a beacon measurement.
        self.w.dash.freeze_btn.setChecked(True)
        base = time.time()
        for i in range(8):
            self.w._on_points([_contact(ip="8.8.8.8", ts=base + i * 300)])
        self.assertIsNotNone(self.w.conns.beacons.verdict("tcp:8.8.8.8:443"))

    def test_socket_health_is_still_measured(self):
        self.w.dash.freeze_btn.setChecked(True)
        for n in (10, 20, 30, 40, 60):
            self.w._on_states({"CLOSE_WAIT": n})
        self.assertIn("CLOSE_WAIT", self.w.dash.log.toPlainText())

    def test_listeners_opened_during_a_freeze_are_reported_on_release(self):
        self.w._on_listeners([_listener(port=22)])
        self.w.dash.freeze_btn.setChecked(True)
        self.w._on_listeners([_listener(port=22), _listener(port=6379)])
        self.assertNotIn("PORT OPENED", self.w.dash.log.toPlainText())
        self.w.dash.freeze_btn.setChecked(False)
        self.w._on_listeners([_listener(port=22), _listener(port=6379)])
        self.assertIn("PORT OPENED", self.w.dash.log.toPlainText())

    def test_the_status_line_says_so(self):
        self.w.dash.freeze_btn.setChecked(True)
        self.assertIn("VIEW: FROZEN", self.w.dash.sub.text())
        self.w.dash.freeze_btn.setChecked(False)
        self.assertIn("VIEW: LIVE", self.w.dash.sub.text())

    def test_freeze_is_not_restored_on_the_next_run(self):
        self.w.dash.config_btn.setChecked(True)
        self.w.dash.freeze_btn.setChecked(True)
        self.w.save_state()
        self.assertIsNone(self.w.cfg.get("dash/frozen", None))


class TestScanInterval(unittest.TestCase):
    def test_the_scanner_accepts_a_new_cadence(self):
        scanner = bit.ConnScanner(interval=3.0)
        scanner.set_interval(30.0)
        self.assertEqual(scanner.interval, 30.0)

    def test_a_silly_interval_is_clamped(self):
        scanner = bit.ConnScanner()
        scanner.set_interval(0.0)
        self.assertGreaterEqual(scanner.interval, 0.2)

    def test_the_poller_accepts_one_too(self):
        poller = bit.TrafficPoller()
        poller.set_interval(5.0)
        self.assertEqual(poller.interval, 5.0)

    def test_the_combo_offers_the_documented_choices(self):
        d = bit.BlackIceDashboard()
        self.assertEqual(
            [d.scan_combo.itemData(i) for i in range(d.scan_combo.count())],
            list(bit.BlackIceDashboard.SCAN_INTERVALS))
        self.assertEqual(d.scan_interval(), 3.0)
        d.deleteLater()

    def test_setting_it_back_does_not_re_emit(self):
        d = bit.BlackIceDashboard()
        seen = []
        d.scanIntervalChanged.connect(seen.append)
        d.set_scan_interval(10.0)
        self.assertEqual(d.scan_interval(), 10.0)
        self.assertEqual(seen, [])
        d.deleteLater()

    def test_choosing_one_reaches_the_scanner(self):
        tmp = tempfile.mkdtemp(prefix="blackice-interval-")
        try:
            w = bit.MainWindow(config=bit.AppConfig(QtCore.QSettings(
                os.path.join(tmp, "c.ini"), QtCore.QSettings.Format.IniFormat)))
            w.dash.set_scan_interval(3.0)
            w.dash.scan_combo.setCurrentIndex(
                list(bit.BlackIceDashboard.SCAN_INTERVALS).index(10.0))
            self.assertEqual(w.scanner.interval, 10.0)
            w.close()
            w.deleteLater()
            _app.processEvents()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestDrillDown(unittest.TestCase):
    def test_a_bar_is_found_under_its_own_pixels(self):
        panel = bit.RollupPanel("T")
        panel.resize(300, 140)
        panel.set_rows([("firefox", 10), ("curl", 5)])
        self.assertEqual(panel.row_at(QtCore.QPoint(50, panel.ROW_TOP + 2)), "firefox")
        self.assertEqual(
            panel.row_at(QtCore.QPoint(50, panel.ROW_TOP + panel.ROW_STEP + 2)), "curl")
        panel.deleteLater()

    def test_the_gap_between_bars_is_not_a_bar(self):
        panel = bit.RollupPanel("T")
        panel.resize(300, 140)
        panel.set_rows([("firefox", 10), ("curl", 5)])
        gap = panel.ROW_TOP + panel.ROW_H + 1
        self.assertIsNone(panel.row_at(QtCore.QPoint(50, gap)))
        panel.deleteLater()

    def test_clicking_past_the_last_bar_finds_nothing(self):
        panel = bit.RollupPanel("T")
        panel.resize(300, 140)
        panel.set_rows([("firefox", 10)])
        self.assertIsNone(panel.row_at(QtCore.QPoint(50, 130)))
        self.assertIsNone(panel.row_at(QtCore.QPoint(50, 0)))
        panel.deleteLater()

    def test_stats_reports_which_rollup_was_clicked(self):
        tab = bit.StatsTab()
        seen = []
        tab.drillDown.connect(lambda f, v: seen.append((f, v)))
        tab.proc_panel.rowActivated.emit("firefox")
        tab.asn_panel.rowActivated.emit("AS13335 CLOUDFLARE")
        self.assertEqual(seen, [("process", "firefox"),
                                ("asn", "AS13335 CLOUDFLARE")])
        tab.deleteLater()

    def test_filtering_to_a_column_ignores_matches_elsewhere(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([_contact(ip="8.8.8.8", process="curl"),
                        _contact(ip="1.1.1.1", port=80, process="firefox",
                                 host="curl.example")])
        tab.filter_to("curl", tab.C_PROCESS)
        self.assertEqual(tab.proxy.rowCount(), 1)
        tab.deleteLater()

    def test_a_country_code_matches_as_a_whole_word(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([_contact(ip="8.8.8.8", location="Dresden DE"),
                        _contact(ip="1.1.1.1", port=80, location="Delhi IN")])
        tab.filter_to("DE", tab.C_LOCATION, whole_word=True)
        self.assertEqual(tab.proxy.rowCount(), 1)
        tab.deleteLater()

    def test_typing_in_the_box_searches_every_column_again(self):
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        tab.add_points([_contact(ip="8.8.8.8", process="curl")])
        tab.filter_to("curl", tab.C_PROCESS)
        tab.filter_edit.setText("8.8.8.8")
        self.assertEqual(tab.proxy.filterKeyColumn(), -1)
        self.assertEqual(tab.proxy.rowCount(), 1)
        tab.deleteLater()

    def test_the_window_switches_to_the_contacts_tab(self):
        tmp = tempfile.mkdtemp(prefix="blackice-drill-")
        try:
            w = bit.MainWindow(config=bit.AppConfig(QtCore.QSettings(
                os.path.join(tmp, "c.ini"), QtCore.QSettings.Format.IniFormat)))
            w.tabs.setCurrentWidget(w.stats)
            w.stats.drillDown.emit("process", "firefox")
            self.assertIs(w.tabs.currentWidget(), w.conns)
            self.assertEqual(w.conns.filter_edit.text(), "firefox")
            self.assertEqual(w.conns.proxy.filterKeyColumn(),
                             bit.ConnectionsTab.C_PROCESS)
            w.close()
            w.deleteLater()
            _app.processEvents()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestTray(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="blackice-tray-")
        self.w = bit.MainWindow(config=bit.AppConfig(QtCore.QSettings(
            os.path.join(self.tmp, "c.ini"), QtCore.QSettings.Format.IniFormat)))

    def tearDown(self):
        self.w._quitting = True
        self.w.close()
        self.w.deleteLater()
        _app.processEvents()
        QtWidgets.QApplication.instance().setQuitOnLastWindowClosed(True)
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_an_icon_is_always_produced(self):
        self.assertFalse(bit.tray_icon().isNull())
        self.assertFalse(bit.tray_icon(alert=True).isNull())

    def test_an_alert_icon_differs_from_the_quiet_one(self):
        quiet = bit.tray_icon(False).pixmap(32, 32).toImage()
        loud = bit.tray_icon(True).pixmap(32, 32).toImage()
        self.assertNotEqual(quiet, loud)

    def test_it_declines_politely_without_a_system_tray(self):
        with mock.patch.object(QtWidgets.QSystemTrayIcon,
                               "isSystemTrayAvailable", return_value=False):
            self.w.dash.tray_btn.setChecked(True)
        self.assertIsNone(self.w.tray)
        self.assertFalse(self.w.dash.tray_btn.isChecked())
        self.assertIn("no system tray", self.w.dash.log.toPlainText())

    def _enable(self):
        with mock.patch.object(QtWidgets.QSystemTrayIcon,
                               "isSystemTrayAvailable", return_value=True):
            self.w.dash.tray_btn.setChecked(True)

    def test_enabling_it_creates_the_icon(self):
        self._enable()
        self.assertIsNotNone(self.w.tray)
        self.assertFalse(
            QtWidgets.QApplication.instance().quitOnLastWindowClosed())

    def test_closing_the_window_hides_it_instead_of_quitting(self):
        self._enable()
        self.w.show()
        self.w.close()
        self.assertTrue(self.w.isHidden())
        self.assertTrue(self.w.scanner.isRunning())

    def test_quitting_from_the_tray_really_stops(self):
        self._enable()
        with mock.patch.object(QtWidgets.QApplication, "quit"):
            self.w._tray_quit()
        self.assertIsNone(self.w.tray)
        self.assertFalse(self.w.scanner.isRunning())

    def test_turning_it_off_restores_ordinary_closing(self):
        self._enable()
        self.w.dash.tray_btn.setChecked(False)
        self.assertIsNone(self.w.tray)
        self.assertTrue(
            QtWidgets.QApplication.instance().quitOnLastWindowClosed())

    def test_the_tooltip_counts_what_is_flagged(self):
        self._enable()
        self.w._on_alert("203.0.113.9:443", "DOCS")
        self.w._refresh_tray()
        self.assertIn("1 flagged", self.w.tray.toolTip())

    def test_the_preference_is_remembered(self):
        self.w.dash.config_btn.setChecked(True)
        self._enable()
        self.w.save_state()
        self.assertTrue(self.w.cfg.get("window/tray", False, bool))


class TestWatchMode(unittest.TestCase):
    Addr = collections.namedtuple("Addr", "ip port")
    Conn = collections.namedtuple("Conn", "fd family type laddr raddr status pid")

    def _args(self, *extra):
        return bit.build_arg_parser().parse_args(
            ["--watch", "--duration", "0.01", "--interval", "0"] + list(extra))

    def _run(self, conns, rules_text="", notify=False):
        tmp = tempfile.mkdtemp(prefix="blackice-watch-")
        try:
            wl = os.path.join(tmp, "w.txt")
            with open(wl, "w", encoding="utf-8") as f:
                f.write(rules_text)
            out, err = io.StringIO(), io.StringIO()
            extra = ["--watchlist", wl] + (["--notify"] if notify else [])
            with mock.patch.object(bit.psutil, "net_connections", return_value=conns), \
                 mock.patch.object(bit.ProcTrafficSampler, "available",
                                   return_value=False), \
                 mock.patch("sys.stdout", new=out), mock.patch("sys.stderr", new=err):
                rc = bit.run_headless(self._args(*extra))
            return rc, out.getvalue() + err.getvalue()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def _contact_conn(self):
        return self.Conn(1, 2, 1, self.Addr("0.0.0.0", 0),
                         self.Addr("8.8.8.8", 443), "ESTABLISHED", None)

    def test_a_quiet_scan_exits_zero(self):
        rc, text = self._run([self._contact_conn()])
        self.assertEqual(rc, bit.EXIT_OK)
        self.assertIn("nothing flagged", text)

    def test_a_watchlist_hit_sets_its_own_code(self):
        rc, text = self._run([self._contact_conn()], "cidr 8.8.8.0/24 GOOGLE\n")
        self.assertEqual(rc, bit.EXIT_WATCHLIST)
        self.assertIn("WATCHLIST", text)

    def test_the_codes_are_distinct_and_documented(self):
        self.assertEqual(
            sorted({bit.EXIT_OK, bit.EXIT_ERROR, bit.EXIT_WATCHLIST,
                    bit.EXIT_EXPOSURE}),
            [0, 2, 3, 4])

    def test_an_exposed_port_outranks_a_watchlist_hit(self):
        rules = bit.RuleSet.parse("cidr 8.8.8.0/24 GOOGLE\n")
        out = io.StringIO()
        verdict = bit._watch_report(
            [{"ip": "8.8.8.8", "port": 443}],
            [_listener(port=6379, process="redis-server")],
            rules, False, out)
        self.assertEqual(verdict, bit.EXIT_EXPOSURE)
        self.assertIn("EXPOSURE", out.getvalue())
        self.assertIn("WATCHLIST", out.getvalue())

    def test_a_harmless_new_port_is_not_an_exposure(self):
        out = io.StringIO()
        verdict = bit._watch_report([], [_listener(port=8080, process="python3")],
                                    bit.RuleSet(), False, out)
        self.assertEqual(verdict, bit.EXIT_OK)

    def test_a_loopback_port_is_not_an_exposure(self):
        out = io.StringIO()
        verdict = bit._watch_report(
            [], [_listener("127.0.0.1", 6379, process="redis-server")],
            bit.RuleSet(), False, out)
        self.assertEqual(verdict, bit.EXIT_OK)

    def test_it_notifies_when_asked(self):
        out = io.StringIO()
        with mock.patch.object(bit, "notify", return_value=True) as sent:
            bit._watch_report([], [_listener(port=6379, process="redis-server")],
                              bit.RuleSet(), True, out)
        self.assertTrue(sent.called)
        self.assertIn("port exposed", sent.call_args[0][0])

    def test_it_stays_quiet_when_not_asked(self):
        out = io.StringIO()
        with mock.patch.object(bit, "notify") as sent:
            bit._watch_report([], [_listener(port=6379, process="redis-server")],
                              bit.RuleSet(), False, out)
        self.assertFalse(sent.called)

    def test_notify_without_notify_send_is_not_an_error(self):
        with mock.patch.object(bit.shutil, "which", return_value=None):
            self.assertFalse(bit.notify("t", "b"))

    def test_notify_survives_a_broken_notify_send(self):
        with mock.patch.object(bit.shutil, "which", return_value="/usr/bin/notify-send"), \
             mock.patch.object(bit.subprocess, "run", side_effect=OSError):
            self.assertFalse(bit.notify("t", "b"))

    def test_watch_implies_headless(self):
        with mock.patch.object(bit, "run_headless", return_value=0) as run, \
             mock.patch.object(bit, "init_geoip"), mock.patch.object(bit, "init_asn"):
            with self.assertRaises(SystemExit):
                bit.main(["--watch", "--duration", "0"])
        self.assertTrue(run.called)


if __name__ == "__main__":
    unittest.main()
