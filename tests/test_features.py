"""Tests for the 0.10.0 feature round: listeners, socket states, reverse DNS,
history, watchlist rules, rollups, the offline map, the CLI and the FX switch.

Run from the project root with the runtime deps installed (PyQt6, psutil):

    QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests
"""
import collections
import csv
import json
import os
import re
import shutil
import struct
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
                         ["BLACK ICE", "CONTACTS", "LISTEN", "MAP", "STATS"])
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
        self.assertIn("WATCHLIST HIT", w.dash.alert_lbl.text())
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
            "dash/hide_virtual", "stats/iface", "stats/window", "contacts/header",
            "contacts/sort_col", "contacts/sort_order", "contacts/filter",
            "map/splitter", "map/show_me", "map/mode", "history/enabled",
            "listen/world_only", "listen/header"})


if __name__ == "__main__":
    unittest.main()
