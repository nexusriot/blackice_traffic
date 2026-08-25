"""Offscreen UI tests: locate-and-focus feature, contact-table sorting and
the location helpers.

Run from the project root with the runtime deps installed (PyQt6, psutil):

    QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests
"""
import collections
import csv
import os
import shutil
import socket
import sys
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6 import QtCore, QtWidgets
from PyQt6.QtCore import Qt

import blackice_traffic as bit

_app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])


class TestIpSortKey(unittest.TestCase):
    def test_v4_numeric_order(self):
        self.assertLess(bit.ip_sort_key("9.0.0.1"), bit.ip_sort_key("10.0.0.1"))
        self.assertLess(bit.ip_sort_key("1.2.3.4"), bit.ip_sort_key("1.2.3.5"))

    def test_v4_sorts_before_v6(self):
        self.assertLess(bit.ip_sort_key("255.255.255.255"), bit.ip_sort_key("::1"))

    def test_garbage_sorts_last(self):
        self.assertGreater(bit.ip_sort_key("not-an-ip"), bit.ip_sort_key("2001:db8::1"))


class TestResolveMyLocation(unittest.TestCase):
    def test_online_fallback_used_without_geoip(self):
        with mock.patch.object(bit, "_geoip_reader", None), \
             mock.patch.object(bit, "geo_online_ipapi", return_value=(48.2, 16.4, "Vienna AT")):
            lat, lon, label = bit.resolve_my_location()
        self.assertEqual((lat, lon, label), (48.2, 16.4, "Vienna AT"))

    def test_geoip_path_wins_when_available(self):
        reader = mock.Mock()
        city = reader.city.return_value
        city.location.latitude = 52.5
        city.location.longitude = 13.4
        city.city.name = "Berlin"
        city.country.iso_code = "DE"
        with mock.patch.object(bit, "HAVE_GEOIP", True), \
             mock.patch.object(bit, "_geoip_reader", reader), \
             mock.patch.object(bit, "get_public_ip", return_value="203.0.113.7"), \
             mock.patch.object(bit, "geo_online_ipapi") as online:
            lat, lon, label = bit.resolve_my_location()
        self.assertEqual((lat, lon), (52.5, 13.4))
        self.assertEqual(label, "Berlin DE")
        online.assert_not_called()

    def test_geoip_failure_falls_back_online(self):
        reader = mock.Mock()
        reader.city.side_effect = ValueError("boom")
        with mock.patch.object(bit, "HAVE_GEOIP", True), \
             mock.patch.object(bit, "_geoip_reader", reader), \
             mock.patch.object(bit, "get_public_ip", return_value="203.0.113.7"), \
             mock.patch.object(bit, "geo_online_ipapi", return_value=(1.0, 2.0, "X")):
            lat, lon, label = bit.resolve_my_location()
        self.assertEqual((lat, lon, label), (1.0, 2.0, "X"))

    def test_total_failure_returns_none(self):
        with mock.patch.object(bit, "_geoip_reader", None), \
             mock.patch.object(bit, "geo_online_ipapi", return_value=(None, None, "ME")):
            lat, lon, _ = bit.resolve_my_location()
        self.assertIsNone(lat)
        self.assertIsNone(lon)


class TestGetPublicIp(unittest.TestCase):
    def test_rejects_non_ip_bodies(self):
        resp = mock.MagicMock()
        resp.__enter__.return_value.read.return_value = b"<html>error</html>"
        with mock.patch.object(bit.urllib.request, "urlopen", return_value=resp):
            self.assertIsNone(bit.get_public_ip())

    def test_accepts_valid_ip(self):
        resp = mock.MagicMock()
        resp.__enter__.return_value.read.return_value = b"203.0.113.7\n"
        with mock.patch.object(bit.urllib.request, "urlopen", return_value=resp):
            self.assertEqual(bit.get_public_ip(), "203.0.113.7")


def _point(ip="203.0.113.7", port=443, proto="tcp", hits_ts=None, label=None):
    return {
        "ip": ip, "port": port, "proto": proto,
        "lat": 1.0, "lon": 2.0,
        "label": label or f"{ip}:{port} ({proto}) — Somewhere XX",
        "ts": hits_ts or time.time(),
        "process": "proc", "asn": "AS64500 Example",
    }


class TestConnectionsTabSorting(unittest.TestCase):
    def setUp(self):
        self.tab = bit.ConnectionsTab()

    def _column_values(self, col):
        return [
            self.tab.proxy.index(r, col).data()
            for r in range(self.tab.proxy.rowCount())
        ]

    def test_hits_sort_numeric_not_lexicographic(self):
        self.tab.add_points([_point(ip=f"203.0.113.{i}") for i in range(1, 4)])
        # bump .1 to 10 hits so a string sort would put "10" before "2"
        for _ in range(9):
            self.tab.add_points([_point(ip="203.0.113.1")])
        for _ in range(1):
            self.tab.add_points([_point(ip="203.0.113.2")])
        self.tab.proxy.sort(8, Qt.SortOrder.DescendingOrder)
        hits = [int(v) for v in self._column_values(8)]
        self.assertEqual(hits, sorted(hits, reverse=True))
        self.assertEqual(hits[0], 10)

    def test_port_sort_numeric(self):
        for port in (8080, 443, 53):
            self.tab.add_points([_point(ip=f"203.0.113.{port % 250}", port=port)])
        self.tab.proxy.sort(4, Qt.SortOrder.AscendingOrder)
        ports = self._column_values(4)
        self.assertEqual(ports, ["53 · DNS", "443 · HTTPS", "8080 · HTTP-ALT"])

    def test_ip_sort_numeric(self):
        for ip in ("203.0.113.10", "203.0.113.9", "9.9.9.9"):
            self.tab.add_points([_point(ip=ip)])
        self.tab.proxy.sort(3, Qt.SortOrder.AscendingOrder)
        self.assertEqual(
            self._column_values(3),
            ["9.9.9.9", "203.0.113.9", "203.0.113.10"],
        )

    def test_dedupe_increments_hits(self):
        self.tab.add_points([_point()])
        self.tab.add_points([_point()])
        self.assertEqual(self.tab.model.rowCount(), 1)
        self.assertEqual(self.tab.model.item(0, 8).text(), "2")


class TestMapTabLocate(unittest.TestCase):
    def setUp(self):
        self.tab = bit.MapTab()
        self.js_calls = []
        self.tab._js = self.js_calls.append

    def _run_locate(self, focus, result=(50.1, 14.4, "Prague CZ")):
        # Run the worker body synchronously instead of spawning the thread.
        with mock.patch.object(bit.threading, "Thread") as thread_cls, \
             mock.patch.object(bit, "resolve_my_location", return_value=result):
            self.tab.locate_me(focus=focus)
            self.assertFalse(self.tab.me_refresh.isEnabled())
            self.assertFalse(self.tab.focus_btn.isEnabled())
            thread_cls.assert_called_once()
            self.tab._locate_worker()

    def test_locate_and_focus_flies_map(self):
        self._run_locate(focus=True)
        joined = "\n".join(self.js_calls)
        self.assertIn("focusMe(8)", joined)
        self.assertIn("setMyLocation", joined)
        self.assertTrue(self.tab.me_enable.isChecked())
        self.assertTrue(self.tab.me_refresh.isEnabled())
        self.assertTrue(self.tab.focus_btn.isEnabled())
        self.assertIn("map focused on ME", self.tab.list.toPlainText())

    def test_plain_locate_does_not_move_map(self):
        self._run_locate(focus=False)
        self.assertNotIn("focusMe", "\n".join(self.js_calls))
        self.assertFalse(self.tab.me_enable.isChecked())
        self.assertEqual(self.tab._me_obj["label"], "ME — Prague CZ")

    def test_failure_reenables_buttons(self):
        self._run_locate(focus=True, result=(None, None, "ME"))
        self.assertTrue(self.tab.me_refresh.isEnabled())
        self.assertTrue(self.tab.focus_btn.isEnabled())
        self.assertFalse(self.tab._focus_pending)
        self.assertIn("failed to locate ME", self.tab.list.toPlainText())
        self.assertNotIn("focusMe", "\n".join(self.js_calls))

    def test_second_click_while_locating_is_coalesced(self):
        with mock.patch.object(bit.threading, "Thread") as thread_cls:
            self.tab.locate_me(focus=False)
            self.tab.locate_me(focus=True)
            self.assertEqual(thread_cls.call_count, 1)
        self.assertTrue(self.tab._focus_pending)

    def test_toggle_off_clears_marker(self):
        self._run_locate(focus=True)
        self.js_calls.clear()
        self.tab.me_enable.setChecked(False)
        self.assertIn("clearMe", "\n".join(self.js_calls))


class TestLeafletBridge(unittest.TestCase):
    def test_js_api_exports_focus_and_clear(self):
        for fn in ("clearMe", "focusMe", "setMyLocation", "upsertPoints"):
            self.assertIn(fn, bit.LEAFLET_HTML)
        self.assertIn(
            "window.BLACKICE = { upsertPoints, setMyLocation, redrawRays, clearRays, clearMe, focusMe }",
            bit.LEAFLET_HTML,
        )


class TestConnScannerDedupe(unittest.TestCase):
    """The dedupe key must be built from the normalized address."""

    @staticmethod
    def _conn(ip, port=443, proto=socket.SOCK_STREAM):
        Addr = collections.namedtuple("Addr", "ip port")
        Conn = collections.namedtuple("Conn", "fd family type laddr raddr status pid")
        return Conn(1, socket.AF_INET6, proto, Addr("::", 0), Addr(ip, port), "ESTABLISHED", None)

    def _scan(self, scanner, conns):
        with mock.patch.object(bit.psutil, "net_connections", return_value=conns):
            return scanner._scan_psutil()

    def test_v4_mapped_and_plain_v4_are_one_contact(self):
        s = bit.ConnScanner()
        pts = self._scan(s, [self._conn("::ffff:203.0.113.9"), self._conn("203.0.113.9")])
        self.assertEqual([p.ip for p in pts], ["203.0.113.9"])
        self.assertEqual(list(s._seen), ["tcp:203.0.113.9:443"])

    def test_v4_mapped_loopback_is_filtered(self):
        s = bit.ConnScanner()
        pts = self._scan(s, [self._conn("::ffff:127.0.0.1", 5432), self._conn("::1", 5432)])
        self.assertEqual(pts, [])

    def test_repeat_scan_within_window_is_deduped(self):
        s = bit.ConnScanner()
        self.assertEqual(len(self._scan(s, [self._conn("203.0.113.9")])), 1)
        self.assertEqual(self._scan(s, [self._conn("::ffff:203.0.113.9")]), [])


class TestDashboardIfaceSwitch(unittest.TestCase):
    def setUp(self):
        self.d = bit.BlackIceDashboard()
        self.d.iface.clear()
        self.d.iface.addItems(["ALL", "eth0", "wlan0"])

    @staticmethod
    def _snap(**nics):
        snap = {n: {"rx_bps": rx, "tx_bps": rx, "rx_total": 10, "tx_total": 10,
                    "pkts_in": 1, "pkts_out": 1} for n, rx in nics.items()}
        snap["_totals"] = {"rx_bps": sum(nics.values()), "tx_bps": sum(nics.values())}
        return snap

    def test_switch_drops_previous_nic_samples(self):
        self.d.iface.setCurrentText("eth0")
        for _ in range(5):
            self.d.update_traffic(self._snap(eth0=2_000_000.0))
        self.assertGreater(self.d.scope._max, 1.0)
        self.d.iface.setCurrentText("wlan0")
        self.assertEqual(self.d.scope._max, 1.0)
        self.assertEqual(max(self.d.scope._rx), 0.0)

    def test_missing_nic_keeps_selection(self):
        self.d.iface.setCurrentText("eth0")
        self.d.update_traffic(self._snap(wlan0=1.0))
        self.assertEqual(self.d.iface.currentText(), "eth0")

    def test_all_totals_exclude_loopback(self):
        self.d.iface.setCurrentText("ALL")
        snap = self._snap(eth0=0.0)
        snap["lo"] = {"rx_bps": 0.0, "tx_bps": 0.0, "rx_total": 1024, "tx_total": 1024,
                      "pkts_in": 1, "pkts_out": 1}
        self.d.update_traffic(snap)
        self.assertIn("RX 10.0 B", self.d.totals_lbl.text())


class TestContactsExport(unittest.TestCase):
    def setUp(self):
        self.tab = bit.ConnectionsTab()
        self.tab.add_points([_point(ip="203.0.113.1", port=443),
                             _point(ip="198.51.100.2", port=22),
                             _point(ip="8.8.8.8", port=53)])
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _export(self):
        path = os.path.join(self.tmp, "out.csv")
        self.tab.export_csv(path)
        with open(path, newline="", encoding="utf-8") as f:
            return list(csv.reader(f))

    def test_export_honors_filter(self):
        self.tab.filter_edit.setText("8.8.8.8")
        rows = self._export()
        self.assertEqual(len(rows), 2)  # header + the one visible contact
        self.assertEqual(rows[1][3], "8.8.8.8")

    def test_export_honors_sort_order(self):
        self.tab.proxy.sort(4, Qt.SortOrder.AscendingOrder)
        rows = self._export()
        self.assertEqual([r[4] for r in rows[1:]],
                         ["22 · SSH", "53 · DNS", "443 · HTTPS"])

    def test_export_header_and_full_table_by_default(self):
        rows = self._export()
        self.assertEqual(rows[0], bit.ConnectionsTab.COLS)
        self.assertEqual(len(rows), 4)

    def test_count_label_reports_filtered_and_total(self):
        self.assertEqual(self.tab.count_lbl.text(), "3 contacts")
        self.tab.filter_edit.setText("8.8.8.8")
        self.assertEqual(self.tab.count_lbl.text(), "1 of 3 contacts")
        self.tab.filter_edit.setText("")
        self.assertEqual(self.tab.count_lbl.text(), "3 contacts")

    def test_clear_resets_count(self):
        self.tab._clear()
        self.assertEqual(self.tab.count_lbl.text(), "0 contacts")


class TestMapTabJsQueue(unittest.TestCase):
    """JS issued before the Leaflet page finishes loading must not be lost."""

    def setUp(self):
        self.tab = bit.MapTab()
        self.executed = []
        self.tab._exec_js = self.executed.append
        self.tab._page_ready = False
        self.tab._pending_js = []

    def test_points_before_load_are_queued_then_flushed(self):
        self.tab.push_points([_point()])
        self.assertEqual(self.executed, [])
        self.assertEqual(len(self.tab._pending_js), 1)
        self.tab._on_load_finished(True)
        self.assertEqual(len(self.executed), 1)
        self.assertIn("upsertPoints", self.executed[0])
        self.assertEqual(self.tab._pending_js, [])

    def test_calls_after_load_run_immediately(self):
        self.tab._on_load_finished(True)
        self.tab.push_points([_point()])
        self.assertEqual(len(self.executed), 1)

    def test_failed_load_drops_the_queue(self):
        self.tab.push_points([_point()])
        self.tab._on_load_finished(False)
        self.assertEqual(self.tab._pending_js, [])
        self.assertEqual(self.executed, [])
        self.assertFalse(self.tab._page_ready)
        self.assertIn("failed to load", self.tab.list.toPlainText())

    def test_queue_is_bounded(self):
        for i in range(bit.MapTab.MAX_PENDING_JS + 20):
            self.tab._js(f"call({i});")
        self.assertEqual(len(self.tab._pending_js), bit.MapTab.MAX_PENDING_JS)
        self.assertIn(f"call({bit.MapTab.MAX_PENDING_JS + 19});", self.tab._pending_js[-1])

    def test_trace_log_is_capped(self):
        self.assertEqual(self.tab.list.maximumBlockCount(), 5000)


class TestHistoryGraphRetention(unittest.TestCase):
    def test_stale_interfaces_are_dropped(self):
        g = bit.HistoryGraph()
        now = time.time()
        g.push(now - 3600, {"veth0": {"rx_bps": 1.0, "tx_bps": 1.0}})
        self.assertIn("veth0", g._data)
        g.push(now, {"eth0": {"rx_bps": 1.0, "tx_bps": 1.0}})
        self.assertNotIn("veth0", g._data)
        self.assertIn("eth0", g._data)

    def test_live_interfaces_are_kept(self):
        g = bit.HistoryGraph()
        now = time.time()
        g.push(now - 10, {"eth0": {"rx_bps": 1.0, "tx_bps": 1.0}})
        g.push(now, {"eth0": {"rx_bps": 2.0, "tx_bps": 2.0}})
        self.assertEqual(len(g._data["eth0"]), 2)


class _CfgCase(unittest.TestCase):
    """Base for config tests: an isolated INI file, never the user's real one."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.ini = os.path.join(self.tmp, "blackice.ini")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _cfg(self):
        return bit.AppConfig(QtCore.QSettings(self.ini, QtCore.QSettings.Format.IniFormat))


class TestAppConfig(_CfgCase):
    def test_disabled_by_default_and_writes_nothing(self):
        cfg = self._cfg()
        self.assertFalse(cfg.enabled)
        cfg.set("window/tab", 3)
        cfg.sync()
        self.assertEqual(cfg.get("window/tab", 0, int), 0)
        self.assertFalse(os.path.exists(self.ini), "must not touch disk while off")

    def test_enabled_flag_survives_a_new_instance(self):
        cfg = self._cfg()
        cfg.set_enabled(True)
        cfg.set("window/tab", 2)
        cfg.sync()
        again = self._cfg()
        self.assertTrue(again.enabled)
        self.assertEqual(again.get("window/tab", 0, int), 2)

    def test_values_are_hidden_again_once_disabled(self):
        cfg = self._cfg()
        cfg.set_enabled(True)
        cfg.set("window/tab", 2)
        cfg.set_enabled(False)
        self.assertEqual(cfg.get("window/tab", 0, int), 0)

    def test_clear_state_keeps_the_flag(self):
        cfg = self._cfg()
        cfg.set_enabled(True)
        cfg.set("window/tab", 2)
        cfg.set("dash/iface", "eth0")
        cfg.clear_state()
        self.assertTrue(cfg.enabled)
        self.assertEqual(cfg.get("window/tab", 0, int), 0)
        self.assertEqual(cfg.get("dash/iface", "", str), "")
        self.assertTrue(self._cfg().enabled)

    def test_mangled_value_falls_back_to_default(self):
        with open(self.ini, "w") as f:
            f.write("[config]\nsave_enabled=true\n\n[window]\ntab=not-a-number\n")
        cfg = self._cfg()
        self.assertTrue(cfg.enabled)
        self.assertEqual(cfg.get("window/tab", 7, int), 7)

    def test_path_is_reported(self):
        self.assertEqual(self._cfg().path, self.ini)


class TestMainWindowConfig(_CfgCase):
    def setUp(self):
        super().setUp()
        self.windows = []

    def tearDown(self):
        for w in self.windows:
            w.close()
            w.deleteLater()
        # DeferredDelete events are not delivered by processEvents() outside a
        # running event loop, and a QWebEnginePage that outlives its profile
        # crashes the interpreter at shutdown.
        _app.processEvents()
        _app.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        super().tearDown()

    def _window(self):
        w = bit.MainWindow(config=self._cfg())
        self.windows.append(w)
        return w

    def test_nothing_persisted_while_toggle_is_off(self):
        w = self._window()
        self.assertFalse(w.dash.config_btn.isChecked())
        w.tabs.setCurrentIndex(2)
        w.save_state()
        self.assertEqual(self._cfg().get("window/tab", -1, int), -1)

    def test_toggle_on_persists_immediately(self):
        w = self._window()
        w.tabs.setCurrentIndex(3)
        w.dash.config_btn.setChecked(True)
        self.assertTrue(self._cfg().enabled)
        self.assertEqual(self._cfg().get("window/tab", -1, int), 3)
        self.assertIn("config saving ENABLED", w.dash.log.toPlainText())

    def test_round_trip_of_window_and_view_state(self):
        w = self._window()
        w.dash.config_btn.setChecked(True)
        w.resize(724, 512)
        w.tabs.setCurrentIndex(1)
        w.dash.iface.setCurrentText("ALL")
        w.stats.win_buttons[900].setChecked(True)
        w.conns.filter_edit.setText("203.0.113")
        w.conns.view.sortByColumn(4, Qt.SortOrder.AscendingOrder)
        w.map.split.setSizes([250, 550])
        w.save_state()

        w2 = self._window()
        self.assertTrue(w2.cfg.enabled)
        self.assertTrue(w2.dash.config_btn.isChecked())
        self.assertEqual(w2.tabs.currentIndex(), 1)
        self.assertEqual(w2.size().width(), 724)
        self.assertEqual(w2.size().height(), 512)
        self.assertEqual(w2.stats.graph._window, 900)
        self.assertTrue(w2.stats.win_buttons[900].isChecked())
        self.assertEqual(w2.conns.filter_edit.text(), "203.0.113")
        header = w2.conns.view.horizontalHeader()
        self.assertEqual(header.sortIndicatorSection(), 4)
        self.assertEqual(header.sortIndicatorOrder(), Qt.SortOrder.AscendingOrder)
        map_w, list_w = w2.map.split.sizes()
        self.assertLess(map_w, list_w)
        self.assertIn("config restored", w2.dash.log.toPlainText())

    def test_close_saves_state(self):
        w = self._window()
        w.dash.config_btn.setChecked(True)
        w.tabs.setCurrentIndex(2)
        w.close()
        self.assertEqual(self._cfg().get("window/tab", -1, int), 2)

    def test_toggle_off_forgets_the_layout(self):
        w = self._window()
        w.dash.config_btn.setChecked(True)
        w.tabs.setCurrentIndex(2)
        w.save_state()
        w.dash.config_btn.setChecked(False)
        stored = self._cfg()
        self.assertFalse(stored.enabled)
        stored.set_enabled(True)  # peek behind the flag: the keys are really gone
        self.assertEqual(stored.get("window/tab", -1, int), -1)

    def test_restore_ignores_out_of_range_tab(self):
        cfg = self._cfg()
        cfg.set_enabled(True)
        cfg.set("window/tab", 99)
        cfg.sync()
        w = self._window()
        self.assertEqual(w.tabs.currentIndex(), 0)

    def test_restore_survives_a_garbage_geometry(self):
        cfg = self._cfg()
        cfg.set_enabled(True)
        cfg.set("window/geometry", QtCore.QByteArray(b"junk"))
        cfg.set("window/tab", 1)
        cfg.sync()
        w = self._window()  # must not raise
        self.assertEqual(w.tabs.currentIndex(), 1)

    def test_show_me_checkbox_is_restored(self):
        w = self._window()
        w.dash.config_btn.setChecked(True)
        with mock.patch.object(bit.threading, "Thread"):
            w.map.me_enable.setChecked(True)
        w.save_state()
        with mock.patch.object(bit.threading, "Thread"):
            w2 = self._window()
        self.assertTrue(w2.map.me_enable.isChecked())


if __name__ == "__main__":
    unittest.main(verbosity=2)
