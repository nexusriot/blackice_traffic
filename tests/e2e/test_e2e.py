"""End-to-end tests against a real socket table, inside a private netns.

Everything the normal suite knows about psutil it learns from a mock. That
proves the code does the right thing with the records it is handed; it cannot
prove the records are what the kernel actually produces — which is precisely
where this project's bugs have been. UDP listeners were dropped for a whole
release because a mocked `status` field said LISTEN and a real one says NONE.

So nothing here is mocked. The tests open real sockets, move real bytes and
read them back out of /proc and `ss`. They need the container for one reason:
in a private network namespace the socket table contains exactly what the test
put there, so "these are the only listeners" is an assertion that can be made.

    tests/e2e/run.sh

Outside the container these skip, so `unittest discover -s tests` is unchanged.
"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

IN_CONTAINER = os.environ.get("BLACKICE_E2E") == "1"

if IN_CONTAINER:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt6 import QtCore, QtWidgets
    import blackice_traffic as bit

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])

requires_container = unittest.skipUnless(
    IN_CONTAINER, "hermetic end-to-end test; run tests/e2e/run.sh")

#: A non-loopback address parked on lo. Contacts to 127.0.0.0/8 are dropped by
#: the scanner on purpose, so testing the contact path at all needs an address
#: that does not look like loopback.
PEER = "203.0.113.9"

APP = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "blackice_traffic.py")


def _have_peer_address() -> bool:
    """Park PEER on lo. Needs NET_ADMIN, so the unprivileged pass goes without."""
    if not IN_CONTAINER:
        return False
    try:
        subprocess.run(["ip", "addr", "add", f"{PEER}/32", "dev", "lo"],
                       capture_output=True, check=False, timeout=10)
        out = subprocess.run(["ip", "-o", "addr", "show", "dev", "lo"],
                             capture_output=True, text=True, timeout=10).stdout
        return PEER in out
    except (OSError, subprocess.SubprocessError):
        return False


HAVE_PEER = _have_peer_address()
requires_peer = unittest.skipUnless(
    HAVE_PEER, "needs NET_ADMIN to park a non-loopback address on lo")


class SocketFixture:
    """Sockets opened for one test, and closed with it."""

    def __init__(self):
        self._open = []

    def listen(self, port: int = 0, host: str = "0.0.0.0") -> socket.socket:
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(8)
        self._open.append(s)
        return s

    def bind_udp(self, port: int = 0, host: str = "0.0.0.0") -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((host, port))
        self._open.append(s)
        return s

    def connect(self, host: str, port: int) -> socket.socket:
        s = socket.create_connection((host, port), timeout=5)
        self._open.append(s)
        return s

    def accept(self, server: socket.socket) -> socket.socket:
        conn, _addr = server.accept()
        self._open.append(conn)
        return conn

    def close(self, s: socket.socket):
        try:
            s.close()
        except OSError:
            pass
        if s in self._open:
            self._open.remove(s)

    def close_all(self):
        while self._open:
            self.close(self._open[-1])


class E2ECase(unittest.TestCase):
    def setUp(self):
        self.sockets = SocketFixture()
        # A scanner of our own: the one inside MainWindow runs on a thread and
        # dedupes a contact for twenty seconds after it first reports it, so a
        # second scan through that object would legitimately return nothing.
        self.scanner = bit.ConnScanner()

    def tearDown(self):
        self.sockets.close_all()

    def scan(self):
        return self.scanner._scan_psutil()

    def listeners(self):
        return self.scan()[1]

    def port_set(self):
        return {(l.proto, l.port) for l in self.listeners()}


@requires_container
class TestTheNamespaceIsOurs(E2ECase):
    """The assertion that only a private netns makes possible."""

    def test_an_empty_namespace_has_no_listeners(self):
        self.assertEqual(self.listeners(), [])

    def test_the_only_listeners_are_the_ones_we_opened(self):
        self.sockets.listen(6379)
        self.sockets.bind_udp(5353)
        self.assertEqual(self.port_set(), {("tcp", 6379), ("udp", 5353)})


@requires_container
class TestRealListeningSockets(E2ECase):
    def test_a_real_tcp_listener_is_classified(self):
        self.sockets.listen(6379)
        found = [l for l in self.listeners() if l.port == 6379]
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0].proto, "tcp")
        self.assertEqual(found[0].exposure, bit.EXPOSURE_WORLD)
        self.assertEqual(found[0].note, "DATABASE (OFTEN UNAUTHENTICATED)")

    def test_a_real_udp_bind_is_reported(self):
        # The regression this whole file exists for: a UDP socket's status is
        # NONE, never LISTEN, and keying on LISTEN hid every one of them.
        self.sockets.bind_udp(5353)
        found = [l for l in self.listeners() if l.port == 5353]
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0].proto, "udp")

    def test_a_loopback_bind_cannot_be_reached_from_outside(self):
        self.sockets.listen(6380, host="127.0.0.1")
        found = [l for l in self.listeners() if l.port == 6380]
        self.assertEqual(found[0].exposure, bit.EXPOSURE_LOCAL)
        self.assertEqual(found[0].note, "")

    def test_a_listener_is_attributed_to_this_process(self):
        self.sockets.listen(6379)
        found = [l for l in self.listeners() if l.port == 6379][0]
        self.assertEqual(found.pid, os.getpid())
        self.assertIn("python", found.process.lower())

    def test_closing_a_socket_removes_it_from_the_inventory(self):
        server = self.sockets.listen(6379)
        self.assertIn(("tcp", 6379), self.port_set())
        self.sockets.close(server)
        self.assertNotIn(("tcp", 6379), self.port_set())


@requires_container
class TestListenerDiffOverRealSockets(E2ECase):
    def setUp(self):
        super().setUp()
        self.tab = bit.ListenersTab()

    def tearDown(self):
        self.tab.deleteLater()
        _app.processEvents()
        super().tearDown()

    def _feed(self):
        self.tab.set_listeners([l.__dict__ for l in self.listeners()])

    def test_opening_a_port_is_reported_once_there_is_a_baseline(self):
        opened = []
        self.tab.listenersOpened.connect(opened.extend)
        self._feed()                       # baseline: an empty namespace
        self.assertEqual(opened, [])
        self.sockets.listen(6379)
        self._feed()
        self.assertEqual([l["port"] for l in opened], [6379])

    def test_closing_a_port_is_reported(self):
        closed = []
        self.tab.listenersClosed.connect(closed.extend)
        server = self.sockets.listen(6379)
        self._feed()
        self.sockets.close(server)
        self._feed()
        self.assertEqual([l["port"] for l in closed], [6379])


@requires_container
@requires_peer
class TestRealContacts(E2ECase):
    def setUp(self):
        super().setUp()
        self.tab = bit.ConnectionsTab()
        self.tab.resolve_hosts = False

    def tearDown(self):
        self.tab.deleteLater()
        _app.processEvents()
        super().tearDown()

    def test_an_established_connection_becomes_a_contact(self):
        server = self.sockets.listen(6379, host=PEER)
        self.sockets.connect(PEER, 6379)
        self.sockets.accept(server)
        contacts, _l, _s = self.scan()
        peers = {(c.ip, c.port) for c in contacts}
        self.assertIn((PEER, 6379), peers)

    def test_the_contact_reaches_the_table_with_its_state(self):
        server = self.sockets.listen(6379, host=PEER)
        self.sockets.connect(PEER, 6379)
        self.sockets.accept(server)
        contacts, _l, _s = self.scan()
        self.tab.add_points([c.__dict__ for c in contacts])
        rows = {self.tab.model.item(r, self.tab.C_IP).text()
                for r in range(self.tab.model.rowCount())}
        self.assertIn(PEER, rows)
        states = {self.tab.model.item(r, self.tab.C_STATE).text()
                  for r in range(self.tab.model.rowCount())}
        self.assertIn("ESTABLISHED", states)

    def test_loopback_contacts_are_still_dropped(self):
        server = self.sockets.listen(6381, host="127.0.0.1")
        self.sockets.connect("127.0.0.1", 6381)
        self.sockets.accept(server)
        contacts, _l, _s = self.scan()
        self.assertEqual([c for c in contacts if c.ip.startswith("127.")], [])


@requires_container
@requires_peer
class TestRealByteCounters(E2ECase):
    """The per-socket numbers come from the kernel's own tcp_info, so the only
    way to know the parser and the delta arithmetic are right is to move a
    known quantity of bytes and read it back."""

    PAYLOAD = 200_000

    def _transfer(self, sender: socket.socket, receiver: socket.socket, n: int):
        data = b"x" * n
        sent = threading.Thread(target=sender.sendall, args=(data,))
        sent.start()
        got = 0
        while got < n:
            chunk = receiver.recv(65536)
            if not chunk:
                break
            got += len(chunk)
        sent.join(timeout=10)
        time.sleep(0.4)          # let the kernel settle before reading tcp_info
        return got

    def test_a_known_transfer_shows_up_against_the_endpoint(self):
        server = self.sockets.listen(6379, host=PEER)
        client = self.sockets.connect(PEER, 6379)
        accepted = self.sockets.accept(server)

        sampler = bit.ProcTrafficSampler()
        self.assertTrue(sampler.enabled, "ss is missing from the image")
        sampler.sample()                      # baseline the existing counters
        self.assertEqual(self._transfer(accepted, client, self.PAYLOAD),
                         self.PAYLOAD)
        totals = sampler.sample()

        # The client pulled the payload from the listening port.
        down = sampler.peer_totals().get(f"tcp:{PEER}:6379", (0, 0))
        self.assertEqual(down[0], self.PAYLOAD)
        # Both ends are this process, so its own total carries it too.
        self.assertGreaterEqual(sum(totals.get("python3", (0, 0))), self.PAYLOAD)

    def test_the_figure_reaches_the_contact_table(self):
        server = self.sockets.listen(6379, host=PEER)
        client = self.sockets.connect(PEER, 6379)
        accepted = self.sockets.accept(server)

        sampler = bit.ProcTrafficSampler()
        sampler.sample()
        self._transfer(accepted, client, self.PAYLOAD)
        sampler.sample()

        contacts, _l, _s = self.scan()
        tab = bit.ConnectionsTab()
        tab.resolve_hosts = False
        try:
            tab.add_points([c.__dict__ for c in contacts])
            tab.set_peer_io(sampler.peer_totals())
            # Both ends of the connection are in this namespace, so one
            # transfer produces two contacts on the same address: the client's
            # view of :6379 and the server's view of the ephemeral port. Only
            # the first pulled the payload, so the port is part of the key.
            shown = {}
            for r in range(tab.model.rowCount()):
                ip = tab.model.item(r, tab.C_IP).text()
                port = tab.model.item(r, tab.C_PORT).text().split(" ", 1)[0]
                shown[(ip, port)] = tab.model.item(r, tab.C_RX).text()
            self.assertIn((PEER, "6379"), shown, shown)
            self.assertIn("KB", shown[(PEER, "6379")])
        finally:
            tab.deleteLater()
            _app.processEvents()

    def test_nothing_moved_means_nothing_counted(self):
        self.sockets.listen(6379, host=PEER)
        self.sockets.connect(PEER, 6379)
        sampler = bit.ProcTrafficSampler()
        sampler.sample()
        time.sleep(0.3)
        sampler.sample()
        self.assertEqual(sampler.peer_totals(), {})


@requires_container
class TestAttributionIsHonest(E2ECase):
    """Both passes of run.sh execute this; each asserts its own half."""

    def test_the_scan_reports_the_privilege_it_ran_with(self):
        self.sockets.listen(16379, host="127.0.0.1")
        self.scan()
        stats = self.scanner.scan_stats
        self.assertEqual(bool(stats["elevated"]), bit.is_elevated())

    def test_root_can_see_every_owner(self):
        if not bit.is_elevated():
            self.skipTest("unprivileged pass")
        self.scan()
        self.assertEqual(bit.attribution_summary(self.scanner.scan_stats),
                         "all processes (root)")

    def test_an_unprivileged_run_says_so(self):
        if bit.is_elevated():
            self.skipTest("privileged pass")
        self.scan()
        self.assertTrue(
            bit.attribution_summary(self.scanner.scan_stats)
            .startswith("own processes"))

    def test_our_own_sockets_are_attributed_either_way(self):
        self.sockets.listen(16380, host="127.0.0.1")
        found = [l for l in self.listeners() if l.port == 16380][0]
        self.assertEqual(found.pid, os.getpid())


@requires_container
class TestHeadlessAgainstRealSockets(E2ECase):
    def _run(self, *args, watchlist: str = ""):
        home = tempfile.mkdtemp(prefix="blackice-e2e-")
        wl = os.path.join(home, "watchlist.txt")
        with open(wl, "w", encoding="utf-8") as f:
            f.write(watchlist)
        env = dict(os.environ, HOME=home)
        proc = subprocess.run(
            [sys.executable, APP, "--watchlist", wl] + list(args),
            capture_output=True, text=True, timeout=60, env=env)
        return proc

    def test_json_stream_carries_the_real_socket_table(self):
        self.sockets.listen(6379)
        proc = self._run("--json", "--duration", "0", "--interval", "1")
        lines = [l for l in proc.stdout.splitlines() if l.strip()]
        self.assertTrue(lines, proc.stderr)
        record = json.loads(lines[0])
        self.assertIn(6379, [l["port"] for l in record["listeners"]])
        self.assertEqual(record["version"], bit.APP_VERSION)

    def test_watch_exits_clean_when_nothing_is_flagged(self):
        proc = self._run("--watch", "--duration", "1", "--interval", "1")
        self.assertEqual(proc.returncode, bit.EXIT_OK, proc.stdout + proc.stderr)

    @requires_peer
    def test_watch_exits_three_on_a_real_watchlist_hit(self):
        server = self.sockets.listen(6379, host=PEER)
        self.sockets.connect(PEER, 6379)
        self.sockets.accept(server)
        proc = self._run("--watch", "--duration", "1", "--interval", "1",
                         watchlist=f"cidr {PEER}/32 E2E PEER\n")
        self.assertEqual(proc.returncode, bit.EXIT_WATCHLIST,
                         proc.stdout + proc.stderr)
        self.assertIn("WATCHLIST", proc.stdout + proc.stderr)

    def test_watch_exits_four_when_a_sensitive_port_opens_mid_run(self):
        opener = threading.Timer(2.0, lambda: self.sockets.listen(6379))
        opener.start()
        try:
            proc = self._run("--watch", "--duration", "5", "--interval", "1")
        finally:
            opener.cancel()
        self.assertEqual(proc.returncode, bit.EXIT_EXPOSURE,
                         proc.stdout + proc.stderr)
        self.assertIn("EXPOSURE", proc.stdout + proc.stderr)

    def test_watch_without_a_duration_runs_until_it_is_stopped(self):
        """--duration 0 under --watch is the systemd shape: no deadline."""
        home = tempfile.mkdtemp(prefix="blackice-e2e-")
        proc = subprocess.Popen(
            [sys.executable, APP, "--watch", "--duration", "0", "--interval", "1"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            env=dict(os.environ, HOME=home))
        try:
            time.sleep(4)
            self.assertIsNone(proc.poll(), "it stopped on its own")
            proc.send_signal(subprocess.signal.SIGINT)
            out, err = proc.communicate(timeout=20)
            self.assertIn("interrupted", out + err)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=10)

    def test_the_app_writes_nothing_into_a_read_only_install(self):
        before = os.stat(APP).st_mtime
        self._run("--headless", "--duration", "0")
        self.assertEqual(os.stat(APP).st_mtime, before)


@requires_container
class TestPersistenceAcrossRuns(E2ECase):
    """History and config are the two things that are supposed to outlive the
    process, which no in-process test can really demonstrate."""

    def setUp(self):
        super().setUp()
        self.home = tempfile.mkdtemp(prefix="blackice-e2e-home-")
        self.db = os.path.join(self.home, "history.db")

    def test_a_contact_recorded_in_one_run_is_known_in_the_next(self):
        first = bit.SessionStore(self.db)
        self.assertTrue(first.open(), first.error)
        first.record_contacts([{"ip": "198.18.0.7", "port": 443, "proto": "tcp",
                                "ts": time.time(), "process": "curl"}])
        first.close()

        second = bit.SessionStore(self.db)
        second.open()
        try:
            self.assertFalse(second.is_new("198.18.0.7"))
            self.assertTrue(second.is_new("198.18.0.8"))
            self.assertEqual(second.sessions_summary()["sessions"], 2)
        finally:
            second.close()

    def test_a_killed_run_still_leaves_a_usable_database(self):
        script = (
            "import sys, time; sys.path.insert(0, %r);"
            "import blackice_traffic as bit;"
            "s = bit.SessionStore(%r); s.open();"
            "s.record_contacts([{'ip':'198.18.0.9','port':443,'proto':'tcp',"
            "'ts':time.time()}]);"
            "s.touch_session(s._session_started + 60);"
            "print('ready', flush=True); time.sleep(60)"
            % (os.path.dirname(APP), self.db))
        proc = subprocess.Popen([sys.executable, "-c", script],
                                stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(proc.stdout.readline().strip(), "ready")
        finally:
            proc.kill()
            proc.wait(timeout=10)
            proc.stdout.close()

        store = bit.SessionStore(self.db)
        self.assertTrue(store.open(), store.error)
        try:
            self.assertFalse(store.is_new("198.18.0.9"))
            self.assertGreaterEqual(store.sessions_summary()["watched"], 60)
        finally:
            store.close()

    def test_config_survives_a_restart(self):
        path = os.path.join(self.home, "blackice.ini")
        first = bit.AppConfig(QtCore.QSettings(
            path, QtCore.QSettings.Format.IniFormat))
        first.set_enabled(True)
        first.set("window/tab", 3)
        first.sync()

        second = bit.AppConfig(QtCore.QSettings(
            path, QtCore.QSettings.Format.IniFormat))
        self.assertTrue(second.enabled)
        self.assertEqual(second.get("window/tab", -1, int), 3)


@requires_container
class TestTheWindowRunsHere(E2ECase):
    """A smoke test for the real thing: build the window, let its threads do a
    pass, shut it down. Catches the import-time and teardown faults that never
    show up when tabs are constructed one at a time."""

    def test_the_window_starts_scans_and_stops_cleanly(self):
        self.sockets.listen(6379)
        home = tempfile.mkdtemp(prefix="blackice-e2e-win-")
        cfg = bit.AppConfig(QtCore.QSettings(
            os.path.join(home, "c.ini"), QtCore.QSettings.Format.IniFormat))
        window = bit.MainWindow(config=cfg,
                                store=bit.SessionStore(os.path.join(home, "h.db")))
        window.show()
        deadline = time.time() + 20
        while time.time() < deadline and window.listen.model.rowCount() == 0:
            _app.processEvents()
            time.sleep(0.1)
        try:
            ports = {window.listen.model.item(r, window.listen.C_PORT).text()
                     for r in range(window.listen.model.rowCount())}
            self.assertTrue(any("6379" in p for p in ports), ports)
            self.assertIn("SCOPE", window.dash.sub.text())
        finally:
            window._quitting = True
            window.close()
            _app.processEvents()
        self.assertFalse(window.scanner.isRunning())
        self.assertFalse(window.poller.isRunning())
        self.assertFalse(window.rdns.isRunning())


if __name__ == "__main__":
    unittest.main()
