"""Unit tests for the pure helper functions in blackice_traffic.

Run from the project root with the runtime deps installed (PyQt6, psutil):

    python3 -m unittest discover -s tests
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import blackice_traffic as bit


class TestHumanBps(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(bit.human_bps(0), "0.0 b/s")

    def test_sub_kilo(self):
        self.assertEqual(bit.human_bps(999), "999.0 b/s")

    def test_kilo_boundary(self):
        # 1000 rolls over to the next (decimal) unit
        self.assertEqual(bit.human_bps(1000), "1.0 Kb/s")

    def test_mega(self):
        self.assertEqual(bit.human_bps(2_500_000), "2.5 Mb/s")

    def test_uses_decimal_1000_not_1024(self):
        self.assertTrue(bit.human_bps(1500).endswith("Kb/s"))


class TestHumanBytes(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(bit.human_bytes(0), "0.0 B")

    def test_sub_kib(self):
        self.assertEqual(bit.human_bytes(1023), "1,023.0 B")

    def test_kib_boundary(self):
        # bytes use binary 1024 steps
        self.assertEqual(bit.human_bytes(1024), "1.0 KB")

    def test_mib(self):
        self.assertEqual(bit.human_bytes(5 * 1024 * 1024), "5.0 MB")


class TestPortService(unittest.TestCase):
    def test_known_ports(self):
        self.assertEqual(bit.port_service(22), "SSH")
        self.assertEqual(bit.port_service(443), "HTTPS")
        self.assertEqual(bit.port_service(3306), "MYSQL")

    def test_unknown_port(self):
        self.assertEqual(bit.port_service(65000), "")

    def test_type_is_str(self):
        self.assertIsInstance(bit.port_service(80), str)


class TestClamp(unittest.TestCase):
    def test_within_range(self):
        self.assertEqual(bit.clamp(5, 0, 10), 5)

    def test_below_low(self):
        self.assertEqual(bit.clamp(-3, 0, 10), 0)

    def test_above_high(self):
        self.assertEqual(bit.clamp(42, 0, 10), 10)

    def test_boundaries(self):
        self.assertEqual(bit.clamp(0, 0, 10), 0)
        self.assertEqual(bit.clamp(10, 0, 10), 10)


class TestNormalizeIp(unittest.TestCase):
    def test_v4_mapped_v6_is_stripped(self):
        self.assertEqual(bit.normalize_ip("::ffff:203.0.113.7"), "203.0.113.7")

    def test_plain_v4_passthrough(self):
        self.assertEqual(bit.normalize_ip("8.8.8.8"), "8.8.8.8")

    def test_plain_v6_passthrough(self):
        self.assertEqual(bit.normalize_ip("2001:db8::1"), "2001:db8::1")


class TestIsPrivateish(unittest.TestCase):
    def test_public_v4(self):
        self.assertFalse(bit.is_privateish("8.8.8.8"))
        self.assertFalse(bit.is_privateish("1.1.1.1"))

    def test_public_v6(self):
        self.assertFalse(bit.is_privateish("2001:4860:4860::8888"))

    def test_rfc1918(self):
        self.assertTrue(bit.is_privateish("192.168.1.10"))
        self.assertTrue(bit.is_privateish("10.0.0.1"))
        self.assertTrue(bit.is_privateish("172.16.5.4"))

    def test_loopback(self):
        self.assertTrue(bit.is_privateish("127.0.0.1"))
        self.assertTrue(bit.is_privateish("::1"))

    def test_link_local(self):
        self.assertTrue(bit.is_privateish("169.254.1.1"))

    def test_multicast(self):
        self.assertTrue(bit.is_privateish("224.0.0.1"))

    def test_garbage_is_treated_as_private(self):
        # Unparseable input must fail closed (not placed on the public map)
        self.assertTrue(bit.is_privateish("not-an-ip"))
        self.assertTrue(bit.is_privateish(""))


if __name__ == "__main__":
    unittest.main(verbosity=2)


class TestIsLoopbackNic(unittest.TestCase):
    def test_common_names(self):
        self.assertTrue(bit.is_loopback_nic("lo"))
        self.assertTrue(bit.is_loopback_nic("lo0"))
        self.assertTrue(bit.is_loopback_nic("Loopback Pseudo-Interface 1"))

    def test_real_nics(self):
        for n in ("eth0", "wlp67s0", "docker0", "veth37bc6b9", "eno0"):
            self.assertFalse(bit.is_loopback_nic(n), n)

    def test_empty(self):
        self.assertFalse(bit.is_loopback_nic(""))


class TestIsLoopbackIp(unittest.TestCase):
    def test_v4(self):
        self.assertTrue(bit.is_loopback_ip("127.0.0.1"))
        self.assertTrue(bit.is_loopback_ip("127.1.2.3"))

    def test_v6(self):
        self.assertTrue(bit.is_loopback_ip("::1"))

    def test_public(self):
        self.assertFalse(bit.is_loopback_ip("8.8.8.8"))

    def test_garbage(self):
        self.assertFalse(bit.is_loopback_ip("not-an-ip"))


def _snetio(rx, tx):
    import psutil
    return psutil._common.snetio(tx, rx, 0, 0, 0, 0, 0, 0)


class TestBuildSnapshot(unittest.TestCase):
    def test_loopback_excluded_from_totals(self):
        prev = {"lo": _snetio(0, 0), "eth0": _snetio(0, 0)}
        now = {"lo": _snetio(125_000, 125_000), "eth0": _snetio(125_000, 125_000)}
        snap = bit.build_snapshot(prev, now, 1.0)
        # only eth0 counts: 125 kB/s == 1 Mb/s
        self.assertEqual(snap["_totals"]["rx_bps"], 1_000_000.0)
        self.assertEqual(snap["_totals"]["tx_bps"], 1_000_000.0)
        # ...but lo is still reported so it can be selected in the combo
        self.assertIn("lo", snap)
        self.assertEqual(snap["lo"]["rx_bps"], 1_000_000.0)

    def test_new_nic_without_baseline_is_skipped(self):
        snap = bit.build_snapshot({}, {"eth0": _snetio(999, 999)}, 1.0)
        self.assertNotIn("eth0", snap)
        self.assertEqual(snap["_totals"]["rx_bps"], 0.0)

    def test_counter_reset_clamps_to_zero(self):
        prev = {"eth0": _snetio(10_000, 10_000)}
        now = {"eth0": _snetio(5, 5)}
        snap = bit.build_snapshot(prev, now, 1.0)
        self.assertEqual(snap["eth0"]["rx_bps"], 0.0)
        self.assertEqual(snap["_totals"]["rx_bps"], 0.0)

    def test_totals_key_always_present(self):
        self.assertEqual(bit.build_snapshot({}, {}, 1.0),
                         {"_totals": {"rx_bps": 0.0, "tx_bps": 0.0}})
