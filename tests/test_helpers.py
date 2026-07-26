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
