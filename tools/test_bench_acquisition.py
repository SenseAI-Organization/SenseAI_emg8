import struct
import unittest

from bench_acquisition import HEADER, SAMPLE, Records


def packet(seq, records, kind=0):
    return HEADER.pack(b'E8', 1, kind, seq, len(records), 0) + b''.join(
        SAMPLE.pack(*record) for record in records)


class DecoderTests(unittest.TestCase):
    def test_current_mapping_and_rates(self):
        r = Records()
        r.feed(packet(1, [(100, 1, 1, -12), (1100, 1, 1, 3), (2100, 1, 1, 7)]))
        self.assertEqual(r.summary()['channels']['0:1:1']['received_hz'], 1000)
        r.feed(packet(2, [(100, 1, 0, 3)]))  # ADC2 ch0 is envelope, not raw
        self.assertEqual(r.invalid, 1)

    def test_sequence_wrap_gaps_and_duplicates(self):
        r = Records()
        for seq in (0xffffffff, 0, 2, 2):
            r.feed(packet(seq, [(100, 0, 0, 1)]))
        self.assertEqual(r.gaps[0], 1)
        self.assertEqual(r.duplicates[0], 1)
        self.assertEqual(len(r.timestamps['0:0:0']), 3)

    def test_truncated_datagram_is_rejected(self):
        r = Records()
        r.feed(packet(0, [(1, 0, 0, 0)])[:-1])
        self.assertEqual(r.invalid, 1)
        self.assertFalse(r.timestamps)

    def test_reordered_packet_recovers_gap_without_losing_records(self):
        r = Records()
        for seq, ts in ((0, 100), (2, 2100), (1, 1100)):
            r.feed(packet(seq, [(ts, 0, 0, 1)]))
        self.assertEqual(r.gaps[0], 0)
        self.assertEqual(r.reordered[0], 1)
        self.assertEqual(r.summary()['channels']['0:0:0']['received_hz'], 1000)

    def test_timestamp_wrap(self):
        r = Records()
        r.feed(packet(0, [(0xffffff00, 0, 0, 0), (744, 0, 0, 0)]))
        self.assertEqual(r.summary()['channels']['0:0:0']['received_hz'], 1000)

    def test_complete_windows_exclude_partial_edges(self):
        r = Records()
        for ts in range(1000, 20002000, 1000):
            r.timestamps['0:0:0'].append(ts)
        self.assertEqual(r.summary()['channels']['0:0:0']['complete_10s_received_hz'], [1000])


if __name__ == '__main__':
    unittest.main()
