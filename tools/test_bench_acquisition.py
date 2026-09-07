import struct
import unittest

from bench_acquisition import HEADER, SAMPLE, Records, firmware_diagnostics


def packet(seq, records, kind=0):
    return HEADER.pack(b'E8', 1, kind, seq, len(records), 0) + b''.join(
        SAMPLE.pack(*record) for record in records)


class DecoderTests(unittest.TestCase):
    def test_missing_or_truncated_diagnostics_are_not_zero_errors(self):
        result = firmware_diagnostics(['#TIMING:4,ready,106623,40858021,377,0',
                                       '#ADC_EVENTS:1,0,0,0,0'])
        self.assertFalse(result['diagnostics_complete'])
        self.assertNotIn('4:ready', result['timing'])
        self.assertIn('ADC_EVENTS:4', result['missing_diagnostics'])
        self.assertEqual(len(result['malformed_diagnostics']), 1)
        self.assertIsNone(firmware_diagnostics([])['diagnostics_complete'])

    def test_complete_diagnostics_and_corrupt_numeric_field(self):
        lines = []
        for adc in range(1, 5):
            for name in ('trigger', 'wake', 'read', 'publish', 'ready', 'turnaround'):
                lines.append(f'#TIMING:{adc},{name},0,0,0,0,0,0,0,0,0,0,0,0')
            lines.append(f'#ADC_EVENTS:{adc},0,0,0,0')
            lines.extend(f'#ACQ:{adc},{ch},0,0,0' for ch in range(4))
        self.assertTrue(firmware_diagnostics(lines)['diagnostics_complete'])
        combined = [line.replace(',read,', ',exchange,') for line in lines]
        result = firmware_diagnostics(combined)
        self.assertTrue(result['diagnostics_complete'])
        self.assertIn('1:exchange', result['timing'])
        self.assertNotIn('1:read', result['timing'])
        combined = [line for line in combined if not line.startswith('#TIMING:4,exchange,')]
        result = firmware_diagnostics(combined)
        self.assertFalse(result['diagnostics_complete'])
        self.assertIn('4:exchange', result['missing_diagnostics'])
        lines[-1] = '#ACQ:4,3,broken,0,0'
        result = firmware_diagnostics(lines)
        self.assertFalse(result['diagnostics_complete'])
        self.assertEqual(result['malformed_diagnostics'], [lines[-1]])

    def test_firmware_timing_and_acquisition_wrap(self):
        result = firmware_diagnostics([
            '#TIMING:1,read,2,120,50,70,0,0,2,0,0,0,0,0',
            '#TIMING:1,wake,0,0,0,0,0,0,0,0,0,0,0,0',
            '#ADC_EVENTS:1,3,4',
            '#ADC_EVENTS:2,0,0,5,7',
            '#NET:192.168.4.2:1234',
            '#NET:TX=100,ERR=2,DROP=3',
            '#ACQ:1,0,3,4294967040,1744',
            '#ACQ:1,1,0,0,0',
        ])
        self.assertEqual(result['timing']['1:read']['mean_us'], 60)
        self.assertEqual(sum(result['timing']['1:read']['bins']), 2)
        self.assertIsNone(result['timing']['1:wake']['mean_us'])
        self.assertEqual(result['device_acquisition']['0:0']['hz'], 1000)
        self.assertIsNone(result['device_acquisition']['0:1']['hz'])
        self.assertEqual(result['adc_events']['1'], {'queue_drops': 3, 'spurious': 4})
        self.assertEqual(result['firmware_network'], {'tx': 100, 'err': 2, 'drop': 3})
        self.assertEqual(result['adc_events']['2']['early_ready'], 5)
        self.assertEqual(result['adc_events']['2']['unasserted_ready'], 7)

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
