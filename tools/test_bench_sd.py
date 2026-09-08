"""Offline checks for the saved-record verification used on hardware."""
import json
from pathlib import Path
import struct
import tempfile
import unittest

from bench_sd import verify, RAW, ENV


class SavedRecords(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sd = self.root / 'sd'
        self.sd.mkdir()
        self.capture = self.root / 'capture'
        self.capture.mkdir()
        self.files = ['s_TEST_0/' + n + '000.bin' for n in 'REIM']
        streams = []
        for channels in (RAW, ENV):
            data = b''.join(struct.pack('<IBBh', ts, a, c, a*10+c)
                            for a in range(4) for c in channels[a] for ts in (100, 1100))
            streams.append(data)
        streams.append(b''.join(struct.pack('<I8h', ts, *([0]*8)) for ts in (100, 5100)))
        header = bytearray(32)
        header[:8] = b'EMG8\x04\x04\x04\x14'
        header[24] = 1
        for name, data in zip('REIM', streams+[header]):
            (self.sd / (name+'000.bin')).write_bytes(data)
        packets = bytearray()
        for kind, data in enumerate(streams):
            size = 20 if kind == 2 else 8
            packet = struct.pack('<2sBBIHH', b'E8', 1, kind, 0, len(data)//size, 0) + data
            packets += struct.pack('<QH', 0, len(packet)) + packet
        (self.capture / 'udp.bin').write_bytes(packets)
        self.summary = {'mode': 1, 'counts': {str(a): [2, 2, 2, 2, 0, 0] for a in range(4)},
                        'failure': None, 'final_status': [1, 0, 1, 1, 0, 0, 0, 0, 0]}
        self.write_summary()

    def write_summary(self):
        (self.capture / 'summary.json').write_text(json.dumps(self.summary))

    def check(self):
        return verify(self.sd, self.files, self.capture)

    def test_matching_saved_bytes(self):
        result = self.check()
        self.assertEqual(result['sd_records'], [16, 16, 2])
        self.assertEqual(result['sd_absent_from_udp'], [0, 0, 0])

    def test_udp_loss_does_not_imply_sd_loss(self):
        f = self.capture / 'udp.bin'
        data = f.read_bytes()
        size = struct.unpack_from('<H', data, 8)[0]
        f.write_bytes(data[10+size:])
        self.assertEqual(self.check()['sd_absent_from_udp'], [16, 0, 0])

    def test_missing_saved_sample_fails_counts(self):
        f = self.sd / 'R000.bin'
        f.write_bytes(f.read_bytes()[:-8])
        with self.assertRaises(AssertionError): self.check()

    def test_duplicate_udp_packet_is_not_a_disk_mismatch(self):
        f = self.capture / 'udp.bin'
        data = f.read_bytes()
        size = struct.unpack_from('<H', data, 8)[0]
        f.write_bytes(data + data[:10+size])
        self.assertEqual(self.check()['udp_absent_from_sd'], [0, 0, 0])

    def test_partial_saved_record_fails(self):
        f = self.sd / 'R000.bin'
        f.write_bytes(f.read_bytes()[:-1])
        with self.assertRaises(AssertionError): self.check()

    def test_wrong_value_fails_byte_comparison(self):
        f = self.sd / 'R000.bin'
        data = bytearray(f.read_bytes())
        struct.pack_into('<h', data, 6, 555)
        f.write_bytes(data)
        with self.assertRaises(AssertionError): self.check()

    def test_duplicate_timestamp_fails(self):
        f = self.sd / 'R000.bin'
        data = bytearray(f.read_bytes())
        data[8:12] = data[:4]
        f.write_bytes(data)
        with self.assertRaises(AssertionError): self.check()

    def test_wrong_recording_header_fails(self):
        f = self.sd / 'M000.bin'
        data = bytearray(f.read_bytes())
        data[24] = 3
        f.write_bytes(data)
        with self.assertRaises(AssertionError): self.check()


if __name__ == '__main__':
    unittest.main()
