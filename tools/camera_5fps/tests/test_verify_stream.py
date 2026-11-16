"""Multipart parser tests; no HTTP requests or camera connections are made."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from verify_stream import MultipartCounter


BOUNDARY = b"offline-boundary"
DELIMITER = b"\r\n--" + BOUNDARY + b"\r\n"


def multipart(jpegs):
    data = DELIMITER[2:]
    for jpeg in jpegs:
        data += (b"Content-Type: image/jpeg\r\nContent-Length: "
                 + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + DELIMITER)
    return data


class MultipartParserTests(unittest.TestCase):
    def test_every_possible_two_chunk_split_counts_all_frames(self):
        jpegs = [b"\xff\xd8alpha\xff\xd9", b"\xff\xd8beta-longer\xff\xd9"]
        stream = multipart(jpegs)
        for offset in range(len(stream) + 1):
            with self.subTest(offset=offset):
                parser = MultipartCounter(BOUNDARY)
                actual = parser.feed(stream[:offset]) + parser.feed(stream[offset:])
                self.assertEqual(actual, [len(frame) for frame in jpegs])

    def test_byte_at_a_time_input_counts_all_frames(self):
        jpegs = [b"\xff\xd8a\xff\xd9", b"\xff\xd8bb\xff\xd9"]
        parser = MultipartCounter(BOUNDARY)
        actual = []
        for value in multipart(jpegs):
            actual.extend(parser.feed(bytes((value,))))
        self.assertEqual(actual, [len(frame) for frame in jpegs])

    def test_multiple_frames_in_one_chunk(self):
        jpegs = [b"\xff\xd8a\xff\xd9"] * 3
        self.assertEqual(MultipartCounter(BOUNDARY).feed(multipart(jpegs)), [5, 5, 5])

    def test_frame_stays_pending_until_following_boundary(self):
        jpeg = b"\xff\xd8payload\xff\xd9"
        stream = multipart([jpeg])
        parser = MultipartCounter(BOUNDARY)
        self.assertEqual(parser.feed(stream[:-len(DELIMITER)]), [])
        self.assertEqual(parser.feed(DELIMITER), [len(jpeg)])

    def test_bad_jpeg_and_non_jpeg_parts_are_rejected(self):
        for jpeg in (b"bad", b"\xff\xd8incomplete", b"missing-start\xff\xd9"):
            with self.subTest(jpeg=jpeg), self.assertRaises(ValueError):
                MultipartCounter(BOUNDARY).feed(multipart([jpeg]))
        with self.assertRaises(ValueError):
            MultipartCounter(BOUNDARY).feed(
                multipart([b"\xff\xd8ok\xff\xd9"]).replace(b"image/jpeg", b"text/plain"))

    def test_excessive_unbounded_data_is_rejected(self):
        with self.assertRaises(ValueError):
            MultipartCounter(BOUNDARY).feed(b"x" * (4 * 1024 * 1024 + 1))


if __name__ == "__main__":
    unittest.main()
