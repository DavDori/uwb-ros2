import unittest

from uwb_serial.uwb import parseSerialDataUWB


class TestUWBNode(unittest.TestCase):

    def test_parse_valid_serial_data(self):
        serial_data = "123 456 789 12.34"
        expected_output = {
            'seq': 123,
            'id_self': 456,
            'id_other': 789,
            'range': 12.34
        }
        self.assertEqual(
            parseSerialDataUWB(serial_data),
            expected_output
            )

    def test_parse_invalid_serial_data(self):
        # Test with empty data
        serial_data = ""
        self.assertIsNone(parseSerialDataUWB(serial_data))

        # Test with incomplete data
        serial_data = "123 456"
        with self.assertRaises(ValueError):
            parseSerialDataUWB(serial_data)
