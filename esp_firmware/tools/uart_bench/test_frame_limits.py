"""Границы длины кадра протокола v2 (риск R-11), сторона хоста.

Прошивка принимает кадр не длиннее SerialReceiver::BUFFER_SIZE - 1 = 127 байт
(esp_firmware/esp32_firmware/serial_receiver.h); кадр длиннее молча
отбрасывается. В обратную сторону хост обязан принимать все кадры прошивки:
самый длинный — $405 с насыщенными счётчиками (138 байт без CRLF,
140 байт на проводе).

Проверяется, что:
  1) худший кадр каждой команды хоста помещается в 127 байт;
  2) кодировщик отказывается собирать кадр длиннее приёмного буфера прошивки;
  3) хост принимает самый длинный кадр, который может прислать прошивка.
"""
import unittest

from protocol import (MAX_COMMAND_BYTES, MAX_FRAME_BYTES, Framer, decode,
                      encode)

# Максимальное число полей команды по таблице "Jetson -> ESP32" в PROTOCOL.md.
COMMAND_FIELDS = {
    101: 2, 102: 2, 103: 0, 104: 2,
    201: 6, 202: 0, 203: 6, 204: 6, 205: 5, 206: 0, 207: 2, 208: 2, 209: 0,
    210: 7, 211: 1, 402: 0, 403: 1, 404: 2, 405: 0, 501: 0,
}

# Худшее представление float64 в формате '.9g' кодировщика: 16 символов
# (например, '-1.00000028e+187').
WORST_FIELD_WIDTH = 16


def worst_frame_length(msg_id, fields):
    """Длина худшего кадра команды при формате '.9g'."""
    return 1 + len(str(msg_id)) + 1 + fields * (WORST_FIELD_WIDTH + 1) + 1


class HostFrameLimitTests(unittest.TestCase):
    def test_worst_command_fits_firmware_buffer(self):
        # Консервативная граница: любое поле считаем худшим (16 символов).
        for msg_id, fields in sorted(COMMAND_FIELDS.items()):
            with self.subTest(id=msg_id):
                length = worst_frame_length(msg_id, fields)
                self.assertLessEqual(length, MAX_COMMAND_BYTES)

    def test_worst_case_bound_is_125_bytes(self):
        # Граница для самой длинной команды ($210, 7 полей) — 125 байт,
        # в приёмном буфере остаются 2 байта.
        self.assertEqual(worst_frame_length(210, 7), 125)
        self.assertEqual(MAX_COMMAND_BYTES - worst_frame_length(210, 7), 2)

    def test_longest_real_command_is_114_bytes(self):
        # Реальный худший случай $210: request_id — целое 1..65535 (5 символов),
        # остальные шесть полей — по 16 символов в формате '.9g'.
        frame = encode(210, 65535, *([-1.00000028e+187] * 6))
        self.assertEqual(len(frame), 114)
        self.assertLessEqual(len(frame), MAX_COMMAND_BYTES)

    def test_encoder_refuses_frame_longer_than_firmware_buffer(self):
        # Кодировщик не должен собирать кадр, который прошивка молча отбросит.
        with self.assertRaises(ValueError):
            encode(210, *([-1.00000028e+187] * 16))

    def test_host_accepts_longest_firmware_frame(self):
        # $405 — самый длинный кадр прошивки: 12 полей по 10 цифр
        # (138 байт без завершающего CRLF, 140 байт на проводе).
        raw = '$405;' + ''.join('4294967295;' for _ in range(12)) + '#'
        self.assertEqual(len(raw), 138)
        self.assertLessEqual(len(raw), MAX_FRAME_BYTES)
        frame = decode(raw)
        self.assertEqual(frame.id, 405)
        self.assertEqual(len(frame.fields), 12)
        framer = Framer()
        self.assertEqual(framer.feed(raw.encode('ascii')), [raw.encode('ascii')])

    def test_firmware_buffer_limit_is_documented_value(self):
        # Приёмный буфер SerialReceiver: BUFFER_SIZE - 1 полезных байт.
        self.assertEqual(MAX_COMMAND_BYTES, 127)


if __name__ == '__main__':
    unittest.main()
