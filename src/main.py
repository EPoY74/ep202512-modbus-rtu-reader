"""
Для работы с ModBusRTU
Программа для чтения информации с com порта и декодирования кадров
pip install pymodbus[serial]
"""

import time
from binascii import hexlify
from struct import unpack

import serial


class ModbusRtuSniffer:
    def __init__(self, port, baudrate=9600, parity='E', stopbits=1, bytesize=8):
        """Инициализация соединения с COM-портом в режиме чтения."""
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=0.1  # Важно: короткий таймаут для детектирования пауз
        )
        print(f"[INFO] Слушаем порт {port}...")  # noqa T201


    def _calc_timings(self):
        # сколько бит в одном "символе" RTU
        parity_bits = 0 if self.ser.parity in (serial.PARITY_NONE, 'N') else 1
        bits_per_char = 1 + self.ser.bytesize + parity_bits + self.ser.stopbits
        char_time = bits_per_char / self.ser.baudrate

        # рекомендация из спецификации для >19200
        if self.ser.baudrate > 19200:
            t15 = 0.00075   # 750 us
            t35 = 0.00175   # 1.750 ms
        else:
            t15 = 1.5 * char_time
            t35 = 3.5 * char_time
        return t15, t35


    def _try_take_frame(self, buf: bytearray) -> bytes | None:
        if len(buf) < 4:
            return None

        addr = buf[0]
        func = buf[1]

        # грубая фильтрация адреса
        if addr > 247:
            buf.pop(0)
            return None

        # exception response: 5 bytes
        if func & 0x80:
            if len(buf) >= 5 and self._crc_ok(bytes(buf[:5])):
                fr = bytes(buf[:5])
                del buf[:5]
                return fr
            buf.pop(0)
            return None

        # Для 03/04: сначала пробуем "запрос" 8 байт
        if func in (3, 4):
            if len(buf) >= 8 and self._crc_ok(bytes(buf[:8])):
                fr = bytes(buf[:8])
                del buf[:8]
                return fr

            # затем пробуем "ответ" с byte_count (3-й байт)
            if len(buf) >= 3:
                bc = buf[2]
                if bc <= 252:
                    n = 5 + bc
                    if len(buf) >= n and self._crc_ok(bytes(buf[:n])):
                        fr = bytes(buf[:n])
                        del buf[:n]
                        return fr

            # не получилось — сдвиг на 1 байт (ресинхронизация)
            buf.pop(0)
            return None

        # Для 01/02 ответы тоже имеют byte_count в 3-м байте
        if func in (1, 2):
            if len(buf) >= 3:
                bc = buf[2]
                if bc <= 252:
                    n = 5 + bc
                    if len(buf) >= n and self._crc_ok(bytes(buf[:n])):
                        fr = bytes(buf[:n])
                        del buf[:n]
                        return fr
            # запросы 01/02 обычно 8 байт
            if len(buf) >= 8 and self._crc_ok(bytes(buf[:8])):
                fr = bytes(buf[:8])
                del buf[:8]
                return fr
            buf.pop(0)
            return None

        # 05/06 и ответы 0F/10 — часто 8 байт
        if func in (5, 6):
            if len(buf) >= 8 and self._crc_ok(bytes(buf[:8])):
                fr = bytes(buf[:8])
                del buf[:8]
                return fr
            buf.pop(0)
            return None
        
        if func in (15, 16):
            # Сначала попробуем "ответ" (он 8 байт)
            if len(buf) >= 8 and self._crc_ok(bytes(buf[:8])):
                fr = bytes(buf[:8])
                del buf[:8]
                return fr

            # Потом попробуем "запрос" с Byte Count
            # addr(0) func(1) start(2) qty(2) => byte_count лежит в buf[6]
            if len(buf) >= 7:
                bc = buf[6]
                n = 9 + bc
                if bc <= 252 and len(buf) >= n and self._crc_ok(bytes(buf[:n])):
                    fr = bytes(buf[:n])
                    del buf[:n]
                    return fr

            buf.pop(0)
            return None

        # неизвестная функция — сдвиг
        buf.pop(0)
        return None

    def calc_crc16(self, data):
        """Вычисление CRC-16 Modbus для проверки целостности кадра."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    

    def _crc_ok(self, frame: bytes) -> bool:
        if len(frame) < 4:
            return False
        data_part = frame[:-2]
        crc_received = int.from_bytes(frame[-2:], byteorder="little")
        crc_calculated = self.calc_crc16(data_part)
        return crc_calculated == crc_received

    def parse_modbus_frame(self, frame):
        """Парсинг и проверка кадра Modbus RTU."""
        if len(frame) < 4:  # Минимальный кадр: Адрес(1) + Функция(1) + CRC(2)
            return None

        # Разделяем кадр: данные и CRC
        data_part = frame[:-2]
        crc_received = frame[-2:]
        crc_calculated = self.calc_crc16(data_part)

        # Проверка CRC (учёт порядка байт: little-endian в кадре)
        if crc_calculated != unpack('<H', crc_received)[0]:
            got = unpack('<H', crc_received)[0]
            print(f"[WARN] CRC mismatch len={len(frame)} "  # noqa T201clear
                f"calc=0x{crc_calculated:04X} got=0x{got:04X} "
                f"raw={hexlify(frame).decode('ascii')}")
            return None

        # Извлечение полей
        slave_addr = data_part[0]
        func_code = data_part[1]
        payload = data_part[2:] if len(data_part) > 2 else b''

        return {
            'addr': slave_addr,
            'func': func_code,
            'payload': payload,
            'raw': frame
        }

    def listen(self):
        buffer = bytearray()
        print("[INFO] Ожидание данных... (Ctrl+C для остановки)")  # noqa T201

        try:
            while True:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if chunk:
                    buffer.extend(chunk)

                    while True:
                        fr = self._try_take_frame(buffer)
                        if not fr:
                            break
                        # тут CRC уже должен сходиться
                        info = self.parse_modbus_frame(fr)  
                        if info:
                            self.print_frame(info)
                else:
                    time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n[INFO] Остановка пользователем.")  # noqa T201
        finally:
            self.ser.close()


    def print_frame(self, info):
        """Красивый вывод разобранного кадра."""
        raw_len = len(info["raw"])
        label_frame = "AMB"
        if info["func"] & 0x80:
            label_frame = "RESP_EX"
        elif info["func"] in (3, 4):
            label_frame = "REQ" if raw_len == 8 else "RESP"
        
        func_map = {
            1: "Read Coils",
            2: "Read Discrete Inputs",
            3: "Read Holding Registers",
            4: "Read Input Registers",
            5: "Write Single Coil",
            6: "Write Single Register",
            15: "Write Multiple Coils",
            16: "Write Multiple Registers"
        }
        func_desc = func_map.get(info['func'], f"Unknown ({info['func']})")

        print(f"\n--- Modbus Frame [{label_frame}] ---")  # noqa T201
        if info['addr'] == 0:
            print("---broadcast---")  # noqa T201
        print(f"Slave Address: {info['addr']}")  # noqa T201
        print(f"Function Code: {info['func']} ({func_desc})")  # noqa T201
        print(f"Payload (hex): {hexlify(info['payload']).decode('ascii') if info['payload'] else 'N/A'}")  # noqa T201
        print(f"Raw Frame (hex): {hexlify(info['raw']).decode('ascii')}")  # noqa T201
        print(f"-------------------")  # noqa T201

# ======== НАСТРОЙКИ ========
PORT_NAME = 'COM4'      # Ваш COM-порт
BAUDRATE = 19200         # Должен совпадать с настройками устройств в сети
PARITY = 'N'            # Четность: 'E' (even), 'N' (none), 'O' (odd)
# ===========================

def main():
    """
    Точка входа
    """
    sniffer = ModbusRtuSniffer(
        port=PORT_NAME,
        baudrate=BAUDRATE,
        parity=PARITY
    )
    sniffer.listen()

if __name__ == "__main__":
    main()
