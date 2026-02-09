# Modbus RTU Sniffer (pyserial) — COM/Serial / RS-485 Traffic Reader

Пассивный **сниффер Modbus RTU** на Python: читает байты из **COM/Serial порта** (в т.ч. через USB‑RS485/USB‑UART «свисток»), выделяет кадры, проверяет **CRC‑16 Modbus** и печатает разбор (**REQ/RESP/EX**).

> ⚠️ Это именно сниффер: он **не отправляет** запросы Modbus, а только **слушает** линию.

---

## Features / Возможности

- Чтение данных из serial‑порта через **pyserial**
- Выделение Modbus RTU кадров из потока байт + ресинхронизация по CRC
- Проверка **CRC‑16 Modbus** (CRC в кадре в **little‑endian**)
- Поддержка разборов для функций:
  - `01`, `02`, `03`, `04`, `05`, `06`, `15 (0F)`, `16 (10)`
  - **Exception response** (`func | 0x80`)
- Вывод:
  - `Slave Address`, `Function Code` + человекочитаемое название
  - `Payload (hex)`
  - `Raw Frame (hex)`

---

## Requirements / Требования

- Python: **3.12+**
- Права/доступ к serial‑порту (на Linux: часто нужна группа `dialout`)
- Зависимость:
  - `pyserial` (модуль `serial`)

> В комментарии к коду упоминается `pymodbus[serial]`, но данный сниффер **не использует pymodbus** напрямую — достаточно `pyserial`.

---

## Installation / Установка

### Вариант 1 — через `uv`
```bash
uv venv
uv pip install pyserial
```

### Вариант 2 — через `pip`
```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux/macOS/WSL
source .venv/bin/activate

pip install pyserial
```

---

## Usage / Запуск

### Настройки (в коде)
В блоке `# ======== НАСТРОЙКИ ========`:

```python
# Windows:
PORT_NAME = "COM4"

# Linux/WSL:
# PORT_NAME = "/dev/ttyUSB0"   # или /dev/ttyACM0

BAUDRATE = 19200
PARITY = "N"  # 'E' (even), 'N' (none), 'O' (odd)
```

### Запуск
Если используешь запуск из корня:
```bash
python main.py
```

Если ты оформил запуск как пакет через `src/__main__.py`, то можно запускать так:
```bash
python -m src
```

Остановка: `Ctrl + C`

---

## WSL notes / Примечания для WSL

Если ты кодишь под **WSL**, то USB‑serial «свисток» должен быть доступен внутри Linux‑окружения.

- Внутри WSL порт обычно выглядит как `/dev/ttyUSB0` или `/dev/ttyACM0`.
- Иногда требуется проброс USB‑устройства в WSL (часто это делают через `usbipd` на Windows).  
  Дальше проверь, что устройство появилось в `/dev/tty*`.

---

## Example output / Пример вывода

```
--- Modbus Frame [REQ] ---
Slave Address: 1
Function Code: 3 (Read Holding Registers)
Payload (hex): 006b0003
Raw Frame (hex): 0103006b00037687
-------------------
```

---

## How it works / Как работает

1. Читает байты из serial‑порта небольшими порциями.
2. Складирует их в буфер.
3. Пытается «вырезать» валидный Modbus RTU кадр:
   - по `function code` пытается понять структуру (request/response)
   - проверяет CRC
   - при неудаче сдвигает буфер на 1 байт (ресинхронизация)
4. Печатает разобранный кадр.

---

## Limitations / Ограничения

- Функция `_calc_timings()` считает интервалы `t1.5/t3.5` (паузы RTU), но в текущей версии **не используется**.
- На реальной RS‑485 шине для «полного» сниффинга (REQ+RESP) важно правильное подключение адаптера/интерфейса.  
  В твоём кейсе чтение идёт через USB‑serial «свисток» в USB‑порт.

---

## Roadmap / Идеи для развития

- CLI‑аргументы (`argparse`): `--port`, `--baudrate`, `--parity`, `--log`
- Разделение кадров по паузе `t3.5` (в дополнение к CRC‑сканированию)
- Логирование в файл, вывод в JSON/CSV
- Декодирование payload в значения регистров/coil (по описанию карты регистров)
- Фильтры по slave id / function / диапазону регистров

---

## SEO keywords / Ключевые слова

**Primary:** modbus rtu sniffer, modbus rtu python, rs485 modbus analyzer, pyserial modbus, serial com port modbus  
**Secondary:** decode modbus frames, modbus request response, modbus crc16, sniff serial port python, protocol analyzer  
**LSI:** industrial automation, plc scada, telemetry, rs485 diagnostics, serial monitoring

---

## GitHub Topics

`modbus` · `modbus-rtu` · `rs485` · `serial` · `pyserial` · `protocol-analyzer` · `sniffer` · `industrial-automation` · `scada` · `plc` · `telemetry` · `python` · `com-port` · `wsl` · `crc16` · `packet-parser`

---

## License

Если хочешь — добавим `MIT` или `Apache-2.0` (и коротко опишем вклад/Contributing).
