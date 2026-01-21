# bme280_pico.py
# Minimal BME280 driver for Raspberry Pi Pico using MicroPython
# SDA = GP0, SCL = GP1

from machine import Pin, I2C
import time
import struct

class BME280:
    def __init__(self, i2c: I2C, address=0x76):
        self.i2c = i2c
        self.addr = address

        chip_id = self._read_u8(0xD0)
        if chip_id != 0x60:
            raise RuntimeError("BME280 not found or wrong chip ID: 0x{:02X}".format(chip_id))

        self._write_u8(0xE0, 0xB6)  # soft reset
        time.sleep_ms(4)

        self._read_calibration()

        # Humidity oversampling x1
        self._write_u8(0xF2, 0x01)
        # Temp oversampling x1, Pressure oversampling x1, Normal mode
        self._write_u8(0xF4, (0x01 << 5) | (0x01 << 2) | 0x03)
        # Standby 62.5ms, filter coeff 16
        self._write_u8(0xF5, (0x02 << 5) | (0x04 << 2))

    # --- I2C helpers ---
    def _read_u8(self, reg): return self.i2c.readfrom_mem(self.addr, reg, 1)[0]
    def _write_u8(self, reg, val): self.i2c.writeto_mem(self.addr, reg, bytes([val]))
    def _read_s16(self, reg): return struct.unpack('<h', self.i2c.readfrom_mem(self.addr, reg, 2))[0]
    def _read_u16(self, reg): return struct.unpack('<H', self.i2c.readfrom_mem(self.addr, reg, 2))[0]

    # --- Calibration data ---
    def _read_calibration(self):
        self.dig_T1 = self._read_u16(0x88)
        self.dig_T2 = self._read_s16(0x8A)
        self.dig_T3 = self._read_s16(0x8C)
        self.dig_P1 = self._read_u16(0x8E)
        self.dig_P2 = self._read_s16(0x90)
        self.dig_P3 = self._read_s16(0x92)
        self.dig_P4 = self._read_s16(0x94)
        self.dig_P5 = self._read_s16(0x96)
        self.dig_P6 = self._read_s16(0x98)
        self.dig_P7 = self._read_s16(0x9A)
        self.dig_P8 = self._read_s16(0x9C)
        self.dig_P9 = self._read_s16(0x9E)
        self.dig_H1 = self._read_u8(0xA1)
        self.dig_H2 = self._read_s16(0xE1)
        self.dig_H3 = self._read_u8(0xE3)
        e4 = self._read_u8(0xE4)
        e5 = self._read_u8(0xE5)
        e6 = self._read_u8(0xE6)
        self.dig_H4 = (e4 << 4) | (e5 & 0x0F)
        self.dig_H5 = (e6 << 4) | (e5 >> 4)
        self.dig_H4 = struct.unpack('<h', struct.pack('<H', self.dig_H4))[0]
        self.dig_H5 = struct.unpack('<h', struct.pack('<H', self.dig_H5))[0]
        self.dig_H6 = struct.unpack('b', bytes([self._read_u8(0xE7)]))[0]
        self.t_fine = 0

    # --- Raw data ---
    def _read_raw(self):
        data = self.i2c.readfrom_mem(self.addr, 0xF7, 8)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_h = (data[6] << 8) | data[7]
        return adc_t, adc_p, adc_h

    # --- Compensation formulas ---
    def _compensate_temperature(self, adc_t):
        var1 = (adc_t / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_t / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        self.t_fine = int(var1 + var2)
        return (var1 + var2) / 5120.0

    def _compensate_pressure(self, adc_p):
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 += var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0: return 0
        p = 1048576.0 - adc_p
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        return p + (var1 + var2 + self.dig_P7) / 16.0

    def _compensate_humidity(self, adc_h):
        h = self.t_fine - 76800.0
        h = (adc_h - (self.dig_H4 * 64.0 + (self.dig_H5 / 16384.0) * h)) * \
            (self.dig_H2 / 65536.0 * (1.0 + (self.dig_H3 / 67108864.0) * h * (1.0 + self.dig_H6 / 67108864.0 * h)))
        h = h * (1.0 - self.dig_H1 * h / 524288.0)
        return max(0.0, min(100.0, h))

    # --- Public API ---
    def read(self):
        adc_t, adc_p, adc_h = self._read_raw()
        T = self._compensate_temperature(adc_t)
        P = self._compensate_pressure(adc_p)
        H = self._compensate_humidity(adc_h)
        return (T, P, H)

# --- Example usage on Raspberry Pi Pico ---
if __name__ == "__main__":
    # Use I2C0 on GP0 (SDA) and GP1 (SCL)
    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

    # Scan bus to confirm address
    print("I2C scan:", [hex(x) for x in i2c.scan()])

    # Initialize sensor (change to 0x77 if needed)
    sensor = BME280(i2c, address=0x76)

    while True:
        t, p, h = sensor.read()
        print("Temp: {:.2f} °C | Pressure: {:.2f} hPa | Humidity: {:.2f} %".format(
            t, p / 100.0, h))
        time.sleep(1)