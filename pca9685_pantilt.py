import smbus2
import time

PCA9685_ADDR = 0x40
I2C_BUS = 7

MODE1     = 0x00
PRESCALE  = 0xFE
LED0_ON_L = 0x06

PAN_CENTER  = 145.0
TILT_CENTER = 28.0

bus = smbus2.SMBus(I2C_BUS)

def reset():
    bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
    time.sleep(0.01)

def set_pwm_freq(freq_hz):
    prescale_val = round(25_000_000.0 / (4096 * freq_hz)) - 1
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    bus.write_byte_data(PCA9685_ADDR, MODE1, (old_mode & 0x7F) | 0x10)
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, prescale_val)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0x80)

def set_pwm(channel, on, off):
    base = LED0_ON_L + 4 * channel
    bus.write_byte_data(PCA9685_ADDR, base,     on  & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, base + 1, on  >> 8)
    bus.write_byte_data(PCA9685_ADDR, base + 2, off & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, base + 3, off >> 8)

def angle_to_pulse(angle_deg, max_angle, freq=50):
    min_us = 500
    max_us = 2500
    period_us = 1_000_000 / freq
    pulse_us = min_us + (max_us - min_us) * angle_deg / max_angle
    return int(4096 * pulse_us / period_us)

def set_pan(angle_deg):
    """CH0: 20KG 270°サーボ (0〜270°)"""
    angle_deg = max(0, min(270, angle_deg))
    set_pwm(0, 0, angle_to_pulse(angle_deg, max_angle=270))

def set_tilt(angle_deg):
    """CH1: 25KG 180°サーボ (0〜180°)"""
    angle_deg = max(0, min(180, angle_deg))
    set_pwm(1, 0, angle_to_pulse(angle_deg, max_angle=180))

# --- メイン ---
reset()
set_pwm_freq(50)

print("PCA9685初期化完了 (bus 7, 0x40, 50Hz)")
print("CH0=pan 20KG 270°サーボ / CH1=tilt 25KG 180°サーボ")

print("センターへ移動中...")
set_pan(PAN_CENTER)
set_tilt(TILT_CENTER)
time.sleep(1.5)
print(f"センター位置: pan={PAN_CENTER}°, tilt={TILT_CENTER}°")

# 動作テスト: pan ±30°
print(f"\n--- panテスト ({PAN_CENTER-30:.0f}° → {PAN_CENTER:.0f}° → {PAN_CENTER+30:.0f}°) ---")
for angle in [PAN_CENTER-30, PAN_CENTER, PAN_CENTER+30, PAN_CENTER]:
    print(f"  pan → {angle:.0f}°")
    set_pan(angle)
    time.sleep(1.0)

# 動作テスト: tilt ±20°
print(f"\n--- tiltテスト ({TILT_CENTER-20:.0f}° → {TILT_CENTER:.0f}° → {TILT_CENTER+20:.0f}°) ---")
for angle in [TILT_CENTER-20, TILT_CENTER, TILT_CENTER+20, TILT_CENTER]:
    print(f"  tilt → {angle:.0f}°")
    set_tilt(angle)
    time.sleep(1.0)

print("\nテスト完了。センター位置でホールド中（Ctrl+Cで終了）")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

bus.close()
