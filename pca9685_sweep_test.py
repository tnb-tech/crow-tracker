import smbus2
import time
import math

PCA9685_ADDR = 0x40
I2C_BUS = 7

MODE1     = 0x00
PRESCALE  = 0xFE
LED0_ON_L = 0x06

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
    angle_deg = max(0, min(270, angle_deg))
    set_pwm(0, 0, angle_to_pulse(angle_deg, max_angle=270))

def set_tilt(angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    set_pwm(1, 0, angle_to_pulse(angle_deg, max_angle=180))

def smooth_move(set_fn, start, end, steps=50, interval=0.02):
    """startからendへsteps分割でなめらかに移動"""
    for i in range(steps + 1):
        angle = start + (end - start) * i / steps
        set_fn(angle)
        time.sleep(interval)

# --- 初期化 ---
reset()
set_pwm_freq(50)
print("PCA9685初期化完了 (bus 7, 0x40, 50Hz)")

# センターへ移動
print("センターへ移動中... pan=135°, tilt=90°")
smooth_move(set_pan, 0, 145, steps=30)
smooth_move(set_tilt, 0, 28, steps=20)
time.sleep(1.0)

# --- スイープテスト ---
SWEEP_CYCLES = 2       # 往復回数
STEP_INTERVAL = 0.02   # ステップ間隔(秒) ← 遅くするなら0.04等に変更

print("\n=== スイープテスト開始 ===")

# 1) panスイープ（tilt固定90°）
print(f"\n[1] pan スイープ: 45° ↔ 225° × {SWEEP_CYCLES}往復（tilt固定90°）")
for i in range(SWEEP_CYCLES):
    print(f"  往路 {i+1}")
    smooth_move(set_pan, 135, 225, steps=60, interval=STEP_INTERVAL)
    time.sleep(0.3)
    print(f"  復路 {i+1}")
    smooth_move(set_pan, 225, 45, steps=120, interval=STEP_INTERVAL)
    time.sleep(0.3)
    smooth_move(set_pan, 45, 135, steps=60, interval=STEP_INTERVAL)
    time.sleep(0.5)

# センター戻し
set_pan(135)
time.sleep(0.5)

# 2) tiltスイープ（pan固定135°）
print(f"\n[2] tilt スイープ: 20° ↔ 160° × {SWEEP_CYCLES}往復（pan固定135°）")
for i in range(SWEEP_CYCLES):
    print(f"  往路 {i+1}")
    smooth_move(set_tilt, 90, 160, steps=50, interval=STEP_INTERVAL)
    time.sleep(0.3)
    print(f"  復路 {i+1}")
    smooth_move(set_tilt, 160, 20, steps=100, interval=STEP_INTERVAL)
    time.sleep(0.3)
    smooth_move(set_tilt, 20, 90, steps=50, interval=STEP_INTERVAL)
    time.sleep(0.5)

# センター戻し
set_tilt(90)
time.sleep(0.5)

# 3) pan+tilt同時スイープ
print(f"\n[3] pan+tilt 同時スイープ × {SWEEP_CYCLES}往復")
for i in range(SWEEP_CYCLES):
    print(f"  対角往路 {i+1}: pan 45→225°, tilt 20→160°")
    steps = 80
    pan_start, pan_end = 135, 225
    tilt_start, tilt_end = 90, 160
    for s in range(steps + 1):
        t = s / steps
        set_pan(pan_start + (pan_end - pan_start) * t)
        set_tilt(tilt_start + (tilt_end - tilt_start) * t)
        time.sleep(STEP_INTERVAL)
    time.sleep(0.3)
    print(f"  対角復路 {i+1}: pan 225→45°, tilt 160→20°")
    pan_start, pan_end = 225, 45
    tilt_start, tilt_end = 160, 20
    for s in range(steps + 1):
        t = s / steps
        set_pan(pan_start + (pan_end - pan_start) * t)
        set_tilt(tilt_start + (tilt_end - tilt_start) * t)
        time.sleep(STEP_INTERVAL)
    time.sleep(0.3)
    smooth_move(set_pan, 45, 135, steps=40, interval=STEP_INTERVAL)
    smooth_move(set_tilt, 20, 90, steps=40, interval=STEP_INTERVAL)
    time.sleep(0.5)

# センター戻し
print("\nセンター位置へ戻します...")
smooth_move(set_pan, 135, 135, steps=1)
smooth_move(set_tilt, 90, 90, steps=1)
set_pan(135)
set_tilt(90)
time.sleep(1.0)
print("=== スイープテスト完了 ===")

bus.close()
