import smbus2
import time

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
    min_us, max_us = 500, 2500
    period_us = 1_000_000 / freq
    pulse_us = min_us + (max_us - min_us) * angle_deg / max_angle
    return int(4096 * pulse_us / period_us)

def set_pan(angle_deg):
    angle_deg = max(0, min(270, angle_deg))
    set_pwm(0, 0, angle_to_pulse(angle_deg, max_angle=270))
    return angle_deg

def set_tilt(angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    set_pwm(1, 0, angle_to_pulse(angle_deg, max_angle=180))
    return angle_deg

def print_status(pan, tilt, pan_center, tilt_center):
    print(f"\n現在位置  → pan: {pan:.1f}°  tilt: {tilt:.1f}°")
    print(f"センター  → pan: {pan_center:.1f}°  tilt: {tilt_center:.1f}°")

def print_help():
    print("\n=== コマンド一覧 ===")
    print("  p+N  / p-N  : panを N度 増減  (例: p+5, p-10)")
    print("  t+N  / t-N  : tiltを N度 増減 (例: t+5, t-10)")
    print("  p=N         : panを N度 に直接指定")
    print("  t=N         : tiltを N度 に直接指定")
    print("  c           : 現在位置をセンターとして確定")
    print("  r           : センター位置へ戻る")
    print("  h           : このヘルプを表示")
    print("  q           : 終了（センター値を表示）")
    print("===================")

# --- 初期化 ---
reset()
set_pwm_freq(50)
print("PCA9685初期化完了")

pan  = 135.0
tilt = 90.0
pan_center  = 135.0
tilt_center = 90.0

print(f"初期センター位置へ移動: pan={pan}°, tilt={tilt}°")
set_pan(pan)
set_tilt(tilt)
time.sleep(1.0)

print_help()
print_status(pan, tilt, pan_center, tilt_center)

while True:
    try:
        cmd = input("\nコマンド> ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        break

    if cmd == 'q':
        break
    elif cmd == 'h':
        print_help()
    elif cmd == 'c':
        pan_center  = pan
        tilt_center = tilt
        print(f"センター確定: pan={pan_center:.1f}°, tilt={tilt_center:.1f}°")
    elif cmd == 'r':
        pan  = set_pan(pan_center)
        tilt = set_tilt(tilt_center)
        print(f"センターへ戻りました")
    elif cmd.startswith('p+'):
        pan = set_pan(pan + float(cmd[2:]))
    elif cmd.startswith('p-'):
        pan = set_pan(pan - float(cmd[2:]))
    elif cmd.startswith('p='):
        pan = set_pan(float(cmd[2:]))
    elif cmd.startswith('t+'):
        tilt = set_tilt(tilt + float(cmd[2:]))
    elif cmd.startswith('t-'):
        tilt = set_tilt(tilt - float(cmd[2:]))
    elif cmd.startswith('t='):
        tilt = set_tilt(float(cmd[2:]))
    else:
        print("不明なコマンドです。h でヘルプ表示")
        continue

    print_status(pan, tilt, pan_center, tilt_center)

print(f"\n=== 調整結果 ===")
print(f"PAN_CENTER  = {pan_center:.1f}")
print(f"TILT_CENTER = {tilt_center:.1f}")
print("この値を pca9685_pantilt.py に反映してください")
bus.close()
