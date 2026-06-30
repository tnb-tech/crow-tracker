import smbus2
import time
import math
import argparse

# --- PCA9685設定 ---
PCA9685_ADDR = 0x40
I2C_BUS      = 7
MODE1        = 0x00
PRESCALE     = 0xFE
LED0_ON_L    = 0x06

PAN_CENTER   = 145.0
TILT_CENTER  = 28.0
PAN_MIN,  PAN_MAX  = 0.0, 270.0
TILT_MIN, TILT_MAX = 0.0,  98.0

# --- カメラ・画像設定 ---
IMG_W        = 640
IMG_H        = 480
H_FOV        = 92.0
V_FOV        = H_FOV * (IMG_H / IMG_W)
DEG_PER_PX_X = H_FOV / IMG_W
DEG_PER_PX_Y = V_FOV / IMG_H

DEADBAND_PX = 10
TARGET_CLASSES = [0, 1, 2]  # 0:Corvus-Splendens 1:Crow 2:Magpie (sparrow除外)

# --- 検出・追尾パラメータ ---
MIN_DETECT_FRAMES  = 3     # 連続N回検出で追尾開始
HOLD_SECONDS       = 2.0   # 見失い後ホールドする秒数
RETURN_DEG_PER_SEC = 15.0  # センターへ戻る速度（度/秒）
CONF_THRESHOLD     = 0.5   # 信頼度閾値

CLASS_NAMES = {0: "Corvus", 1: "Crow", 2: "Magpie"}


# =====================
#  PIDコントローラー
# =====================
class PIDController:
    def __init__(self, kp, ki, kd, output_limit=30.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def update(self, error_px, deg_per_px, now):
        error_deg = error_px * deg_per_px
        dt = (now - self._prev_time) if self._prev_time else 0.033
        dt = max(dt, 1e-6)
        self._integral += error_deg * dt
        derivative      = (error_deg - self._prev_error) / dt
        output = (self.kp * error_deg
                + self.ki * self._integral
                + self.kd * derivative)
        output = max(-self.output_limit, min(self.output_limit, output))
        self._prev_error = error_deg
        self._prev_time  = now
        return output


# =====================
#  サーボ制御
# =====================
class PanTiltController:
    def __init__(self):
        self.bus  = smbus2.SMBus(I2C_BUS)
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER
        self._init()

    def _init(self):
        self.bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
        time.sleep(0.01)
        freq     = 50
        prescale = round(25_000_000.0 / (4096 * freq)) - 1
        old      = self.bus.read_byte_data(PCA9685_ADDR, MODE1)
        self.bus.write_byte_data(PCA9685_ADDR, MODE1, (old & 0x7F) | 0x10)
        self.bus.write_byte_data(PCA9685_ADDR, PRESCALE, prescale)
        self.bus.write_byte_data(PCA9685_ADDR, MODE1, old)
        time.sleep(0.005)
        self.bus.write_byte_data(PCA9685_ADDR, MODE1, old | 0x80)

    def _set_pwm(self, ch, off_count):
        base = LED0_ON_L + 4 * ch
        self.bus.write_byte_data(PCA9685_ADDR, base,     0)
        self.bus.write_byte_data(PCA9685_ADDR, base + 1, 0)
        self.bus.write_byte_data(PCA9685_ADDR, base + 2, off_count & 0xFF)
        self.bus.write_byte_data(PCA9685_ADDR, base + 3, off_count >> 8)

    def _angle_to_count(self, angle, max_angle):
        pulse_us = 500 + 2000 * angle / max_angle
        return int(4096 * pulse_us / 20000)

    def move(self, pan=None, tilt=None):
        if pan is not None:
            self.pan = max(PAN_MIN, min(PAN_MAX, pan))
            self._set_pwm(0, self._angle_to_count(self.pan, 270))
        if tilt is not None:
            self.tilt = max(TILT_MIN, min(TILT_MAX, tilt))
            self._set_pwm(1, self._angle_to_count(self.tilt, 180))

    def return_to_center(self, dt):
        step = RETURN_DEG_PER_SEC * dt
        pan_diff  = PAN_CENTER  - self.pan
        tilt_diff = TILT_CENTER - self.tilt
        new_pan  = self.pan  + (step if pan_diff  > 0 else -step) if abs(pan_diff)  > step else PAN_CENTER
        new_tilt = self.tilt + (step if tilt_diff > 0 else -step) if abs(tilt_diff) > step else TILT_CENTER
        self.move(new_pan, new_tilt)

    def center(self):
        self.move(PAN_CENTER, TILT_CENTER)

    def close(self):
        self.bus.close()


# =====================
#  オーバーレイ描画
# =====================
def draw_overlay(cv2, frame, boxes, best_box, tx, ty, status, servo):
    # 全検出ボックスを薄い色で描画
    for b in boxes:
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        cls_id = int(b.cls[0])
        label = "%s %.2f" % (CLASS_NAMES.get(cls_id, "?"), float(b.conf[0]))
        cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
        cv2.putText(frame, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

    # 追尾対象ボックスを緑で描画
    if best_box is not None:
        x1, y1, x2, y2 = [int(v) for v in best_box.xyxy[0].tolist()]
        cls_id = int(best_box.cls[0])
        conf   = float(best_box.conf[0])
        label  = "%s %.2f" % (CLASS_NAMES.get(cls_id, "?"), conf)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        # ターゲット中心に十字マーク
        cv2.drawMarker(frame, (int(tx), int(ty)), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

    # 画面中心の十字
    cx, cy = IMG_W // 2, IMG_H // 2
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)

    # デッドバンド円
    cv2.circle(frame, (cx, cy), DEADBAND_PX, (80, 80, 80), 1)

    # 状態テキスト（左上）
    cv2.putText(frame, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

    # pan/tilt角度（左下）
    angle_text = "pan: %.1f  tilt: %.1f" % (servo.pan, servo.tilt)
    cv2.putText(frame, angle_text, (10, IMG_H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    return frame


# =====================
#  追尾メインループ
# =====================
def run_tracker(simulate=False, no_display=False):
    import cv2
    from ultralytics import YOLO

    servo = PanTiltController()
    servo.center()
    time.sleep(1.0)
    print("センター位置: pan=%.1f deg, tilt=%.1f deg" % (PAN_CENTER, TILT_CENTER))

    pid_pan  = PIDController(kp=0.3, ki=0.001, kd=0.02, output_limit=30.0)
    pid_tilt = PIDController(kp=0.3, ki=0.001, kd=0.02, output_limit=20.0)

    model = YOLO("yolo11s_crow.engine", task="detect")
    print("カスタムカラスモデルロード完了（Corvus/Crow/Magpie）")

    if simulate:
        print("\n=== シミュレーションモード ===")
        print("仮想ターゲットが世界座標を動き回ります（Ctrl+Cで終了）\n")
        t0 = time.time()

        sim_consecutive = 0
        sim_last_detect = None
        prev_time = time.time()

        try:
            while True:
                now     = time.time()
                dt      = now - prev_time
                prev_time = now
                elapsed = now - t0

                target_pan_world  = PAN_CENTER  + 90.0 * math.sin(elapsed * 0.4)
                target_tilt_world = TILT_CENTER + 50.0 * math.sin(elapsed * 0.3)

                err_x = (target_pan_world  - servo.pan)  / DEG_PER_PX_X
                err_y = (target_tilt_world - servo.tilt) / DEG_PER_PX_Y
                err_x = max(-IMG_W / 2, min(IMG_W / 2, err_x))
                err_y = max(-IMG_H / 2, min(IMG_H / 2, err_y))
                tx = int(IMG_W / 2 + err_x)
                ty = int(IMG_H / 2 + err_y)

                in_frame = (0 < tx < IMG_W and 0 < ty < IMG_H)

                if in_frame:
                    sim_consecutive += 1
                    sim_last_detect  = now
                    if sim_consecutive >= MIN_DETECT_FRAMES:
                        if abs(err_x) > DEADBAND_PX:
                            servo.pan += pid_pan.update(err_x, DEG_PER_PX_X, now)
                        else:
                            pid_pan.reset()
                        if abs(err_y) > DEADBAND_PX:
                            servo.tilt += pid_tilt.update(err_y, DEG_PER_PX_Y, now)
                        else:
                            pid_tilt.reset()
                        servo.move(servo.pan, servo.tilt)
                        status = "追尾中(%df)" % sim_consecutive
                    else:
                        status = "確認中(%d/%d)" % (sim_consecutive, MIN_DETECT_FRAMES)
                else:
                    sim_consecutive = 0
                    pid_pan.reset()
                    pid_tilt.reset()
                    if sim_last_detect is None:
                        status = "待機中"
                    elif now - sim_last_detect < HOLD_SECONDS:
                        status = "ホールド(%.1fs)" % (now - sim_last_detect)
                    else:
                        servo.return_to_center(dt)
                        status = "センターへ戻り中"

                print("world=(%6.1f,%5.1f)  frame=(%3d,%3d)  err=(%+5.1f,%+5.1f)  pan=%6.1f  tilt=%5.1f  [%s]" % (
                    target_pan_world, target_tilt_world, tx, ty,
                    err_x, err_y, servo.pan, servo.tilt, status))
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
        display = not no_display
        print("\n=== カメラ追尾モード開始（Ctrl+Cで終了）===")
        print("連続%d回検出で追尾開始、見失い後%.1fs ホールド後センター復帰" % (
            MIN_DETECT_FRAMES, HOLD_SECONDS))
        print("映像表示: %s\n" % ("ON（qキーで終了）" if display else "OFF"))

        consecutive  = 0
        last_detect  = None
        prev_time    = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("カメラ読み取りエラー")
                    break

                now = time.time()
                dt  = now - prev_time
                prev_time = now

                results = model(frame, classes=TARGET_CLASSES, conf=CONF_THRESHOLD, verbose=False)
                boxes   = results[0].boxes
                best_box = None
                tx, ty   = IMG_W / 2, IMG_H / 2

                if len(boxes) > 0:
                    best_box = max(boxes, key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0])
                                                       * (b.xyxy[0][3] - b.xyxy[0][1]))
                    x1, y1, x2, y2 = best_box.xyxy[0].tolist()
                    tx   = (x1 + x2) / 2
                    ty   = (y1 + y2) / 2
                    conf = float(best_box.conf[0])

                    consecutive += 1
                    last_detect  = now

                    if consecutive >= MIN_DETECT_FRAMES:
                        err_x = tx - IMG_W // 2
                        err_y = ty - IMG_H // 2
                        if abs(err_x) > DEADBAND_PX:
                            servo.pan -= pid_pan.update(err_x, DEG_PER_PX_X, now)
                        else:
                            pid_pan.reset()
                        if abs(err_y) > DEADBAND_PX:
                            servo.tilt -= pid_tilt.update(err_y, DEG_PER_PX_Y, now)
                        else:
                            pid_tilt.reset()
                        servo.move(servo.pan, servo.tilt)
                        status = "追尾中 %df" % consecutive
                        print("conf=%.2f  target=(%.0f,%.0f)  err=(%+.0f,%+.0f)  pan=%.1f  tilt=%.1f  [%s]" % (
                            conf, tx, ty, tx - IMG_W//2, ty - IMG_H//2,
                            servo.pan, servo.tilt, status))
                    else:
                        status = "確認中 %d/%d" % (consecutive, MIN_DETECT_FRAMES)
                        print("conf=%.2f  [%s]" % (conf, status))

                else:
                    consecutive = 0
                    pid_pan.reset()
                    pid_tilt.reset()

                    if last_detect is None:
                        status = "待機中"
                    elif now - last_detect < HOLD_SECONDS:
                        status = "ホールド中 %.1f/%.1fs" % (now - last_detect, HOLD_SECONDS)
                    else:
                        servo.return_to_center(dt)
                        status = "センターへ戻り中"
                    print("[%s] pan=%.1f  tilt=%.1f" % (status, servo.pan, servo.tilt))

                if display:
                    frame = draw_overlay(cv2, frame, boxes, best_box, tx, ty, status, servo)
                    cv2.imshow("crow tracker", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            if display:
                cv2.destroyAllWindows()

    servo.center()
    servo.close()
    print("\n終了。センター位置へ戻りました。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate",   action="store_true",
                        help="カメラなしでPID動作をシミュレーション")
    parser.add_argument("--no-display", action="store_true",
                        help="映像ウィンドウを表示しない（SSH実行時）")
    args = parser.parse_args()
    run_tracker(simulate=args.simulate, no_display=args.no_display)
