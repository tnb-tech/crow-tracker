# crow-tracker

カラス追い払いシステム - Yahboom 2DOF-PTZ + PCA9685 + YOLO11s on Jetson Orin Nano Super

## ハードウェア構成
- Jetson Orin Nano Super（Ubuntu 22.04 / JetPack 6.2.2）
- Yahboom 2DOF-PTZ（パン20KG 270° + チルト25KG 180°）
- PCA9685（I2C bus 7, address 0x40）
- ELP-USBFHD05MT-KL36IR-JP USBカメラ（92° FOV、1080p）

## モデル
- yolo11s_crow.engine（TensorRT FP16、640x640）
- クラス：Corvus-Splendens / Crow / Magpie（sparrow除外）

## ファイル構成
| ファイル | 説明 |
|----------|------|
| crow_tracker.py | メイン追尾スクリプト |
| pca9685_pantilt.py | サーボ基本動作テスト |
| pca9685_sweep_test.py | スイープ動作テスト |
| pca9685_calibrate.py | センター位置調整ツール |

## 実行方法
```bash
# カメラ追尾モード
python3 crow_tracker.py

# シミュレーションモード（カメラなし）
python3 crow_tracker.py --simulate
```

## サーボ設定
- PAN_CENTER=145.0°、TILT_CENTER=28.0°
- pan増加=左旋回、tilt増加=上向き
