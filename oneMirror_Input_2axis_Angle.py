import optoMDC
import time
import math
from AngleOutput import get_phi_theta

# 角度からXY座標への変換関数
def angle_to_xy(angle):
    return math.tan(math.radians(angle)) / math.tan(math.radians(50))

# ユーザー入力をクランプする関数
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

# COMポート定義
port = "COM4"

# optoMDCライブラリを通じてデバイスに接続
mre2 = optoMDC.connect(port=port)

# 縦方向はX軸、上方向回転が－、下方向回転が＋
# Channel_0 (X軸) の制御モードをXYモード（Closed Loop）に設定
ch_0 = mre2.Mirror.Channel_0
ch_0.SetControlMode(optoMDC.Units.XY)
static_input_0 = ch_0.StaticInput

# 水平方向はY軸、左方向回転が－、右方向回転が＋
# Channel_1 (Y軸) の制御モードをXYモード（Closed Loop）に設定
ch_1 = mre2.Mirror.Channel_1
ch_1.SetControlMode(optoMDC.Units.XY)
static_input_1 = ch_1.StaticInput

try:
    for phi_value, theta_value in get_phi_theta():
        if phi_value is None or theta_value is None:
            break

        xy_value_x = clamp(angle_to_xy(-theta_value), -1, 1)
        xy_value_y = clamp(angle_to_xy(-
                                       phi_value), -1, 1)


        static_input_0.SetXY(xy_value_x)
        static_input_1.SetXY(xy_value_y)

        # 設定後、0.1秒待機
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

# デバイスから切断
mre2.disconnect()
