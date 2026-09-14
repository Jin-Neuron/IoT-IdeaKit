import machine
import utime

# ==========================================
# 1. ピンの設定
# ==========================================
# サーボ設定
fl = machine.PWM(machine.Pin("ESC_SERVO_FL"))
fr = machine.PWM(machine.Pin("ESC_SERVO_FR"))
rl = machine.PWM(machine.Pin("ESC_SERVO_RL"))
rr = machine.PWM(machine.Pin("ESC_SERVO_RR"))

servo_array = [fl, fr, rl, rr]
current_angles = [90, 90, 90, 90] 
for servo in servo_array:
    servo.freq(50)

# 超音波センサーとLED設定
trig = machine.Pin("TRIG_TX", machine.Pin.OUT)
echo = machine.Pin("ECHO_RX", machine.Pin.IN)
red = machine.Pin("LED_L", machine.Pin.OUT)
blue = machine.Pin("LED_R", machine.Pin.OUT)

# ==========================================
# 2. センサー用関数
# ==========================================
def get_distance():
    """ 超音波センサーで距離(cm)を測る """
    trig.low()
    utime.sleep_us(2)
    trig.high()
    utime.sleep_us(10)
    trig.low()

    # パルス幅を計測 (タイムアウトを30000us = 約5mに設定してフリーズ防止)
    duration = machine.time_pulse_us(echo, 1, 30000)

    if duration < 0:
        return 999.0  # エラーやタイムアウト時は安全のため遠い距離を返す

    # 音速(0.0343 cm/us)を使って距離を計算
    distance = (duration * 0.0343) / 2
    return distance

# ==========================================
# 3. 歩行用基本関数
# ==========================================
def set_angle(servo_idx, angle):
    """ サーボを動かす関数（左右反転を自動吸収） """
    current_angles[servo_idx] = angle
    # 左側のサーボ（FL: 0, RL: 2）は角度を反転
    if servo_idx == 0 or servo_idx == 2:
        actual_angle = 180 - angle
    else:
        actual_angle = angle
        
    actual_angle = max(0, min(180, actual_angle))
    min_duty = 1638
    max_duty = 7864
    duty = int(min_duty + (max_duty - min_duty) * (actual_angle / 180))
    servo_array[servo_idx].duty_u16(duty)

def smooth_move_all(target_angles, step=5, delay_ms=10):
    """ 4本のサーボを同時に動かす """
    global current_angles
    while True:
        all_reached = True
        for i in range(4):
            target = target_angles[i]
            current = current_angles[i]
            if abs(target - current) <= step:
                set_angle(i, target)
            else:
                all_reached = False
                if target > current:
                    set_angle(i, current + step)
                else:
                    set_angle(i, current - step)
        if all_reached:
            break
        utime.sleep_ms(delay_ms)

# ==========================================
# 4. 歩行モーション関数
# ==========================================
TALL = 90
def tilt_walk(L_FWD, L_BCK, R_FWD, R_BCK, delay_ms=10):
    """ 重心移動歩行（前進・後退用） """
    fast, slow, pause = 5, 3, 50
    # フェーズ1 & 2
    smooth_move_all([L_FWD, TALL, TALL, R_FWD], step=fast, delay_ms=delay_ms)
    utime.sleep_ms(pause)
    smooth_move_all([TALL, R_BCK, L_BCK, TALL], step=slow, delay_ms=delay_ms)
    utime.sleep_ms(pause)
    # フェーズ3 & 4
    smooth_move_all([TALL, R_FWD, L_FWD, TALL], step=fast, delay_ms=delay_ms)
    utime.sleep_ms(pause)
    smooth_move_all([L_BCK, TALL, TALL, R_BCK], step=slow, delay_ms=delay_ms)
    utime.sleep_ms(pause)

def turn_right_super():
    """ 対角交互駆動による超信地旋回（右回り） """
    fast, slow = 15, 3
    smooth_move_all([130, 90, 90, 50], step=fast, delay_ms=10)
    utime.sleep_ms(30)
    smooth_move_all([50, 90, 90, 130], step=slow, delay_ms=10)
    utime.sleep_ms(30)
    smooth_move_all([90, 50, 130, 90], step=fast, delay_ms=10)
    utime.sleep_ms(30)
    smooth_move_all([90, 130, 50, 90], step=slow, delay_ms=10)
    utime.sleep_ms(30)

# ==========================================
# 5. メインループ（衝突回避ロジック）
# ==========================================
try:
    print("初期位置(90度)に移動します...")
    smooth_move_all([90, 90, 90, 90], step=2, delay_ms=10)
    red.value(0)
    blue.value(0)
    utime.sleep(1)

    while True:
        # 1. まず距離を測る
        dist = get_distance()
        print("Distance: {:.1f} cm".format(dist))

        # 2. 距離に応じて行動を変える
        if dist < 15.0:
            # --- 障害物あり！回避モード ---
            print("障害物発見！回避します。")
            blue.value(0)
            red.value(1)  # 赤色LED点灯
            
            # 少し後退して距離を取る
            for _ in range(2):
                tilt_walk(50, 130, 50, 130)
                
            # 右に旋回して方向を変える
            for _ in range(4):
                turn_right_super()
                
        else:
            # --- 障害物なし！前進モード ---
            print("前進中...")
            blue.value(1)  # 青色LED点灯
            red.value(0)
            
            # 1歩だけ進む（1歩進むごとにセンサーで確認するため）
            tilt_walk(130, 50, 130, 50)

except KeyboardInterrupt:
    print("停止処理中...")
    red.value(0)
    blue.value(0)
    for servo in servo_array:
        servo.deinit()