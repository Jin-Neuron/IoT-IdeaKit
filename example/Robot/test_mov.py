import machine
import utime, time

# サーボを接続したピン
# ※ 通常は machine.Pin(15) などの数値ですが、お使いの環境に合わせてそのままにしています
fl = machine.PWM(machine.Pin("ESC_SERVO_FL"))
fr = machine.PWM(machine.Pin("ESC_SERVO_FR"))
rl = machine.PWM(machine.Pin("ESC_SERVO_RL"))
rr = machine.PWM(machine.Pin("ESC_SERVO_RR"))

for servo in (fl, fr, rl, rr):
    servo.freq(50)

# サーボ配列と現在の角度（初期位置を90度と仮定）
servo_array = [fl, fr, rl, rr]
current_angles = [90, 90, 90, 90] 
TRIM = [0, 0, 0, 0] 

def set_angle(servo_idx, angle):
    """ 指定したインデックスのサーボを動かす関数 """
    current_angles[servo_idx] = angle
    
    # ★ ここでトリム（微調整）を加算
    angle_with_trim = angle + TRIM[servo_idx]
    
    # 左右の取り付け向きの反転を吸収
    if servo_idx == 0 or servo_idx == 2:
        actual_angle = 180 - angle_with_trim
    else:
        actual_angle = angle_with_trim
        
    actual_angle = max(0, min(180, actual_angle))
    
    min_duty = 1638  # 500μs相当
    max_duty = 7864  # 2400μs相当
    duty = int(min_duty + (max_duty - min_duty) * (actual_angle / 180))
    servo_array[servo_idx].duty_u16(duty)

def smooth_move_all(target_angles, step=5, delay_ms=10):
    """
    4つのサーボを同時に目標角度まで滑らかに動かす関数
    target_angles: [FL, FR, RL, RR] の目標角度リスト
    """
    global current_angles
    
    while True:
        all_reached = True
        for i in range(4):
            target = target_angles[i]
            current = current_angles[i]
            
            # 目標との差がstep以内なら直接目標値を入れる（無限ループ防止）
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

TALL = 90  # 足が一番長くなる角度（地面を強く押して背伸びする）

def tilt_walk(L_FWD, L_BCK, R_FWD, R_BCK, delay_ms=10):
    """
    重心を左右に揺さぶりながら歩く「チルト・トロット歩行」
    左右の歩幅を別々に指定できます。
    """
    fast = 8  # 足を浮かせて前に出すスピード
    slow = 5  # 地面を蹴って体を押し出すスピード
    pause = 50 # 揺れを収めるための待機時間

    # 順番は [左前(FL), 右前(FR), 左後(RL), 右後(RR)]

    # --- フェーズ1：左前と右後を前に出す ---
    # 右前と左後を真っ直ぐ(TALL)にして背伸びし、対角線に体重を乗せる
    # 体重が乗っていない左前と右後を、スッと前に出す
    smooth_move_all([L_FWD, TALL, TALL, R_FWD], step=fast, delay_ms=delay_ms)
    utime.sleep_ms(pause)

    # --- フェーズ2：地面を蹴って進む ---
    # 前にある左前・右後を真っ直ぐ(TALL)にして地面を掴み、体を持ち上げる。
    # 同時に、背伸びしていた右前・左後は後ろ(BCK)に倒れ込みながら地面を蹴る。
    smooth_move_all([TALL, R_BCK, L_BCK, TALL], step=slow, delay_ms=delay_ms)
    utime.sleep_ms(pause)

    # --- フェーズ3：右前と左後を前に出す ---
    # フェーズ2の終わりで左前・右後が真っ直ぐ(TALL)になっているので、今度はそちらに体重が乗っている
    # 体重が乗っていない右前と左後を、スッと前に出す
    smooth_move_all([TALL, R_FWD, L_FWD, TALL], step=fast, delay_ms=delay_ms)
    utime.sleep_ms(pause)

    # --- フェーズ4：地面を蹴って進む ---
    # 前にある右前・左後を真っ直ぐ(TALL)にし、左前・右後を後ろ(BCK)へ蹴る
    smooth_move_all([L_BCK, TALL, TALL, R_BCK], step=slow, delay_ms=delay_ms)
    utime.sleep_ms(pause)

def turn_right_super():
    """ 
    右への超信地旋回（対角交互駆動）
    その場でコマのように美しく回ります。
    """
    fast = 15
    slow = 3

    # --- 第1フェーズ：対角線A（FLとRR）で回る ---
    # 1. 準備：FL(左前)は前(130)、RR(右後)は後ろ(50)へ。他は90度。
    smooth_move_all([130, 90, 90, 50], step=fast, delay_ms=10)
    utime.sleep_ms(30)
    
    # 2. 蹴る：FLを後ろ(50)、RRを前(130)に蹴って右回転！
    smooth_move_all([50, 90, 90, 130], step=slow, delay_ms=10)
    utime.sleep_ms(30)

    # --- 第2フェーズ：対角線B（FRとRL）で回る ---
    # 3. 準備：FR(右前)は後ろ(50)、RL(左後)は前(130)へ。
    # 同時に、蹴り終わったFLとRRは90度に戻して次の軸にする。
    smooth_move_all([90, 50, 130, 90], step=fast, delay_ms=10)
    utime.sleep_ms(30)
    
    # 4. 蹴る：FRを前(130)、RLを後ろ(50)に蹴ってさらに右回転！
    smooth_move_all([90, 130, 50, 90], step=slow, delay_ms=10)
    utime.sleep_ms(30)


def turn_left_super():
    """ 
    左への超信地旋回（対角交互駆動）
    """
    fast = 15
    slow = 3

    # --- 第1フェーズ：対角線A（FLとRR）で回る ---
    # 1. 準備：FL(左前)は後ろ(50)、RR(右後)は前(130)へ。他は90度。
    smooth_move_all([50, 90, 90, 130], step=fast, delay_ms=10)
    utime.sleep_ms(30)
    
    # 2. 蹴る：FLを前(130)、RRを後ろ(50)に蹴って左回転！
    smooth_move_all([130, 90, 90, 50], step=slow, delay_ms=10)
    utime.sleep_ms(30)

    # --- 第2フェーズ：対角線B（FRとRL）で回る ---
    # 3. 準備：FR(右前)は前(130)、RL(左後)は後ろ(50)へ。他は90度。
    smooth_move_all([90, 130, 50, 90], step=fast, delay_ms=10)
    utime.sleep_ms(30)
    
    # 4. 蹴る：FRを後ろ(50)、RLを前(130)に蹴ってさらに左回転！
    smooth_move_all([90, 50, 130, 90], step=slow, delay_ms=10)
    utime.sleep_ms(30)

# --- メインループの書き換え ---
try:
    print("初期位置(90度)に移動します...")
    smooth_move_all([90, 90, 90, 90], step=8, delay_ms=10)
    utime.sleep(1)

    while True:
        # --- 右旋回 ---
        print("前進")
        for _ in range(10):
            tilt_walk(130, 50, 130, 50)
            
        # --- 右旋回 ---
        print("右旋回")
        for _ in range(10):
            turn_right_super()
            
        # --- 左旋回 ---
        print("後進")
        for _ in range(10):
            tilt_walk(50, 130, 50, 130)

        # --- 右旋回 ---
        print("右旋回")
        for _ in range(10):
            turn_right_super()
            
except KeyboardInterrupt:
    print("停止処理中...")
    for servo in servo_array:
        servo.deinit()