# bno055.py
import time
from machine import Pin, PWM, I2C
from BNO055 import BNO055

## ==========================================
# 2. PWMの初期設定
# ==========================================
# 扱いやすいようにリスト化しておきます
motor_rr = PWM(Pin("ESC_SERVO_RR"))
motor_fr = PWM(Pin("ESC_SERVO_FR"))
motor_rl = PWM(Pin("ESC_SERVO_RL"))
motor_fl = PWM(Pin("ESC_SERVO_FL"))

motors = [motor_rr, motor_fr, motor_rl, motor_fl]
# すべてのモーターを50Hz (周期20,000µs) に設定
for m in motors:
    m.freq(50)

def set_motor_us(motor, us):
    # 安全のため、このテストでは出力を1000〜1200に制限
    us = max(1000, min(1200, us))
    duty = int((us / 20000) * 65535)
    motor.duty_u16(duty)

def emergency_stop():
    """全モーターを強制停止"""
    for m in motors:
        set_motor_us(m, 1000)

# ==========================================
# 2. ESCの初期化（アーミング）シーケンス
# ==========================================
print("=== フェーズ1: ESCアーミング ===")
print("Picoからスロットルゼロ(1000µs)を出力しています。")
emergency_stop() # 全モーターに1000usを出力

print("\n>>> 今すぐ、LiPoバッテリーをドローンに接続してください <<<")
# time.sleep(5) を input関数 に変更
input("ESCから「ピー、ピー（起動完了音）」が鳴り終わったら、Enterを押して次へ進んでください...")

print("【完了】ESCの安全ロックが解除されました。\n")

# ==========================================
# 3. センサー初期化とキャリブレーション
# ==========================================
print("=== フェーズ2: センサー初期化と水平出し ===")
i2c = I2C(0, sda=Pin("I2C_SDA"), scl=Pin("I2C_SCL"), freq=400000)
sensor = BNO055(i2c, address=0x29)

print("機体を平らな机に置き、キャリブレーション（ACC:3）が完了するまで動かさないでください。")

while True:
    sys, gyr, acc, mag = sensor.calibration_status()
    heading, roll, pitch = sensor.read_euler()
    
    print(f"状態 [SYS:{sys} GYR:{gyr} ACC:{acc}] | Roll:{roll:5.1f}° Pitch:{pitch:5.1f}°  ", end="\r")
    
    if acc == 3:
        print("\n【完了】キャリブレーション完了！ 水平基準が設定されました。\n")
        break
    time.sleep(0.1)

# ==========================================
# 4. PID制御（反発テスト）パラメータ
# ==========================================
BASE_THROTTLE = 1040   # 軽く回る程度のベース推力
TEST_DURATION = 15.0   # テスト時間（秒）

# Pゲイン：1度傾いたときに何µs出力を変えるか
Kp_roll = 3.0
Kp_pitch = 3.0

TARGET_ROLL = 0.0
TARGET_PITCH = 0.0

print("=== フェーズ3: PID反発テスト ===")
print("【安全機能】機体が40度以上傾いた場合、自動で全停止します。")
input("準備ができたら Enter を押してモーターを回転させてください...")

start_time = time.time()

try:
    print("\nテスト開始！ 機体をケーブルの範囲で前後左右に少し傾けてみてください。")
    print("「傾けた側のモーターが強く回り、押し返してくる感覚」があれば成功です。")
    
    while time.time() - start_time < TEST_DURATION:
        # センサーから現在の角度を取得
        heading, roll, pitch = sensor.read_euler()
        
        # 【安全装置】 異常な傾きを検知したらループを抜けて停止
        if abs(roll) > 40 or abs(pitch) > 40:
            print(f"\n【緊急停止】限界角度オーバー (Roll:{roll:.1f}, Pitch:{pitch:.1f})")
            break
            
        # エラー（目標とのズレ）を計算
        error_roll = TARGET_ROLL - roll
        error_pitch = TARGET_PITCH - pitch
        
        # PID計算 (P制御のみ)
        pid_roll = error_roll * Kp_roll
        pid_pitch = error_pitch * Kp_pitch
        
        # ミキシング (センサーの取り付け向きによって足し引きが逆になる場合があります)
        out_fl = BASE_THROTTLE - pid_roll - pid_pitch
        out_fr = BASE_THROTTLE + pid_roll - pid_pitch
        out_rl = BASE_THROTTLE - pid_roll + pid_pitch
        out_rr = BASE_THROTTLE + pid_roll + pid_pitch
        
        # モーターへ出力
        set_motor_us(motor_fl, out_fl)
        set_motor_us(motor_fr, out_fr)
        set_motor_us(motor_rl, out_rl)
        set_motor_us(motor_rr, out_rr)
        
        print(f"R:{roll:5.1f} P:{pitch:5.1f} | FL:{int(out_fl)} FR:{int(out_fr)} RL:{int(out_rl)} RR:{int(out_rr)}  ", end="\r")
        
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n手動で中断されました。")
finally:
    # どんなエラーや終了時でも必ず止める
    emergency_stop()
    print("\nモーターを安全に停止しました。バッテリーを外してください。")