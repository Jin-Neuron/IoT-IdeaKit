import time
from machine import Pin, PWM

# ==========================================
# 2. PWMの初期設定
# ==========================================
# 扱いやすいようにリスト化しておきます
motors = [
    PWM(Pin("ESC_SERVO_FL")),
    PWM(Pin("ESC_SERVO_FR")),
    PWM(Pin("ESC_SERVO_RL")),
    PWM(Pin("ESC_SERVO_RR"))
]

# すべてのモーターを50Hz (周期20,000µs) に設定
for m in motors:
    m.freq(50)

# ==========================================
# 3. 制御用関数（4つ同時に信号を送る）
# ==========================================
def set_throttle_all_us(us):
    """4つのモーターすべてに同じパルス幅(us)を出力する"""
    duty = int((us / 20000) * 65535)
    for m in motors:
        m.duty_u16(duty)

set_throttle_all_us(1000)
print("（「テレテレッテレー♪」という完了メロディが鳴れば成功です）")
time.sleep(5)

print("\n【Step 4】テスト回転（1080µs）")
print("※4つのモーターが同時に回転します！")
#set_throttle_all_us(1600)
time.sleep(20)

print("\n【Step 5】モーターを停止します。")
set_throttle_all_us(1000)
print("完了しました。")

'''for speed in range(1000, 1500, 50):
    set_throttle_all_us(speed)
    time.sleep(0.5)
    
print("停止します")
set_throttle_all_us(1000)'''
