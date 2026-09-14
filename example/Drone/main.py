import machine
import struct
import rp2
import time
import fc_core
from BNO055 import BNO055

pin_scl = machine.Pin("I2C_SCL")
pin_sda = machine.Pin("I2C_SDA")

isStarted = False
exec_time_us = 0
interval_us = 0
t_last_irq = 0

MAX_PITCH = 180
MAX_ROLL = 90

def dma_rx_irq_handler(dma_obj):
    global isStarted, exec_time_us, interval_us, t_last_irq
    global pitch, roll, sensor

    t_now = time.ticks_us()
    
    if t_last_irq > 0:
        interval_us = time.ticks_diff(t_now, t_last_irq)

    t_start = time.ticks_us()

    # 完了したチャンネルに応じてバッファを取得 & 次回周回用に再装填
    if dma_obj.channel == sensor.rx_dma_a.channel:
        raw = sensor.rx_buf_a
    else:
        raw = sensor.rx_buf_b
    
    fc_core.set_sensor_buffer(raw)
    dma_obj.write = raw
    dma_obj.count = len(raw)

    if isStarted:
        fc_core.process()

    if not sensor.dma_ch.active():
        sensor.dma_ch.read = sensor.tx_cmds
        sensor.dma_ch.count = len(sensor.tx_cmds)
        sensor.dma_ch.active(1)
        
    exec_time_us = time.ticks_diff(time.ticks_us(), t_start)
    
    t_last_irq = time.ticks_us()

    '''# 6バイトの受信完了！(CPUを介さずに rx_buf に直接入っている)
    heading, roll, pitch = struct.unpack("<hhh", raw)
    heading /= 16.0
    pitch /= 16.0
    roll /= 16.0

    print(f"heading: {heading:6.2f} | roll: {roll:6.2f} | pitch: {pitch:6.2f}", end="\r")'''



print("BNO055をIMUモード(6軸)で初期化しています...")

sensor = BNO055(0, pin_scl, pin_sda, address=0x29)

# ファイルが存在すれば読み込み、無ければスキップ
if not sensor.load_calibration_from_file():
    print("Warning: Running with default/raw offsets.")

sensor.configure(dma_rx_irq_handler)

fc_core.set_sensor_buffer(sensor.rx_buf_a)

print("rp2.DMA + pack_ctrl Ring Buffer Autonomous Stream Started.")

# ==========================================
# 1. モーター用PIOステートマシンの定義と起動
# ==========================================
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW)
def motor_pwm():
    wrap_target()

    pull(noblock)                       # [1] FIFOから取得(無ければ前回値を保持)
    mov(x, osr)
    mov(isr, x)

    label("high_loop")
    jmp(x_dec, "high_loop") .side(1)    # [2] x = パルス幅 (us)

    # --- LOW (オフ時間) 出力部 ---
    set(y, 31)              .side(0)    # [4] Pin = LOW# [5] 外側カウンタ (31)

    label("outer_low")
    set(x, 31)                          # [6] 内側カウンタ (31) -> 31 * 31 ≈ 1000 ループ以上

    label("inner_low")
    jmp(x_dec, "inner_low")

    jmp(y_dec, "outer_low")

    mov(x, isr)
    wrap()
    
# モーター出力用ピンの割り当て（ご自身の配線に合わせて変更してください）
pin_rr = machine.Pin("ESC_SERVO_RR", machine.Pin.OUT)
pin_fr = machine.Pin("ESC_SERVO_FR", machine.Pin.OUT)
pin_rl = machine.Pin("ESC_SERVO_RL", machine.Pin.OUT)
pin_fl = machine.Pin("ESC_SERVO_FL", machine.Pin.OUT)

freq = 1000000  # PIOクロック
sm_rr = rp2.StateMachine(4, motor_pwm, freq=freq, sideset_base=pin_rr)
sm_fr = rp2.StateMachine(5, motor_pwm, freq=freq, sideset_base=pin_fr)
sm_rl = rp2.StateMachine(6, motor_pwm, freq=freq, sideset_base=pin_rl)
sm_fl = rp2.StateMachine(7, motor_pwm, freq=freq, sideset_base=pin_fl)

# 各ステートマシンの初期化と起動（最大PWM周期：2000us）
for sm in (sm_rr, sm_fr, sm_rl, sm_fl):
    sm.put(1000)
    sm.active(1)

print("Motor PIO State Machines Initialized.")

# 引数: PIOインスタンス(1), SM_RR(0), SM_FR(1), SM_RL(2), SM_FL(3)
fc_core.init_hardware(1, 0, 1, 2, 3)

# PIDパラメータと dt（割り込み周期）のセット
# 例: Kp_roll=0.5, Kp_pitch=0.5, Kd_roll=0.2, Kd_pitch=0.2, dt=0.001 (1000Hz)
fc_core.set_pid(0.8, 0.7, 0.1, 0.1)

# 現在の姿勢をキャリブレーション
fc_core.calibrate()
print("Zero point calibrated.")

MAX_TILT_DEG = 20.0

print("FC Core & Hardware Initialized. System ready.")

base_throttle_start = 1300.0
base_throttle_max = 1370.0

RAMP_UP_DURATION = 0.5  # 離陸時のランプアップ時間（秒）
HOVER_DURATION = 50
LANDING_DURATION = 0.8

try:
    print("ARMING MOTORS...")
    fc_core.set_armed(True)

    # ★ここでオペレーターの入力を確実に待機する
    input("Press ENTER to ARM and start takeoff sequence...")

    time.sleep(2)
    isStarted = True
    sensor.activate()
    start_time = time.ticks_ms()
    
    while True:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
        
        # --- 離陸時のランプアップ制御 ---
        if elapsed < RAMP_UP_DURATION:
            # RAMP_UP_DURATION 秒かけてベーススロットルを1250から1400へ滑らかに引き上げる
            throttle = base_throttle_start + (base_throttle_max - base_throttle_start) * (elapsed / RAMP_UP_DURATION)
            fc_core.set_throttle(throttle)
        elif elapsed < (RAMP_UP_DURATION + HOVER_DURATION):
            # 2. ホバリング維持
            fc_core.set_throttle(base_throttle_max)

        elif elapsed < (RAMP_UP_DURATION + HOVER_DURATION + LANDING_DURATION):
            # 3. 着地ランプダウン
            land_elapsed = elapsed - (RAMP_UP_DURATION + HOVER_DURATION)
            throttle = base_throttle_max - (
                base_throttle_max - base_throttle_start
            ) * (land_elapsed / LANDING_DURATION)
            fc_core.set_throttle(throttle)
        else:
            # 4. 着地完了・安全停止
            fc_core.set_throttle(base_throttle_start)
            fc_core.set_armed(False)
            isStarted = False
            print("\nFLIGHT COMPLETED: SAFELY DISARMED.")
            break

        # --- 安全監視（キルスイッチ） ---
        roll, pitch = fc_core.get_angles()
        #if abs(roll) > MAX_TILT_DEG or abs(pitch) > MAX_TILT_DEG:
        #    raise RuntimeError(
        #        f"Excessive tilt detected (Roll: {roll:.1f}°, Pitch: {pitch:.1f}°)"
        #    )

        # 呼び出し間隔から周波数 (Hz) を計算 (0除算防止)
        freq_hz = (1_000_000 / interval_us) if interval_us > 0 else 0

        print(
            f"Elapsed: {elapsed:4.1f}s | Roll: {roll:5.1f}°, Pitch: {pitch:5.1f}° | "
            f"Interval: {interval_us:4d}us ({freq_hz:4.0f}Hz) | IRQ Exec: {exec_time_us:.1f}us"
        )
        
        # Python側は制御ループから解放されているため、適度に休止してCPUを専有しない
        time.sleep_ms(10)

except KeyboardInterrupt:
    print("\nUSER INTERRUPT: SAFELY DISARMED.")

except Exception as e:
    print(f"\nERROR OCCURRED: {e}. SAFELY DISARMED.")

finally:
    isStarted = False
    fc_core.set_armed(False)
    sensor.deactivate()
    
    for sm in (sm_rr, sm_fr, sm_rl, sm_fl):
        sm.active(0)

    time.sleep(2)