import machine
import rp2
import time
import fc_core
from BNO055 import BNO055

pin_scl = machine.Pin("I2C_SCL")
pin_sda = machine.Pin("I2C_SDA")

print("BNO055をIMUモード(6軸)で初期化しています...")

sensor = BNO055(0, pin_scl, pin_sda, address=0x29)

# キャリブレーション計測コード
print("Calibrating BNO055... Move the drone slightly if needed.")

while True:
    sys, gyro, acc, mag = sensor.get_calibration_status()
    print(f"Status -> SYS:{sys} GYRO:{gyro} ACC:{acc} MAG:{mag}")

    # IMUモードの場合、GYROとACCが 3 になれば準備完了 (SYSも3になればベスト)
    if gyro == 3 and acc == 3:
        print("\nCalibration Complete!")
        sensor.save_calibration_to_file()  # ファイルへ22バイト保存
        break

    time.sleep(0.5)