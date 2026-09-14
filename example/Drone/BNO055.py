# bno055.py
import time
import array
import struct
import rp2
import time
from machine import I2C, Pin

# BNO055 Registers
_CHIP_ID = const(0x00)
_PAGE_ID = const(0x07)
_OPR_MODE = const(0x3D)
_SYS_TRIGGER = const(0x3F)
_EULER_H_LSB = const(0x1A)
_CALIB_STAT = const(0x35) # ←追加: キャリブレーション状態レジスタ

# Operation Modes
_NDOF_MODE = const(0x0C)  # 9軸 (地磁気あり)
_IMU_MODE = const(0x08)   # ←追加: 6軸 (ジャイロ＋加速度のみ)
_CONFIG_MODE = const(0x00)

CALIB_FILE = "bno055_calib.bin"

# RP2040 / RP2350 I2C0 レジスタ定義
I2C0_BASE = 0x40044000
IC_DATA_CMD = I2C0_BASE + 0x10

class BNO055:
    # デフォルトを _IMU_MODE に変更
    def __init__(self, id, pin_scl, pin_sda, address=0x28, mode=_IMU_MODE):
        self.pin_scl = pin_scl
        self.pin_sda = pin_sda
        self.i2c = I2C(id, sda=pin_sda, scl=pin_scl, freq=400000)
        self.address = address
        self.tx_cmds = array.array(
            "I",
            [
                0x001C,  # レジスタ 0x1C
                0x0500,  # Restart + Read (1B目)
                0x0100,  # Read (2B目)
                0x0100,  # Read (3B目)
                0x0300,  # Stop + Read (4B目)
            ],
        )

        self._config_mode()
        
        if self._read_register(_CHIP_ID) != 0xA0:
            raise RuntimeError("Failed to find BNO055! Check wiring.")
            
        self._reset()
        self._set_mode(mode)

    def configure(self, irq_handler=None):
        self.dma_ch = rp2.DMA()
        ctrl_val = self.dma_ch.pack_ctrl(
            size=2,             # 2 = 32-bit Byte転送
            inc_read=True,
            inc_write=False,
            treq_sel=32,         # DREQ_I2C0_TX = 32
        )

        # 4. DMA構成と起動
        self.dma_ch.config(
            read=self.tx_cmds,
            write=IC_DATA_CMD,
            count=len(self.tx_cmds),
            ctrl=ctrl_val,
            trigger=False
        )

        self.rx_dma_a = rp2.DMA()
        self.rx_dma_b = rp2.DMA()

        self.rx_ctrl_a = self.rx_dma_a.pack_ctrl(
            size=0,          # 8-bit 転送 (1バイト単位)
            inc_read=False,  # ソース(PIO RX FIFO)は固定
            inc_write=True,  # 転送先(rx_buf)はアドレスインクリメント
            treq_sel=33,      # DREQ_I2C0_RX
            irq_quiet=False,
            chain_to=self.rx_dma_b.channel,  # ★ A完了時に自動で B を起動
        )

        self.rx_ctrl_b = self.rx_dma_b.pack_ctrl(
            size=0,          # 8-bit 転送 (1バイト単位)
            inc_read=False,  # ソース(PIO RX FIFO)は固定
            inc_write=True,  # 転送先(rx_buf)はアドレスインクリメント
            treq_sel=33,      # DREQ_I2C0_RX 
            irq_quiet=False,
            chain_to=self.rx_dma_a.channel,  # ★ B完了時に自動で A を起動
        )

        # 受信データ用バッファ (6バイト)
        self.rx_buf_a = bytearray(4)
        self.rx_buf_b = bytearray(4)

        self.rx_dma_a.irq(handler=irq_handler)
        self.rx_dma_b.irq(handler=irq_handler)


        self.rx_dma_a.config(
            read=IC_DATA_CMD,
            write=self.rx_buf_a,
            count=4,
            ctrl=self.rx_ctrl_a,
            trigger=False
        )

        self.rx_dma_b.config(
            read=IC_DATA_CMD,
            write=self.rx_buf_b,
            count=4,
            ctrl=self.rx_ctrl_b,
            trigger=False
        )

    def activate(self):
        self.rx_dma_a.active(1)
        self.rx_dma_b.active(0)  # B は A が完了したら自動で起動
        self.dma_ch.active(1)

    def deactivate(self):
        self.dma_ch.active(0)
        self.rx_dma_a.active(0)
        self.rx_dma_b.active(0)

        time.sleep(0.5)  # DMA停止待ち

        self.pin_scl.init(Pin.IN)
        self.pin_sda.init(Pin.IN)

        time.sleep(0.5)  # ピン解放待ち

        print("Stopped and Cleaned up.")

    def _read_register(self, register):
        return self.i2c.readfrom_mem(self.address, register, 1)[0]

    def _write_register(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes([value]))

    def _config_mode(self):
        self._set_mode(_CONFIG_MODE)

    def _set_mode(self, mode):
        self._write_register(_OPR_MODE, mode)
        time.sleep_ms(30)

    def _reset(self):
        self._write_register(_SYS_TRIGGER, 0x20)
        time.sleep_ms(700)
        
    def read_euler(self):
        data = self.i2c.readfrom_mem(self.address, _EULER_H_LSB, 6)
        heading, roll, pitch = struct.unpack('<hhh', data)
        return (heading / 16.0, roll / 16.0, pitch / 16.0)

        
    def get_calibration_status(self):
        """現在のキャリブレーション状態を取得 (0~3: 3が完全校正)"""
        # レジスタ 0x35: CALIB_STAT
        stat = self._read_register(0x35)
        sys = (stat >> 6) & 0x03
        gyro = (stat >> 4) & 0x03
        acc = (stat >> 2) & 0x03
        mag = stat & 0x03
        return sys, gyro, acc, mag

    def save_calibration_to_file(self, filename=CALIB_FILE):
        """キャリブレーションオフセット(22バイト)を取得してファイルに保存"""
        print("Saving calibration data...")

        # 1. CONFIGMODE に変更
        self._write_register(0x3D, 0x00)
        time.sleep_ms(25)

        # 2. オフセットレジスタ (0x55 から 22バイト) を一括読み出し
        calib_list = []
        for reg in range(0x55, 0x55 + 22):
            val = self._read_register(
                reg
            ) 
            calib_list.append(val)
        
        calib_data = bytes(calib_list)
        print("Read bytes:", [hex(b) for b in calib_data])

        # 3. ファイルへバイナリ保存
        with open(filename, "wb") as f:
            f.write(calib_data)

        # 4. 動作モード (IMUモード 0x08) に復帰
        self._write_register(0x3D, 0x08)
        time.sleep_ms(25)

        print(f"Successfully saved 22 bytes calibration to {filename}")

    def load_calibration_from_file(self, filename=CALIB_FILE):
        """ファイルからオフセットを読み込んで BNO055 に適用"""
        with open(filename, "rb") as f:
            calib_data = f.read()

        if len(calib_data) != 22:
            print("Calibration file is corrupted (size != 22)")
            return False

        print(f"Loading calibration from {filename}...")

        print("Write bytes:", [hex(b) for b in calib_data])

        # 1. CONFIGMODE に変更
        self._write_register(0x3D, 0x00)
        time.sleep_ms(25)

        # 2. 0x55 から 22 バイトを 1 バイトずつ int 型として書き込む
        for i, b in enumerate(calib_data):
            # b は int 型 (0~255) になっているため TypeError にならない
            self._write_register(0x55 + i, b)
        
        # 3. 動作モード (IMUモード 0x08) に戻す
        self._write_register(0x3D, 0x08)

        # 3. 動作モード (IMUモード 0x08) に戻す
        self._write_register(0x3D, 0x08)
        time.sleep_ms(25)

        print("Calibration offsets successfully loaded into BNO055!")
        return True
