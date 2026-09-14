import array
import struct
import time
from machine import I2C, Pin
import rp2

# --------------------------------------------------
# 1. ハードウェア I2C 初期化 & BNO055 設定
# --------------------------------------------------
i2c = I2C(0, scl=Pin("I2C_SCL"), sda=Pin("I2C_SDA"), freq=400000)

# IMU モード (0x08) で起動
i2c.writeto_mem(0x29, 0x3D, b"\x00")
time.sleep_ms(50)
i2c.writeto_mem(0x29, 0x3D, b"\x08")
time.sleep_ms(50)

# RP2040 / RP2350 I2C0 レジスタ定義
I2C0_BASE = 0x40044000
IC_DATA_CMD = I2C0_BASE + 0x10

# --------------------------------------------------
# 2. DMA 送信コマンド列 (Pitch/Roll 4バイト読み出し)
# --------------------------------------------------
tx_cmds = array.array(
    "I",
    [
        0x001C,  # レジスタ 0x1C
        0x0500,  # Restart + Read (1B目)
        0x0100,  # Read (2B目)
        0x0100,  # Read (3B目)
        0x0300,  # Stop + Read (4B目)
    ],
)

# --------------------------------------------------
# 3. グローバルバッファ & 変数
# --------------------------------------------------
rx_buf_a = bytearray(4)
rx_buf_b = bytearray(4)

t_last_irq = 0
exec_time_us = 0
interval_us = 0
isStarted = True
MAX_ROLL = 45.0
MAX_PITCH = 45.0

# メインループ表示用の共有変数
pitch = 0.0
roll = 0.0

# --------------------------------------------------
# 4. 割り込みハンドラ (Ping-Pong チェーン & 制御)
# --------------------------------------------------
def dma_rx_irq_handler(dma_obj):
    global rx_buf_a, rx_buf_b, rx_dma_a, rx_dma_b, dma_tx, tx_cmds
    global isStarted, exec_time_us, interval_us, t_last_irq
    global pitch, roll

    t_now = time.ticks_us()
    
    if t_last_irq > 0:
        interval_us = time.ticks_diff(t_now, t_last_irq)

    t_start = time.ticks_us()

    # 完了したチャンネルに応じてバッファを取得 & 次回周回用に再装填
    if dma_obj.channel == rx_dma_a.channel:
        raw = rx_buf_a
        rx_dma_a.write = rx_buf_a
        rx_dma_a.count = 4
    else:
        raw = rx_buf_b
        rx_dma_b.write = rx_buf_b
        rx_dma_b.count = 4

    # 姿勢角デコード (Pitch, Roll)
    pitch = struct.unpack_from("<h", raw, 0)[0] / 16.0
    roll = struct.unpack_from("<h", raw, 2)[0] / 16.0

    if isStarted:
        # fc_core.process() などの制御ループ処理
        pass

    if not dma_tx.active():
        dma_tx.read = tx_cmds
        dma_tx.count = len(tx_cmds)
        dma_tx.active(1)
        
    exec_time_us = time.ticks_diff(time.ticks_us(), t_start)
    
    t_last_irq = time.ticks_us()

# --------------------------------------------------
# 5. DMA チャンネル設定 & チェーン構築
# --------------------------------------------------
# DMA チャンネル確保
dma_tx = rp2.DMA()
rx_dma_a = rp2.DMA()
rx_dma_b = rp2.DMA()

# 送信 DMA (TX)
tx_ctrl = dma_tx.pack_ctrl(
    size=2,
    inc_read=True,
    inc_write=False,
    treq_sel=32,  # I2C0_TX
)

# RX DMA A (完了時 -> B にチェーン & 割り込み発生)
rx_ctrl_a = rx_dma_a.pack_ctrl(
    size=0,  # 8-bit
    inc_read=False,
    inc_write=True,
    treq_sel=33,  # I2C0_RX
    chain_to=rx_dma_b.channel,
    irq_quiet=False,
)

# RX DMA B (完了時 -> A にチェーン & 割り込み発生)
rx_ctrl_b = rx_dma_b.pack_ctrl(
    size=0,  # 8-bit
    inc_read=False,
    inc_write=True,
    treq_sel=33,  # I2C0_RX
    chain_to=rx_dma_a.channel,
    irq_quiet=False,
)

# 割り込み登録
rx_dma_a.irq(handler=dma_rx_irq_handler)
rx_dma_b.irq(handler=dma_rx_irq_handler)

# 初期待機設定 (A をトリガー待ち、B をスタンバイ)
rx_dma_b.config(
    read=IC_DATA_CMD,
    write=rx_buf_b,
    count=len(rx_buf_b),
    ctrl=rx_ctrl_b,
    trigger=False,
)
rx_dma_a.config(
    read=IC_DATA_CMD,
    write=rx_buf_a,
    count=len(rx_buf_a),
    ctrl=rx_ctrl_a,
    trigger=True,
)

# --------------------------------------------------
# 6. 送信トリガー関数 & 実行ループ
# --------------------------------------------------
dma_tx.config(
    read=tx_cmds,
    write=IC_DATA_CMD,
    count=len(tx_cmds),
    ctrl=tx_ctrl,
    trigger=True,
)

print("Flat Ping-Pong DMA Running...")

while True:
    print(
        f"Roll: {roll:6.1f}° | Pitch: {pitch:6.1f}° | "
        f"Interval: {interval_us:5d} us | Exec: {exec_time_us:4d} us"
    )
    time.sleep_ms(100)  # 50Hz 周期