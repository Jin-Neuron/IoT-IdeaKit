#include "py/runtime.h"
#include "py/obj.h"
#include "py/binary.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ==========================================
// 1. 共有状態変数とハードウェア設定
// ==========================================
static volatile bool is_armed = false;
static volatile float current_base_throttle = 0.0f;

// センサーバッファ
static uint8_t static_sensor_buf[6] = {0};
static uint8_t *sensor_buf_ptr = static_sensor_buf;

// 角度・オフセット
static volatile float current_roll = 0.0f;
static volatile float current_pitch = 0.0f;
static volatile float roll_offset = 0.0f;
static volatile float pitch_offset = 0.0f;

// PID制御用変数
static volatile float Kp_roll = 0.5f, Kp_pitch = 0.5f;
static volatile float Kd_roll = 0.2f, Kd_pitch = 0.2f;
static volatile float prev_error_roll = 0.0f, prev_error_pitch = 0.0f;
static volatile float dt = 0.005f; // デフォルト 5ms (200Hz)

// モーター用PIO
static PIO motor_pio = pio0;
static uint sm_rr = 0;
static uint sm_fr = 1;
static uint sm_rl = 2;
static uint sm_fl = 3;

static volatile uint32_t last_process_time_us = 0;

#define MOTOR_OFF_DSHOT 0.0f 

// パケット生成関数の追加
static inline uint32_t make_dshot_packet(uint16_t throttle) {
    if (throttle > 2047) throttle = 2047;
    // 1〜47は特殊コマンドのため、実スロットルの下限は48
    if (throttle < 48 && throttle != 0) throttle = 48;

    uint16_t packet = (throttle << 1); // テレメトリ = 0
    uint16_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    uint16_t frame = (packet << 4) | csum;

    return ((uint32_t)frame << 16);
}

// 0.0%〜100.0% を DShot値 (48〜2047) に変換
static inline uint16_t percent_to_dshot(float pct) {
    if (!is_armed) return 0; // ディスアーム時は停止(0)

    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    // 0% -> 48, 100% -> 2047
    return 48 + (uint16_t)((pct / 100.0f) * (2047.0f - 48.0f));
}

// ==========================================
// 2. モーター出力関数
// ==========================================
static inline void update_motors_raw(float rr, float fr, float rl, float fl) {
if (!is_armed) {
        uint32_t off_pkt = make_dshot_packet(0);
        pio_sm_put(motor_pio, sm_rr, off_pkt);
        pio_sm_put(motor_pio, sm_fr, off_pkt);
        pio_sm_put(motor_pio, sm_rl, off_pkt);
        pio_sm_put(motor_pio, sm_fl, off_pkt);
        return;
    }

    pio_sm_put(motor_pio, sm_rr, make_dshot_packet(percent_to_dshot(rr)));
    pio_sm_put(motor_pio, sm_fr, make_dshot_packet(percent_to_dshot(fr)));
    pio_sm_put(motor_pio, sm_rl, make_dshot_packet(percent_to_dshot(rl)));
    pio_sm_put(motor_pio, sm_fl, make_dshot_packet(percent_to_dshot(fl)));}

// ==========================================
// 3. DMA割り込みハンドラ（バックグラウンド自動処理）
// ==========================================
static mp_obj_t fc_process_handler(void) {

    uint32_t now_us = time_us_32();
    
    if (last_process_time_us == 0) {
        // 初回実行時は直近の割り込み間隔を初期設定
        last_process_time_us = now_us;
        return mp_const_none;
    }

    // 前回からの経過時間 (秒)
    float calculated_dt = (float)(now_us - last_process_time_us) / 1000000.0f;
    last_process_time_us = now_us;

    // 異常値ガード (例: 割り込み落ち等で 50ms 以上空いた場合や、小さすぎる場合はスキップ)
    if (calculated_dt <= 0.0001f || calculated_dt > 0.05f) {
        calculated_dt = 0.001f; // 安全用のフォールバック値
    }

    // --------------------------------------------------
    // 2. センサー読み込みと角度復元
    // --------------------------------------------------
    if (sensor_buf_ptr != NULL) {
        int16_t raw_roll  = (int16_t)((sensor_buf_ptr[3] << 8) | sensor_buf_ptr[2]);
        int16_t raw_pitch = (int16_t)((sensor_buf_ptr[5] << 8) | sensor_buf_ptr[4]);
        
        current_roll  = ((float)raw_roll / 16.0f) - roll_offset;
        current_pitch = ((float)raw_pitch / 16.0f) - pitch_offset;
    }

    if (!is_armed) {
        update_motors_raw(MOTOR_OFF_DSHOT, MOTOR_OFF_DSHOT, MOTOR_OFF_DSHOT, MOTOR_OFF_DSHOT);
        return mp_const_none;
    }

    // --------------------------------------------------
    // 3. PID 計算 (動的 dt を使用)
    // --------------------------------------------------
    float error_roll  = 0.0f - current_roll;
    float error_pitch = 0.0f - current_pitch;

    // 動的に計算した calculated_dt を使用！
    float d_roll  = (error_roll - prev_error_roll) / calculated_dt;
    float d_pitch = (error_pitch - prev_error_pitch) / calculated_dt;

    float pid_roll  = (Kp_roll * error_roll) + (Kd_roll * d_roll);
    float pid_pitch = (Kp_pitch * error_pitch) + (Kd_pitch * d_pitch);

    prev_error_roll  = error_roll;
    prev_error_pitch = error_pitch;

    // モーター出力計算...
    float us_rr = current_base_throttle + pid_roll + pid_pitch;
    float us_fr = current_base_throttle + pid_roll - pid_pitch;
    float us_rl = current_base_throttle - pid_roll + pid_pitch;
    float us_fl = current_base_throttle - pid_roll - pid_pitch;

    update_motors_raw(us_rr, us_fr, us_rl, us_fl);
    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_process_handler_obj, fc_process_handler);

// ==========================================
// 4. MicroPython API群
// ==========================================
static mp_obj_t fc_init_hardware(size_t n_args, const mp_obj_t *args) {
    int pio_id = mp_obj_get_int(args[0]);
    motor_pio = (pio_id == 0) ? pio0 : pio1;
    
    sm_rr = mp_obj_get_int(args[1]);
    sm_fr = mp_obj_get_int(args[2]);
    sm_rl = mp_obj_get_int(args[3]);
    sm_fl = mp_obj_get_int(args[4]);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(fc_init_hardware_obj, 5, 5, fc_init_hardware);

// Python側で作った bytearray をバッファとして登録
static mp_obj_t fc_set_sensor_buffer(mp_obj_t buffer_obj) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len >= 6) {
        sensor_buf_ptr = (uint8_t *)bufinfo.buf;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_set_sensor_buffer_obj, fc_set_sensor_buffer);

static mp_obj_t fc_set_armed(mp_obj_t armed_obj) {
    is_armed = mp_obj_is_true(armed_obj);
    if (!is_armed) {
        update_motors_raw(MOTOR_OFF_DSHOT, MOTOR_OFF_DSHOT, MOTOR_OFF_DSHOT, MOTOR_OFF_DSHOT);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_set_armed_obj, fc_set_armed);

static mp_obj_t fc_set_throttle(mp_obj_t throttle_obj) {
    current_base_throttle = (float)mp_obj_get_float(throttle_obj);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_set_throttle_obj, fc_set_throttle);

static mp_obj_t fc_set_pid(size_t n_args, const mp_obj_t *args) {
    Kp_roll  = (float)mp_obj_get_float(args[0]);
    Kp_pitch = (float)mp_obj_get_float(args[1]);
    Kd_roll  = (float)mp_obj_get_float(args[2]);
    Kd_pitch = (float)mp_obj_get_float(args[3]);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(fc_set_pid_obj, 4, 4, fc_set_pid);

static mp_obj_t fc_calibrate_offset(void) {
    roll_offset  = current_roll + roll_offset;   // 現在値をオフセットに加算
    pitch_offset = current_pitch + pitch_offset;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_offset_obj, fc_calibrate_offset);

static mp_obj_t fc_get_angles(void) {
    mp_obj_t tuple[2] = {
        mp_obj_new_float(current_roll),
        mp_obj_new_float(current_pitch),
    };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_get_angles_obj, fc_get_angles);

// ==========================================
// 5. モジュール登録
// ==========================================
static const mp_rom_map_elem_t fc_core_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fc_core) },
    { MP_ROM_QSTR(MP_QSTR_init_hardware), MP_ROM_PTR(&fc_init_hardware_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sensor_buffer), MP_ROM_PTR(&fc_set_sensor_buffer_obj) },
    { MP_ROM_QSTR(MP_QSTR_process), MP_ROM_PTR(&fc_process_handler_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_armed), MP_ROM_PTR(&fc_set_armed_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_throttle), MP_ROM_PTR(&fc_set_throttle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pid), MP_ROM_PTR(&fc_set_pid_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate), MP_ROM_PTR(&fc_calibrate_offset_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angles), MP_ROM_PTR(&fc_get_angles_obj) },
};

static MP_DEFINE_CONST_DICT(fc_core_globals, fc_core_globals_table);

const mp_obj_module_t fc_core_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&fc_core_globals,
};

MP_REGISTER_MODULE(MP_QSTR_fc_core, fc_core_user_cmodule);